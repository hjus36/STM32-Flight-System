#include "lkm_bno086.h"
#include <string.h>

#define LKM_SPI_TIMEOUT_MS      30u
#define LKM_STARTUP_TIMEOUT_MS 1500u
#define LKM_REQUEST_TIMEOUT_MS  250u
#define LKM_RESPONSE_TIMEOUT_MS 750u
#define LKM_MAX_STARTUP_PACKETS 12u
#define LKM_READ_CHUNK_BYTES     64u

static bool valid_io(const lkm_bno086_t *device)
{
    if (device == NULL) return false;
    const lkm_bno086_io_t *io = &device->io;
    return (io->spi != NULL) &&
           (io->cs_port != NULL) && (io->irq_port != NULL) &&
           (io->reset_port != NULL) && (io->wake_port != NULL) &&
           (io->cs_pin != 0u) && (io->irq_pin != 0u) &&
           (io->reset_pin != 0u) && (io->wake_pin != 0u);
}

static void cs_high(const lkm_bno086_t *device)
{
    HAL_GPIO_WritePin(device->io.cs_port, device->io.cs_pin, GPIO_PIN_SET);
}

static void cs_low(const lkm_bno086_t *device)
{
    HAL_GPIO_WritePin(device->io.cs_port, device->io.cs_pin, GPIO_PIN_RESET);
}

static bool irq_asserted(const lkm_bno086_t *device)
{
    return HAL_GPIO_ReadPin(device->io.irq_port, device->io.irq_pin) == GPIO_PIN_RESET;
}

static bool wait_irq_low(const lkm_bno086_t *device, uint32_t timeout_ms)
{
    const uint32_t start = HAL_GetTick();
    while (!irq_asserted(device)) {
        if ((uint32_t)(HAL_GetTick() - start) >= timeout_ms) return false;
        HAL_Delay(1u);
    }
    return true;
}

void lkm_bno086_bind(lkm_bno086_t *device, const lkm_bno086_io_t *io)
{
    if ((device == NULL) || (io == NULL)) return;
    memset(device, 0, sizeof(*device));
    device->io = *io;
}

static lkm_bno086_status_t read_payload(lkm_bno086_t *device,
                                        uint8_t *destination, uint16_t length)
{
    uint8_t tx[LKM_READ_CHUNK_BYTES];
    uint8_t rx[LKM_READ_CHUNK_BYTES];
    memset(tx, 0xFF, sizeof(tx));

    uint16_t consumed = 0u;
    while (consumed < length) {
        uint16_t count = (uint16_t)(length - consumed);
        if (count > LKM_READ_CHUNK_BYTES) count = LKM_READ_CHUNK_BYTES;
        if (HAL_SPI_TransmitReceive(device->io.spi, tx, rx, count,
                                    LKM_SPI_TIMEOUT_MS) != HAL_OK) {
            return LKM_BNO086_SPI_ERROR;
        }
        memcpy(&destination[consumed], rx, count);
        consumed = (uint16_t)(consumed + count);
    }
    return LKM_BNO086_OK;
}

lkm_bno086_status_t lkm_bno086_receive(lkm_bno086_t *device,
                                       lkm_bno086_packet_t *packet)
{
    if (!valid_io(device) || (packet == NULL)) return LKM_BNO086_BAD_ARGUMENT;
    if (!irq_asserted(device)) return LKM_BNO086_IRQ_TIMEOUT;

    uint8_t dummy[4] = {0xFFu, 0xFFu, 0xFFu, 0xFFu};
    uint8_t header[4] = {0};

    cs_low(device);
    HAL_StatusTypeDef hw = HAL_SPI_TransmitReceive(device->io.spi, dummy,
                                                   header, 4u, LKM_SPI_TIMEOUT_MS);
    if (hw != HAL_OK) {
        cs_high(device);
        return LKM_BNO086_SPI_ERROR;
    }

    lkm_shtp_result_t result = lkm_shtp_decode_header(header,
                                                      LKM_BNO086_MAX_PAYLOAD,
                                                      &packet->header);
    if (result != LKM_SHTP_OK) {
        cs_high(device);
        if (result == LKM_SHTP_ERR_CONTINUATION) {
            return LKM_BNO086_FRAGMENT_UNSUPPORTED;
        }
        if (result == LKM_SHTP_ERR_CAPACITY) return LKM_BNO086_TOO_LARGE;
        return LKM_BNO086_SHTP_ERROR;
    }

    lkm_bno086_status_t status = read_payload(device, packet->payload,
                                              packet->header.payload_length);
    cs_high(device);  /* Required even on a truncated / failed SPI read. */
    if (status == LKM_BNO086_OK) device->received_packets++;
    return status;
}

static uint32_t le32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

lkm_bno086_status_t lkm_bno086_parse_product_id(
    const lkm_bno086_packet_t *packet, lkm_bno086_product_id_t *id)
{
    if ((packet == NULL) || (id == NULL)) return LKM_BNO086_BAD_ARGUMENT;
    if ((packet->header.channel != LKM_BNO086_CONTROL_CHANNEL) ||
        (packet->header.payload_length < 14u) ||
        (packet->header.continuation) ||
        (packet->payload[0] != LKM_BNO086_PRODUCT_RESPONSE)) {
        return LKM_BNO086_BAD_PRODUCT_RESPONSE;
    }
    id->reset_cause = packet->payload[1];
    id->sw_major = packet->payload[2];
    id->sw_minor = packet->payload[3];
    id->sw_part_number = le32(&packet->payload[4]);
    id->sw_build_number = le32(&packet->payload[8]);
    id->sw_patch = (uint16_t)packet->payload[12] |
                   (uint16_t)((uint16_t)packet->payload[13] << 8);
    return LKM_BNO086_OK;
}

static lkm_bno086_status_t send_product_request(lkm_bno086_t *device)
{
    if (!valid_io(device)) return LKM_BNO086_BAD_ARGUMENT;
    uint8_t tx[6] = {0};
    lkm_shtp_result_t s = lkm_shtp_encode_header(
        2u, LKM_BNO086_CONTROL_CHANNEL,
        device->tx_seq[LKM_BNO086_CONTROL_CHANNEL], tx);
    if (s != LKM_SHTP_OK) return LKM_BNO086_SHTP_ERROR;
    tx[4] = LKM_BNO086_PRODUCT_REQUEST;
    tx[5] = 0u;

    /* The pin is protocol select at reset; after startup it is active-low WAKE. */
    HAL_GPIO_WritePin(device->io.wake_port, device->io.wake_pin, GPIO_PIN_RESET);
    const bool ready = wait_irq_low(device, LKM_REQUEST_TIMEOUT_MS);
    HAL_GPIO_WritePin(device->io.wake_port, device->io.wake_pin, GPIO_PIN_SET);
    if (!ready) return LKM_BNO086_IRQ_TIMEOUT;

    cs_low(device);
    HAL_StatusTypeDef hw = HAL_SPI_Transmit(device->io.spi, tx,
                                            (uint16_t)sizeof(tx), LKM_SPI_TIMEOUT_MS);
    cs_high(device);
    if (hw != HAL_OK) return LKM_BNO086_SPI_ERROR;
    device->tx_seq[LKM_BNO086_CONTROL_CHANNEL]++;
    return LKM_BNO086_OK;
}

lkm_bno086_status_t lkm_bno086_probe(lkm_bno086_t *device,
                                     lkm_bno086_packet_t *scratch,
                                     lkm_bno086_product_id_t *id)
{
    if (!valid_io(device) || (scratch == NULL) || (id == NULL)) {
        return LKM_BNO086_BAD_ARGUMENT;
    }

    memset(device->tx_seq, 0, sizeof(device->tx_seq));
    device->received_packets = 0u;
    device->startup_mask = 0u;
    memset(id, 0, sizeof(*id));

    /* Maintain PS1 (external pull-up) and PS0 high through sensor reset. */
    cs_high(device);
    HAL_GPIO_WritePin(device->io.wake_port, device->io.wake_pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(device->io.reset_port, device->io.reset_pin, GPIO_PIN_SET);
    HAL_Delay(2u);
    HAL_GPIO_WritePin(device->io.reset_port, device->io.reset_pin, GPIO_PIN_RESET);
    HAL_Delay(10u);
    HAL_GPIO_WritePin(device->io.reset_port, device->io.reset_pin, GPIO_PIN_SET);

    /* Receive initial advertisement (channel 0), executable reset (1), and
     * SH-2 initialization (2). No command is sent before these. */
    const uint32_t boot_start = HAL_GetTick();
    uint32_t packets = 0u;
    while ((uint32_t)(HAL_GetTick() - boot_start) < LKM_STARTUP_TIMEOUT_MS &&
           packets < LKM_MAX_STARTUP_PACKETS && device->startup_mask != 0x07u) {
        if (!irq_asserted(device)) {
            HAL_Delay(1u);
            continue;
        }
        lkm_bno086_status_t s = lkm_bno086_receive(device, scratch);
        if (s != LKM_BNO086_OK) return s;
        packets++;
        if (scratch->header.channel <= 2u) {
            device->startup_mask = (uint8_t)(device->startup_mask |
                                      (uint8_t)(1u << scratch->header.channel));
        }
    }
    if (device->startup_mask != 0x07u) return LKM_BNO086_STARTUP_INCOMPLETE;

    lkm_bno086_status_t s = send_product_request(device);
    if (s != LKM_BNO086_OK) return s;

    const uint32_t response_start = HAL_GetTick();
    while ((uint32_t)(HAL_GetTick() - response_start) < LKM_RESPONSE_TIMEOUT_MS) {
        if (!irq_asserted(device)) {
            HAL_Delay(1u);
            continue;
        }
        s = lkm_bno086_receive(device, scratch);
        if (s != LKM_BNO086_OK) return s;
        if ((scratch->header.channel == LKM_BNO086_CONTROL_CHANNEL) &&
            (scratch->header.payload_length > 0u) &&
            (scratch->payload[0] == LKM_BNO086_PRODUCT_RESPONSE)) {
            return lkm_bno086_parse_product_id(scratch, id);
        }
    }
    return LKM_BNO086_RESPONSE_TIMEOUT;
}

const char *lkm_bno086_status_text(lkm_bno086_status_t status)
{
    switch (status) {
        case LKM_BNO086_OK: return "OK";
        case LKM_BNO086_BAD_ARGUMENT: return "BAD_ARGUMENT";
        case LKM_BNO086_IRQ_TIMEOUT: return "IRQ_TIMEOUT";
        case LKM_BNO086_SPI_ERROR: return "SPI_ERROR";
        case LKM_BNO086_SHTP_ERROR: return "SHTP_ERROR";
        case LKM_BNO086_TOO_LARGE: return "PACKET_TOO_LARGE";
        case LKM_BNO086_FRAGMENT_UNSUPPORTED: return "FRAGMENT_UNSUPPORTED";
        case LKM_BNO086_STARTUP_INCOMPLETE: return "STARTUP_INCOMPLETE";
        case LKM_BNO086_RESPONSE_TIMEOUT: return "RESPONSE_TIMEOUT";
        case LKM_BNO086_BAD_PRODUCT_RESPONSE: return "BAD_PRODUCT_RESPONSE";
        default: return "UNKNOWN_ERROR";
    }
}
