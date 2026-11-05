#include "lkm_bno086_probe_duplex.h"
#include <string.h>

/* Conservative diagnostic timeouts; these are NOT timing guarantees. */
#define DP_SPI_TIMEOUT_MS        30u
#define DP_STARTUP_TIMEOUT_MS  1500u
#define DP_REQUEST_TIMEOUT_MS   250u
#define DP_RESPONSE_TIMEOUT_MS  750u
#define DP_MAX_STARTUP_PACKETS   12u
#define DP_SPI_CHUNK             64u
#define DP_HEADER_BYTES           4u

static bool valid(const lkm_bno086_t *d)
{
    return d != NULL && d->io.spi != NULL &&
           d->io.cs_port != NULL && d->io.cs_pin != 0u &&
           d->io.irq_port != NULL && d->io.irq_pin != 0u &&
           d->io.reset_port != NULL && d->io.reset_pin != 0u &&
           d->io.wake_port != NULL && d->io.wake_pin != 0u;
}

static bool intn_low(const lkm_bno086_t *d)
{
    return HAL_GPIO_ReadPin(d->io.irq_port, d->io.irq_pin) == GPIO_PIN_RESET;
}

static bool wait_intn(const lkm_bno086_t *d, uint32_t limit_ms)
{
    const uint32_t start = HAL_GetTick();
    while (!intn_low(d)) {
        if ((uint32_t)(HAL_GetTick() - start) >= limit_ms) return false;
        HAL_Delay(1u);
    }
    return true;
}

/* A single full-duplex SHTP transaction; one CS assertion and at most 512
 * received cargo bytes. Host's NULL SHTP header is four 0x00 bytes.
 * Important: SPI clocks both directions, even when the host is transmitting
 * a product request and the sensor is delivering an unrelated cargo.
 * Every SPI failure is returned to the caller, which must fail closed.
 */
static lkm_bno086_status_t transfer(lkm_bno086_t *d,
                                    lkm_bno086_packet_t *packet,
                                    const uint8_t *host,
                                    uint16_t host_length,
                                    bool *got_rx)
{
    if (!valid(d) || packet == NULL || got_rx == NULL)
        return LKM_BNO086_BAD_ARGUMENT;
    *got_rx = false;

    if (host == NULL) {
        if (host_length != 0u) return LKM_BNO086_BAD_ARGUMENT;
        if (!intn_low(d)) return LKM_BNO086_IRQ_TIMEOUT;
    } else {
        lkm_shtp_header_t outgoing = {0};
        if (host_length != 6u ||
            lkm_shtp_decode_header(host, LKM_BNO086_MAX_PAYLOAD,
                                   &outgoing) != LKM_SHTP_OK ||
            outgoing.total_length != host_length ||
            outgoing.channel != LKM_BNO086_CONTROL_CHANNEL ||
            outgoing.sequence != d->tx_seq[LKM_BNO086_CONTROL_CHANNEL] ||
            host[4] != LKM_BNO086_PRODUCT_REQUEST || host[5] != 0u)
            return LKM_BNO086_BAD_ARGUMENT;
        HAL_GPIO_WritePin(d->io.wake_port, d->io.wake_pin, GPIO_PIN_RESET);
        if (!wait_intn(d, DP_REQUEST_TIMEOUT_MS)) {
            HAL_GPIO_WritePin(d->io.wake_port, d->io.wake_pin, GPIO_PIN_SET);
            return LKM_BNO086_IRQ_TIMEOUT;
        }
    }

    uint8_t tx_h[DP_HEADER_BYTES] = {0u};
    uint8_t rx_h[DP_HEADER_BYTES] = {0u};
    if (host != NULL) memcpy(tx_h, host, DP_HEADER_BYTES);
    lkm_bno086_status_t result = LKM_BNO086_OK;
    lkm_shtp_header_t decoded = {0};
    bool received = false;
    uint16_t rx_bytes = 0u;
    const uint16_t tx_bytes = (host == NULL) ? 0u :
                              (uint16_t)(host_length - DP_HEADER_BYTES);

    HAL_GPIO_WritePin(d->io.cs_port, d->io.cs_pin, GPIO_PIN_RESET);
    if (HAL_SPI_TransmitReceive(d->io.spi, tx_h, rx_h,
                                DP_HEADER_BYTES, DP_SPI_TIMEOUT_MS) != HAL_OK) {
        result = LKM_BNO086_SPI_ERROR;
        goto cleanup;
    }
    const bool null_header = rx_h[0] == 0u && rx_h[1] == 0u &&
                             rx_h[2] == 0u && rx_h[3] == 0u;
    if (!null_header) {
        const lkm_shtp_result_t decoded_status = lkm_shtp_decode_header(
            rx_h, LKM_BNO086_MAX_PAYLOAD, &decoded);
        if (decoded_status != LKM_SHTP_OK) {
            result = decoded_status == LKM_SHTP_ERR_CAPACITY ? LKM_BNO086_TOO_LARGE :
                     decoded_status == LKM_SHTP_ERR_CONTINUATION ?
                         LKM_BNO086_FRAGMENT_UNSUPPORTED : LKM_BNO086_SHTP_ERROR;
            goto cleanup;
        }
        rx_bytes = decoded.payload_length;
        received = true;
    }

    const uint16_t clocks = tx_bytes > rx_bytes ? tx_bytes : rx_bytes;
    for (uint16_t offset = 0u; offset < clocks;) {
        uint16_t n = (uint16_t)(clocks - offset);
        if (n > DP_SPI_CHUNK) n = DP_SPI_CHUNK;
        uint8_t tx[DP_SPI_CHUNK] = {0u};
        uint8_t rx[DP_SPI_CHUNK] = {0u};
        if (offset < tx_bytes) {
            uint16_t count = (uint16_t)(tx_bytes - offset);
            if (count > n) count = n;
            memcpy(tx, host + DP_HEADER_BYTES + offset, count);
        }
        if (HAL_SPI_TransmitReceive(d->io.spi, tx, rx, n,
                                    DP_SPI_TIMEOUT_MS) != HAL_OK) {
            result = LKM_BNO086_SPI_ERROR;
            goto cleanup;
        }
        if (offset < rx_bytes) {
            uint16_t count = (uint16_t)(rx_bytes - offset);
            if (count > n) count = n;
            memcpy(packet->payload + offset, rx, count);
        }
        offset = (uint16_t)(offset + n);
    }

    if (received) {
        packet->header = decoded;
        ++d->received_packets;
        *got_rx = true;
    }
    if (host != NULL)
        ++d->tx_seq[LKM_BNO086_CONTROL_CHANNEL];

cleanup:
    /* CS and WAKE must always be released, including partial SPI failures. */
    HAL_GPIO_WritePin(d->io.cs_port, d->io.cs_pin, GPIO_PIN_SET);
    if (host != NULL)
        HAL_GPIO_WritePin(d->io.wake_port, d->io.wake_pin, GPIO_PIN_SET);
    return result;
}

static bool product_response(const lkm_bno086_packet_t *packet)
{
    return packet->header.channel == LKM_BNO086_CONTROL_CHANNEL &&
           packet->header.payload_length > 0u &&
           packet->payload[0] == LKM_BNO086_PRODUCT_RESPONSE;
}

lkm_bno086_status_t lkm_bno086_probe_duplex(
    lkm_bno086_t *d,
    lkm_bno086_packet_t *scratch,
    lkm_bno086_product_id_t *id)
{
    if (!valid(d) || scratch == NULL || id == NULL)
        return LKM_BNO086_BAD_ARGUMENT;

    memset(d->tx_seq, 0, sizeof(d->tx_seq));
    d->received_packets = 0u;
    d->startup_mask = 0u;
    memset(id, 0, sizeof(*id));
    memset(scratch, 0, sizeof(*scratch));

    /* The interface-select strap must be HIGH from before reset until the
     * first H_INTN assertion. After startup PS0 may act as active-low WAKE. */
    HAL_GPIO_WritePin(d->io.cs_port, d->io.cs_pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(d->io.wake_port, d->io.wake_pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(d->io.reset_port, d->io.reset_pin, GPIO_PIN_SET);
    HAL_Delay(2u);
    HAL_GPIO_WritePin(d->io.reset_port, d->io.reset_pin, GPIO_PIN_RESET);
    HAL_Delay(10u);
    HAL_GPIO_WritePin(d->io.reset_port, d->io.reset_pin, GPIO_PIN_SET);

    const uint32_t start = HAL_GetTick();
    unsigned seen = 0u;
    while ((uint32_t)(HAL_GetTick() - start) < DP_STARTUP_TIMEOUT_MS &&
           seen < DP_MAX_STARTUP_PACKETS && d->startup_mask != 0x07u) {
        if (!intn_low(d)) {
            HAL_Delay(1u);
            continue;
        }
        bool got = false;
        lkm_bno086_status_t s = transfer(d, scratch, NULL, 0u, &got);
        if (s != LKM_BNO086_OK) return s;
        ++seen;
        if (got && scratch->header.channel <= 2u)
            d->startup_mask = (uint8_t)(d->startup_mask |
                (uint8_t)(1u << scratch->header.channel));
    }
    if (d->startup_mask != 0x07u) return LKM_BNO086_STARTUP_INCOMPLETE;

    uint8_t cmd[6] = {0u};
    if (lkm_shtp_encode_header(2u, LKM_BNO086_CONTROL_CHANNEL,
         d->tx_seq[LKM_BNO086_CONTROL_CHANNEL], cmd) != LKM_SHTP_OK)
        return LKM_BNO086_SHTP_ERROR;
    cmd[4] = LKM_BNO086_PRODUCT_REQUEST;
    cmd[5] = 0u;

    bool got = false;
    lkm_bno086_status_t s = transfer(d, scratch, cmd, sizeof(cmd), &got);
    if (s != LKM_BNO086_OK) return s;
    if (got && product_response(scratch))
        return lkm_bno086_parse_product_id(scratch, id);

    const uint32_t reply_start = HAL_GetTick();
    while ((uint32_t)(HAL_GetTick() - reply_start) < DP_RESPONSE_TIMEOUT_MS) {
        if (!intn_low(d)) {
            HAL_Delay(1u);
            continue;
        }
        s = transfer(d, scratch, NULL, 0u, &got);
        if (s != LKM_BNO086_OK) return s;
        if (got && product_response(scratch))
            return lkm_bno086_parse_product_id(scratch, id);
    }
    return LKM_BNO086_RESPONSE_TIMEOUT;
}
