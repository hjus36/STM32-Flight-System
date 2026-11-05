#include "lkm_bno086_link.h"
#include <string.h>

#define LINK_CONTROL_CHANNEL 2u
#define LINK_HEADER_BYTES 4u
#define LINK_SPI_CHUNK 64u
#define LINK_SPI_TIMEOUT_MS 30u
#define LINK_TX_READY_TIMEOUT_MS 50u

static bool valid_device(const lkm_bno086_t *d)
{
    return d != NULL && d->io.spi != NULL &&
           d->io.cs_port != NULL && d->io.cs_pin != 0u &&
           d->io.irq_port != NULL && d->io.irq_pin != 0u &&
           d->io.wake_port != NULL && d->io.wake_pin != 0u;
}

static void fail_closed(lkm_bno086_link_t *link)
{
    if (link == NULL || link->stream == NULL) return;
    link->stream->sample_valid = false;
    link->stream->feature_confirmed = false;
}

static lkm_link_result_t latch(lkm_bno086_link_t *link, lkm_link_result_t cause)
{
    fail_closed(link);
    link->fault_latched = true;
    return cause;
}

static bool irq_low(const lkm_bno086_link_t *link)
{
    return HAL_GPIO_ReadPin(link->device->io.irq_port,
                            link->device->io.irq_pin) == GPIO_PIN_RESET;
}

static bool wait_ready(const lkm_bno086_link_t *link)
{
    const uint32_t start = HAL_GetTick();
    while (!irq_low(link)) {
        if ((uint32_t)(HAL_GetTick() - start) >= LINK_TX_READY_TIMEOUT_MS)
            return false;
        HAL_Delay(1u);
    }
    return true;
}

static void set_wake(const lkm_bno086_link_t *link, GPIO_PinState s)
{
    HAL_GPIO_WritePin(link->device->io.wake_port,
                      link->device->io.wake_pin, s);
}
static void set_cs(const lkm_bno086_link_t *link, GPIO_PinState s)
{
    HAL_GPIO_WritePin(link->device->io.cs_port,
                      link->device->io.cs_pin, s);
}

lkm_link_result_t lkm_bno086_link_init(
    lkm_bno086_link_t *link, lkm_bno086_t *device,
    lkm_bno086_stream_t *stream, lkm_bno086_packet_t *packet,
    const lkm_stream_config_t *config, bool probe_verified)
{
    if (link == NULL || !valid_device(device) || stream == NULL ||
        packet == NULL || config == NULL) return LKM_LINK_BAD_ARGUMENT;
    if (!probe_verified || device->startup_mask != 0x07u)
        return LKM_LINK_NOT_PROBED;
    if (lkm_bno086_stream_init(stream, config) != LKM_STREAM_OK)
        return LKM_LINK_BAD_ARGUMENT;
    memset(link, 0, sizeof(*link));
    link->device = device;
    link->stream = stream;
    link->packet = packet;
    link->ready = true;
    link->last_stream_result = LKM_STREAM_ERR_NO_SAMPLE;
    /* Single owner for outgoing channel-2 sequence after Product ID. */
    stream->next_control_tx_sequence = device->tx_seq[LINK_CONTROL_CHANNEL];
    return LKM_LINK_OK;
}

static lkm_link_result_t preflight(lkm_bno086_link_t *link)
{
    if (link == NULL || !link->ready || !valid_device(link->device) ||
        link->stream == NULL || link->packet == NULL) return LKM_LINK_NOT_READY;
    if (link->fault_latched) return LKM_LINK_FAULT_LATCHED;
    if (link->in_transfer) return LKM_LINK_NOT_READY;
    return LKM_LINK_OK;
}

/* One CS assertion, always duplex. A host packet may be sent at the same time
 * as the sensor returns an SHTP packet (including a valid 0xFC reply).
 * For read-only transfers the MOSI header is a 4-byte NULL SHTP header (0x00).
 * Receive payload is capped at 512; continuation/reassembly is NOT supported.
 * No partial RX is passed to the stream. All failure paths release CS/WAKE.
 * On a partially clocked host write, delivery status is AMBIGUOUS; caller
 * latches a fault rather than blindly retrying the control sequence.
 */
static lkm_link_result_t exchange(
    lkm_bno086_link_t *link, const uint8_t *host_frame,
    uint16_t host_length, bool *received)
{
    uint8_t mosihdr[LINK_HEADER_BYTES] = {0u};
    uint8_t misohdr[LINK_HEADER_BYTES] = {0u};
    lkm_link_result_t result = LKM_LINK_OK;
    lkm_shtp_header_t parsed = {0};
    uint16_t rx_length = 0u;
    uint16_t tx_length = 0u;
    bool null_header = false;
    bool cs_asserted = false;
    bool wake_asserted = false;
    *received = false;

    if (host_frame != NULL) {
        if (host_length < LINK_HEADER_BYTES ||
            host_length > (LINK_HEADER_BYTES + LKM_BNO086_MAX_PAYLOAD))
            return LKM_LINK_BAD_ARGUMENT;
        tx_length = (uint16_t)(host_length - LINK_HEADER_BYTES);
        memcpy(mosihdr, host_frame, LINK_HEADER_BYTES);
        /* Verify the host frame before touching GPIO. */
        lkm_shtp_header_t out = {0};
        if (lkm_shtp_decode_header(mosihdr, LKM_BNO086_MAX_PAYLOAD,
                                   &out) != LKM_SHTP_OK ||
            out.total_length != host_length ||
            out.channel != LINK_CONTROL_CHANNEL)
            return LKM_LINK_BAD_ARGUMENT;
        set_wake(link, GPIO_PIN_RESET);
        wake_asserted = true;
        if (!wait_ready(link)) {
            result = LKM_LINK_HANDSHAKE_TIMEOUT;
            goto cleanup;
        }
    } else {
        if (host_length != 0u) return LKM_LINK_BAD_ARGUMENT;
        if (!irq_low(link)) return LKM_LINK_NO_PACKET;
    }

    set_cs(link, GPIO_PIN_RESET);
    cs_asserted = true;
    if (HAL_SPI_TransmitReceive(link->device->io.spi, mosihdr, misohdr,
                                LINK_HEADER_BYTES,
                                LINK_SPI_TIMEOUT_MS) != HAL_OK) {
        result = LKM_LINK_SPI_FAILURE;
        goto cleanup;
    }
    null_header = (misohdr[0] == 0u && misohdr[1] == 0u &&
                   misohdr[2] == 0u && misohdr[3] == 0u);
    if (!null_header) {
        lkm_shtp_result_t decoded = lkm_shtp_decode_header(
            misohdr, LKM_BNO086_MAX_PAYLOAD, &parsed);
        if (decoded != LKM_SHTP_OK) {
            result = decoded == LKM_SHTP_ERR_CAPACITY ? LKM_LINK_TOO_LARGE :
                     decoded == LKM_SHTP_ERR_CONTINUATION ? LKM_LINK_FRAGMENT_UNSUPPORTED :
                     LKM_LINK_INVALID_HEADER;
            goto cleanup;
        }
        rx_length = parsed.payload_length;
    }

    uint16_t clocks = tx_length > rx_length ? tx_length : rx_length;
    uint16_t offset = 0u;
    while (offset < clocks) {
        uint16_t n = (uint16_t)(clocks - offset);
        if (n > LINK_SPI_CHUNK) n = LINK_SPI_CHUNK;
        uint8_t tx[LINK_SPI_CHUNK] = {0u};
        uint8_t rx[LINK_SPI_CHUNK] = {0u};
        if (offset < tx_length) {
            uint16_t have = (uint16_t)(tx_length - offset);
            if (have > n) have = n;
            memcpy(tx, host_frame + LINK_HEADER_BYTES + offset, have);
        }
        if (HAL_SPI_TransmitReceive(link->device->io.spi, tx, rx, n,
                                    LINK_SPI_TIMEOUT_MS) != HAL_OK) {
            result = LKM_LINK_SPI_FAILURE;
            goto cleanup;
        }
        if (offset < rx_length) {
            uint16_t count = (uint16_t)(rx_length - offset);
            if (count > n) count = n;
            memcpy(link->packet->payload + offset, rx, count);
        }
        offset = (uint16_t)(offset + n);
    }
    if (!null_header) {
        link->packet->header = parsed;
        *received = true;
    } else {
        ++link->null_responses;
    }

cleanup:
    if (cs_asserted) set_cs(link, GPIO_PIN_SET);
    if (wake_asserted) set_wake(link, GPIO_PIN_SET);
    return result;
}

static lkm_link_result_t consume_rx(lkm_bno086_link_t *link, bool received)
{
    if (!received) return LKM_LINK_OK;
    ++link->rx_packets;
    const lkm_stream_result_t sr = lkm_bno086_stream_consume(
        link->stream, &link->packet->header, link->packet->payload,
        HAL_GetTick());
    link->last_stream_result = sr;
    if (sr == LKM_STREAM_OK || sr == LKM_STREAM_IGNORED)
        return LKM_LINK_OK;
    ++link->rejected_stream_packets;
    return LKM_LINK_STREAM_FAILURE;
}

static lkm_link_result_t send_feature(lkm_bno086_link_t *link, bool set)
{
    lkm_link_result_t status = preflight(link);
    if (status != LKM_LINK_OK) return status;
    if (link->device->tx_seq[LINK_CONTROL_CHANNEL] !=
        link->stream->next_control_tx_sequence)
        return latch(link, LKM_LINK_TX_SEQUENCE_CONFLICT);

    uint8_t frame[LKM_STREAM_SET_FRAME_LENGTH] = {0u};
    const lkm_stream_result_t s = set ?
        lkm_bno086_stream_make_set(link->stream, frame) :
        lkm_bno086_stream_make_get(link->stream, frame);
    if (s != LKM_STREAM_OK) {
        link->last_stream_result = s;
        return LKM_LINK_STREAM_FAILURE;
    }
    const uint16_t frame_length = set ? LKM_STREAM_SET_FRAME_LENGTH :
                                       LKM_STREAM_GET_FRAME_LENGTH;
    link->in_transfer = true;
    bool received = false;
    status = exchange(link, frame, frame_length, &received);
    link->in_transfer = false;
    if (status != LKM_LINK_OK) {
        (void)lkm_bno086_stream_mark_transmitted(link->stream, false);
        /* Before CS assertion there was no SPI delivery to the hub. */
        if (status == LKM_LINK_HANDSHAKE_TIMEOUT) return status;
        return latch(link, status);
    }
    if (lkm_bno086_stream_mark_transmitted(link->stream, true) != LKM_STREAM_OK)
        return latch(link, LKM_LINK_STREAM_FAILURE);
    ++link->device->tx_seq[LINK_CONTROL_CHANNEL];
    ++link->tx_committed;
    /* Mark command sent BEFORE processing a possible duplex response. */
    return consume_rx(link, received);
}

lkm_link_result_t lkm_bno086_link_send_set(lkm_bno086_link_t *link)
{
    return send_feature(link, true);
}

lkm_link_result_t lkm_bno086_link_send_get(lkm_bno086_link_t *link)
{
    return send_feature(link, false);
}

lkm_link_result_t lkm_bno086_link_poll(lkm_bno086_link_t *link)
{
    lkm_link_result_t status = preflight(link);
    if (status != LKM_LINK_OK) return status;
    if (!irq_low(link)) return LKM_LINK_NO_PACKET;
    link->in_transfer = true;
    bool received = false;
    status = exchange(link, NULL, 0u, &received);
    link->in_transfer = false;
    if (status != LKM_LINK_OK) return latch(link, status);
    if (!received) return LKM_LINK_NO_PACKET;
    return consume_rx(link, received);
}

lkm_link_result_t lkm_bno086_link_get_attitude(
    const lkm_bno086_link_t *link, uint32_t now_ms,
    lkm_bno086_attitude_t *out)
{
    if (link == NULL || out == NULL) return LKM_LINK_BAD_ARGUMENT;
    if (!link->ready || link->stream == NULL) return LKM_LINK_NOT_READY;
    if (link->fault_latched) return LKM_LINK_FAULT_LATCHED;
    return lkm_bno086_stream_get_attitude(link->stream, now_ms, out) ==
           LKM_STREAM_OK ? LKM_LINK_OK : LKM_LINK_STREAM_FAILURE;
}

const char *lkm_bno086_link_result_text(lkm_link_result_t result)
{
    switch (result) {
    case LKM_LINK_OK: return "OK";
    case LKM_LINK_NO_PACKET: return "NO_PACKET";
    case LKM_LINK_BAD_ARGUMENT: return "BAD_ARGUMENT";
    case LKM_LINK_NOT_PROBED: return "NOT_PROBED";
    case LKM_LINK_NOT_READY: return "NOT_READY";
    case LKM_LINK_HANDSHAKE_TIMEOUT: return "HANDSHAKE_TIMEOUT";
    case LKM_LINK_SPI_FAILURE: return "SPI_FAILURE";
    case LKM_LINK_INVALID_HEADER: return "INVALID_HEADER";
    case LKM_LINK_TOO_LARGE: return "PACKET_TOO_LARGE";
    case LKM_LINK_FRAGMENT_UNSUPPORTED: return "FRAGMENT_UNSUPPORTED";
    case LKM_LINK_TX_SEQUENCE_CONFLICT: return "TX_SEQUENCE_CONFLICT";
    case LKM_LINK_STREAM_FAILURE: return "STREAM_FAILURE";
    case LKM_LINK_FAULT_LATCHED: return "FAULT_LATCHED";
    default: return "UNKNOWN";
    }
}
