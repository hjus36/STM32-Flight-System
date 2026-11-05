#include "lkm_bno086_shtp.h"

lkm_shtp_result_t lkm_shtp_decode_header(
    const uint8_t bytes[LKM_SHTP_HEADER_BYTES],
    uint16_t max_payload,
    lkm_shtp_header_t *decoded)
{
    if ((bytes == (const uint8_t *)0) || (decoded == (lkm_shtp_header_t *)0)) {
        return LKM_SHTP_ERR_ARGUMENT;
    }

    const uint16_t raw_length =
        (uint16_t)((uint16_t)bytes[0] | ((uint16_t)bytes[1] << 8));
    const bool continuation = (raw_length & (uint16_t)0x8000u) != 0u;
    const uint16_t total_length = (uint16_t)(raw_length & (uint16_t)0x7FFFu);

    /* Reject invalid or reserved length values. */
    if ((raw_length == (uint16_t)0xFFFFu) ||
        (total_length < (uint16_t)LKM_SHTP_HEADER_BYTES)) {
        return LKM_SHTP_ERR_LENGTH;
    }
    if (bytes[2] > (uint8_t)LKM_SHTP_CHANNEL_MAX) {
        return LKM_SHTP_ERR_CHANNEL;
    }

    const uint16_t payload_length =
        (uint16_t)(total_length - (uint16_t)LKM_SHTP_HEADER_BYTES);
    if (payload_length > max_payload) {
        return LKM_SHTP_ERR_CAPACITY;
    }

    decoded->total_length = total_length;
    decoded->payload_length = payload_length;
    decoded->channel = bytes[2];
    decoded->sequence = bytes[3];
    decoded->continuation = continuation;

    /* A caller can explicitly handle segmented packets in a later transport layer. */
    return continuation ? LKM_SHTP_ERR_CONTINUATION : LKM_SHTP_OK;
}

lkm_shtp_result_t lkm_shtp_encode_header(
    uint16_t payload_length,
    uint8_t channel,
    uint8_t sequence,
    uint8_t bytes[LKM_SHTP_HEADER_BYTES])
{
    if (bytes == (uint8_t *)0) {
        return LKM_SHTP_ERR_ARGUMENT;
    }
    if (channel > (uint8_t)LKM_SHTP_CHANNEL_MAX) {
        return LKM_SHTP_ERR_CHANNEL;
    }
    if ((uint32_t)payload_length + (uint32_t)LKM_SHTP_HEADER_BYTES >
        (uint32_t)LKM_SHTP_MAX_TOTAL) {
        return LKM_SHTP_ERR_LENGTH;
    }

    const uint16_t total = (uint16_t)(payload_length + LKM_SHTP_HEADER_BYTES);
    bytes[0] = (uint8_t)(total & (uint16_t)0xFFu);
    bytes[1] = (uint8_t)(total >> 8);
    bytes[2] = channel;
    bytes[3] = sequence;
    return LKM_SHTP_OK;
}
