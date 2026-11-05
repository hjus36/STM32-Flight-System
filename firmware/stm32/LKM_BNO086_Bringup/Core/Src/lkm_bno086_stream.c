#include "lkm_bno086_stream.h"
#include <string.h>

#define LKM_STREAM_TX_NONE 0u
#define LKM_STREAM_TX_SET  1u
#define LKM_STREAM_TX_GET  2u

static bool valid_report_id(uint8_t id)
{
    return (id == LKM_ATTITUDE_ROTATION_VECTOR) ||
           (id == LKM_ATTITUDE_GAME_ROTATION_VECTOR);
}

static void store_le32(uint8_t *dest, uint32_t value)
{
    dest[0] = (uint8_t)(value & 0xFFu);
    dest[1] = (uint8_t)((value >> 8) & 0xFFu);
    dest[2] = (uint8_t)((value >> 16) & 0xFFu);
    dest[3] = (uint8_t)((value >> 24) & 0xFFu);
}

static uint32_t load_le32(const uint8_t *src)
{
    return ((uint32_t)src[0]) | ((uint32_t)src[1] << 8) |
           ((uint32_t)src[2] << 16) | ((uint32_t)src[3] << 24);
}

lkm_stream_result_t lkm_bno086_stream_init(
    lkm_bno086_stream_t *stream, const lkm_stream_config_t *config)
{
    if (stream == NULL || config == NULL) return LKM_STREAM_ERR_ARGUMENT;
    /* These are conservative software policy limits, not sensor capabilities. */
    if (!valid_report_id(config->report_id) ||
        config->report_interval_us < 1000u ||
        config->report_interval_us > 1000000u ||
        config->stale_after_ms == 0u || config->stale_after_ms > 60000u ||
        config->minimum_accuracy > 3u) return LKM_STREAM_ERR_CONFIG;
    memset(stream, 0, sizeof(*stream));
    stream->config = *config;
    return LKM_STREAM_OK;
}

lkm_stream_result_t lkm_bno086_stream_make_set(
    lkm_bno086_stream_t *stream, uint8_t frame[LKM_STREAM_SET_FRAME_LENGTH])
{
    if (stream == NULL || frame == NULL) return LKM_STREAM_ERR_ARGUMENT;
    if (!valid_report_id(stream->config.report_id) || stream->pending_tx != 0u)
        return LKM_STREAM_ERR_STATE;
    uint8_t temp[LKM_STREAM_SET_FRAME_LENGTH] = {0};
    if (lkm_shtp_encode_header(
        LKM_STREAM_SET_PAYLOAD_LENGTH, LKM_STREAM_CONTROL_CHANNEL,
        stream->next_control_tx_sequence, temp) != LKM_SHTP_OK)
        return LKM_STREAM_ERR_FRAME;
    temp[4] = LKM_STREAM_SET_FEATURE_ID;
    temp[5] = stream->config.report_id;
    /* flag, change sensitivity, batch interval, specific config = 0 */
    store_le32(&temp[9], stream->config.report_interval_us);
    memcpy(frame, temp, sizeof(temp));
    stream->pending_tx = LKM_STREAM_TX_SET;
    return LKM_STREAM_OK;
}

lkm_stream_result_t lkm_bno086_stream_make_get(
    lkm_bno086_stream_t *stream, uint8_t frame[LKM_STREAM_GET_FRAME_LENGTH])
{
    if (stream == NULL || frame == NULL) return LKM_STREAM_ERR_ARGUMENT;
    if (!stream->set_sent || stream->pending_tx != 0u)
        return LKM_STREAM_ERR_STATE;
    uint8_t temp[LKM_STREAM_GET_FRAME_LENGTH] = {0};
    if (lkm_shtp_encode_header(
        LKM_STREAM_GET_PAYLOAD_LENGTH, LKM_STREAM_CONTROL_CHANNEL,
        stream->next_control_tx_sequence, temp) != LKM_SHTP_OK)
        return LKM_STREAM_ERR_FRAME;
    temp[4] = LKM_STREAM_GET_FEATURE_ID;
    temp[5] = stream->config.report_id;
    memcpy(frame, temp, sizeof(temp));
    stream->pending_tx = LKM_STREAM_TX_GET;
    return LKM_STREAM_OK;
}

lkm_stream_result_t lkm_bno086_stream_mark_transmitted(
    lkm_bno086_stream_t *stream, bool success)
{
    if (stream == NULL) return LKM_STREAM_ERR_ARGUMENT;
    if (stream->pending_tx == LKM_STREAM_TX_NONE) return LKM_STREAM_ERR_STATE;
    const uint8_t sent_type = stream->pending_tx;
    stream->pending_tx = LKM_STREAM_TX_NONE;
    if (!success) return LKM_STREAM_OK;
    ++stream->next_control_tx_sequence; /* uint8_t wraps at 255 -> 0 */
    if (sent_type == LKM_STREAM_TX_SET) {
        stream->set_sent = true;
        stream->get_sent = false;
        stream->feature_confirmed = false;
        stream->observed_interval_us = 0u;
        stream->sample_valid = false;
        memset(stream->seen_sensor_shtp_seq, 0,
               sizeof(stream->seen_sensor_shtp_seq));
    } else if (sent_type == LKM_STREAM_TX_GET) {
        stream->get_sent = true;
    }
    return LKM_STREAM_OK;
}

lkm_stream_result_t lkm_bno086_stream_consume(
    lkm_bno086_stream_t *stream, const lkm_shtp_header_t *header,
    const uint8_t *payload, uint32_t host_receipt_ms)
{
    if (stream == NULL || header == NULL || payload == NULL)
        return LKM_STREAM_ERR_ARGUMENT;
    if (header->continuation || header->payload_length == 0u ||
        header->payload_length > LKM_STREAM_MAX_PACKET_PAYLOAD ||
        header->total_length != (uint16_t)(header->payload_length + 4u) ||
        header->channel > LKM_SHTP_CHANNEL_MAX) {
        /* A malformed sensor frame must never leave a previously cached
         * attitude usable for control. The transport itself is responsible
         * for dropping/reassembling bytes and guarding the SPI CS line. */
        if (header->channel == LKM_ATTITUDE_SHTP_NORMAL_CHANNEL ||
            header->channel == LKM_ATTITUDE_SHTP_WAKE_CHANNEL) {
            stream->sample_valid = false;
            ++stream->rejected_samples;
        }
        return LKM_STREAM_ERR_FRAME;
    }

    if (header->channel == LKM_STREAM_CONTROL_CHANNEL) {
        if (payload[0] != LKM_STREAM_GET_FEATURE_REPLY)
            return LKM_STREAM_IGNORED;
        if (!stream->set_sent || !stream->get_sent)
            return LKM_STREAM_ERR_STATE;
        if (header->payload_length != 17u ||
            payload[1] != stream->config.report_id) {
            stream->feature_confirmed = false;
            stream->sample_valid = false;
            return LKM_STREAM_ERR_FEATURE_RESPONSE;
        }
        const uint32_t interval = load_le32(&payload[5]);
        ++stream->feature_responses;
        stream->observed_interval_us = interval;
        if (interval != stream->config.report_interval_us) {
            stream->feature_confirmed = false;
            stream->sample_valid = false;
            return LKM_STREAM_ERR_FEATURE_RESPONSE;
        }
        stream->feature_confirmed = true;
        return LKM_STREAM_OK;
    }

    if (header->channel != LKM_ATTITUDE_SHTP_NORMAL_CHANNEL &&
        header->channel != LKM_ATTITUDE_SHTP_WAKE_CHANNEL)
        return LKM_STREAM_IGNORED;

    if (!stream->feature_confirmed) return LKM_STREAM_ERR_STATE;

    const uint8_t slot = (uint8_t)(header->channel - LKM_ATTITUDE_SHTP_NORMAL_CHANNEL);
    if (stream->seen_sensor_shtp_seq[slot] &&
        header->sequence != (uint8_t)(stream->last_sensor_shtp_seq[slot] + 1u)) {
        stream->last_sensor_shtp_seq[slot] = header->sequence;
        stream->sample_valid = false;
        ++stream->seq_gaps;
        ++stream->rejected_samples;
        return LKM_STREAM_ERR_SEQUENCE;
    }
    stream->seen_sensor_shtp_seq[slot] = true;
    stream->last_sensor_shtp_seq[slot] = header->sequence;

    lkm_bno086_attitude_t parsed = {0};
    const lkm_attitude_result_t status = lkm_bno086_attitude_parse_payload(
        payload, header->payload_length, header->channel, &parsed);
    if (status != LKM_ATTITUDE_OK ||
        parsed.report_id != stream->config.report_id) {
        stream->sample_valid = false;
        ++stream->rejected_samples;
        return LKM_STREAM_ERR_ATTITUDE;
    }

    stream->last_attitude = parsed;
    stream->last_receive_ms = host_receipt_ms;
    stream->sample_valid = true;
    ++stream->valid_samples;
    return LKM_STREAM_OK;
}

lkm_stream_result_t lkm_bno086_stream_get_attitude(
    const lkm_bno086_stream_t *stream, uint32_t host_now_ms,
    lkm_bno086_attitude_t *out)
{
    if (stream == NULL || out == NULL) return LKM_STREAM_ERR_ARGUMENT;
    if (!stream->feature_confirmed) return LKM_STREAM_ERR_STATE;
    if (!stream->sample_valid) return LKM_STREAM_ERR_NO_SAMPLE;
    /* This subtraction is valid across a 32-bit tick wrap for short TTLs. */
    if ((uint32_t)(host_now_ms - stream->last_receive_ms) >
        stream->config.stale_after_ms) return LKM_STREAM_ERR_STALE;
    if (stream->last_attitude.accuracy_status < stream->config.minimum_accuracy)
        return LKM_STREAM_ERR_QUALITY;
    *out = stream->last_attitude;
    return LKM_STREAM_OK;
}

const char *lkm_bno086_stream_result_text(lkm_stream_result_t result)
{
    switch (result) {
    case LKM_STREAM_OK: return "OK";
    case LKM_STREAM_IGNORED: return "IGNORED";
    case LKM_STREAM_ERR_ARGUMENT: return "BAD_ARGUMENT";
    case LKM_STREAM_ERR_CONFIG: return "BAD_CONFIG";
    case LKM_STREAM_ERR_STATE: return "STATE_NOT_READY";
    case LKM_STREAM_ERR_FRAME: return "BAD_SHTP_FRAME";
    case LKM_STREAM_ERR_SEQUENCE: return "SHTP_SEQUENCE_GAP";
    case LKM_STREAM_ERR_FEATURE_RESPONSE: return "FEATURE_MISMATCH";
    case LKM_STREAM_ERR_ATTITUDE: return "BAD_ATTITUDE_PAYLOAD";
    case LKM_STREAM_ERR_STALE: return "STALE_SAMPLE";
    case LKM_STREAM_ERR_QUALITY: return "LOW_ACCURACY";
    case LKM_STREAM_ERR_NO_SAMPLE: return "NO_SAMPLE";
    default: return "UNKNOWN";
    }
}
