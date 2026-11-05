#ifndef LKM_BNO086_STREAM_H
#define LKM_BNO086_STREAM_H

/* PC-offline, HAL-free feature-configuration and attitude-data gate.
 * This module does NOT perform SPI transactions, sensor reset, PWM, or arming.
 * Existing lkm_bno086_shtp.h and lkm_bno086_attitude.h are required.
 * No flight-control decisions may be based solely on this module.
 */
#include <stdbool.h>
#include <stdint.h>
#include "lkm_bno086_shtp.h"
#include "lkm_bno086_attitude.h"

#define LKM_STREAM_SET_FEATURE_ID      0xFDu
#define LKM_STREAM_GET_FEATURE_ID      0xFEu
#define LKM_STREAM_GET_FEATURE_REPLY   0xFCu
#define LKM_STREAM_CONTROL_CHANNEL     2u
#define LKM_STREAM_SET_PAYLOAD_LENGTH 17u
#define LKM_STREAM_GET_PAYLOAD_LENGTH 2u
#define LKM_STREAM_SET_FRAME_LENGTH   21u
#define LKM_STREAM_GET_FRAME_LENGTH    6u
#define LKM_STREAM_MAX_PACKET_PAYLOAD 512u

typedef enum {
    LKM_STREAM_OK = 0,
    LKM_STREAM_IGNORED,
    LKM_STREAM_ERR_ARGUMENT,
    LKM_STREAM_ERR_CONFIG,
    LKM_STREAM_ERR_STATE,
    LKM_STREAM_ERR_FRAME,
    LKM_STREAM_ERR_SEQUENCE,
    LKM_STREAM_ERR_FEATURE_RESPONSE,
    LKM_STREAM_ERR_ATTITUDE,
    LKM_STREAM_ERR_STALE,
    LKM_STREAM_ERR_QUALITY,
    LKM_STREAM_ERR_NO_SAMPLE
} lkm_stream_result_t;

typedef struct {
    uint8_t report_id;         /* 0x05 or 0x08 only */
    uint32_t report_interval_us; /* e.g. 10000 = 100 Hz */
    uint32_t stale_after_ms;    /* maximum host receipt age, nonzero */
    uint8_t minimum_accuracy;   /* 0..3; use >= 1 for initial diagnostics */
} lkm_stream_config_t;

typedef struct {
    lkm_stream_config_t config;
    uint8_t next_control_tx_sequence;
    uint8_t pending_tx;       /* internal: 0=none, 1=set, 2=get */
    bool set_sent;
    bool get_sent;
    bool feature_confirmed;
    uint32_t observed_interval_us;

    bool seen_sensor_shtp_seq[2];
    uint8_t last_sensor_shtp_seq[2];
    bool sample_valid;
    lkm_bno086_attitude_t last_attitude;
    uint32_t last_receive_ms;

    uint32_t valid_samples;
    uint32_t rejected_samples;
    uint32_t seq_gaps;
    uint32_t feature_responses;
} lkm_bno086_stream_t;

/* Returns ERR_CONFIG for unknown report IDs or unreasonable timer values.
 * Only supports 0x05 / 0x08; not a complete SH-2 driver. */
lkm_stream_result_t lkm_bno086_stream_init(
    lkm_bno086_stream_t *stream, const lkm_stream_config_t *config);

/* Builds a complete SHTP frame including 4-byte header. Must not send on SPI
 * until the MCU is ready and transport-level H_INTN/WAKE handshake succeeds.
 * These functions do NOT advance sequence/state until mark_transmitted(true).
 */
lkm_stream_result_t lkm_bno086_stream_make_set(
    lkm_bno086_stream_t *stream, uint8_t frame[LKM_STREAM_SET_FRAME_LENGTH]);
lkm_stream_result_t lkm_bno086_stream_make_get(
    lkm_bno086_stream_t *stream, uint8_t frame[LKM_STREAM_GET_FRAME_LENGTH]);
/* Call ONLY after a transport TX result: true for success, false to cancel.
 * Sequence number advances only on successful transmission. */
lkm_stream_result_t lkm_bno086_stream_mark_transmitted(
    lkm_bno086_stream_t *stream, bool success);

/* Feeds a completed, already validated/non-fragmented received SHTP packet.
 * Packet must be obtained through a transport with its own buffer and CS guard.
 * Host timestamp is receipt time, NOT the sensor's 0xFB timestamp.
 * A successful 0xFD TX alone does not imply enabled/configured status:
 * Get Feature 0xFE must be sent and matching 0xFC response received.
 */
lkm_stream_result_t lkm_bno086_stream_consume(
    lkm_bno086_stream_t *stream, const lkm_shtp_header_t *header,
    const uint8_t *payload, uint32_t host_receipt_ms);

/* Fails closed when configuration unconfirmed, sample missing, stale, or
 * below minimum accuracy. On non-OK leaves *out unchanged. */
lkm_stream_result_t lkm_bno086_stream_get_attitude(
    const lkm_bno086_stream_t *stream, uint32_t host_now_ms,
    lkm_bno086_attitude_t *out);

const char *lkm_bno086_stream_result_text(lkm_stream_result_t result);

#endif /* LKM_BNO086_STREAM_H */
