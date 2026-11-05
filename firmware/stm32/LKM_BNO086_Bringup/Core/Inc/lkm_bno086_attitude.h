#ifndef LKM_BNO086_ATTITUDE_H
#define LKM_BNO086_ATTITUDE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* HAL-free SH-2 rotation-vector payload decoder for LKM_FC_V1.
 * Input is the PAYLOAD after the 4-byte SHTP header (not the whole frame).
 * Revision V1 intentionally supports only 0x05 and 0x08 sensor reports.
 * It does not configure the sensor, manage SPI or promise fresh data.
 */

#define LKM_ATTITUDE_SHTP_NORMAL_CHANNEL   3u
#define LKM_ATTITUDE_SHTP_WAKE_CHANNEL     4u
#define LKM_ATTITUDE_BASE_TIMESTAMP        0xFBu
#define LKM_ATTITUDE_ROTATION_VECTOR       0x05u
#define LKM_ATTITUDE_GAME_ROTATION_VECTOR  0x08u
#define LKM_ATTITUDE_MAX_PAYLOAD           512u

typedef enum {
    LKM_ATTITUDE_OK = 0,
    LKM_ATTITUDE_ERR_ARGUMENT,
    LKM_ATTITUDE_ERR_CHANNEL,
    LKM_ATTITUDE_ERR_CAPACITY,
    LKM_ATTITUDE_ERR_FORMAT,
    LKM_ATTITUDE_ERR_TIMESTAMP,
    LKM_ATTITUDE_ERR_UNSUPPORTED_REPORT,
    LKM_ATTITUDE_ERR_QUATERNION,
    LKM_ATTITUDE_NO_REPORT
} lkm_attitude_result_t;

typedef struct {
    /* SH-2 quaternion order: i = x, j = y, k = z, real = w. */
    float i;
    float j;
    float k;
    float real;

    /* Intrinsic ZYX yaw-pitch-roll Euler convention, degrees.
       Coordinate-frame / PCB mounting alignment is NOT applied. */
    float roll_deg;
    float pitch_deg;
    float yaw_deg;

    /* 0x05: heading accuracy Q12 -> radians; 0x08: not present. */
    float heading_accuracy_rad;
    bool heading_accuracy_present;

    uint8_t report_id;             /* 0x05 or 0x08 */
    uint8_t sequence;              /* Sensor report sequence (not SHTP seq) */
    uint8_t accuracy_status;       /* 0 unreliable, 1 low, 2 medium, 3 high */
    uint16_t report_delay_100us;   /* Encoded 14-bit delay, 100 us ticks */
    int32_t base_delta_100us;      /* Signed 0xFB base delta, 100 us ticks */
    uint8_t rotation_reports_seen; /* Returns LAST quaternion in this packet */
} lkm_bno086_attitude_t;

/* Parses one complete, already-validated, unfragmented SHTP PAYLOAD.
 * Requires initial 0xFB + 4-byte base delta. Supports one or more
 * consecutive 0x05/0x08 reports and returns the last one.
 * Unknown/mixed sensor reports, timestamp rebases, and fragmentation
 * are NOT accepted in V1. On any non-OK result, *out is unchanged.
 */
lkm_attitude_result_t lkm_bno086_attitude_parse_payload(
    const uint8_t *payload,
    size_t payload_length,
    uint8_t shtp_channel,
    lkm_bno086_attitude_t *out);

const char *lkm_bno086_attitude_result_text(lkm_attitude_result_t result);

#endif /* LKM_BNO086_ATTITUDE_H */
