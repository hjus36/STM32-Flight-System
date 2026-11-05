#include "lkm_bno086_attitude.h"

#include <math.h>
#include <stdint.h>

#define LKM_ATTITUDE_Q14_INV     (1.0f / 16384.0f)
#define LKM_ATTITUDE_Q12_INV     (1.0f / 4096.0f)
#define LKM_ATTITUDE_RAD_TO_DEG  (57.29577951308232f)
/* Reject corrupted/incomplete quaternion vectors before normalization. */
#define LKM_ATTITUDE_NORM2_MIN   (0.80f)
#define LKM_ATTITUDE_NORM2_MAX   (1.20f)

static uint16_t read_le_u16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static int16_t read_le_s16(const uint8_t *p)
{
    const uint16_t bits = read_le_u16(p);
    const int32_t signed_value = (bits <= 0x7FFFu)
        ? (int32_t)bits : ((int32_t)bits - 65536);
    return (int16_t)signed_value;
}

static uint32_t read_le_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8)
         | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static int32_t read_le_s32(const uint8_t *p)
{
    const uint32_t bits = read_le_u32(p);
    const int64_t signed_value = (bits <= UINT32_C(0x7FFFFFFF))
        ? (int64_t)bits : ((int64_t)bits - INT64_C(4294967296));
    return (int32_t)signed_value;
}

static float clamp_unit(float x)
{
    if (x > 1.0f) return 1.0f;
    if (x < -1.0f) return -1.0f;
    return x;
}

static lkm_attitude_result_t decode_one(
    const uint8_t *report, size_t report_len,
    int32_t base_delta, lkm_bno086_attitude_t *decoded)
{
    if (report_len != 12u && report_len != 14u) {
        return LKM_ATTITUDE_ERR_FORMAT;
    }

    const float x = (float)read_le_s16(report + 4u) * LKM_ATTITUDE_Q14_INV;
    const float y = (float)read_le_s16(report + 6u) * LKM_ATTITUDE_Q14_INV;
    const float z = (float)read_le_s16(report + 8u) * LKM_ATTITUDE_Q14_INV;
    const float w = (float)read_le_s16(report + 10u) * LKM_ATTITUDE_Q14_INV;

    const float norm2 = x*x + y*y + z*z + w*w;
    if (!(norm2 >= LKM_ATTITUDE_NORM2_MIN && norm2 <= LKM_ATTITUDE_NORM2_MAX)) {
        return LKM_ATTITUDE_ERR_QUATERNION;
    }

    const float norm_inv = 1.0f / sqrtf(norm2);
    const float nx = x * norm_inv;
    const float ny = y * norm_inv;
    const float nz = z * norm_inv;
    const float nw = w * norm_inv;

    /* ZYX Euler angles: R = Rz(yaw) * Ry(pitch) * Rx(roll). */
    const float sinr = 2.0f * (nw*nx + ny*nz);
    const float cosr = 1.0f - 2.0f * (nx*nx + ny*ny);
    const float sinp = clamp_unit(2.0f * (nw*ny - nz*nx));
    const float siny = 2.0f * (nw*nz + nx*ny);
    const float cosy = 1.0f - 2.0f * (ny*ny + nz*nz);

    decoded->i = nx;
    decoded->j = ny;
    decoded->k = nz;
    decoded->real = nw;
    decoded->roll_deg = atan2f(sinr, cosr) * LKM_ATTITUDE_RAD_TO_DEG;
    decoded->pitch_deg = asinf(sinp) * LKM_ATTITUDE_RAD_TO_DEG;
    decoded->yaw_deg = atan2f(siny, cosy) * LKM_ATTITUDE_RAD_TO_DEG;

    decoded->report_id = report[0];
    decoded->sequence = report[1];
    decoded->accuracy_status = (uint8_t)(report[2] & 0x03u);
    decoded->report_delay_100us =
        (uint16_t)(((uint16_t)(report[2] & 0xFCu) << 6) | (uint16_t)report[3]);
    decoded->base_delta_100us = base_delta;
    decoded->heading_accuracy_present = (report[0] == LKM_ATTITUDE_ROTATION_VECTOR);
    decoded->heading_accuracy_rad = decoded->heading_accuracy_present
        ? (float)read_le_u16(report + 12u) * LKM_ATTITUDE_Q12_INV : 0.0f;

    return LKM_ATTITUDE_OK;
}

lkm_attitude_result_t lkm_bno086_attitude_parse_payload(
    const uint8_t *payload,
    size_t payload_length,
    uint8_t shtp_channel,
    lkm_bno086_attitude_t *out)
{
    if (payload == NULL || out == NULL) return LKM_ATTITUDE_ERR_ARGUMENT;
    if (shtp_channel != LKM_ATTITUDE_SHTP_NORMAL_CHANNEL &&
        shtp_channel != LKM_ATTITUDE_SHTP_WAKE_CHANNEL) {
        return LKM_ATTITUDE_ERR_CHANNEL;
    }
    if (payload_length > LKM_ATTITUDE_MAX_PAYLOAD) {
        return LKM_ATTITUDE_ERR_CAPACITY;
    }
    if (payload_length < 5u || payload[0] != LKM_ATTITUDE_BASE_TIMESTAMP) {
        return LKM_ATTITUDE_ERR_FORMAT;
    }

    const uint32_t raw_delta = read_le_u32(payload + 1u);
    /* 0x7FFFFFFF is the reserved "unrepresentable base delta" sentinel. */
    if (raw_delta == UINT32_C(0x7FFFFFFF)) {
        return LKM_ATTITUDE_ERR_TIMESTAMP;
    }
    const int32_t base_delta = read_le_s32(payload + 1u);

    lkm_bno086_attitude_t candidate = {0};
    size_t cursor = 5u;
    uint8_t count = 0u;
    while (cursor < payload_length) {
        const uint8_t report_id = payload[cursor];
        size_t report_length;
        if (report_id == LKM_ATTITUDE_ROTATION_VECTOR) {
            report_length = 14u;
        } else if (report_id == LKM_ATTITUDE_GAME_ROTATION_VECTOR) {
            report_length = 12u;
        } else {
            /* Cannot safely infer unknown report lengths or skip rebases. */
            return LKM_ATTITUDE_ERR_UNSUPPORTED_REPORT;
        }
        if ((payload_length - cursor) < report_length) {
            return LKM_ATTITUDE_ERR_FORMAT;
        }
        const lkm_attitude_result_t status = decode_one(
            payload + cursor, report_length, base_delta, &candidate);
        if (status != LKM_ATTITUDE_OK) return status;
        ++count;
        cursor += report_length;
    }

    if (count == 0u) return LKM_ATTITUDE_NO_REPORT;
    candidate.rotation_reports_seen = count;
    *out = candidate; /* Commit only after the whole packet is validated. */
    return LKM_ATTITUDE_OK;
}

const char *lkm_bno086_attitude_result_text(lkm_attitude_result_t result)
{
    switch (result) {
        case LKM_ATTITUDE_OK: return "OK";
        case LKM_ATTITUDE_ERR_ARGUMENT: return "BAD_ARGUMENT";
        case LKM_ATTITUDE_ERR_CHANNEL: return "BAD_CHANNEL";
        case LKM_ATTITUDE_ERR_CAPACITY: return "PAYLOAD_TOO_LARGE";
        case LKM_ATTITUDE_ERR_FORMAT: return "BAD_FORMAT";
        case LKM_ATTITUDE_ERR_TIMESTAMP: return "INVALID_BASE_TIMESTAMP";
        case LKM_ATTITUDE_ERR_UNSUPPORTED_REPORT: return "UNSUPPORTED_REPORT";
        case LKM_ATTITUDE_ERR_QUATERNION: return "INVALID_QUATERNION";
        case LKM_ATTITUDE_NO_REPORT: return "NO_ROTATION_REPORT";
        default: return "UNKNOWN_RESULT";
    }
}
