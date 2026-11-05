#ifndef LKM_BNO086_SHTP_H
#define LKM_BNO086_SHTP_H

#include <stdbool.h>
#include <stdint.h>

/* Protocol-only module: no HAL, GPIO, SPI, motor, or scheduler dependencies. */
#define LKM_SHTP_HEADER_BYTES   (4u)
#define LKM_SHTP_CHANNEL_MAX    (5u)
#define LKM_SHTP_MAX_TOTAL      (0x7FFFu)

typedef enum {
    LKM_SHTP_OK = 0,
    LKM_SHTP_ERR_ARGUMENT,
    LKM_SHTP_ERR_LENGTH,
    LKM_SHTP_ERR_CHANNEL,
    LKM_SHTP_ERR_CAPACITY,
    LKM_SHTP_ERR_CONTINUATION
} lkm_shtp_result_t;

typedef struct {
    uint16_t total_length;   /* Includes the four-byte SHTP header. */
    uint16_t payload_length;
    uint8_t channel;
    uint8_t sequence;
    bool continuation;
} lkm_shtp_header_t;

/* Decode one received SHTP header; payload is not consumed here.
   max_payload is a host-side buffer limit, not a device packet limit.
   Continuation is reported to the caller, never silently ignored. */
lkm_shtp_result_t lkm_shtp_decode_header(
    const uint8_t bytes[LKM_SHTP_HEADER_BYTES],
    uint16_t max_payload,
    lkm_shtp_header_t *decoded);

/* Encode a non-fragmented host-to-device header. */
lkm_shtp_result_t lkm_shtp_encode_header(
    uint16_t payload_length,
    uint8_t channel,
    uint8_t sequence,
    uint8_t bytes[LKM_SHTP_HEADER_BYTES]);

#endif
