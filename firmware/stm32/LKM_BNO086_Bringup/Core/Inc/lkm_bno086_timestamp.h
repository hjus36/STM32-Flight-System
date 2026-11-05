#ifndef LKM_BNO086_TIMESTAMP_H
#define LKM_BNO086_TIMESTAMP_H
#include <stdint.h>
#include "lkm_bno086_attitude.h"

/* Host receive timestamp metadata. This is the HAL_GetTick() value captured
 * immediately AFTER the complete SHTP packet has been parsed, NOT the SH-2
 * sensor sample time and NOT a common hardware-trigger timestamp.
 * The packet counter increases on accepted SHTP sensor packets and can wrap.
 */
typedef struct {
    lkm_bno086_attitude_t attitude;
    uint32_t host_received_ms;
    uint32_t host_age_ms;
    uint32_t accepted_packet_counter;
} lkm_bno086_timed_attitude_t;


#endif /* LKM_BNO086_TIMESTAMP_H */
