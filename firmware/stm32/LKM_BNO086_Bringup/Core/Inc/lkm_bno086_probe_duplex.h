#ifndef LKM_BNO086_PROBE_DUPLEX_H
#define LKM_BNO086_PROBE_DUPLEX_H

/* Alternative initial Product-ID probe for BNO086, with duplex SHTP SPI.
 * This is a diagnostic-only, blocking, experimental implementation.
 * NOT for use in an ISR, 1 kHz control tick, or motor-arming path.
 * It does NOT replace the legacy probe or modify its API.
 */
#include "lkm_bno086.h"

/* Resets the hub, reads bounded startup messages, sends Product-ID Request
 * over HAL_SPI_TransmitReceive, consumes a concurrent inbound cargo if any,
 * then waits for Product-ID Response. Calls are host-mock tested, not tested
 * on an actual BNO086. The caller must not retry after any SPI failure
 * without resetting/re-probing, because a write may have been delivered.
 */
lkm_bno086_status_t lkm_bno086_probe_duplex(
    lkm_bno086_t *device,
    lkm_bno086_packet_t *scratch,
    lkm_bno086_product_id_t *id);

#endif /* LKM_BNO086_PROBE_DUPLEX_H */
