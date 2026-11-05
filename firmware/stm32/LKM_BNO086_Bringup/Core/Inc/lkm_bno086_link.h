#ifndef LKM_BNO086_LINK_H
#define LKM_BNO086_LINK_H

/* STM32 HAL SPI2 ↔ BNO08x SHTP ↔ existing SH-2 stream adapter.
 * Experimental, blocking, offline-tested ONLY. NOT a flight-loop driver.
 * This module is not called by main.c; no motor/ESC functionality.
 * OWNERSHIP RULE: after bind, no other code may write control channel 2,
 * including lkm_bno086_probe(), until link is discarded/reinitialized.
 */
#include <stdbool.h>
#include <stdint.h>
#include "lkm_bno086.h"
#include "lkm_bno086_stream.h"

typedef enum {
    LKM_LINK_OK = 0,
    LKM_LINK_NO_PACKET,
    LKM_LINK_BAD_ARGUMENT,
    LKM_LINK_NOT_PROBED,
    LKM_LINK_NOT_READY,
    LKM_LINK_HANDSHAKE_TIMEOUT,
    LKM_LINK_SPI_FAILURE,
    LKM_LINK_INVALID_HEADER,
    LKM_LINK_TOO_LARGE,
    LKM_LINK_FRAGMENT_UNSUPPORTED,
    LKM_LINK_TX_SEQUENCE_CONFLICT,
    LKM_LINK_STREAM_FAILURE,
    LKM_LINK_FAULT_LATCHED
} lkm_link_result_t;

typedef struct {
    lkm_bno086_t *device;
    lkm_bno086_stream_t *stream;
    lkm_bno086_packet_t *packet; /* persistent caller-owned scratch >= 512 B */
    bool ready;
    bool fault_latched;   /* HAL uncertainty / broken framing => re-probe needed */
    bool in_transfer;
    uint32_t tx_committed;
    uint32_t rx_packets;
    uint32_t null_responses;
    uint32_t rejected_stream_packets;
    lkm_stream_result_t last_stream_result;
} lkm_bno086_link_t;

/* Requires successful Product ID probe first. 'probe_verified' must be taken
 * from the actual probe result (not an assumption). Existing g_device in
 * lkm_bno086_app.c is static, so this is NOT wired into that app yet.
 * Initializes stream, acquiring control TX sequence from device->tx_seq[2].
 */
lkm_link_result_t lkm_bno086_link_init(
    lkm_bno086_link_t *link, lkm_bno086_t *device,
    lkm_bno086_stream_t *stream, lkm_bno086_packet_t *packet,
    const lkm_stream_config_t *config, bool probe_verified);

/* Blocking calls for diagnostic orchestration only, NEVER an ISR or 1kHz ISR.
 * After an ambiguous SPI error or a malformed/truncated transfer, link locks
 * until the sensor is reset/re-probed and link_init is called again.
 */
lkm_link_result_t lkm_bno086_link_send_set(lkm_bno086_link_t *link);
lkm_link_result_t lkm_bno086_link_send_get(lkm_bno086_link_t *link);
/* Polls at most one inbound SHTP transaction; no busy-wait on empty IRQ. */
lkm_link_result_t lkm_bno086_link_poll(lkm_bno086_link_t *link);
/* Outputs only a previously confirmed, fresh, quality-gated attitude. */
lkm_link_result_t lkm_bno086_link_get_attitude(
    const lkm_bno086_link_t *link, uint32_t now_ms,
    lkm_bno086_attitude_t *out);
const char *lkm_bno086_link_result_text(lkm_link_result_t result);

#endif
