#ifndef LKM_BNO086_H
#define LKM_BNO086_H

#include <stdbool.h>
#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "lkm_bno086_shtp.h"

/* Deliberately separate from flight control and ESC outputs. */
#define LKM_BNO086_MAX_PAYLOAD       512u
#define LKM_BNO086_CONTROL_CHANNEL   2u
#define LKM_BNO086_PRODUCT_REQUEST   0xF9u
#define LKM_BNO086_PRODUCT_RESPONSE  0xF8u

typedef enum {
    LKM_BNO086_OK = 0,
    LKM_BNO086_BAD_ARGUMENT,
    LKM_BNO086_IRQ_TIMEOUT,
    LKM_BNO086_SPI_ERROR,
    LKM_BNO086_SHTP_ERROR,
    LKM_BNO086_TOO_LARGE,
    LKM_BNO086_FRAGMENT_UNSUPPORTED,
    LKM_BNO086_STARTUP_INCOMPLETE,
    LKM_BNO086_RESPONSE_TIMEOUT,
    LKM_BNO086_BAD_PRODUCT_RESPONSE
} lkm_bno086_status_t;

typedef struct {
    SPI_HandleTypeDef *spi;
    GPIO_TypeDef *cs_port;
    uint16_t cs_pin;
    GPIO_TypeDef *irq_port;
    uint16_t irq_pin;
    GPIO_TypeDef *reset_port;
    uint16_t reset_pin;
    GPIO_TypeDef *wake_port;
    uint16_t wake_pin;
} lkm_bno086_io_t;

typedef struct {
    lkm_bno086_io_t io;
    uint8_t tx_seq[LKM_SHTP_CHANNEL_MAX + 1u];
    uint32_t received_packets;
    uint8_t startup_mask;
} lkm_bno086_t;

typedef struct {
    lkm_shtp_header_t header;
    uint8_t payload[LKM_BNO086_MAX_PAYLOAD];
} lkm_bno086_packet_t;

typedef struct {
    uint8_t reset_cause;
    uint8_t sw_major;
    uint8_t sw_minor;
    uint16_t sw_patch;
    uint32_t sw_part_number;
    uint32_t sw_build_number;
} lkm_bno086_product_id_t;

void lkm_bno086_bind(lkm_bno086_t *device, const lkm_bno086_io_t *io);

/* Reads one packet, only while H_INTN is asserted low. Always releases CS. */
lkm_bno086_status_t lkm_bno086_receive(
    lkm_bno086_t *device, lkm_bno086_packet_t *packet);

/* Pure parser, independently testable on the host. */
lkm_bno086_status_t lkm_bno086_parse_product_id(
    const lkm_bno086_packet_t *packet, lkm_bno086_product_id_t *id);

/* Blocking, diagnostic-only initial probe; never call from an ISR or flight loop.
 * Flushes startup messages, sends Product ID Request, awaits one response.
 * Does not enable motion reports or drive PWM/ESC outputs. */
lkm_bno086_status_t lkm_bno086_probe(
    lkm_bno086_t *device,
    lkm_bno086_packet_t *scratch,
    lkm_bno086_product_id_t *id);

const char *lkm_bno086_status_text(lkm_bno086_status_t status);

#endif
