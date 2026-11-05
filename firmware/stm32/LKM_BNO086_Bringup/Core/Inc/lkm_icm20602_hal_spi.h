#ifndef LKM_ICM20602_HAL_SPI_H
#define LKM_ICM20602_HAL_SPI_H

/* STM32F405 HAL adapter for the existing pin-agnostic ICM-20602 driver.
 * NOT connected to main.c. No pin or SPI instance is assumed.
 * A PCB-reviewed, dedicated SPI bus must be supplied by the application.
 */
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "lkm_icm20602_gyro.h"

#define LKM_ICM20602_HAL_MAX_FRAME_BYTES 7u
#define LKM_ICM20602_HAL_SPI_MIN_HZ 100000u
#define LKM_ICM20602_HAL_SPI_MAX_HZ 10000000u

typedef struct {
    SPI_HandleTypeDef *spi;
    GPIO_TypeDef *cs_port;
    uint16_t cs_pin;
    uint32_t timeout_ms;
    /* These are explicit REVIEW GATES, not measurements or automatic checks.
       Do not set them true without verifying LKM_FC_V1 Altium netlist and
       CubeMX pinmux. Shared bus requires arbitration across BOTH drivers;
       this V1 intentionally refuses to run on a shared bus. */
    bool pcb_pinmap_reviewed;
    bool dedicated_spi_bus_reviewed;
} lkm_icm20602_hal_spi_config_t;

typedef struct {
    lkm_icm20602_hal_spi_config_t config;
    bool initialized;
    bool fault_latched;
    uint32_t successful_transactions;
    uint32_t failed_transactions;
} lkm_icm20602_hal_spi_t;

/* Init and MakeBus do not drive GPIO, call SPI, or delay. */
bool LKM_ICM20602_HAL_Init(lkm_icm20602_hal_spi_t *adapter,
                          const lkm_icm20602_hal_spi_config_t *config);
bool LKM_ICM20602_HAL_MakeBus(lkm_icm20602_hal_spi_t *adapter,
                             lkm_icm20602_bus_t *out_bus);

/* This is the driver's atomic CS-low SPI register transfer callback.
   On an HAL transfer error, CS is released, output bytes are cleared,
   and the adapter is latched faulted (no blind retry). */
bool LKM_ICM20602_HAL_Transfer(void *context, const uint8_t *tx,
                               uint8_t *rx, size_t length);
uint32_t LKM_ICM20602_HAL_Millis(void *context);
void LKM_ICM20602_HAL_DelayMs(void *context, uint32_t millis);

#endif
