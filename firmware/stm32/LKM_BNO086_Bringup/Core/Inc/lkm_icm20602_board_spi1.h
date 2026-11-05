#ifndef LKM_ICM20602_BOARD_SPI1_H
#define LKM_ICM20602_BOARD_SPI1_H

/* LKM_FC_V1: ICM-20602 board-specific, SPI1-only safe callback binding.
 * Source of board map: LKM_FC_V1_Schematic (02_MCU, 03_Sensors), 2026-11-02.
 * SPI1 = PA5 SCK / PA6 MISO / PA7 MOSI, CS=PC4, INT=PC5.
 * No SPI transaction, gyro configure, calibration, motor control, or main()
 * modification occurs in this module. Caller MUST use generated CubeMX SPI1.
 * Schematic routing is not proof of assembled-board functionality or axes.
 */

#include <stdbool.h>
#include <stdint.h>
#include "lkm_icm20602_hal_spi.h"

typedef enum {
    LKM_ICM_BOARD_OK = 0,
    LKM_ICM_BOARD_ERR_ARGUMENT,
    LKM_ICM_BOARD_ERR_REVIEW_NOT_APPROVED,
    LKM_ICM_BOARD_ERR_PINMAP,
    LKM_ICM_BOARD_ERR_SPI_NOT_READY,
    LKM_ICM_BOARD_ERR_CS_NOT_INACTIVE,
    LKM_ICM_BOARD_ERR_ADAPTER_CONFIG
} lkm_icm20602_board_result_t;

/* No peripheral I/O, no SPI clocks. HAL_GetState and GPIO_ReadPin only.
 * verified_pcb_to_cubemx is an explicit software pinmap review gate, NOT
 * proof of fabricated PCB or sensor-axis orientation. In firmware production
 * do not set true before design/netlist and generated pinmux are reviewed.
 * timeout_ms must be finite: range [1,1000], default suggestion 10.
 */
lkm_icm20602_board_result_t LKM_ICM20602_BoardSPI1_Prepare(
    lkm_icm20602_hal_spi_t *adapter,
    lkm_icm20602_bus_t *out_bus,
    bool verified_pcb_to_cubemx,
    uint32_t timeout_ms);
const char *LKM_ICM20602_BoardSPI1_ResultText(lkm_icm20602_board_result_t result);

#endif /* LKM_ICM20602_BOARD_SPI1_H */
