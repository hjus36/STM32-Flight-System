#include "lkm_icm20602_board_spi1.h"
#include "main.h"
#include "spi.h"
#include <string.h>

/* Deliberate compile-time checks: generated CubeMX names required. */
#ifndef ICM_CS_Pin
#error "CubeMX GPIO label ICM_CS (PC4) missing in main.h"
#endif
#ifndef ICM_INT_Pin
#error "CubeMX GPIO label ICM_INT (PC5) missing in main.h"
#endif
#ifndef ICM_CS_GPIO_Port
#error "CubeMX ICM_CS_GPIO_Port missing in main.h"
#endif
#ifndef ICM_INT_GPIO_Port
#error "CubeMX ICM_INT_GPIO_Port missing in main.h"
#endif
/* HAL GPIO_PIN_n macros include C casts, so do not compare in #if. */
_Static_assert(ICM_CS_Pin == GPIO_PIN_4, "LKM_FC_V1 ICM_CS must be PC4");
_Static_assert(ICM_INT_Pin == GPIO_PIN_5, "LKM_FC_V1 ICM_INT must be PC5");

lkm_icm20602_board_result_t LKM_ICM20602_BoardSPI1_Prepare(
    lkm_icm20602_hal_spi_t *adapter,
    lkm_icm20602_bus_t *out_bus,
    bool verified_pcb_to_cubemx,
    uint32_t timeout_ms)
{
    if (out_bus != NULL) memset(out_bus, 0, sizeof(*out_bus));
    if (adapter != NULL) memset(adapter, 0, sizeof(*adapter));
    if (adapter == NULL || out_bus == NULL)
        return LKM_ICM_BOARD_ERR_ARGUMENT;
    if (!verified_pcb_to_cubemx)
        return LKM_ICM_BOARD_ERR_REVIEW_NOT_APPROVED;

    /* SPI1 dedicated to ICM. Never bind SPI2/BNO086 or SPI3/LPS22HH. */
    if (ICM_CS_GPIO_Port != GPIOC || ICM_CS_Pin != GPIO_PIN_4 ||
        ICM_INT_GPIO_Port != GPIOC || ICM_INT_Pin != GPIO_PIN_5 ||
        hspi1.Instance != SPI1)
        return LKM_ICM_BOARD_ERR_PINMAP;
    if (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY)
        return LKM_ICM_BOARD_ERR_SPI_NOT_READY;
    if (HAL_GPIO_ReadPin(ICM_CS_GPIO_Port, ICM_CS_Pin) != GPIO_PIN_SET)
        return LKM_ICM_BOARD_ERR_CS_NOT_INACTIVE;

    lkm_icm20602_hal_spi_config_t config;
    memset(&config, 0, sizeof(config));
    config.spi = &hspi1;
    config.cs_port = ICM_CS_GPIO_Port;
    config.cs_pin = ICM_CS_Pin;
    config.timeout_ms = timeout_ms;
    config.pcb_pinmap_reviewed = verified_pcb_to_cubemx;
    config.dedicated_spi_bus_reviewed = verified_pcb_to_cubemx;
    if (!LKM_ICM20602_HAL_Init(adapter, &config) ||
        !LKM_ICM20602_HAL_MakeBus(adapter, out_bus)) {
        memset(adapter, 0, sizeof(*adapter));
        memset(out_bus, 0, sizeof(*out_bus));
        return LKM_ICM_BOARD_ERR_ADAPTER_CONFIG;
    }
    return LKM_ICM_BOARD_OK;
}

const char *LKM_ICM20602_BoardSPI1_ResultText(lkm_icm20602_board_result_t result)
{
    switch (result) {
    case LKM_ICM_BOARD_OK: return "OK";
    case LKM_ICM_BOARD_ERR_ARGUMENT: return "ERR_ARGUMENT";
    case LKM_ICM_BOARD_ERR_REVIEW_NOT_APPROVED: return "ERR_REVIEW_NOT_APPROVED";
    case LKM_ICM_BOARD_ERR_PINMAP: return "ERR_PINMAP";
    case LKM_ICM_BOARD_ERR_SPI_NOT_READY: return "ERR_SPI_NOT_READY";
    case LKM_ICM_BOARD_ERR_CS_NOT_INACTIVE: return "ERR_CS_NOT_INACTIVE";
    case LKM_ICM_BOARD_ERR_ADAPTER_CONFIG: return "ERR_ADAPTER_CONFIG";
    default: return "ERR_UNKNOWN";
    }
}
