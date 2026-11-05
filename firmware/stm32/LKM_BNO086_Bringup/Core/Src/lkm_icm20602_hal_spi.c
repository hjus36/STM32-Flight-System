#include "lkm_icm20602_hal_spi.h"
#include <string.h>

static bool has_exactly_one_bit(uint16_t v)
{
    return v != 0u && (uint16_t)(v & (uint16_t)(v - 1u)) == 0u;
}

static uint32_t spi_divisor(uint32_t prescaler)
{
    switch (prescaler) {
    case SPI_BAUDRATEPRESCALER_2: return 2u;
    case SPI_BAUDRATEPRESCALER_4: return 4u;
    case SPI_BAUDRATEPRESCALER_8: return 8u;
    case SPI_BAUDRATEPRESCALER_16: return 16u;
    case SPI_BAUDRATEPRESCALER_32: return 32u;
    case SPI_BAUDRATEPRESCALER_64: return 64u;
    case SPI_BAUDRATEPRESCALER_128: return 128u;
    case SPI_BAUDRATEPRESCALER_256: return 256u;
    default: return 0u;
    }
}

static bool hw_config_valid(const SPI_HandleTypeDef *hspi)
{
    if (hspi == NULL) return false;
    const bool mode_0 = hspi->Init.CLKPolarity == SPI_POLARITY_LOW &&
                        hspi->Init.CLKPhase == SPI_PHASE_1EDGE;
    const bool mode_3 = hspi->Init.CLKPolarity == SPI_POLARITY_HIGH &&
                        hspi->Init.CLKPhase == SPI_PHASE_2EDGE;
    if (!(mode_0 || mode_3) || hspi->Init.Mode != SPI_MODE_MASTER ||
        hspi->Init.Direction != SPI_DIRECTION_2LINES ||
        hspi->Init.DataSize != SPI_DATASIZE_8BIT ||
        hspi->Init.NSS != SPI_NSS_SOFT ||
        hspi->Init.FirstBit != SPI_FIRSTBIT_MSB) return false;
    const uint32_t divisor = spi_divisor(hspi->Init.BaudRatePrescaler);
    if (divisor == 0u) return false;
    uint32_t pclk;
    if (hspi->Instance == SPI1) {
        pclk = HAL_RCC_GetPCLK2Freq();
    } else if (hspi->Instance == SPI2 || hspi->Instance == SPI3) {
        pclk = HAL_RCC_GetPCLK1Freq();
    } else {
        return false;
    }
    if (pclk == 0u) return false;
    const uint32_t bus_hz = pclk / divisor;
    return bus_hz >= LKM_ICM20602_HAL_SPI_MIN_HZ &&
           bus_hz <= LKM_ICM20602_HAL_SPI_MAX_HZ;
}

bool LKM_ICM20602_HAL_Init(lkm_icm20602_hal_spi_t *adapter,
                          const lkm_icm20602_hal_spi_config_t *config)
{
    if (adapter == NULL || config == NULL) return false;
    memset(adapter, 0, sizeof(*adapter));
    if (!config->pcb_pinmap_reviewed || !config->dedicated_spi_bus_reviewed ||
        config->spi == NULL || config->cs_port == NULL ||
        !has_exactly_one_bit(config->cs_pin) || config->timeout_ms == 0u ||
        config->timeout_ms > 1000u || !hw_config_valid(config->spi)) return false;
    adapter->config = *config;
    adapter->initialized = true;
    return true;
}

bool LKM_ICM20602_HAL_MakeBus(lkm_icm20602_hal_spi_t *adapter,
                             lkm_icm20602_bus_t *out_bus)
{
    if (out_bus != NULL) memset(out_bus, 0, sizeof(*out_bus));
    if (adapter == NULL || out_bus == NULL || !adapter->initialized ||
        adapter->fault_latched) return false;
    out_bus->context = adapter;
    out_bus->spi_transfer = LKM_ICM20602_HAL_Transfer;
    out_bus->millis = LKM_ICM20602_HAL_Millis;
    out_bus->delay_ms = LKM_ICM20602_HAL_DelayMs;
    return true;
}

bool LKM_ICM20602_HAL_Transfer(void *context, const uint8_t *tx,
                               uint8_t *rx, size_t length)
{
    lkm_icm20602_hal_spi_t *adapter = (lkm_icm20602_hal_spi_t *)context;
    if (adapter == NULL || !adapter->initialized || adapter->fault_latched)
        return false;
    /* Current driver sends only 2-byte register write or 2..7 byte burst read.
       Reject accidental use with larger data / null buffers. */
    if (tx == NULL || rx == NULL || length < 2u ||
        length > LKM_ICM20602_HAL_MAX_FRAME_BYTES) return false;
    memset(rx, 0, length);
    const lkm_icm20602_hal_spi_config_t *cfg = &adapter->config;
    if (!hw_config_valid(cfg->spi) ||
        HAL_SPI_GetState(cfg->spi) != HAL_SPI_STATE_READY) {
        ++adapter->failed_transactions;
        return false;
    }
    if (HAL_GPIO_ReadPin(cfg->cs_port, cfg->cs_pin) != GPIO_PIN_SET) {
        /* The previously-reviewed CS pin is unexpectedly asserted.
           Release it and latch a fault rather than clocking the sensor. */
        HAL_GPIO_WritePin(cfg->cs_port, cfg->cs_pin, GPIO_PIN_SET);
        ++adapter->failed_transactions;
        adapter->fault_latched = true;
        return false;
    }
    uint8_t tx_copy[LKM_ICM20602_HAL_MAX_FRAME_BYTES] = {0};
    uint8_t rx_copy[LKM_ICM20602_HAL_MAX_FRAME_BYTES] = {0};
    memcpy(tx_copy, tx, length);
    HAL_GPIO_WritePin(cfg->cs_port, cfg->cs_pin, GPIO_PIN_RESET);
    HAL_StatusTypeDef result = HAL_SPI_TransmitReceive(cfg->spi, tx_copy, rx_copy,
                                                          (uint16_t)length,
                                                          cfg->timeout_ms);
    HAL_GPIO_WritePin(cfg->cs_port, cfg->cs_pin, GPIO_PIN_SET);
    if (result != HAL_OK) {
        ++adapter->failed_transactions;
        adapter->fault_latched = true;
        return false;
    }
    memcpy(rx, rx_copy, length);
    ++adapter->successful_transactions;
    return true;
}

uint32_t LKM_ICM20602_HAL_Millis(void *context)
{
    const lkm_icm20602_hal_spi_t *adapter =
        (const lkm_icm20602_hal_spi_t *)context;
    return adapter != NULL && adapter->initialized ? HAL_GetTick() : 0u;
}

void LKM_ICM20602_HAL_DelayMs(void *context, uint32_t millis)
{
    const lkm_icm20602_hal_spi_t *adapter =
        (const lkm_icm20602_hal_spi_t *)context;
    if (adapter != NULL && adapter->initialized) HAL_Delay(millis);
}
