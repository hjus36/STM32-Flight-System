#include "lkm_icm20602_gyro.h"
#include <math.h>
#include <string.h>

#define ICM_REG_SMPLRT_DIV  0x19u
#define ICM_REG_CONFIG      0x1Au
#define ICM_REG_GYRO_CFG    0x1Bu
#define ICM_REG_INT_ENABLE  0x38u
#define ICM_REG_INT_STATUS  0x3Au
#define ICM_REG_PWR_MGMT_1  0x6Bu
#define ICM_REG_PWR_MGMT_2  0x6Cu
#define ICM_REG_I2C_IF      0x70u
#define ICM_BIT_DATA_READY  0x01u
#define ICM_BIT_I2C_DISABLE 0x40u
#define ICM_RAD_PER_DEG (0.01745329251994329577f)

static bool valid_config(const lkm_icm20602_config_t *c)
{
    return c != NULL && c->gyro_fs_sel <= 3u &&
        c->dlpf_cfg >= 1u && c->dlpf_cfg <= 6u &&
        c->calibration_samples >= 32u && c->calibration_samples <= 2048u &&
        isfinite(c->stationary_limit_dps) && c->stationary_limit_dps > 0.0f &&
        isfinite(c->max_stationary_span_dps) && c->max_stationary_span_dps > 0.0f &&
        isfinite(c->max_bias_dps) && c->max_bias_dps > 0.0f &&
        c->max_age_ms > 0u && c->max_age_ms <= LKM_ICM20602_MAX_VALID_AGE;
}

lkm_icm20602_config_t LKM_ICM20602_DefaultConfig(void)
{
    lkm_icm20602_config_t c;
    c.gyro_fs_sel = 2u; /* 32.8 LSB/dps */
    c.dlpf_cfg = 3u;
    c.sample_rate_div = 9u;
    c.calibration_samples = 128u;
    c.stationary_limit_dps = 5.0f;
    c.max_stationary_span_dps = 1.5f;
    c.max_bias_dps = 4.0f;
    c.max_age_ms = 20u;
    return c;
}

lkm_icm20602_result_t LKM_ICM20602_Init(lkm_icm20602_t *imu,
    const lkm_icm20602_bus_t *bus, const lkm_icm20602_config_t *config)
{
    if (imu == NULL || bus == NULL || config == NULL)
        return LKM_ICM_ERR_ARGUMENT;
    memset(imu, 0, sizeof(*imu));
    if (bus->spi_transfer == NULL || bus->millis == NULL ||
        bus->delay_ms == NULL || !valid_config(config)) return LKM_ICM_ERR_CONFIG;
    static const float scale[4] = {131.0f, 65.5f, 32.8f, 16.4f};
    imu->bus = *bus;
    imu->config = *config;
    imu->lsb_per_dps = scale[config->gyro_fs_sel];
    imu->initialized = true;
    return LKM_ICM_OK;
}

static lkm_icm20602_result_t fault(lkm_icm20602_t *imu, lkm_icm20602_result_t code)
{
    imu->latest.valid = false;
    imu->fault_latched = true;
    imu->calibration_active = false;
    imu->calibrated = false;
    return code;
}

static bool spi_register_read(lkm_icm20602_t *imu,
                               uint8_t address, uint8_t *out, size_t count)
{
    if (count == 0u || count > 6u || out == NULL) return false;
    uint8_t tx[7] = {0}, rx[7] = {0};
    tx[0] = (uint8_t)(address | 0x80u);
    if (!imu->bus.spi_transfer(imu->bus.context, tx, rx, count + 1u)) return false;
    memcpy(out, rx + 1u, count);
    return true;
}

static bool spi_register_write(lkm_icm20602_t *imu, uint8_t addr, uint8_t value)
{
    uint8_t tx[2] = {(uint8_t)(addr & 0x7Fu), value}, rx[2] = {0};
    return imu->bus.spi_transfer(imu->bus.context, tx, rx, sizeof(tx));
}

static bool write_checked(lkm_icm20602_t *imu, uint8_t addr, uint8_t value)
{
    uint8_t readback = 0u;
    return spi_register_write(imu, addr, value) &&
        spi_register_read(imu, addr, &readback, 1u) && readback == value;
}

lkm_icm20602_result_t LKM_ICM20602_Configure(lkm_icm20602_t *imu)
{
    if (imu == NULL) return LKM_ICM_ERR_ARGUMENT;
    if (!imu->initialized) return LKM_ICM_ERR_NOT_CONFIGURED;
    if (imu->fault_latched) return LKM_ICM_ERR_FAULT;
    imu->configured = false;
    imu->calibrated = false;
    imu->latest.valid = false;
    /* Datasheet: registers available ~2ms after power-up; application must
     * additionally ensure rail and CS have actually reached stable levels. */
    imu->bus.delay_ms(imu->bus.context, 2u);
    /* Datasheet requires I2C_IF_DIS as early as possible for SPI-only mode. */
    if (!write_checked(imu, ICM_REG_I2C_IF, ICM_BIT_I2C_DISABLE))
        return fault(imu, LKM_ICM_ERR_VERIFY);
    uint8_t id = 0;
    if (!spi_register_read(imu, LKM_ICM20602_WHO_AM_I_REG, &id, 1u))
        return fault(imu, LKM_ICM_ERR_BUS);
    if (id != LKM_ICM20602_WHO_AM_I_VAL)
        return fault(imu, LKM_ICM_ERR_ID);
    if (!write_checked(imu, ICM_REG_PWR_MGMT_1, 0x01u) ||
        !write_checked(imu, ICM_REG_PWR_MGMT_2, 0x00u))
        return fault(imu, LKM_ICM_ERR_VERIFY);
    imu->bus.delay_ms(imu->bus.context, 100u); /* conservative gyro warmup */
    if (!write_checked(imu, ICM_REG_INT_ENABLE, 0x00u) ||
        !write_checked(imu, ICM_REG_CONFIG, imu->config.dlpf_cfg) ||
        !write_checked(imu, ICM_REG_GYRO_CFG, (uint8_t)(imu->config.gyro_fs_sel << 3)) ||
        !write_checked(imu, ICM_REG_SMPLRT_DIV, imu->config.sample_rate_div) ||
        !write_checked(imu, ICM_REG_INT_ENABLE, ICM_BIT_DATA_READY))
        return fault(imu, LKM_ICM_ERR_VERIFY);
    imu->configured = true;
    return LKM_ICM_OK;
}

lkm_icm20602_result_t LKM_ICM20602_StartCalibration(lkm_icm20602_t *imu)
{
    if (imu == NULL) return LKM_ICM_ERR_ARGUMENT;
    if (!imu->configured) return LKM_ICM_ERR_NOT_CONFIGURED;
    if (imu->fault_latched) return LKM_ICM_ERR_FAULT;
    imu->calibrated = false;
    imu->calibration_active = true;
    imu->calibration_seen = 0;
    imu->latest.valid = false;
    memset(imu->bias_sensor_rad_s, 0, sizeof(imu->bias_sensor_rad_s));
    memset(imu->calibration_mean_dps, 0, sizeof(imu->calibration_mean_dps));
    memset(imu->calibration_min_dps, 0, sizeof(imu->calibration_min_dps));
    memset(imu->calibration_max_dps, 0, sizeof(imu->calibration_max_dps));
    return LKM_ICM_OK;
}

static int16_t s16_be(uint8_t hi, uint8_t lo)
{
    uint16_t u = (uint16_t)(((uint16_t)hi << 8) | (uint16_t)lo);
    int32_t s = (u <= 32767u) ? (int32_t)u : ((int32_t)u - 65536);
    return (int16_t)s;
}

lkm_icm20602_result_t LKM_ICM20602_Poll(lkm_icm20602_t *imu,
                                          lkm_icm20602_sample_t *out)
{
    if (out != NULL) memset(out, 0, sizeof(*out));
    if (imu == NULL || out == NULL) return LKM_ICM_ERR_ARGUMENT;
    if (!imu->configured) return LKM_ICM_ERR_NOT_CONFIGURED;
    if (imu->fault_latched) return LKM_ICM_ERR_FAULT;
    uint8_t status = 0;
    if (!spi_register_read(imu, ICM_REG_INT_STATUS, &status, 1u))
        return fault(imu, LKM_ICM_ERR_BUS);
    if (!(status & ICM_BIT_DATA_READY)) {
        ++imu->skipped_not_ready;
        return LKM_ICM_NO_DATA;
    }
    uint8_t raw_bytes[6] = {0};
    if (!spi_register_read(imu, LKM_ICM20602_GYRO_XOUT_H, raw_bytes, sizeof(raw_bytes)))
        return fault(imu, LKM_ICM_ERR_BUS);
    /* Timestamp after the complete SPI read, not an invented sensor latch. */
    const uint32_t received_ms = imu->bus.millis(imu->bus.context);
    int16_t raw[3];
    float rate_dps[3];
    for (unsigned i = 0u; i < 3u; ++i) {
        raw[i] = s16_be(raw_bytes[i * 2u], raw_bytes[i * 2u + 1u]);
        if (raw[i] == INT16_MAX || raw[i] == INT16_MIN)
            return fault(imu, LKM_ICM_ERR_SATURATED);
        rate_dps[i] = (float)raw[i] / imu->lsb_per_dps;
    }
    if (imu->calibration_active) {
        ++imu->calibration_seen;
        const float n = (float)imu->calibration_seen;
        for (unsigned i = 0u; i < 3u; ++i) {
            float value = rate_dps[i];
            if (fabsf(value) > imu->config.stationary_limit_dps)
                return fault(imu, LKM_ICM_ERR_CAL_MOTION);
            if (imu->calibration_seen == 1u) {
                imu->calibration_min_dps[i] = value;
                imu->calibration_max_dps[i] = value;
            } else {
                if (value < imu->calibration_min_dps[i]) imu->calibration_min_dps[i] = value;
                if (value > imu->calibration_max_dps[i]) imu->calibration_max_dps[i] = value;
            }
            if (imu->calibration_max_dps[i] - imu->calibration_min_dps[i] >
                imu->config.max_stationary_span_dps)
                return fault(imu, LKM_ICM_ERR_CAL_MOTION);
            imu->calibration_mean_dps[i] += (value - imu->calibration_mean_dps[i]) / n;
        }
        if (imu->calibration_seen < imu->config.calibration_samples)
            return LKM_ICM_CAL_COLLECTING;
        for (unsigned i = 0u; i < 3u; ++i) {
            if (fabsf(imu->calibration_mean_dps[i]) > imu->config.max_bias_dps)
                return fault(imu, LKM_ICM_ERR_CAL_MOTION);
            imu->bias_sensor_rad_s[i] = imu->calibration_mean_dps[i] * ICM_RAD_PER_DEG;
        }
        imu->calibration_active = false;
        imu->calibrated = true;
        return LKM_ICM_CAL_DONE; /* this sample is not released */
    }
    if (!imu->calibrated) return LKM_ICM_ERR_UNCALIBRATED;
    for (unsigned i = 0; i < 3u; ++i) {
        out->gyro_raw_xyz[i] = raw[i];
        out->gyro_sensor_xyz_rad_s[i] = rate_dps[i] * ICM_RAD_PER_DEG -
                                           imu->bias_sensor_rad_s[i];
    }
    out->received_ms = received_ms;
    out->sample_sequence = ++imu->accepted_samples;
    out->valid = true;
    imu->latest = *out;
    return LKM_ICM_OK;
}

lkm_icm20602_result_t LKM_ICM20602_GetLatest(lkm_icm20602_t *imu,
    lkm_icm20602_sample_t *out)
{
    if (out != NULL) memset(out, 0, sizeof(*out));
    if (imu == NULL || out == NULL) return LKM_ICM_ERR_ARGUMENT;
    if (imu->fault_latched) return LKM_ICM_ERR_FAULT;
    if (!imu->configured) return LKM_ICM_ERR_NOT_CONFIGURED;
    if (!imu->calibrated) return LKM_ICM_ERR_UNCALIBRATED;
    if (!imu->latest.valid) return LKM_ICM_NO_DATA;
    uint32_t age = (uint32_t)(imu->bus.millis(imu->bus.context) - imu->latest.received_ms);
    if (age > imu->config.max_age_ms || age > LKM_ICM20602_MAX_VALID_AGE) {
        imu->latest.valid = false;
        return LKM_ICM_ERR_STALE;
    }
    *out = imu->latest;
    return LKM_ICM_OK;
}

const char *LKM_ICM20602_ResultText(lkm_icm20602_result_t result)
{
    switch (result) {
    case LKM_ICM_OK: return "OK";
    case LKM_ICM_NO_DATA: return "NO_DATA";
    case LKM_ICM_CAL_COLLECTING: return "CAL_COLLECTING";
    case LKM_ICM_CAL_DONE: return "CAL_DONE";
    case LKM_ICM_ERR_ARGUMENT: return "ARGUMENT";
    case LKM_ICM_ERR_CONFIG: return "CONFIG";
    case LKM_ICM_ERR_NOT_CONFIGURED: return "NOT_CONFIGURED";
    case LKM_ICM_ERR_BUS: return "BUS";
    case LKM_ICM_ERR_ID: return "ID";
    case LKM_ICM_ERR_VERIFY: return "VERIFY";
    case LKM_ICM_ERR_UNCALIBRATED: return "UNCALIBRATED";
    case LKM_ICM_ERR_CAL_MOTION: return "CAL_MOTION";
    case LKM_ICM_ERR_SATURATED: return "SATURATED";
    case LKM_ICM_ERR_FAULT: return "FAULT";
    case LKM_ICM_ERR_STALE: return "STALE";
    default: return "UNKNOWN";
    }
}
