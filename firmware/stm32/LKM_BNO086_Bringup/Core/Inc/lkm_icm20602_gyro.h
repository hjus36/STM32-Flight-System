#ifndef LKM_ICM20602_GYRO_H
#define LKM_ICM20602_GYRO_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* ICM-20602 SPI register driver; intentionally HAL-free.
 * SPI callback: one ATOMIC transaction, CS low for the complete buffer, high
 * at return, mode and frequency checked against the PCB and datasheet by the
 * platform adapter. It must not share BNO086 CS or perform motor output.
 * Each read sends (address|0x80) followed by dummy MOSI bytes.
 * NOT physically frame-aligned. Never equate sensor X/Y with body p/q
 * without an explicit verified mounting transform.
 */
#define LKM_ICM20602_WHO_AM_I_REG  0x75u
#define LKM_ICM20602_WHO_AM_I_VAL  0x12u
#define LKM_ICM20602_GYRO_XOUT_H  0x43u
#define LKM_ICM20602_MAX_VALID_AGE 0x7FFFFFFFu

typedef bool (*lkm_icm20602_transfer_fn)(void *context,
                                      const uint8_t *tx, uint8_t *rx,
                                      size_t length);
typedef uint32_t (*lkm_icm20602_millis_fn)(void *context);
typedef void (*lkm_icm20602_delay_fn)(void *context, uint32_t millis);

typedef struct {
    lkm_icm20602_transfer_fn spi_transfer;
    lkm_icm20602_millis_fn millis;
    lkm_icm20602_delay_fn delay_ms;
    void *context;
} lkm_icm20602_bus_t;

typedef struct {
    uint8_t gyro_fs_sel;     /* 0:250, 1:500, 2:1000, 3:2000 dps */
    uint8_t dlpf_cfg;       /* 1..6; FCHOICE_B remains 0 */
    uint8_t sample_rate_div; /* 1kHz / (1 + divider); 9 => 100Hz */
    uint16_t calibration_samples; /* 32..2048 stationary, fresh samples */
    float stationary_limit_dps; /* per-sample abs rate maximum */
    float max_stationary_span_dps; /* max - min across calibration window */
    float max_bias_dps;     /* max abs window average */
    uint32_t max_age_ms;    /* sample age cutoff; default 20ms */
} lkm_icm20602_config_t;

typedef enum {
    LKM_ICM_OK = 0,
    LKM_ICM_NO_DATA,
    LKM_ICM_CAL_COLLECTING,
    LKM_ICM_CAL_DONE,
    LKM_ICM_ERR_ARGUMENT,
    LKM_ICM_ERR_CONFIG,
    LKM_ICM_ERR_NOT_CONFIGURED,
    LKM_ICM_ERR_BUS,
    LKM_ICM_ERR_ID,
    LKM_ICM_ERR_VERIFY,
    LKM_ICM_ERR_UNCALIBRATED,
    LKM_ICM_ERR_CAL_MOTION,
    LKM_ICM_ERR_SATURATED,
    LKM_ICM_ERR_FAULT,
    LKM_ICM_ERR_STALE
} lkm_icm20602_result_t;

typedef struct {
    bool valid; /* only true after confirmed stationary calibration */
    int16_t gyro_raw_xyz[3];
    float gyro_sensor_xyz_rad_s[3]; /* BIAS-CORRECTED sensor-axis rate */
    uint32_t received_ms; /* end of host SPI transaction, NOT IMU latch time */
    uint32_t sample_sequence; /* host increment, not hardware sequence */
} lkm_icm20602_sample_t;

typedef struct {
    bool initialized;
    bool configured;
    bool fault_latched;
    bool calibration_active;
    bool calibrated;
    lkm_icm20602_bus_t bus;
    lkm_icm20602_config_t config;
    float lsb_per_dps;
    float bias_sensor_rad_s[3];
    uint16_t calibration_seen;
    float calibration_mean_dps[3];
    float calibration_min_dps[3];
    float calibration_max_dps[3];
    lkm_icm20602_sample_t latest;
    uint32_t accepted_samples;
    uint32_t skipped_not_ready;
} lkm_icm20602_t;

/* Defaults: +-1000 dps, DLPF_CFG=3 (~41Hz), divider=9, 100Hz nominal,
 * 128 stationary calibration samples, 20ms age. Test configuration ONLY. */
lkm_icm20602_config_t LKM_ICM20602_DefaultConfig(void);
lkm_icm20602_result_t LKM_ICM20602_Init(lkm_icm20602_t *imu,
    const lkm_icm20602_bus_t *bus, const lkm_icm20602_config_t *config);
/* No MCU/IO called until Configure(); explicit 2ms delay before register
 * access and 100ms gyro startup delay are used. No chip reset or IRQ pin. */
lkm_icm20602_result_t LKM_ICM20602_Configure(lkm_icm20602_t *imu);
lkm_icm20602_result_t LKM_ICM20602_StartCalibration(lkm_icm20602_t *imu);
/* Poll INT_STATUS(0x3A) data-ready, then burst read six gyro bytes.
 * No new sample => NO_DATA and ZERO invalid out; never reuse old sample.
 * Calibration needs NEW sensor samples and never produces usable feedback. */
lkm_icm20602_result_t LKM_ICM20602_Poll(lkm_icm20602_t *imu,
                                          lkm_icm20602_sample_t *out);
/* Aged data cannot be reused. No downstream gyro_ok until verified frame. */
lkm_icm20602_result_t LKM_ICM20602_GetLatest(lkm_icm20602_t *imu,
    lkm_icm20602_sample_t *out);
const char *LKM_ICM20602_ResultText(lkm_icm20602_result_t result);

#endif /* LKM_ICM20602_GYRO_H */
