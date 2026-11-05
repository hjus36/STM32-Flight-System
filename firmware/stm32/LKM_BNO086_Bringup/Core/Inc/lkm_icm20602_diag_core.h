#ifndef LKM_ICM20602_DIAG_CORE_H
#define LKM_ICM20602_DIAG_CORE_H

#include <stdbool.h>
#include <stdint.h>
#include "lkm_icm20602_gyro.h"

/* ICM-20602 diagnostic state manager. HAL-free; not a flight control input.
 * No SPI/GPIO operations in Init(). Start() explicitly configures sensor.
 * Rates are measured SENSOR X/Y/Z, NOT verified body p/q/r.
 */
typedef enum {
    LKM_ICM_DIAG_OFF = 0,
    LKM_ICM_DIAG_NEED_START,
    LKM_ICM_DIAG_CALIBRATING,
    LKM_ICM_DIAG_OBSERVING,
    LKM_ICM_DIAG_FAULT
} lkm_icm_diag_state_t;

typedef struct {
    lkm_icm20602_t gyro;
    lkm_icm_diag_state_t state;
    lkm_icm20602_result_t last_result;
    uint32_t min_poll_interval_ms;
    uint32_t last_poll_ms;
    bool has_polled;
    uint32_t service_calls;
    uint32_t polls_attempted;
    uint32_t valid_new_samples;
    uint32_t no_data_polls;
} lkm_icm_diag_core_t;

/* Initializes without touching hardware; zero poll interval disallowed. */
bool LKM_ICM_DiagCore_Init(lkm_icm_diag_core_t *diag,
                          const lkm_icm20602_bus_t *bus,
                          const lkm_icm20602_config_t *config,
                          uint32_t min_poll_interval_ms);
/* Explicitly starts WHO_AM_I/configure and stationary calibration.
 * Configure uses blocking sensor startup delays; call only in a diagnostic
 * context, never a 100Hz PID step or ISR. */
bool LKM_ICM_DiagCore_Start(lkm_icm_diag_core_t *diag);
/* One bounded gyro poll at most, subject to interval. No data is not a fault. */
void LKM_ICM_DiagCore_Service(lkm_icm_diag_core_t *diag);
/* Returns a fresh calibrated SENSOR-AXIS sample, never fabricates p/q. */
bool LKM_ICM_DiagCore_TryGet(lkm_icm_diag_core_t *diag,
                             lkm_icm20602_sample_t *out);
const char *LKM_ICM_DiagCore_StateText(lkm_icm_diag_state_t state);

#endif
