#include "lkm_icm20602_diag_core.h"
#include <string.h>

static void clear_latest(lkm_icm_diag_core_t *diag)
{
    diag->gyro.latest.valid = false;
}

bool LKM_ICM_DiagCore_Init(lkm_icm_diag_core_t *diag,
                          const lkm_icm20602_bus_t *bus,
                          const lkm_icm20602_config_t *config,
                          uint32_t min_poll_interval_ms)
{
    if (diag == NULL) return false;
    memset(diag, 0, sizeof(*diag));
    diag->state = LKM_ICM_DIAG_OFF;
    diag->last_result = LKM_ICM_ERR_NOT_CONFIGURED;
    if (bus == NULL || config == NULL || min_poll_interval_ms == 0u ||
        min_poll_interval_ms > 1000u) return false;
    diag->last_result = LKM_ICM20602_Init(&diag->gyro, bus, config);
    if (diag->last_result != LKM_ICM_OK) return false;
    diag->min_poll_interval_ms = min_poll_interval_ms;
    diag->state = LKM_ICM_DIAG_NEED_START;
    return true;
}

bool LKM_ICM_DiagCore_Start(lkm_icm_diag_core_t *diag)
{
    if (diag == NULL || diag->state != LKM_ICM_DIAG_NEED_START) return false;
    diag->last_result = LKM_ICM20602_Configure(&diag->gyro);
    if (diag->last_result != LKM_ICM_OK) {
        diag->state = LKM_ICM_DIAG_FAULT;
        clear_latest(diag);
        return false;
    }
    diag->last_result = LKM_ICM20602_StartCalibration(&diag->gyro);
    if (diag->last_result != LKM_ICM_OK) {
        diag->state = LKM_ICM_DIAG_FAULT;
        clear_latest(diag);
        return false;
    }
    diag->state = LKM_ICM_DIAG_CALIBRATING;
    diag->has_polled = false;
    return true;
}

void LKM_ICM_DiagCore_Service(lkm_icm_diag_core_t *diag)
{
    if (diag == NULL) return;
    ++diag->service_calls;
    if (diag->state != LKM_ICM_DIAG_CALIBRATING &&
        diag->state != LKM_ICM_DIAG_OBSERVING) return;
    const uint32_t now = diag->gyro.bus.millis(diag->gyro.bus.context);
    if (diag->has_polled) {
        const uint32_t age = (uint32_t)(now - diag->last_poll_ms);
        /* Protect against reversed host time (except ordinary wrap). */
        if (age >= 0x80000000u) {
            diag->last_result = LKM_ICM_ERR_STALE;
            diag->state = LKM_ICM_DIAG_FAULT;
            clear_latest(diag);
            return;
        }
        if (age < diag->min_poll_interval_ms) return;
    }
    diag->has_polled = true;
    diag->last_poll_ms = now;
    ++diag->polls_attempted;
    lkm_icm20602_sample_t fresh = {0};
    diag->last_result = LKM_ICM20602_Poll(&diag->gyro, &fresh);
    switch (diag->last_result) {
    case LKM_ICM_NO_DATA:
        ++diag->no_data_polls;
        break;
    case LKM_ICM_CAL_COLLECTING:
        break;
    case LKM_ICM_CAL_DONE:
        diag->state = LKM_ICM_DIAG_OBSERVING;
        break;
    case LKM_ICM_OK:
        if (diag->state == LKM_ICM_DIAG_OBSERVING && fresh.valid)
            ++diag->valid_new_samples;
        break;
    default:
        diag->state = LKM_ICM_DIAG_FAULT;
        clear_latest(diag);
        break;
    }
}

bool LKM_ICM_DiagCore_TryGet(lkm_icm_diag_core_t *diag,
                             lkm_icm20602_sample_t *out)
{
    if (out != NULL) memset(out, 0, sizeof(*out));
    if (diag == NULL || out == NULL ||
        diag->state != LKM_ICM_DIAG_OBSERVING) return false;
    return LKM_ICM20602_GetLatest(&diag->gyro, out) == LKM_ICM_OK && out->valid;
}

const char *LKM_ICM_DiagCore_StateText(lkm_icm_diag_state_t state)
{
    switch (state) {
    case LKM_ICM_DIAG_OFF: return "OFF";
    case LKM_ICM_DIAG_NEED_START: return "NEED_START";
    case LKM_ICM_DIAG_CALIBRATING: return "CALIBRATING";
    case LKM_ICM_DIAG_OBSERVING: return "OBSERVING";
    case LKM_ICM_DIAG_FAULT: return "FAULT";
    default: return "UNKNOWN";
    }
}
