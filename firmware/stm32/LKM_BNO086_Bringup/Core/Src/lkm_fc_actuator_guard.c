#include "lkm_fc_actuator_guard.h"
#include <math.h>
#include <string.h>
#include <limits.h>

#if LKM_ACTUATOR_FIXTURE_ACTIVE
static void count_inc(uint32_t *v) { if (*v != UINT32_MAX) ++*v; }
static bool age_ok(uint32_t now, uint32_t stamp, uint32_t limit)
{
    const uint32_t age=now-stamp;
    return age<=limit && age<UINT32_C(0x80000000);
}
#endif
static bool config_ok(const lkm_act_config_t *c)
{
    return c && c->max_receiver_age_ms>=1u && c->max_receiver_age_ms<=500u &&
        c->max_imu_age_ms>=1u && c->max_imu_age_ms<=100u &&
        c->max_battery_age_ms>=1u && c->max_battery_age_ms<=2000u &&
        c->max_controller_age_ms>=1u && c->max_controller_age_ms<=100u &&
        c->max_scheduler_age_ms>=1u && c->max_scheduler_age_ms<=100u &&
        c->max_command_age_ms>=1u && c->max_command_age_ms<=50u &&
        isfinite(c->max_motor_model_n) && c->max_motor_model_n>0.0f &&
        c->max_motor_model_n<=LKM_ACTUATOR_MODEL_MAX_PER_MOTOR_N &&
        isfinite(c->max_collective_model_n) && c->max_collective_model_n>0.0f &&
        c->max_collective_model_n<=LKM_ACTUATOR_MODEL_MAX_COLLECTIVE_N &&
        isfinite(c->max_abs_torque_model_nm) && c->max_abs_torque_model_nm>0.0f &&
        c->max_abs_torque_model_nm<=1.0f;
}
lkm_act_config_t LKM_FC_Actuator_DefaultConfig(void)
{
    lkm_act_config_t c={0};
    c.max_receiver_age_ms=100u;
    c.max_imu_age_ms=50u;
    c.max_battery_age_ms=500u;
    c.max_controller_age_ms=20u;
    c.max_scheduler_age_ms=20u;
    c.max_command_age_ms=20u;
    c.max_motor_model_n=2.0f;
    c.max_collective_model_n=8.0f;
    c.max_abs_torque_model_nm=0.5f;
    return c;
}
lkm_act_result_t LKM_FC_Actuator_Guard_Init(lkm_act_guard_t *g,
    const lkm_act_config_t *cfg,bool request_host_fixture)
{
    if (!g) return LKM_ACT_ERR_ARGUMENT;
    memset(g,0,sizeof(*g));
    g->state=LKM_ACT_STATE_LOCKED;
    g->last_result=LKM_ACT_ERR_PRODUCTION_LOCK;
    if (!config_ok(cfg)) { g->last_result=LKM_ACT_ERR_CONFIG;return g->last_result; }
    g->config=*cfg;
    g->initialized=true;
#if LKM_ACTUATOR_FIXTURE_ACTIVE
    if (request_host_fixture) {
        g->fixture_enabled=true;
        g->state=LKM_ACT_STATE_DISARMED;
        g->last_result=LKM_ACT_DISARMED;
        return LKM_ACT_OK;
    }
#else
    (void)request_host_fixture;
#endif
    return LKM_ACT_ERR_PRODUCTION_LOCK;
}

#if LKM_ACTUATOR_FIXTURE_ACTIVE
static lkm_act_result_t health_result(const lkm_act_guard_t *g,
    const lkm_act_health_t *h,uint32_t now)
{
    if (!h) return LKM_ACT_ERR_ARGUMENT;
    if (h->emergency_stop) return LKM_ACT_ERR_ESTOP;
    if (!h->receiver_link_ok) return LKM_ACT_ERR_RECEIVER;
    if (!h->receiver_throttle_low) return LKM_ACT_ERR_THROTTLE;
    if (!h->battery_ok) return LKM_ACT_ERR_BATTERY;
    if (!h->sensors_ok || !h->axes_physically_verified || !h->gyro_calibrated)
        return LKM_ACT_ERR_SENSOR_OR_AXES;
    if (!h->scheduler_ok) return LKM_ACT_ERR_SCHEDULER;
    if (!h->controller_data_valid) return LKM_ACT_ERR_CONTROLLER;
    if (!age_ok(now,h->receiver_rx_ms,g->config.max_receiver_age_ms) ||
        !age_ok(now,h->imu_rx_ms,g->config.max_imu_age_ms) ||
        !age_ok(now,h->battery_rx_ms,g->config.max_battery_age_ms) ||
        !age_ok(now,h->controller_rx_ms,g->config.max_controller_age_ms) ||
        !age_ok(now,h->scheduler_rx_ms,g->config.max_scheduler_age_ms))
        return LKM_ACT_ERR_HEARTBEAT;
    return LKM_ACT_OK;
}

static lkm_act_result_t model_result(const lkm_act_guard_t *g,
    const lkm_act_model_command_t *m,uint32_t now)
{
    if (!m || !m->valid) return LKM_ACT_ERR_COMMAND;
    if (!age_ok(now,m->produced_ms,g->config.max_command_age_ms))
        return LKM_ACT_ERR_HEARTBEAT;
    if (g->have_last_command && (uint32_t)(m->sequence-g->last_command_sequence)!=1u)
        return LKM_ACT_ERR_COMMAND_SEQUENCE;
    if (!isfinite(m->collective_model_n) || m->collective_model_n<0.0f ||
        m->collective_model_n>g->config.max_collective_model_n ||
        !isfinite(m->roll_model_nm) || !isfinite(m->pitch_model_nm) ||
        !isfinite(m->yaw_model_nm) ||
        fabsf(m->roll_model_nm)>g->config.max_abs_torque_model_nm ||
        fabsf(m->pitch_model_nm)>g->config.max_abs_torque_model_nm ||
        fabsf(m->yaw_model_nm)>g->config.max_abs_torque_model_nm)
        return LKM_ACT_ERR_COMMAND;
    float sum=0.0f;
    for (unsigned i=0;i<LKM_ACTUATOR_MOTORS;++i) {
        const float x=m->motor_model_n[i];
        if (!isfinite(x) || x<0.0f || x>g->config.max_motor_model_n)
            return LKM_ACT_ERR_MIXER_RANGE;
        sum+=x;
    }
    /* Bounded Quad-X allocator is designed to conserve clamped collective.
       This consistency check cannot replace motor order/thrust calibration. */
    if (!isfinite(sum) || fabsf(sum-m->collective_model_n)>0.05f)
        return LKM_ACT_ERR_MIXER_RANGE;
    return LKM_ACT_OK;
}

static lkm_act_result_t failsafe(lkm_act_guard_t *g,lkm_act_result_t cause)
{
    if (g->state!=LKM_ACT_STATE_FAILSAFE_LATCHED)
        count_inc(&g->failsafe_events);
    g->state=LKM_ACT_STATE_FAILSAFE_LATCHED;
    g->last_result=cause;
    g->saw_arm_switch_low=false;
    count_inc(&g->rejects);
    return cause;
}

#endif
lkm_act_result_t LKM_FC_Actuator_Guard_Tick(lkm_act_guard_t *g,
    const lkm_act_health_t *health,const lkm_act_model_command_t *model,
    uint32_t now_ms,lkm_act_virtual_output_t *out)
{
    if (out) memset(out,0,sizeof(*out));
    if (!g || !out) return LKM_ACT_ERR_ARGUMENT;
    if (!g->initialized) return LKM_ACT_ERR_UNINITIALIZED;
#if !LKM_ACTUATOR_FIXTURE_ACTIVE
    /* HARD COMPILE-TIME LOCK: no arming/virtual thrust code in MCU object. */
    (void)health; (void)model; (void)now_ms;
    return g->last_result=LKM_ACT_ERR_PRODUCTION_LOCK;
#else
    if (!g->fixture_enabled || g->state==LKM_ACT_STATE_LOCKED)
        return g->last_result=LKM_ACT_ERR_PRODUCTION_LOCK;
    if (g->state==LKM_ACT_STATE_FAILSAFE_LATCHED)
        return LKM_ACT_FAILSAFE_LATCHED;
    if (g->have_last_tick && (uint32_t)(now_ms-g->last_tick_ms)>=UINT32_C(0x80000000))
        return failsafe(g,LKM_ACT_ERR_TIME_BACKWARD);
    g->have_last_tick=true;
    g->last_tick_ms=now_ms;
    const lkm_act_result_t h=health_result(g,health,now_ms);
    if (h!=LKM_ACT_OK) {
        if (g->state==LKM_ACT_STATE_ARMED_VIRTUAL) return failsafe(g,h);
        g->saw_arm_switch_low=false;
        g->previous_switch_high=health?health->receiver_arm_switch_high:false;
        return g->last_result=h;
    }
    if (g->state==LKM_ACT_STATE_DISARMED) {
        if (!health->receiver_arm_switch_high) {
            g->saw_arm_switch_low=true;
            g->previous_switch_high=false;
            return g->last_result=LKM_ACT_DISARMED;
        }
        if (!g->saw_arm_switch_low || g->previous_switch_high) {
            g->previous_switch_high=true;
            return g->last_result=LKM_ACT_ARM_EDGE_REQUIRED;
        }
        g->previous_switch_high=true;
        g->state=LKM_ACT_STATE_ARMED_VIRTUAL;
        g->have_last_command=false;
        /* Arming edge never emits a virtual thrust frame. */
        return g->last_result=LKM_ACT_ARMED_VIRTUAL;
    }
    if (!health->receiver_arm_switch_high) {
        g->state=LKM_ACT_STATE_DISARMED;
        g->saw_arm_switch_low=true;
        g->previous_switch_high=false;
        g->have_last_command=false;
        count_inc(&g->manual_disarms);
        return g->last_result=LKM_ACT_MANUAL_DISARM;
    }
    const lkm_act_result_t m=model_result(g,model,now_ms);
    if (m!=LKM_ACT_OK) return failsafe(g,m);
    for (unsigned i=0;i<LKM_ACTUATOR_MOTORS;++i)
        out->virtual_model_n[i]=model->motor_model_n[i];
    out->virtual_valid=true;
    /* There is deliberately no API converting N to pulses or DShot values. */
    out->controller_ready=false;
    out->esc_output_enabled=false;
    g->have_last_command=true;
    g->last_command_sequence=model->sequence;
    count_inc(&g->virtual_frames);
    return g->last_result=LKM_ACT_OK;
#endif
}

lkm_act_result_t LKM_FC_Actuator_Guard_Reset(lkm_act_guard_t *g,
    const lkm_act_health_t *health,uint32_t now_ms)
{
    if (!g) return LKM_ACT_ERR_ARGUMENT;
    if (!g->initialized) return LKM_ACT_ERR_UNINITIALIZED;
#if !LKM_ACTUATOR_FIXTURE_ACTIVE
    (void)health; (void)now_ms;
    return LKM_ACT_ERR_PRODUCTION_LOCK;
#else
    if (!g->fixture_enabled) return LKM_ACT_ERR_PRODUCTION_LOCK;
    if (g->state!=LKM_ACT_STATE_FAILSAFE_LATCHED)
        return LKM_ACT_ERR_ARGUMENT;
    if (!health || health->receiver_arm_switch_high)
        return LKM_ACT_ARM_EDGE_REQUIRED;
    if (g->have_last_tick &&
        (uint32_t)(now_ms-g->last_tick_ms)>=UINT32_C(0x80000000))
        return LKM_ACT_ERR_TIME_BACKWARD;
    const lkm_act_result_t h=health_result(g,health,now_ms);
    if (h!=LKM_ACT_OK) return h;
    g->state=LKM_ACT_STATE_DISARMED;
    g->last_result=LKM_ACT_DISARMED;
    g->have_last_tick=true;
    g->last_tick_ms=now_ms;
    g->have_last_command=false;
    g->saw_arm_switch_low=false; /* a fresh low tick is required after reset */
    g->previous_switch_high=false;
    return LKM_ACT_OK;
#endif
}

const char *LKM_FC_Actuator_ResultText(lkm_act_result_t r)
{
    switch(r){
    case LKM_ACT_OK:return "OK";
    case LKM_ACT_ERR_ARGUMENT:return "ARGUMENT";
    case LKM_ACT_ERR_CONFIG:return "CONFIG";
    case LKM_ACT_ERR_UNINITIALIZED:return "UNINITIALIZED";
    case LKM_ACT_ERR_PRODUCTION_LOCK:return "PRODUCTION_LOCK";
    case LKM_ACT_DISARMED:return "DISARMED";
    case LKM_ACT_ARM_EDGE_REQUIRED:return "ARM_EDGE_REQUIRED";
    case LKM_ACT_ARMED_VIRTUAL:return "ARMED_VIRTUAL";
    case LKM_ACT_MANUAL_DISARM:return "MANUAL_DISARM";
    case LKM_ACT_FAILSAFE_LATCHED:return "FAILSAFE_LATCHED";
    case LKM_ACT_ERR_TIME_BACKWARD:return "TIME_BACKWARD";
    case LKM_ACT_ERR_ESTOP:return "ESTOP";
    case LKM_ACT_ERR_RECEIVER:return "RECEIVER";
    case LKM_ACT_ERR_THROTTLE:return "THROTTLE";
    case LKM_ACT_ERR_BATTERY:return "BATTERY";
    case LKM_ACT_ERR_SENSOR_OR_AXES:return "SENSOR_OR_AXES";
    case LKM_ACT_ERR_SCHEDULER:return "SCHEDULER";
    case LKM_ACT_ERR_CONTROLLER:return "CONTROLLER";
    case LKM_ACT_ERR_HEARTBEAT:return "HEARTBEAT";
    case LKM_ACT_ERR_COMMAND:return "COMMAND";
    case LKM_ACT_ERR_COMMAND_SEQUENCE:return "COMMAND_SEQUENCE";
    case LKM_ACT_ERR_MIXER_RANGE:return "MIXER_RANGE";
    default:return "UNKNOWN";
    }
}
