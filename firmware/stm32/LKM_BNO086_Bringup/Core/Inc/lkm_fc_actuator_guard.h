#ifndef LKM_FC_ACTUATOR_GUARD_H
#define LKM_FC_ACTUATOR_GUARD_H
/*
 * LKM_FC_V1 Step 5 - HAL-independent, VIRTUAL thrust safety contract.
 * Simulink Motor_N_Thrust is a MODEL VALUE IN NEWTONS, NOT ESC PWM/DShot.
 * No motor order, spin direction, ESC protocol, calibration, receiver,
 * voltage ADC or real-world arming is supplied or approved by this module.
 * The production MCU compile target cannot enable the HOST fixture.
 */
#include <stdbool.h>
#include <stdint.h>

#if defined(LKM_FC_ACTUATOR_HOST_FIXTURE) && LKM_FC_ACTUATOR_HOST_FIXTURE
# if defined(STM32F405xx) || defined(USE_HAL_DRIVER) || defined(__arm__) || defined(__thumb__) || defined(__ARM_ARCH)
#  error "Actuator HOST_FIXTURE is forbidden on MCU / STM32 production builds"
# endif
# define LKM_ACTUATOR_FIXTURE_ACTIVE 1
#else
# define LKM_ACTUATOR_FIXTURE_ACTIVE 0
#endif

#define LKM_ACTUATOR_MOTORS 4u
#define LKM_ACTUATOR_MODEL_MAX_PER_MOTOR_N 2.0f
#define LKM_ACTUATOR_MODEL_MAX_COLLECTIVE_N 8.0f

typedef enum {
    LKM_ACT_OK=0,
    LKM_ACT_ERR_ARGUMENT,
    LKM_ACT_ERR_CONFIG,
    LKM_ACT_ERR_UNINITIALIZED,
    LKM_ACT_ERR_PRODUCTION_LOCK,
    LKM_ACT_DISARMED,
    LKM_ACT_ARM_EDGE_REQUIRED,
    LKM_ACT_ARMED_VIRTUAL,
    LKM_ACT_MANUAL_DISARM,
    LKM_ACT_FAILSAFE_LATCHED,
    LKM_ACT_ERR_TIME_BACKWARD,
    LKM_ACT_ERR_ESTOP,
    LKM_ACT_ERR_RECEIVER,
    LKM_ACT_ERR_THROTTLE,
    LKM_ACT_ERR_BATTERY,
    LKM_ACT_ERR_SENSOR_OR_AXES,
    LKM_ACT_ERR_SCHEDULER,
    LKM_ACT_ERR_CONTROLLER,
    LKM_ACT_ERR_HEARTBEAT,
    LKM_ACT_ERR_COMMAND,
    LKM_ACT_ERR_COMMAND_SEQUENCE,
    LKM_ACT_ERR_MIXER_RANGE
} lkm_act_result_t;

typedef enum {
    LKM_ACT_STATE_LOCKED=0,
    LKM_ACT_STATE_DISARMED,
    LKM_ACT_STATE_ARMED_VIRTUAL,
    LKM_ACT_STATE_FAILSAFE_LATCHED
} lkm_act_state_t;

typedef struct {
    /* TEST envelope, not flight-certified motor bounds. Limits are model N. */
    uint32_t max_receiver_age_ms;
    uint32_t max_imu_age_ms;
    uint32_t max_battery_age_ms;
    uint32_t max_controller_age_ms;
    uint32_t max_scheduler_age_ms;
    uint32_t max_command_age_ms;
    float max_motor_model_n;          /* <=2.0 model N */
    float max_collective_model_n;     /* <=8.0 model N */
    float max_abs_torque_model_nm;    /* fixture test envelope */
} lkm_act_config_t;

typedef struct {
    /* MOCK predicates, not actual receiver/ADC or hardware evidence. */
    bool receiver_link_ok;
    bool receiver_throttle_low;
    bool receiver_arm_switch_high;
    bool battery_ok;
    bool sensors_ok;
    bool axes_physically_verified; /* HOST FIXTURE ONLY */
    bool gyro_calibrated;
    bool scheduler_ok;
    bool controller_data_valid;
    bool emergency_stop;
    uint32_t receiver_rx_ms;
    uint32_t imu_rx_ms;
    uint32_t battery_rx_ms;
    uint32_t controller_rx_ms;
    uint32_t scheduler_rx_ms;
} lkm_act_health_t;

typedef struct {
    bool valid;                     /* synthetic generated mixer result */
    uint32_t produced_ms;
    uint32_t sequence;
    float collective_model_n;
    float roll_model_nm;
    float pitch_model_nm;
    float yaw_model_nm;
    float motor_model_n[LKM_ACTUATOR_MOTORS];
} lkm_act_model_command_t;

typedef struct {
    bool virtual_valid;             /* HOST FIXTURE ONLY; NEVER an ESC grant */
    float virtual_model_n[LKM_ACTUATOR_MOTORS];
    bool esc_output_enabled;        /* ALWAYS false */
    bool controller_ready;          /* ALWAYS false */
} lkm_act_virtual_output_t;

typedef struct {
    bool initialized;
    bool fixture_enabled;
    bool saw_arm_switch_low;
    bool previous_switch_high;
    bool have_last_tick;
    bool have_last_command;
    lkm_act_state_t state;
    lkm_act_result_t last_result;
    lkm_act_config_t config;
    uint32_t last_tick_ms;
    uint32_t last_command_sequence;
    uint32_t virtual_frames;
    uint32_t failsafe_events;
    uint32_t rejects;
    uint32_t manual_disarms;
    bool controller_ready;         /* ALWAYS false */
    bool esc_output_enabled;       /* ALWAYS false */
} lkm_act_guard_t;

lkm_act_config_t LKM_FC_Actuator_DefaultConfig(void);
lkm_act_result_t LKM_FC_Actuator_Guard_Init(lkm_act_guard_t *g,
    const lkm_act_config_t *cfg, bool request_host_fixture);
lkm_act_result_t LKM_FC_Actuator_Guard_Tick(lkm_act_guard_t *g,
    const lkm_act_health_t *health, const lkm_act_model_command_t *model,
    uint32_t now_ms, lkm_act_virtual_output_t *out);
/* Explicit host-only failsafe reset; requires healthy RX and arm switch LOW.
 * Does not arm. A NEW low-to-high switch transition is always required. */
lkm_act_result_t LKM_FC_Actuator_Guard_Reset(lkm_act_guard_t *g,
    const lkm_act_health_t *health, uint32_t now_ms);
const char *LKM_FC_Actuator_ResultText(lkm_act_result_t result);

#endif
