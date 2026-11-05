#ifndef LKM_FC_VIRTUAL_MIXER_H
#define LKM_FC_VIRTUAL_MIXER_H
/* Exact MATLAB-generated Quad-X Mixer may be invoked only by a HOST fixture.
 * Input and output are MODEL N/Nm (not ESC protocol commands). Production
 * STM32 cannot invoke the generated mixer via this wrapper. */
#include "lkm_fc_actuator_guard.h"
typedef struct {
    bool valid;
    uint32_t produced_ms;
    uint32_t sequence;
    float collective_n;
    float roll_torque_nm;
    float pitch_torque_nm;
    float yaw_torque_nm;
} lkm_act_virtual_mixer_input_t;

/* Always clears output on failure; requires HOST_FIXTURE=1. */
bool LKM_FC_VirtualMixer_Evaluate(const lkm_act_virtual_mixer_input_t *in,
    lkm_act_model_command_t *out);
#endif
