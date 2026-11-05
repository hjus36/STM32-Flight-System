#include "lkm_fc_virtual_mixer.h"
#include <string.h>
#if LKM_ACTUATOR_FIXTURE_ACTIVE
#include "LKM_AW_Mixer_F405.h"
#include <math.h>
#endif

bool LKM_FC_VirtualMixer_Evaluate(const lkm_act_virtual_mixer_input_t *in,
    lkm_act_model_command_t *out)
{
    if (out) memset(out,0,sizeof(*out));
    if (!out || !in) return false;
#if LKM_ACTUATOR_FIXTURE_ACTIVE
    /* Validate the RAW mixer input: MATLAB allocator may otherwise convert
       an invalid NaN torque into benign-looking zero motor outputs. */
    if (!in->valid || !isfinite(in->collective_n) ||
        in->collective_n<0.0f || in->collective_n>8.0f ||
        !isfinite(in->roll_torque_nm) ||
        !isfinite(in->pitch_torque_nm) ||
        !isfinite(in->yaw_torque_nm) ||
        fabsf(in->roll_torque_nm)>0.5f ||
        fabsf(in->pitch_torque_nm)>0.5f ||
        fabsf(in->yaw_torque_nm)>0.5f) return false;
    out->produced_ms=in->produced_ms;
    out->sequence=in->sequence;
    out->collective_model_n=in->collective_n;
    out->roll_model_nm=in->roll_torque_nm;
    out->pitch_model_nm=in->pitch_torque_nm;
    out->yaw_model_nm=in->yaw_torque_nm;
    LKM_AW_Mixer_F405(&in->collective_n,&in->roll_torque_nm,
        &in->pitch_torque_nm,&in->yaw_torque_nm,
        &out->motor_model_n[0],&out->motor_model_n[1],
        &out->motor_model_n[2],&out->motor_model_n[3]);
    float sum=0.0f;
    for (unsigned i=0;i<LKM_ACTUATOR_MOTORS;++i) {
        const float x=out->motor_model_n[i];
        if (!isfinite(x) || x<0.0f || x>2.0f) {
            memset(out,0,sizeof(*out));return false;
        }
        sum+=x;
    }
    if (!isfinite(sum) || fabsf(sum-in->collective_n)>0.05f) {
        memset(out,0,sizeof(*out));return false;
    }
    out->valid=true;
    return true;
#else
    /* No generated mixer entrypoint is referenced in production. */
    return false;
#endif
}
