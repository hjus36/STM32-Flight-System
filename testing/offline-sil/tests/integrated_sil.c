/* LKM_FC_V1 integrated SIL host fixture. 100% synthetic inputs.
 * Uses the user's existing generated Simulink PID+Mixer and the actual
 * BNO086 parser, timepair, frame+gate, scheduler+buffer, actuator guard.
 * These results are NOT evidence of real sensor frame approval, motor
 * calibration, ESC drive, MCU execution, or physical flight stability.
 * Compile ONLY as a host test, NEVER in STM32 firmware. */
#if !defined(LKM_SIL_HOST_ONLY) || !LKM_SIL_HOST_ONLY
#error "Integrated SIL fixture is host-only and must never enter production firmware"
#endif
#if defined(STM32F405xx) || defined(USE_HAL_DRIVER) || defined(__arm__) || defined(__thumb__) || defined(__ARM_ARCH)
#error "Integrated SIL fixture is forbidden for STM32/ARM targets"
#endif
#if !defined(LKM_RP_FRAME_HOST_FIXTURE) || !LKM_RP_FRAME_HOST_FIXTURE
#error "Frame host fixture required"
#endif
#if !defined(LKM_FC_ACTUATOR_HOST_FIXTURE) || !LKM_FC_ACTUATOR_HOST_FIXTURE
#error "Actuator host fixture required"
#endif
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <limits.h>
#include "lkm_bno086_attitude.h"
#include "lkm_imu_time_pair.h"
#include "lkm_rp_frame_contract.h"
#include "lkm_fc100hz_core.h"
#include "lkm_aw_core_preview.h"
#include "lkm_fc_virtual_mixer.h"
#include "lkm_fc_actuator_guard.h"

#define RAD_DEG 57.29577951308232f
#define DEG_RAD 0.01745329251994329577f
#define DT 0.01f
#define MAX_SIL_ABS_ANGLE 0.65f
#define MAX_SIL_ABS_RATE 8.0f
#define INJECTION_I 120u
#define RECOVERY_I 220u

/* These two second-order plant parameters are invented synthetic test inputs
 * (NOT QAV210 mass/inertia/aero coefficients), used only for graphing. */
#define SYNTH_J_ROLL_KGM2 0.025f
#define SYNTH_J_PITCH_KGM2 0.025f
#define SYNTH_D_ROLL_NMS 0.11f
#define SYNTH_D_PITCH_NMS 0.11f

typedef enum {
    CASE_BASELINE, CASE_RECEIVER_LOSS, CASE_RECEIVER_RECOVERY,
    CASE_BATTERY_LOW, CASE_IMU_STALE, CASE_TIME_SKEW,
    CASE_DUPLICATE, CASE_SCHED_OVERRUN, CASE_AXIS_UNVERIFIED,
    CASE_BAD_QUAT_QUALITY, CASE_NAN_TARGET, CASE_NAN_MIXER,
    CASE_ESTOP, CASE_RC_HEARTBEAT, CASE_BATTERY_HEARTBEAT,
    CASE_COUNT
} case_kind_t;

static const char *const case_names[CASE_COUNT]={
    "baseline", "receiver_loss", "receiver_recovery", "battery_low",
    "imu_stale", "timestamp_skew", "duplicate_sample", "scheduler_overrun",
    "axis_unverified", "low_bno_accuracy", "nan_target", "nan_mixer",
    "emergency_stop", "receiver_heartbeat_stale", "battery_heartbeat_stale"
};

static const lkm_act_result_t case_causes[CASE_COUNT]={
    LKM_ACT_OK,LKM_ACT_ERR_RECEIVER,LKM_ACT_ERR_RECEIVER,LKM_ACT_ERR_BATTERY,
    LKM_ACT_ERR_SENSOR_OR_AXES,LKM_ACT_ERR_SENSOR_OR_AXES,
    LKM_ACT_ERR_SENSOR_OR_AXES,LKM_ACT_ERR_SCHEDULER,
    LKM_ACT_ERR_SENSOR_OR_AXES,LKM_ACT_ERR_SENSOR_OR_AXES,
    LKM_ACT_ERR_CONTROLLER,LKM_ACT_ERR_COMMAND,
    LKM_ACT_ERR_ESTOP,LKM_ACT_ERR_HEARTBEAT,LKM_ACT_ERR_HEARTBEAT
};

static unsigned checks=0,failed=0;
#define REQUIRE(label, expression) do { ++checks; if (!(expression)) { \
    ++failed; fprintf(stderr,"FAIL | %s | line=%d\n",label,__LINE__); } } while (0)

static void put_i16(uint8_t *p,int16_t v){uint16_t u=(uint16_t)v;p[0]=(uint8_t)u;p[1]=(uint8_t)(u>>8);}
static int16_t q14(float x){return (int16_t)lroundf(x*16384.0f);}

/* A fabricated SH-2 0xFB + 0x05 Q14 payload, decoded by the actual parser. */
static void make_bno_payload(uint8_t p[19], float roll_rad,float pitch_rad,
                             uint8_t sequence,uint8_t quality){
    memset(p,0,19);
    p[0]=0xFBu;p[5]=LKM_ATTITUDE_ROTATION_VECTOR;p[6]=sequence;p[7]=quality;
    float cr=cosf(roll_rad*.5f),sr=sinf(roll_rad*.5f);
    float cp=cosf(pitch_rad*.5f),sp=sinf(pitch_rad*.5f);
    float x=sr*cp,y=cr*sp,z=-sr*sp,w=cr*cp;
    put_i16(p+9,q14(x));put_i16(p+11,q14(y));
    put_i16(p+13,q14(z));put_i16(p+15,q14(w));
}

typedef struct {
    float roll_rad,pitch_rad,roll_rate_rad_s,pitch_rate_rad_s;
} synth_plant_t;
static void advance_synthetic_plant(synth_plant_t *p,float t_roll_nm,float t_pitch_nm){
    p->roll_rate_rad_s+=DT*(t_roll_nm/SYNTH_J_ROLL_KGM2-
                                SYNTH_D_ROLL_NMS*p->roll_rate_rad_s/SYNTH_J_ROLL_KGM2);
    p->pitch_rate_rad_s+=DT*(t_pitch_nm/SYNTH_J_PITCH_KGM2-
                                 SYNTH_D_PITCH_NMS*p->pitch_rate_rad_s/SYNTH_J_PITCH_KGM2);
    p->roll_rad+=DT*p->roll_rate_rad_s;
    p->pitch_rad+=DT*p->pitch_rate_rad_s;
}

static void target(uint32_t i,float *roll,float *pitch){
    /* Host-generated reference trajectory, no receiver command involved. */
    if(i<50u){*roll=0.0f;*pitch=0.0f;}
    else if(i<350u){*roll=5.0f;*pitch=0.0f;}
    else if(i<700u){*roll=5.0f;*pitch=-3.0f;}
    else{*roll=-4.0f;*pitch=2.0f;}
}

static bool frame_config(lkm_rp_frame_contract_t *frame){
    lkm_rp_frame_config_t fc=LKM_RP_Frame_DefaultLocked();
    /* Fixture-only flags: these do NOT represent measured board mounting. */
    fc.bno_mount_physically_verified=true;
    fc.icm_mount_physically_verified=true;
    fc.bno_quaternion_convention_verified=true;
    fc.gyro_stationary_calibration_verified=true;
    return LKM_RP_Frame_Init(frame,&fc)==LKM_FRAME_OK;
}

static bool is_zero_out(const lkm_act_virtual_output_t *out){
    if(out->virtual_valid||out->controller_ready||out->esc_output_enabled)return false;
    for(unsigned n=0;n<4;n++)if(out->virtual_model_n[n]!=0.0f)return false;
    return true;
}

static void csv_head(FILE *f){
    fprintf(f,"scenario,t_ms,t_s,injection,target_roll_deg,roll_deg,target_pitch_deg,pitch_deg,"
      "roll_rate_rad_s,pitch_rate_rad_s,requested_roll_torque_nm,achieved_roll_torque_nm,"
      "requested_pitch_torque_nm,achieved_pitch_torque_nm,"
      "motor1_model_n,motor2_model_n,motor3_model_n,motor4_model_n,"
      "virtual_valid,esc_enabled,controller_ready,actuator_state,actuator_result,"
      "bno_age_ms,icm_age_ms,host_skew_ms,timepair_result,frame_result,"
      "buffer_result,scheduler_result,preview_result,mixer_ok,model_step_count\n");
}

static void csv_row(FILE *f,const char *case_name,uint32_t ms,bool inject,
       float tr,float tp,const synth_plant_t *p,const lkm_aw_preview_output_t *model,
       const lkm_act_virtual_output_t *vout,const lkm_act_guard_t *guard,
       lkm_act_result_t result,uint32_t bno_age,uint32_t icm_age,int32_t skew,
       lkm_timepair_result_t pair_result,lkm_rp_frame_result_t frame_result,
       lkm_fc100hz_result_t buffer_result,lkm_fc100hz_result_t sched_result,
       lkm_aw_preview_result_t preview_result,bool mixer_ok,uint32_t steps){
    fprintf(f,"%s,%u,%.3f,%u,%.6f,%.6f,%.6f,%.6f,%.7f,%.7f,"
       "%.7f,%.7f,%.7f,%.7f,%.7f,%.7f,%.7f,%.7f,"
       "%u,%u,%u,%u,%s,%u,%u,%d,%s,%s,%s,%s,%s,%u,%u\n",
       case_name,ms,(double)ms/1000.0,inject?1u:0u,
       tr,p->roll_rad*RAD_DEG,tp,p->pitch_rad*RAD_DEG,
       p->roll_rate_rad_s,p->pitch_rate_rad_s,
       model->requested_roll_torque_nm,model->achieved_roll_torque_nm,
       model->requested_pitch_torque_nm,model->achieved_pitch_torque_nm,
       vout->virtual_model_n[0],vout->virtual_model_n[1],
       vout->virtual_model_n[2],vout->virtual_model_n[3],
       vout->virtual_valid?1u:0u,vout->esc_output_enabled?1u:0u,
       vout->controller_ready?1u:0u,(unsigned)guard->state,
       LKM_FC_Actuator_ResultText(result),bno_age,icm_age,skew,
       lkm_timepair_result_text(pair_result),LKM_RP_Frame_ResultText(frame_result),
       lkm_fc100hz_result_text(buffer_result),lkm_fc100hz_result_text(sched_result),
       LKM_AW_Preview_ResultText(preview_result),mixer_ok?1u:0u,steps);
}

static void run_one(FILE *csv,case_kind_t kind){
    const bool baseline=(kind==CASE_BASELINE);
    const bool recovery=(kind==CASE_RECEIVER_RECOVERY);
    const unsigned n=baseline?1002u:(recovery?310u:320u);
    const char *name=case_names[kind];
    synth_plant_t plant={0};
    lkm_timepair_t pair={0};
    lkm_timepair_config_t pc={50u,20u,15u,40.0f,LKM_ATTITUDE_ROTATION_VECTOR,2u};
    lkm_rp_frame_contract_t frame={0};
    lkm_fc100hz_scheduler_t schedule={0};
    lkm_fc100hz_buffer_t buffer={0};
    lkm_aw_preview_t preview={0};
    lkm_aw_preview_config_t model_cfg={true,10u,0.5f,8.0f};
    lkm_act_guard_t guard={0};
    lkm_act_config_t act_cfg=LKM_FC_Actuator_DefaultConfig();
    REQUIRE("timepair fixture init",lkm_timepair_init(&pair,&pc)==LKM_TIMEPAIR_OBSERVATION_OK);
    REQUIRE("frame fixture init",frame_config(&frame));
    REQUIRE("scheduler fixture init",lkm_fc100hz_sched_init(&schedule,0u,2u,2u)==LKM_FC100HZ_OK);
    lkm_fc100hz_buffer_init(&buffer);
    REQUIRE("model fixture init",LKM_AW_Preview_Init(&preview,&model_cfg)==LKM_AW_PREVIEW_OK);
    REQUIRE("actuator fixture init",LKM_FC_Actuator_Guard_Init(&guard,&act_cfg,true)==LKM_ACT_OK);
    bool saw_virtual=false,latched=false,saw_recovery=false;
    bool saw_event=false, bad_after_event=false, bad_hardlock=false;
    bool all_outputs_zero_after_event=true;
    bool no_auto_rearm=true;
    bool all_samples_before_event=true;
    uint32_t virtual_before=0u;
    uint32_t first_failure_ms=0;
    lkm_act_result_t first_cause=LKM_ACT_OK;
    uint32_t valid_steps=0,model_steps=0;
    for(uint32_t i=0;i<n;i++){
        uint32_t ms=(i+1u)*10u;
        bool injected=!baseline&&i>=INJECTION_I;
        bool at_event=!baseline && i==INJECTION_I;
        bool still_fault=injected && !(recovery&&i>=151u);
        if(kind==CASE_AXIS_UNVERIFIED && at_event)
            frame.config.bno_mount_physically_verified=false;
        float t_roll=0,t_pitch=0;
        target(i,&t_roll,&t_pitch);
        /* The injection cases retain a small synthetic 5 degree reference. */
        if(!baseline){t_roll=5.0f;t_pitch=-3.0f;}
        lkm_fc100hz_result_t sr=lkm_fc100hz_sched_poll(&schedule,ms);
        bool sched_ok=(sr==LKM_FC100HZ_READY);
        uint32_t work=1u;
        if(kind==CASE_SCHED_OVERRUN && at_event)work=8u;
        if(sched_ok){
            sr=lkm_fc100hz_sched_finish(&schedule,ms+work);
            sched_ok=(sr==LKM_FC100HZ_OK);
        }
        uint8_t p[19];
        uint32_t seq=i+1u;
        uint32_t gyro_seq=seq;
        if(kind==CASE_DUPLICATE && at_event){seq=i;gyro_seq=i;}
        uint8_t quality=(kind==CASE_BAD_QUAT_QUALITY && still_fault)?1u:3u;
        make_bno_payload(p,plant.roll_rad,plant.pitch_rad,(uint8_t)seq,quality);
        lkm_bno086_attitude_t att={0};
        lkm_attitude_result_t decoded=lkm_bno086_attitude_parse_payload(
             p,sizeof(p),LKM_ATTITUDE_SHTP_NORMAL_CHANNEL,&att);
        REQUIRE("synthetic Q14 parser step",decoded==LKM_ATTITUDE_OK);
        lkm_bno086_timed_attitude_t bno={0};
        bno.attitude=att;
        bno.accepted_packet_counter=seq;
        bno.host_received_ms=ms-2u;
        lkm_icm20602_sample_t gyro={0};
        gyro.valid=true;gyro.sample_sequence=gyro_seq;
        gyro.received_ms=ms-1u;
        gyro.gyro_sensor_xyz_rad_s[0]=plant.roll_rate_rad_s;
        gyro.gyro_sensor_xyz_rad_s[1]=plant.pitch_rate_rad_s;
        if(kind==CASE_IMU_STALE && still_fault)gyro.received_ms=ms-25u;
        if(kind==CASE_TIME_SKEW && still_fault){bno.host_received_ms=ms-18u;gyro.received_ms=ms;}
        lkm_timepair_observation_t paired={0};
        lkm_timepair_result_t tr=lkm_timepair_observe(&pair,&bno,&gyro,ms,&paired);
        lkm_rp_frame_observation_t f={0};
        lkm_rp_frame_result_t fr=LKM_FRAME_ERR_PAIR;
        if(tr==LKM_TIMEPAIR_OBSERVATION_OK)
            fr=LKM_RP_Frame_Observe(&frame,&paired,ms,&f);
        lkm_fc100hz_result_t br=LKM_FC100HZ_NO_SAMPLE;
        if(fr==LKM_FRAME_OK){
            lkm_fc100hz_snapshot_t snapshot={0},latest={0};
            snapshot.bno_present=true;snapshot.gyro_present=true;
            snapshot.bno_host_received_ms=bno.host_received_ms;
            snapshot.gyro_host_received_ms=gyro.received_ms;
            snapshot.bno_counter=bno.accepted_packet_counter;
            snapshot.gyro_counter=gyro.sample_sequence;
            snapshot.roll_deg=f.body_roll_deg;
            snapshot.pitch_deg=f.body_pitch_deg;
            memcpy(snapshot.gyro_sensor_xyz_rad_s,f.body_gyro_xyz_rad_s,
                   sizeof(snapshot.gyro_sensor_xyz_rad_s));
            br=lkm_fc100hz_buffer_push(&buffer,&snapshot,ms);
            if(br==LKM_FC100HZ_OK)
                br=lkm_fc100hz_buffer_take_latest(&buffer,ms,&latest);
            REQUIRE("no buffer controller authorization",!latest.controller_ready);
        }
        bool valid_sensors=(tr==LKM_TIMEPAIR_OBSERVATION_OK &&
                            fr==LKM_FRAME_OK && br==LKM_FC100HZ_OK);
        /* Fixture only: these are NEVER true on the production STM32. */
        bool preview_allowed=valid_sensors&&sched_ok&&
             guard.state!=LKM_ACT_STATE_FAILSAFE_LATCHED;
        lkm_aw_preview_output_t model_out={0};
        lkm_aw_preview_result_t mr=LKM_AW_PREVIEW_ERR_FEEDBACK;
        lkm_act_virtual_mixer_input_t mixin={0};
        lkm_act_model_command_t mixcmd={0};
        bool mixed=false;
        if(preview_allowed && i>=2u){
            lkm_aw_preview_command_t mc={true,t_roll*DEG_RAD,t_pitch*DEG_RAD,4.0f};
            if(kind==CASE_NAN_TARGET && still_fault)mc.target_roll_angle_rad=NAN;
            mr=LKM_AW_Preview_Tick(&preview,&f.feedback,&mc,ms,&model_out);
            if(mr==LKM_AW_PREVIEW_OK){
                ++model_steps;
                mixin.valid=true;
                mixin.produced_ms=ms;
                mixin.sequence=guard.virtual_frames+1u;
                mixin.collective_n=model_out.collective_thrust_n;
                mixin.roll_torque_nm=model_out.requested_roll_torque_nm;
                mixin.pitch_torque_nm=model_out.requested_pitch_torque_nm;
                mixin.yaw_torque_nm=0.0f;
                if(kind==CASE_NAN_MIXER && still_fault)mixin.roll_torque_nm=NAN;
                mixed=LKM_FC_VirtualMixer_Evaluate(&mixin,&mixcmd);
                if(mixed){
                    /* The standalone generated Mixer must reproduce the
                     * generated Core's motor-model thrust vector in N. */
                    REQUIRE("generated mixer equivalence motor1",
                      fabsf(mixcmd.motor_model_n[0]-model_out.motor_1_thrust_preview_n)<0.0005f);
                    REQUIRE("generated mixer equivalence motor2",
                      fabsf(mixcmd.motor_model_n[1]-model_out.motor_2_thrust_preview_n)<0.0005f);
                    REQUIRE("generated mixer equivalence motor3",
                      fabsf(mixcmd.motor_model_n[2]-model_out.motor_3_thrust_preview_n)<0.0005f);
                    REQUIRE("generated mixer equivalence motor4",
                      fabsf(mixcmd.motor_model_n[3]-model_out.motor_4_thrust_preview_n)<0.0005f);
                }
            }
        }
        lkm_act_health_t health={0};
        health.receiver_link_ok=!(kind==CASE_RECEIVER_LOSS&&still_fault) &&
                                !(recovery&&i>=INJECTION_I&&i<151u);
        health.receiver_throttle_low=true;
        health.receiver_arm_switch_high=(i!=0u);
        if(recovery&&(i==RECOVERY_I||i==RECOVERY_I+1u))
            health.receiver_arm_switch_high=false;
        health.battery_ok=!(kind==CASE_BATTERY_LOW&&still_fault);
        health.sensors_ok=valid_sensors;
        health.axes_physically_verified=(fr==LKM_FRAME_OK); /* synthetic-only */
        health.gyro_calibrated=true; /* synthetic-only */
        health.scheduler_ok=sched_ok;
        health.controller_data_valid=(i<2u)||(mr==LKM_AW_PREVIEW_OK && mixed);
        if(kind==CASE_NAN_MIXER && still_fault && mr==LKM_AW_PREVIEW_OK)
            health.controller_data_valid=true; /* isolate mixer rejection */
        health.emergency_stop=kind==CASE_ESTOP&&still_fault;
        health.receiver_rx_ms=ms;
        health.imu_rx_ms=ms;
        health.battery_rx_ms=ms;
        health.controller_rx_ms=ms;
        health.scheduler_rx_ms=ms;
        if(kind==CASE_RC_HEARTBEAT && still_fault)health.receiver_rx_ms=ms-150u;
        if(kind==CASE_BATTERY_HEARTBEAT && still_fault)health.battery_rx_ms=ms-600u;
        if(recovery&&i==RECOVERY_I){
            /* A healthy test-only controller heartbeat is required to reset.
             * This flag DOES NOT authorize physical model/ESC execution. */
            health.controller_data_valid=valid_sensors && sched_ok;
            REQUIRE("recovery requires explicitly low switch",
              LKM_FC_Actuator_Guard_Reset(&guard,&health,ms)==LKM_ACT_OK);
            LKM_AW_Preview_Invalidate(&preview);
            saw_recovery=true;
        }
        lkm_act_virtual_output_t vout={0};
        lkm_act_result_t ar=LKM_FC_Actuator_Guard_Tick(&guard,&health,
                   mixed?&mixcmd:NULL,ms,&vout);
        if(vout.virtual_valid){
            REQUIRE("actual output is virtual-model only",guard.state==LKM_ACT_STATE_ARMED_VIRTUAL);
            REQUIRE("successful virtual frame has verified upstream",valid_sensors&&sched_ok&&mixed&&mr==LKM_AW_PREVIEW_OK);
            ++valid_steps;
            saw_virtual=true;
            /* Only the artificial plant responds to MODEL achieved torque. */
            advance_synthetic_plant(&plant,model_out.achieved_roll_torque_nm,
                                            model_out.achieved_pitch_torque_nm);
        }
        if(!isfinite(plant.roll_rad)||!isfinite(plant.pitch_rad)||
           !isfinite(plant.roll_rate_rad_s)||!isfinite(plant.pitch_rate_rad_s)||
           fabsf(plant.roll_rad)>MAX_SIL_ABS_ANGLE||
           fabsf(plant.pitch_rad)>MAX_SIL_ABS_ANGLE||
           fabsf(plant.roll_rate_rad_s)>MAX_SIL_ABS_RATE||
           fabsf(plant.pitch_rate_rad_s)>MAX_SIL_ABS_RATE){
            ++failed;fprintf(stderr,"FAIL | synthetic plant diverged | %s %u\n",name,ms);
        }
        if(at_event){
            saw_event=true;first_failure_ms=ms;first_cause=ar;
            virtual_before=guard.virtual_frames;
            REQUIRE("injected fault triggers a safe latched failure",
                guard.state==LKM_ACT_STATE_FAILSAFE_LATCHED);
            REQUIRE("injected fault reason matches selected scenario",ar==case_causes[kind]);
        }
        if(saw_event && i>=INJECTION_I && !(recovery&&i>=RECOVERY_I)){
            if(vout.virtual_valid||!is_zero_out(&vout))all_outputs_zero_after_event=false;
            if(guard.state!=LKM_ACT_STATE_FAILSAFE_LATCHED)no_auto_rearm=false;
        }
        if(guard.state==LKM_ACT_STATE_FAILSAFE_LATCHED){
            latched=true;
            if(!is_zero_out(&vout))bad_after_event=true;
        }
        if(vout.esc_output_enabled||vout.controller_ready||guard.esc_output_enabled||
           guard.controller_ready|| f.controller_ready || f.esc_output_enabled ||
           paired.controller_feedback_ready)bad_hardlock=true;
        if(i>=2u && i<INJECTION_I && !vout.virtual_valid)all_samples_before_event=false;
        uint32_t bno_age=ms-bno.host_received_ms,icm_age=ms-gyro.received_ms;
        int32_t skew=(int32_t)gyro.received_ms-(int32_t)bno.host_received_ms;
        csv_row(csv,name,ms,injected,t_roll,t_pitch,&plant,&model_out,&vout,
            &guard,ar,bno_age,icm_age,skew,tr,fr,br,sr,mr,mixed,model_steps);
        if(guard.state==LKM_ACT_STATE_FAILSAFE_LATCHED) LKM_AW_Preview_Invalidate(&preview);
    }
    REQUIRE("all scenarios never enable actual ESC/control",!bad_hardlock);
    REQUIRE("safe virtual output cleared after failure",!bad_after_event);
    REQUIRE("virtual model frames exist before injection",saw_virtual && all_samples_before_event);
    if(baseline){
        REQUIRE("baseline 1000 exact virtual frames",guard.virtual_frames==1000u);
        REQUIRE("baseline zero failure events",guard.failsafe_events==0);
        REQUIRE("baseline no dropped deadlines",schedule.missed_slots==0&&schedule.overrun_events==0);
        REQUIRE("baseline exactly 1000 model steps",model_steps==1000u);
    }else{
        REQUIRE("fault scenario reached specified injection",saw_event && first_failure_ms==1210u);
        REQUIRE("fault scenario latches failsafe",latched);
        REQUIRE("fault clears virtual outputs after event",all_outputs_zero_after_event);
        REQUIRE("fault cannot automatically re-arm",no_auto_rearm);
        if(recovery){
            REQUIRE("manual recovery explicitly reset and rearmed",saw_recovery&&guard.state==LKM_ACT_STATE_ARMED_VIRTUAL);
            REQUIRE("manual recovery generates fresh virtual frames only",guard.virtual_frames>virtual_before);
            REQUIRE("manual recovery keeps at least one failsafe log",guard.failsafe_events==1);
        }else{
            REQUIRE("fault remains latched for entire remaining scenario",guard.state==LKM_ACT_STATE_FAILSAFE_LATCHED);
            REQUIRE("exactly one failsafe event latched",guard.failsafe_events==1u);
            REQUIRE("no output accepted after fault",guard.virtual_frames==virtual_before);
        }
    }
    printf("CASE %-27s | virtual=%-5u | fault_events=%u | first_fault=%u | cause=%s\n",
           name,guard.virtual_frames,guard.failsafe_events,first_failure_ms,
           LKM_FC_Actuator_ResultText(first_cause));
}

int main(int argc,char **argv){
    if(argc!=2){fprintf(stderr,"Usage: integrated_sil <absolute_output_csv_path>\n");return 2;}
    FILE *f=fopen(argv[1],"w");
    if(!f){perror("output csv");return 2;}
    csv_head(f);
    for(unsigned k=0;k<CASE_COUNT;k++)run_one(f,(case_kind_t)k);
    if(fclose(f)!=0){perror("CSV fclose");return 2;}
    printf("=== INTEGRATED HOST SIL: %u/%u CHECKS PASS; %u SCENARIOS ===\n",
           checks-failed,checks,(unsigned)CASE_COUNT);
    printf("CSV: %s\n",argv[1]);
    printf("HOST FIXTURE ONLY: synthetic IMU packet bytes, mock RC/battery, "
           "invented linear plant; NO physical axes, MCU, ESC or flight.\n");
    return failed?1:0;
}
