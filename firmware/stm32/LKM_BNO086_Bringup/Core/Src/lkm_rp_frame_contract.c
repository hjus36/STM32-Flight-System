#include "lkm_rp_frame_contract.h"
#include <limits.h>
#include <math.h>
#include <string.h>

#if (defined(STM32F405xx) || defined(USE_HAL_DRIVER) || defined(__arm__) || defined(__thumb__)) && \
    defined(LKM_RP_FRAME_HOST_FIXTURE) && LKM_RP_FRAME_HOST_FIXTURE
#error "Synthetic frame approval is forbidden in STM32F405 production firmware"
#endif

#if defined(LKM_RP_FRAME_HOST_FIXTURE) && LKM_RP_FRAME_HOST_FIXTURE
#define LKM_FRAME_TEST_APPROVAL 1
#else
#define LKM_FRAME_TEST_APPROVAL 0
#endif

#define RAD_TO_DEG 57.2957795130823208768f
#define MAX_SAFE_AGE 0x7FFFFFFFu

typedef struct {float w,x,y,z;} q_t;
static void bump(uint32_t *n){if (*n!=UINT32_MAX)++(*n);}
static bool finite4(float a,float b,float c,float d) {
    return isfinite(a)&&isfinite(b)&&isfinite(c)&&isfinite(d);
}
static bool flags_verified(const lkm_rp_frame_config_t *c) {
    return c->bno_mount_physically_verified && c->icm_mount_physically_verified &&
           c->bno_quaternion_convention_verified && c->gyro_stationary_calibration_verified;
}
static bool any_flags(const lkm_rp_frame_config_t *c) {
    return c->bno_mount_physically_verified || c->icm_mount_physically_verified ||
           c->bno_quaternion_convention_verified || c->gyro_stationary_calibration_verified;
}

bool LKM_RP_Mount90_IsProper(const lkm_rp_mount90_t *m) {
    if (!m)return false;
    for (unsigned i=0;i<3;i++) {
        unsigned row=0,col=0;
        for (unsigned j=0;j<3;j++) {
            int a=m->m[i][j],b=m->m[j][i];
            if (a!=0&&a!=1&&a!=-1)return false;
            if (b!=0&&b!=1&&b!=-1)return false;
            if (a!=0)++row;
            if (b!=0)++col;
        }
        if (row!=1||col!=1)return false;
    }
    /* det is computed in integer arithmetic. */
    int a00=m->m[0][0],a01=m->m[0][1],a02=m->m[0][2];
    int a10=m->m[1][0],a11=m->m[1][1],a12=m->m[1][2];
    int a20=m->m[2][0],a21=m->m[2][1],a22=m->m[2][2];
    int det=a00*(a11*a22-a12*a21)-a01*(a10*a22-a12*a20)+a02*(a10*a21-a11*a20);
    return det==1;
}

lkm_rp_frame_config_t LKM_RP_Frame_DefaultLocked(void) {
    lkm_rp_frame_config_t c={0};
    for (unsigned i=0;i<3;i++) {
        c.bno_sensor_to_body.m[i][i]=1;
        c.icm_sensor_to_body.m[i][i]=1;
    }
    c.expected_bno_report_id=LKM_ATTITUDE_ROTATION_VECTOR;
    c.min_accuracy=2u;
    c.max_bno_age_ms=50u;
    c.max_gyro_age_ms=20u;
    c.max_host_skew_ms=15u;
    c.max_abs_roll_deg=70.0f;
    c.max_abs_pitch_deg=60.0f;
    c.max_abs_body_rate_rad_s=25.0f;
    return c;
}
static bool valid_cfg(const lkm_rp_frame_config_t *c) {
    return LKM_RP_Mount90_IsProper(&c->bno_sensor_to_body) &&
        LKM_RP_Mount90_IsProper(&c->icm_sensor_to_body) &&
        (c->expected_bno_report_id==LKM_ATTITUDE_ROTATION_VECTOR ||
         c->expected_bno_report_id==LKM_ATTITUDE_GAME_ROTATION_VECTOR) &&
        c->min_accuracy<=3u &&
        c->max_bno_age_ms>0 && c->max_bno_age_ms<=60000u &&
        c->max_gyro_age_ms>0 && c->max_gyro_age_ms<=60000u &&
        c->max_host_skew_ms>0 && c->max_host_skew_ms<=60000u &&
        isfinite(c->max_abs_roll_deg) && isfinite(c->max_abs_pitch_deg) &&
        isfinite(c->max_abs_body_rate_rad_s) &&
        c->max_abs_roll_deg>0 && c->max_abs_roll_deg<=180 &&
        c->max_abs_pitch_deg>0 && c->max_abs_pitch_deg<=90 &&
        c->max_abs_body_rate_rad_s>0 && c->max_abs_body_rate_rad_s<=100;
}

lkm_rp_frame_result_t LKM_RP_Frame_Init(
    lkm_rp_frame_contract_t *ctx, const lkm_rp_frame_config_t *c) {
    if (!ctx||!c)return LKM_FRAME_ERR_ARGUMENT;
    memset(ctx,0,sizeof(*ctx));
    if (!valid_cfg(c))return LKM_FRAME_ERR_CONFIG;
    /* No source-code change or caller-provided TRUE bypasses the build lock. */
    if (!LKM_FRAME_TEST_APPROVAL && any_flags(c))return LKM_FRAME_ERR_APPROVAL_LOCKED;
    ctx->config=*c;
    lkm_rp_gate_config_t g={0};
    g.angle_axes_verified=LKM_FRAME_TEST_APPROVAL && flags_verified(c);
    g.gyro_axes_verified=LKM_FRAME_TEST_APPROVAL && flags_verified(c);
    g.roll_sign=1;g.pitch_sign=1;g.body_p_sign=1;g.body_q_sign=1;
    g.expected_report_id=c->expected_bno_report_id;
    g.minimum_accuracy=c->min_accuracy;
    g.max_attitude_age_ms=c->max_bno_age_ms;
    g.max_gyro_age_ms=c->max_gyro_age_ms;
    g.max_sample_skew_ms=c->max_host_skew_ms;
    g.max_abs_roll_deg=c->max_abs_roll_deg;
    g.max_abs_pitch_deg=c->max_abs_pitch_deg;
    g.max_abs_body_rate_rad_s=c->max_abs_body_rate_rad_s;
    if (LKM_RP_Gate_Init(&ctx->gate,&g)!=LKM_RP_GATE_OK)return LKM_FRAME_ERR_CONFIG;
    ctx->initialized=true;
    ctx->last_gate_result=LKM_RP_GATE_ERR_FRAME_UNVERIFIED;
    return LKM_FRAME_OK;
}

static q_t q_mul(q_t a,q_t b) {
    q_t q={a.w*b.w-a.x*b.x-a.y*b.y-a.z*b.z,
          a.w*b.x+a.x*b.w+a.y*b.z-a.z*b.y,
          a.w*b.y-a.x*b.z+a.y*b.w+a.z*b.x,
          a.w*b.z+a.x*b.y-a.y*b.x+a.z*b.w};
    return q;
}
static q_t q_conj(q_t q) {q_t r={q.w,-q.x,-q.y,-q.z};return r;}
static q_t q_from_euler(float roll,float pitch,float yaw) {
    float cr=cosf(roll*.5f),sr=sinf(roll*.5f);
    float cp=cosf(pitch*.5f),sp=sinf(pitch*.5f);
    float cy=cosf(yaw*.5f),sy=sinf(yaw*.5f);
    q_t q={cr*cp*cy+sr*sp*sy,sr*cp*cy-cr*sp*sy,
          cr*sp*cy+sr*cp*sy,cr*cp*sy-sr*sp*cy};
    return q;
}
static bool q_unit(q_t *q) {
    if (!finite4(q->w,q->x,q->y,q->z))return false;
    float s=q->w*q->w+q->x*q->x+q->y*q->y+q->z*q->z;
    if (!isfinite(s)||s<.94f||s>1.06f)return false;
    float inv=1.0f/sqrtf(s);
    q->w*=inv;q->x*=inv;q->y*=inv;q->z*=inv;
    return finite4(q->w,q->x,q->y,q->z);
}
/* For a 90-degree signed permutation rotation matrix R_BS, construct its
 * Hamilton active q_BS. Choose max diagonal branch to avoid cancellations. */
static bool matrix_quat(const lkm_rp_mount90_t *matrix,q_t *out) {
    const float a=(float)matrix->m[0][0],b=(float)matrix->m[0][1],c=(float)matrix->m[0][2];
    const float d=(float)matrix->m[1][0],e=(float)matrix->m[1][1],f=(float)matrix->m[1][2];
    const float g=(float)matrix->m[2][0],h=(float)matrix->m[2][1],i=(float)matrix->m[2][2];
    q_t q={0};float s;
    float trace=a+e+i;
    if (trace>0.0f) {
        s=sqrtf(trace+1.0f)*2.0f;
        q.w=.25f*s;q.x=(h-f)/s;q.y=(c-g)/s;q.z=(d-b)/s;
    } else if(a>e && a>i) {
        s=sqrtf(1.0f+a-e-i)*2.0f;
        q.w=(h-f)/s;q.x=.25f*s;q.y=(b+d)/s;q.z=(c+g)/s;
    } else if(e>i) {
        s=sqrtf(1.0f+e-a-i)*2.0f;
        q.w=(c-g)/s;q.x=(b+d)/s;q.y=.25f*s;q.z=(f+h)/s;
    } else {
        s=sqrtf(1.0f+i-a-e)*2.0f;
        q.w=(d-b)/s;q.x=(c+g)/s;q.y=(f+h)/s;q.z=.25f*s;
    }
    if (!q_unit(&q))return false;
    *out=q;return true;
}
static bool q_euler(q_t q,float *r,float *p,float *y) {
    const float sinr=2.0f*(q.w*q.x+q.y*q.z);
    const float cosr=1.0f-2.0f*(q.x*q.x+q.y*q.y);
    const float sinp=2.0f*(q.w*q.y-q.z*q.x);
    const float siny=2.0f*(q.w*q.z+q.x*q.y);
    const float cosy=1.0f-2.0f*(q.y*q.y+q.z*q.z);
    float rp=atan2f(sinr,cosr)*RAD_TO_DEG;
    float pp=asinf(fmaxf(-1.0f,fminf(1.0f,sinp)))*RAD_TO_DEG;
    float yp=atan2f(siny,cosy)*RAD_TO_DEG;
    if (!isfinite(rp)||!isfinite(pp)||!isfinite(yp))return false;
    *r=rp;*p=pp;*y=yp;return true;
}
static bool monotonic(uint32_t newer,uint32_t older) {
    uint32_t delta=(uint32_t)(newer-older);
    return delta!=0u && delta<MAX_SAFE_AGE;
}
static bool age_ok(uint32_t now,uint32_t rx,uint32_t ttl) {
    return (uint32_t)(now-rx)<=ttl;
}
static bool skew_ok(uint32_t a,uint32_t b,uint32_t max_skew) {
    uint32_t d=(uint32_t)(a-b);
    uint32_t abs_d=d<=MAX_SAFE_AGE?d:(uint32_t)(0u-d);
    return abs_d<=max_skew;
}

lkm_rp_frame_result_t LKM_RP_Frame_Observe(
    lkm_rp_frame_contract_t *ctx,const lkm_timepair_observation_t *pair,
    uint32_t now_ms,lkm_rp_frame_observation_t *out) {
    if (out)memset(out,0,sizeof(*out));
    if (!ctx||!out)return LKM_FRAME_ERR_ARGUMENT;
    if (!ctx->initialized)return LKM_FRAME_ERR_NOT_INITIALIZED;
    if (!LKM_FRAME_TEST_APPROVAL || !flags_verified(&ctx->config)) {
        bump(&ctx->rejected);
        return LKM_FRAME_ERR_UNVERIFIED;
    }
    if (!pair || pair->controller_feedback_ready || !pair->gyro_sensor.valid ||
        pair->bno.accepted_packet_counter==0 || pair->gyro_sensor.sample_sequence==0 ||
        pair->bno.attitude.report_id!=ctx->config.expected_bno_report_id ||
        pair->bno.attitude.accuracy_status>3 ||
        pair->bno.attitude.accuracy_status<ctx->config.min_accuracy ||
        !age_ok(now_ms,pair->bno.host_received_ms,ctx->config.max_bno_age_ms) ||
        !age_ok(now_ms,pair->gyro_sensor.received_ms,ctx->config.max_gyro_age_ms) ||
        !skew_ok(pair->bno.host_received_ms,pair->gyro_sensor.received_ms,
                 ctx->config.max_host_skew_ms)) {
        bump(&ctx->rejected);return LKM_FRAME_ERR_PAIR;
    }
    if (ctx->have_last_pair &&
       (!monotonic(pair->bno.accepted_packet_counter,ctx->last_bno_packet_counter) ||
        !monotonic(pair->gyro_sensor.sample_sequence,ctx->last_icm_sample_sequence))) {
        bump(&ctx->rejected);return LKM_FRAME_ERR_OLD_PAIR;
    }
    const lkm_bno086_attitude_t *a=&pair->bno.attitude;
    q_t q_ws={a->real,a->i,a->j,a->k};
    if (!q_unit(&q_ws)) {bump(&ctx->rejected);return LKM_FRAME_ERR_QUATERNION;}
    if (!finite4(a->roll_deg,a->pitch_deg,a->yaw_deg,0.0f) ||
        fabsf(a->roll_deg)>180.001f || fabsf(a->pitch_deg)>90.001f ||
        fabsf(a->yaw_deg)>180.001f) {
        bump(&ctx->rejected);return LKM_FRAME_ERR_QUATERNION;
    }
    const q_t expected=q_from_euler(a->roll_deg/RAD_TO_DEG,
                                    a->pitch_deg/RAD_TO_DEG,
                                    a->yaw_deg/RAD_TO_DEG);
    const float dot=fabsf(q_ws.w*expected.w+q_ws.x*expected.x+
                          q_ws.y*expected.y+q_ws.z*expected.z);
    if (!isfinite(dot)||dot<0.9999f) {
        bump(&ctx->rejected);return LKM_FRAME_ERR_EULER_MISMATCH;
    }
    for (unsigned i=0;i<3;i++) {
        if (!isfinite(pair->gyro_sensor.gyro_sensor_xyz_rad_s[i]) ||
            fabsf(pair->gyro_sensor.gyro_sensor_xyz_rad_s[i])>100.0f) {
            bump(&ctx->rejected);return LKM_FRAME_ERR_GYRO;
        }
    }
    q_t q_bs;
    if (!matrix_quat(&ctx->config.bno_sensor_to_body,&q_bs)) {
        bump(&ctx->rejected);return LKM_FRAME_ERR_CONFIG;
    }
    q_t q_wb=q_mul(q_ws,q_conj(q_bs));
    if (!q_unit(&q_wb)) {bump(&ctx->rejected);return LKM_FRAME_ERR_QUATERNION;}
    float roll=0.0f,pitch=0.0f,yaw=0.0f;
    if (!q_euler(q_wb,&roll,&pitch,&yaw)) {
        bump(&ctx->rejected);return LKM_FRAME_ERR_QUATERNION;
    }
    float w_body[3]={0};
    for (unsigned i=0;i<3;i++) for(unsigned j=0;j<3;j++)
        w_body[i]+=(float)ctx->config.icm_sensor_to_body.m[i][j]*
                   pair->gyro_sensor.gyro_sensor_xyz_rad_s[j];
    lkm_bno086_attitude_t body_pose=*a;
    body_pose.i=q_wb.x;body_pose.j=q_wb.y;body_pose.k=q_wb.z;body_pose.real=q_wb.w;
    body_pose.roll_deg=roll;body_pose.pitch_deg=pitch;body_pose.yaw_deg=yaw;
    lkm_rp_gate_input_t gi={0};
    gi.attitude_ok=true;gi.attitude=&body_pose;
    gi.attitude_received_ms=pair->bno.host_received_ms;
    gi.gyro_ok=true;gi.gyro_body_p_rad_s=w_body[0];gi.gyro_body_q_rad_s=w_body[1];
    gi.gyro_received_ms=pair->gyro_sensor.received_ms;
    lkm_rp_feedback_t feedback={0};
    ctx->last_gate_result=LKM_RP_Gate_Evaluate(&ctx->gate,&gi,now_ms,&feedback);
    if (ctx->last_gate_result!=LKM_RP_GATE_OK || !feedback.feedback_valid) {
        bump(&ctx->rejected);return LKM_FRAME_ERR_GATE;
    }
    out->diagnostic_valid=true;
    out->controller_ready=false;
    out->esc_output_enabled=false;
    out->body_gyro_xyz_rad_s[0]=w_body[0];
    out->body_gyro_xyz_rad_s[1]=w_body[1];
    out->body_gyro_xyz_rad_s[2]=w_body[2];
    out->body_roll_deg=roll;out->body_pitch_deg=pitch;out->body_yaw_deg=yaw;
    out->bno_received_ms=pair->bno.host_received_ms;
    out->icm_received_ms=pair->gyro_sensor.received_ms;
    out->bno_packet_counter=pair->bno.accepted_packet_counter;
    out->icm_sample_sequence=pair->gyro_sensor.sample_sequence;
    out->feedback=feedback; /* A host fixture result, NOT flight/arming authority */
    ctx->last_bno_packet_counter=pair->bno.accepted_packet_counter;
    ctx->last_icm_sample_sequence=pair->gyro_sensor.sample_sequence;
    ctx->have_last_pair=true;
    bump(&ctx->accepted);
    return LKM_FRAME_OK;
}

const char *LKM_RP_Frame_ResultText(lkm_rp_frame_result_t r) {
    switch(r) {
    case LKM_FRAME_OK:return "SYNTHETIC_ONLY";
    case LKM_FRAME_ERR_ARGUMENT:return "ARGUMENT";
    case LKM_FRAME_ERR_CONFIG:return "CONFIG";
    case LKM_FRAME_ERR_APPROVAL_LOCKED:return "APPROVAL_LOCKED";
    case LKM_FRAME_ERR_NOT_INITIALIZED:return "NOT_INITIALIZED";
    case LKM_FRAME_ERR_UNVERIFIED:return "FRAME_UNVERIFIED";
    case LKM_FRAME_ERR_PAIR:return "PAIR_INVALID";
    case LKM_FRAME_ERR_OLD_PAIR:return "PAIR_REUSED_OR_REWOUND";
    case LKM_FRAME_ERR_QUATERNION:return "QUATERNION_INVALID";
    case LKM_FRAME_ERR_EULER_MISMATCH:return "EULER_MISMATCH";
    case LKM_FRAME_ERR_GYRO:return "GYRO_INVALID";
    case LKM_FRAME_ERR_GATE:return "FEEDBACK_GATE_REJECTED";
    default:return "UNKNOWN";
    }
}
