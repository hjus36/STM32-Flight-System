/* Negative production fixture: no synthetic arming flags defined. */
#if defined(LKM_FC_ACTUATOR_HOST_FIXTURE) || defined(LKM_RP_FRAME_HOST_FIXTURE)
#error "This test MUST compile without host fixture approval macros"
#endif
#include <stdio.h>
#include <string.h>
#include "lkm_fc_actuator_guard.h"
#include "lkm_rp_frame_contract.h"
static int n=0,fail=0;
#define TEST(expr) do { n++; if(!(expr)){++fail;fprintf(stderr,"FAIL production check %d\n",n);} }while(0)
int main(void){
    lkm_act_guard_t g={0};
    lkm_act_config_t cfg=LKM_FC_Actuator_DefaultConfig();
    lkm_act_virtual_output_t out;memset(&out,0x55,sizeof(out));
    TEST(LKM_FC_Actuator_Guard_Init(&g,&cfg,true)==LKM_ACT_ERR_PRODUCTION_LOCK);
    TEST(g.state==LKM_ACT_STATE_LOCKED);
    TEST(LKM_FC_Actuator_Guard_Tick(&g,NULL,NULL,100,&out)==LKM_ACT_ERR_PRODUCTION_LOCK);
    TEST(!out.virtual_valid && !out.esc_output_enabled && !out.controller_ready);
    for(unsigned i=0;i<4;i++)TEST(out.virtual_model_n[i]==0.0f);
    TEST(LKM_FC_Actuator_Guard_Reset(&g,NULL,110)==LKM_ACT_ERR_PRODUCTION_LOCK);
    lkm_rp_frame_contract_t frame={0};
    lkm_rp_frame_config_t locked=LKM_RP_Frame_DefaultLocked();
    TEST(LKM_RP_Frame_Init(&frame,&locked)==LKM_FRAME_OK);
    lkm_rp_frame_observation_t o={0};
    TEST(LKM_RP_Frame_Observe(&frame,NULL,100,&o)==LKM_FRAME_ERR_UNVERIFIED);
    TEST(!o.diagnostic_valid && !o.controller_ready && !o.esc_output_enabled);
    locked.bno_mount_physically_verified=true;
    TEST(LKM_RP_Frame_Init(&frame,&locked)==LKM_FRAME_ERR_APPROVAL_LOCKED);
    printf("=== PRODUCTION HARD LOCK: %d/%d PASS ===\n",n-fail,n);
    return fail?1:0;
}
