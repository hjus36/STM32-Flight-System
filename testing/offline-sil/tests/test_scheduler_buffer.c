#include "lkm_fc100hz_core.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdint.h>
#include <limits.h>
static unsigned passed;
#define CHECK(LABEL, CONDITION) do {if (!(CONDITION)) {fprintf(stderr,"FAIL %u: %s (line %d)\n",passed+1u,LABEL,__LINE__);return 1;} else {++passed;printf("PASS %02u: %s\n",passed,LABEL);}} while(0)
#define INIT(S,T) lkm_fc100hz_sched_init(&(S),(T),2u,5u)
static lkm_fc100hz_snapshot_t bno(uint32_t seq, uint32_t t)
{
    lkm_fc100hz_snapshot_t s = {0};s.bno_present=true;s.bno_counter=seq;
    s.bno_host_received_ms=t;s.roll_deg=3.0f;s.pitch_deg=-2.0f;
    s.controller_ready=true; /* hostile/unsafe input must be cleared */
    s.gyro_sensor_xyz_rad_s[0]=123.0f; /* absent sensor must be zeroed */
    return s;
}
static lkm_fc100hz_snapshot_t gyro(uint32_t seq, uint32_t t)
{
    lkm_fc100hz_snapshot_t s={0};s.gyro_present=true;s.gyro_counter=seq;
    s.gyro_host_received_ms=t;s.gyro_sensor_xyz_rad_s[0]=0.5f;
    s.bno_counter=777u;s.roll_deg=100.0f; /* absent BNO must be zeroed */
    return s;
}
int main(void)
{
    lkm_fc100hz_scheduler_t s={0};
    CHECK("init null",lkm_fc100hz_sched_init(NULL,0,2,5)==LKM_FC100HZ_BAD_ARGUMENT);
    CHECK("bad lateness zero",lkm_fc100hz_sched_init(&s,0,0,5)==LKM_FC100HZ_BAD_CONFIG);
    CHECK("bad lateness 10",lkm_fc100hz_sched_init(&s,0,10,5)==LKM_FC100HZ_BAD_CONFIG);
    CHECK("bad work zero",lkm_fc100hz_sched_init(&s,0,2,0)==LKM_FC100HZ_BAD_CONFIG);
    CHECK("bad work 10",lkm_fc100hz_sched_init(&s,0,2,10)==LKM_FC100HZ_BAD_CONFIG);
    CHECK("normal init",INIT(s,100)==LKM_FC100HZ_OK);
    CHECK("first deadline 110",s.next_due_ms==110u);
    CHECK("controller disabled",!s.controller_ready);
    CHECK("poll before due",lkm_fc100hz_sched_poll(&s,100)==LKM_FC100HZ_WAIT);
    CHECK("poll t109 wait",lkm_fc100hz_sched_poll(&s,109)==LKM_FC100HZ_WAIT);
    CHECK("poll at 110 ready",lkm_fc100hz_sched_poll(&s,110)==LKM_FC100HZ_READY);
    CHECK("inflight set",s.work_in_flight);
    CHECK("deadline advanced to 120",s.next_due_ms==120u);
    CHECK("one dispatch",s.dispatch_count==1u);
    CHECK("finish within 1ms",lkm_fc100hz_sched_finish(&s,111)==LKM_FC100HZ_OK);
    CHECK("inflight cleared",!s.work_in_flight);
    CHECK("work max1",s.max_work_ms_seen==1u);
    CHECK("t119 wait",lkm_fc100hz_sched_poll(&s,119)==LKM_FC100HZ_WAIT);
    CHECK("t120 ready",lkm_fc100hz_sched_poll(&s,120)==LKM_FC100HZ_READY);
    CHECK("finish immediate",lkm_fc100hz_sched_finish(&s,120)==LKM_FC100HZ_OK);
    CHECK("2 dispatches",s.dispatch_count==2u);
    CHECK("on-time no missed",s.missed_slots==0u);
    CHECK("allow 2ms lateness",lkm_fc100hz_sched_poll(&s,132)==LKM_FC100HZ_READY);
    CHECK("lateness max 2",s.max_lateness_ms==2u);
    CHECK("finish after late+3 within budget",lkm_fc100hz_sched_finish(&s,135)==LKM_FC100HZ_OK);
    CHECK("t140 ready",lkm_fc100hz_sched_poll(&s,140)==LKM_FC100HZ_READY);
    CHECK("finish t145 max budget",lkm_fc100hz_sched_finish(&s,145)==LKM_FC100HZ_OK);
    CHECK("max work5",s.max_work_ms_seen==5u);
    CHECK("t150 ready",lkm_fc100hz_sched_poll(&s,150)==LKM_FC100HZ_READY);
    CHECK("overrun 6ms latched",lkm_fc100hz_sched_finish(&s,156)==LKM_FC100HZ_OVERRUN);
    CHECK("overrun count",s.overrun_events==1u);
    CHECK("overrun fault latch",s.fault_latched);
    CHECK("no more tick after overrun",lkm_fc100hz_sched_poll(&s,160)==LKM_FC100HZ_FAULT);
    CHECK("reinit clears fault",INIT(s,200)==LKM_FC100HZ_OK && !s.fault_latched);
    CHECK("3ms late fails closed",lkm_fc100hz_sched_poll(&s,213)==LKM_FC100HZ_LATE_DROP);
    CHECK("late counted",s.late_events==1u);
    CHECK("miss current deadline",s.missed_slots==1u);
    CHECK("late fault latch",s.fault_latched);
    CHECK("reinit again",INIT(s,0)==LKM_FC100HZ_OK);
    CHECK("many missed ticks drop",lkm_fc100hz_sched_poll(&s,55)==LKM_FC100HZ_LATE_DROP);
    CHECK("five slots missed total",s.missed_slots==5u);
    CHECK("one late event",s.late_events==1u);
    CHECK("no catchup dispatch",s.dispatch_count==0u);
    CHECK("reinit third",INIT(s,0)==LKM_FC100HZ_OK);
    CHECK("double poll first ready",lkm_fc100hz_sched_poll(&s,10)==LKM_FC100HZ_READY);
    CHECK("second poll without finish fault",lkm_fc100hz_sched_poll(&s,10)==LKM_FC100HZ_FAULT);
    CHECK("double poll fault latched",s.fault_latched);
    CHECK("reinit fourth",INIT(s,0)==LKM_FC100HZ_OK);
    CHECK("finish without start invalid",lkm_fc100hz_sched_finish(&s,1)==LKM_FC100HZ_FAULT);
    CHECK("reinit fifth",INIT(s,100)==LKM_FC100HZ_OK);
    CHECK("backward time fault",lkm_fc100hz_sched_poll(&s,99)==LKM_FC100HZ_FAULT);
    CHECK("backward fault latch",s.fault_latched);
    CHECK("wrap init",INIT(s,UINT32_MAX-5u)==LKM_FC100HZ_OK);
    CHECK("wrap next due t4",s.next_due_ms==4u);
    CHECK("wrap wait past max",lkm_fc100hz_sched_poll(&s,UINT32_MAX-1u)==LKM_FC100HZ_WAIT);
    CHECK("wrap due",lkm_fc100hz_sched_poll(&s,4)==LKM_FC100HZ_READY);
    CHECK("wrap finish",lkm_fc100hz_sched_finish(&s,5)==LKM_FC100HZ_OK);
    CHECK("wrap next tick ready",lkm_fc100hz_sched_poll(&s,14)==LKM_FC100HZ_READY);
    CHECK("wrap second finish",lkm_fc100hz_sched_finish(&s,15)==LKM_FC100HZ_OK);
    CHECK("no wrap false miss",s.missed_slots==0u);
    CHECK("too large clock jump rejected",lkm_fc100hz_sched_poll(&s,UINT32_C(0x90000000))==LKM_FC100HZ_FAULT);
    CHECK("1000-cycle mock init",INIT(s,0)==LKM_FC100HZ_OK);
    bool thousand_good=true;
    for (uint32_t cycle=1u; cycle<=1000u; ++cycle) {
        const uint32_t t=cycle*LKM_FC100HZ_PERIOD_MS;
        if (lkm_fc100hz_sched_poll(&s,t)!=LKM_FC100HZ_READY ||
            lkm_fc100hz_sched_finish(&s,t+1u)!=LKM_FC100HZ_OK) {
            thousand_good=false;break;
        }
    }
    CHECK("1000 mock 100Hz slots dispatch",thousand_good && s.dispatch_count==1000u);
    CHECK("1000 slots: no missed, no overruns",s.missed_slots==0u && s.overrun_events==0u);
    CHECK("1000 slots advance to 10010",s.next_due_ms==10010u);
    CHECK("no real MCU timer claimed",!s.controller_ready);
    CHECK("NULL scheduler poll",lkm_fc100hz_sched_poll(NULL,0)==LKM_FC100HZ_BAD_ARGUMENT);
    CHECK("NULL scheduler finish",lkm_fc100hz_sched_finish(NULL,0)==LKM_FC100HZ_BAD_ARGUMENT);
    CHECK("result status text",strcmp(lkm_fc100hz_result_text(LKM_FC100HZ_LATE_DROP),"LATE_DROP")==0);
    CHECK("result unknown safe",strcmp(lkm_fc100hz_result_text((lkm_fc100hz_result_t)400),"UNKNOWN")==0);

    lkm_fc100hz_buffer_t q={0};lkm_fc100hz_snapshot_t out={0},s1=bno(1,100);
    CHECK("NULL buffer push",lkm_fc100hz_buffer_push(NULL,&s1,100)==LKM_FC100HZ_BAD_ARGUMENT);
    CHECK("NULL sample push",lkm_fc100hz_buffer_push(&q,NULL,100)==LKM_FC100HZ_BAD_ARGUMENT);
    CHECK("uninit buffer blocks",lkm_fc100hz_buffer_push(&q,&s1,100)==LKM_FC100HZ_FAULT);
    CHECK("NULL take output",lkm_fc100hz_buffer_take_latest(&q,100,NULL)==LKM_FC100HZ_BAD_ARGUMENT);
    CHECK("NULL take context",lkm_fc100hz_buffer_take_latest(NULL,100,&out)==LKM_FC100HZ_BAD_ARGUMENT);
    lkm_fc100hz_buffer_init(&q);
    CHECK("init buffer",q.initialized && q.count==0u);
    out.controller_ready=true;
    CHECK("empty take no sample",lkm_fc100hz_buffer_take_latest(&q,100,&out)==LKM_FC100HZ_NO_SAMPLE);
    CHECK("empty take zero output",!out.controller_ready && !out.bno_present);
    lkm_fc100hz_snapshot_t empty={0};
    CHECK("reject empty sources",lkm_fc100hz_buffer_push(&q,&empty,100)==LKM_FC100HZ_BAD_SAMPLE);
    CHECK("bad count increment",q.bad_count==1u);
    CHECK("accept BNO only",lkm_fc100hz_buffer_push(&q,&s1,100)==LKM_FC100HZ_OK);
    CHECK("one enqueued",q.count==1u);
    CHECK("duplicate BNO rejected",lkm_fc100hz_buffer_push(&q,&s1,101)==LKM_FC100HZ_DUPLICATE);
    CHECK("duplicate count",q.duplicate_count==1u);
    CHECK("BNO latest pop",lkm_fc100hz_buffer_take_latest(&q,101,&out)==LKM_FC100HZ_OK);
    CHECK("BNO numbers preserved",out.bno_counter==1u && out.roll_deg==3.0f);
    CHECK("absent gyro fields wiped",out.gyro_sensor_xyz_rad_s[0]==0.0f && !out.gyro_present);
    CHECK("never controller ready",!out.controller_ready);
    CHECK("buffer drained",q.count==0u);
    lkm_fc100hz_snapshot_t g=gyro(1,110);
    CHECK("accept gyro only",lkm_fc100hz_buffer_push(&q,&g,110)==LKM_FC100HZ_OK);
    CHECK("gyro latest pop",lkm_fc100hz_buffer_take_latest(&q,111,&out)==LKM_FC100HZ_OK);
    CHECK("gyro values preserved",out.gyro_sensor_xyz_rad_s[0]==0.5f);
    CHECK("absent BNO fields wiped",out.bno_counter==0u && out.roll_deg==0.0f && !out.bno_present);
    CHECK("gyro frame not body ready",!out.controller_ready);
    g.gyro_counter=2;g.gyro_host_received_ms=120;g.gyro_sensor_xyz_rad_s[2]=NAN;
    CHECK("nonfinite gyro reject",lkm_fc100hz_buffer_push(&q,&g,120)==LKM_FC100HZ_BAD_SAMPLE);
    g.gyro_sensor_xyz_rad_s[2]=41.0f;
    CHECK("out of bound gyro reject",lkm_fc100hz_buffer_push(&q,&g,120)==LKM_FC100HZ_BAD_SAMPLE);
    g.gyro_sensor_xyz_rad_s[2]=0.1f;
    CHECK("valid gyro again",lkm_fc100hz_buffer_push(&q,&g,120)==LKM_FC100HZ_OK);
    CHECK("duplicate gyro rejected",lkm_fc100hz_buffer_push(&q,&g,121)==LKM_FC100HZ_DUPLICATE);
    CHECK("gyro age20 inclusive",lkm_fc100hz_buffer_take_latest(&q,140,&out)==LKM_FC100HZ_OK);
    g.gyro_counter=3;g.gyro_host_received_ms=150;
    CHECK("queue fresh gyro",lkm_fc100hz_buffer_push(&q,&g,150)==LKM_FC100HZ_OK);
    out.controller_ready=true;
    CHECK("gyro age21 stale",lkm_fc100hz_buffer_take_latest(&q,171,&out)==LKM_FC100HZ_STALE);
    CHECK("stale pop zeros output",!out.controller_ready && !out.gyro_present && out.gyro_counter==0u);
    CHECK("stale count updated",q.stale_count==1u);
    lkm_fc100hz_snapshot_t b=bno(2,180);
    b.roll_deg=NAN;
    CHECK("NaN BNO reject",lkm_fc100hz_buffer_push(&q,&b,180)==LKM_FC100HZ_BAD_SAMPLE);
    b.roll_deg=181;
    CHECK("BNO roll reject",lkm_fc100hz_buffer_push(&q,&b,180)==LKM_FC100HZ_BAD_SAMPLE);
    b.roll_deg=0;b.pitch_deg=91;
    CHECK("BNO pitch reject",lkm_fc100hz_buffer_push(&q,&b,180)==LKM_FC100HZ_BAD_SAMPLE);
    b.pitch_deg=0;
    CHECK("BNO age50 inclusive",lkm_fc100hz_buffer_push(&q,&b,230)==LKM_FC100HZ_OK);
    CHECK("take at boundary",lkm_fc100hz_buffer_take_latest(&q,230,&out)==LKM_FC100HZ_OK);
    b.bno_counter=3;
    CHECK("BNO age51 rejected",lkm_fc100hz_buffer_push(&q,&b,231)==LKM_FC100HZ_STALE);
    b.bno_host_received_ms=240;
    CHECK("future BNO rejected",lkm_fc100hz_buffer_push(&q,&b,239)==LKM_FC100HZ_STALE);
    CHECK("future gyro rejected",lkm_fc100hz_buffer_push(&q,&g,149)==LKM_FC100HZ_STALE);
    b=bno(3,250); g=gyro(4,249);
    b.gyro_present=true;b.gyro_counter=g.gyro_counter;b.gyro_host_received_ms=g.gyro_host_received_ms;
    b.gyro_sensor_xyz_rad_s[0]=g.gyro_sensor_xyz_rad_s[0];
    CHECK("both IMU mock fields accepted",lkm_fc100hz_buffer_push(&q,&b,251)==LKM_FC100HZ_OK);
    CHECK("both IMU latest",lkm_fc100hz_buffer_take_latest(&q,252,&out)==LKM_FC100HZ_OK);
    CHECK("two-sensor still no PID",out.bno_present && out.gyro_present && !out.controller_ready);
    CHECK("timestamp real mock values",out.bno_host_received_ms==250u && out.gyro_host_received_ms==249u);
    CHECK("reinitialize clears queue",(lkm_fc100hz_buffer_init(&q),q.count==0u));
    /* Fill bound and prove safe full rejection, then skip older data. */
    for (uint32_t i=0;i<LKM_FC100HZ_BUFFER_CAPACITY;i++){
        b=bno(i+1u,1000u+i);
        CHECK("accept consecutive records",lkm_fc100hz_buffer_push(&q,&b,1010u)==LKM_FC100HZ_OK);
    }
    CHECK("ring capacity eight",q.count==8u);
    b=bno(9,1008);
    CHECK("full ring rejects ninth",lkm_fc100hz_buffer_push(&q,&b,1010)==LKM_FC100HZ_BUFFER_FULL);
    CHECK("full counter",q.overflow_count==1u);
    CHECK("full ring count unchanged",q.count==8u);
    CHECK("latest only take",lkm_fc100hz_buffer_take_latest(&q,1010,&out)==LKM_FC100HZ_OK);
    CHECK("latest seq eight",out.bno_counter==8u);
    CHECK("older seven discarded",q.dropped_older_on_latest_take==7u);
    CHECK("queue empty after take",q.count==0u);
    CHECK("rejected ninth can be retried",lkm_fc100hz_buffer_push(&q,&b,1010)==LKM_FC100HZ_OK);
    CHECK("retry accepted latest",lkm_fc100hz_buffer_take_latest(&q,1010,&out)==LKM_FC100HZ_OK && out.bno_counter==9u);
    lkm_fc100hz_buffer_init(&q);
    b=bno(UINT32_MAX,UINT32_MAX-5u);
    CHECK("wrap seed accepted",lkm_fc100hz_buffer_push(&q,&b,UINT32_MAX-4u)==LKM_FC100HZ_OK);
    b=bno(1,3u);
    CHECK("wrap counter accepted",lkm_fc100hz_buffer_push(&q,&b,4u)==LKM_FC100HZ_OK);
    CHECK("wrap latest read",lkm_fc100hz_buffer_take_latest(&q,5u,&out)==LKM_FC100HZ_OK && out.bno_counter==1u);
    CHECK("wrap still no PID",!out.controller_ready);
    CHECK("sample status text",strcmp(lkm_fc100hz_result_text(LKM_FC100HZ_STALE),"STALE")==0);
    printf("=== 100HZ SCHEDULER + BUFFER: %u/%u PASS ===\n",passed,passed);
    return 0;
}
