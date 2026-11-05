#ifndef LKM_FC100HZ_CORE_H
#define LKM_FC100HZ_CORE_H

/* HAL-independent, OBSERVATION-ONLY 10-ms cooperative scheduling monitor.
 * Not a hard real-time clock, interrupt controller, PID, or motor command path.
 * Uses unsigned 32-bit millisecond time on one timebase (HAL_GetTick or mock).
 * Every output frame is SENSOR-frame data; body axes remain unverified.
 */
#include <stdbool.h>
#include <stdint.h>

#define LKM_FC100HZ_PERIOD_MS             10u
#define LKM_FC100HZ_BUFFER_CAPACITY       8u
#define LKM_FC100HZ_BNO_MAX_AGE_MS        50u
#define LKM_FC100HZ_ICM_MAX_AGE_MS        20u
#define LKM_FC100HZ_MAX_SENSOR_RATE_RAD_S 40.0f

typedef enum {
    LKM_FC100HZ_OK = 0,
    LKM_FC100HZ_WAIT,
    LKM_FC100HZ_READY,
    LKM_FC100HZ_LATE_DROP,
    LKM_FC100HZ_OVERRUN,
    LKM_FC100HZ_FAULT,
    LKM_FC100HZ_BAD_ARGUMENT,
    LKM_FC100HZ_BAD_CONFIG,
    LKM_FC100HZ_NO_SAMPLE,
    LKM_FC100HZ_DUPLICATE,
    LKM_FC100HZ_STALE,
    LKM_FC100HZ_BAD_SAMPLE,
    LKM_FC100HZ_BUFFER_FULL
} lkm_fc100hz_result_t;

typedef struct {
    bool initialized;
    bool fault_latched;
    bool work_in_flight;
    uint32_t next_due_ms;
    uint32_t last_poll_ms;
    uint32_t last_dispatch_ms;
    uint32_t last_deadline_ms;
    uint32_t dispatch_count;
    uint32_t missed_slots;
    uint32_t late_events;
    uint32_t overrun_events;
    uint32_t max_lateness_ms;
    uint32_t max_work_ms_seen;
    uint32_t max_allowed_lateness_ms;
    uint32_t max_allowed_work_ms;
    lkm_fc100hz_result_t last_result;
    bool controller_ready; /* HARD FALSE; never an arming or PID grant. */
} lkm_fc100hz_scheduler_t;

/* start_ms: logical reset point. First deadline = start_ms+10 (wrap allowed).
 * Maximum allowed lateness and work must be < 10ms, nonzero.
 * A late/overrun/backward-time fault is LATCHED until explicit reinitialization.
 */
lkm_fc100hz_result_t lkm_fc100hz_sched_init(
    lkm_fc100hz_scheduler_t *ctx, uint32_t start_ms,
    uint32_t allowed_lateness_ms, uint32_t allowed_work_ms);

/* Call once per superloop iteration with a monotonic millisecond clock.
 * READY: caller may do diagnostic snapshot work, then MUST call finish once.
 * WAIT: not yet due. LATE_DROP/FAULT: skip work and remain fail-closed.
 * No catch-up bursts are ever generated.
 */
lkm_fc100hz_result_t lkm_fc100hz_sched_poll(
    lkm_fc100hz_scheduler_t *ctx, uint32_t now_ms);
lkm_fc100hz_result_t lkm_fc100hz_sched_finish(
    lkm_fc100hz_scheduler_t *ctx, uint32_t done_ms);

typedef struct {
    bool bno_present;
    bool gyro_present;
    uint32_t stored_at_ms;
    uint32_t bno_host_received_ms;
    uint32_t bno_counter;
    float roll_deg;
    float pitch_deg;
    uint32_t gyro_host_received_ms;
    uint32_t gyro_counter;
    float gyro_sensor_xyz_rad_s[3]; /* NOT body p/q/r! */
    bool controller_ready;          /* Always cleared on ingress and egress. */
} lkm_fc100hz_snapshot_t;

typedef struct {
    bool initialized;
    uint8_t head;
    uint8_t count;
    bool bno_counter_seen;
    bool gyro_counter_seen;
    uint32_t last_bno_counter;
    uint32_t last_gyro_counter;
    uint32_t accepted_count;
    uint32_t duplicate_count;
    uint32_t stale_count;
    uint32_t bad_count;
    uint32_t overflow_count;
    uint32_t dropped_older_on_latest_take;
    lkm_fc100hz_snapshot_t slots[LKM_FC100HZ_BUFFER_CAPACITY];
} lkm_fc100hz_buffer_t;

void lkm_fc100hz_buffer_init(lkm_fc100hz_buffer_t *ctx);
/* Accept only new, bounded, finite SENSOR-frame samples, with at least one
 * valid source. Host receipt stamps must be fresh and not in the future.
 * No IO and NO APPROVAL for PID even with two valid sensor sources.
 * Reject on full rather than silently dropping unseen data.
 */
lkm_fc100hz_result_t lkm_fc100hz_buffer_push(
    lkm_fc100hz_buffer_t *ctx, const lkm_fc100hz_snapshot_t *sample,
    uint32_t now_ms);
/* Read newest only and discard the older queued records to prevent a delayed
 * diagnostic consumer from processing outdated frames. No stale data leaks.
 * On failure, output is ZEROED and buffer is drained as appropriate.
 */
lkm_fc100hz_result_t lkm_fc100hz_buffer_take_latest(
    lkm_fc100hz_buffer_t *ctx, uint32_t now_ms,
    lkm_fc100hz_snapshot_t *out);
const char *lkm_fc100hz_result_text(lkm_fc100hz_result_t result);
#endif
