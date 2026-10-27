#include "robot_diagnostics.h"

#if APP_DIAGNOSTICS
#include "../subsystem2/motion_codec.h"
#include "../subsystem2/wheel_encoder.h"
#include <sys/sysdef.h>

/* Measurements and counters keep the full unsigned timer/count range. The
 * two conditions use bits. Counters saturate instead of silently wrapping.
 * Elapsed subtraction assumes consecutive calls are less than 2^32 us apart.
 * These diagnostic working views are not MQTT/inter-task wire formats. */
typedef struct {
    uint32_t samples;
    uint32_t publish_skips;
    uint32_t last_end_us;
    uint32_t gap_min_us;
    uint32_t gap_max_us;
    uint32_t exec_max_us;
    uint8_t gap_valid : 1;
} DIAG_SUMMARY;

typedef struct {
    DIAG_SUMMARY summary;
    uint32_t previous_start_us;
    uint8_t seen_start : 1;
} DIAG_PRODUCER;

static ID g_diagnostics_mutex;
/* The reporter never reads producer-owned state. A complete summary is
 * published under the mutex; a skipped publication loses no extrema. */
static DIAG_PRODUCER g_producer[ROBOT_DIAG_SOURCE_COUNT];
static DIAG_SUMMARY g_published[ROBOT_DIAG_SOURCE_COUNT];

static uint32_t increment_saturated(uint32_t value)
{
    return value == UINT32_MAX ? value : value + 1U;
}

ER robot_diagnostics_init(void)
{
    T_CMTX mutex = {0};
    ID id;
    unsigned i;
    if (g_diagnostics_mutex > 0) { return E_OBJ; }
    mutex.mtxatr = TA_INHERIT;
    id = tk_cre_mtx(&mutex);
    if (id < E_OK) { return (ER)id; }
    for (i = 0U; i < ROBOT_DIAG_SOURCE_COUNT; i++) {
        g_producer[i] = (DIAG_PRODUCER){0};
        g_published[i] = (DIAG_SUMMARY){0};
    }
    g_diagnostics_mutex = id;
    return E_OK;
}

uint32_t robot_diagnostics_now_us(void)
{
    return in_w(TIMER_TIMERAWL);
}

void robot_diagnostics_record(unsigned source, uint32_t started_us,
                              uint32_t finished_us)
{
    DIAG_PRODUCER *producer;
    DIAG_SUMMARY *summary;
    uint32_t execution;
    if (source >= ROBOT_DIAG_SOURCE_COUNT || g_diagnostics_mutex <= 0) {
        return;
    }
    producer = &g_producer[source];
    summary = &producer->summary;
    execution = finished_us - started_us;
    if (producer->seen_start) {
        uint32_t gap = started_us - producer->previous_start_us;
        if (!summary->gap_valid || gap < summary->gap_min_us) {
            summary->gap_min_us = gap;
        }
        if (!summary->gap_valid || gap > summary->gap_max_us) {
            summary->gap_max_us = gap;
        }
        summary->gap_valid = 1U;
    }
    producer->seen_start = 1U;
    producer->previous_start_us = started_us;
    summary->samples = increment_saturated(summary->samples);
    summary->last_end_us = finished_us;
    if (execution > summary->exec_max_us) { summary->exec_max_us = execution; }

    /* No high-priority producer waits for a diagnostic reporter. Its private
     * summary continues collecting every call even while publication is busy.
     * The next success includes intervening measurements and skip count. */
    if (tk_loc_mtx(g_diagnostics_mutex, TMO_POL) != E_OK) {
        summary->publish_skips = increment_saturated(summary->publish_skips);
        return;
    }
    g_published[source] = *summary;
    (void)tk_unl_mtx(g_diagnostics_mutex);
}

static bool estimator_mms(float meters_per_second, int32_t *out)
{
    float scaled = meters_per_second * 1000.0f;
    /* Check before conversion: NaN, infinity and out-of-int32 estimates must
     * not trigger undefined behaviour. Numeric validity is not calibration. */
    if (scaled != scaled || scaled >= 2147483648.0f ||
        scaled < -2147483648.0f) { return false; }
    *out = (int32_t)scaled;
    return true;
}

static void report_motion_observation(void)
{
    motion_snapshot_t motion;
    int32_t left = enc_l_count();
    int32_t right = enc_r_count();
    uint32_t glitches_left = wheel_encoder_glitches(WHEEL_LEFT);
    uint32_t glitches_right = wheel_encoder_glitches(WHEEL_RIGHT);
    bool valid = motion_read_snapshot(&motion) && motion_snapshot_valid(&motion);
    int32_t left_mms = 0, right_mms = 0;
    bool left_numeric_valid = estimator_mms(wheel_speed_left_mps(), &left_mms);
    bool right_numeric_valid = estimator_mms(wheel_speed_right_mps(), &right_mms);

    /* Keep individual tm_printf calls short: the inherited console formatter
     * masks local interrupts. These are still observer-overhead measurements. */
    tm_printf((const UB *)"[DIAG] encoder=PIO glitches_available=0\n");
    tm_printf((const UB *)"[DIAG] counts left=%d right=%d glitches_left=%u glitches_right=%u\n",
        (int)left, (int)right, (unsigned)glitches_left, (unsigned)glitches_right);
    if (valid) {
        tm_printf((const UB *)"[DIAG] motion snapshot_valid=1 state=%u fault=%u busy=%u snapshot_speed_valid=%u\n",
            (unsigned)motion_snapshot_state(&motion),
            (unsigned)motion_snapshot_fault(&motion),
            (unsigned)motion_snapshot_busy(&motion),
            (unsigned)motion_snapshot_speed_valid(&motion));
    } else {
        tm_printf((const UB *)"[DIAG] motion snapshot_valid=0\n");
    }
    tm_printf((const UB *)"[DIAG] estimate left_valid=%u left_mms=%d right_valid=%u right_mms=%d\n",
        (unsigned)left_numeric_valid, (int)left_mms,
        (unsigned)right_numeric_valid, (int)right_mms);
}

void robot_diagnostics_report(void)
{
    DIAG_SUMMARY snapshot[ROBOT_DIAG_SOURCE_COUNT];
    unsigned i;
    if (g_diagnostics_mutex <= 0 ||
        tk_loc_mtx(g_diagnostics_mutex, TMO_POL) != E_OK) { return; }
    for (i = 0U; i < ROBOT_DIAG_SOURCE_COUNT; i++) {
        snapshot[i] = g_published[i];
    }
    (void)tk_unl_mtx(g_diagnostics_mutex);

    for (i = 0U; i < ROBOT_DIAG_SOURCE_COUNT; i++) {
        const DIAG_SUMMARY *p = &snapshot[i];
        tm_printf((const UB *)"[DIAG] timing source=%s samples=%u publish_skips=%u last_end_us=%u\n",
            i == ROBOT_DIAG_MOTION ? "motion" : "ir",
            (unsigned)p->samples, (unsigned)p->publish_skips,
            (unsigned)p->last_end_us);
        tm_printf((const UB *)"[DIAG] interval source=%s gap_valid=%u gap_min_us=%u gap_max_us=%u exec_max_us=%u\n",
            i == ROBOT_DIAG_MOTION ? "motion" : "ir", (unsigned)p->gap_valid,
            (unsigned)p->gap_min_us, (unsigned)p->gap_max_us,
            (unsigned)p->exec_max_us);
    }
    /* The existing reporting API does not consume MOTION_COMPLETE. Invalid
     * speed fields are not evidence that a wheel is physically stationary. */
    report_motion_observation();
}
#endif
