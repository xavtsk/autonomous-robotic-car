/** @file robot_diagnostics.c
 * @brief Collect and report optional motion and IR timing observations over
 * USB.
 * Unsigned casts spell out the original modulo conversion.
 */
#include "robot_diagnostics.h"
#include "../app_types.h"

#if APP_DIAGNOSTICS
#include "../subsystem2/codec_motion.h"
#include "../subsystem2/wheel_encoder.h"
#include <sys/sysdef.h>
#ifdef ROBOT_DIAGNOSTICS_TEST
#include "firmware_main.h"
#endif

/* Measurements and counters keep the full unsigned timer/count range. The
 * two conditions use bits. Counters saturate instead of silently wrapping.
 * Elapsed subtraction assumes consecutive calls are less than 2^32 us apart.
 * These diagnostic working views are not MQTT/inter-task wire formats. */
typedef struct
{
    uint32_t samples;
    uint32_t publish_skips;
    uint32_t last_end_us;
    uint32_t gap_min_us;
    uint32_t gap_max_us;
    uint32_t exec_max_us;
    uint8_t  b_is_gap_valid : 1;
} diag_summary_t;

typedef struct
{
    diag_summary_t summary;
    uint32_t       previous_start_us;
    uint8_t        b_has_seen_start : 1;
} diag_producer_t;

static ID gh_diagnostics_mutex;
/* The reporter never reads producer-owned state. A complete summary is
 * published under the mutex; a skipped publication loses no extrema. */
static diag_producer_t g_producer[ROBOT_DIAG_SOURCE_COUNT];
static diag_summary_t  g_published[ROBOT_DIAG_SOURCE_COUNT];

static uint32_t increment_saturated (uint32_t value);

static bool estimator_mms (float32_t meters_per_second, int32_t * p_out);

static void report_motion_observation (void);

/** @brief Create the publication mutex and clear all producer and published
 * summaries. Return E_OBJ if already initialized or the kernel error
 * if mutex creation fails.
 */
ER robot_diagnostics_init (void)
{
    T_CMTX   mutex = {0};
    ID       h_mutex;
    unsigned source_index;

    if (gh_diagnostics_mutex > 0)
    {
        return E_OBJ;
    }

    mutex.mtxatr = TA_INHERIT;
    h_mutex      = tk_cre_mtx(&mutex);

    if (h_mutex < E_OK)
    {
        return (ER)h_mutex;
    }

    for (source_index = 0U; source_index < (unsigned)ROBOT_DIAG_SOURCE_COUNT;
         source_index++)
    {
        g_producer[source_index]  = (diag_producer_t){0};
        g_published[source_index] = (diag_summary_t){0};
    }

    gh_diagnostics_mutex = h_mutex;
    return E_OK;
}

/** @brief Read the low 32 bits of the RP2040 microsecond timer for unsigned
 * elapsed-time measurements.
 */
uint32_t robot_diagnostics_now_us (void)
{
    return in_w(TIMER_TIMERAWL);
}

/** @brief Record one producer's wrapping start/end times in microseconds. Each
 * source has one writer. A failed mutex poll counts a skipped
 * publication while retaining the private measurements for the next
 * successful poll.
 */
void robot_diagnostics_record (unsigned source, uint32_t started_us,
                               uint32_t finished_us)
{
    diag_producer_t * p_producer = NULL;
    diag_summary_t *  p_summary  = NULL;
    uint32_t          execution;

    if ((source >= (unsigned)ROBOT_DIAG_SOURCE_COUNT) ||
        (gh_diagnostics_mutex <= 0))
    {
        return;
    }

    p_producer = &g_producer[source];
    p_summary  = &p_producer->summary;
    execution  = finished_us - started_us;

    if (p_producer->b_has_seen_start)
    {
        uint32_t gap = started_us - p_producer->previous_start_us;

        if ((!p_summary->b_is_gap_valid) || (gap < p_summary->gap_min_us))
        {
            p_summary->gap_min_us = gap;
        }

        if ((!p_summary->b_is_gap_valid) || (gap > p_summary->gap_max_us))
        {
            p_summary->gap_max_us = gap;
        }

        p_summary->b_is_gap_valid = 1U;
    }

    p_producer->b_has_seen_start  = 1U;
    p_producer->previous_start_us = started_us;
    p_summary->samples            = increment_saturated(p_summary->samples);
    p_summary->last_end_us        = finished_us;

    if (execution > p_summary->exec_max_us)
    {
        p_summary->exec_max_us = execution;
    }

    /* No high-priority producer waits for a diagnostic reporter. Its private
     * summary continues collecting every call even while publication is busy.
     * The next success includes intervening measurements and skip count. */
    if (tk_loc_mtx(gh_diagnostics_mutex, TMO_POL) != E_OK)
    {
        p_summary->publish_skips =
            increment_saturated(p_summary->publish_skips);
        return;
    }

    g_published[source] = *p_summary;
    (void)tk_unl_mtx(gh_diagnostics_mutex);
}

/** @brief Poll the mutex and copy the published timing summaries. Print them
 * and read the motion observations after unlocking. An unavailable
 * mutex defers the report.
 */
void robot_diagnostics_report (void)
{
    diag_summary_t snapshot[ROBOT_DIAG_SOURCE_COUNT];
    unsigned       source_index;

    if ((gh_diagnostics_mutex <= 0) ||
        (tk_loc_mtx(gh_diagnostics_mutex, TMO_POL) != E_OK))
    {
        return;
    }

    for (source_index = 0U; source_index < (unsigned)ROBOT_DIAG_SOURCE_COUNT;
         source_index++)
    {
        snapshot[source_index] = g_published[source_index];
    }

    (void)tk_unl_mtx(gh_diagnostics_mutex);

    for (source_index = 0U; source_index < (unsigned)ROBOT_DIAG_SOURCE_COUNT;
         source_index++)
    {
        const diag_summary_t * p_summary = &snapshot[source_index];
        tm_printf((const UB *)"[DIAG] timing source=%s samples=%u "
                              "publish_skips=%u last_end_us=%u\n",
                  (unsigned)ROBOT_DIAG_MOTION == source_index ? "motion" : "ir",
                  (unsigned)p_summary->samples,
                  (unsigned)p_summary->publish_skips,
                  (unsigned)p_summary->last_end_us);
        tm_printf((const UB *)"[DIAG] interval source=%s gap_valid=%u "
                              "gap_min_us=%u gap_max_us=%u exec_max_us=%u\n",
                  (unsigned)ROBOT_DIAG_MOTION == source_index ? "motion" : "ir",
                  (unsigned)p_summary->b_is_gap_valid,
                  (unsigned)p_summary->gap_min_us,
                  (unsigned)p_summary->gap_max_us,
                  (unsigned)p_summary->exec_max_us);
    }
    /* The existing reporting API does not consume MOTION_COMPLETE. Invalid
     * speed fields are not evidence that a wheel is physically stationary. */
    report_motion_observation();
}

#ifdef ROBOT_DIAGNOSTICS_TEST
/** @brief Copy published observations while the fixture is single-threaded. */
robot_diagnostics_test_summary_t
robot_diagnostics_test_published (robot_diagnostics_source_t source)
{
    const diag_summary_t * p_summary = &g_published[source];
    return (robot_diagnostics_test_summary_t){
        .samples        = p_summary->samples,
        .publish_skips  = p_summary->publish_skips,
        .gap_min_us     = p_summary->gap_min_us,
        .gap_max_us     = p_summary->gap_max_us,
        .exec_max_us    = p_summary->exec_max_us,
        .b_is_gap_valid = p_summary->b_is_gap_valid,
    };
}

/** @brief Return the actual producer, published and total storage sizes. */
robot_diagnostics_test_layout_t robot_diagnostics_test_layout (void)
{
    return (robot_diagnostics_test_layout_t){
        .producer_bytes  = sizeof(diag_producer_t),
        .published_bytes = sizeof(diag_summary_t),
        .total_bytes     = sizeof(g_producer) + sizeof(g_published),
    };
}

/** @brief Apply the same saturation rule used by the diagnostic counters. */
uint32_t robot_diagnostics_test_increment (uint32_t value)
{
    return increment_saturated(value);
}

/** @brief Apply the reporter's checked conversion from m/s to signed mm/s. */
bool robot_diagnostics_test_estimate (float32_t speed_mps, int32_t * p_out)
{
    return estimator_mms(speed_mps, p_out);
}
#endif

/** @brief Increment an unsigned measurement count up to UINT32_MAX. Once
 * saturated, retain the maximum value.
 */
static uint32_t increment_saturated (uint32_t value)
{
    return UINT32_MAX == value ? value : value + 1U;
}

/** @brief Convert metres per second to whole signed millimetres per second by
 * truncation. Reject NaN, infinity and values outside the conversion
 * range before writing the caller's result.
 */
static bool estimator_mms (float32_t meters_per_second, int32_t * p_out)
{
    float32_t scaled = meters_per_second * 1000.0f;
    /* Check before conversion: NaN, infinity and out-of-int32 estimates must
     * not trigger undefined behaviour. Numeric validity is not calibration. */
    if (((scaled != scaled) || (scaled >= 2147483648.0f)) ||
        (scaled < -2147483648.0f))
    {
        return false;
    }

    *p_out = (int32_t)scaled;
    return true;
}

/** @brief Print sequential encoder, motion snapshot and speed estimator
 * observations. Reading the motion snapshot leaves the completion
 * queue available to its consumer.
 */
static void report_motion_observation (void)
{
    codec_motion_snapshot_t motion;
    int32_t                 left          = enc_l_count();
    int32_t                 right         = enc_r_count();
    uint32_t                glitches_left = wheel_encoder_glitches(WHEEL_LEFT);
    uint32_t glitches_right               = wheel_encoder_glitches(WHEEL_RIGHT);
    bool     b_is_snapshot_valid =
        (snapshot_motion_read(&motion)) && (codec_motion_valid(&motion));
    int32_t left_mms  = 0;
    int32_t right_mms = 0;
    bool    b_is_left_value_valid =
        estimator_mms(motion_wheel_left_speed_mps(), &left_mms);
    bool b_is_right_value_valid =
        estimator_mms(motion_wheel_right_speed_mps(), &right_mms);

    /* Keep individual tm_printf calls short: the inherited console formatter
     * masks local interrupts. These are still observer-overhead measurements.
     */
    tm_printf((const UB *)"[DIAG] encoder=PIO glitches_available=0\n");
    tm_printf((const UB *)"[DIAG] counts left=%d right=%d glitches_left=%u "
                          "glitches_right=%u\n",
              (int)left, (int)right, (unsigned)glitches_left,
              (unsigned)glitches_right);

    if (b_is_snapshot_valid)
    {
        tm_printf((const UB *)"[DIAG] motion snapshot_valid=1 state=%u "
                              "fault=%u busy=%u snapshot_speed_valid=%u\n",
                  (unsigned)codec_motion_state(&motion),
                  (unsigned)codec_motion_fault(&motion),
                  (unsigned)codec_motion_busy(&motion),
                  (unsigned)codec_motion_speed_valid(&motion));
    }
    else
    {
        tm_printf((const UB *)"[DIAG] motion snapshot_valid=0\n");
    }

    tm_printf((const UB *)"[DIAG] estimate left_valid=%u left_mms=%d "
                          "right_valid=%u right_mms=%d\n",
              (unsigned)b_is_left_value_valid, (int)left_mms,
              (unsigned)b_is_right_value_valid, (int)right_mms);
}
#endif

/* End of file. */

