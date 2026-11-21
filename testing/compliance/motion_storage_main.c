/** @file motion_storage_main.c
 * @brief Characterize retained motion values through the unchanged wide APIs.
 *
 * Use the candidate fixture against either subject checkout. Compile with
 * -I SUBJECT -I SUBJECT/app_program/subsystem2 -I testing/subsystem3 and link
 * ctrl_motion.c, motion_pack.c, motion_estimation.c and speed_pid.c from
 * SUBJECT. Link motion.c with host-only probes to seed otherwise unreachable
 * boundary values. Exercise real motion creation, initialization, processing
 * and read calls for both revisions.
 */
#include "motion_storage_main.h"
#include "../../app_program/app_types.h"
#include "test_motion_feedback_main.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>

static bool     gb_should_fail_lock;
static unsigned g_lock_calls;
static unsigned g_unlock_calls;

/** @brief Count a fake mutex attempt and optionally inject its failure. */
bool motion_storage_allow_lock (bool b_is_held)
{
    g_lock_calls++;
    if (gb_should_fail_lock)
    {
        assert(!b_is_held);
        return false;
    }
    return true;
}

/** @brief Count releases before the fake kernel checks ownership. */
void motion_storage_observe_unlock (void)
{
    g_unlock_calls++;
}

#define CALIBRATION_FLAG_CASES (512U)
#define FEEDBACK_FLAG_CASES (8U)

static unsigned g_cases;

/** @brief Copy a binary32 value into an unsigned word for exact trace
 * output.
 */
static uint32_t float_word (float32_t value)
{
    uint32_t word;
    memcpy(&word, &value, sizeof(word));
    return word;
}

/** @brief Copy a binary64 value into an unsigned word for exact trace
 * output.
 */
static uint64_t double_word (float64_t value)
{
    uint64_t word;
    memcpy(&word, &value, sizeof(word));
    return word;
}

/** @brief Encode the fixture's calibration predicates into a trace bit mask.
 */
static unsigned calibration_flags (const motion_calibration_t * p_calibration)
{
    return (unsigned)p_calibration->b_is_calibration_valid |
           ((unsigned)p_calibration->b_is_left_motor_inverted << 1) |
           ((unsigned)p_calibration->b_is_right_motor_inverted << 2) |
           ((unsigned)p_calibration->b_is_left_enc_inverted << 3) |
           ((unsigned)p_calibration->b_is_right_enc_inverted << 4) |
           ((unsigned)p_calibration->b_is_left_enc_ok << 5) |
           ((unsigned)p_calibration->b_is_right_enc_ok << 6) |
           ((unsigned)p_calibration->b_are_encoders_swapped << 7) |
           ((unsigned)p_calibration->b_was_aborted << 8);
}

/** @brief Read injected calibration results over boundary bit patterns and
 * compare retained flags and floating-point representations.
 */
static void test_calibration_storage (void)
{
    static const uint32_t words[] = {
        UINT32_C(0),          UINT32_C(0x80000000), UINT32_C(0x00000001),
        UINT32_C(0x7f7fffff), UINT32_C(0x7fc01234), UINT32_C(0xc5abcdef)};
    for (unsigned flags = 0U; flags < CALIBRATION_FLAG_CASES; ++flags)
    {
        motion_calibration_t expected = {0};
        motion_calibration_t actual;
        float32_t *          fields[] = {
            &expected.counts_per_rev,   &expected.max_speed_mps,
            &expected.counts_per_rev_l, &expected.counts_per_rev_r,
            &expected.top_speed_l_mps,  &expected.top_speed_r_mps};
        expected.b_is_calibration_valid    = (flags & 1U) != 0U;
        expected.b_is_left_motor_inverted  = (flags & 2U) != 0U;
        expected.b_is_right_motor_inverted = (flags & 4U) != 0U;
        expected.b_is_left_enc_inverted    = (flags & 8U) != 0U;
        expected.b_is_right_enc_inverted   = (flags & 16U) != 0U;
        expected.b_is_left_enc_ok          = (flags & 32U) != 0U;
        expected.b_is_right_enc_ok         = (flags & 64U) != 0U;
        expected.b_are_encoders_swapped    = (flags & 128U) != 0U;
        expected.b_was_aborted             = (flags & 256U) != 0U;
        for (unsigned index = 0U; index < (sizeof fields / sizeof fields[0]);
             ++index)
        {
            memcpy(fields[index], &words[(flags + index) % 6U],
                   sizeof(float32_t));
        }
        motion_storage_test_seed_calibration(&expected);
        memset(&actual, 0xa5, sizeof(actual));
        unsigned locks_before   = g_lock_calls;
        unsigned unlocks_before = g_unlock_calls;
        assert(motion_get_calibration(&actual));
        assert(!motion_fixture_locked() && g_lock_calls == locks_before + 1U &&
               g_unlock_calls == unlocks_before + 1U);
        assert(calibration_flags(&actual) == flags);
        const float32_t actual_fields[] = {
            actual.counts_per_rev,   actual.max_speed_mps,
            actual.counts_per_rev_l, actual.counts_per_rev_r,
            actual.top_speed_l_mps,  actual.top_speed_r_mps};
        printf("MSTORE CAL %03x", flags);
        for (unsigned index = 0U; index < (sizeof fields / sizeof fields[0]);
             ++index)
        {
            assert(float_word(actual_fields[index]) ==
                   float_word(*fields[index]));
            printf(" %08" PRIx32, float_word(actual_fields[index]));
        }
        putchar('\n');
        g_cases++;
    }
}

/** @brief Inject unavailable motion storage and lock failures and check
 * whether caller-provided outputs retain their prior bytes.
 */
static void test_failure_outputs (void)
{
    motion_feedback_t       feedback;
    motion_feedback_t       feedback_before;
    motion_debug_snapshot_t debug;
    motion_debug_snapshot_t debug_before;
    motion_calibration_t    calibration;
    motion_calibration_t    calibration_before;
    memset(&feedback, 0xa5, sizeof(feedback));
    memset(&debug, 0x5a, sizeof(debug));
    memset(&calibration, 0x3c, sizeof(calibration));
    memcpy(&feedback_before, &feedback, sizeof(feedback));
    memcpy(&debug_before, &debug, sizeof(debug));
    memcpy(&calibration_before, &calibration, sizeof(calibration));
    const ID h_mutex_before = motion_storage_test_exchange_mutex(0);
    assert(!motion_get_feedback(&feedback));
    assert(!motion_try_debug_snapshot(&debug));
    assert(!motion_get_feedback(NULL) && !motion_try_debug_snapshot(NULL));
    assert(memcmp(&feedback, &feedback_before, sizeof(feedback)) == 0);
    assert(memcmp(&debug, &debug_before, sizeof(debug)) == 0);
    (void)motion_storage_test_exchange_mutex(h_mutex_before);
    gb_should_fail_lock     = true;
    unsigned unlocks_before = g_unlock_calls;
    assert(!motion_get_feedback(&feedback));
    assert(!motion_try_debug_snapshot(&debug));
    assert(!motion_fixture_locked() && g_unlock_calls == unlocks_before);
    assert(memcmp(&feedback, &feedback_before, sizeof(feedback)) == 0);
    assert(memcmp(&debug, &debug_before, sizeof(debug)) == 0);
    gb_should_fail_lock = false;
    motion_storage_test_clear_calibration();
    assert(!motion_get_calibration(&calibration));
    assert(memcmp(&calibration, &calibration_before, sizeof(calibration)) == 0);
    assert(!motion_fixture_locked());
    puts("MSTORE FAILURE outputs-preserved lock-balanced");
    g_cases++;
}

/** @brief Exercise the feedback clock, epoch, distance and flag boundaries
 * and emit exact values for the baseline comparison.
 */
static void test_feedback_storage (void)
{
    static const uint32_t clocks[]  = {0U,
                                       1U,
                                       UINT32_C(0x7fffffff),
                                       UINT32_C(0x80000000),
                                       UINT32_MAX - 1U,
                                       UINT32_MAX};
    static const uint64_t centres[] = {UINT64_C(0),
                                       UINT64_C(0x8000000000000000),
                                       UINT64_C(1),
                                       UINT64_C(0x7fefffffffffffff),
                                       UINT64_C(0x7ff8000000001234),
                                       UINT64_C(0xc0fabcde12345678)};
    motion_init();
    motion_fixture_enable_encoder();
    for (unsigned flags = 0U; flags < FEEDBACK_FLAG_CASES; ++flags)
    {
        for (unsigned stamp = 0U; stamp < (sizeof clocks / sizeof clocks[0]);
             ++stamp)
        {
            for (unsigned epoch = 0U;
                 epoch < (sizeof clocks / sizeof clocks[0]); ++epoch)
            {
                motion_feedback_t       feedback;
                motion_debug_snapshot_t debug;
                uint32_t                losses = clocks[(stamp + epoch) % 6U];
                uint64_t                centre = centres[(stamp + epoch) % 6U];
                motion_storage_test_seed_feedback(clocks[stamp], clocks[epoch],
                                                  centre, losses, flags);
                unsigned locks_before   = g_lock_calls;
                unsigned unlocks_before = g_unlock_calls;
                assert(motion_get_feedback(&feedback));
                assert(motion_try_debug_snapshot(&debug));
                assert(!motion_fixture_locked() &&
                       g_lock_calls == locks_before + 2U &&
                       g_unlock_calls == unlocks_before + 2U);
                assert(feedback.timestamp_us == clocks[stamp] &&
                       feedback.epoch == clocks[epoch] &&
                       feedback.report_loss_count == losses &&
                       double_word(feedback.centre_m) == centre);
                assert(feedback.b_is_feedback_valid == ((flags & 1U) != 0U));
                assert(debug.timestamp_us == clocks[stamp]);
                assert(debug.b_is_debug_valid == ((flags & 3U) == 3U));
                assert((motion_state_t)MOTION_IDLE == feedback.state &&
                       (motion_fault_t)MOTION_FAULT_NONE == feedback.fault);
                assert(debug.state == feedback.state &&
                       debug.fault == feedback.fault);
                assert(debug.b_is_braked == feedback.b_is_braked &&
                       !debug.b_is_debug_busy);
                printf("MSTORE FB %u %08" PRIx32 " %08" PRIx32 " %08" PRIx32
                       " %016" PRIx64 " %u %u\n",
                       flags, feedback.timestamp_us, feedback.epoch,
                       feedback.report_loss_count,
                       double_word(feedback.centre_m),
                       feedback.b_is_feedback_valid, debug.b_is_debug_valid);
                g_cases++;
            }
        }
    }
}

/** @brief Check clock, epoch and report-loss wraparound and lease behavior
 * using the seeded motion state.
 */
static void test_feedback_wrap (void)
{
    motion_feedback_t feedback;
    motion_storage_test_seed_feedback(UINT32_MAX, UINT32_MAX, 0U, UINT32_MAX,
                                      3U);
    motion_fixture_set_time(UINT32_MAX - 4999U);
    motion_init();
    assert(motion_get_feedback(&feedback));
    assert(0U == feedback.epoch && UINT32_MAX == feedback.timestamp_us &&
           !feedback.b_is_feedback_valid);
    motion_fixture_set_time(in_w(TIMER_TIMERAWL) + 10000U);
    motion_process();
    assert(motion_get_feedback(&feedback));
    assert(0U == feedback.epoch && 5000U == feedback.timestamp_us &&
           feedback.b_is_feedback_valid);
    assert(double_word(feedback.centre_m) == 0U);
    const motion_report_t report = {255U, MOTION_OUTCOME_CANCELLED,
                                    MOTION_FAULT_NONE, false};
    motion_storage_test_post_to_full(motion_fixture_fill_reports, &report);
    assert(motion_get_feedback(&feedback) && 0U == feedback.report_loss_count);
    motion_navigation_heartbeat(true);
    assert(motion_request_calibration() == (motion_result_t)MOTION_ERR_BUSY);
    motion_navigation_heartbeat(false);
    assert(motion_request_calibration() == (motion_result_t)MOTION_OK);
    assert(motion_get_feedback(&feedback) && !feedback.b_is_feedback_valid);
    puts("MSTORE WRAP clock epoch report-loss lease-exclusion");
    g_cases++;
}

/** @brief Run the motion fixture and inspect retained calibration, feedback
 * and failure-output values through the public interfaces.
 */
int main (void)
{
    assert(motion_fixture_run() == 0);
    test_calibration_storage();
    test_failure_outputs();
    test_feedback_storage();
    test_feedback_wrap();
    size_t sizes[3];
    motion_storage_test_sizes(sizes);
    fprintf(stderr,
            "Retained sizes: calibration=%zu feedback=%zu flags=%zu; wide "
            "calibration=%zu feedback=%zu\n",
            sizes[0], sizes[1], sizes[2], sizeof(motion_calibration_t),
            sizeof(motion_feedback_t));
    printf("PASS: %u motion storage characterization cases\n", g_cases);
    return 0;
}

/* End of file. */

