/** @file motion_storage_test.c
 * @brief Characterize retained motion values through the unchanged wide APIs.
 *
 * Use the candidate fixture against either subject checkout. Compile with
 * -I SUBJECT -I SUBJECT/app_program/subsystem2 -I testing/subsystem3 and link
 * ctrl_motion.c, motion_pack.c, motion_estimation.c and speed_pid.c from SUBJECT.
 * motion.c is included here so tests can seed otherwise unreachable boundary
 * values. Real motion_create/init/process/getter logic is still exercised.
 * Define MOTION_COMPACT_RETAINED only for the future private-storage adapter.
 */
#define main motion_existing_test_suite
#define tk_loc_mtx motion_existing_loc_mtx
#define tk_unl_mtx motion_existing_unl_mtx
#include "test_motion_feedback.c"
#undef tk_unl_mtx
#undef tk_loc_mtx
#undef main

#include <inttypes.h>

static bool g_fail_lock;
static unsigned g_lock_calls;
static unsigned g_unlock_calls;

ER tk_loc_mtx(ID id, int timeout)
{
    g_lock_calls++;
    if (g_fail_lock)
    {
        assert(!locked);
        return -1;
    }
    return motion_existing_loc_mtx(id, timeout);
}

ER tk_unl_mtx(ID id)
{
    g_unlock_calls++;
    return motion_existing_unl_mtx(id);
}

#include "app_program/subsystem2/motion.c"

static unsigned g_cases;

static uint32_t float_word(float value)
{
    uint32_t word;
    memcpy(&word, &value, sizeof(word));
    return word;
}

static uint64_t double_word(double value)
{
    uint64_t word;
    memcpy(&word, &value, sizeof(word));
    return word;
}

static unsigned calibration_flags(const motion_calibration_t *value)
{
    return (unsigned)value->b_valid | ((unsigned)value->b_m1_inverted << 1) |
           ((unsigned)value->b_m2_inverted << 2) |
           ((unsigned)value->b_enc_l_inverted << 3) |
           ((unsigned)value->b_enc_r_inverted << 4) |
           ((unsigned)value->b_enc_l_ok << 5) |
           ((unsigned)value->b_enc_r_ok << 6) |
           ((unsigned)value->b_encoders_swapped << 7) |
           ((unsigned)value->b_aborted << 8);
}

static void seed_calibration(const motion_calibration_t *value)
{
    motion_lock();
#ifdef MOTION_COMPACT_RETAINED
    motion_calibration_store(&g_cal_result, value);
#else
    g_cal_result = *value;
#endif
    g_flags.b_cal_done = 1U;
    motion_unlock();
}

static void test_calibration_storage(void)
{
    static const uint32_t words[] = {
        UINT32_C(0), UINT32_C(0x80000000), UINT32_C(0x00000001),
        UINT32_C(0x7f7fffff), UINT32_C(0x7fc01234), UINT32_C(0xc5abcdef)};
    for (unsigned flags = 0U; flags < 512U; ++flags)
    {
        motion_calibration_t expected = {0}, actual;
        float *fields[] = {&expected.counts_per_rev, &expected.max_speed_mps,
            &expected.counts_per_rev_l, &expected.counts_per_rev_r,
            &expected.top_speed_l_mps, &expected.top_speed_r_mps};
        expected.b_valid = (flags & 1U) != 0U;
        expected.b_m1_inverted = (flags & 2U) != 0U;
        expected.b_m2_inverted = (flags & 4U) != 0U;
        expected.b_enc_l_inverted = (flags & 8U) != 0U;
        expected.b_enc_r_inverted = (flags & 16U) != 0U;
        expected.b_enc_l_ok = (flags & 32U) != 0U;
        expected.b_enc_r_ok = (flags & 64U) != 0U;
        expected.b_encoders_swapped = (flags & 128U) != 0U;
        expected.b_aborted = (flags & 256U) != 0U;
        for (unsigned index = 0U; index < 6U; ++index)
        {
            memcpy(fields[index], &words[(flags + index) % 6U], sizeof(float));
        }
        seed_calibration(&expected);
        memset(&actual, 0xa5, sizeof(actual));
        unsigned locks_before = g_lock_calls, unlocks_before = g_unlock_calls;
        assert(motion_get_calibration(&actual));
        assert(!locked && g_lock_calls == locks_before + 1U &&
               g_unlock_calls == unlocks_before + 1U);
        assert(calibration_flags(&actual) == flags);
        const float actual_fields[] = {actual.counts_per_rev, actual.max_speed_mps,
            actual.counts_per_rev_l, actual.counts_per_rev_r,
            actual.top_speed_l_mps, actual.top_speed_r_mps};
        printf("MSTORE CAL %03x", flags);
        for (unsigned index = 0U; index < 6U; ++index)
        {
            assert(float_word(actual_fields[index]) == float_word(*fields[index]));
            printf(" %08" PRIx32, float_word(actual_fields[index]));
        }
        putchar('\n');
        g_cases++;
    }
}

static void test_failure_outputs(void)
{
    motion_feedback_t feedback, feedback_before;
    motion_debug_snapshot_t debug, debug_before;
    motion_calibration_t calibration, calibration_before;
    ID mutex_before = g_mutex_id;
    memset(&feedback, 0xa5, sizeof(feedback));
    memset(&debug, 0x5a, sizeof(debug));
    memset(&calibration, 0x3c, sizeof(calibration));
    memcpy(&feedback_before, &feedback, sizeof(feedback));
    memcpy(&debug_before, &debug, sizeof(debug));
    memcpy(&calibration_before, &calibration, sizeof(calibration));
    g_mutex_id = 0;
    assert(!motion_get_feedback(&feedback));
    assert(!motion_try_debug_snapshot(&debug));
    assert(!motion_get_feedback(NULL) && !motion_try_debug_snapshot(NULL));
    assert(memcmp(&feedback, &feedback_before, sizeof(feedback)) == 0);
    assert(memcmp(&debug, &debug_before, sizeof(debug)) == 0);
    g_mutex_id = mutex_before;
    g_fail_lock = true;
    unsigned unlocks_before = g_unlock_calls;
    assert(!motion_get_feedback(&feedback));
    assert(!motion_try_debug_snapshot(&debug));
    assert(!locked && g_unlock_calls == unlocks_before);
    assert(memcmp(&feedback, &feedback_before, sizeof(feedback)) == 0);
    assert(memcmp(&debug, &debug_before, sizeof(debug)) == 0);
    g_fail_lock = false;
    motion_lock();
    g_flags.b_cal_done = 0U;
    motion_unlock();
    assert(!motion_get_calibration(&calibration));
    assert(memcmp(&calibration, &calibration_before, sizeof(calibration)) == 0);
    assert(!locked);
    puts("MSTORE FAILURE outputs-preserved lock-balanced");
    g_cases++;
}

static void seed_feedback(uint32_t stamp, uint32_t epoch, uint64_t centre,
                          uint32_t losses, unsigned flags)
{
    motion_lock();
    g_flags = (motion_flags_t){0};
    memcpy(&g_feedback.centre_m, &centre, sizeof(centre));
    g_feedback.timestamp_us = stamp;
    g_feedback.epoch = epoch;
    g_report_loss_count = losses;
#ifdef MOTION_COMPACT_RETAINED
    g_flags.b_feedback_valid = (flags & 1U) != 0U;
    g_flags.b_feedback_ready = (flags & 2U) != 0U;
    g_flags.b_navigation_lease = (flags & 4U) != 0U;
#else
    g_feedback.valid = (flags & 1U) != 0U;
    g_feedback_ready = (flags & 2U) != 0U;
    g_navigation_lease = (flags & 4U) != 0U;
#endif
    motion_unlock();
}

static void test_feedback_storage(void)
{
    static const uint32_t clocks[] = {0U, 1U, UINT32_C(0x7fffffff),
        UINT32_C(0x80000000), UINT32_MAX - 1U, UINT32_MAX};
    static const uint64_t centres[] = {UINT64_C(0), UINT64_C(0x8000000000000000),
        UINT64_C(1), UINT64_C(0x7fefffffffffffff),
        UINT64_C(0x7ff8000000001234), UINT64_C(0xc0fabcde12345678)};
    motion_init();
    encoder_running = true;
    for (unsigned flags = 0U; flags < 8U; ++flags)
    {
        for (unsigned stamp = 0U; stamp < 6U; ++stamp)
        {
            for (unsigned epoch = 0U; epoch < 6U; ++epoch)
            {
                motion_feedback_t feedback;
                motion_debug_snapshot_t debug;
                uint32_t losses = clocks[(stamp + epoch) % 6U];
                uint64_t centre = centres[(stamp + epoch) % 6U];
                seed_feedback(clocks[stamp], clocks[epoch], centre, losses, flags);
                unsigned locks_before = g_lock_calls, unlocks_before = g_unlock_calls;
                assert(motion_get_feedback(&feedback));
                assert(motion_try_debug_snapshot(&debug));
                assert(!locked && g_lock_calls == locks_before + 2U &&
                       g_unlock_calls == unlocks_before + 2U);
                assert(feedback.timestamp_us == clocks[stamp] &&
                       feedback.epoch == clocks[epoch] &&
                       feedback.report_loss_count == losses &&
                       double_word(feedback.centre_m) == centre);
                assert(feedback.valid == ((flags & 1U) != 0U));
                assert(debug.timestamp_us == clocks[stamp]);
                assert(debug.valid == ((flags & 3U) == 3U));
                assert(feedback.state == MOTION_IDLE && feedback.fault == MOTION_FAULT_NONE);
                assert(debug.state == feedback.state && debug.fault == feedback.fault);
                assert(debug.braked == feedback.braked && !debug.busy);
                printf("MSTORE FB %u %08" PRIx32 " %08" PRIx32 " %08" PRIx32
                       " %016" PRIx64 " %u %u\n", flags, feedback.timestamp_us,
                       feedback.epoch, feedback.report_loss_count,
                       double_word(feedback.centre_m), feedback.valid, debug.valid);
                g_cases++;
            }
        }
    }
}

static void test_feedback_wrap(void)
{
    motion_feedback_t feedback;
    seed_feedback(UINT32_MAX, UINT32_MAX, 0U, UINT32_MAX, 3U);
    clock_us = UINT32_MAX - 4999U;
    motion_init();
    assert(motion_get_feedback(&feedback));
    assert(feedback.epoch == 0U && feedback.timestamp_us == UINT32_MAX && !feedback.valid);
    clock_us += 10000U;
    motion_process();
    assert(motion_get_feedback(&feedback));
    assert(feedback.epoch == 0U && feedback.timestamp_us == 5000U && feedback.valid);
    assert(double_word(feedback.centre_m) == 0U);
    motion_lock();
    queued = MOTION_REPORT_QUEUE_LEN;
    const motion_report_t report = {255U, MOTION_OUTCOME_CANCELLED, MOTION_FAULT_NONE, false};
    report_post(&report);
    motion_unlock();
    assert(motion_get_feedback(&feedback) && feedback.report_loss_count == 0U);
    motion_navigation_heartbeat(true);
    assert(motion_request_calibration() == MOTION_ERR_BUSY);
    motion_navigation_heartbeat(false);
    assert(motion_request_calibration() == MOTION_OK);
    assert(motion_get_feedback(&feedback) && !feedback.valid);
    puts("MSTORE WRAP clock epoch report-loss lease-exclusion");
    g_cases++;
}

int main(void)
{
    assert(motion_existing_test_suite() == 0);
    test_calibration_storage();
    test_failure_outputs();
    test_feedback_storage();
    test_feedback_wrap();
    fprintf(stderr, "Retained sizes: calibration=%zu feedback=%zu flags=%zu; wide calibration=%zu feedback=%zu\n",
            sizeof(g_cal_result), sizeof(g_feedback), sizeof(g_flags),
            sizeof(motion_calibration_t), sizeof(motion_feedback_t));
    printf("PASS: %u motion storage characterization cases\n", g_cases);
    return 0;
}
