/** @file firmware_main.c
 * @brief Exercise the optional diagnostics module with simulated RTOS,
 * MMIO and motion observations. Inspect its published summaries.
 * Unsigned casts spell out the original modulo conversion.
 */
#include "firmware_main.h"
#include "../../app_program/app_types.h"
#include "../../app_program/subsystem2/codec_motion.h"
#include "../../app_program/subsystem2/wheel_encoder.h"
#include <assert.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

static int                     g_locked;
static int                     g_fail_create;
static int                     g_fail_lock;
static unsigned                g_lock_calls;
static unsigned                g_unlock_calls;
static unsigned                g_snapshot_calls;
static unsigned                g_print_calls;
static uint32_t                g_clock_value;
static bool                    gb_is_snapshot_available = true;
static codec_motion_snapshot_t g_motion_value;
static float32_t               g_left_speed  = 0.1234f;
static float32_t               g_right_speed = -0.1189f;
static char                    g_output[16384];
static size_t                  g_output_used;

static void clear_output (void);

/** @brief Verify unlocked creation with priority inheritance. Return the
 * injected failure or mutex ID seven.
 */
ID tk_cre_mtx (const T_CMTX * p_value)
{
    assert(p_value && TA_INHERIT == p_value->mtxatr && !g_locked);
    return g_fail_create ? -91 : 7;
}

/** @brief Count nonblocking polls of mutex seven. An injected timeout leaves
 * it free. Otherwise mark it locked and return success.
 */
ER tk_loc_mtx (ID h_mutex, TMO timeout)
{
    assert(7 == h_mutex && TMO_POL == timeout && !g_locked);
    g_lock_calls++;

    if (g_fail_lock)
    {
        return E_TMOUT;
    }

    g_locked = 1;
    return E_OK;
}

/** @brief Require the held mutex with ID seven, release it and count the
 * unlock.
 */
ER tk_unl_mtx (ID h_mutex)
{
    assert(7 == h_mutex && g_locked);
    g_locked = 0;
    g_unlock_calls++;
    return E_OK;
}

/** @brief Append formatted diagnostics to the bounded output buffer while
 * unlocked. Count the print and return its character count.
 */
INT tm_printf (const UB * p_format, ...)
{
    va_list args;
    int     count;
    assert(!g_locked && g_output_used < sizeof(g_output));
    g_print_calls++;
    va_start(args, p_format);
    count =
        vsnprintf(g_output + g_output_used, sizeof(g_output) - g_output_used,
                  (const char *)p_format, args);
    va_end(args);
    assert(count >= 0 && (size_t)count < sizeof(g_output) - g_output_used);
    g_output_used += (size_t)count;
    return count;
}

/** @brief Require the timer's raw-low register address and return the
 * fixture's current clock value.
 */
uint32_t diagnostics_test_in_w (uint32_t address)
{
    assert(UINT32_C(0x40054028) == address);
    return g_clock_value;
}

/** @brief Require an unlocked diagnostic read and return the simulated left
 * count of minus 12345.
 */
int32_t enc_l_count (void)
{
    assert(!g_locked);
    return -12345;
}

/** @brief Require an unlocked diagnostic read and return the simulated right
 * count of 67890.
 */
int32_t enc_r_count (void)
{
    assert(!g_locked);
    return 67890;
}

/** @brief Return the selected left-wheel speed in metres per second and
 * assert that the diagnostics mutex is free.
 */
float32_t motion_wheel_left_speed_mps (void)
{
    assert(!g_locked);
    return g_left_speed;
}

/** @brief Return the selected right-wheel speed in metres per second and
 * assert that the diagnostics mutex is free.
 */
float32_t motion_wheel_right_speed_mps (void)
{
    assert(!g_locked);
    return g_right_speed;
}

/** @brief Read the simulated unlocked glitch count. The left wheel reports
 * four and the right reports nine.
 */
uint32_t wheel_encoder_glitches (wheel_encoder_id_t wheel)
{
    assert(!g_locked);
    return (unsigned)WHEEL_LEFT == wheel ? 4U : 9U;
}

/** @brief Count an unlocked snapshot request. Copy the selected motion
 * report on success. An unavailable snapshot leaves the destination
 * untouched.
 */
bool snapshot_motion_read (codec_motion_snapshot_t * p_out)
{
    assert(!g_locked);
    g_snapshot_calls++;

    if (!gb_is_snapshot_available)
    {
        return false;
    }

    *p_out = g_motion_value;
    return true;
}

/* No motion_poll or movement/reset symbols are defined: linking would fail
 * if diagnostics started consuming completion or commanding motors. */
/** @brief Inspect diagnostics publication and skipped updates under
 * simulated lock failures, then print the measured record sizes.
 */
int main (void)
{
    robot_diagnostics_record(ROBOT_DIAG_MOTION, 10U, 20U);
    robot_diagnostics_report();
    assert(0U == g_lock_calls && 0U == g_print_calls);
    g_fail_create = 1;
    assert(robot_diagnostics_init() == -91);
    g_fail_create = 0;
    assert(robot_diagnostics_init() == E_OK);
    assert(robot_diagnostics_init() == E_OBJ);
    g_clock_value = UINT32_C(0xfffffff0);
    assert(robot_diagnostics_now_us() == g_clock_value);
    robot_diagnostics_record(ROBOT_DIAG_SOURCE_COUNT, 1U, 2U);
    assert(0U == g_lock_calls);

    robot_diagnostics_record(ROBOT_DIAG_MOTION, 1000U, 1100U);
    robot_diagnostics_test_summary_t published =
        robot_diagnostics_test_published(ROBOT_DIAG_MOTION);
    assert(1U == published.samples);
    assert(!published.b_is_gap_valid);
    assert(100U == published.exec_max_us);

    /* A busy publication does not lose measurements or lengthen the next
     * measured gap into a false multiple of the source period. */
    g_fail_lock = 1;
    robot_diagnostics_record(ROBOT_DIAG_MOTION, 11000U, 11700U);
    published = robot_diagnostics_test_published(ROBOT_DIAG_MOTION);
    assert(1U == published.samples);
    robot_diagnostics_record(ROBOT_DIAG_MOTION, 21000U, 21300U);
    g_fail_lock = 0;
    robot_diagnostics_record(ROBOT_DIAG_MOTION, 31000U, 31200U);
    published = robot_diagnostics_test_published(ROBOT_DIAG_MOTION);
    assert(4U == published.samples);
    assert(2U == published.publish_skips);
    assert(10000U == published.gap_min_us);
    assert(10000U == published.gap_max_us);
    assert(700U == published.exec_max_us);

    robot_diagnostics_record(ROBOT_DIAG_IR, UINT32_C(0xfffffff0), 10U);
    robot_diagnostics_record(ROBOT_DIAG_IR, 1000U, 1050U);
    published = robot_diagnostics_test_published(ROBOT_DIAG_IR);
    assert(2U == published.samples);
    assert(1016U == published.gap_min_us);
    assert(50U == published.exec_max_us);
    assert(robot_diagnostics_test_increment(UINT32_MAX) == UINT32_MAX);
    assert(robot_diagnostics_test_increment(UINT32_MAX - 1U) == UINT32_MAX);

    assert(codec_motion_pack(&g_motion_value, MOTION_COMPLETE,
                             MOTION_FAULT_NONE, false, true, 120, -118));
    clear_output();
    robot_diagnostics_report();
    assert(8U == g_print_calls && 1U == g_snapshot_calls && !g_locked);
    assert(strstr(g_output, "encoder=PIO glitches_available=0"));
    assert(strstr(g_output, "source=motion samples=4 publish_skips=2"));
    assert(
        strstr(g_output, "gap_min_us=10000 gap_max_us=10000 exec_max_us=700"));
    assert(strstr(g_output, "counts left=-12345 right=67890"));
    assert(strstr(g_output, "glitches_left=4 glitches_right=9"));
    assert(strstr(g_output, "state=4 fault=0 busy=0 snapshot_speed_valid=1"));
    assert(strstr(
        g_output,
        "estimate left_valid=1 left_mms=123 right_valid=1 right_mms=-118"));
    assert(codec_motion_state(&g_motion_value) == (unsigned)MOTION_COMPLETE);
    robot_diagnostics_report();
    assert(codec_motion_state(&g_motion_value) == (unsigned)MOTION_COMPLETE);

    clear_output();
    g_fail_lock = 1;
    robot_diagnostics_report();
    assert(0U == g_print_calls);
    g_fail_lock              = 0;
    gb_is_snapshot_available = false;
    robot_diagnostics_report();
    assert(strstr(g_output, "snapshot_valid=0\n"));
    assert(!strstr(g_output, "snapshot_speed_valid="));
    gb_is_snapshot_available = true;
    assert(codec_motion_pack(&g_motion_value, MOTION_DRIVE, MOTION_FAULT_NONE,
                             true, false, 0, 0));
    clear_output();
    robot_diagnostics_report();
    assert(strstr(g_output, "busy=1 snapshot_speed_valid=0"));
    assert(strstr(
        g_output,
        "estimate left_valid=1 left_mms=123 right_valid=1 right_mms=-118"));
    {
        int32_t result = 91;

        app_types_binary32_view_t special;

        assert(robot_diagnostics_test_estimate(0.0f, &result) && 0 == result);
        assert(robot_diagnostics_test_estimate(-2147483.648f, &result) &&
               INT32_MIN == result);
        assert(!robot_diagnostics_test_estimate(2147483.648f, &result));
        assert(!robot_diagnostics_test_estimate(-2147484.0f, &result));
        special.bits  = UINT32_C(0x7fc00000); /* Use a quiet NaN. */
        g_left_speed  = special.value;
        special.bits  = UINT32_C(0x7f800000); /* Use positive infinity. */
        g_right_speed = special.value;
        clear_output();
        robot_diagnostics_report();
        assert(strstr(
            g_output,
            "estimate left_valid=0 left_mms=0 right_valid=0 right_mms=0"));
        result = 91;
        assert(!robot_diagnostics_test_estimate(g_left_speed, &result) &&
               91 == result);
        assert(!robot_diagnostics_test_estimate(g_right_speed, &result) &&
               91 == result);
    }
    assert(!g_locked &&
           g_unlock_calls + 3U == g_lock_calls); /* Two skips and one failure
                                                    account for these calls. */
    printf("PASS: diagnostic timing, first samples, wrap, deferred "
           "publication, saturation, lock boundaries and non-consuming motion "
           "observations\n");
    const robot_diagnostics_test_layout_t layout =
        robot_diagnostics_test_layout();
    printf("Private/published channel sizes: %zu/%zu bytes; retained "
           "diagnostic state %zu bytes plus mutex ID\n",
           layout.producer_bytes, layout.published_bytes, layout.total_bytes);
    return 0;
}

/** @brief Empty the diagnostic console buffer and reset its used length and
 * print count.
 */
static void clear_output (void)
{
    g_output_used = 0U;
    g_output[0]   = '\0';
    g_print_calls = 0U;
}

/* End of file. */

