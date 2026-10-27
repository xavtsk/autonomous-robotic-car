/* Include the actual optional module to inspect publication versus owner-only
 * summaries; only RTOS/MMIO/status observations are substituted. */
#include "firmware_test_kernel.h"
#include "../../app_program/subsystem2/motion_codec.h"
#include "../../app_program/subsystem2/wheel_encoder.h"
#include <assert.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

static int locked;
static int fail_create;
static int fail_lock;
static unsigned lock_calls, unlock_calls, snapshot_calls, print_calls;
static uint32_t clock_value;
static bool snapshot_available = true;
static motion_snapshot_t motion_value;
static float left_speed = 0.1234f, right_speed = -0.1189f;
static char output[16384];
static size_t output_used;

ID tk_cre_mtx(const T_CMTX *value)
{
    assert(value && value->mtxatr == TA_INHERIT && !locked);
    return fail_create ? -91 : 7;
}
ER tk_loc_mtx(ID id, TMO timeout)
{
    assert(id == 7 && timeout == TMO_POL && !locked);
    lock_calls++;
    if (fail_lock) { return E_TMOUT; }
    locked = 1;
    return E_OK;
}
ER tk_unl_mtx(ID id)
{
    assert(id == 7 && locked);
    locked = 0;
    unlock_calls++;
    return E_OK;
}
INT tm_printf(const UB *format, ...)
{
    va_list args;
    int count;
    assert(!locked && output_used < sizeof(output));
    print_calls++;
    va_start(args, format);
    count = vsnprintf(output + output_used, sizeof(output) - output_used,
                      (const char *)format, args);
    va_end(args);
    assert(count >= 0 && (size_t)count < sizeof(output) - output_used);
    output_used += (size_t)count;
    return count;
}
uint32_t diagnostics_test_in_w(uint32_t address)
{
    assert(address == UINT32_C(0x40054028));
    return clock_value;
}
int32_t enc_l_count(void) { assert(!locked); return -12345; }
int32_t enc_r_count(void) { assert(!locked); return 67890; }
float wheel_speed_left_mps(void) { assert(!locked); return left_speed; }
float wheel_speed_right_mps(void) { assert(!locked); return right_speed; }
uint32_t wheel_encoder_glitches(wheel_id_t wheel)
{ assert(!locked); return wheel == WHEEL_LEFT ? 4U : 9U; }
bool motion_read_snapshot(motion_snapshot_t *out)
{
    assert(!locked);
    snapshot_calls++;
    if (!snapshot_available) { return false; }
    *out = motion_value;
    return true;
}
/* No motion_poll or movement/reset symbols are defined: linking would fail
 * if diagnostics started consuming completion or commanding motors. */
#include "../../app_program/diagnostics/robot_diagnostics.c"

static void clear_output(void)
{ output_used = 0U; output[0] = '\0'; print_calls = 0U; }

int main(void)
{
    robot_diagnostics_record(ROBOT_DIAG_MOTION, 10U, 20U);
    robot_diagnostics_report();
    assert(lock_calls == 0U && print_calls == 0U);
    fail_create = 1;
    assert(robot_diagnostics_init() == -91);
    fail_create = 0;
    assert(robot_diagnostics_init() == E_OK);
    assert(robot_diagnostics_init() == E_OBJ);
    clock_value = UINT32_C(0xfffffff0);
    assert(robot_diagnostics_now_us() == clock_value);
    robot_diagnostics_record(ROBOT_DIAG_SOURCE_COUNT, 1U, 2U);
    assert(lock_calls == 0U);

    robot_diagnostics_record(ROBOT_DIAG_MOTION, 1000U, 1100U);
    assert(g_published[ROBOT_DIAG_MOTION].samples == 1U);
    assert(!g_published[ROBOT_DIAG_MOTION].gap_valid);
    assert(g_published[ROBOT_DIAG_MOTION].exec_max_us == 100U);

    /* A busy publication does not lose measurements or lengthen the next
     * measured gap into a false multiple of the source period. */
    fail_lock = 1;
    robot_diagnostics_record(ROBOT_DIAG_MOTION, 11000U, 11700U);
    assert(g_published[ROBOT_DIAG_MOTION].samples == 1U);
    robot_diagnostics_record(ROBOT_DIAG_MOTION, 21000U, 21300U);
    fail_lock = 0;
    robot_diagnostics_record(ROBOT_DIAG_MOTION, 31000U, 31200U);
    assert(g_published[ROBOT_DIAG_MOTION].samples == 4U);
    assert(g_published[ROBOT_DIAG_MOTION].publish_skips == 2U);
    assert(g_published[ROBOT_DIAG_MOTION].gap_min_us == 10000U);
    assert(g_published[ROBOT_DIAG_MOTION].gap_max_us == 10000U);
    assert(g_published[ROBOT_DIAG_MOTION].exec_max_us == 700U);

    robot_diagnostics_record(ROBOT_DIAG_IR, UINT32_C(0xfffffff0), 10U);
    robot_diagnostics_record(ROBOT_DIAG_IR, 1000U, 1050U);
    assert(g_published[ROBOT_DIAG_IR].samples == 2U);
    assert(g_published[ROBOT_DIAG_IR].gap_min_us == 1016U);
    assert(g_published[ROBOT_DIAG_IR].exec_max_us == 50U);
    assert(increment_saturated(UINT32_MAX) == UINT32_MAX);
    assert(increment_saturated(UINT32_MAX - 1U) == UINT32_MAX);

    assert(motion_snapshot_pack(&motion_value, MOTION_COMPLETE,
        MOTION_FAULT_NONE, false, true, 120, -118));
    clear_output();
    robot_diagnostics_report();
    assert(print_calls == 8U && snapshot_calls == 1U && !locked);
    assert(strstr(output, "encoder=PIO glitches_available=0"));
    assert(strstr(output, "source=motion samples=4 publish_skips=2"));
    assert(strstr(output, "gap_min_us=10000 gap_max_us=10000 exec_max_us=700"));
    assert(strstr(output, "counts left=-12345 right=67890"));
    assert(strstr(output, "glitches_left=4 glitches_right=9"));
    assert(strstr(output, "state=4 fault=0 busy=0 snapshot_speed_valid=1"));
    assert(strstr(output, "estimate left_valid=1 left_mms=123 right_valid=1 right_mms=-118"));
    assert(motion_snapshot_state(&motion_value) == MOTION_COMPLETE);
    robot_diagnostics_report();
    assert(motion_snapshot_state(&motion_value) == MOTION_COMPLETE);

    clear_output();
    fail_lock = 1;
    robot_diagnostics_report();
    assert(print_calls == 0U);
    fail_lock = 0;
    snapshot_available = false;
    robot_diagnostics_report();
    assert(strstr(output, "snapshot_valid=0\n"));
    assert(!strstr(output, "snapshot_speed_valid="));
    snapshot_available = true;
    assert(motion_snapshot_pack(&motion_value, MOTION_DRIVE,
        MOTION_FAULT_NONE, true, false, 0, 0));
    clear_output();
    robot_diagnostics_report();
    assert(strstr(output, "busy=1 snapshot_speed_valid=0"));
    assert(strstr(output, "estimate left_valid=1 left_mms=123 right_valid=1 right_mms=-118"));
    {
        int32_t result = 91;
        union { uint32_t bits; float value; } special;
        assert(estimator_mms(0.0f, &result) && result == 0);
        assert(estimator_mms(-2147483.648f, &result) && result == INT32_MIN);
        assert(!estimator_mms(2147483.648f, &result));
        assert(!estimator_mms(-2147484.0f, &result));
        special.bits = UINT32_C(0x7fc00000); /* quiet NaN */
        left_speed = special.value;
        special.bits = UINT32_C(0x7f800000); /* positive infinity */
        right_speed = special.value;
        clear_output();
        robot_diagnostics_report();
        assert(strstr(output, "estimate left_valid=0 left_mms=0 right_valid=0 right_mms=0"));
        result = 91;
        assert(!estimator_mms(left_speed, &result) && result == 91);
        assert(!estimator_mms(right_speed, &result) && result == 91);
    }
    assert(!locked && unlock_calls + 3U == lock_calls); /* 2 skipped + 1 failed report */
    printf("PASS: diagnostic timing, first samples, wrap, deferred publication, saturation, lock boundaries and non-consuming motion observations\n");
    printf("Private/published channel sizes: %zu/%zu bytes; retained diagnostic state %zu bytes plus mutex ID\n",
        sizeof(DIAG_PRODUCER), sizeof(DIAG_SUMMARY), sizeof(g_producer) + sizeof(g_published));
    return 0;
}
