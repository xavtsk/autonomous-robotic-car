/** @file s5_task_main.c
 * @brief Exercise obstacle task ownership, requests and snapshots with fakes.
 * The selected narrow integer operands fit in int.
 * Unsigned casts spell out the original modulo conversion.
 */
#include "s5_task_main.h"
#include "../../app_program/subsystem5/buddy5.h"
#include "../../app_program/subsystem5/scan_servo.h"
#include "../../app_program/subsystem5/ultrasonic_sensor.h"
#include "../../app_program/subsystem2/motion.h"
#include <assert.h>
#include <setjmp.h>
#include <stdio.h>

#define OBSTACLE_CASE_COUNT 5U

static jmp_buf  g_finish;
static uint32_t g_clock_us;
static uint32_t g_trigger_us;
static uint32_t g_last_trigger;
static unsigned g_angle = 90;
static unsigned g_loops;
static unsigned g_locks;
static unsigned g_scenario;
static unsigned g_stage;
static unsigned g_scan_angles;
static bool     gb_has_hardware_started;
static bool     gb_is_pinging;
static bool     gb_are_wheels_stationary = true;
static bool     gb_should_fail_echo;
static unsigned g_bad_pings;
static ER       g_snapshot_lock_error;

static void ( * gp_owner)(INT start_code, void * p_extra_info) = NULL;

static void request_fixture_scan (void);

/** @brief Return the current simulated microsecond count for the timer read.
 */
uint32_t fake_read (uint32_t address)
{
    (void)address;
    return g_clock_us;
}

/** @brief Discard the register write at the simulated hardware boundary.
 */
void fake_write (uint32_t address, uint32_t value)
{
    (void)address;
    (void)value;
}

/** @brief Verify the priority-inheritance attribute and return the configured
 * fake mutex ID.
 */
ID tk_cre_mtx (const T_CMTX * p_mutex)
{
    assert(TA_INHERIT == p_mutex->mtxatr);
    return 1;
}

/** @brief Require the owner mutex to be free. Inject the configured polling
 * failure or grant the lock.
 */
ER tk_loc_mtx (ID h_mutex, TMO timeout)
{
    assert(1 == h_mutex && !g_locks);

    if ((TMO_POL == timeout) && g_snapshot_lock_error)
    {
        return g_snapshot_lock_error;
    }

    g_locks++;
    return E_OK;
}

/** @brief Require the acquired fake mutex, then release it and report
 * success.
 */
ER tk_unl_mtx (ID h_mutex)
{
    assert(1 == h_mutex && g_locks);
    g_locks--;
    return E_OK;
}

/** @brief Check the required obstacle-task priority and core, then save its
 * entry and return the test task ID.
 */
ID app_start_task (FP p_task_entry, PRI priority, UW core, const char * p_name)
{
    (void)p_name;
    assert(20 == priority && TP_PRC1 == core);
    gp_owner = (void (*)(INT, void *))p_task_entry;
    return 2;
}

/** @brief Provide valid, timestamped idle motion feedback with the fixture's
 * stationary-wheel flag outside the owner lock.
 */
bool motion_get_feedback (motion_feedback_t * p_feedback)
{
    assert(!g_locks);
    *p_feedback = (motion_feedback_t){.b_is_feedback_valid = true,
                                      .b_is_braked  = gb_are_wheels_stationary,
                                      .timestamp_us = g_clock_us,
                                      .state        = MOTION_IDLE};
    return true;
}

/** @brief Mark the simulated ultrasonic hardware initialized while outside
 * the owner lock.
 */
bool ultrasonic_sensor_init (void)
{
    assert(!g_locks);
    gb_has_hardware_started = true;
    return true;
}

/** @brief Require successful ultrasonic initialization and an unlocked owner
 * before acknowledging servo startup.
 */
bool scan_servo_init (void)
{
    assert(!g_locks && gb_has_hardware_started);
    return true;
}

/** @brief Record the requested angle in degrees. Require stopped wheels and
 * count scan positions when steering away from centre.
 */
bool scan_servo_position (unsigned value)
{
    assert(!g_locks);

    if (value != 90U)
    {
        assert(gb_are_wheels_stationary);
        g_scan_angles++;
    }

    g_angle = value;
    return true;
}

/** @brief Accept the servo disable request without touching PWM registers.
 */
void scan_servo_disable (void)
{
}

/** @brief Require an idle ranger and at least 70 ms since the previous
 * trigger, then record the new trigger timestamp.
 */
bool ultrasonic_sensor_trigger (uint32_t now)
{
    assert(!g_locks && !gb_is_pinging &&
           (uint32_t)(now - g_last_trigger) >= 70000U);
    /* clang-format off */
    g_trigger_us  = g_last_trigger = now;
    gb_is_pinging = true;
    /* clang-format on */
    return true;
}

/** @brief Complete a pending echo after 10 ms. Write its configured distance
 * in millimetres and validity, including an injected failure.
 */
bool ultrasonic_sensor_poll (uint32_t now, uint16_t * p_mm, bool * pb_is_valid)
{
    assert(!g_locks);

    if ((!gb_is_pinging) || ((now - g_trigger_us) < 10000U))
    {
        return false;
    }

    gb_is_pinging = false;

    if (gb_should_fail_echo)
    {
        g_bad_pings++;
    }

    *pb_is_valid = !gb_should_fail_echo;
    *p_mm        = gb_should_fail_echo ? 0 : (90U == g_angle ? 250 : 2000);
    return true;
}

/** @brief Clear a pending simulated ping and require the owner lock to be
 * free.
 */
void ultrasonic_sensor_cancel (void)
{
    assert(!g_locks);
    gb_is_pinging = false;
}

/** @brief Advance the simulated clock by the requested milliseconds and
 * exercise the task scenario at each delay boundary.
 */
ER tk_dly_tsk (unsigned milliseconds)
{
    s5_codec_observation_t observation = {0}; /* GCC 13 -Wmaybe-uninitialized */
    assert(!g_locks);
    g_clock_us += milliseconds * 1000U;
    g_loops++;
    assert(s5_api_snapshot(&observation));

    if ((0U == g_stage) && (observation.b_is_range_valid))
    {
        if (2U == g_scenario)
        {
            gb_should_fail_echo = true;
            g_stage             = 1;
        }
        else if (4U == g_scenario)
        {
            /* Side peek: invalid angles refused, then one reading at 30 deg. */
            assert(!s5_api_request_peek(77, 90) &&
                   !s5_api_request_peek(77, 65) && !s5_api_request_peek(0, 30));
            assert(s5_api_request_peek(77, 30) && !s5_api_request_scan(78));
            g_stage = 1;
        }
        else
        {
            request_fixture_scan();
        }
    }

    if (((0U == g_scenario) && (1U == g_stage)) && (observation.b_is_scan_done))
    {
        assert(42 == (int)observation.scan_id &&
               observation.profile.b_is_valid && g_scan_angles >= 6U);
        g_stage = 2;
    }

    if (((0U == g_scenario) && (2U == g_stage)) &&
        (observation.b_is_range_valid))
    {
        longjmp(g_finish, 1);
    }

    if (((1U == g_scenario) && (1U == g_stage)) && (g_angle != 90U))
    {
        s5_api_cancel_scan();
        g_stage = 2;
    }

    if ((((1U == g_scenario) && (2U == g_stage)) && (90U == g_angle)) &&
        (observation.b_is_range_valid))
    {
        assert(!observation.b_is_scan_done);
        longjmp(g_finish, 1);
    }

    if ((((4U == g_scenario) && (1U == g_stage)) &&
         (observation.b_is_scan_done)) &&
        (77 == (int)observation.scan_id))
    {
        /* One servo move, that side's clearance reported, the other side
         * untouched. */
        assert(1U == g_scan_angles && observation.profile.b_is_valid &&
               2000 == (int)observation.profile.right_clear_mm &&
               S5_RANGE_MAX_MM == observation.profile.left_clear_mm);
        g_stage = 2;
    }

    if ((((4U == g_scenario) && (2U == g_stage)) && (90U == g_angle)) &&
        (observation.b_is_range_valid))
    {
        assert(250 == (int)observation.range_mm);
        longjmp(g_finish, 1);
    }

    if (((3U == g_scenario) && (1U == g_stage)) && (observation.b_is_scan_done))
    {
        assert(!observation.profile.b_is_valid && 0U == g_scan_angles);
        longjmp(g_finish, 1);
    }
    /* One or two bad pings keep the last good reading; three latch no_echo. */
    if ((((2U == g_scenario) && (1U == g_stage)) && (g_bad_pings >= 1U)) &&
        (g_bad_pings < 3U))
    {
        assert(!observation.b_has_no_echo && observation.b_is_range_valid &&
               250 == (int)observation.range_mm);
    }

    if (((2U == g_scenario) && (1U == g_stage)) && (observation.b_has_no_echo))
    {
        assert(g_bad_pings >= 3U && !observation.b_is_range_valid &&
               0 == (int)observation.range_mm);
        longjmp(g_finish, 1);
    }

    assert(g_loops < 1500U);
    return E_OK;
}

/** @brief Exercise the ultrasonic owner with simulated polls and a bounded
 * mailbox. The unconfirmed-hardware profile checks startup refusal.
 */
int main (void)
{
#if !S5_HARDWARE_CONFIRMED
    assert(subsystem5_init() == E_PAR);
    assert(subsystem5_start() == E_OBJ);
    assert(!gb_has_hardware_started);
    puts("PASS: unconfirmed hardware cannot initialise or start Buddy 5");
    return 0;
#endif
    for (g_scenario = 0; g_scenario < OBSTACLE_CASE_COUNT; g_scenario++)
    {
        s5_task_test_reset_mailbox();
        /* clang-format off */
        g_clock_us                   =
            g_loops = g_locks = g_stage = g_scan_angles =
            g_last_trigger                       = 0;
        g_angle                      = 90;
        gb_has_hardware_started      =
            gb_is_pinging = gb_should_fail_echo = false;
        gb_are_wheels_stationary     = true;
        g_bad_pings                  = 0;
        /* clang-format on */
        assert(subsystem5_init() == E_OK && subsystem5_start() == E_OK);
        {
            s5_codec_observation_t copy = {.range_mm = 123};
            assert(s5_api_read_snapshot(&copy) == (unsigned)S5_SNAPSHOT_OK);
            g_snapshot_lock_error = E_TMOUT;
            assert(s5_api_read_snapshot(&copy) == (unsigned)S5_SNAPSHOT_BUSY &&
                   !g_locks);
            assert(!s5_api_snapshot(&copy));
            g_snapshot_lock_error = E_OBJ;
            assert(s5_api_read_snapshot(&copy) ==
                   (unsigned)S5_SNAPSHOT_UNAVAILABLE);
            g_snapshot_lock_error = 0;
            s5_task_test_set_reserved_bit(true);
            assert(s5_api_read_snapshot(&copy) ==
                       (unsigned)S5_SNAPSHOT_INVALID &&
                   !g_locks);
            s5_task_test_set_reserved_bit(false);
            assert(s5_api_read_snapshot(&copy) == (unsigned)S5_SNAPSHOT_OK);
        }

        if (!setjmp(g_finish))
        {
            gp_owner(0, 0);
        }
    }

    puts("PASS: production sensing task with fake clock/kernel/sensor; "
         "stop-gated scan, bounded mailbox, packed snapshots, "
         "cancellation/recentering, confirmed no-echo, side peek");
}

/** @brief Request a full scan in the current test scenario.
 * The moving scenario clears the stationary flag before the requests.
 * Record the accepted request and assert that the second request is rejected.
 */
static void request_fixture_scan (void)
{
    if (3U == g_scenario)
    {
        gb_are_wheels_stationary = false;
    }

    assert(s5_api_request_scan(42));
    assert(!s5_api_request_scan(43));
    g_stage = 1;
}

/* End of file. */

