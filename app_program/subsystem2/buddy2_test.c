/** @file buddy2_test.c
 * @brief On-target unit tests (make TEST=1). IDs match
 * testing/subsystem2/TEST_LOG.md.
 */
/*
 * Owner: Buddy 2 - Motion control using motors and wheel encoders.
 * The test IDs are listed in testing/subsystem2/TEST_LOG.md.
 * Motion test constants are nonnegative and fit unsigned.
 * S2-U01..U10 and S2-U13..U20 never move the robot: every velocity target is
 * 0 and every drive request has zero length. S2-U11 / U12 / U21 / U22 drive
 * forward, turn left, turn right and drive back, ending where they started;
 * they run only when S2_TEST_MOVE is 1 (robot on the floor, 30 cm
 * clear ahead).
 * S2-U05..U22 need the CALIBRATE values in motion.c and report SKIP without.
 *
 * S2_TEST_CALIBRATE = 1 runs calibration instead of the tests and prints the
 * CALIBRATE values for motion.c. Follow the [CAL] prompts on the console.
 */

#include "buddy2_test.h"
#include "../app_types.h"
#include "motion.h"
#include "motion_pack.h"
#if APP_TEST
#include "../test_report.h"
#ifndef S2_TEST_MOVE
#define S2_TEST_MOVE (0) /* One enables motion tests. */
#endif

#ifndef S2_TEST_CALIBRATE
#define S2_TEST_CALIBRATE (0) /* One runs only calibration. */
#endif
#define TEST_IDLE ((unsigned)MOTION_IDLE)
#define TEST_OK ((unsigned)MOTION_OK)
#define TEST_CANCEL ((unsigned)MOTION_CMD_CANCEL)
#define TEST_BAD_ARG ((unsigned)MOTION_ERR_BAD_ARG)
#define STARTUP_WAIT_MS (100) /* Wait for motion_init(). */
#define CAL_PRINT_MS (1000)   /* Print counts while waiting. */
#define STILL_WAIT_MS (200)
#define STOP_WAIT_MS (20)   /* Wait two control periods. */
#define STALE_WAIT_MS (400) /* Wait past the 250 ms timeout. */
#define RESET_WAIT_MS (300) /* Wait past 200 ms of stillness. */
#define MOVE_TIMEOUT_MS (10000)
#define POLL_MS (20)

#define MOVE_DIST_M (0.20f)
#define MOVE_TOL_M (0.02f)
#define TURN_DEG (90.0f)
#define TURN_TOL_DEG (10.0f)

/** @brief Print the test ID and the reason it was skipped. */
static void skip (const char * p_id, const char * p_why)
{
    tm_printf((UB *)"[TEST] %s SKIP  %s\n", p_id, p_why);
}

/** @brief Return the magnitude using the original negative-value test. */
static float32_t abs_f (float32_t value)
{
    return (value < 0.0f) ? -value : value;
}

/** @brief Allow half a quantization step plus the test's rounding margin. */
static bool near_f (float32_t first, float32_t second, float32_t step)
{
    return abs_f(first - second) <= ((0.5f * step) + 0.0001f);
}

#if S2_TEST_MOVE

/** @brief Poll the drive until completion, fault or timeout. Return the last
 * observed motion state. */
static motion_state_t wait_move (void)
{
    motion_state_t state  = motion_poll();
    int32_t        waited = 0;

    while ((state != (unsigned)MOTION_COMPLETE) &&
           (state != (unsigned)MOTION_FAULT) && (waited < MOVE_TIMEOUT_MS))
    {
        (void)tk_dly_tsk(POLL_MS);
        waited += POLL_MS;
        state   = motion_poll();
    }

    return state;
}
#endif /* S2_TEST_MOVE */
/** @brief Return a static true or false string for calibration output. */
static const char * bool_text (bool b_is_true)
{
    return b_is_true ? "true" : "false";
}

/** @brief Print a speed in metres per second with three decimal places. */
static void print_mps (const char * p_name, float32_t mps)
{
    int32_t speed_mmps = (int32_t)((mps * 1000.0f) + 0.5f);

    tm_printf((UB *)"#define %-23s (%d.%03df)\n", p_name,
              (int)(speed_mmps / 1000), (int)(speed_mmps % 1000));
}

/** @brief Run the guided motion calibration and print its configuration. */
static void calibrate (void)
{
    motion_calibration_t cal;

    tm_printf((UB *)"[CAL] 1. Turn EACH wheel by hand exactly 10 full turns"
                    " forward,\n");
    tm_printf((UB *)"[CAL]    then leave both wheels still for 3 s.\n");
    tm_printf((UB *)"[CAL] 2. Lift the robot: 5 s later each motor runs at"
                    " full power.\n");

    if (motion_request_calibration() != (unsigned)MOTION_OK)
    {
        tm_printf((UB *)"[CAL] could not start: robot not IDLE\n");
        return;
    }
    while (motion_is_calibrating())
    {
        tm_printf((UB *)"[CAL] counts L=%d R=%d\n", (int)enc_l_count(),
                  (int)enc_r_count());
        (void)tk_dly_tsk(CAL_PRINT_MS);
    }
    if (!motion_get_calibration(&cal))
    {
        tm_printf((UB *)"[CAL] no result\n");
        return;
    }
    if (cal.b_was_aborted)
    {
        tm_printf((UB *)"[CAL] stopped by stop(): no result, motors braked\n");

        return;
    }
    if (!cal.b_is_left_enc_ok || !cal.b_is_right_enc_ok)
    {
        tm_printf((UB *)"[CAL] FAILED: encoder not counting (L ok=%s,"
                        " R ok=%s). Check A/B wiring and 3.3 V.\n",
                  bool_text(cal.b_is_left_enc_ok),
                  bool_text(cal.b_is_right_enc_ok));

        return;
    }

    tm_printf((UB *)"[CAL] counts per turn L=%d R=%d (should agree within"
                    " 5 %%)\n",
              (int)(cal.counts_per_rev_l + 0.5f),
              (int)(cal.counts_per_rev_r + 0.5f));
    tm_printf((UB *)"[CAL] top speed L=%d R=%d mm/s, encoders swapped=%s\n",
              (int)((cal.top_speed_l_mps * 1000.0f) + 0.5f),
              (int)((cal.top_speed_r_mps * 1000.0f) + 0.5f),
              bool_text(cal.b_are_encoders_swapped));
    tm_printf((UB *)"[CAL] result %s\n",
              cal.b_is_calibration_valid
                  ? "VALID: copy into motion.c"
                  : "NOT VALID: fix the problem above and retry");

    tm_printf((UB *)"#define ENCODER_COUNTS_PER_REV  (%d.0f)\n",
              (int)(cal.counts_per_rev + 0.5f));
    print_mps("MOTOR_MAX_SPEED_MPS", cal.max_speed_mps);
    tm_printf((UB *)"#define M1_INVERTED             (%s)\n",
              bool_text(cal.b_is_left_motor_inverted));
    tm_printf((UB *)"#define M2_INVERTED             (%s)\n",
              bool_text(cal.b_is_right_motor_inverted));
    tm_printf((UB *)"#define ENC_L_INVERTED          (%s)\n",
              bool_text(cal.b_is_left_enc_inverted));
    tm_printf((UB *)"#define ENC_R_INVERTED          (%s)\n",
              bool_text(cal.b_is_right_enc_inverted));
}

/** @brief Run checks S2-U01 through S2-U22 with the configured motion gate. */
static void run_unit_tests (void)
{
    volatile float32_t zero         = 0.0f;
    float32_t          not_a_number = zero / zero;
    int32_t            left;
    int32_t            right;
    bool               b_met; /* True means every check so far passed. */
    uint8_t            id1;
    uint8_t            id2;
    uint32_t           index;
    uint32_t           count;
    motion_report_t    rep;

    TEST_INIT();

    /* ---- Start-up ------------------------------------------------------ */
    TEST_CHECK("S2-U01",
               (motion_poll() == TEST_IDLE) &&
                   (motion_get_fault() == (unsigned)MOTION_FAULT_NONE),
               "starts in IDLE with no fault");

    left  = enc_l_count();
    right = enc_r_count();
    (void)tk_dly_tsk(STILL_WAIT_MS);
    TEST_CHECK("S2-U02", (enc_l_count() == left) && (enc_r_count() == right),
               "encoder counts steady while braked");

    /* ---- Argument checks (before calibration is looked at) ------------- */
    TEST_CHECK("S2-U03",
               (moveForward(-0.1f) == TEST_BAD_ARG) &&
                   (moveBackward(-0.1f) == TEST_BAD_ARG),
               "negative distance rejected");

    TEST_CHECK("S2-U04",
               (turnLeft(not_a_number) == TEST_BAD_ARG) &&
                   (turnRight(-5.0f) == TEST_BAD_ARG),
               "NaN / negative angle rejected");

    if (!motion_is_calibrated())
    {
        TEST_CHECK("S2-U05",
                   (moveForward(0.1f) == (unsigned)MOTION_ERR_NOT_CALIBRATED) &&
                       (motion_request_velocity(0.0f, 0.0f) ==
                        (unsigned)MOTION_ERR_NOT_CALIBRATED),
                   "moves rejected until calibrated");
        skip("S2-U06..U22", "ENCODER_COUNTS_PER_REV not set in motion.c");
        TEST_SUMMARY("subsystem2");

        return;
    }
    skip("S2-U05", "robot is calibrated");

    /* ---- Statechart (robot does not move) ------------------------------ */
    b_met = (motion_request_drive(0.0f, 0.2f) == (unsigned)MOTION_OK);
    b_met = b_met && (motion_poll() == (unsigned)MOTION_COMPLETE);
    b_met = b_met && (motion_poll() == (unsigned)MOTION_IDLE);
    TEST_CHECK("S2-U06", b_met, "COMPLETE reported once, then IDLE");

    TEST_CHECK("S2-U07",
               (motion_request_drive(0.1f, 0.0f) == TEST_BAD_ARG) &&
                   (motion_request_turn(90.0f, -0.1f) == TEST_BAD_ARG),
               "zero / negative speed rejected");

    b_met = (motion_request_velocity(0.0f, 0.0f) == (unsigned)MOTION_OK);
    b_met = b_met && (motion_poll() == (unsigned)MOTION_VELOCITY);
    b_met = b_met && (moveForward(0.1f) == (unsigned)MOTION_ERR_BUSY);
    stop();
    (void)tk_dly_tsk(STOP_WAIT_MS);
    b_met = b_met && (motion_poll() == (unsigned)MOTION_IDLE);
    TEST_CHECK("S2-U08", b_met, "VELOCITY busy for moves; stop() -> IDLE");
    b_met = (motion_request_velocity(0.0f, 0.0f) == (unsigned)MOTION_OK);
    (void)tk_dly_tsk(STALE_WAIT_MS);
    b_met = b_met && (motion_poll() == (unsigned)MOTION_FAULT);
    b_met =
        b_met && (motion_get_fault() == (unsigned)MOTION_FAULT_STALE_COMMAND);
    b_met = b_met && (motion_get_fault_bits() ==
                      motion_pack_fault_bit(MOTION_FAULT_STALE_COMMAND));
    b_met = b_met && (moveForward(0.1f) == (unsigned)MOTION_ERR_FAULT);
    TEST_CHECK("S2-U09", b_met, "stale velocity command -> MOTION FAULT");

    (void)tk_dly_tsk(RESET_WAIT_MS);
    b_met = motion_reset();
    b_met = b_met && (motion_poll() == (unsigned)MOTION_IDLE);
    b_met = b_met && (motion_get_fault() == (unsigned)MOTION_FAULT_NONE) &&
            (motion_get_fault_bits() == 0U);
    TEST_CHECK("S2-U10", b_met, "motion_reset() clears the fault");

    /* ---- Command IDs and reports (robot does not move) ----------------- */
    while (motion_get_report(&rep))
    {
        /* Drain reports left by S2-U06 through U10. */
    }
    b_met = (motion_request_drive(0.0f, 0.2f) == (unsigned)MOTION_OK);
    id1   = motion_command_id();
    b_met = b_met && (id1 != 0U) && motion_get_report(&rep);
    b_met = b_met && (rep.command_id == id1) &&
            ((unsigned)MOTION_OUTCOME_DONE == rep.outcome);
    b_met = b_met && (motion_poll() == TEST_IDLE) && !motion_get_report(&rep);
    TEST_CHECK("S2-U13", b_met, "DONE report carries the command ID, once");

    b_met = (motion_request_velocity(0.0f, 0.0f) == (unsigned)MOTION_OK);
    id2   = motion_command_id();
    b_met = b_met && (id2 != id1);
    b_met = b_met && (motion_request_velocity(0.0f, 0.0f) == TEST_OK) &&
            (motion_command_id() == id2);
    b_met = b_met && (motion_cancel(id1) == (unsigned)MOTION_ERR_OBSOLETE);
    b_met = b_met && (motion_cancel(id2) == (unsigned)MOTION_OK);
    (void)tk_dly_tsk(STOP_WAIT_MS);
    b_met = b_met && motion_get_report(&rep) && (rep.command_id == id2) &&
            ((unsigned)MOTION_OUTCOME_CANCELLED == rep.outcome);
    b_met = b_met && (motion_poll() == (unsigned)MOTION_IDLE);
    TEST_CHECK("S2-U14", b_met, "cancel: old ID rejected, report cancelled");

    b_met = (motion_request_velocity(0.0f, 0.0f) == (unsigned)MOTION_OK);
    id2   = motion_command_id();
    (void)tk_dly_tsk(STALE_WAIT_MS);
    b_met = b_met && motion_get_report(&rep) && (rep.command_id == id2) &&
            ((unsigned)MOTION_OUTCOME_FAULT == rep.outcome) &&
            ((unsigned)MOTION_FAULT_STALE_COMMAND == rep.fault);
    stop();
    b_met = b_met && (motion_cancel(id2) == (unsigned)MOTION_ERR_FAULT) &&
            (motion_poll() == (unsigned)MOTION_FAULT);
    TEST_CHECK("S2-U15", b_met,
               "fault report carries the ID; stop keeps fault");
    (void)tk_dly_tsk(RESET_WAIT_MS);
    b_met = motion_reset() && motion_is_stopped();
    TEST_CHECK("S2-U16", b_met, "stopped acknowledgement after reset");

    /* ---- Report queue (robot does not move) ---------------------------- */
    b_met = (motion_request_velocity(0.0f, 0.0f) == (unsigned)MOTION_OK);
    id1   = motion_command_id();
    stop();                         /* Report cancellation first. */
    (void)tk_dly_tsk(STOP_WAIT_MS); /* Wait for the brake. */
    b_met = b_met && (motion_request_drive(0.0f, 0.2f) == (unsigned)MOTION_OK);
    id2   = motion_command_id(); /* Report completion second. */
    b_met = b_met && motion_get_report(&rep) && (rep.command_id == id1) &&
            ((unsigned)MOTION_OUTCOME_CANCELLED == rep.outcome);
    b_met = b_met && motion_get_report(&rep) && (rep.command_id == id2) &&
            ((unsigned)MOTION_OUTCOME_DONE == rep.outcome);
    b_met = b_met && (motion_poll() == (unsigned)MOTION_IDLE) &&
            (motion_report_peak() >= 2U);
    TEST_CHECK("S2-U17", b_met, "two reports queued, read in order");
    /* Fill one report past the queue capacity. */
    for (index = 0U; index <= MOTION_REPORT_QUEUE_LEN; index++)
    {
        (void)motion_request_drive(0.0f, 0.2f);
        (void)motion_poll(); /* COMPLETE -> IDLE        */
    }
    count = 0U;
    while (motion_get_report(&rep))
    {
        count++;
    } /* clang-format off */
    b_met = (MOTION_REPORT_QUEUE_LEN == count);
    b_met = b_met && (motion_request_drive(0.0f, 0.2f) ==
        (unsigned)MOTION_OK) && motion_get_report(&rep) && rep.b_was_lost;
    b_met = b_met && (motion_request_drive(0.0f, 0.2f) ==
        (unsigned)MOTION_OK) && motion_get_report(&rep) && !rep.b_was_lost;
    TEST_CHECK("S2-U18", b_met,
        "full queue: no wait, next report marks lost"); /* clang-format on */
    /* ---- Minimum-bit packing (no motion) ------------------------------- */
    {
        motion_status_t   status = {MOTION_TURN, MOTION_FAULT_TIMEOUT,
                                    255U,        0.27f,
                                    -0.412f,     -1.234f,
                                    -90.0f,      true,
                                    false,       true,
                                    0U};
        motion_status_t   st2;
        motion_report_t   report = {255U, MOTION_OUTCOME_FAULT,
                                    MOTION_FAULT_TIMEOUT, true};
        motion_report_t   rp2;
        motion_pack_cmd_t cmd;

        motion_pack_read_status(motion_pack_status(&status), &st2);
        b_met =
            (st2.fault_bits == motion_pack_fault_bit(MOTION_FAULT_TIMEOUT)) &&
            (st2.state == status.state) && (st2.fault == status.fault) &&
            (255U == st2.command_id) &&
            near_f(st2.speed_left_mps, 0.27f, MOTION_PACK_SPEED_STEP_MPS) &&
            near_f(st2.speed_right_mps, -0.412f, MOTION_PACK_SPEED_STEP_MPS) &&
            near_f(st2.distance_m, -1.234f, MOTION_PACK_DIST_STEP_M) &&
            near_f(st2.heading_deg, 270.0f, 1.0f) && st2.b_is_stopped &&
            !st2.b_is_busy && st2.b_is_status_valid;

        /* Saturate and encode unsigned zero. */
        status.speed_left_mps  = 2.0f;
        status.speed_right_mps = -0.0004f;
        status.distance_m      = 40.0f;
        motion_pack_read_status(motion_pack_status(&status), &st2);
        b_met = b_met && near_f(st2.speed_left_mps, 1.022f, 0.0f) &&
                (0.0f == st2.speed_right_mps) &&
                near_f(st2.distance_m, 32.767f, 0.0f);
        /* clang-format off */
        status.fault      = MOTION_FAULT_NONE;
        status.fault_bits =
            (unsigned)motion_pack_fault_bit(MOTION_FAULT_STALL) |
            (unsigned)motion_pack_fault_bit(MOTION_FAULT_ENCODER);
        motion_pack_read_status(motion_pack_status(&status), &st2);
        b_met = b_met && (st2.fault_bits == status.fault_bits) &&
               ((unsigned)MOTION_FAULT_STALL == st2.fault);
        /* clang-format on */
        motion_pack_read_report(motion_pack_report(&report), &rp2);
        b_met = b_met && (motion_pack_report(&report) < (1U << 14)) &&
                (255U == rp2.command_id) &&
                ((unsigned)MOTION_OUTCOME_FAULT == rp2.outcome) &&
                ((unsigned)MOTION_FAULT_TIMEOUT == rp2.fault) && rp2.b_was_lost;

        b_met = b_met &&
                motion_pack_decode(motion_pack_velocity(0.1f, -0.2f), &cmd) &&
                ((unsigned)MOTION_CMD_VELOCITY == cmd.type) &&
                near_f(cmd.left_mps, 0.1f, MOTION_PACK_SPEED_STEP_MPS) &&
                near_f(cmd.right_mps, -0.2f, MOTION_PACK_SPEED_STEP_MPS);
        b_met = b_met &&
                motion_pack_decode(motion_pack_turn(-90.0f, 0.15f), &cmd) &&
                ((unsigned)MOTION_CMD_TURN == cmd.type) &&
                near_f(cmd.degrees, -90.0f, 1.0f) &&
                near_f(cmd.speed_mps, 0.15f, MOTION_PACK_SPEED_STEP_MPS);
        b_met = b_met &&
                motion_pack_decode(motion_pack_drive(-0.5f, 0.2f), &cmd) &&
                near_f(cmd.meters, -0.5f, MOTION_PACK_DIST_STEP_M);
        /* The 90% top command stays below the clipping limit. */
        b_met = b_met &&
                motion_pack_decode(motion_pack_velocity(0.8f, -0.8f), &cmd) &&
                near_f(cmd.left_mps, 0.8f, MOTION_PACK_SPEED_STEP_MPS) &&
                near_f(cmd.right_mps, -0.8f, MOTION_PACK_SPEED_STEP_MPS);
        b_met = b_met &&
                motion_pack_decode(motion_pack_drive(1.0f, 0.8f), &cmd) &&
                near_f(cmd.speed_mps, 0.8f, MOTION_PACK_SPEED_STEP_MPS);
        b_met = b_met && motion_pack_decode(motion_cmd_cancel(7U), &cmd) &&
                (TEST_CANCEL == cmd.type) && (7U == cmd.command_id);
        b_met = b_met && !motion_pack_decode(7U, &cmd); /* Reject type 7. */
        TEST_CHECK("S2-U19", b_met, "pack / unpack: values, limits, sizes");
    }
    b_met = (motion_request_packed(motion_pack_velocity(0.0f, 0.0f)) ==
             (unsigned)MOTION_OK);
    id1   = motion_command_id();
    b_met = b_met && (motion_poll() == (unsigned)MOTION_VELOCITY);
    b_met = b_met && (motion_request_packed(motion_cmd_cancel(id1)) == TEST_OK);
    (void)tk_dly_tsk(STOP_WAIT_MS);
    b_met = b_met && motion_get_report(&rep) && (rep.command_id == id1) &&
            ((unsigned)MOTION_OUTCOME_CANCELLED == rep.outcome);
    b_met =
        b_met && (motion_request_packed(7U) == (unsigned)MOTION_ERR_BAD_ARG);
    b_met = b_met && (((uint32_t)motion_get_status_packed() & 0x7U) ==
                      (uint32_t)MOTION_IDLE);
    TEST_CHECK("S2-U20", b_met, "packed command and status word");

    /* ---- Moves (robot drives) ------------------------------------------ */
#if S2_TEST_MOVE
    {
        float32_t origin_m = robot_distance_m();
        float32_t yaw_deg;

        b_met = (moveForward(MOVE_DIST_M) == (unsigned)MOTION_OK);
        b_met = b_met && (wait_move() == (unsigned)MOTION_COMPLETE);
        b_met = b_met && (abs_f(robot_distance_m() - origin_m - MOVE_DIST_M) <=
                          MOVE_TOL_M);
        TEST_CHECK("S2-U11", b_met, "moveForward(0.20) within 2 cm");

        (void)tk_dly_tsk(STILL_WAIT_MS);
        yaw_deg = robot_heading_deg();
        b_met   = (turnLeft(TURN_DEG) == (unsigned)MOTION_OK);
        b_met   = b_met && (wait_move() == (unsigned)MOTION_COMPLETE);
        b_met   = b_met && (abs_f(robot_heading_deg() - yaw_deg - TURN_DEG) <=
                          TURN_TOL_DEG);
        TEST_CHECK("S2-U12", b_met, "turnLeft(90) within 10 deg");

        /* Turn right with the left wheel forward. */
        (void)tk_dly_tsk(STILL_WAIT_MS);
        yaw_deg = robot_heading_deg();
        b_met   = (turnRight(TURN_DEG) == (unsigned)MOTION_OK);
        b_met   = b_met && (wait_move() == (unsigned)MOTION_COMPLETE);
        b_met   = b_met && (abs_f(robot_heading_deg() - yaw_deg + TURN_DEG) <=
                          TURN_TOL_DEG);
        TEST_CHECK("S2-U21", b_met, "turnRight(90) within 10 deg");
        /* clang-format off */
        /* Reverse both wheels to return to the start. */
        (void)tk_dly_tsk(STILL_WAIT_MS);
        origin_m = robot_distance_m();
        b_met    = (moveBackward(MOVE_DIST_M) == (unsigned)MOTION_OK);
        b_met    = b_met && (wait_move() == (unsigned)MOTION_COMPLETE);
        b_met    = b_met && (abs_f(robot_distance_m() - origin_m + MOVE_DIST_M)
                    <= MOVE_TOL_M); /* clang-format on */
        TEST_CHECK("S2-U22", b_met, "moveBackward(0.20) within 2 cm");

        if (motion_poll() == (unsigned)MOTION_FAULT)
        {
            tm_printf((UB *)"[TEST] subsystem2 fault: %s\n",
                      motion_fault_name(motion_get_fault()));
        }

        stop();
    }
#else
    skip("S2-U11, U12, U21, U22",
         "build with S2_TEST_MOVE=1 to drive the robot");
#endif /* S2_TEST_MOVE */

    TEST_SUMMARY("subsystem2");
}

/** @brief Wait for startup, then run motor calibration or unit checks as
 * selected at build time. Report through the device test console. */
void subsystem2_test (void)
{
    (void)tk_dly_tsk(STARTUP_WAIT_MS);

    if (S2_TEST_CALIBRATE)
    {
        calibrate();
        skip("S2-U01..U22", "S2_TEST_CALIBRATE is 1");
    }
    else
    {
        run_unit_tests();
    }
}

#endif /* APP_TEST */

/* End of file. */

