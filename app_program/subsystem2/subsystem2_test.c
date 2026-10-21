/*
 * File: subsystem2_test.c
 * Owner: Buddy 2 - Motion control using motors and wheel encoders.
 * Purpose: On-target unit tests (make TEST=1). IDs match
 *          testing/subsystem2/TEST_LOG.md.
 *
 * S2-U01..U10 and S2-U13..U20 never move the robot: every velocity target is
 * 0 and every drive request has zero length. S2-U11 / U12 / U21 / U22 drive
 * forward, turn left, turn right and drive back, ending where they started;
 * they run only when S2_TEST_MOVE is 1 (robot on the floor, 30 cm
 * clear ahead).
 * S2-U05..U20 need the CALIBRATE values in motion.c and report SKIP without.
 *
 * S2_TEST_CALIBRATE = 1 runs calibration instead of the tests and prints the
 * CALIBRATE values for motion.c. Follow the [CAL] prompts on the console.
 */

#include "subsystem2.h"
#include "motion.h"
#include "motion_pack.h"

#if APP_TEST

#include "../test_report.h"

#ifndef S2_TEST_MOVE
#define S2_TEST_MOVE            (0)     /* 1 = run the tests that move   */
#endif

#ifndef S2_TEST_CALIBRATE
#define S2_TEST_CALIBRATE       (0)     /* 1 = calibrate, skip the tests  */
#endif

#define STARTUP_WAIT_MS         (100)   /* motion task runs motion_init() */
#define CAL_PRINT_MS            (1000)  /* encoder counts while waiting   */
#define STILL_WAIT_MS           (200)
#define STOP_WAIT_MS            (20)    /* two control periods            */
#define STALE_WAIT_MS           (400)   /* > 250 ms velocity timeout      */
#define RESET_WAIT_MS           (300)   /* > 200 ms "wheels still"        */
#define MOVE_TIMEOUT_MS         (10000)
#define POLL_MS                 (20)

#define MOVE_DIST_M             (0.20f)
#define MOVE_TOL_M              (0.02f)
#define TURN_DEG                (90.0f)
#define TURN_TOL_DEG            (10.0f)

static void skip(const char *p_id, const char *p_why)
{
    tm_printf((UB *) "[TEST] %s SKIP  %s\n", p_id, p_why);
}

static float abs_f(float x)
{
    return (x < 0.0f) ? -x : x;
}

/* True when a packed value came back within half a step */
static bool near_f(float a, float b, float step)
{
    return abs_f(a - b) <= ((0.5f * step) + 0.0001f);
}

#if S2_TEST_MOVE

/* Poll until the move finishes; returns MOTION_COMPLETE, MOTION_FAULT or the
 * state at the time-out */
static motion_state_t wait_move(void)
{
    motion_state_t s = motion_poll();
    int32_t        waited = 0;

    while ((s != MOTION_COMPLETE) && (s != MOTION_FAULT)
           && (waited < MOVE_TIMEOUT_MS))
    {
        (void)tk_dly_tsk(POLL_MS);
        waited += POLL_MS;
        s = motion_poll();
    }

    return s;
}
#endif /* S2_TEST_MOVE */

static const char *bool_text(bool b_value)
{
    return b_value ? "true" : "false";
}

/* Print a speed as a C float literal with 3 decimals, e.g. (0.312f) */
static void print_mps(const char *p_name, float mps)
{
    int32_t mm = (int32_t)((mps * 1000.0f) + 0.5f);

    tm_printf((UB *) "#define %-23s (%d.%03df)\n", p_name, (int)(mm / 1000),
              (int)(mm % 1000));
}

/* Run motion_request_calibration() and print the CALIBRATE block */
static void calibrate(void)
{
    motion_calibration_t cal;

    tm_printf((UB *) "[CAL] 1. Turn EACH wheel by hand exactly 10 full turns"
              " forward,\n");
    tm_printf((UB *) "[CAL]    then leave both wheels still for 3 s.\n");
    tm_printf((UB *) "[CAL] 2. Lift the robot: 5 s later each motor runs at"
              " full power.\n");

    if (motion_request_calibration() != MOTION_OK)
    {
        tm_printf((UB *) "[CAL] could not start: robot not IDLE\n");

        return;
    }
    while (motion_is_calibrating())
    {
        tm_printf((UB *) "[CAL] counts L=%d R=%d\n", (int)enc_l_count(),
                  (int)enc_r_count());
        (void)tk_dly_tsk(CAL_PRINT_MS);
    }
    if (!motion_get_calibration(&cal))
    {
        tm_printf((UB *) "[CAL] no result\n");

        return;
    }
    if (!cal.b_enc_l_ok || !cal.b_enc_r_ok)
    {
        tm_printf((UB *) "[CAL] FAILED: encoder not counting (L ok=%s,"
                  " R ok=%s). Check A/B wiring and 3.3 V.\n",
                  bool_text(cal.b_enc_l_ok), bool_text(cal.b_enc_r_ok));

        return;
    }

    tm_printf((UB *) "[CAL] counts per turn L=%d R=%d (should agree within"
              " 5 %%)\n", (int)(cal.counts_per_rev_l + 0.5f),
              (int)(cal.counts_per_rev_r + 0.5f));
    tm_printf((UB *) "[CAL] top speed L=%d R=%d mm/s, encoders swapped=%s\n",
              (int)((cal.top_speed_l_mps * 1000.0f) + 0.5f),
              (int)((cal.top_speed_r_mps * 1000.0f) + 0.5f),
              bool_text(cal.b_encoders_swapped));
    tm_printf((UB *) "[CAL] result %s\n",
              cal.b_valid ? "VALID: copy into motion.c"
                          : "NOT VALID: fix the problem above and retry");

    tm_printf((UB *) "#define ENCODER_COUNTS_PER_REV  (%d.0f)\n",
              (int)(cal.counts_per_rev + 0.5f));
    print_mps("MOTOR_MAX_SPEED_MPS", cal.max_speed_mps);
    tm_printf((UB *) "#define M1_INVERTED             (%s)\n",
              bool_text(cal.b_m1_inverted));
    tm_printf((UB *) "#define M2_INVERTED             (%s)\n",
              bool_text(cal.b_m2_inverted));
    tm_printf((UB *) "#define ENC_L_INVERTED          (%s)\n",
              bool_text(cal.b_enc_l_inverted));
    tm_printf((UB *) "#define ENC_R_INVERTED          (%s)\n",
              bool_text(cal.b_enc_r_inverted));
}

/* S2-U01..U20; see the file header */
static void run_unit_tests(void)
{
    volatile float zero = 0.0f;
    float          not_a_number = zero / zero;
    int32_t        l0;
    int32_t        r0;
    bool           b_ok;
    uint8_t        id1;
    uint8_t        id2;
    uint32_t       i;
    uint32_t       n;
    motion_report_t rep;

    TEST_INIT();

    /* ---- Start-up ------------------------------------------------------ */
    TEST_CHECK("S2-U01",
               (motion_poll() == MOTION_IDLE)
               && (motion_get_fault() == MOTION_FAULT_NONE),
               "starts in IDLE with no fault");

    l0 = enc_l_count();
    r0 = enc_r_count();
    (void)tk_dly_tsk(STILL_WAIT_MS);
    TEST_CHECK("S2-U02",
               (enc_l_count() == l0) && (enc_r_count() == r0),
               "encoder counts steady while braked");

    /* ---- Argument checks (before calibration is looked at) ------------- */
    TEST_CHECK("S2-U03",
               (moveForward(-0.1f) == MOTION_ERR_BAD_ARG)
               && (moveBackward(-0.1f) == MOTION_ERR_BAD_ARG),
               "negative distance rejected");

    TEST_CHECK("S2-U04",
               (turnLeft(not_a_number) == MOTION_ERR_BAD_ARG)
               && (turnRight(-5.0f) == MOTION_ERR_BAD_ARG),
               "NaN / negative angle rejected");

    if (!motion_is_calibrated())
    {
        TEST_CHECK("S2-U05",
                   (moveForward(0.1f) == MOTION_ERR_NOT_CALIBRATED)
                   && (motion_request_velocity(0.0f, 0.0f)
                       == MOTION_ERR_NOT_CALIBRATED),
                   "moves rejected until calibrated");
        skip("S2-U06..U20", "ENCODER_COUNTS_PER_REV not set in motion.c");
        TEST_SUMMARY("subsystem2");

        return;
    }
    skip("S2-U05", "robot is calibrated");

    /* ---- Statechart (robot does not move) ------------------------------ */
    b_ok = (motion_request_drive(0.0f, 0.2f) == MOTION_OK);
    b_ok = b_ok && (motion_poll() == MOTION_COMPLETE);
    b_ok = b_ok && (motion_poll() == MOTION_IDLE);
    TEST_CHECK("S2-U06", b_ok, "COMPLETE reported once, then IDLE");

    TEST_CHECK("S2-U07",
               (motion_request_drive(0.1f, 0.0f) == MOTION_ERR_BAD_ARG)
               && (motion_request_turn(90.0f, -0.1f) == MOTION_ERR_BAD_ARG),
               "zero / negative speed rejected");

    b_ok = (motion_request_velocity(0.0f, 0.0f) == MOTION_OK);
    b_ok = b_ok && (motion_poll() == MOTION_VELOCITY);
    b_ok = b_ok && (moveForward(0.1f) == MOTION_ERR_BUSY);
    stop();
    (void)tk_dly_tsk(STOP_WAIT_MS);
    b_ok = b_ok && (motion_poll() == MOTION_IDLE);
    TEST_CHECK("S2-U08", b_ok, "VELOCITY busy for moves; stop() -> IDLE");

    b_ok = (motion_request_velocity(0.0f, 0.0f) == MOTION_OK);
    (void)tk_dly_tsk(STALE_WAIT_MS);
    b_ok = b_ok && (motion_poll() == MOTION_FAULT);
    b_ok = b_ok && (motion_get_fault() == MOTION_FAULT_STALE_COMMAND);
    b_ok = b_ok && (motion_get_fault_bits()
                    == MOTION_FAULT_BIT(MOTION_FAULT_STALE_COMMAND));
    b_ok = b_ok && (moveForward(0.1f) == MOTION_ERR_FAULT);
    TEST_CHECK("S2-U09", b_ok, "stale velocity command -> MOTION FAULT");

    (void)tk_dly_tsk(RESET_WAIT_MS);
    b_ok = motion_reset();
    b_ok = b_ok && (motion_poll() == MOTION_IDLE);
    b_ok = b_ok && (motion_get_fault() == MOTION_FAULT_NONE)
           && (motion_get_fault_bits() == 0U);
    TEST_CHECK("S2-U10", b_ok, "motion_reset() clears the fault");

    /* ---- Command IDs and reports (robot does not move) ----------------- */
    while (motion_get_report(&rep))
    {
        /* drop the reports left by S2-U06..U10 */
    }
    b_ok = (motion_request_drive(0.0f, 0.2f) == MOTION_OK);
    id1 = motion_command_id();
    b_ok = b_ok && (id1 != 0U) && motion_get_report(&rep);
    b_ok = b_ok && (rep.command_id == id1)
           && (rep.outcome == MOTION_OUTCOME_DONE);
    b_ok = b_ok && (motion_poll() == MOTION_IDLE) && !motion_get_report(&rep);
    TEST_CHECK("S2-U13", b_ok, "DONE report carries the command ID, once");

    b_ok = (motion_request_velocity(0.0f, 0.0f) == MOTION_OK);
    id2 = motion_command_id();
    b_ok = b_ok && (id2 != id1);
    b_ok = b_ok && (motion_request_velocity(0.0f, 0.0f) == MOTION_OK)
           && (motion_command_id() == id2);
    b_ok = b_ok && (motion_cancel(id1) == MOTION_ERR_OBSOLETE);
    b_ok = b_ok && (motion_cancel(id2) == MOTION_OK);
    (void)tk_dly_tsk(STOP_WAIT_MS);
    b_ok = b_ok && motion_get_report(&rep) && (rep.command_id == id2)
           && (rep.outcome == MOTION_OUTCOME_CANCELLED);
    b_ok = b_ok && (motion_poll() == MOTION_IDLE);
    TEST_CHECK("S2-U14", b_ok, "cancel: old ID rejected, report cancelled");

    b_ok = (motion_request_velocity(0.0f, 0.0f) == MOTION_OK);
    id2 = motion_command_id();
    (void)tk_dly_tsk(STALE_WAIT_MS);
    b_ok = b_ok && motion_get_report(&rep) && (rep.command_id == id2)
           && (rep.outcome == MOTION_OUTCOME_FAULT)
           && (rep.fault == MOTION_FAULT_STALE_COMMAND);
    stop();
    b_ok = b_ok && (motion_cancel(id2) == MOTION_ERR_FAULT)
           && (motion_poll() == MOTION_FAULT);
    TEST_CHECK("S2-U15", b_ok, "fault report carries the ID; stop keeps fault");

    (void)tk_dly_tsk(RESET_WAIT_MS);
    b_ok = motion_reset() && motion_is_stopped();
    TEST_CHECK("S2-U16", b_ok, "stopped acknowledgement after reset");

    /* ---- Report queue (robot does not move) ---------------------------- */
    b_ok = (motion_request_velocity(0.0f, 0.0f) == MOTION_OK);
    id1 = motion_command_id();
    stop();                                         /* report 1: cancelled */
    b_ok = b_ok && (motion_request_drive(0.0f, 0.2f) == MOTION_OK);
    id2 = motion_command_id();                      /* report 2: done      */
    b_ok = b_ok && motion_get_report(&rep) && (rep.command_id == id1)
           && (rep.outcome == MOTION_OUTCOME_CANCELLED);
    b_ok = b_ok && motion_get_report(&rep) && (rep.command_id == id2)
           && (rep.outcome == MOTION_OUTCOME_DONE);
    b_ok = b_ok && (motion_poll() == MOTION_IDLE)
           && (motion_report_peak() >= 2U);
    TEST_CHECK("S2-U17", b_ok, "two reports queued, read in order");

    for (i = 0U; i <= MOTION_REPORT_QUEUE_LEN; i++)  /* one more than fits */
    {
        (void)motion_request_drive(0.0f, 0.2f);
        (void)motion_poll();                    /* COMPLETE -> IDLE        */
    }
    n = 0U;
    while (motion_get_report(&rep))
    {
        n++;
    }
    b_ok = (n == MOTION_REPORT_QUEUE_LEN);
    b_ok = b_ok && (motion_request_drive(0.0f, 0.2f) == MOTION_OK)
           && motion_get_report(&rep) && rep.b_lost;
    b_ok = b_ok && (motion_request_drive(0.0f, 0.2f) == MOTION_OK)
           && motion_get_report(&rep) && !rep.b_lost;
    TEST_CHECK("S2-U18", b_ok, "full queue: no wait, next report marks lost");

    /* ---- Minimum-bit packing (no motion) ------------------------------- */
    {
        motion_status_t st = { MOTION_TURN, MOTION_FAULT_TIMEOUT, 255U,
                               0.27f, -0.412f, -1.234f, -90.0f,
                               true, false, true, 0U };
        motion_status_t st2;
        motion_report_t rp = { 255U, MOTION_OUTCOME_FAULT,
                               MOTION_FAULT_TIMEOUT, true };
        motion_report_t rp2;
        motion_cmd_t    cmd;
        uint64_t        word;

        word = motion_pack_status(&st);
        motion_unpack_status(word, &st2);
        b_ok = (st2.fault_bits == MOTION_FAULT_BIT(MOTION_FAULT_TIMEOUT))
               && (st2.state == st.state) && (st2.fault == st.fault)
               && (st2.command_id == 255U)
               && near_f(st2.speed_left_mps, 0.27f,
                         MOTION_PACK_SPEED_STEP_MPS)
               && near_f(st2.speed_right_mps, -0.412f,
                         MOTION_PACK_SPEED_STEP_MPS)
               && near_f(st2.distance_m, -1.234f, MOTION_PACK_DIST_STEP_M)
               && near_f(st2.heading_deg, 270.0f, 1.0f)
               && st2.b_stopped && !st2.b_busy && st2.b_valid;

        /* Saturation, no negative zero */
        st.speed_left_mps = 2.0f;
        st.speed_right_mps = -0.0004f;
        st.distance_m = 40.0f;
        motion_unpack_status(motion_pack_status(&st), &st2);
        b_ok = b_ok && near_f(st2.speed_left_mps, 1.022f, 0.0f)
               && (st2.speed_right_mps == 0.0f)
               && near_f(st2.distance_m, 32.767f, 0.0f);

        /* Two faults at once: both bits survive, the lowest is the code */
        st.fault = MOTION_FAULT_NONE;
        st.fault_bits = MOTION_FAULT_BIT(MOTION_FAULT_STALL)
                        | MOTION_FAULT_BIT(MOTION_FAULT_ENCODER);
        motion_unpack_status(motion_pack_status(&st), &st2);
        b_ok = b_ok && (st2.fault_bits == st.fault_bits)
               && (st2.fault == MOTION_FAULT_STALL);

        motion_unpack_report(motion_pack_report(&rp), &rp2);
        b_ok = b_ok && (motion_pack_report(&rp) < (1U << 14))
               && (rp2.command_id == 255U)
               && (rp2.outcome == MOTION_OUTCOME_FAULT)
               && (rp2.fault == MOTION_FAULT_TIMEOUT) && rp2.b_lost;

        b_ok = b_ok && motion_unpack_cmd(motion_cmd_velocity(0.1f, -0.2f),
                                         &cmd)
               && (cmd.type == MOTION_CMD_VELOCITY)
               && near_f(cmd.left_mps, 0.1f, MOTION_PACK_SPEED_STEP_MPS)
               && near_f(cmd.right_mps, -0.2f, MOTION_PACK_SPEED_STEP_MPS);
        b_ok = b_ok && motion_unpack_cmd(motion_cmd_turn(-90.0f, 0.15f), &cmd)
               && (cmd.type == MOTION_CMD_TURN)
               && near_f(cmd.degrees, -90.0f, 1.0f)
               && near_f(cmd.speed_mps, 0.15f, MOTION_PACK_SPEED_STEP_MPS);
        b_ok = b_ok && motion_unpack_cmd(motion_cmd_drive(-0.5f, 0.2f), &cmd)
               && near_f(cmd.meters, -0.5f, MOTION_PACK_DIST_STEP_M);
        /* Top commanded speed (90 % of MOTOR_MAX_SPEED_MPS) is not clipped */
        b_ok = b_ok && motion_unpack_cmd(motion_cmd_velocity(0.8f, -0.8f),
                                         &cmd)
               && near_f(cmd.left_mps, 0.8f, MOTION_PACK_SPEED_STEP_MPS)
               && near_f(cmd.right_mps, -0.8f, MOTION_PACK_SPEED_STEP_MPS);
        b_ok = b_ok && motion_unpack_cmd(motion_cmd_drive(1.0f, 0.8f), &cmd)
               && near_f(cmd.speed_mps, 0.8f, MOTION_PACK_SPEED_STEP_MPS);
        b_ok = b_ok && motion_unpack_cmd(motion_cmd_cancel(7U), &cmd)
               && (cmd.type == MOTION_CMD_CANCEL) && (cmd.command_id == 7U);
        b_ok = b_ok && !motion_unpack_cmd(7U, &cmd);    /* unknown type */
        TEST_CHECK("S2-U19", b_ok, "pack / unpack: values, limits, sizes");
    }

    b_ok = (motion_request_packed(motion_cmd_velocity(0.0f, 0.0f))
            == MOTION_OK);
    id1 = motion_command_id();
    b_ok = b_ok && (motion_poll() == MOTION_VELOCITY);
    b_ok = b_ok && (motion_request_packed(motion_cmd_cancel(id1))
                    == MOTION_OK);
    (void)tk_dly_tsk(STOP_WAIT_MS);
    b_ok = b_ok && motion_get_report(&rep) && (rep.command_id == id1)
           && (rep.outcome == MOTION_OUTCOME_CANCELLED);
    b_ok = b_ok && (motion_request_packed(7U) == MOTION_ERR_BAD_ARG);
    b_ok = b_ok && (((uint32_t)motion_get_status_packed() & 0x7U)
                    == (uint32_t)MOTION_IDLE);
    TEST_CHECK("S2-U20", b_ok, "packed command and status word");

    /* ---- Moves (robot drives) ------------------------------------------ */
#if S2_TEST_MOVE
    {
        float d0 = robot_distance_m();
        float h0;

        b_ok = (moveForward(MOVE_DIST_M) == MOTION_OK);
        b_ok = b_ok && (wait_move() == MOTION_COMPLETE);
        b_ok = b_ok
               && (abs_f(robot_distance_m() - d0 - MOVE_DIST_M) <= MOVE_TOL_M);
        TEST_CHECK("S2-U11", b_ok, "moveForward(0.20) within 2 cm");

        (void)tk_dly_tsk(STILL_WAIT_MS);
        h0 = robot_heading_deg();
        b_ok = (turnLeft(TURN_DEG) == MOTION_OK);
        b_ok = b_ok && (wait_move() == MOTION_COMPLETE);
        b_ok = b_ok
               && (abs_f(robot_heading_deg() - h0 - TURN_DEG) <= TURN_TOL_DEG);
        TEST_CHECK("S2-U12", b_ok, "turnLeft(90) within 10 deg");

        /* The other turn sign: left wheel forward, right wheel back */
        (void)tk_dly_tsk(STILL_WAIT_MS);
        h0 = robot_heading_deg();
        b_ok = (turnRight(TURN_DEG) == MOTION_OK);
        b_ok = b_ok && (wait_move() == MOTION_COMPLETE);
        b_ok = b_ok
               && (abs_f(robot_heading_deg() - h0 + TURN_DEG) <= TURN_TOL_DEG);
        TEST_CHECK("S2-U21", b_ok, "turnRight(90) within 10 deg");

        /* Both wheels backward; ends back on the start mark */
        (void)tk_dly_tsk(STILL_WAIT_MS);
        d0 = robot_distance_m();
        b_ok = (moveBackward(MOVE_DIST_M) == MOTION_OK);
        b_ok = b_ok && (wait_move() == MOTION_COMPLETE);
        b_ok = b_ok
               && (abs_f(robot_distance_m() - d0 + MOVE_DIST_M) <= MOVE_TOL_M);
        TEST_CHECK("S2-U22", b_ok, "moveBackward(0.20) within 2 cm");

        if (motion_poll() == MOTION_FAULT)
        {
            tm_printf((UB *) "[TEST] subsystem2 fault: %s\n",
                      motion_fault_name(motion_get_fault()));
        }
        stop();
    }
#else
    skip("S2-U11, U12, U21, U22", "build with S2_TEST_MOVE=1 to drive the robot");
#endif /* S2_TEST_MOVE */

    TEST_SUMMARY("subsystem2");
}

void subsystem2_test(void)
{
    (void)tk_dly_tsk(STARTUP_WAIT_MS);
    if (S2_TEST_CALIBRATE)
    {
        calibrate();
        skip("S2-U01..U20", "S2_TEST_CALIBRATE is 1");
    }
    else
    {
        run_unit_tests();
    }
}

#endif /* APP_TEST */
