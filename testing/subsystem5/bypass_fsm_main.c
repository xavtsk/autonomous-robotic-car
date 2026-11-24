/** @file bypass_fsm_main.c
 * @brief Exercise scan codecs, avoidance planning, manoeuvres and line
 * recovery.
 * Unsigned casts spell out the original modulo conversion.
 * The selected narrow integer operands fit in int.
 * Packet fields and fixture masks are interpreted as unsigned bits.
 */
#include "bypass_fsm_main.h"
#include "../../app_program/app_types.h"
#include "avoid_obstacle.h"
#include "scan_obstacle.h"
#include "ultrasonic_sensor.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
#ifdef TEST_SEMANTIC_TRACE
#include "obstacle_trace.h"
#endif

#define PROFILE_SAMPLE_COUNT 5U
#define BACKUP_TURN_WAIT_TICKS 20U
#define SCAN_AIM_WAIT_TICKS 50U
#define DETOUR_FIRST_TURN_WAIT_TICKS 30U
#define SURVEY_SAMPLE_COUNT 12U
#define DETOUR_MOVEMENT_WAIT_TICKS 40U
#define FAR_LINE_CHECK_TICKS 100U

#define MIN_SCAN_ANGLE_DEG 30
#define MAX_SCAN_ANGLE_DEG 150U
#define MIN_ECHO_MM 30
#define MAX_ECHO_MM 4000U
#define BLOCKED_RETRY_LIMIT 3U
#define S8_NEGATIVE_MAGNITUDE 128
#define S8_MAX 127
#define S7_NEGATIVE_MAGNITUDE 64
#define S7_MAX 63

#define OBSERVATION_PACKET_CASES 20000
#define DEPTH_RUN_TICK_LIMIT 3000
#define LINE_RETURN_TICK_LIMIT 4000
#define RECOVERY_RUN_TICK_LIMIT 6000

/* Generic scripted bypass for the 17 Nov rules. Hooks: side reading for the
 * n-th peek, forward range by leg, tape by leg/pass. Records every action. */
typedef struct
{
    unsigned peek_side[8];
    unsigned
        side_peek_count; /* Follow the readings with 2000 mm of clearance. */
    unsigned outbound_range[4];
    unsigned
         outbound_count; /* Follow leg 1 readings with 2000 mm of clearance. */
    int  tape_leg;
    int  tape_pass;          /* Detect tape while that command is active. */
    int  alternate_tape_leg; /* Also detect tape on this leg unless zero. */
    bool b_has_finish_tape;
    bool b_lose_tape_once; /* Set tape visibility during FINISH. */
    unsigned closest;
    int      left_edge;
} scr_t;

typedef struct
{
    avoid_obstacle_action_kind_t kind;
    int                          value;
    unsigned                     leg;
    unsigned                     pass;
    unsigned                     speed;
} rec_t;

static unsigned g_peeks;
static bool     gb_is_first_side_only;
static int      g_turns[16];
static unsigned g_turn_count;
static unsigned g_outbound_drive_mm;
static unsigned g_return_drive_mm;

static rec_t    g_records[96];
static unsigned g_record_count;

/* Archived subject headers predate the declaration of this host-only hook. */
unsigned    pass_extra_test (const avoid_obstacle_t * p_detour);
static void fsm_depth (bool b_can_see_line, bool b_is_blocked,
                       bool b_has_invalid_range, unsigned deep);

static avoid_obstacle_action_t
step_and_observe (avoid_obstacle_t *             p_avoidance,
                  const avoid_obstacle_input_t * p_input);

static profile_obstacle_t profile (void);

static void codecs (void);

static void scans (void);
static void routing_count_tests (void);
static void face_width_tests (const s5_config_t *  p_cfg,
                              profile_obstacle_t * p_profile);

static void fsm (bool b_can_see_line, bool b_is_blocked,
                 bool b_has_invalid_range);

static void fsm_depth (bool b_can_see_line, bool b_is_blocked,
                       bool b_has_invalid_range, unsigned deep);

static void fsm_line (int found_leg, int * p_sweep, unsigned * p_back_drive);

static int fsm_backup (unsigned closest, int left_edge);

static int fsm_aim_turn (int left_edge, int right_edge, unsigned closest,
                         bool b_is_second_off, float64_t turn);

static int fsm_aim (int left_edge_mm, int right_edge_mm, unsigned clearance_mm,
                    bool b_is_off);

static void fsm_cross (float64_t turn, unsigned tape_at_mm, unsigned * p_drives,
                       unsigned * p_navigation, bool * pb_did_stop_early);

static void rules_17nov (void);

/** @brief Exercise scan packing and the avoidance state machine with
 * simulated routes, missing echoes, faults and line reacquisition.
 */
int main (void)
{
    /* The unsigned representation preserves every former signed bit-field
     * value, including zero initialization and untouched neighbouring bits. */
    {
        avoid_obstacle_t stored = {0};
        assert(avoid_obstacle_relative_get(&stored) == 0 &&
               avoid_obstacle_aim_get(&stored) == 0);
        stored.side_turn     = 13U;
        stored.side_turns    = 3U;
        stored.blocked_reads = 3U;

        for (int degrees = -S8_NEGATIVE_MAGNITUDE; degrees <= S8_MAX; ++degrees)
        {
            avoid_obstacle_relative_set(&stored, degrees);
            assert(avoid_obstacle_relative_get(&stored) == degrees);
            assert(13U == stored.side_turn && 3U == stored.side_turns);
        }

        for (int degrees = -S7_NEGATIVE_MAGNITUDE; degrees <= S7_MAX; ++degrees)
        {
            avoid_obstacle_aim_set(&stored, degrees);
            assert(avoid_obstacle_aim_get(&stored) == degrees);
            assert(3U == stored.blocked_reads);
        }
    }
    codecs();
    scans();
    rules_17nov();
    fsm(true, false, false);
    fsm(true, true, false);
    fsm(true, false, true);
    fsm(false, false, false);
    {
        /* 10 cm box at 196 mm: beam smear at 60/120 must not block either side.
         */
        unsigned ang[12] = {30, 60, 90, 120, 150, 75, 80, 85, 90, 95, 100, 105};
        unsigned echo_ranges_mm[12] = {4000, 215, 196, 215, 4000, 203,
                                       199,  197, 196, 197, 199,  203};
        profile_obstacle_sample_t scan_samples[12];
        s5_config_t               test_config = s5_config_defaults();
        profile_obstacle_t        input_profile;
        avoidance_planner_plan_t  planned_route;

        for (unsigned sample_index = 0; sample_index < SURVEY_SAMPLE_COUNT;
             sample_index++)
        {
            assert(profile_obstacle_sample_pack(
                ang[sample_index], echo_ranges_mm[sample_index], true,
                &scan_samples[sample_index]));
        }

        assert(profile_obstacle_build(scan_samples, 12, &test_config,
                                      &input_profile) &&
               (int)input_profile.width_mm <= 230 &&
               4000 == (int)input_profile.left_clear_mm &&
               4000 == (int)input_profile.right_clear_mm);
        assert(avoidance_planner_build(&input_profile, &test_config,
                                       &planned_route) &&
               (int)planned_route.offset_mm < 300);
    }
    {
        /* 45-degree out-leg sweeps forward: a wide obstacle close ahead blocks
         * the diagonal even with a clear sensor path (no hidden corner hit). */
        s5_config_t      test_config = s5_config_defaults();
        avoid_obstacle_t detour_state;
        avoid_obstacle_init(&detour_state, &test_config);
        avoid_obstacle_input_t step_input = {.b_is_healthy    = true,
                                             .b_is_stopped    = true,
                                             .b_is_line_ready = true};
        avoid_obstacle_begin(&detour_state, 0);
        step_input.now_us = 10000;
        (void)step_and_observe(&detour_state, &step_input);
        step_input.now_us                            = 20000;
        step_input.observation.b_is_scan_done        = true;
        step_input.observation.scan_us               = 20000;
        step_input.observation.profile               = profile();
        step_input.observation.profile.closest_mm    = 120;
        step_input.observation.profile.left_edge_mm  = 300;
        step_input.observation.profile.right_edge_mm = -300;
        step_input.observation.profile.width_mm      = 600;
        (void)step_and_observe(&detour_state, &step_input);
        assert(S5_MOVING == (int)detour_state.state);

        for (unsigned tick_index = 0;
             (tick_index < DETOUR_MOVEMENT_WAIT_TICKS) &&
             (S5_MOVING == (int)detour_state.state);
             tick_index++)
        {
            avoid_obstacle_action_t action;
            /* clang-format off */
            step_input.now_us                      += 10000U;
            step_input.observation.range_us         = step_input.now_us;
            step_input.observation.b_is_range_valid = true;
            step_input.observation.range_mm         = 2000;
            step_input.b_is_motion_done             =
                (0U != detour_state.b_has_issued);
            action                                  =
                step_and_observe(&detour_state, &step_input);
            /* clang-format on */
            assert(action.kind !=
                   (unsigned)
                       S5_ACT_DRIVE); /* Do not drive the blocked diagonal. */
        }
        /* Both sides would need more reversing than allowed: stop, no side
         * swap. */
        assert(S5_FAULT == (int)detour_state.state &&
               S5_NO_PATH == (int)detour_state.fault &&
               !detour_state.b_has_swapped);
    }
#if S5_ALLOW_OTHER_SIDE
    /* Preferred side blocked after the turn: 2 x 45 deg, take the other side.
     */
    gb_is_first_side_only = true;
    fsm(true, true, false);
    gb_is_first_side_only = false;
#endif
    {
        /* Aim angle: centre x over (closest + sensor-to-axle), + = left. */
        profile_obstacle_t measured_profile = profile();
        measured_profile.left_edge_mm       = 44;
        measured_profile.right_edge_mm      = -44;
        assert(avoid_obstacle_aim_deg(&measured_profile) == 0); /* centred */
        measured_profile.left_edge_mm  = 60;
        measured_profile.right_edge_mm = -30;
        assert(avoid_obstacle_aim_deg(&measured_profile) ==
               0); /* This 2.6 degree bearing is below 5 degrees. */
        measured_profile.left_edge_mm  = 150;
        measured_profile.right_edge_mm = -10;
        assert(avoid_obstacle_aim_deg(&measured_profile) ==
               12); /* atan(70/325) = 12.2 */
        measured_profile.left_edge_mm  = 10;
        measured_profile.right_edge_mm = -150;
        assert(avoid_obstacle_aim_deg(&measured_profile) ==
               -12); /* Mirror the bearing to the right. */
        measured_profile.left_edge_mm  = 700;
        measured_profile.right_edge_mm = 300;
        assert(avoid_obstacle_aim_deg(&measured_profile) ==
               S5_AIM_MAX_DEG); /* Cap the 57 degree turn. */
        measured_profile.left_edge_mm  = -100;
        measured_profile.right_edge_mm = -800;
        assert(avoid_obstacle_aim_deg(&measured_profile) == -S5_AIM_MAX_DEG);
        measured_profile.closest_mm    = 150;
        measured_profile.left_edge_mm  = 130;
        measured_profile.right_edge_mm = 10;
        assert(avoid_obstacle_aim_deg(&measured_profile) ==
               17); /* This closer face yields a 17.3 degree bearing. */
    }
    {
        /* Bend aim (17 Nov 2026: 1x, was 2x; the side check while passing now
         * corrects a crooked start). On a bend the cap is S5_BEND_AIM_MAX_DEG.
         */
        profile_obstacle_t measured_profile = profile();
        bool               b_is_on_bend;
        measured_profile.left_edge_mm = 150;
        measured_profile.right_edge_mm =
            -10; /* Use a bearing of 12 degrees left. */
        assert(S5_BEND_PCT == 20 && S5_BEND_AIM_PCT == 100);
        assert(avoid_obstacle_aim_deg_bend(&measured_profile, 5,
                                           &b_is_on_bend) == 12 &&
               !b_is_on_bend); /* straight */
        assert(avoid_obstacle_aim_deg_bend(&measured_profile, 15,
                                           &b_is_on_bend) == 12 &&
               !b_is_on_bend); /* Small steering still counts as a straight. */
        assert(avoid_obstacle_aim_deg_bend(&measured_profile, 20,
                                           &b_is_on_bend) == 12 &&
               b_is_on_bend); /* The bend uses the same aim. */
        assert(avoid_obstacle_aim_deg_bend(&measured_profile, -20,
                                           &b_is_on_bend) == 12 &&
               b_is_on_bend);
        measured_profile.left_edge_mm  = 10;
        measured_profile.right_edge_mm = -150;
        assert(avoid_obstacle_aim_deg_bend(&measured_profile, -20,
                                           &b_is_on_bend) == -12);
        measured_profile.left_edge_mm  = 60;
        measured_profile.right_edge_mm = -30;
        assert(
            avoid_obstacle_aim_deg_bend(&measured_profile, 30, &b_is_on_bend) ==
            0); /* The bearing is below 5 degrees. */
        measured_profile.left_edge_mm  = 360;
        measured_profile.right_edge_mm = 200;
        measured_profile.width_mm      = 160;
        assert(
            avoid_obstacle_aim_deg_bend(&measured_profile, 30, &b_is_on_bend) ==
            S5_BEND_AIM_MAX_DEG); /* Cap this 41 degree bearing. */
    }
    assert(fsm_aim_turn(150, -10, 250, false, 0.20) ==
           12); /* Aim, rescan and then plan on a bend. */
    assert(fsm_aim_turn(150, -10, 250, false, 0.05) ==
           12); /* Treat gentle steering as a straight. */
    {
        /* Return diagonal: straight keeps the full diagonal (tape crossing
         * ignored); bend stops on tape, then drives IR-to-axle; bend away gets
         * one extra drive. */
        s5_config_t              test_config = s5_config_defaults();
        avoidance_planner_plan_t planned_route;
        profile_obstacle_t       input_profile = profile();
        assert(avoidance_planner_build(&input_profile, &test_config,
                                       &planned_route));
        unsigned diag = ((planned_route.offset_mm * 1414U) + 999U) / 1000U;
        unsigned test_distances_mm[4];
        unsigned drive_count;
        bool     b_did_stop_early;
        /* Straight: tape stops the car too, then only IR-to-axle (never on past
         * the line). */
        fsm_cross(0.0, 60, test_distances_mm, &drive_count, &b_did_stop_early);
        assert(b_did_stop_early && 2U == drive_count &&
               test_distances_mm[0] == diag &&
               S5_IR_AXLE_MM == test_distances_mm[1]);
        fsm_cross(0.0, diag - S5_IR_AXLE_MM, test_distances_mm, &drive_count,
                  &b_did_stop_early);
        assert(b_did_stop_early && 2U == drive_count &&
               S5_IR_AXLE_MM == test_distances_mm[1]);
        fsm_cross(0.0, 0, test_distances_mm, &drive_count, &b_did_stop_early);
        assert(!b_did_stop_early && 1U == drive_count &&
               test_distances_mm[0] ==
                   diag); /* Skip the extra step on a straight. */
        fsm_cross(0.25, 60, test_distances_mm, &drive_count, &b_did_stop_early);
        assert(b_did_stop_early && 2U == drive_count &&
               test_distances_mm[0] == diag &&
               S5_IR_AXLE_MM == test_distances_mm[1]);
        assert(S5_RETURN_EXTRA_MM ==
               0U); /* Stop driving at the end of the diagonal. */
        fsm_cross(-0.25, diag + 50U, test_distances_mm, &drive_count,
                  &b_did_stop_early);
        assert(!b_did_stop_early && 1U == drive_count &&
               test_distances_mm[0] == diag);
        fsm_cross(0.25, 0, test_distances_mm, &drive_count, &b_did_stop_early);
        assert(!b_did_stop_early && 1U == drive_count);
    }
    {
        /* 20 cm worst case: the offset never plans for less than a 20 cm wide
         * obstacle about the scanned centre; wider readings still win. */
        s5_config_t              test_config = s5_config_defaults();
        avoidance_planner_plan_t planned_route;
        profile_obstacle_t       measured_profile = profile();
        const unsigned           extra_mm         = 5U;
        const unsigned           half_width_mm    = test_config.half_width_mm;
        const unsigned           margin_mm        = test_config.margin_mm;
        const unsigned           pad = (half_width_mm + margin_mm) + extra_mm;
        assert(S5_OBSTACLE_MAX_MM == 200);
        measured_profile.left_edge_mm  = 22;
        measured_profile.right_edge_mm = -22;
        assert(avoidance_planner_build(&measured_profile, &test_config,
                                       &planned_route) &&
               (100U + pad) == planned_route.offset_mm);
        measured_profile.left_edge_mm  = 150;
        measured_profile.right_edge_mm = -150;
        assert(avoidance_planner_build(&measured_profile, &test_config,
                                       &planned_route) &&
               (150U + pad) == planned_route.offset_mm);
        measured_profile.left_edge_mm  = 120;
        measured_profile.right_edge_mm = 40;
        assert(avoidance_planner_build(&measured_profile, &test_config,
                                       &planned_route) &&
               (180U + pad) == planned_route.offset_mm); /* centre 80 + 100 */
    }
    assert(fsm_aim(44, -44, 250, false) ==
           0); /* A straight uses one scan without an aim turn. */
    assert(fsm_aim(150, -10, 250, false) ==
           12); /* Correct the heading, then rescan and plan. */
    assert(fsm_aim(150, -10, 250, true) == 12); /* Allow only one aim turn. */
    /* Too close: 100 mm face, edge 75 (20 cm worst case: 100) -> corner needs
     * 90+100+30=220 vs 175: back 45. */
    assert(fsm_backup(100, 75) == -45);
    assert(fsm_backup(250, 44) ==
           0); /* Skip reverse when enough room is available. */
    assert(fsm_backup(60, 200) ==
           1); /* The 185 mm reverse exceeds the allowed backup. */
    /* Unknown depth: 4 peeks see the obstacle (4 x 50 mm steps), the 5th is
     * clear. */
    fsm_depth(true, false, false, 4);
    /* Deeper than max_depth_mm: stop with NO_PATH instead of driving on blind.
     */
    fsm_depth(true, false, false, 11);
    {
        /* Finding the line after the return diagonal (same length as the out
         * one). Not seen on the way (a line bending away): swerve to the line
         * side first, -90 from the passing heading, then +80, then back to it.
         */
        int              sweep[4] = {0};
        unsigned         back;
        s5_config_t      test_config = s5_config_defaults();
        avoid_obstacle_t second_detour_state;
        avoid_obstacle_init(&second_detour_state, &test_config);
        avoidance_planner_plan_t planned_route;
        profile_obstacle_t       input_profile = profile();
        assert(avoidance_planner_build(&input_profile, &test_config,
                                       &planned_route));
        fsm_line(-1, sweep, &back); /* This fixture must avoid a -45 degree turn
                                     * to -90 degrees, a +170 degree turn to +80
                                     * degrees, a -80 degree turn to zero, and
                                     * the NO_LINE outcome.
                                     */
        assert(back == (planned_route.offset_mm * 1414U + 999U) / 1000U);
        assert(S5_SWERVE_IN_DEG == 90 && S5_SWERVE_OUT_DEG == 80 &&
               sweep[0] == -45 && 170 == sweep[1] && sweep[2] == -80);
        sweep[0] = sweep[1] = sweep[2] = 0;
        fsm_line(6, sweep,
                 &back); /* Stop at tape on the first recovery sweep. */
        assert(sweep[0] == -45 && 0 == sweep[1]);
        sweep[0] = sweep[1] = sweep[2] = 0;
        fsm_line(7, sweep,
                 &back); /* Stop after finding tape on the second sweep. */
        assert(sweep[0] == -45 && 170 == sweep[1] && 0 == sweep[2]);
    }
    {
        /* Pass distance after "clear": peek geometry + sensor-to-rear + margin
         * (was 0.577 x whole offset + sensor-to-axle + half width + margin). */
        s5_config_t      test_config = s5_config_defaults();
        avoid_obstacle_t detour_state;
        avoid_obstacle_init(&detour_state, &test_config);
        const unsigned extra_mm      = 5U;
        const unsigned half_width_mm = test_config.half_width_mm;
        const unsigned margin_mm     = test_config.margin_mm;
        const unsigned pad           = (half_width_mm + margin_mm) + extra_mm;
        assert(63U == test_config.half_width_mm && S5_SONAR_AXLE_MM == 75U &&
               S5_PEEK_STEP_MM == 25U);
        detour_state.plan.offset_mm = 250U;
        /* The extra 5 mm of side gap adds 3 mm of post-peek travel:
         * ceil(0.577 x 98) + 75 + 55 + 30 = 217 mm. */
        assert(S5_PASS_EXTRA_V2 == 0 &&
               pass_extra_test(&detour_state) ==
                   (pad * 577U + 999U) / 1000U + S5_SONAR_AXLE_MM +
                       S5_REAR_AXLE_MM + test_config.margin_mm);
        assert(pass_extra_test(&detour_state) == 217U);
    }
    {
        /* Single-angle peek scan and merge. */
        scan_obstacle_t    scan;
        profile_obstacle_t mirrored_profile = {0};
        assert(!scan_obstacle_begin_at(&scan, 0, 90) &&
               !scan_obstacle_begin_at(&scan, 0, 65) &&
               scan_obstacle_begin_at(&scan, 0, 30));
        scan_obstacle_accept(&scan, 300000, true, 180);
        assert(scan.b_is_done && !scan.b_has_failed && 1 == (int)scan.count);
        assert(
            profile_obstacle_merge_peek(&mirrored_profile, &scan.sample[0]) &&
            mirrored_profile.b_is_valid &&
            180 == (int)mirrored_profile.right_clear_mm &&
            S5_RANGE_MAX_MM == mirrored_profile.left_clear_mm);
        profile_obstacle_t measured_profile = profile();
        assert(
            profile_obstacle_merge_peek(&measured_profile, &scan.sample[0]) &&
            180 == (int)measured_profile.right_clear_mm &&
            88 == (int)measured_profile.width_mm);
    }
    /* White alone must never confirm the original line, even in corridor. */
    line_recovery_t rec = {0};

    for (unsigned tick_index = 0; tick_index < FAR_LINE_CHECK_TICKS;
         tick_index++)
    {
        assert(!line_recovery_observe(&rec, tick_index * 10000U, true, false,
                                      false, true));
    }

    assert(!line_recovery_observe(&rec, 1000000, true, true, false, false));
    s5_config_t      cfg = s5_config_defaults();
    avoid_obstacle_t detour_state;
    avoid_obstacle_init(&detour_state, &cfg);
    avoid_obstacle_begin(&detour_state, 0);
    avoid_obstacle_input_t step_input = {.now_us       = 1000001,
                                         .b_is_healthy = true};
    assert(step_and_observe(&detour_state, &step_input).kind ==
               (unsigned)S5_ACT_STOP &&
           S5_DEADLINE == (int)detour_state.fault);
    puts("PASS: 45-degree bypass legs (equal out/back diagonals, left-only, "
         "reverse-if-too-close, realign+sweep line search); range/codec "
         "exhaustive boundaries, hostile packets, coarse/fine scans, "
         "invalid echo, route clearance, bounded FSM, faults and line "
         "evidence");
}

/** @brief Advance avoidance once and optionally record the returned action. */
static avoid_obstacle_action_t
step_and_observe (avoid_obstacle_t *             p_avoidance,
                  const avoid_obstacle_input_t * p_input)
{
    const avoid_obstacle_action_t action =
        avoid_obstacle_step(p_avoidance, p_input);
#ifdef TEST_SEMANTIC_TRACE
    obstacle_trace_action(p_avoidance, p_input, &action);
#endif
    return action;
}

/** @brief Construct a five-angle range scan and verify the derived obstacle
 * width, forward range and side clearances in millimetres.
 */
static profile_obstacle_t profile (void)
{
    profile_obstacle_sample_t samples[5];
    s5_config_t               cfg = s5_config_defaults();
    profile_obstacle_t        scan_profile;
    unsigned                  range[5] = {1800, 1500, 250, 1600, 1900};

    for (unsigned angle_index = 0; angle_index < PROFILE_SAMPLE_COUNT;
         angle_index++)
    {
        assert(profile_obstacle_sample_pack(30U + 30U * angle_index,
                                            range[angle_index], true,
                                            &samples[angle_index]));
    }

    assert(profile_obstacle_build(samples, 5, &cfg, &scan_profile));
    /* Edge padding scales with the farthest obstacle return: 250 mm * .173 ->
     * 44 each side. */
    assert(S5_EDGE_INFLATE_PERMIL == 173U && 250U == cfg.detect_mm &&
           S5_LEG_CLEAR_MM == 200U);
    assert(scan_profile.b_is_valid && 250 == (int)scan_profile.closest_mm &&
           88 == (int)scan_profile.width_mm &&
           1900 == (int)scan_profile.left_clear_mm &&
           1800 == (int)scan_profile.right_clear_mm);
    return scan_profile;
}

/** @brief Exercise obstacle sample and observation encodings over the
 * accepted angle and distance ranges, including timestamp wrap.
 */
static void codecs (void)
{
    for (unsigned angle_deg                          = MIN_SCAN_ANGLE_DEG;
         angle_deg <= MAX_SCAN_ANGLE_DEG; angle_deg += 5U)
    {
        for (unsigned distance_mm = MIN_ECHO_MM; distance_mm <= MAX_ECHO_MM;
             distance_mm++)
        {
            profile_obstacle_sample_t sample;
            unsigned                  angle;
            unsigned                  sample_range_mm;
            bool                      b_is_valid;
            assert(profile_obstacle_sample_pack(angle_deg, distance_mm, true,
                                                &sample));
            assert(profile_obstacle_sample_unpack(
                &sample, &angle, &sample_range_mm, &b_is_valid));
            assert(angle == angle_deg && sample_range_mm == distance_mm &&
                   b_is_valid);
        }
    }

    profile_obstacle_sample_t bad = {{0, 0, 0x80}};
    unsigned                  decoded_angle_deg;
    unsigned                  decoded_range_mm;
    bool                      b_has_decoded_echo;
    assert(!profile_obstacle_sample_unpack(
        &bad, &decoded_angle_deg, &decoded_range_mm, &b_has_decoded_echo));
    s5_codec_observation_t observation = {.range_us         = 0xfffffff0U,
                                          .scan_us          = 123,
                                          .scan_id          = 65535,
                                          .range_mm         = 4000,
                                          .b_is_range_valid = true,
                                          .b_is_scan_done   = true,
                                          .profile          = profile()};
    s5_codec_observation_t out         = {0};
    uint8_t                bytes[S5_PACKET_BYTES];
    uint8_t                again[S5_PACKET_BYTES];
    observation.profile.face_width_min_mm = 118U;
    observation.profile.face_width_max_mm = 180U;
    assert(s5_codec_observation_pack(&observation, bytes));
    assert(s5_codec_observation_unpack(bytes, &out));
    assert(
        (((out.range_us == observation.range_us) && (4000 == out.range_mm)) &&
         ((65535 == out.scan_id) && (1900 == out.profile.left_clear_mm))) &&
        ((118 == out.profile.face_width_min_mm) &&
         (180 == out.profile.face_width_max_mm)));
    assert(s5_codec_observation_pack(&out, again) &&
           !memcmp(bytes, again, sizeof(bytes)));
    /* Widen the byte for the reserved-bit mask. The result stays in 0..255. */
    const unsigned last_byte = S5_PACKET_BYTES - 1U;
    bytes[last_byte]         = (uint8_t)((unsigned)bytes[last_byte] | 0x80U);
    assert(!s5_codec_observation_unpack(bytes, &out));
    observation.b_has_no_echo = true;
    assert(!s5_codec_observation_pack(&observation, bytes));
    observation.b_has_no_echo = false;
    observation.range_mm      = 4096;
    assert(!s5_codec_observation_pack(&observation, bytes));
    unsigned seed = 42;

    for (unsigned packet_index = 0;
         packet_index < (unsigned)OBSERVATION_PACKET_CASES; packet_index++)
    {
        for (unsigned byte_index = 0; byte_index < sizeof(bytes); byte_index++)
        {
            seed              = (seed * 1664525U) + 1013904223U;
            bytes[byte_index] = (uint8_t)(seed >> 24);
        }

        if (s5_codec_observation_unpack(bytes, &out))
        {
            assert(s5_codec_observation_pack(&out, again) &&
                   !memcmp(bytes, again, sizeof(bytes)));
        }
    }

    uint16_t sample_range_mm;
    assert(ultrasonic_sensor_pulse_mm(2000, &sample_range_mm) &&
           343 == (int)sample_range_mm);
    assert(!ultrasonic_sensor_pulse_mm(0, &sample_range_mm));
    assert(!ultrasonic_sensor_pulse_mm(25000, &sample_range_mm));
    /* Open space is clear, not a fault; too-short pulses stay invalid. */
    assert(ultrasonic_sensor_echo_mm(2000, &sample_range_mm) &&
           343 == (int)sample_range_mm);
    assert(ultrasonic_sensor_echo_mm(25000, &sample_range_mm) &&
           S5_RANGE_MAX_MM == sample_range_mm);
    assert(ultrasonic_sensor_echo_mm(23900, &sample_range_mm) &&
           S5_RANGE_MAX_MM == sample_range_mm);
    assert(!ultrasonic_sensor_echo_mm(0, &sample_range_mm) &&
           0 == (int)sample_range_mm);
    assert(!ultrasonic_sensor_echo_mm(100, &sample_range_mm));
    observation = (s5_codec_observation_t){
        .b_is_range_valid = true, .range_mm = 1000, .range_us = 0xfffffff0U};
    assert(avoid_obstacle_range_ready(&observation, 100));
    assert(!avoid_obstacle_range_ready(&observation, 300000));
}

/** @brief Drive the scan sequencer from a wrapping timestamp and verify
 * handling of accepted readings and invalid profiles.
 */
static void scans (void)
{
    scan_obstacle_t scan;
    uint32_t        now = 0xfffffff0U;
    scan_obstacle_begin(&scan, now);

    while (!scan.b_is_done)
    {
        now += 300000U;
        scan_obstacle_accept(&scan, now, true,
                             90 == (int)scan.angle ? 250 : 1800);
    }

    assert((((!scan.b_has_failed) &&
             (12U == scan_obstacle_profile_count(&scan))) &&
            (16 == scan.count)) &&
           scan.b_is_fine);
    s5_config_t        cfg = s5_config_defaults();
    profile_obstacle_t profile;
    assert(profile_obstacle_build(
        scan.sample, scan_obstacle_profile_count(&scan), &cfg, &profile));
    routing_count_tests();
    face_width_tests(&cfg, &profile);
    avoidance_planner_plan_t plan;
    assert(avoidance_planner_build(&profile, &cfg, &plan) &&
           S5_ROUTE_LEFT == (int)plan.route);
    /* Diagonal side readings only pick the side; too wide an offset still
     * fails. */
    profile.left_clear_mm  = 10;
    profile.right_clear_mm = 20;
    assert(avoidance_planner_build(&profile, &cfg, &plan) &&
           (int)plan.route == (S5_PREFER_ROUTE == 2U ? S5_ROUTE_RIGHT
                               : S5_PREFER_ROUTE == 1U ? S5_ROUTE_LEFT
                                                       : S5_ROUTE_RIGHT) &&
           (int)plan.b_has_alt_route == (S5_ALLOW_OTHER_SIDE != 0));
    /* Left too wide but right fine: left-only never plans the right side. */
    {
        profile_obstacle_t measured_profile = profile;
        measured_profile.left_edge_mm       = 900;
        measured_profile.right_edge_mm      = -20;
        assert(avoidance_planner_build(&measured_profile, &cfg, &plan) ==
               (S5_ALLOW_OTHER_SIDE != 0 || S5_PREFER_ROUTE != 1U));
    }
    profile.left_edge_mm  = 900;
    profile.right_edge_mm = -900;
    assert(!avoidance_planner_build(&profile, &cfg, &plan));
    scan_obstacle_begin(&scan, 0);

    for (unsigned retry_index = 1; retry_index <= BLOCKED_RETRY_LIMIT;
         retry_index++)
    {
        scan_obstacle_accept(&scan, retry_index * 70000U, false, 0);
    }

    assert(scan.b_has_failed && scan.b_is_done && 0 == (int)scan.count);
    scan_obstacle_begin(&scan, 0);
    scan_obstacle_accept(&scan, S5_SCAN_TIMEOUT_US, true, 200);
    assert(scan.b_has_failed && (0U == scan_obstacle_profile_count(&scan)));
    profile_obstacle_sample_t invalid[5];

    for (unsigned angle_index = 0;
         angle_index < (sizeof invalid / sizeof invalid[0]); angle_index++)
    {
        assert(profile_obstacle_sample_pack(30U + 30U * angle_index, 0, false,
                                            &invalid[angle_index]));
    }

    assert(!profile_obstacle_build(invalid, 5, &cfg, &profile));
}

/** @brief Check the base count at every possible coarse minimum, both
 * before and after the optional width measurements.
 */
static void routing_count_tests (void)
{
    const unsigned  coarse_angles[] = {30U, 60U, 90U, 120U, 150U};
    const unsigned  counts[]        = {9U, 12U, 12U, 12U, 9U};
    const uint32_t  interval_us     = 200000U;
    scan_obstacle_t scan;

    for (unsigned angle_index = 0U;
         angle_index < (sizeof coarse_angles / sizeof coarse_angles[0]);
         ++angle_index)
    {
        const unsigned nearest  = coarse_angles[angle_index];
        const unsigned expected = counts[angle_index];
        uint32_t       now      = 0U;
        scan_obstacle_begin(&scan, now);

        for (unsigned sample_index = 0U; sample_index < expected;
             ++sample_index)
        {
            assert(0U == scan_obstacle_profile_count(&scan));
            assert(!scan.b_is_done);
            const unsigned angle    = scan.angle;
            const unsigned range_mm = (angle == nearest) ? 200U : 240U;
            now                    += interval_us;
            scan_obstacle_accept(&scan, now, true, range_mm);
        }

        const unsigned count = scan.count;
        assert(count == expected);
        assert(expected == scan_obstacle_profile_count(&scan));
        assert(!scan.b_is_done);

        while (!scan.b_is_done)
        {
            now += interval_us;
            scan_obstacle_accept(&scan, now, true, 4000U);
            assert(expected == scan_obstacle_profile_count(&scan));
        }

        const unsigned final_count = scan.count;
        assert(final_count > expected);
        assert(!scan.b_has_failed);
    }

    assert(scan_obstacle_begin_at(&scan, 0U, 30U));
    scan_obstacle_accept(&scan, interval_us, true, 300U);
    assert(scan.b_is_done && (0U == scan_obstacle_profile_count(&scan)));
}

/** @brief Check width bounds, open edges and optional-sweep failure handling.
 * @param[in] p_cfg Stable scan configuration.
 * @param[in,out] p_profile Valid routing profile, with bounds restored to
 * unavailable by the end of the test.
 */
static void face_width_tests (const s5_config_t *  p_cfg,
                              profile_obstacle_t * p_profile)
{
    scan_obstacle_t scan;
    const uint32_t  interval_us = 300000U;

    /* Centred flat 150 mm face at 233 mm. The 30-degree bearings hit
     * it and the 35-degree bearings just clear it through a 15-degree cone. */
    scan_obstacle_begin(&scan, 0U);
    while (!scan.b_is_done)
    {
        const unsigned angle   = scan.angle;
        const unsigned bearing = (angle > 90U) ? (angle - 90U) : (90U - angle);
        const unsigned distance =
            (bearing <= 15U) ? 233U : ((bearing == 30U) ? 241U : 4000U);
        /* The nonnegative sample count widens exactly to uint32_t. */
        scan_obstacle_accept(&scan, interval_us * (uint32_t)scan.count, true,
                             distance);
    }

    profile_obstacle_face_width(scan.sample, scan.count, p_cfg, p_profile);
    assert(
        ((scan_obstacle_profile_count(&scan) == 12U) && (scan.count == 16)) &&
        ((p_profile->face_width_min_mm == 118) &&
         (p_profile->face_width_max_mm == 180)));

    /* No clear ray on the left: a nearer return cannot bracket the edge. */
    const unsigned count = scan.count;
    for (unsigned sample_index = 0U; sample_index < count; ++sample_index)
    {
        unsigned angle;
        unsigned distance;
        bool     b_has_echo;
        assert(profile_obstacle_sample_unpack(&scan.sample[sample_index],
                                              &angle, &distance, &b_has_echo));
        if ((angle > 120U) && (distance == 4000U))
        {
            assert(profile_obstacle_sample_pack(angle, 90U, true,
                                                &scan.sample[sample_index]));
        }
    }

    profile_obstacle_face_width(scan.sample, scan.count, p_cfg, p_profile);
    assert((p_profile->face_width_min_mm == 0) &&
           (p_profile->face_width_max_mm == 0));

    scan_obstacle_begin(&scan, 0U);
    while (!scan.b_has_profile)
    {
        /* The nonnegative sample count widens exactly to uint32_t. */
        scan_obstacle_accept(&scan, interval_us * (uint32_t)scan.count, true,
                             240U);
    }

    scan_obstacle_t late = scan;
    scan_obstacle_accept(&late, S5_SCAN_TIMEOUT_US, true, 240U);
    const unsigned late_count = late.count;
    assert((late.b_is_done && (!late.b_has_failed)) &&
           (late_count == scan_obstacle_profile_count(&late)));

    /* The nonnegative sample count widens exactly to uint32_t. */
    scan_obstacle_accept(&scan, interval_us * (uint32_t)scan.count, false, 0U);
    const unsigned fallback_count = scan.count;
    assert((scan.b_is_done && (!scan.b_has_failed)) &&
           (scan_obstacle_profile_count(&scan) == fallback_count));
}

/** @brief Run the bypass scenario with the requested line, blockage and
 * range conditions using the default depth argument.
 */
static void fsm (bool b_can_see_line, bool b_is_blocked,
                 bool b_has_invalid_range)
{
    fsm_depth(b_can_see_line, b_is_blocked, b_has_invalid_range, 0);
}

/** @brief Exercise the avoidance state machine with injected line, obstacle
 * and range outcomes, then verify its abort transition to idle.
 */
static void fsm_depth (bool b_can_see_line, bool b_is_blocked,
                       bool b_has_invalid_range, unsigned deep)
{
    s5_config_t      cfg = s5_config_defaults();
    avoid_obstacle_t avoidance;
    avoid_obstacle_init(&avoidance, &cfg);
    avoid_obstacle_input_t input = {
        .b_is_healthy = true, .b_is_stopped = false, .b_is_line_ready = true};
    avoid_obstacle_begin(&avoidance, 0);
    assert(step_and_observe(&avoidance, &input).kind == (unsigned)S5_ACT_NONE &&
           S5_WAIT_STOP == (int)avoidance.state);
    input.now_us       = 10000;
    input.b_is_stopped = true;
    assert(step_and_observe(&avoidance, &input).kind == (unsigned)S5_ACT_SCAN);
    input.now_us                     = 20000;
    input.observation.b_is_scan_done = true;
    input.observation.scan_us        = 20000;
    input.observation.profile        = profile();

    if (b_is_blocked)
    {
        input.observation.profile.left_clear_mm =
            input.observation.profile.right_clear_mm = 10;
    }

    avoid_obstacle_action_t action = step_and_observe(&avoidance, &input);
    g_peeks                        = 0;
    g_turn_count                   = 0;
    g_outbound_drive_mm            = 0;
    g_return_drive_mm              = 0;

    for (unsigned tick = 0; ((tick < (unsigned)DEPTH_RUN_TICK_LIMIT) &&
                             ((int)avoidance.state != S5_COMPLETE)) &&
                            ((int)avoidance.state != S5_FAULT);
         tick++)
    {
        input.now_us                      += 10000U;
        input.observation.range_us         = input.now_us;
        input.observation.b_is_range_valid = true;
        /* block: after turning, the sideways path reads 150 mm (< offset +
         * margin). */
        /* clang-format off */
        input.observation.range_mm =
            ((b_is_blocked && (1 == (int)avoidance.leg)) &&
             ((!gb_is_first_side_only) || (!avoidance.b_has_swapped)))
                ? 150
                : 2000;
        input.b_has_tape           = false;
        input.b_is_motion_done     = false;
        /* clang-format on */

        if (((unsigned)S5_ACT_TURN == action.kind) && (g_turn_count < 16U))
        {
            g_turns[g_turn_count++] = action.value;
        }

        if (((unsigned)S5_ACT_DRIVE == action.kind) &&
            (1 == (int)avoidance.leg))
        {
            g_outbound_drive_mm = (unsigned)action.value;
        }

        if (((unsigned)S5_ACT_DRIVE == action.kind) &&
            (5 == (int)avoidance.leg))
        {
            g_return_drive_mm = (unsigned)action.value;
        }

        if ((unsigned)S5_ACT_PEEK == action.kind)
        {
            /* LEFT bypass looks right (30 deg); report that side's reading. */
            assert(3 == (int)avoidance.leg &&
                   S5_PASS_PEEKING == (int)avoidance.pass &&
                   action.value ==
                       (S5_ROUTE_LEFT == (int)avoidance.plan.route ? 30 : 150));
            input.observation.b_is_scan_done = true;
            input.observation.scan_us        = input.now_us;

            unsigned side =
                g_peeks++ < deep
                    ? 97U
                    : 2000U; /* A 97 mm echo at the planned angle puts it
                              * beside the car at the intended gap. The
                              * projected separation is 0.966 times 97 mm,
                              * or about 93 mm.
                              */
            if (S5_ROUTE_LEFT == (int)avoidance.plan.route)
            {
                input.observation.profile.right_clear_mm = (uint16_t)side;
            }
            else
            {
                input.observation.profile.left_clear_mm = (uint16_t)side;
            }
        }

        if (S5_FINISH == (int)avoidance.state)
        {
            /* The IR sensor stopped over the tape. */
            input.b_has_tape = b_can_see_line;
        }

        if (avoidance.b_has_issued)
        {
            if (b_has_invalid_range && (1 == (int)avoidance.leg))
            {
                input.observation.b_is_range_valid = false;
            }

            const bool b_is_final_recovery = (6 == (int)avoidance.leg);

            if (b_is_final_recovery)
            {
                /* Back on the original line: tape under the IR while turning
                 * back to the heading. Hold it 40 ms (needs 30 ms). */
                input.b_has_tape = b_can_see_line;
            }

            if ((b_is_final_recovery && b_can_see_line) &&
                ((!avoidance.recovery.b_is_touching) ||
                 ((uint32_t)(input.now_us - avoidance.recovery.since_us) <
                  40000U)))
            {
                action = step_and_observe(&avoidance, &input);
                continue;
            }

            input.b_is_motion_done = true;
        }

        action = step_and_observe(&avoidance, &input);
    }

    if (b_is_blocked && gb_is_first_side_only)
    {
        /* Out 45, blocked, 2 x 45 to the other diagonal, then that side. */
        int scenario_index =
            S5_ROUTE_LEFT == (int)avoidance.plan.route ? 1 : -1;
        assert(avoidance.b_has_swapped && S5_COMPLETE == (int)avoidance.state &&
               g_turn_count >= 5U);
        assert(
            g_turns[0] == -scenario_index * 45 &&
            g_turns[1] == scenario_index * 90 &&
            g_turns[2] == -scenario_index * 45 &&
            g_turns[3] == -scenario_index * 45 &&
            g_turns[4] ==
                -scenario_index *
                    (S5_SWERVE_IN_DEG -
                     45)); /* Sweep toward the tape when it was not crossed. */
        return;
    }

    if ((((!b_is_blocked) && (!b_has_invalid_range)) && b_can_see_line) &&
        ((deep * S5_PEEK_STEP_MM) <= cfg.max_depth_mm))
    {
        /* 45-degree legs: out, parallel, back, then the swerve (this script
         * never shows tape on the return diagonal, so: line side first). */
        int scenario_index =
            S5_ROUTE_LEFT == (int)avoidance.plan.route ? 1 : -1;
        assert(g_turn_count >= 4U && g_turns[0] == scenario_index * 45 &&
               g_turns[1] == -scenario_index * 45 &&
               g_turns[2] == -scenario_index * 45 &&
               g_turns[3] == -scenario_index * (S5_SWERVE_IN_DEG - 45));
        assert(g_outbound_drive_mm ==
                   (avoidance.plan.offset_mm * 1414U + 999U) / 1000U &&
               g_return_drive_mm == g_outbound_drive_mm);
    }

    if (b_is_blocked)
    {
        assert(S5_FAULT == (int)avoidance.state &&
               S5_NO_PATH == (int)avoidance.fault && 1 == (int)avoidance.leg &&
               !avoidance.b_has_issued &&
               (int)avoidance.b_has_swapped == (S5_ALLOW_OTHER_SIDE != 0) &&
               (S5_ROUTE_LEFT + (S5_ALLOW_OTHER_SIDE != 0)) ==
                   (int)avoidance.plan.route);
        return;
    }

    if ((deep * S5_PEEK_STEP_MM) > cfg.max_depth_mm)
    {
        assert(S5_FAULT == (int)avoidance.state &&
               S5_NO_PATH == (int)avoidance.fault);
        return;
    }

    if (((!b_has_invalid_range) && (!b_is_blocked)) && (g_peeks != (deep + 1U)))
    {
        fprintf(
            stderr,
            "deep=%u peeks=%u searched=%u state=%u fault=%u leg=%u pass=%u\n",
            deep, g_peeks, avoidance.searched_mm, avoidance.state,
            avoidance.fault, avoidance.leg, avoidance.pass);
    }

    if (b_has_invalid_range)
    {
        assert(S5_FAULT == (int)avoidance.state &&
               S5_BAD_RANGE == (int)avoidance.fault);
    }
    else if (b_can_see_line)
    {
        assert(S5_COMPLETE == (int)avoidance.state &&
               (unsigned)S5_ACT_RESUME == action.kind);
    }
    else
    {
        assert(S5_FAULT == (int)avoidance.state &&
               S5_NO_LINE == (int)avoidance.fault);
    }

    avoid_obstacle_abort(&avoidance);
    assert(S5_IDLE == (int)avoidance.state);
}

/** @brief Present tape only on the selected bypass leg. Tape on return leg
 * five must be ignored. Values six through nine exercise the
 * realignment and sweeps. A value of minus one keeps the tape
 * absent.
 */
static void fsm_line (int found_leg, int * p_sweep, unsigned * p_back_drive)
{
    s5_config_t      cfg = s5_config_defaults();
    avoid_obstacle_t avoidance;
    avoid_obstacle_init(&avoidance, &cfg);
    avoid_obstacle_input_t input = {
        .b_is_healthy = true, .b_is_stopped = true, .b_is_line_ready = true};
    avoid_obstacle_action_t action;
    unsigned                action_index = 0;
    avoid_obstacle_begin(&avoidance, 0);
    input.now_us = 10000;
    (void)step_and_observe(&avoidance, &input);
    input.now_us                     = 20000;
    input.observation.b_is_scan_done = true;
    input.observation.scan_us        = 20000;
    input.observation.profile        = profile();
    action                           = step_and_observe(&avoidance, &input);
    *p_back_drive                    = 0;

    for (unsigned tick = 0; ((tick < (unsigned)LINE_RETURN_TICK_LIMIT) &&
                             ((int)avoidance.state != S5_COMPLETE)) &&
                            ((int)avoidance.state != S5_FAULT);
         tick++)
    {
        input.now_us                      += 10000U;
        input.observation.range_us         = input.now_us;
        input.observation.b_is_range_valid = true;
        input.observation.range_mm         = 2000;

        if ((unsigned)S5_ACT_PEEK == action.kind)
        {
            /* clang-format off */
            input.observation.b_is_scan_done         = true;
            input.observation.scan_us                = input.now_us;
            input.observation.profile.right_clear_mm =
                input.observation.profile.left_clear_mm = 2000;
            /* clang-format on */
        }

        if (((unsigned)S5_ACT_DRIVE == action.kind) &&
            (5 == (int)avoidance.leg))
        {
            *p_back_drive = (unsigned)action.value;
        }

        if (((((unsigned)S5_ACT_TURN == action.kind) &&
              ((int)avoidance.leg >= 6)) &&
             (S5_MOVING == (int)avoidance.state)) &&
            (action_index < 4U))
        {
            p_sweep[action_index++] = action.value;
        }

        assert(!((unsigned)S5_ACT_DRIVE == action.kind &&
                 (int)avoidance.leg >
                     5)); /* Do not creep straight off the path. */
        /* clang-format off */
        input.b_has_tape       =
            (((found_leg >= 5) && ((int)avoidance.leg == found_leg)) &&
             (avoidance.b_has_issued)) ||
            ((S5_FINISH == (int)avoidance.state) && (found_leg >= 6));
        input.b_is_motion_done = false;
        /* clang-format on */

        if (avoidance.b_has_issued)
        {
            /* Hold the motion open for 50 ms while tape is visible (the
             * crossing on leg 5 is brief: it just passes under the IR). */
            if ((((input.b_has_tape) && ((int)avoidance.leg >= 6)) &&
                 (S5_MOVING == (int)avoidance.state)) &&
                ((!avoidance.recovery.b_is_touching) ||
                 ((uint32_t)(input.now_us - avoidance.recovery.since_us) <
                  50000U)))
            {
                action = step_and_observe(&avoidance, &input);
                continue;
            }

            input.b_is_motion_done = true;
        }

        action = step_and_observe(&avoidance, &input);
    }

    if (found_leg < 6)
    {
        assert(S5_FAULT == (int)avoidance.state &&
               S5_NO_LINE == (int)avoidance.fault);
    }
    else
    {
        assert(S5_COMPLETE == (int)avoidance.state &&
               (unsigned)S5_ACT_RESUME == action.kind);
    }
}

/** @brief Exercise an obstacle closer than the 45-degree departure allows.
 * Return the commanded backup distance and check that only one
 * reverse is requested.
 */
static int fsm_backup (unsigned closest, int left_edge)
{
    s5_config_t      config = s5_config_defaults();
    avoid_obstacle_t avoidance;
    avoid_obstacle_init(&avoidance, &config);
    avoid_obstacle_input_t input = {
        .b_is_healthy = true, .b_is_stopped = true, .b_is_line_ready = true};
    avoid_obstacle_action_t action;
    int                     back = 0;
    unsigned                turn = 0;
    avoid_obstacle_begin(&avoidance, 0);
    input.now_us = 10000;
    (void)step_and_observe(&avoidance, &input);
    /* clang-format off */
    input.now_us                            = 20000;
    input.observation.b_is_scan_done        = true;
    input.observation.scan_us               = 20000;
    input.observation.profile               = profile();
    input.observation.profile.closest_mm    = (uint16_t)closest;
    input.observation.profile.left_edge_mm  = (int16_t)left_edge;
    input.observation.profile.right_edge_mm =
        (int16_t)-left_edge; /* Skip aiming when already centred. */
    /* clang-format on */
    action = step_and_observe(&avoidance, &input);
    assert(S5_MOVING == (int)avoidance.state);

    for (unsigned tick_index = 0; ((tick_index < BACKUP_TURN_WAIT_TICKS) &&
                                   (S5_MOVING == (int)avoidance.state)) &&
                                  (!turn);
         tick_index++)
    {
        /* clang-format off */
        input.now_us                      += 10000U;
        input.observation.range_us         = input.now_us;
        input.observation.b_is_range_valid = true;
        input.observation.range_mm         =
            avoidance.b_is_backing
                ? 60
                : 2000; /* Allow the close face during reverse. */
        /* clang-format on */
        input.b_is_motion_done = (0U != avoidance.b_has_issued);
        action                 = step_and_observe(&avoidance, &input);

        if ((unsigned)S5_ACT_DRIVE == action.kind)
        {
            assert(0 == (int)avoidance.leg && avoidance.b_is_backing && !back);
            back = action.value;
        }

        if ((unsigned)S5_ACT_TURN == action.kind)
        {
            assert(0 == (int)avoidance.leg && S5_BYPASS_DEG == action.value);
            turn = 1;
        }
    }

    if (S5_FAULT == (int)avoidance.state)
    {
        assert(S5_NO_PATH == (int)avoidance.fault && !back);
        return 1;
    }

    assert(turn && avoidance.b_has_backed && !avoidance.b_is_backing);
    return back;
}

/** @brief Stop the simulated car at an angle to the obstacle. Return its
 * corrective aim turn, or zero when none is requested, and check the
 * rescan before planning.
 */
static int fsm_aim_turn (int left_edge, int right_edge, unsigned closest,
                         bool b_is_second_off, float64_t turn)
{
    s5_config_t      config = s5_config_defaults();
    avoid_obstacle_t avoidance;
    avoid_obstacle_init(&avoidance, &config);
    const int8_t turn_pct =
        (int8_t)(turn >= 0.0 ? (turn * 100.0) + 0.5 : (turn * 100.0) - 0.5);
    avoid_obstacle_input_t  input = {.b_is_healthy    = true,
                                     .b_is_stopped    = true,
                                     .b_is_line_ready = true,
                                     .turn_pct        = turn_pct};
    avoid_obstacle_action_t action;
    int                     turn_deg = 0;
    unsigned                scans    = 1;
    avoid_obstacle_begin(&avoidance, 0);
    input.now_us = 10000;
    assert(step_and_observe(&avoidance, &input).kind == (unsigned)S5_ACT_SCAN);
    /* clang-format off */
    input.now_us                            = 20000;
    input.observation.b_is_scan_done        = true;
    input.observation.scan_us               = 20000;
    input.observation.profile               = profile();
    input.observation.profile.closest_mm    = (uint16_t)closest;
    input.observation.profile.left_edge_mm  = (int16_t)left_edge;
    input.observation.profile.right_edge_mm = (int16_t)right_edge;
    action                                  =
        step_and_observe(&avoidance, &input);
    /* clang-format on */

    for (unsigned tick_index = 0; (tick_index < SCAN_AIM_WAIT_TICKS) &&
                                  (S5_SCANNING == (int)avoidance.state);
         tick_index++)
    {
        if ((unsigned)S5_ACT_TURN == action.kind)
        {
            assert(!turn_deg && avoidance.b_is_aiming &&
                   S5_SCANNING == (int)avoidance.state);
            turn_deg = action.value;
        }

        if ((unsigned)S5_ACT_SCAN == action.kind)
        {
            /* Rescan after the aim: centred now, unless the test keeps it off.
             */
            scans++;
            input.observation.scan_us = input.now_us + 5000U;

            if (!b_is_second_off)
            {
                input.observation.profile = profile();
            }
        }

        input.now_us          += 10000U;
        input.b_is_motion_done = (0U != avoidance.b_has_issued);
        input.b_is_stopped     = !avoidance.b_has_issued;
        action                 = step_and_observe(&avoidance, &input);
    }

    assert(S5_MOVING == (int)avoidance.state && !avoidance.b_has_issued &&
           !avoidance.b_is_aiming && avoidance.b_has_aimed);
    assert(scans == (turn_deg ? 2U : 1U) &&
           avoid_obstacle_aim_get(&avoidance) == turn_deg &&
           (int)avoidance.b_is_on_bend == (turn * 100.0 >= S5_BEND_PCT - 0.5 ||
                                           turn * 100.0 <= -S5_BEND_PCT + 0.5));
    return turn_deg;
}

/** @brief Return the simulated aiming result for the supplied obstacle edges
 * and nearest range with the initial turn offset set to zero.
 */
static int fsm_aim (int left_edge_mm, int right_edge_mm, unsigned clearance_mm,
                    bool b_is_off)
{
    return fsm_aim_turn(left_edge_mm, right_edge_mm, clearance_mm, b_is_off,
                        0.0);
}

/** @brief Report tape after tape_at_mm millimetres of travel on the return
 * diagonal. Zero keeps it absent on that leg. Record diagonal drive
 * requests and early stopping, then show tape during the turn back.
 */
static void fsm_cross (float64_t turn, unsigned tape_at_mm, unsigned * p_drives,
                       unsigned * p_navigation, bool * pb_did_stop_early)
{
    s5_config_t      config = s5_config_defaults();
    avoid_obstacle_t avoidance;
    avoid_obstacle_init(&avoidance, &config);
    const int8_t turn_pct =
        (int8_t)(turn >= 0.0 ? (turn * 100.0) + 0.5 : (turn * 100.0) - 0.5);
    avoid_obstacle_input_t  input = {.b_is_healthy    = true,
                                     .b_is_stopped    = true,
                                     .b_is_line_ready = true,
                                     .turn_pct        = turn_pct};
    avoid_obstacle_action_t action;
    float64_t               pos  = 0.0;
    unsigned                want = 0;
    avoid_obstacle_begin(&avoidance, 0);
    input.now_us = 10000;
    (void)step_and_observe(&avoidance, &input);
    input.now_us                     = 20000;
    input.observation.b_is_scan_done = true;
    input.observation.scan_us        = 20000;
    input.observation.profile        = profile();
    action                           = step_and_observe(&avoidance, &input);
    *p_navigation                    = 0;
    *pb_did_stop_early               = false;

    for (unsigned tick = 0; ((tick < (unsigned)RECOVERY_RUN_TICK_LIMIT) &&
                             ((int)avoidance.state != S5_COMPLETE)) &&
                            ((int)avoidance.state != S5_FAULT);
         tick++)
    {
        input.now_us                      += 10000U;
        input.observation.range_us         = input.now_us;
        input.observation.b_is_range_valid = true;
        input.observation.range_mm         = 2000;

        if ((unsigned)S5_ACT_PEEK == action.kind)
        {
            /* clang-format off */
            input.observation.b_is_scan_done         = true;
            input.observation.scan_us                = input.now_us;
            input.observation.profile.right_clear_mm =
                input.observation.profile.left_clear_mm = 2000;
            /* clang-format on */
        }

        if (((unsigned)S5_ACT_DRIVE == action.kind) &&
            (5 == (int)avoidance.leg))
        {
            assert(*p_navigation < 4U);
            p_drives[(*p_navigation)++] = (unsigned)action.value;
            want                        = (unsigned)action.value;
            pos                         = 0.0;
        }

        if (((unsigned)S5_ACT_STOP == action.kind) && (5 == (int)avoidance.leg))
        {
            *pb_did_stop_early = true;
        }

        input.b_is_motion_done = false;
        input.b_has_tape       = false;

        if ((avoidance.b_has_issued) && (5 == (int)avoidance.leg))
        {
            unsigned before = 0;

            for (unsigned leg_index = 0; (leg_index + 1U) < *p_navigation;
                 leg_index++)
            {
                before += p_drives[leg_index];
            }

            pos             += 5.0;
            input.b_has_tape = ((tape_at_mm && (!avoidance.b_is_after_tape)) &&
                                ((before + (unsigned)pos) >= tape_at_mm)) &&
                               ((before + (unsigned)pos) < (tape_at_mm + 20U));

            if (pos >= (float64_t)want)
            {
                input.b_is_motion_done = true;
            }
        }
        else if (S5_FINISH == (int)avoidance.state)
        {
            input.b_has_tape       = true;
            input.b_is_motion_done = (0U != avoidance.b_has_issued);
        }
        else if (avoidance.b_has_issued)
        {
            input.b_has_tape = 6 == (int)avoidance.leg;

            if ((input.b_has_tape) &&
                ((!avoidance.recovery.b_is_touching) ||
                 ((uint32_t)(input.now_us - avoidance.recovery.since_us) <
                  50000U)))
            {
                action = step_and_observe(&avoidance, &input);
                continue;
            }

            input.b_is_motion_done = true;
        }
        else
        {
            /* Leave the simulated motion input as supplied. */
        }

        action = step_and_observe(&avoidance, &input);
    }

    assert(S5_COMPLETE == (int)avoidance.state);
}

/** @brief Execute a scripted bypass, retaining issued actions, distances,
 * legs and speeds in the bounded fixture history.
 */
static avoid_obstacle_t scripted (const scr_t * p_output)
{
    s5_config_t      config = s5_config_defaults();
    avoid_obstacle_t avoidance;
    avoid_obstacle_init(&avoidance, &config);
    avoid_obstacle_input_t input = {
        .b_is_healthy = true, .b_is_stopped = true, .b_is_line_ready = true};
    avoid_obstacle_action_t action;
    unsigned                peek_index      = 0;
    unsigned                outbound_index  = 0;
    unsigned                finish_ticks    = 0;
    bool                    b_has_lost_line = false;
    avoid_obstacle_begin(&avoidance, 0);
    input.now_us = 10000;
    (void)step_and_observe(&avoidance, &input);
    input.now_us                     = 20000;
    input.observation.b_is_scan_done = true;
    input.observation.scan_us        = 20000;
    input.observation.profile        = profile();

    if (p_output->closest)
    {
        input.observation.profile.closest_mm = (uint16_t)p_output->closest;
    }

    if (p_output->left_edge)
    {
        input.observation.profile.left_edge_mm  = (int16_t)p_output->left_edge;
        input.observation.profile.right_edge_mm = (int16_t)-p_output->left_edge;
    }

    action         = step_and_observe(&avoidance, &input);
    g_record_count = 0;

    for (unsigned tick = 0; ((tick < (unsigned)RECOVERY_RUN_TICK_LIMIT) &&
                             ((int)avoidance.state != S5_COMPLETE)) &&
                            ((int)avoidance.state != S5_FAULT);
         tick++)
    {
        if ((action.kind != (unsigned)S5_ACT_NONE) && (g_record_count < 96U))
        {
            g_records[g_record_count++] =
                (rec_t){action.kind, action.value, avoidance.leg,
                        avoidance.pass, action.speed_mms};
        }

        input.now_us                      += 10000U;
        input.observation.range_us         = input.now_us;
        input.observation.b_is_range_valid = true;
        input.observation.range_mm         = 2000;

        if (((S5_MOVING == (int)avoidance.state) &&
             (1 == (int)avoidance.leg)) &&
            (!avoidance.b_has_issued))
        {
            input.observation.range_mm =
                (uint16_t)(outbound_index < p_output->outbound_count
                               ? p_output->outbound_range[outbound_index]
                               : 2000U);

            if (outbound_index < p_output->outbound_count)
            {
                outbound_index++;
            }
        }

        if (avoidance.b_is_backing)
        {
            input.observation.range_mm = 60;
        }

        if ((unsigned)S5_ACT_PEEK == action.kind)
        {
            unsigned side = peek_index < p_output->side_peek_count
                                ? p_output->peek_side[peek_index]
                                : 2000U;
            peek_index++;
            input.observation.b_is_scan_done = true;
            input.observation.scan_us        = input.now_us;

            if (S5_ROUTE_LEFT == (int)avoidance.plan.route)
            {
                input.observation.profile.right_clear_mm = (uint16_t)side;
            }
            else
            {
                input.observation.profile.left_clear_mm = (uint16_t)side;
            }
        }

        input.b_is_motion_done = false;
        input.b_has_tape       = false;

        if (S5_FINISH == (int)avoidance.state)
        {
            finish_ticks++;
            input.b_has_tape =
                (p_output->b_has_finish_tape) &&
                (!(((p_output->b_lose_tape_once) && (!b_has_lost_line)) &&
                   (finish_ticks < 3U)));

            if ((p_output->b_lose_tape_once) && (avoidance.b_has_issued))
            {
                b_has_lost_line  = true;
                input.b_has_tape = true;
            }

            input.b_is_motion_done = (0U != avoidance.b_has_issued);
            input.b_is_stopped     = !avoidance.b_has_issued;
        }
        else if (avoidance.b_has_issued)
        {
            input.b_has_tape =
                (((int)avoidance.leg == p_output->tape_leg) &&
                 ((p_output->tape_pass < 0) ||
                  ((int)avoidance.pass == p_output->tape_pass))) ||
                (((p_output->alternate_tape_leg) &&
                  ((int)avoidance.leg == p_output->alternate_tape_leg)) &&
                 (!((5U == avoidance.leg) && (avoidance.b_is_after_tape))));

            if ((input.b_has_tape) &&
                ((!avoidance.recovery.b_is_touching) ||
                 ((uint32_t)(input.now_us - avoidance.recovery.since_us) <
                  50000U)))
            {
                action = step_and_observe(&avoidance, &input);
                continue;
            }

            input.b_is_motion_done = true;
            input.b_is_stopped     = true;
        }
        else
        {
            input.b_is_stopped = true;
        }

        action = step_and_observe(&avoidance, &input);
    }

    if ((action.kind != (unsigned)S5_ACT_NONE) && (g_record_count < 96U))
    {
        g_records[g_record_count++] =
            (rec_t){action.kind, action.value, avoidance.leg, avoidance.pass,
                    action.speed_mms};
    }

    return avoidance;
}

/** @brief Find the first matching action and leg at or after the supplied
 * index. Return minus one when absent.
 */
static int rec_find (avoid_obstacle_action_kind_t kind, unsigned leg,
                     unsigned from)
{
    for (unsigned record_index = from; record_index < g_record_count;
         record_index++)
    {
        if ((g_records[record_index].kind == kind) &&
            (g_records[record_index].leg == leg))
        {
            return (int)record_index;
        }
    }

    return -1;
}

/** @brief Check the scripted return-to-line, blocked-side and
 * obstacle-geometry decisions against the recorded motion actions.
 */
static void rules_17nov (void)
{
    const s5_config_t config = s5_config_defaults();
    const unsigned    pad    = config.half_width_mm + config.margin_mm;
    {
        /* Straight, line crossed on the return: stop, 135 mm, swerve back
         * toward the bypass side, stopped on the tape, hand back. */
        scr_t            scenario     = {.tape_leg           = 6,
                                         .tape_pass          = -1,
                                         .alternate_tape_leg = 5,
                                         .b_has_finish_tape  = true};
        avoid_obstacle_t detour_state = scripted(&scenario);
        int              turn_index   = rec_find(S5_ACT_TURN, 6, 0);
#ifndef NDEBUG
        const int stop_record = rec_find(S5_ACT_STOP, 5, 0);
        assert(stop_record >= 0 &&
               (int)S5_IR_AXLE_MM ==
                   g_records[rec_find(S5_ACT_DRIVE, 5, (unsigned)stop_record)]
                       .value);
#endif
        assert(S5_COMPLETE == (int)detour_state.state &&
               !detour_state.b_has_swapped && turn_index >= 0 &&
               (S5_SWERVE_OUT_DEG + S5_BYPASS_DEG) ==
                   g_records[turn_index].value);
    }
    {
        /* Converging while passing: a peek 70 mm away (gap 67 < 93 - 8) turns
         * the car away by min(10, (93-67+1)/2) and peeks again; the turn-in
         * then aims 45 deg off the passing heading (45 + 10). */
        scr_t            scenario     = {.peek_side         = {70, 97},
                                         .side_peek_count   = 2,
                                         .tape_leg          = 6,
                                         .tape_pass         = -1,
                                         .b_has_finish_tape = true};
        avoid_obstacle_t detour_state = scripted(&scenario);
        int              away_turn    = rec_find(S5_ACT_TURN, 3, 0);
        int              return_turn  = rec_find(S5_ACT_TURN, 4, 0);
        int              repeat_peek  = -1;
        assert(S5_COMPLETE == (int)detour_state.state && away_turn >= 0 &&
               g_records[away_turn].value == (int)S5_SIDE_TURN_MAX_DEG &&
               1U == detour_state.side_turns);

        for (int record_index = away_turn; record_index < (int)g_record_count;
             record_index++)
        {
            if ((unsigned)S5_ACT_PEEK == g_records[record_index].kind)
            {
                repeat_peek = record_index;
                break;
            }
        }

        assert(repeat_peek > away_turn); /* Peek again after turning away. */
        assert(return_turn >= 0 &&
               g_records[return_turn].value ==
                   -(S5_BYPASS_DEG + (int)S5_SIDE_TURN_MAX_DEG));
        assert((70U * 966U) / 1000U + S5_SIDE_TOL_MM < pad);
    }
    {
        /* Drifted away while passing: the last side peek says 150 mm (gap 144):
         * the return diagonal covers that, not just the planned offset. */
        scr_t            scenario             = {.peek_side         = {150},
                                                 .side_peek_count   = 1,
                                                 .tape_leg          = 6,
                                                 .tape_pass         = -1,
                                                 .b_has_finish_tape = true};
        avoid_obstacle_t detour_state         = scripted(&scenario);
        int              straight_drive_index = rec_find(S5_ACT_DRIVE, 5, 0);
        unsigned         edge = detour_state.plan.offset_mm - (pad + 5U);
        unsigned         lat  = edge + ((150U * 966U) / 1000U);
        assert(S5_COMPLETE == (int)detour_state.state &&
               straight_drive_index >= 0 &&
               g_records[straight_drive_index].value ==
                   (int)((lat * 1414U + 999U) / 1000U) &&
               lat > detour_state.plan.offset_mm);
    }
    {
        /* Bend toward the bypass side: the line comes under the car while
         * passing (leg 3, after the front face). Stop, 135 mm to put the axle
         * on it, then swerve toward the bypass side from the passing heading.
         */
        scr_t            scenario     = {.peek_side          = {97},
                                         .side_peek_count    = 1,
                                         .tape_leg           = 3,
                                         .tape_pass          = S5_PASS_EXTRA,
                                         .alternate_tape_leg = 6,
                                         .b_has_finish_tape  = true};
        avoid_obstacle_t detour_state = scripted(&scenario);
        int              stop_index   = -1;
        int              drive_index;
        int              track_turn;

        for (unsigned record_index = 0; record_index < g_record_count;
             record_index++)
        {
            /* This is the first stop. */
            if ((unsigned)S5_ACT_STOP == g_records[record_index].kind)
            {
                stop_index = (int)record_index;
                break;
            }
        }

        assert(stop_index >= 0);
        drive_index = rec_find(S5_ACT_DRIVE, 5, (unsigned)stop_index);
        track_turn  = rec_find(S5_ACT_TURN, 6, (unsigned)stop_index);
        assert(drive_index > stop_index &&
               g_records[drive_index].value == (int)S5_IR_AXLE_MM &&
               rec_find(S5_ACT_TURN, 4, 0) < 0); /* Skip the turn in. */
        assert(track_turn > drive_index &&
               S5_SWERVE_OUT_DEG == g_records[track_turn].value &&
               S5_COMPLETE == (int)detour_state.state);
    }
    {
        /* Hand-back check: braking carried the IR off the tape. Turn back
         * slowly by S5_CENTRE_DEG the way it came, stop on the tape, then hand
         * back. */
        scr_t            scenario     = {.tape_leg          = 6,
                                         .tape_pass         = -1,
                                         .b_has_finish_tape = true,
                                         .b_lose_tape_once  = true};
        avoid_obstacle_t detour_state = scripted(&scenario);
        int              turn_index   = -1;

        for (unsigned record_index = 0; record_index < g_record_count;
             record_index++)
        {
            if ((((unsigned)S5_ACT_TURN == g_records[record_index].kind) &&
                 (6U == g_records[record_index].leg)) &&
                (S5_CENTRE_SPEED_MMS == g_records[record_index].speed))
            {
                turn_index = (int)record_index;
            }
        }

        int swerve_index =
            rec_find(S5_ACT_TURN, 6, 0); /* Keep the sweep that found tape. */
        assert(S5_COMPLETE == (int)detour_state.state && turn_index >= 0 &&
               swerve_index >= 0 && swerve_index < turn_index &&
               g_records[turn_index].value == (g_records[swerve_index].value > 0
                                                   ? -S5_CENTRE_DEG
                                                   : S5_CENTRE_DEG));
    }
#if S5_ALLOW_OTHER_SIDE
    {
        /* One blocked reading on the left diagonal is not believed: the next
         * two are clear, so the car stays LEFT. */
        scr_t            scenario     = {.outbound_range    = {150},
                                         .outbound_count    = 1,
                                         .tape_leg          = 6,
                                         .tape_pass         = -1,
                                         .b_has_finish_tape = true};
        avoid_obstacle_t detour_state = scripted(&scenario);
        assert(S5_COMPLETE == (int)detour_state.state &&
               !detour_state.b_has_swapped &&
               S5_ROUTE_LEFT == (int)detour_state.plan.route);
    }
    {
        /* Blocked on S5_BLOCK_CONFIRM readings: 2 x 45 deg to the right
         * diagonal and go round on the RIGHT (the right side is a real route).
         */
        scr_t            scenario     = {.outbound_range    = {150, 150, 150},
                                         .outbound_count    = 3,
                                         .tape_leg          = 6,
                                         .tape_pass         = -1,
                                         .b_has_finish_tape = true};
        avoid_obstacle_t detour_state = scripted(&scenario);
        int              swerve_index = rec_find(S5_ACT_TURN, 0, 1);
        assert(S5_BLOCK_CONFIRM == 3U &&
               S5_COMPLETE == (int)detour_state.state &&
               detour_state.b_has_swapped &&
               S5_ROUTE_RIGHT == (int)detour_state.plan.route);
        assert(swerve_index >= 0 &&
               (-2 * S5_BYPASS_DEG) == g_records[swerve_index].value);
        assert(rec_find(S5_ACT_PEEK, 3, 0) >= 0 &&
               150 == g_records[rec_find(S5_ACT_PEEK, 3, 0)]
                          .value); /* Point the sensor left. */
    }
    {
        /* Left needs more room than reversing gives but the right fits:
         * take the right before turning at all. Face 160 + 75 = 235 mm ahead
         * of the axle; left edge 240 needs 90+240+30-235 = 125 > 100 mm of
         * reversing, right edge 210 needs 95 (reverse 95, then turn right). */
        s5_config_t      test_config = s5_config_defaults();
        avoid_obstacle_t detour_state;
        avoid_obstacle_init(&detour_state, &test_config);
        avoid_obstacle_input_t  input = {.b_is_healthy    = true,
                                         .b_is_stopped    = true,
                                         .b_is_line_ready = true};
        avoid_obstacle_action_t action;
        int                     first_turn = 0;
        avoid_obstacle_begin(&detour_state, 0);
        input.now_us = 10000;
        (void)step_and_observe(&detour_state, &input);
        /* clang-format off */
        input.now_us                            = 20000;
        input.observation.b_is_scan_done        = true;
        input.observation.scan_us               = 20000;
        input.observation.profile               = profile();
        input.observation.profile.closest_mm    = 160;
        input.observation.profile.left_edge_mm  = 240;
        input.observation.profile.right_edge_mm = -210;
        input.observation.profile.width_mm      = 450;
        action                                  =
            step_and_observe(&detour_state, &input);
        /* clang-format on */
        assert(S5_MOVING == (int)detour_state.state);

        for (unsigned tick_index = 0;
             ((tick_index < DETOUR_FIRST_TURN_WAIT_TICKS) &&
              (S5_MOVING == (int)detour_state.state)) &&
             (!first_turn);
             tick_index++)
        {
            /* clang-format off */
            input.now_us                      += 10000U;
            input.observation.range_us         = input.now_us;
            input.observation.b_is_range_valid = true;
            input.observation.range_mm         =
                detour_state.b_is_backing ? 60 : 2000;
            input.b_is_motion_done             =
                (0U != detour_state.b_has_issued);
            action                             =
                step_and_observe(&detour_state, &input);
            /* clang-format on */

            if ((unsigned)S5_ACT_DRIVE == action.kind)
            {
                assert(action.value == -95 && detour_state.b_has_swapped);
            }

            if ((((unsigned)S5_ACT_TURN == action.kind) &&
                 (0U == detour_state.leg)) &&
                (!detour_state.b_is_aiming))
            {
                first_turn = action.value;
            }
        }

        assert(detour_state.b_has_swapped &&
               S5_ROUTE_RIGHT == (int)detour_state.plan.route &&
               first_turn == -S5_BYPASS_DEG);
    }
#endif
    {
        /* A picture wider than 1.5 x the largest obstacle (obstacle + wall) is
         * not aimed at. */
        profile_obstacle_t scan_profile = profile();
        scan_profile.left_edge_mm       = 250;
        scan_profile.right_edge_mm      = -60;
        scan_profile.width_mm           = 310;
        assert(avoid_obstacle_aim_deg(&scan_profile) == 0);
        scan_profile.width_mm = 290;
        assert(avoid_obstacle_aim_deg(&scan_profile) != 0);
    }
    puts("PASS: 17 Nov bypass rules: swerve stops on the tape, side check "
         "turns away while passing, measured return distance, line seen while "
         "passing, hand-back re-centring, confirmed block -> right side, too "
         "close for left -> right, no aim at a wide picture");
}

/* End of file. */

