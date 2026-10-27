/* Course simulator for the "tape between the sensors" line-following profile.
 *
 * Runs the REAL code that goes on the car: Buddy 3 navigation (calibration,
 * reflectance, line detection, steering), the s3_drive controller with the
 * course profile (course_profile.h), and Buddy 2's motion controller, PID and
 * estimator. Only the physical world is modelled here:
 *   - two driven wheels (first-order speed lag, static friction / deadband),
 *     quadrature counts from wheel travel (2534 counts per 65.4 mm wheel turn);
 *   - IR sensors as small round spots over 18 mm black tape on white, using
 *     the built-in calibration's raw white/black levels.
 * The physical constants marked GUESS are not measured; results show whether
 * the LOGIC works for a plausible car, not that the real car will.
 *
 * Each scenario prints one line; the program fails if any expectation fails. */
#include "motion_controller.h"
#include "motion_estimation.h"
#include "motion_lease.h"
#include "motor_control.h"
#include "wheel_encoder.h"
#include "course_profile.h"
#include "builtin_calibration.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ---- Car (measured where stated) --------------------------------------- */
#define TRACK_M       0.116    /* Buddy 2 TRACK_WIDTH_M                    */
#define WHEEL_D_M     0.0654   /* Buddy 2 WHEEL_DIAMETER_M                 */
#define COUNTS_REV    2534.0   /* Buddy 2 calibration, 21 Oct              */
#define VMAX_MPS      0.884    /* Buddy 2 calibration, full duty           */
#define TAU_S         0.06     /* GUESS: wheel speed time constant         */
#define BREAKAWAY     0.08     /* GUESS: duty needed to start a still wheel */
#define KEEP_MOVING   0.03     /* GUESS: below this duty a wheel stops     */
#define TAPE_W        0.018    /* course spec: 18 mm                       */
#define SPOT_R        0.0025   /* GUESS: sensing spot radius               */
#define MIN_WHEEL     0.06f    /* integration velocity floor (m/s)         */

/* ---- Tape: straight segments and arcs ----------------------------------- */
typedef struct { int arc; double x0, y0, x1, y1, cx, cy, r, a0, a1; } piece_t;
#define MAX_PIECES 16
static piece_t track[MAX_PIECES];
static int pieces;
static double tape_end_x = 1e9; /* tape removed beyond this x (scenario 9) */

static void add_line(double x0, double y0, double x1, double y1)
{ track[pieces++] = (piece_t){0, x0, y0, x1, y1, 0, 0, 0, 0, 0}; }
static void add_arc(double cx, double cy, double r, double a0, double a1)
{ track[pieces++] = (piece_t){1, 0, 0, 0, 0, cx, cy, r, a0, a1}; }

static double angle_in(double a, double lo, double hi)
{   /* is angle a inside the sweep lo..hi (either direction)? */
    double s = hi - lo, t = a - lo;
    while (t < 0) t += 2 * M_PI;
    while (t >= 2 * M_PI) t -= 2 * M_PI;
    if (s >= 0) return t <= s;
    t -= 2 * M_PI;
    return t >= s;
}
static double dist_to_tape(double x, double y)
{
    double best = 1e9;
    if (x > tape_end_x) return best;
    for (int i = 0; i < pieces; i++) {
        const piece_t *p = &track[i];
        double d;
        if (!p->arc) {
            double vx = p->x1 - p->x0, vy = p->y1 - p->y0;
            double t = ((x - p->x0) * vx + (y - p->y0) * vy) / (vx * vx + vy * vy);
            t = t < 0 ? 0 : (t > 1 ? 1 : t);
            d = hypot(x - (p->x0 + t * vx), y - (p->y0 + t * vy));
        } else {
            double a = atan2(y - p->cy, x - p->cx);
            if (angle_in(a, p->a0, p->a1)) {
                d = fabs(hypot(x - p->cx, y - p->cy) - p->r);
            } else {
                double e0 = hypot(x - (p->cx + p->r * cos(p->a0)), y - (p->cy + p->r * sin(p->a0)));
                double e1 = hypot(x - (p->cx + p->r * cos(p->a1)), y - (p->cy + p->r * sin(p->a1)));
                d = e0 < e1 ? e0 : e1;
            }
        }
        if (d < best) best = d;
    }
    return best;
}
/* Fraction of a round spot covered by the tape (linear edge approximation). */
static double coverage(double x, double y)
{
    double c = (TAPE_W / 2 + SPOT_R - dist_to_tape(x, y)) / (2 * SPOT_R);
    return c < 0 ? 0 : (c > 1 ? 1 : c);
}

/* ---- Wheels and encoders (hardware boundary of Buddy 2) ----------------- */
static float duty[2];
static double wheel_v[2], wheel_pos[2];
static int32_t counts[2];
int32_t wheel_encoder_count(wheel_id_t w) { return counts[w]; }
uint32_t wheel_encoder_bits(wheel_id_t w) { return (uint32_t)counts[w] & WHEEL_COUNT_MASK; }
int32_t wheel_encoder_delta(uint32_t now_bits, uint32_t then_bits)
{
    uint32_t delta = (now_bits - then_bits) & WHEEL_COUNT_MASK;
    return (delta & (1UL << (WHEEL_COUNT_BITS - 1U)))
               ? (int32_t)delta - (int32_t)(WHEEL_COUNT_MASK + 1U) : (int32_t)delta;
}
uint32_t wheel_encoder_glitches(wheel_id_t w) { (void)w; return 0U; }
void motor_control_set(wheel_id_t w, float value) { duty[w] = value; }
void motor_control_brake_all(void) { duty[0] = duty[1] = 0.0f; }

/* ---- Car pose and Buddy 2 boundary --------------------------------------- */
static double px, py, ph;            /* axle centre, heading (rad)           */
static double sensor_ahead = 0.060;  /* axle -> front sensor heads           */
static double sensor_gap = 0.022;    /* head centre to head centre           */
static uint32_t now, lease_stamp;
static bool lease;
static motion_report_t reports[MOTION_REPORT_QUEUE_LEN];
static unsigned report_count, turns_done, advances;
static s3_drive_t drive;

static void post(const motion_report_t *r)
{ if (report_count < MOTION_REPORT_QUEUE_LEN) reports[report_count++] = *r; }
static bool receive(motion_report_t *r)
{
    if (report_count == 0U) return false;
    *r = reports[0];
    memmove(&reports[0], &reports[1], (report_count - 1U) * sizeof reports[0]);
    report_count--;
    if (r->outcome == MOTION_OUTCOME_DONE) motion_controller_ack_complete(r->command_id);
    return true;
}
static float floor_wheel(float v) { return (v > 0.0f && v < MIN_WHEEL) ? MIN_WHEEL : v; }
static motion_result_t velocity(float l, float r)
{ return motion_controller_request_velocity(floor_wheel(l), floor_wheel(r), now); }
static motion_result_t turn_deg(float deg, float speed)   /* motion.c mapping */
{
    float arc = fabsf(deg) / 360.0f * (float)M_PI * (float)TRACK_M;
    int32_t sl = deg > 0.0f ? -1 : 1;
    turns_done++;
    return motion_controller_request_move(MOTION_TURN, sl, -sl, arc, speed, now);
}
static motion_result_t manoeuvre(manoeuvre_t m)
{
    switch (m) {
    case MANOEUVRE_LEFT: return turn_deg(90.0f, 0.15f);
    case MANOEUVRE_RIGHT: return turn_deg(-90.0f, 0.15f);
    case MANOEUVRE_U_TURN: return turn_deg(180.0f, 0.15f);
    case MANOEUVRE_STRAIGHT:
        return motion_controller_request_move(MOTION_DRIVE, 1, 1, 0.05f, 0.20f, now);
    default: return MOTION_ERR_BAD_ARG;
    }
}
static motion_result_t drive_m(float m, float speed)
{
    int32_t s = m >= 0.0f ? 1 : -1;
    advances++;
    return motion_controller_request_move(MOTION_DRIVE, s, s, fabsf(m), speed, now);
}
static void brake(void) { motion_controller_stop(); }
static bool reset(void) { return motion_controller_reset(); }
static void heartbeat(bool on) { lease = on; if (on) lease_stamp = now; }

static void physics_10ms(void)
{
    const double mpc = M_PI * WHEEL_D_M / COUNTS_REV;
    for (int k = 0; k < 10; k++) {
        for (int w = 0; w < 2; w++) {
            double target = duty[w] * VMAX_MPS, ad = fabs(duty[w]);
            if (wheel_v[w] == 0.0 && ad < BREAKAWAY) target = 0.0;   /* static */
            if (ad < KEEP_MOVING) target = 0.0;
            wheel_v[w] += (target - wheel_v[w]) * (0.001 / TAU_S);
            if (fabs(wheel_v[w]) < 1e-4 && target == 0.0) wheel_v[w] = 0.0;
            wheel_pos[w] += wheel_v[w] * 0.001;
            counts[w] = (int32_t)floor(wheel_pos[w] / mpc);
        }
        double v = (wheel_v[0] + wheel_v[1]) / 2, om = (wheel_v[1] - wheel_v[0]) / TRACK_M;
        px += v * cos(ph) * 0.001;
        py += v * sin(ph) * 0.001;
        ph += om * 0.001;
    }
}
static void sensor_xy(int which, double *x, double *y)   /* 0 left, 1 right */
{
    double fx = px + sensor_ahead * cos(ph), fy = py + sensor_ahead * sin(ph);
    double side = (which == 0 ? 1 : -1) * sensor_gap / 2;
    *x = fx - side * sin(ph);
    *y = fy + side * cos(ph);
}
static uint16_t raw(int which)
{
    static const uint16_t cal[2][4] = {S3_BUILTIN_CAL_LEFT, S3_BUILTIN_CAL_RIGHT};
    double x, y, white, black;
    sensor_xy(which, &x, &y);
    white = (cal[which][0] + cal[which][1]) / 2.0;
    black = (cal[which][2] + cal[which][3]) / 2.0;
    return (uint16_t)(white + coverage(x, y) * (black - white));
}
static void tick(void)
{
    motion_feedback_t fb;
    s3_sensor_frame_t f = {0};
    physics_10ms();
    now += 10000U;
    motion_estimation_update(0.01f);
    if (motion_lease_expired(lease, now, lease_stamp)) { motion_controller_navigation_timeout(); lease = false; }
    motion_controller_step(0.01f, now);
    fb = (motion_feedback_t){motion_estimation_robot_distance(), now, 1U,
                             motion_controller_state(), motion_controller_fault(), true,
                             motion_controller_is_stopped(), motion_controller_command_id(), 0U};
    f.timestamp_us = now;
    f.sensor[S3_SENSOR_BARCODE] = (s3_raw_sample_t){now, 158U, 0, true, true};
    f.sensor[S3_SENSOR_LEFT] = (s3_raw_sample_t){now, raw(0), 0, true, true};
    f.sensor[S3_SENSOR_RIGHT] = (s3_raw_sample_t){now, raw(1), 0, true, true};
    (void)s3_drive_process(&drive, &f, &fb);
}

/* ---- Scenario harness ---------------------------------------------------- */
static void calibrate(void)
{
    static const uint16_t table[S3_SENSOR_COUNT][4] = {
        S3_BUILTIN_CAL_BARCODE, S3_BUILTIN_CAL_LEFT, S3_BUILTIN_CAL_RIGHT};
    for (unsigned i = 0U; i < S3_SENSOR_COUNT; i++) {
        s3_calibration_collector_t c;
        s3_calibration_begin(&c);
        for (unsigned k = 0U; k < 12U; k++) {
            (void)s3_calibration_add(&c, IR_SURFACE_WHITE, (uint16_t)(table[i][0] + k % 2U * (table[i][1] - table[i][0])));
            (void)s3_calibration_add(&c, IR_SURFACE_BLACK, (uint16_t)(table[i][2] + k % 2U * (table[i][3] - table[i][2])));
        }
        if (!s3_navigation_apply_calibration(&drive.navigation, (s3_sensor_id_t)i, &c)) {
            puts("FAIL: built-in calibration rejected"); exit(1);
        }
    }
}
static void setup(double x, double y, double heading, double advance_m)
{
    s3_navigation_config_t cfg = s3_navigation_defaults();
    const s3_drive_port_t port = {velocity, manoeuvre, brake, receive,
                                  motion_controller_command_id, reset, heartbeat};
    s3_drive_options_t opt = s3_course_options();
    now = 0U; lease = false; report_count = turns_done = advances = 0U;
    memset(duty, 0, sizeof duty); memset(wheel_v, 0, sizeof wheel_v);
    memset(wheel_pos, 0, sizeof wheel_pos); memset(counts, 0, sizeof counts);
    px = x; py = y; ph = heading;
    motion_estimation_init((float)(M_PI * WHEEL_D_M / COUNTS_REV), (float)TRACK_M);
    motion_controller_init((float)VMAX_MPS, 0.0f, 0.0f, post);
    cfg.acquisition.enabled_mask = S3_ALL_SENSORS;
    s3_course_navigation(&cfg);
    if (!s3_drive_init(&drive, &cfg, &port)) { puts("FAIL: init"); exit(1); }
    opt.turn_advance_m = (float)advance_m;
    opt.advance_mps = 0.10f;
    opt.drive = drive_m;
    if (!s3_drive_set_options(&drive, &opt)) { puts("FAIL: options"); exit(1); }
    calibrate();
    for (int i = 0; i < 20; i++) tick();
    if (!s3_drive_start(&drive, now)) { puts("FAIL: START refused"); exit(1); }
}
/* Distance from the middle point between the two sensors to the tape. */
static double centre_error(void)
{
    double lx, ly, rx, ry;
    sensor_xy(0, &lx, &ly); sensor_xy(1, &rx, &ry);
    return dist_to_tape((lx + rx) / 2, (ly + ry) / 2);
}
static const char *state_name(s3_drive_state_t s)
{
    static const char *n[] = {"OFF", "FOLLOW", "BRAKING", "MANOEUVRE", "REACQUIRE", "HALTED", "FAULT"};
    return n[s];
}
static const char *reason_name(s3_drive_reason_t r)
{
    static const char *n[] = {"ok", "sensor fault", "motion fault", "rejected", "line lost",
                              "no direction", "unmapped", "timeout", "report fault", "bad input"};
    return n[r];
}
static int failures;
static void verdict(const char *name, bool ok, const char *detail)
{
    printf("%-4s %-52s %s\n", ok ? "PASS" : "FAIL", name, detail);
    if (!ok) failures++;
}
/* Drive until x/y condition or time out; track worst centre error after the
 * first 0.3 s (start transient) while following. */
static double run(double seconds, bool (*until)(void))
{
    double worst = 0;
    for (unsigned i = 0; i < (unsigned)(seconds * 100); i++) {
        tick();
        if (i > 30 && drive.state == S3_DRIVE_FOLLOW) {
            double e = centre_error();
            if (e > worst) worst = e;
        }
        if (drive.state == S3_DRIVE_HALTED || drive.state == S3_DRIVE_FAULT) break;
        if (until && until()) break;
    }
    return worst;
}

static double goal_x, goal_y;
static bool past_x(void) { return px > goal_x; }
static bool below_y(void) { return py < goal_y; }
static bool above_y(void) { return py > goal_y; }
static bool left_of_x(void) { return px < goal_x; }

static void scenario_straight(double offset)
{
    char d[160];
    pieces = 0; tape_end_x = 1e9;
    add_line(-0.2, 0, 2.0, 0);
    setup(0, offset, 0, 0);
    goal_x = 1.5;
    double worst = run(25, past_x);
    snprintf(d, sizeof d, "reached x=%.2f m, worst centre error %.1f mm, %s", px, worst * 1000,
             state_name(drive.state));
    char n[64]; snprintf(n, sizeof n, "straight 1.5 m, start %+.0f mm off centre", offset * 1000);
    verdict(n, px > 1.5 && drive.state == S3_DRIVE_FOLLOW && worst < 0.012, d);
}
/* Straight, then a 180 degree curve of radius r (left or right), then back. */
static void scenario_curve(double r, int dir, bool expect_ok)
{
    char d[160], n[64];
    pieces = 0; tape_end_x = 1e9;
    add_line(-0.2, 0, 0.4, 0);
    if (dir > 0) { add_arc(0.4, r, r, -M_PI / 2, M_PI / 2); add_line(0.4, 2 * r, -0.4, 2 * r); }
    else { add_arc(0.4, -r, r, M_PI / 2, -M_PI / 2); add_line(0.4, -2 * r, -0.4, -2 * r); }
    setup(0, 0, 0, 0);
    goal_x = 0.0;
    for (int i = 0; i < 300; i++) tick();   /* get going before the stop test */
    double worst = run(40, left_of_x);
    bool ok = px < 0.0 && drive.state == S3_DRIVE_FOLLOW && worst < 0.015 &&
              fabs(py - dir * 2 * r) < 0.02;
    snprintf(d, sizeof d, "end (%.2f, %.2f) m, worst centre error %.1f mm, %s", px, py,
             worst * 1000, state_name(drive.state));
    snprintf(n, sizeof n, "%s U-shaped curve, radius %.0f cm%s", dir > 0 ? "left" : "right",
             r * 100, expect_ok ? "" : " (expected to fail)");
    verdict(n, ok == expect_ok, d);
}
static void scenario_s_bend(double r)
{
    char d[160], n[64];
    pieces = 0; tape_end_x = 1e9;
    add_line(-0.2, 0, 0.3, 0);
    add_arc(0.3, r, r, -M_PI / 2, 0);           /* left 90 */
    add_arc(0.3 + 2 * r, r, r, M_PI, M_PI / 2);  /* right 90 */
    add_line(0.3 + 2 * r, 2 * r, 1.5, 2 * r);
    setup(0, 0, 0, 0);
    goal_x = 1.2;
    double worst = run(40, past_x);
    bool ok = px > 1.2 && fabs(py - 2 * r) < 0.012 && drive.state == S3_DRIVE_FOLLOW && worst < 0.015;
    snprintf(d, sizeof d, "end (%.2f, %.2f) m, worst centre error %.1f mm, %s", px, py, worst * 1000,
             state_name(drive.state));
    snprintf(n, sizeof n, "S-bend, two 90 degree curves of radius %.0f cm", r * 100);
    verdict(n, ok, d);
}
/* A 90 degree cross at x = 0.6. */
static void cross_track(void)
{
    pieces = 0; tape_end_x = 1e9;
    add_line(-0.2, 0, 1.6, 0);
    add_line(0.6, -0.6, 0.6, 0.6);
}
static void scenario_cross_straight(s3_nav_action_t pending)
{
    char d[160], n[64];
    cross_track();
    setup(0, 0, 0, 0);
    if (pending != S3_NAV_NONE) { drive.pending = pending; drive.pending_us = now; }
    goal_x = 1.2;
    double worst = run(30, past_x);
    bool ok = px > 1.2 && fabs(py) < 0.008 && drive.state == S3_DRIVE_FOLLOW && worst < 0.012;
    snprintf(d, sizeof d, "end (%.2f, %.3f) m, worst centre error %.1f mm, %s %s", px, py,
             worst * 1000, state_name(drive.state), reason_name(drive.reason));
    snprintf(n, sizeof n, "crossing, %s", pending == S3_NAV_NONE ? "no barcode: straight across"
                                                                    : "STRAIGHT action: across");
    verdict(n, ok, d);
}
static void scenario_turn(s3_nav_action_t action, double advance_m, bool expect_ok)
{
    char d[200], n[96];
    cross_track();
    setup(0, 0, 0, advance_m);
    drive.pending = action; drive.pending_us = now;
    bool ok;
    if (action == S3_NAV_UTURN) {
        goal_x = -0.1;
        (void)run(40, left_of_x);
        ok = px < -0.1 && fabs(py) < 0.01;
    } else if (action == S3_NAV_LEFT) {
        goal_y = 0.4;
        (void)run(40, above_y);
        ok = py > 0.4 && fabs(px - 0.6) < 0.01;
    } else {
        goal_y = -0.4;
        (void)run(40, below_y);
        ok = py < -0.4 && fabs(px - 0.6) < 0.01;
    }
    ok = ok && drive.state == S3_DRIVE_FOLLOW;
    snprintf(d, sizeof d, "end (%.2f, %.2f) m heading %.0f deg, %s %s, advances=%u", px, py,
             fmod(ph * 180 / M_PI + 720, 360), state_name(drive.state), reason_name(drive.reason),
             advances);
    snprintf(n, sizeof n, "%s at crossing, advance %.0f mm%s",
             action == S3_NAV_LEFT ? "LEFT" : action == S3_NAV_RIGHT ? "RIGHT" : "U-TURN",
             advance_m * 1000, expect_ok ? "" : " (expected: halt)");
    if (!expect_ok) {
        ok = drive.state == S3_DRIVE_HALTED && drive.reason == S3_DRIVE_LINE_LOST;
        verdict(n, ok, d);
    } else {
        verdict(n, ok, d);
    }
}
static void scenario_tape_gone(void)
{
    char d[160];
    pieces = 0; tape_end_x = 0.6;
    add_line(-0.2, 0, 2.0, 0);
    setup(0, 0, 0, 0);
    goal_x = 1.2;
    (void)run(30, past_x);
    snprintf(d, sizeof d, "end (%.2f, %.3f) m, %s (no line-lost stop by design; GP21 stops it)",
             px, py, state_name(drive.state));
    verdict("tape ends at 0.6 m: keeps driving straight", px > 1.2 && drive.state == S3_DRIVE_FOLLOW, d);
}

int main(int argc, char **argv)
{
    if (argc > 1) sensor_gap = atof(argv[1]) / 1000.0;
    if (argc > 2) sensor_ahead = atof(argv[2]) / 1000.0;
    const double adv = sensor_ahead + S3_CROSSING_CENTRE_MM / 1000.0;
    printf("Car model: sensor gap %.0f mm, sensors %.0f mm ahead of the axle, tape %.0f mm\n",
           sensor_gap * 1000, sensor_ahead * 1000, TAPE_W * 1000);
    scenario_straight(0.0);
    scenario_straight(0.002);
    scenario_straight(-0.002);
    scenario_curve(0.20, +1, true);
    scenario_curve(0.20, -1, true);
    scenario_curve(0.15, +1, true);
    scenario_curve(0.15, -1, true);
    scenario_s_bend(0.15);
    scenario_cross_straight(S3_NAV_NONE);
    scenario_cross_straight(S3_NAV_STRAIGHT);
    scenario_turn(S3_NAV_LEFT, adv, true);
    scenario_turn(S3_NAV_RIGHT, adv, true);
    scenario_turn(S3_NAV_UTURN, adv, true);
    scenario_turn(S3_NAV_LEFT, 0.0, false);
    scenario_turn(S3_NAV_RIGHT, 0.0, false);
    scenario_tape_gone();
    printf("%s: %d failure(s)\n", failures ? "COURSE SIM FAIL" : "COURSE SIM PASS", failures);
    return failures ? 1 : 0;
}
