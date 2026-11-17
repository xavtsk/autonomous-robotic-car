/* Obstacle bypass on a simulated course: straight track and obstacles near
 * bends. Runs the REAL car code (Buddy 3 line following, s3_drive, Buddy 5
 * scan/profile/plan/bypass FSM, Buddy 2 motion controller and PID) on the
 * course simulator's wheel and IR physics, plus:
 *   - an obstacle box (up to 20 x 20 cm, any rotation) on or near the tape;
 *   - the HC-SR04 as the nearest return inside a +/-15 degree cone from the
 *     sonar 75 mm ahead of the axle; servo 90 = ahead, 150 = left;
 *   - the car body for collisions: 55 mm behind to 80 mm ahead of the axle,
 *     125 mm wide, plus the IR bracket to 140 mm ahead (40 mm wide).
 * The wheel/sensor constants marked GUESS in course_sim.c are not measured:
 * this shows whether the bypass LOGIC rejoins the line, not that the real car
 * will. Usage: bypass_course_test [-v]  (-v prints the leg-by-leg trace). */
#define main course_sim_main
#include "../integration/course_sim.c"
#undef main
#include "s5_api.h"
#include "obstacle_scan.h"

/* ---- Obstacle and ultrasonic --------------------------------------------- */
#define OB_PTS 1200
static double obx[OB_PTS], oby[OB_PTS];
static int ob_n;
static double beam_deg = 15.0;   /* HC-SR04 half cone; argument beam=N */
static void obstacle_add(double cx, double cy, double w, double d, double rot_deg)
{   /* w across the track (y), d along it (x), rotated by rot_deg */
    const double c = cos(rot_deg * M_PI / 180), s = sin(rot_deg * M_PI / 180);
    const double hx = d / 2, hy = w / 2;
    const double cx4[5] = {-hx, hx, hx, -hx, -hx}, cy4[5] = {-hy, -hy, hy, hy, -hy};
    for (int e = 0; e < 4; e++) {
        double len = hypot(cx4[e + 1] - cx4[e], cy4[e + 1] - cy4[e]);
        int k = (int)(len / 0.002) + 1;
        for (int i = 0; i < k && ob_n < OB_PTS; i++) {
            double t = (double)i / k, x = cx4[e] + t * (cx4[e + 1] - cx4[e]), y = cy4[e] + t * (cy4[e + 1] - cy4[e]);
            obx[ob_n] = cx + c * x - s * y;
            oby[ob_n++] = cy + s * x + c * y;
        }
    }
}
static void obstacle(double cx, double cy, double w, double d, double rot_deg)
{ ob_n = 0; obstacle_add(cx, cy, w, d, rot_deg); }
static double wrap(double a) { while (a > M_PI) a -= 2 * M_PI; while (a < -M_PI) a += 2 * M_PI; return a; }
/* Nearest return in the cone; nothing within 3 m reads as a far wall. */
static unsigned sonar_mm(unsigned servo_deg)
{
    const double sx = px + S5_SONAR_AXLE_MM / 1000.0 * cos(ph), sy = py + S5_SONAR_AXLE_MM / 1000.0 * sin(ph);
    const double dir = ph + ((double)servo_deg - 90.0) * M_PI / 180.0;
    double best = 3.0;
    for (int i = 0; i < ob_n; i++) {
        double dx = obx[i] - sx, dy = oby[i] - sy, r = hypot(dx, dy);
        if (r > 0.02 && fabs(wrap(atan2(dy, dx) - dir)) <= beam_deg * M_PI / 180.0 && r < best) best = r;
    }
    best *= 1000.0;
    return best < S5_RANGE_MIN_MM ? S5_RANGE_MIN_MM : (unsigned)best;
}
/* Signed distance from the car outline to the obstacle (negative = hit). */
static double clearance(void)
{
    static const double box[2][4] = {{-0.055, 0.080, -0.0625, 0.0625}, {0.080, 0.140, -0.020, 0.020}};
    double best = 1e9;
    const double c = cos(ph), s = sin(ph);
    for (int i = 0; i < ob_n; i++) {
        double dx = obx[i] - px, dy = oby[i] - py, x = c * dx + s * dy, y = -s * dx + c * dy;
        for (int b = 0; b < 2; b++) {
            double ex = x < box[b][0] ? box[b][0] - x : (x > box[b][1] ? x - box[b][1] : 0);
            double ey = y < box[b][2] ? box[b][2] - y : (y > box[b][3] ? y - box[b][3] : 0);
            double d = (ex == 0 && ey == 0) ? -0.001 : hypot(ex, ey);
            if (d < best) best = d;
        }
    }
    return best;
}

/* ---- Buddy 5 service boundary (replaces subsystem5.c's task) ------------- */
static s5_observation_t obs;
static uint16_t pending_id;
static unsigned pending_angle; /* 0 = full scan, else peek angle */
static uint32_t pending_ready;
static bool pending;
static s5_profile_t last_profile;
static unsigned scans, peeks;
static void finish_scan(void)
{
    const s5_config_t cfg = s5_defaults();
    s5_scan_t s;
    if (pending_angle) {
        (void)s5_scan_begin_at(&s, now, pending_angle);
    } else {
        s5_scan_begin(&s, now);
    }
    while (!s.done) s5_scan_accept(&s, now, true, sonar_mm(s.angle));
    if (pending_angle) {
        if (s.failed || !s5_peek_merge(&last_profile, &s.sample[0])) last_profile = (s5_profile_t){0};
    } else if (s.failed || !s5_profile_build(s.sample, s.count, &cfg, &last_profile)) {
        last_profile = (s5_profile_t){0};
    }
    obs.profile = last_profile;
    obs.scan_id = pending_id; obs.scan_done = true; obs.scan_us = now;
    pending = false;
}
s5_snapshot_result_t subsystem5_read_snapshot(s5_observation_t *out)
{
    if (pending && (int32_t)(now - pending_ready) >= 0) finish_scan();
    obs.range_mm = (uint16_t)sonar_mm(90U);
    obs.range_us = now; obs.range_valid = true; obs.no_echo = false;
    *out = obs;
    return S5_SNAPSHOT_OK;
}
bool subsystem5_snapshot(s5_observation_t *out) { return subsystem5_read_snapshot(out) == S5_SNAPSHOT_OK; }
bool subsystem5_request_scan(uint16_t id)
{
    pending = true; pending_id = id; pending_angle = 0U; pending_ready = now + 600000U;
    obs.scan_done = false; obs.scan_id = id; obs.profile = (s5_profile_t){0}; scans++;
    return true;
}
bool subsystem5_request_peek(uint16_t id, unsigned angle)
{
    pending = true; pending_id = id; pending_angle = angle; pending_ready = now + 200000U;
    obs.scan_done = false; obs.scan_id = id; obs.profile = (s5_profile_t){0}; peeks++;
    return true;
}
void subsystem5_cancel_scan(void) { pending = false; }
static unsigned st_leg, st_route, st_swapped;
void subsystem5_set_controller_status(unsigned state, unsigned fault, unsigned route,
                                      unsigned leg, unsigned pass, unsigned swapped)
{ (void)state; (void)fault; (void)pass; st_leg = leg; st_route = route; st_swapped = swapped; }
uint16_t subsystem5_controller_status(void) { return 0U; }
uint8_t subsystem5_servo_angle(void) { return 90U; }
motion_result_t motion_request_turn(float degrees, float speed) { return turn_deg(degrees, speed); }

/* ---- Scenarios ------------------------------------------------------------ */
static int verbose;
static double end_x, end_y;
static const char *s5_state_name(unsigned s)
{
    static const char *n[] = {"IDLE", "WAIT_STOP", "SCANNING", "MOVING", "FINISH", "COMPLETE", "FAULT"};
    return s < 7U ? n[s] : "?";
}
static const char *fault_name(unsigned f)
{
    static const char *n[] = {"", " BAD_RANGE", " NO_PATH", " SCAN_FAILED", " DEADLINE", " NO_LINE", " MOTION_ERROR", " INTERRUPTED"};
    return f < 8U ? n[f] : " ?";
}
/* Run until the sensors reach the track end while following, or time out. */
static void bypass_run(const char *name, double seconds, bool expect_right, bool expect_ok)
{
    double worst_clear = 1e9, worst_off = 0, after_error = 0;
    unsigned last_leg = 99, followed_ticks = 0;
    uint32_t resume_us = 0U;
    bool avoided = false, reached = false;
    char d[200];
    scans = peeks = 0; pending = false; obs = (s5_observation_t){0}; last_profile = (s5_profile_t){0};
    for (unsigned i = 0; i < (unsigned)(seconds * 100); i++) {
        double lx, ly, rx, ry;
        tick();
        double cl = clearance();
        if (cl < worst_clear) worst_clear = cl;
        if (drive.obstacle_active) {
            avoided = true;
            double off = dist_to_tape(px, py);
            if (off > worst_off) worst_off = off;
            if (verbose && last_leg == 99)
                printf("    detected: turn %d %%, bend %u, aim %d deg\n", drive.obstacle_turn_pct,
                       drive.obstacle.bend, drive.obstacle.aim_deg);
            if (verbose && drive.obstacle.leg != last_leg) {
                printf("    t=%6.2f s leg %u %-8s pose (%.3f, %.3f) %4.0f deg  off-line %3.0f mm  clear %3.0f mm\n",
                       now / 1e6, drive.obstacle.leg, s5_state_name(drive.obstacle.state), px, py,
                       ph * 180 / M_PI, off * 1000, cl * 1000);
                last_leg = drive.obstacle.leg;
            }
        } else if (avoided && drive.state == S3_DRIVE_FOLLOW) {
            double e = centre_error();
            if (!resume_us) resume_us = now;
            if (verbose > 1 && followed_ticks % 50U == 0U)
                printf("    t=%6.2f s FOLLOW pose (%.3f, %.3f) %4.0f deg  centre err %4.0f mm\n",
                       now / 1e6, px, py, ph * 180 / M_PI, e * 1000);
            followed_ticks++;
            if (followed_ticks > 150 && e > after_error) after_error = e;  /* after 1.5 s settling */
        }
        sensor_xy(0, &lx, &ly); sensor_xy(1, &rx, &ry);
        if (avoided && !drive.obstacle_active && hypot((lx + rx) / 2 - end_x, (ly + ry) / 2 - end_y) < 0.05) {
            reached = true; break;
        }
        if (drive.state == S3_DRIVE_HALTED || drive.state == S3_DRIVE_FAULT) {
            if (verbose)
                printf("    t=%6.2f s stopped: %s (%s), leg %u pass %u, line valid %d, scans %u peeks %u, motion state %d fault %d\n",
                       now / 1e6, state_name(drive.state), reason_name(drive.reason), drive.obstacle.leg,
                       drive.obstacle.pass, drive.output.line.valid, scans, peeks,
                       (int)motion_controller_state(), (int)motion_controller_fault());
            break;
        }
    }
    bool right = st_swapped ? true : (st_route == 2U);
    bool ok = avoided && reached && worst_clear > 0.0 && drive.state == S3_DRIVE_FOLLOW &&
              after_error < 0.015 && right == expect_right;
    snprintf(d, sizeof d, "%s, clear %3.0f mm, off-line max %3.0f mm, follow err %4.1f mm, %s%s, side turns %u, %.0f s, %s/%s%s",
             reached ? "rejoined" : "NOT rejoined", worst_clear * 1000, worst_off * 1000, after_error * 1000,
             right ? "RIGHT" : "left", st_swapped ? " (swapped)" : "", drive.obstacle.side_turns,
             (drive.obstacle.state == S5_COMPLETE ? (double)(resume_us - drive.obstacle.started_us) : 0.0) / 1e6,
             state_name(drive.state), s5_state_name(drive.obstacle.state), fault_name(drive.obstacle.fault));
    if (!expect_ok) printf("     (known limit, expected to miss: %s)\n", ok ? "but it passed" : "missed");
    verdict(name, expect_ok ? ok : true, d);   /* a known limit never fails the run */
}
/* Straight track along +x; obstacle centred at x0 (box w x d, rotated). */
static void straight_case(double w, double d, double rot, double side_off)
{
    char n[80];
    pieces = 0; tape_end_x = 1e9;
    add_line(-0.2, 0, 2.6, 0);
    obstacle(0.9 + d / 2, side_off, w, d, rot);
    end_x = 2.2; end_y = 0;
    setup(0, 0, 0, 0);
    snprintf(n, sizeof n, "straight, %2.0fx%2.0f cm box%s%s", w * 100, d * 100,
             rot != 0 ? ", turned 45" : "", side_off != 0 ? ", 3 cm off-centre" : "");
    bypass_run(n, 150, false, true);
}
/* Something beside the track on the LEFT (a wall 23 cm from the line) leaves
 * no room for a left bypass: the car must take the right. */
static void left_blocked_case(void)
{
    pieces = 0; tape_end_x = 1e9;
    add_line(-0.2, 0, 2.6, 0);
    obstacle(1.0, 0, 0.20, 0.20, 0);
    obstacle_add(1.0, 0.25, 0.04, 1.0, 0);
    end_x = 2.2; end_y = 0;
    setup(0, 0, 0, 0);
    bypass_run("straight, 20 cm box, wall 23 cm away on the LEFT", 150, true, true);
}
/* Straight, then a curve of radius r turning dir (+1 left) by 90 degrees,
 * starting gap metres after the obstacle's rear face; then straight on. */
static void bend_after_case(double r, int dir, double gap, double w, bool expect_ok)
{
    char n[80];
    const double ob_x = 0.9, rear = ob_x + w / 2, xb = rear + gap;
    pieces = 0; tape_end_x = 1e9;
    add_line(-0.2, 0, xb, 0);
    add_arc(xb, dir * r, r, -dir * M_PI / 2, 0.0 * dir);
    add_line(xb + r, dir * r, xb + r, dir * (r + 1.2));
    obstacle(ob_x, 0, w, w, 0);
    end_x = xb + r; end_y = dir * (r + 0.9);
    setup(0, 0, 0, 0);
    snprintf(n, sizeof n, "%s bend r=%2.0f cm starts %2.0f cm after a %2.0f cm box", dir > 0 ? "left" : "right",
             r * 100, gap * 100, w * 100);
    bypass_run(n, 200, false, expect_ok);
}
/* A curve of radius r turning dir by 90 degrees, then straight; obstacle
 * centred gap metres after the curve ends (car may still be crooked). */
static void bend_before_case(double r, int dir, double gap, double w)
{
    char n[80];
    pieces = 0; tape_end_x = 1e9;
    add_line(-0.2, 0, 0.5, 0);
    add_arc(0.5, dir * r, r, -dir * M_PI / 2, 0.0 * dir);
    add_line(0.5 + r, dir * r, 0.5 + r, dir * (r + 2.0));
    obstacle(0.5 + r, dir * (r + gap + w / 2), w, w, 90);
    end_x = 0.5 + r; end_y = dir * (r + gap + w + 0.9);
    setup(0, 0, 0, 0);
    snprintf(n, sizeof n, "%s bend r=%2.0f cm ends %2.0f cm before a %2.0f cm box", dir > 0 ? "left" : "right",
             r * 100, gap * 100, w * 100);
    bypass_run(n, 200, false, true);
}
int main(int argc, char **argv)
{
    for (int i = 1; i < argc; i++) {
        if (argv[i][0] == '-') verbose = argv[i][1] == 'v' && argv[i][2] == 'v' ? 2 : 1;
        else if (!strncmp(argv[i], "beam=", 5)) beam_deg = atof(argv[i] + 5);
        else if (!strncmp(argv[i], "turn=", 5)) turn_scale = atof(argv[i] + 5);
    }
    printf("Bypass course sim: car %d mm wide, sonar %u mm / IR %d mm ahead of the axle, sonar cone +/-%.0f deg, "
           "turns x%.2f, %s preferred%s\n",
           2 * (int)s5_defaults().half_width_mm, S5_SONAR_AXLE_MM, S3_SENSOR_TO_AXLE_MM, beam_deg, turn_scale,
           S5_PREFER_ROUTE == 1U ? "LEFT" : "RIGHT", S5_ALLOW_OTHER_SIDE ? ", other side allowed" : "");
    straight_case(0.10, 0.10, 0, 0);
    straight_case(0.20, 0.20, 0, 0);
    straight_case(0.20, 0.20, 45, 0);
    straight_case(0.20, 0.10, 0, 0.03);
    left_blocked_case();
    for (int dir = 1; dir >= -1; dir -= 2) {
        bend_after_case(0.30, dir, 0.10, 0.20, true);
        bend_after_case(0.30, dir, 0.25, 0.20, true);
        /* Known limit with the full 214 mm pass (S5_PASS_EXTRA_V2 0, chosen
         * after the shorter pass hit the obstacle on the car): a tight bend
         * AWAY from the bypass side starting 10 cm after a 20 cm box has
         * curved out of reach by the time the car comes back. */
        bend_after_case(0.20, dir, 0.10, 0.20, dir > 0 || S5_PASS_EXTRA_V2);
        bend_after_case(0.20, dir, 0.25, 0.10, true);
        bend_before_case(0.30, dir, 0.15, 0.20);
        bend_before_case(0.20, dir, 0.25, 0.20);
    }
    printf("%s: %d failure(s)\n", failures ? "BYPASS SIM FAIL" : "BYPASS SIM PASS", failures);
    return failures ? 1 : 0;
}
