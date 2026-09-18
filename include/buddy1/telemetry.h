/*
 * telemetry.h - outbound telemetry for the INF2004 robot car.
 * Buddy 1: WiFi communication, command and telemetry.
 *
 * This is the ONLY header your teammates need to include. They call one
 * tele_report_*() function and carry on; they never touch MQTT, WiFi or
 * the queue.
 *
 * DESIGN RULE THAT DRIVES EVERYTHING HERE:
 *   tele_report_*() must NEVER perform network I/O. It formats into bounded
 *   storage and returns. Periodic samples coalesce; important events use a
 *   separate FIFO. If the network is slow or dead, the robot keeps driving.
 *
 * This file contains no Pico SDK calls, so it compiles and runs on a laptop.
 * See host_test.c.
 *
 * Style: Barr-C:2018 (the standard shipped with the project).
 * Language: C99.
 */
#ifndef BUDDY1_TELEMETRY_H
#define BUDDY1_TELEMETRY_H

#include <stdint.h>
#include <stdbool.h>

#include "shared/robot_types.h"

/* -------------------------------------------------------------------------
 * Configuration
 * ------------------------------------------------------------------------- */

/*
 * Root of every topic this car publishes.
 *
 * On demo day EVERY group connects to the same broker. Without a group
 * prefix you cannot tell your robot's messages from anyone else's.
 *
 * !! CHANGE grpXX TO YOUR GROUP NUMBER !!
 * Or override at build time:  -DTELE_TOPIC_ROOT="\"grp07/car\""
 */
#ifndef TELE_TOPIC_ROOT
#define TELE_TOPIC_ROOT "grpXX/car"
#endif

/*
 * Longest payload we will ever build.
 *
 * Sized from the worst case rather than guessed:
 *
 *   envelope  {"t":4294967295,"seq":4294967295,        33
 *   snapshot  "snap":true,                             12   (snapshots only)
 *   body      TELE_BODY_MAX - 1                       143
 *   close     }                                         1
 *                                                     ---
 *                                                     189
 *
 * 224 leaves genuine headroom rather than the three bytes that arithmetic
 * would otherwise leave. telemetry.c asserts this at COMPILE TIME, so adding
 * a field to any message can never silently overflow - the build stops
 * instead.
 *
 * Every snprintf() in telemetry.c is bounded by this, so a payload can be
 * truncated but can never overflow the buffer.
 */
#define TELE_PAYLOAD_MAX  224u

/*
 * How many important event messages we can hold while the network is busy.
 *
 * MUST BE A POWER OF TWO. We advance the queue indices with a bitwise AND
 * instead of the % operator, because the Cortex-M0+ has NO HARDWARE DIVIDE
 * instruction - every % is a call into a software division routine. An AND
 * is one cycle. (Lab 6's optimisation exercise is built on this fact.)
 *
 * Periodic motion/line/IMU samples use three separate latest-value slots.
 */
#define TELE_QUEUE_SLOTS  16u

/* -------------------------------------------------------------------------
 * Shared vocabulary
 * ------------------------------------------------------------------------- */

/*
 * Compatibility names for the existing telemetry API. The actual state enum
 * lives once in robot_types.h and is shared with the mission controller.
 */
typedef mission_state_t tele_state_t;

#define TELE_STATE_BOOT               MISSION_STATE_BOOT
#define TELE_STATE_LINE_FOLLOW        MISSION_STATE_LINE_FOLLOW
#define TELE_STATE_AT_JUNCTION         MISSION_STATE_AT_JUNCTION
#define TELE_STATE_HUMP                MISSION_STATE_HUMP
#define TELE_STATE_OBSTACLE_STOPPING   MISSION_STATE_OBSTACLE_STOPPING
#define TELE_STATE_OBSTACLE_SCAN       MISSION_STATE_OBSTACLE_SCAN
#define TELE_STATE_BYPASS              MISSION_STATE_BYPASS
#define TELE_STATE_LINE_SEARCH         MISSION_STATE_LINE_SEARCH
#define TELE_STATE_STOPPED             MISSION_STATE_STOPPED
#define TELE_STATE_FAULT               MISSION_STATE_FAULT
#define TELE_STATE_COUNT               MISSION_STATE_COUNT

/* Human-readable names, for payloads and for debugging. */
/* All four are declared in robot_types.h and implemented once in
   robot_types.c. tele_state_name keeps its old spelling so existing callers
   and tests are unaffected. */
#define tele_state_name(state)  mission_state_name(state)

/* -------------------------------------------------------------------------
 * One queued message
 * ------------------------------------------------------------------------- */

typedef struct
{
    /*
     * Topic is a POINTER, not a copy. Every topic we publish is a string
     * literal known at compile time (see the TELE_TOPIC_* macros in
     * telemetry.c), so it already lives in flash for the life of the
     * program. Copying it into every slot would waste ~40 bytes x 16 slots
     * of RAM for no benefit.
     */
    const char * topic;
    char         payload[TELE_PAYLOAD_MAX];
} tele_msg_t;

/* -------------------------------------------------------------------------
 * Clock hook
 * ------------------------------------------------------------------------- */

/*
 * Returns milliseconds since power-on.
 *
 * Declared here but NOT implemented in telemetry.c, so the same telemetry
 * code links against two different clocks:
 *   - on the Pico:  to_ms_since_boot(get_absolute_time())
 *   - on a laptop:  a fake counter we control, so tests are repeatable
 *
 * This is the same host/target split Lab 6 uses (optimise_host.c vs
 * optimise_pico.c), and it is what lets us test all of this with no
 * hardware attached.
 */
uint32_t tele_now_ms(void);

/* -------------------------------------------------------------------------
 * Locking hook
 *
 * Declared here, implemented per platform - the same trick as tele_now_ms().
 * The queue bookkeeping below is a read-modify-write on shared indices, and
 * "volatile is not atomicity" (W2.2 Interrupts). Something has to make those
 * few instructions indivisible once more than one context can reach them.
 *
 *   host tests      : no-op, single threaded
 *   bare-metal Pico : no-op, single threaded main loop (see the note in
 *                     telemetry_port_pico.c for why that is currently true)
 *   under uT-Kernel : enter/leave a short non-sleeping critical section
 *
 * Putting the seam here means the RTOS port is ONE SMALL FILE rather than
 * edits scattered through telemetry.c, and the pure logic stays testable on
 * a laptop with no kernel present.
 *
 * THREE RULES for any implementation:
 *   1. It must NOT sleep or wait indefinitely; tele_report_*() has a bounded
 *      deadline and is called from control tasks.
 *   2. It must NOT need to be recursive. telemetry.c never nests these.
 *   3. Critical sections must stay short - index arithmetic and one bounded
 *      snprintf, never I/O. (Lab 5 Part 4: a long critical section voids
 *      every real-time guarantee in the system.)
 * ------------------------------------------------------------------------- */
void tele_lock(void);
void tele_unlock(void);

/* -------------------------------------------------------------------------
 * Lifecycle
 * ------------------------------------------------------------------------- */

/* Empty the queue and reset the counters. Call once at startup. */
void tele_init(void);

/* -------------------------------------------------------------------------
 * The reporting API - THIS IS WHAT YOUR TEAMMATES CALL
 *
 * Every one of these returns:
 *   true  - accepted without losing an important event
 *   false - accepted, but an older important event had to be discarded
 *
 * Motion, line and IMU are latest-value periodic telemetry: if a newer value
 * arrives before the old one is sent, it replaces the stale one. State,
 * barcode, hump, obstacle and log messages use the event FIFO. This prevents
 * a control-rate stream from evicting the one barcode or obstacle report that
 * explains a mission decision.
 * ------------------------------------------------------------------------- */

/* Shared Vehicle Controller mission state. */
bool tele_report_state(tele_state_t state);

/*
 * Buddy 2 - motion.
 *
 * Speeds are reported PER WHEEL, not as one number. The whole reason the PID
 * exists is that two motors given identical PWM turn at different rates; a
 * single averaged speed would hide exactly the quantity being controlled.
 * Encoder counts are raw and cumulative - the write-up p.5 asks for them by
 * name, and they are what lets anyone check the distance calculation.
 */
bool tele_report_motion(int16_t left_mms,
                        int16_t right_mms,
                        int32_t dist_mm,
                        uint32_t left_encoder,
                        uint32_t right_encoder);
/*
 *   left_mms       wheel speed, mm/s. NEGATIVE = that wheel is reversing.
 *   right_mms      reported separately, never averaged - the difference
 *                  between them is the quantity the PID exists to control.
 *   dist_mm        SIGNED net distance travelled since power-on. Reversing
 *                  makes it go down. Signed because the telemetry is meant
 *                  to let someone reconstruct the route, and a total that
 *                  only ever rises cannot express "backed up 200 mm".
 *   left_encoder   raw pulse counts, cumulative since power-on, MONOTONIC.
 *   right_encoder  The lab encoder is a single LED/detector pair, so it
 *                  CANNOT sense direction - the count rises whichever way
 *                  the wheel turns. That is why dist_mm is a separate field
 *                  and not something the receiver can derive from these.
 */

/*
 * Buddy 3 - line following.
 * pos is the estimated offset from the centre of the line; negative is left.
 * The raw sensor readings travel too, so calibration can be checked from the
 * telemetry rather than only from a serial cable.
 */
bool tele_report_line(int8_t pos,
                      bool at_junction,
                      uint16_t left_raw,
                      uint16_t right_raw);
/*
 *   pos        NORMALISED line error, -100 .. +100. NOT millimetres.
 *              0 = centred, negative = line is to the LEFT of the car,
 *              +/-100 = line at the outer limit of what the sensors can see.
 *              Normalised rather than physical because this is a control
 *              error, and its scale depends on how far apart the two sensors
 *              were mounted - which is Buddy 3's choice, not a constant.
 *   left_raw   raw ADC counts, 0 .. 4095 (the Pico ADC is 12-bit).
 *   right_raw  ditto. Sent so IR calibration can be checked from telemetry.
 */

/*
 * Buddy 3 - a decoded barcode and what it means under the CURRENT mapping.
 *
 *   decoded_char  'A'..'Z'. Anything else is reported as '?'.
 *   action        what that letter means under the frozen run configuration.
 *   was_reversed  true if the symbol was read back-to-front and the decoder
 *                 had to flip it. The brief REQUIRES bidirectional decoding,
 *                 and this flag is the evidence that it works - without it
 *                 you cannot prove from the telemetry that the reverse path
 *                 was ever exercised.
 */
bool tele_report_barcode(char decoded_char,
                         nav_action_t action,
                         bool was_reversed);

/*
 * Buddy 4 - terrain.
 *
 *   peak_mm      the height of THIS hump.
 *   max_peak_mm  the tallest hump seen so far this run. Mission requirement 5
 *                says the ROBOT shall report the highest peak - so the robot
 *                tracks it, rather than leaving the dashboard to work it out.
 *   hump_count   how many humps have been crossed so far.
 */
bool tele_report_hump(uint16_t peak_mm,
                      uint16_t max_peak_mm,
                      uint16_t hump_count);

/*
 * Buddy 4 - motion and orientation.
 * pitch is in DECI-degrees (tenths) so tilt keeps one decimal place of
 * resolution without any floating point: -125 means -12.5 degrees.
 */
bool tele_report_imu(uint16_t heading_dg,
                     int16_t pitch_ddg,
                     int16_t turn_rate_dps,
                     imu_event_t event);
/*
 *   heading_dg     compass bearing, 0 .. 359. UNSIGNED, because a negative
 *                  bearing is meaningless - 350 not -10.
 *   pitch_ddg      tilt in DECI-degrees, -900 .. +900 (i.e. -90.0 .. +90.0).
 *                  POSITIVE = nose UP. Tenths give a decimal place with no
 *                  floating point, which this chip has no hardware for.
 *   turn_rate_dps  degrees per second. POSITIVE = turning RIGHT (clockwise
 *                  seen from above). Derived, not measured: the GY-511 has
 *                  no gyroscope, so this comes from successive headings.
 *   event          how the car is currently moving.
 */

/*
 * Buddy 5 - obstacle profile and the avoidance decision.
 * All five fields the write-up p.9 Stage 3 asks for, plus the Stage 4 plan,
 * so the telemetry shows not just what was seen but what was decided.
 */
bool tele_report_obstacle(int16_t angle_dg,
                          uint16_t closest_mm,
                          uint16_t width_mm,
                          uint16_t clear_left_mm,
                          uint16_t clear_right_mm,
                          avoid_plan_t plan);
/*
 *   angle_dg        bearing to the middle of the obstacle, -90 .. +90.
 *                   0 = dead ahead, NEGATIVE = to the LEFT.
 *   closest_mm      distance to its nearest point, 20 .. 4000 (the HC-SR04's
 *                   usable range). ZERO MEANS "NO VALID ECHO" - the sensor
 *                   cannot report 0 legitimately, because its minimum range
 *                   is 20 mm, so 0 is an unambiguous sentinel.
 *   width_mm        estimated width.
 *   clear_left_mm   free space on each side, capped at 4000 (sensor maximum).
 *   clear_right_mm  0 means genuinely blocked, which is why the "no reading"
 *                   sentinel above is 0 for distance but NOT reused here.
 *   plan            what the car decided to do about it.
 */

/* Free-text event, for anything that does not fit the structured topics.
   NOT cached for snapshots: a log line is an event, not a state. */
bool tele_report_log(const char * text);

/* -------------------------------------------------------------------------
 * Snapshot
 * ------------------------------------------------------------------------- */

/*
 * Answer "tell me everything right now" - the requirement from the briefing
 * that the car must respond to a request with the current picture, not just
 * stream events.
 *
 * Re-queues the most recent value of every topic that has reported at least
 * once, each on its own topic, each carrying "snap":true so a subscriber can
 * tell replayed state from a live event.
 *
 * Returns how many messages were queued. Zero means nothing has ever been
 * reported, which is itself worth knowing.
 */
uint32_t tele_publish_snapshot(void);

/* -------------------------------------------------------------------------
 * The draining API - THIS IS WHAT THE NETWORK TASK CALLS
 * ------------------------------------------------------------------------- */

/*
 * Take the oldest message out of the queue.
 * Returns false and leaves *out untouched if the queue is empty.
 */
bool tele_queue_pop(tele_msg_t * out);

/* How many messages are waiting. */
uint32_t tele_queue_depth(void);

/*
 * How many messages have been discarded since startup because the queue was
 * full. Publish this in your status message: it turns "telemetry sometimes
 * goes missing" from a suspicion into a measurement.
 */
uint32_t tele_dropped_count(void);

/* Number of stale periodic samples deliberately replaced by fresher ones. */
uint32_t tele_coalesced_count(void);

/* The sequence number the next published message will carry. */
uint32_t tele_next_seq(void);

#endif /* BUDDY1_TELEMETRY_H */
