/** @file imu_host_main.c
 * @brief Exercise IMU, terrain, hump, pickup and overturn processing.
 * The selected narrow integer operands fit in int.
 * Unsigned casts spell out the original modulo conversion.
 * Packet fields and fixture masks are interpreted as unsigned bits.
 */
#include "imu_host_main.h"
#include "../../app_program/app_types.h"
#include "imu_sensor.h"
#include "terr_data.h"
#include "tilt.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#ifdef TEST_SEMANTIC_TRACE
#include "terrain_trace.h"
#endif

#define LEVEL_WARMUP_SAMPLE_COUNT 110U
#define PITCH_ONLY_SAMPLE_COUNT 20U
#define GEOMETRY_CASE_COUNT 3U
#define GEOMETRY_ARM_SAMPLE_COUNT 5U
#define GEOMETRY_CLIMB_SAMPLE_COUNT 10U
#define REPEATED_HUMP_COUNT 2U
#define NEGATIVE_PICKUP_SAMPLE_COUNT 4U
#define LEVEL_CALIBRATION_SAMPLE_COUNT 100U
#define LONGITUDINAL_ASCENT_END 10U
#define LONGITUDINAL_DESCENT_END 20U
#define LONGITUDINAL_FLAT_END 25U

#define LONG_ROLL_SAMPLE_COUNT 40U
#define LEVEL_REARM_SAMPLE_COUNT 25U
#define SHORT_ROLL_SAMPLE_COUNT 20U
#define PITCH_PROFILE_SEGMENT_COUNT 6U
#define PITCH_SEGMENT_SAMPLE_COUNT 20U
#define DEBUG_CLIMB_SAMPLE_COUNT 20U
#define LATERAL_ACCELERATION_SAMPLE_COUNT 10U
#define LIFT_CONFIRMATION_PREFIX_COUNT 2U
#define RECOVERY_BEFORE_INVALID_SAMPLE_COUNT 5U
#define RECOVERY_AFTER_INVALID_SAMPLE_COUNT 15U
#define ANGLED_LIFT_CONFIRMATION_PREFIX_COUNT 2U
#define OVERTURN_HOLD_SAMPLE_COUNT 20U
#define OVERTURN_RECOVERY_SAMPLE_COUNT 25U

#define MAX_TILT_TENTHS_DEG 900
#define MAX_HUMP_HEIGHT_MM 500

typedef struct
{
    terr_data_monitor_t  monitor;
    imu_sensor_sample_t  sample;
    terr_data_odometry_t odometry;
    unsigned             completed;
    unsigned             max_live_mm;
} terrain_fixture_t;

static uint8_t g_acc[256];
static uint8_t g_mag[256];
static bool    gb_should_fail;

static void sample_and_observe (terr_data_monitor_t *        p_monitor,
                                const imu_sensor_sample_t *  p_sample,
                                const terr_data_odometry_t * p_odometry);

static bool rd (void * p_config, uint8_t addr, uint8_t reg, uint8_t * p_out,
                unsigned byte_count);

static bool wr (void * p_config, uint8_t addr, uint8_t reg, uint8_t wire_value);

static void le (unsigned reg, int value);

static uint32_t bits (const uint8_t * p_packet, unsigned start, unsigned count);

static void right_wheel_tests (void);

static void short_crossing_tests (void);

#if APP_IMU_DEBUG
static void hump_debug_tests (void);
#endif

static void level_pickup_rules_tests (void);

static void pickup_and_overturn_tests (void);

/** @brief Use a simulated IMU bus and wheel data to check calibration,
 * attitude, hump geometry, packet representation and fault recovery.
 */
int main (void)
{
    imu_sensor_bus_t     bus    = {0, rd, wr};
    imu_sensor_sample_t  sample = {0};
    terr_data_monitor_t  monitor;
    terr_data_odometry_t odometry = {0};
    float32_t            pitch;
    float32_t            roll;
    uint8_t              packed[TERRAIN_PACKET_SIZE];
    unsigned             index;
    g_mag[10] = 'H';
    g_mag[11] = '4';
    g_mag[12] = '3';
    assert(imu_sensor_init(&bus));
    assert(0x47 == (int)g_acc[0x20] && 0x98 == (int)g_acc[0x23]);
    g_mag[10] = 0;
    assert(!imu_sensor_init(&bus));
    g_mag[10]   = 'H';
    g_acc[0x27] = 8;
    le(0x28, -100);
    le(0x2a, 200);
    le(0x2c, 500);
    assert(imu_sensor_read(&bus, 123, &sample));
    assert(fabsf(sample.accel_g[0] + 0.4f) < 0.001f);
    assert(fabsf(sample.accel_g[1] + 0.2f) < 0.001f &&
           fabsf(sample.accel_g[2] - 1) < 0.001f);
    gb_should_fail = true;
    assert(!imu_sensor_read(&bus, 124, &sample) && !sample.b_is_valid);
    gb_should_fail = false;
    le(0x28, -2048);
    assert(!imu_sensor_read(&bus, 125, &sample));
    le(0x28, 0);
    g_mag[9] = 1;
    g_mag[3] = 0;
    g_mag[4] = 110;
    g_mag[5] = 0;
    g_mag[6] = 98;
    g_mag[7] = 0xff;
    g_mag[8] = 0x92;
    assert(imu_sensor_read(&bus, 126, &sample) && sample.b_is_mag_valid);
    assert(fabsf(sample.mag_gauss[0] - 0.1f) < 0.001f &&
           fabsf(sample.mag_gauss[1] - 0.1f) < 0.001f &&
           fabsf(sample.mag_gauss[2] - 0.1f) < 0.001f);
    g_mag[3] = 0xf0;
    g_mag[4] = 0;
    assert(imu_sensor_read(&bus, 127, &sample) && !sample.b_is_mag_valid);
    {
        float32_t acceleration[3] = {0.5f, 0, 0.8660254f};
        assert(tilt_estimation(acceleration, &pitch, &roll));
        assert(fabsf(pitch - 30) < 0.01f);
    }
    /* Right side up = positive roll: the vehicle-left axis then points down. */
    {
        float32_t acceleration[3] = {0, -0.5f, 0.8660254f};
        assert(tilt_estimation(acceleration, &pitch, &roll));
        assert(fabsf(roll - 30) < 0.01f && fabsf(pitch) < 0.01f);
    }
    assert(fabsf(tilt_angle_delta(1, 359) - 2) < 0.001f);
    terr_data_init(&monitor);
    odometry.b_is_stationary = true;
    sample                   = (imu_sensor_sample_t){0};
    sample.b_is_valid        = true;
    sample.accel_g[2]        = 1;

    for (index = 1; index <= LEVEL_CALIBRATION_SAMPLE_COUNT; index++)
    {
        sample.timestamp_us = index * 20000U;
        sample_and_observe(&monitor, &sample, &odometry);
    }

    assert(monitor.calibration.b_is_calibrated);
    sample.timestamp_us += 20000U;
    sample_and_observe(&monitor, &sample, &odometry);
    assert(monitor.result.flags & TERRAIN_TILT_VALID);
    assert(!(monitor.result.flags & TERRAIN_TURN_VALID));
    sample.timestamp_us += 200000U;
    sample_and_observe(&monitor, &sample, &odometry);
    assert(monitor.result.flags & TERRAIN_FAULT);
    {
        hump_detection_t hump_state = {0};

        for (index = 0; index < LONGITUDINAL_ASCENT_END; index++)
        {
            hump_detection_update(&hump_state, 30, 0.01f, true, index * 20000U);
        }

        for (index = LONGITUDINAL_ASCENT_END; index < LONGITUDINAL_DESCENT_END;
             index++)
        {
            hump_detection_update(&hump_state, -30, 0.01f, true,
                                  index * 20000U);
        }

        for (index = LONGITUDINAL_DESCENT_END; index < LONGITUDINAL_FLAT_END;
             index++)
        {
            hump_detection_update(&hump_state, 0, 0.01f, true, index * 20000U);
        }

        assert(hump_state.b_is_peak_valid &&
               fabsf(hump_state.highest_m - 0.05f) < 0.00001f &&
               hump_state.b_has_completed_hump);
        hump_detection_reset(&hump_state);
        hump_detection_update(&hump_state, 20, 0.01f, true, 0);
        hump_detection_update(&hump_state, 20, 0.01f, false, 20000);
        assert(!hump_state.b_is_hump_tracking && !hump_state.b_is_peak_valid);
    }
    monitor.result = (terr_data_result_t){.timestamp_us = 123,
                                          .pitch_deg    = -12.3f,
                                          .roll_deg     = 45.6f,
                                          .turn_dps     = -30,
                                          .peak_m       = 0.123f,
                                          .flags        = 15,
                                          .state        = TERRAIN_CLIMBING};
    terr_data_pack(&monitor.result, packed);
    assert(bits(packed, 0, 32) == 123U &&
           bits(packed, 32, 3) == (unsigned)TERRAIN_CLIMBING);
    assert(bits(packed, 40, 11) == (2048U - 123U) &&
           bits(packed, 51, 11) == 456U &&
           bits(packed, 62, 10) == 1024U - 30U && bits(packed, 72, 9) == 123U);
    monitor.result.peak_m = 0.501f;
    terr_data_pack(&monitor.result, packed);
    assert(!(bits(packed, 35, 5) & TERRAIN_PEAK_VALID));
    monitor.result.pitch_deg = NAN;
    terr_data_pack(&monitor.result, packed);
    assert(!(bits(packed, 35, 5) & TERRAIN_TILT_VALID));
    monitor.hump.b_is_peak_valid = true;
    monitor.hump.highest_m       = 0.1f;
    terr_data_reset_run(&monitor);
    assert(!monitor.hump.b_is_peak_valid &&
           monitor.calibration.b_is_calibrated);
    {
        terr_data_result_t decoded;

        for (int angle = -MAX_TILT_TENTHS_DEG; angle <= MAX_TILT_TENTHS_DEG;
             angle++)
        {
            monitor.result = (terr_data_result_t){
                .pitch_deg = angle / 10.0f,
                .roll_deg  = -angle / 10.0f,
                .flags     = TERRAIN_TILT_VALID | TERRAIN_TURN_VALID |
                         TERRAIN_PEAK_VALID,
                .turn_dps           = (float32_t)(angle % 361),
                .peak_m             = 0.5f,
                .b_is_hump_complete = true,
                .b_has_impact       = true};
            terr_data_pack(&monitor.result, packed);
            terr_data_unpack(packed, &decoded);
            assert(fabsf(decoded.pitch_deg - monitor.result.pitch_deg) <
                   0.001f);
            assert(fabsf(decoded.roll_deg - monitor.result.roll_deg) < 0.001f);
            assert(decoded.turn_dps == monitor.result.turn_dps &&
                   0.5f == decoded.peak_m);
            assert(decoded.b_is_hump_complete && decoded.b_has_impact &&
                   !decoded.b_is_hump_active && (packed[11] & 0xf8U) == 0U);
        }
        /* Live climb height: 1 + 9 bits after the impact bit, 0..500 mm, zero
         * when no hump. */
        for (int height_mm = 0; height_mm <= MAX_HUMP_HEIGHT_MM; height_mm++)
        {
            monitor.result = (terr_data_result_t){
                .b_is_hump_active = true, .hump_height_m = height_mm / 1000.0f};
            terr_data_pack(&monitor.result, packed);
            terr_data_unpack(packed, &decoded);
            assert(bits(packed, 83, 1) == 1U &&
                   bits(packed, 84, 9) == (unsigned)height_mm &&
                   (packed[11] & 0xe0U) == 0U);
            assert(decoded.b_is_hump_active &&
                   fabsf(decoded.hump_height_m - height_mm / 1000.0f) <
                       0.0001f);
        }

        monitor.result = (terr_data_result_t){.b_is_hump_active = true,
                                              .hump_height_m    = 0.501f};
        terr_data_pack(&monitor.result, packed);
        assert(bits(packed, 83, 10) == 0U);
        monitor.result = (terr_data_result_t){.b_is_hump_active = false,
                                              .hump_height_m    = 0.2f};
        terr_data_pack(&monitor.result, packed);
        assert(bits(packed, 83, 10) == 0U);
    }
    /* The monitor reports the live climb while crossing a hump. */
    {
        hump_detection_t hump_state = {0};

        for (index = 0; index < LONGITUDINAL_ASCENT_END; index++)
        {
            hump_detection_update(&hump_state, 30, 0.01f, true, index * 20000U);
        }

        assert(hump_state.b_is_hump_tracking && hump_state.height_m > 0.04f);
    }
    right_wheel_tests();
    short_crossing_tests();
#if APP_IMU_DEBUG
    hump_debug_tests();
#endif
    level_pickup_rules_tests();
    pickup_and_overturn_tests();
    puts("Buddy 4 host tests PASS: driver, calibration, tilt, hump geometry,"
         " packing, pickup, overturn and recovery");
    return 0;
}

/** @brief Process one IMU/odometry sample and optionally record its result. */
static void sample_and_observe (terr_data_monitor_t *        p_monitor,
                                const imu_sensor_sample_t *  p_sample,
                                const terr_data_odometry_t * p_odometry)
{
    terr_data_process(p_monitor, p_sample, p_odometry);
#ifdef TEST_SEMANTIC_TRACE
    terrain_trace_result(&p_monitor->result);
#endif
}

/** @brief Read the requested byte count from simulated accelerometer or
 * magnetometer registers. Injected failure leaves the destination
 * untouched.
 */
static bool rd (void * p_config, uint8_t addr, uint8_t reg, uint8_t * p_out,
                unsigned byte_count)
{
    (void)p_config;

    if (gb_should_fail)
    {
        return false;
    }

    memcpy(p_out,
           (0x19 == (int)addr ? g_acc : g_mag) +
               (0x19 == (int)addr ? ((unsigned)reg & 0x7fU) : (unsigned)reg),
           byte_count);
    return true;
}

/** @brief Write one register in the selected simulated sensor. An injected
 * failure returns false before modifying the register.
 */
static bool wr (void * p_config, uint8_t addr, uint8_t reg, uint8_t wire_value)
{
    (void)p_config;

    if (gb_should_fail)
    {
        return false;
    }

    (0x19 == (int)addr ? g_acc : g_mag)[reg] = wire_value;
    return true;
}

/** @brief Encode a signed acceleration test word into adjacent little-endian
 * simulated registers with four unused low bits.
 */
static void le (unsigned reg, int value)
{
    unsigned wire_value = (unsigned)(value * 16) & 65535U;
    g_acc[reg]          = (uint8_t)wire_value;
    g_acc[reg + 1U]     = (uint8_t)(wire_value >> 8);
}

/** @brief Extract count bits starting at the supplied packet bit offset into
 * an unsigned value, least significant bit first.
 */
static uint32_t bits (const uint8_t * p_packet, unsigned start, unsigned count)
{
    uint32_t wire_value = 0;
    unsigned index;

    for (index = 0; index < count; index++)
    {
        wire_value |= (((unsigned)p_packet[(start + index) / 8U] >>
                        ((start + index) % 8U)) &
                       1U)
                      << index;
    }

    return wire_value;
}

/** @brief Return the roll angle in degrees for a one-wheel lift at the given
 * pitch and wheel spacing.
 */
static float32_t roll_for_height (float32_t height, float32_t pitch,
                                  float32_t track)
{
    return asinf(height / (track * cosf(pitch * 0.01745329252f))) *
           57.295779513f;
}

/** @brief Advance time by 20 ms per sample and feed a right-wheel lift at
 * the given height and distance increment.
 */
static void right_samples (hump_detection_t * p_hump, uint32_t * p_now,
                           float32_t height, float32_t wheel_step_m,
                           unsigned count)
{
    for (unsigned sample_index = 0; sample_index < count; sample_index++)
    {
        *p_now += 20000U;
        hump_detection_update_right(p_hump, 0,
                                    roll_for_height(height, 0, 0.116f), 0.116f,
                                    wheel_step_m, HUMP_SAMPLE_VALID, *p_now);
    }
}

/** @brief Feed one acceleration vector, distance increment and time gap,
 * retain valid live-height evidence and return the decoded safety
 * state.
 */
static terr_data_safety_t terrain_accel (terrain_fixture_t * p_frame,
                                         float32_t accel_x, float32_t accel_y,
                                         float32_t accel_z,
                                         float32_t wheel_step_m, uint32_t gap)
{
    uint8_t            packet[TERRAIN_PACKET_SIZE];
    terr_data_result_t decoded;
    p_frame->sample.timestamp_us     += gap;
    p_frame->sample.accel_g[0]        = accel_x;
    p_frame->sample.accel_g[1]        = accel_y;
    p_frame->sample.accel_g[2]        = accel_z;
    p_frame->odometry.distance_m     += wheel_step_m;
    p_frame->odometry.b_is_stationary = 0 == wheel_step_m;
    sample_and_observe(&p_frame->monitor, &p_frame->sample, &p_frame->odometry);
    terr_data_pack(&p_frame->monitor.result, packet);
    terr_data_unpack(packet, &decoded);
    assert(decoded.safety == p_frame->monitor.result.safety);

    if (decoded.b_is_hump_complete)
    {
        ++p_frame->completed;
    }

    if (decoded.b_is_hump_active)
    {
        unsigned millimetres = (unsigned)lroundf(decoded.hump_height_m * 1000);

        if (millimetres > p_frame->max_live_mm)
        {
            p_frame->max_live_mm = millimetres;
        }
    }

    return decoded.safety;
}

/** @brief Convert pitch and roll in degrees to a gravity vector and pass it
 * to the simulated terrain update.
 */
static terr_data_safety_t terrain_step (terrain_fixture_t * p_frame,
                                        float32_t pitch, float32_t roll,
                                        float32_t wheel_step_m, uint32_t gap)
{
    const float32_t pitch_rad = pitch * 0.01745329252f;
    const float32_t roll_rad  = roll * 0.01745329252f;
    return terrain_accel(p_frame, sinf(pitch_rad),
                         -sinf(roll_rad) * cosf(pitch_rad),
                         cosf(roll_rad) * cosf(pitch_rad), wheel_step_m, gap);
}

/** @brief Reset the terrain monitor, select a hump geometry and feed level
 * calibration samples until tilt is valid.
 */
static void terrain_fixture_init (terrain_fixture_t *       p_frame,
                                  terr_data_hump_geometry_t geometry)
{
    *p_frame = (terrain_fixture_t){0};
    terr_data_init(&p_frame->monitor);
    assert((unsigned)TERRAIN_HUMP_LONGITUDINAL ==
           p_frame->monitor.hump_geometry);
    assert(terr_data_set_geometry(&p_frame->monitor, geometry, 0.116f));
    p_frame->sample.b_is_valid   = true;
    p_frame->odometry.b_is_valid = true;
    p_frame->odometry.epoch      = 1;

    for (unsigned sample_index = 0; sample_index < LEVEL_WARMUP_SAMPLE_COUNT;
         sample_index++)
    {
        terrain_step(p_frame, 0, 0, 0, 20000);
    }

    assert(p_frame->monitor.calibration.b_is_calibrated &&
           (p_frame->monitor.result.flags & TERRAIN_TILT_VALID));
}

/** @brief Cross two short 10 mm right-wheel humps at 100 mm/s and 50 Hz.
 * A commanded pivot between them mimics a rise even with encoder drift.
 */
static void short_crossing_tests (void)
{
    const float32_t heights_m[][7] = {
        {0.004f, 0.008f, 0.010f, 0.010f, 0.008f, 0.004f, 0.0f},
        {0.000f, 0.007f, 0.010f, 0.010f, 0.000f, 0.000f, 0.0f}};
    const unsigned    passes  = sizeof heights_m / sizeof heights_m[0];
    const unsigned    samples = sizeof heights_m[0] / sizeof heights_m[0][0];
    const unsigned    settled_samples = 6U;
    const unsigned    rearm_samples   = 5U;
    const uint32_t    sample_us       = 20000U;
    const float32_t   travel_m        = 0.002f;
    const float32_t   track_m         = 0.116f;
    const float32_t   crest           = roll_for_height(0.010f, 0.0f, track_m);
    terrain_fixture_t fixture;
    terrain_fixture_init(&fixture, TERRAIN_HUMP_RIGHT_WHEEL);

    /* One 20 ms excursion must not even start a crossing. */
    terrain_step(&fixture, 0.0f, crest, travel_m, sample_us);
    assert(!fixture.monitor.result.b_is_hump_active);
    terrain_step(&fixture, 0.0f, 0.0f, travel_m, sample_us);
    terrain_step(&fixture, 0.0f, 0.0f, travel_m, sample_us);

    for (unsigned pass = 0U; pass < passes; ++pass)
    {
        fixture.max_live_mm = 0U;
        for (unsigned sample_index = 0U; sample_index < samples; ++sample_index)
        {
            const float32_t roll =
                roll_for_height(heights_m[pass][sample_index], 0.0f, track_m);
            terrain_step(&fixture, 0.0f, roll, travel_m, sample_us);
        }

        for (unsigned sample_index = 0U; sample_index < settled_samples;
             ++sample_index)
        {
            terrain_step(&fixture, 0.0f, 0.0f, travel_m, sample_us);
        }

        assert(fixture.completed == (pass + 1U));
        assert(10U == fixture.max_live_mm);
        if (0U == pass)
        {
            fixture.odometry.b_is_pivoting = true;
            for (unsigned sample_index = 0U; sample_index < settled_samples;
                 ++sample_index)
            {
                terrain_step(&fixture, 0.0f, crest, travel_m, sample_us);
            }

            assert((!fixture.monitor.hump.b_is_hump_tracking) &&
                   (!fixture.monitor.hump.b_is_armed));
            fixture.odometry.b_is_pivoting = false;
            for (unsigned sample_index = 0U; sample_index < rearm_samples;
                 ++sample_index)
            {
                terrain_step(&fixture, 0.0f, 0.0f, travel_m, sample_us);
            }

            assert(fixture.monitor.hump.b_is_armed &&
                   (1U == fixture.completed));
        }
    }
}

/** @brief Check right-wheel hump arming, completed heights and rejection
 * cases against controlled travel and roll.
 * @details Start from a calibrated level reference. Exercise a right-wheel
 * rise, plateau and return to level with forward odometry. Include reverse
 * travel, stale samples and stationary tilts. Verify that brief disturbances
 * can leave a valid crossing active, and that rejected crossings do not
 * overwrite accepted peak history.
 */
static void right_wheel_tests (void)
{
    hump_detection_t detector = {0};
    uint32_t         now      = 0;
    /* No count without an observed level baseline, or from stationary tilting.
     */
    right_samples(&detector, &now, 0.01f, 0.001f, 20);
    assert(!detector.b_is_hump_tracking && !detector.b_is_peak_valid);
    right_samples(&detector, &now, 0, 0, 5);
    right_samples(&detector, &now, 0.01f, 0, 20);
    assert(!detector.b_is_hump_tracking && !detector.b_is_peak_valid);
    right_samples(&detector, &now, 0, 0, 5);
    right_samples(&detector, &now, 0.004f, 0.001f, 20);
    assert(!detector.b_is_hump_tracking && !detector.b_is_peak_valid);
    right_samples(&detector, &now, 0, 0, 5);
    /* One 10 mm right-wheel hump. A held plateau is active, not completed. */
    right_samples(&detector, &now, 0.01f, 0.0008f, 100);
    assert(detector.b_is_hump_tracking && !detector.b_is_peak_valid &&
           !detector.b_has_completed_hump &&
           fabsf(detector.height_m - 0.01f) < 0.000001f);
    right_samples(&detector, &now, 0, 0.0008f, 4);
    assert(detector.b_is_hump_tracking && !detector.b_has_completed_hump);
    right_samples(&detector, &now, 0, 0.0008f, 1);
    assert(!detector.b_is_hump_tracking && detector.b_has_completed_hump &&
           detector.b_is_peak_valid &&
           fabsf(detector.highest_m - 0.01f) < 0.000001f);
    right_samples(&detector, &now, 0, 0, 20);
    assert(!detector.b_has_completed_hump);
    right_samples(&detector, &now, 0.015f, 0.001f, 10);
    right_samples(&detector, &now, 0, 0.001f, 5);
    assert(detector.b_has_completed_hump &&
           fabsf(detector.highest_m - 0.015f) < 0.000001f);
    /* Invalid/reverse/timeout cancels the candidate, preserving prior peaks. */
    right_samples(&detector, &now, 0.02f, 0.001f, 10);
    hump_detection_update_right(&detector, 0, 5, 0.116f, -0.001f,
                                HUMP_SAMPLE_VALID, now += 20000U);
    assert(!detector.b_is_hump_tracking && !detector.b_is_armed &&
           fabsf(detector.highest_m - 0.015f) < 0.000001f);
    right_samples(&detector, &now, 0, 0.001f, 5);
    right_samples(&detector, &now, 0.02f, 0.001f, 10);
    hump_detection_update_right(&detector, 0, 5, 0.116f, 0.001f,
                                HUMP_SAMPLE_INVALID, now += 20000U);
    assert(!detector.b_is_hump_tracking && !detector.b_is_armed &&
           detector.b_is_peak_valid);
    right_samples(&detector, &now, 0, 0.001f, 5);
    right_samples(&detector, &now, 0.01f, 0.001f, 10);
    now = detector.started_us + 10000001U;
    hump_detection_update_right(&detector, 0, 5, 0.116f, 0.001f,
                                HUMP_SAMPLE_VALID, now);
    assert(!detector.b_is_hump_tracking && !detector.b_is_armed);
    right_samples(&detector, &now, 0.01f, 0.001f, 20);
    assert(
        !detector.b_is_hump_tracking); /* Do not restart on the same plateau. */
    /* Left-only elevation, pitch-only humps, brief roll spikes and steep roll
     * do not count. */
    hump_detection_reset(&detector);
    right_samples(&detector, &now, 0, 0.001f, 5);
    right_samples(&detector, &now, -0.01f, 0.001f, 20);
    right_samples(&detector, &now, 0, 0.001f, 5);
    assert(!detector.b_is_peak_valid);

    for (unsigned sample_index = 0; sample_index < PITCH_ONLY_SAMPLE_COUNT;
         sample_index++)
    {
        hump_detection_update_right(&detector, 15, 0, 0.116f, 0.001f,
                                    HUMP_SAMPLE_VALID, now += 20000U);
    }

    assert(!detector.b_is_hump_tracking && !detector.b_is_peak_valid);
    right_samples(&detector, &now, 0.01f, 0.0008f, 1);
    right_samples(&detector, &now, 0, 0.001f, 5);
    assert(!detector.b_is_peak_valid && !detector.b_has_completed_hump);
    hump_detection_update_right(&detector, 0, 10, 0.116f, 0.001f,
                                HUMP_SAMPLE_VALID, now += 20000U);
    assert(!detector.b_is_hump_tracking && !detector.b_is_armed);
    /* Width and pitch affect the calculation; the result is not fixed at 1 cm.
     */
    for (unsigned variant = 0; variant < GEOMETRY_CASE_COUNT; variant++)
    {
        const float32_t track = 0U == variant ? 0.116f : 0.2f;
        const float32_t pitch = 2U == variant ? 20 : 0;
        const float32_t roll  = roll_for_height(0.012f, pitch, track);
        hump_detection_reset(&detector);

        for (unsigned sample_index = 0;
             sample_index < GEOMETRY_ARM_SAMPLE_COUNT; sample_index++)
        {
            hump_detection_update_right(&detector, 0, 0, track, 0.001f,
                                        HUMP_SAMPLE_VALID, now += 20000U);
        }

        for (unsigned sample_index = 0;
             sample_index < GEOMETRY_CLIMB_SAMPLE_COUNT; sample_index++)
        {
            hump_detection_update_right(&detector, pitch, roll, track, 0.001f,
                                        HUMP_SAMPLE_VALID, now += 20000U);
        }

        assert(detector.b_is_hump_tracking &&
               fabsf(detector.height_m - 0.012f) < 0.000001f);
    }

    {
        terrain_fixture_t terrain_fixture;
        const float32_t   roll = roll_for_height(0.01f, 0, 0.116f);
        terrain_fixture_init(&terrain_fixture, TERRAIN_HUMP_RIGHT_WHEEL);

        for (unsigned pass = 0; pass < REPEATED_HUMP_COUNT; pass++)
        {
            for (unsigned sample_index = 0;
                 sample_index < LONG_ROLL_SAMPLE_COUNT; sample_index++)
            {
                assert(terrain_step(&terrain_fixture, 0, roll, 0.0008f,
                                    20000) == (unsigned)TERRAIN_SAFETY_NONE);
            }

            assert(terrain_fixture.monitor.result.b_is_hump_active &&
                   terrain_fixture.completed == pass);
            /* A brief forward jolt must preserve the crossing and its trusted
             * peak. */
            float32_t peak   = terrain_fixture.monitor.hump.peak_m;
            float32_t travel = terrain_fixture.monitor.hump.travel_m;
            terrain_accel(&terrain_fixture, -0.13f, -0.01f / 0.116f, 0.9963f,
                          0.0008f, 40000);
            assert(terrain_fixture.monitor.hump.b_is_hump_tracking &&
                   terrain_fixture.monitor.hump.b_did_skip_acceleration);
            assert(terrain_fixture.monitor.hump.peak_m == peak &&
                   terrain_fixture.monitor.hump.travel_m == travel);

            for (unsigned sample_index = 0;
                 sample_index < LEVEL_REARM_SAMPLE_COUNT; sample_index++)
            {
                assert(terrain_step(&terrain_fixture, 0, 0, 0.0008f, 20000) ==
                       (unsigned)TERRAIN_SAFETY_NONE);
            }

            assert(terrain_fixture.completed == pass + 1U &&
                   (terrain_fixture.monitor.result.flags & TERRAIN_PEAK_VALID));
        }

        assert(10U == terrain_fixture.max_live_mm &&
               fabsf(terrain_fixture.monitor.result.peak_m - 0.01f) < 0.0001f);
        /* Raw invalidity and real sample gaps must disarm immediately. */
        for (unsigned sample_index = 0; sample_index < SHORT_ROLL_SAMPLE_COUNT;
             sample_index++)
        {
            terrain_step(&terrain_fixture, 0, roll, 0.0008f, 20000);
        }

        terrain_fixture.sample.b_is_valid = false;
        terrain_step(&terrain_fixture, 0, roll, 0.0008f, 20000);
        terrain_fixture.sample.b_is_valid = true;
        assert(!terrain_fixture.monitor.hump.b_is_hump_tracking &&
               !terrain_fixture.monitor.hump.b_is_armed &&
               terrain_fixture.monitor.hump.b_is_peak_valid);

        for (unsigned sample_index = 0; sample_index < SHORT_ROLL_SAMPLE_COUNT;
             sample_index++)
        {
            terrain_step(&terrain_fixture, 0, roll, 0.0008f, 20000);
        }

        assert(!terrain_fixture.monitor.hump.b_is_hump_tracking &&
               2U == terrain_fixture.completed);

        for (unsigned sample_index = 0; sample_index < LEVEL_REARM_SAMPLE_COUNT;
             sample_index++)
        {
            terrain_step(&terrain_fixture, 0, 0, 0.0008f, 20000);
        }

        for (unsigned sample_index = 0; sample_index < SHORT_ROLL_SAMPLE_COUNT;
             sample_index++)
        {
            terrain_step(&terrain_fixture, 0, roll, 0.0008f, 20000);
        }

        terrain_step(&terrain_fixture, 0, roll, 0.0008f, 120000);
        assert(!terrain_fixture.monitor.hump.b_is_hump_tracking &&
               !terrain_fixture.monitor.hump.b_is_armed &&
               2U == terrain_fixture.completed);
        terr_data_reset_run(&terrain_fixture.monitor);
        assert((unsigned)TERRAIN_HUMP_RIGHT_WHEEL ==
                   terrain_fixture.monitor.hump_geometry &&
               0.116f == terrain_fixture.monitor.wheel_track_m);
        assert(terrain_fixture.monitor.calibration.b_is_calibrated &&
               !terrain_fixture.monitor.hump.b_is_peak_valid);
        assert(!terr_data_set_geometry(&terrain_fixture.monitor,
                                       TERRAIN_HUMP_RIGHT_WHEEL, 0));
        assert(!terr_data_set_geometry(&terrain_fixture.monitor,
                                       (terr_data_hump_geometry_t)99, 0.116f));
    }
    {
        terrain_fixture_t terrain_fixture;
        const float32_t   roll = roll_for_height(0.01f, 0, 0.116f);
        terrain_fixture_init(&terrain_fixture, TERRAIN_HUMP_RIGHT_WHEEL);

        for (unsigned sample_index = 0; sample_index < LONG_ROLL_SAMPLE_COUNT;
             sample_index++)
        {
            terrain_step(&terrain_fixture, 0, roll, 0, 20000);
        }

        for (unsigned sample_index = 0; sample_index < LEVEL_REARM_SAMPLE_COUNT;
             sample_index++)
        {
            terrain_step(&terrain_fixture, 0, 0, 0, 20000);
        }

        assert(0U == terrain_fixture.completed &&
               !terrain_fixture.monitor.hump
                    .b_is_peak_valid); /* Tilt by hand without wheel travel. */
        for (unsigned sample_index = 0; sample_index < LONG_ROLL_SAMPLE_COUNT;
             sample_index++)
        {
            terrain_step(&terrain_fixture, 0, roll, 0.0008f, 20000);
        }

        assert(terrain_fixture.monitor.hump.b_is_hump_tracking);
        terrain_fixture.odometry.b_is_valid = false;
        terrain_step(&terrain_fixture, 0, roll, 0.0008f, 20000);
        terrain_fixture.odometry.b_is_valid = true;
        assert(!terrain_fixture.monitor.hump.b_is_hump_tracking &&
               !terrain_fixture.monitor.hump.b_is_armed);

        for (unsigned sample_index = 0; sample_index < LEVEL_REARM_SAMPLE_COUNT;
             sample_index++)
        {
            terrain_step(&terrain_fixture, 0, 0, 0.0008f, 20000);
        }

        for (unsigned sample_index = 0; sample_index < LONG_ROLL_SAMPLE_COUNT;
             sample_index++)
        {
            terrain_step(&terrain_fixture, 0, roll, 0.0008f, 20000);
        }

        assert(terrain_fixture.monitor.hump.b_is_hump_tracking);
        /* Two consecutive lateral disturbances cancel a crossing even with
         * plausible filtered roll. The first cannot inflate the peak. */
        float32_t peak = terrain_fixture.monitor.hump.peak_m;
        terrain_step(&terrain_fixture, 0, 14, 0.0008f, 20000);
        assert(terrain_fixture.monitor.hump.b_is_hump_tracking &&
               terrain_fixture.monitor.hump.peak_m == peak);
        terrain_step(&terrain_fixture, 0, 14, 0.0008f, 20000);
        assert((terrain_fixture.monitor.result.flags & TERRAIN_TILT_VALID) &&
               fabsf(terrain_fixture.monitor.result.roll_deg) < 10);
        assert(!terrain_fixture.monitor.hump.b_is_hump_tracking &&
               !terrain_fixture.monitor.hump.b_is_armed &&
               0U == terrain_fixture.completed);
    }
    {
        terrain_fixture_t legacy;
        terrain_fixture_t right;
        terrain_fixture_init(&legacy, TERRAIN_HUMP_LONGITUDINAL);
        terrain_fixture_init(&right, TERRAIN_HUMP_RIGHT_WHEEL);
        /* A smooth both-wheel pitch hump still belongs to the original profile.
         */
        for (unsigned segment = 0; segment < PITCH_PROFILE_SEGMENT_COUNT;
             segment++)
        {
            for (unsigned sample_index = 0;
                 sample_index < PITCH_SEGMENT_SAMPLE_COUNT; sample_index++)
            {
                float32_t pitch =
                    0U == segment ? (float32_t)(sample_index + 1U)
                    : 1U == segment ? 20
                    : 2U == segment ? (float32_t)(19U - sample_index)
                    : 3U == segment ? -(float32_t)(sample_index + 1U)
                    : 4U == segment ? -20
                                    : -(float32_t)(19U - sample_index);
                terrain_step(&legacy, pitch, 0, 0.001f, 20000);
                terrain_step(&right, pitch, 0, 0.001f, 20000);
            }
        }

        for (unsigned sample_index = 0; sample_index < LEVEL_REARM_SAMPLE_COUNT;
             sample_index++)
        {
            terrain_step(&legacy, 0, 0, 0.001f, 20000);
            terrain_step(&right, 0, 0, 0.001f, 20000);
        }

        assert(1U == legacy.completed && legacy.monitor.hump.b_is_peak_valid);
        assert(0U == right.completed && !right.monitor.hump.b_is_peak_valid);
    }
}
#if APP_IMU_DEBUG
/** @brief Check reported hump flags, rejection reasons and wheel-distance
 * validity across simulated sample sequences.
 */
static void hump_debug_tests (void)
{
    hump_detection_t detector = {0};
    uint32_t         now      = 0;
    right_samples(&detector, &now, 0, 0, 5);
    assert((unsigned)detector.debug.flags ==
           ((unsigned)HUMP_DEBUG_ARMED | (unsigned)HUMP_DEBUG_IS_ARMED));
    right_samples(&detector, &now, 0.01f, 0, 1);
    assert(HUMP_DEBUG_IS_ARMED == (int)detector.debug.flags &&
           HUMP_REASON_NO_FORWARD_TRAVEL == (int)detector.debug.reason);
    right_samples(&detector, &now, 0.01f, 0.002f, 1);
    assert((unsigned)detector.debug.flags ==
           ((unsigned)HUMP_DEBUG_STARTED | (unsigned)HUMP_DEBUG_ACTIVE));
    assert(fabsf(detector.debug.travel_m - 0.002f) < 0.000001f);
    hump_detection_update_right(&detector, 0, 5, 0.116f, 0.002f,
                                HUMP_SAMPLE_ACCELERATION_SPIKE, now += 20000U);
    assert((unsigned)detector.debug.flags ==
               ((unsigned)HUMP_DEBUG_SKIPPED | (unsigned)HUMP_DEBUG_ACTIVE) &&
           HUMP_REASON_ACCELERATION_SPIKE == (int)detector.debug.reason);
    assert(fabsf(detector.debug.travel_m - 0.002f) < 0.000001f);
    hump_detection_update_right(&detector, 0, 5, 0.116f, -0.001f,
                                HUMP_SAMPLE_VALID, now += 20000U);
    assert(HUMP_DEBUG_DISCARDED == (int)detector.debug.flags &&
           HUMP_REASON_REVERSE_TRAVEL == (int)detector.debug.reason);
    assert(fabsf(detector.debug.travel_m - 0.002f) < 0.000001f &&
           0 == detector.travel_m);

    right_samples(&detector, &now, 0, 0, 5);
    right_samples(&detector, &now, 0.01f, 0.001f, 1);
    right_samples(&detector, &now, 0, 0.001f, 5);
    assert((unsigned)detector.debug.flags ==
           ((unsigned)HUMP_DEBUG_DISCARDED | (unsigned)HUMP_DEBUG_ARMED |
            (unsigned)HUMP_DEBUG_IS_ARMED));
    assert(HUMP_REASON_INSUFFICIENT_CROSSING_TRAVEL ==
           (int)detector.debug.reason);
    assert(fabsf(detector.debug.travel_m - 0.001f) < 0.000001f);
    right_samples(&detector, &now, 0.01f, 0.002f, 4);
    right_samples(&detector, &now, 0, 0.002f, 5);
    assert((unsigned)detector.debug.flags ==
           ((unsigned)HUMP_DEBUG_COMPLETED | (unsigned)HUMP_DEBUG_ARMED |
            (unsigned)HUMP_DEBUG_IS_ARMED));
    assert(HUMP_REASON_NONE == (int)detector.debug.reason &&
           fabsf(detector.debug.travel_m - 0.008f) < 0.000001f);
    right_samples(&detector, &now, 0, 0, 1);
    assert(HUMP_DEBUG_IS_ARMED == (int)detector.debug.flags &&
           0 == detector.debug.travel_m);

    terrain_fixture_t fixture;
    terrain_fixture_init(&fixture, TERRAIN_HUMP_RIGHT_WHEEL);
    assert(fixture.monitor.hump.debug.b_is_wheel_delta_valid &&
           0 == fixture.monitor.hump.debug.wheel_delta_m);
    const float32_t roll = roll_for_height(0.01f, 0, 0.116f);

    for (unsigned sample_index = 0; sample_index < DEBUG_CLIMB_SAMPLE_COUNT;
         sample_index++)
    {
        terrain_step(&fixture, 0, roll, 0.001f, 20000);
    }

    assert(fixture.monitor.hump.b_is_hump_tracking &&
           fixture.monitor.hump.debug.b_is_wheel_delta_valid);
    const float32_t travel = fixture.monitor.hump.travel_m;
    terrain_step(&fixture, 0, roll, 0.051f, 20000);
    assert(HUMP_DEBUG_DISCARDED == (int)fixture.monitor.hump.debug.flags);
    assert(HUMP_REASON_ODOMETRY_STEP_OUT_OF_RANGE ==
           (int)fixture.monitor.hump.debug.reason);
    assert(fixture.monitor.hump.debug.b_is_wheel_delta_valid &&
           fabsf(fixture.monitor.hump.debug.wheel_delta_m - 0.051f) <
               0.000001f);
    assert(fixture.monitor.hump.debug.travel_m == travel);
    fixture.odometry.b_is_valid = false;
    terrain_step(&fixture, 0, 0, 0, 20000);
    assert(!fixture.monitor.hump.debug.b_is_wheel_delta_valid &&
           HUMP_REASON_ODOMETRY_UNAVAILABLE ==
               (int)fixture.monitor.hump.debug.reason);
    fixture.odometry.b_is_valid = true;
    terrain_step(&fixture, 0, 0, 0, 20000);
    assert(!fixture.monitor.hump.debug.b_is_wheel_delta_valid &&
           HUMP_REASON_ODOMETRY_BASELINE ==
               (int)fixture.monitor.hump.debug.reason);
    terrain_step(&fixture, 0, 0, 0, 120000);
    assert(!fixture.monitor.hump.debug.b_is_wheel_delta_valid &&
           HUMP_REASON_IMU_GAP == (int)fixture.monitor.hump.debug.reason);
}
#endif
/** @brief Apply measured acceleration sequences to the level-pickup detector
 * and check the resulting safety states.
 */
static void level_pickup_rules_tests (void)
{
    terrain_fixture_t fixture;
    terrain_fixture_init(&fixture, TERRAIN_HUMP_RIGHT_WHEEL);
    /* Just below the gentle vertical cutoff. */
    for (unsigned sample_index = 0; sample_index < NEGATIVE_PICKUP_SAMPLE_COUNT;
         sample_index++)
    {
        assert(terrain_accel(&fixture, 0.03f, 0, 1.099f, 0.001f, 20000) ==
               (unsigned)TERRAIN_SAFETY_NONE);
    }
    /* This diagonal motion exceeds the horizontal limit even though each
     * individual axis is within it. */
    for (unsigned sample_index = 0; sample_index < NEGATIVE_PICKUP_SAMPLE_COUNT;
         sample_index++)
    {
        assert(terrain_accel(&fixture, 0.11f, 0.11f, 1.18f, 0.001f, 20000) ==
               (unsigned)TERRAIN_SAFETY_NONE);
    }
    /* Run 10's highest recorded peak fails the firm directional ratio. */
    for (unsigned sample_index = 0; sample_index < NEGATIVE_PICKUP_SAMPLE_COUNT;
         sample_index++)
    {
        assert(terrain_accel(&fixture, -0.385f, 0.144f, 1.801f, 0.001f,
                             20000) == (unsigned)TERRAIN_SAFETY_NONE);
    }

    /* A gentler pickup below the old cutoff still needs a full 40 ms. */
    assert(terrain_accel(&fixture, 0.03f, 0, 1.11f, 0.001f, 20000) ==
           (unsigned)TERRAIN_SAFETY_NONE);
    assert(terrain_accel(&fixture, 0.03f, 0, 1.11f, 0.001f, 20000) ==
           (unsigned)TERRAIN_SAFETY_NONE);
    assert(terrain_accel(&fixture, 0.03f, 0, 1.11f, 0.001f, 20000) ==
           (unsigned)TERRAIN_SAFETY_LIFTED);

    /* Run 9, using the first two recorded calibrated samples of the gentle
     * pickup. */
    terrain_fixture_init(&fixture, TERRAIN_HUMP_RIGHT_WHEEL);
    assert(terrain_accel(&fixture, -0.037f, 0.018f, 1.165f, 0.001f, 20000) ==
           (unsigned)TERRAIN_SAFETY_NONE);
    assert(terrain_accel(&fixture, -0.043f, 0.002f, 1.199f, 0.001f, 40019) ==
           (unsigned)TERRAIN_SAFETY_LIFTED);

    /* Run 10's firm pickup still needs 40 ms of continuous qualification. */
    terrain_fixture_init(&fixture, TERRAIN_HUMP_RIGHT_WHEEL);
    assert(terrain_accel(&fixture, -0.247f, 0.028f, 1.545f, 0.001f, 20000) ==
           (unsigned)TERRAIN_SAFETY_NONE);
    assert(terrain_accel(&fixture, -0.319f, 0.042f, 1.763f, 0.001f, 39999) ==
           (unsigned)TERRAIN_SAFETY_NONE);
    assert(terrain_accel(&fixture, -0.247f, 0.028f, 1.545f, 0.001f, 50001) ==
           (unsigned)TERRAIN_SAFETY_NONE);
    assert(terrain_accel(&fixture, -0.319f, 0.042f, 1.763f, 0.001f, 40110) ==
           (unsigned)TERRAIN_SAFETY_LIFTED);
}

/** @brief Feed motion and acceleration histories that distinguish normal
 * travel, pickup and overturn.
 */
static void pickup_and_overturn_tests (void)
{
    terrain_fixture_t fixture;
    terrain_fixture_init(&fixture, TERRAIN_HUMP_RIGHT_WHEEL);
    /* Horizontal acceleration/braking and a single vertical jolt must not end a
     * drive. */
    for (unsigned sample_index = 0;
         sample_index < LATERAL_ACCELERATION_SAMPLE_COUNT; sample_index++)
    {
        assert(terrain_accel(&fixture, 0.20f, 0, 1.0f, 0.001f, 20000) ==
               (unsigned)TERRAIN_SAFETY_NONE);
    }

    for (unsigned sample_index = 0;
         sample_index < LATERAL_ACCELERATION_SAMPLE_COUNT; sample_index++)
    {
        assert(terrain_accel(&fixture, -0.20f, 0, 1.0f, 0.001f, 20000) ==
               (unsigned)TERRAIN_SAFETY_NONE);
    }

    assert(terrain_accel(&fixture, 0, 0, 1.32f, 0.001f, 20000) ==
           (unsigned)TERRAIN_SAFETY_NONE);
    assert(terrain_step(&fixture, 0, 0, 0.001f, 20000) ==
           (unsigned)TERRAIN_SAFETY_NONE);

    /* Upward acceleration while still level and while the encoders report
     * motion. */
    for (unsigned sample_index = 0;
         sample_index < LIFT_CONFIRMATION_PREFIX_COUNT; sample_index++)
    {
        assert(terrain_accel(&fixture, 0, 0, 1.32f, 0.001f, 20000) ==
               (unsigned)TERRAIN_SAFETY_NONE);
    }

    assert(terrain_accel(&fixture, 0, 0, 1.32f, 0.001f, 20000) ==
           (unsigned)TERRAIN_SAFETY_LIFTED);
    assert(!fixture.monitor.result.b_is_hump_active &&
           !fixture.monitor.result.b_is_hump_complete);
    terr_data_reset_run(&fixture.monitor);

    for (unsigned sample_index = 0;
         sample_index < RECOVERY_BEFORE_INVALID_SAMPLE_COUNT; sample_index++)
    {
        assert(terrain_step(&fixture, 0, 0, 0, 20000) ==
               (unsigned)TERRAIN_SAFETY_LIFTED);
    }

    fixture.sample.b_is_valid = false;
    assert(terrain_step(&fixture, 0, 0, 0, 20000) ==
           (unsigned)TERRAIN_SAFETY_LIFTED);
    fixture.sample.b_is_valid = true;

    for (unsigned sample_index = 0;
         sample_index < RECOVERY_AFTER_INVALID_SAMPLE_COUNT; sample_index++)
    {
        assert(terrain_step(&fixture, 0, 0, 0, 20000) ==
               (unsigned)TERRAIN_SAFETY_LIFTED);
    }

    terrain_step(&fixture, 0, 0, 0, 20000);
    assert((unsigned)TERRAIN_SAFETY_NONE == fixture.monitor.result.safety);

    /* An angled pickup and an upside-down gravity vector are distinct hazards.
     */
    for (unsigned sample_index = 0;
         sample_index < ANGLED_LIFT_CONFIRMATION_PREFIX_COUNT; sample_index++)
    {
        assert(terrain_step(&fixture, 0, 30, 0.001f, 20000) ==
               (unsigned)TERRAIN_SAFETY_NONE);
    }

    assert(terrain_step(&fixture, 0, 30, 0.001f, 20000) ==
           (unsigned)TERRAIN_SAFETY_LIFTED);
    assert(terrain_accel(&fixture, 0, 0, -1.0f, 0.001f, 20000) ==
           (unsigned)TERRAIN_SAFETY_OVERTURNED);

    for (unsigned sample_index = 0; sample_index < OVERTURN_HOLD_SAMPLE_COUNT;
         sample_index++)
    {
        assert(terrain_accel(&fixture, 0, 0, -1.0f, 0, 20000) ==
               (unsigned)TERRAIN_SAFETY_OVERTURNED);
    }

    for (unsigned sample_index = 0;
         sample_index < OVERTURN_RECOVERY_SAMPLE_COUNT; sample_index++)
    {
        terrain_step(&fixture, 0, 0, 0, 20000);
    }

    assert((unsigned)TERRAIN_SAFETY_NONE == fixture.monitor.result.safety);
    assert(terrain_accel(&fixture, 0, 0, 0.10f, 0, 20000) ==
           (unsigned)TERRAIN_SAFETY_LIFTED);
}

/* End of file. */

