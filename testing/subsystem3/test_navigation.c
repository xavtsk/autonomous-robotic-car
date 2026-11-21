/** @file test_navigation.c
 * @brief Exercise navigation, barcode and odometry integration using synthetic
 * sensor frames.
 * Unsigned casts spell out the original modulo conversion.
 * The selected narrow integer operands fit in int.
 */
/* Synthetic integration inputs. No hardware results are implied by these tests.
 */
#include "test_navigation.h"
#include "../../app_program/app_types.h"
#include "host_trace_main.h"
#include <assert.h>
#include <math.h>
#include <string.h>
#include "line_navigation.h"

#define CALIBRATION_SAMPLE_COUNT 10U
#define START_CASE_COUNT 8U
#define ODOMETRY_JITTER_TICK_COUNT 40U

#define CODE39_ELEMENT_COUNT 9U
#define FIXTURE_FRAME_RUN_COUNT 29U
#define SAMPLING_CADENCE_CASE_COUNT 2U
#define SCAN_DIRECTION_COUNT 2U

typedef struct
{
    line_navigation_t          nav;
    line_navigation_output_t   out;
    ir_acquisition_frame_t     frame;
    line_navigation_odometry_t odometry;
    uint32_t                   now;
    uint32_t                   step;
    float64_t                  velocity;
    unsigned int               barcodes;
    unsigned int               lost;
    unsigned int               acquired;
    unsigned int               junctions;
    unsigned int               faults;
    unsigned int               rejects;
    unsigned int               diagnostic_letters;
    char                       letter;
    line_navigation_action_t   action;
    bool                       b_has_jitter;
    bool                       b_should_omit_odometry;
    bool                       b_was_last_reversed;
} rig_t;

static void test_readiness_and_updates (void);

static void test_faults (void);

static void test_rearming_and_timing (void);

static void test_line_modes_and_diagnostics (void);

static void test_clearance_epochs (void);

#ifndef S3_BASELINE_TRACE
static void test_faint_barcode (void);
#endif

/** @brief Run the synthetic navigation integration assertions.
 * @see test_navigation.h for the selected cases and failure contract.
 */
void test_navigation (void)
{
#ifndef S3_BASELINE_TRACE
    test_faint_barcode();
#endif
    test_readiness_and_updates();
    test_faults();
    test_rearming_and_timing();
    test_line_modes_and_diagnostics();
    test_clearance_epochs();
}

/** @brief Build one collector from ten white samples at 150 ADC counts and
 * ten black samples at 280 counts. Assert acceptance of each
 * reading.
 */
static ir_calibration_collector_t calibration (void)
{
    ir_calibration_collector_t collector;
    unsigned int               index;
    ir_calibration_begin(&collector);

    for (index = 0U; index < CALIBRATION_SAMPLE_COUNT; index++)
    {
        assert(ir_calibration_add(&collector, IR_SURFACE_WHITE, 150U));
        assert(ir_calibration_add(&collector, IR_SURFACE_BLACK, 280U));
    }

    return collector;
}

/** @brief Apply the fixture's accepted calibration to every simulated
 * navigation sensor.
 */
static void accept_calibrations (rig_t * p_rig)
{
    ir_calibration_collector_t collector = calibration();
    unsigned int               index;

    for (index = 0U; index < (unsigned)S3_SENSOR_COUNT; index++)
    {
        assert(line_navigation_set_calibration(
            &p_rig->nav, (ir_acquisition_sensor_id_t)index, &collector));
    }
}

/** @brief Stamp the three raw readings and odometry with the fixture clock.
 * Mark the sensor readings and odometry as available.
 */
static void prepare_frame (rig_t * p_rig, uint16_t barcode, uint16_t left,
                           uint16_t right)
{
    unsigned int index;
    p_rig->frame              = (ir_acquisition_frame_t){0};
    p_rig->frame.timestamp_us = p_rig->now;

    for (index = 0U; index < (unsigned)S3_SENSOR_COUNT; index++)
    {
        p_rig->frame.sensor[index] =
            (ir_acquisition_sample_t){p_rig->now, 0U, 0, true, true};
    }

    p_rig->frame.sensor[0].raw   = barcode;
    p_rig->frame.sensor[1].raw   = left;
    p_rig->frame.sensor[2].raw   = right;
    p_rig->odometry.timestamp_us = p_rig->now;
    p_rig->odometry.b_is_valid   = true;
}

/** @brief Run one navigation update with optional odometry, accumulate
 * events and advance the fixture clock and centre distance by the
 * selected step.
 */
static void process (rig_t * p_rig, bool b_has_odometry)
{
    uint32_t step = p_rig->step;
    assert(test_nav_step(&p_rig->nav, &p_rig->frame,
                         b_has_odometry ? &p_rig->odometry : 0, &p_rig->out));

    if ((unsigned)S3_BARCODE_DECODED == p_rig->out.barcode_status)
    {
        if (p_rig->out.b_is_diagnostic)
        {
            p_rig->diagnostic_letters++;
        }

        p_rig->b_was_last_reversed = p_rig->out.barcode.b_is_reversed;
    }

    if ((unsigned)p_rig->out.events & (unsigned)S3_EVENT_BARCODE)
    {
        p_rig->barcodes++;
        p_rig->letter = p_rig->out.barcode.letter;
        p_rig->action = p_rig->out.action;
    }

    if ((unsigned)p_rig->out.events & (unsigned)S3_EVENT_LINE_LOST)
    {
        p_rig->lost++;
    }

    if ((unsigned)p_rig->out.events & (unsigned)S3_EVENT_LINE_REACQUIRED)
    {
        p_rig->acquired++;
    }

    if ((unsigned)p_rig->out.events & (unsigned)S3_EVENT_JUNCTION)
    {
        p_rig->junctions++;
    }

    if ((unsigned)p_rig->out.events & (unsigned)S3_EVENT_SENSOR_FAULT)
    {
        p_rig->faults++;
    }

    if ((unsigned)S3_BARCODE_REJECTED == p_rig->out.barcode_status)
    {
        p_rig->rejects++;
    }

    if (p_rig->out.lifecycle != (unsigned)S3_RUN_ACTIVE)
    {
        assert(!p_rig->out.steering.b_is_valid &&
               (unsigned)S3_NAV_NONE == p_rig->out.action);
        assert((p_rig->out.events & ~(uint32_t)S3_EVENT_SENSOR_FAULT) == 0U);
    }

    if (p_rig->b_has_jitter)
    {
        step = ((p_rig->now % 2U) == 0U) ? step + 1001U : step - 1001U;
    }

    p_rig->now               += step;
    p_rig->odometry.centre_m += p_rig->velocity * (float64_t)step / 1000000.0;
}

/** @brief Build one frame from the barcode, left and right ADC counts and
 * process it with the rig's current odometry setting.
 */
static void tick (rig_t * p_rig, uint16_t barcode, uint16_t left,
                  uint16_t right)
{
    prepare_frame(p_rig, barcode, left, right);
    process(p_rig, !p_rig->b_should_omit_odometry);
}

/** @brief Repeat the supplied ADC readings until the requested number of
 * microseconds has elapsed, allowing clock wrap.
 */
static void hold (rig_t * p_rig, uint32_t duration, uint16_t barcode,
                  uint16_t left, uint16_t right)
{
    uint32_t start = p_rig->now;

    do
    {
        tick(p_rig, barcode, left, right);
    } while ((uint32_t)(p_rig->now - start) < duration);
}

/** @brief Initialize the navigation fixture at the supplied sample period,
 * apply calibration, feed the first frame and check that start is
 * accepted.
 */
static void init_rig (rig_t * p_rig, uint32_t step, bool b_is_active)
{
    line_navigation_config_t cfg = line_navigation_defaults();
    *p_rig                       = (rig_t){0};
    p_rig->step                  = step;
    cfg.acquisition.enabled_mask = S3_ALL_SENSORS;
    assert(line_navigation_init(&p_rig->nav, &cfg));

    if (b_is_active)
    {
        accept_calibrations(p_rig);
        tick(p_rig, 150U, 202U, 202U);
        assert(line_navigation_start(&p_rig->nav, p_rig->now));
    }
}

/** @brief Feed leading paper, a framed Code 39 letter in the selected
 * direction, and trailing paper to the navigation fixture.
 */
static void scan (rig_t * p_rig, char letter, bool b_is_reverse)
{
    static const uint16_t patterns[26] = {
        0x109, 0x049, 0x148, 0x019, 0x118, 0x058, 0x00D, 0x10C, 0x04C,
        0x01C, 0x103, 0x043, 0x142, 0x013, 0x112, 0x052, 0x007, 0x106,
        0x046, 0x016, 0x181, 0x0C1, 0x1C0, 0x091, 0x190, 0x0D0};
    uint32_t     runs[29];
    uint16_t     symbols[3] = {0x094U, patterns[(unsigned int)(letter - 'A')],
                               0x094U};
    unsigned int letter_index;
    unsigned int index;

    for (letter_index = 0U; letter_index < (sizeof symbols / sizeof symbols[0]);
         letter_index++)
    {
        for (index = 0U; index < CODE39_ELEMENT_COUNT; index++)
        {
            runs[(letter_index * 10U) + index] =
                (symbols[letter_index] & (1U << (8U - index))) ? 120000U
                                                               : 40000U;
        }

        if (letter_index < 2U)
        {
            runs[(letter_index * 10U) + 9U] = 40000U;
        }
    }

    hold(p_rig, 600000U, 150U, 202U, 202U);

    for (index = 0U; index < FIXTURE_FRAME_RUN_COUNT; index++)
    {
        hold(p_rig, runs[b_is_reverse ? 28U - index : index],
             (index % 2U) ? 150U : 280U, 202U, 202U);
    }

    hold(p_rig, 600000U, 150U, 202U, 202U);
}

/** @brief Check initialization, calibration readiness and configuration
 * rejection, including an out-of-range map action.
 * @details Calibrate all channels in the fixture and force fresh IR and
 * odometry before starting. Verify that a rejected map leaves the active
 * mapping intact and that a run locks both map and calibration changes.
 * After stopping, verify diagnostic steering only and test each missing
 * sensor channel as a separate start blocker.
 */
static void test_readiness_and_updates (void)
{
    rig_t                      rig;
    line_navigation_config_t   cfg       = line_navigation_defaults();
    ir_calibration_collector_t collector = calibration();
    ir_calibration_collector_t empty;
    line_navigation_action_t   map[26] = {S3_NAV_RIGHT};
    unsigned int               index;
    init_rig(&rig, 10000U, false);
    assert(S3_RUN_INACTIVE == (int)rig.nav.lifecycle);
    tick(&rig, 150U, 202U, 202U);
    assert((unsigned)S3_READY_CALIBRATION == rig.out.readiness_blockers);
    assert(!line_navigation_start(&rig.nav, rig.now));
    assert(0U == rig.nav.accepted_calibration);
    assert(
        line_navigation_set_calibration(&rig.nav, S3_SENSOR_LEFT, &collector));
    assert(S3_SENSOR_BIT(S3_SENSOR_LEFT) == rig.nav.accepted_calibration);
    ir_calibration_begin(&empty);
    assert(!line_navigation_set_calibration(&rig.nav, S3_SENSOR_LEFT, &empty));
    assert(S3_SENSOR_BIT(S3_SENSOR_LEFT) == rig.nav.accepted_calibration);
    assert(150U == rig.nav.config.calibration[1].white_raw);
    accept_calibrations(&rig);
    assert(!line_navigation_start(
        &rig.nav, rig.now)); /* New calibration needs fresh inputs. */
    prepare_frame(&rig, 150U, 202U, 202U);
    process(&rig, false);
    assert((unsigned)S3_READY_ODOMETRY == rig.out.readiness_blockers);
    assert(!line_navigation_start(&rig.nav, rig.now));
    tick(&rig, 150U, 202U, 202U);
    assert(line_navigation_readiness(&rig.nav, rig.now) == 0U);
    assert(!line_navigation_start(&rig.nav, rig.now + 40000U));
    map[25] = S3_NAV_UTURN;
    assert(line_navigation_set_mapping(&rig.nav, map));
    map[0]  = S3_NAV_LEFT;
    map[25] = S3_NAV_ACTION_COUNT;
    assert(!line_navigation_set_mapping(&rig.nav, map));
    assert((unsigned)S3_NAV_RIGHT == rig.nav.config.map[0] &&
           (unsigned)S3_NAV_UTURN == rig.nav.config.map[25]);
    map[25] = (line_navigation_action_t)-1;
    assert(!line_navigation_set_mapping(&rig.nav, map));
    map[25] = S3_NAV_NONE;
    assert(!line_navigation_set_mapping(&rig.nav, 0));
    assert(line_navigation_start(&rig.nav, rig.now));
    assert(!line_navigation_start(&rig.nav, rig.now));
    assert(
        !line_navigation_set_calibration(&rig.nav, S3_SENSOR_LEFT, &collector));
    assert(!line_navigation_set_mapping(&rig.nav, map));
    assert((unsigned)S3_NAV_RIGHT == rig.nav.config.map[0]);
    hold(&rig, 50000U, 150U, 202U, 202U);
    assert(rig.out.steering.b_is_valid && 1U == rig.acquired);
    assert(line_navigation_end_run(&rig.nav));
    hold(&rig, 50000U, 150U, 202U, 202U);
    assert(rig.out.steering_preview.b_is_valid && !rig.out.steering.b_is_valid);
    assert(line_navigation_set_mapping(&rig.nav, map));
    assert(
        line_navigation_set_calibration(&rig.nav, S3_SENSOR_LEFT, &collector));
    /* Each disabled channel independently prevents start. */
    for (index = 0U; index < (unsigned)S3_SENSOR_COUNT; index++)
    {
        cfg.acquisition.enabled_mask =
            (uint8_t)(S3_ALL_SENSORS & ~S3_SENSOR_BIT(index));
        assert(line_navigation_init(&rig.nav, &cfg));
        accept_calibrations(&rig);
        tick(&rig, 150U, 202U, 202U);
        assert((unsigned)rig.out.readiness_blockers &
               (unsigned)S3_READY_CHANNELS);
        assert(!line_navigation_start(&rig.nav, rig.now));
    }

    cfg = line_navigation_defaults();
    assert(line_navigation_init(&rig.nav, &cfg));
    tick(&rig, 150U, 202U, 202U);
    assert(rig.out.sensor[0].b_is_valid && !rig.out.sensor[1].b_is_valid &&
           0U == rig.out.fault_mask);
    cfg.rearm_distance_m = NAN;
    assert(!line_navigation_init(&rig.nav, &cfg));
    cfg                = line_navigation_defaults();
    cfg.fault_clear_us = 0U;
    assert(!line_navigation_init(&rig.nav, &cfg));
    cfg        = line_navigation_defaults();
    cfg.map[0] = S3_NAV_ACTION_COUNT;
    assert(!line_navigation_init(&rig.nav, &cfg));
}

/** @brief Inject eight navigation fault scenarios and check their reported
 * reasons and reset conditions.
 */
static void test_faults (void)
{
    unsigned int kind;

    for (kind = 0U; kind < START_CASE_COUNT; kind++)
    {
        rig_t                      rig;
        uint32_t                   reason;
        ir_calibration_collector_t collector = calibration();
        init_rig(&rig, 10000U, true);
        hold(&rig, 50000U, 150U, 202U, 202U);

        if (1U == kind)
        {
            rig.now += 40000U;
        }

        prepare_frame(&rig, 150U, 202U, 202U);

        switch (kind)
        {
        case 0U:
            rig.frame.sensor[1].b_is_valid = false;
            reason                         = S3_FAULT_SENSOR;
        break;
        case 1U:
            reason = S3_FAULT_TIMING;
        break;
        case 2U:
            rig.odometry.timestamp_us -= 40000U;
            reason                     = S3_FAULT_ODOMETRY;
        break;
        case 3U:
            rig.odometry.epoch++;
            reason = S3_FAULT_ODOMETRY_EPOCH;
        break;
        case 4U:
            rig.odometry.centre_m = NAN;
            reason                = S3_FAULT_ODOMETRY;
        break;
        case 5U:
            reason = S3_FAULT_ODOMETRY;
        break;
        case 6U:
            rig.frame.sensor[2].timestamp_us -= 40000U;
            reason                            = S3_FAULT_SENSOR;
        break;
        default:
            rig.odometry.b_is_valid = false;
            reason                  = S3_FAULT_ODOMETRY;
        break;
        }

        process(&rig, kind != 5U);
        assert(rig.out.b_is_stop_requested &&
               (unsigned)S3_RUN_FAULT_LATCHED == rig.out.lifecycle);
        assert(rig.out.latched_faults & reason);
        assert(1U == rig.faults && !rig.out.steering.b_is_valid);
        assert(!line_navigation_reset_fault(&rig.nav, rig.now));
        assert(!line_navigation_end_run(&rig.nav));
        assert(!line_navigation_start(&rig.nav, rig.now));
        assert(!line_navigation_set_calibration(&rig.nav, S3_SENSOR_LEFT,
                                                &collector));
        assert(!line_navigation_set_mapping(&rig.nav, rig.nav.config.map));
        rig.odometry.centre_m = 0.0;
        hold(&rig, 50000U, 150U, 202U, 202U);
        assert(rig.out.b_is_stop_requested && 1U == rig.faults &&
               0U == rig.out.fault_mask);
        assert(!line_navigation_reset_fault(&rig.nav, rig.now));
        hold(&rig, 100000U, 150U, 202U, 202U);
        assert(rig.out.b_is_stop_requested && !rig.out.steering.b_is_valid);
        assert(!line_navigation_reset_fault(
            &rig.nav, rig.now + 40000U)); /* Stale at reset call. */
        assert(line_navigation_reset_fault(&rig.nav, rig.now));
        tick(&rig, 150U, 202U, 202U);
        assert(!rig.out.b_is_stop_requested && 0U == rig.out.latched_faults &&
               !rig.out.steering.b_is_valid);
        assert(line_navigation_start(&rig.nav, rig.now));
        hold(&rig, 50000U, 150U, 202U, 202U);
        assert(rig.out.steering.b_is_valid && 1U == rig.faults);
    }
    /* Continuous healthy samples are required; waiting without processing is
     * insufficient. */
    {
        rig_t rig;
        init_rig(&rig, 10000U, true);
        prepare_frame(&rig, 150U, 202U, 202U);
        rig.frame.sensor[0].error = -9;
        process(&rig, true);
        tick(&rig, 150U, 202U, 202U);
        assert(!line_navigation_reset_fault(&rig.nav, rig.now + 100000U));
        hold(&rig, 80000U, 150U, 202U, 202U);
        prepare_frame(&rig, 150U, 202U, 202U);
        rig.odometry.epoch++;
        process(&rig, true);
        hold(&rig, 50000U, 150U, 202U, 202U);
        assert(!line_navigation_reset_fault(&rig.nav, rig.now));
    }
}

/** @brief Check barcode actions and rearming across sample periods, scan
 * directions and subsequent scans.
 */
static void test_rearming_and_timing (void)
{
    unsigned int cadence;
    unsigned int direction;

    for (cadence = 0U; cadence < SAMPLING_CADENCE_CASE_COUNT; cadence++)
    {
        for (direction = 0U; direction < SCAN_DIRECTION_COUNT; direction++)
        {
            rig_t        rig;
            float64_t    anchor;
            unsigned int index;
            init_rig(&rig, cadence ? 10000U : 5000U, true);
            rig.b_has_jitter = true;
            scan(&rig, 'A', direction != 0U);
            assert(1U == rig.barcodes && 'A' == rig.letter &&
                   (unsigned)S3_NAV_LEFT == rig.action);
            assert(rig.b_was_last_reversed == (direction != 0U));
            assert(rig.out.b_is_rearming &&
                   rig.out.capture_distance.b_is_start_valid &&
                   rig.out.capture_distance.b_is_end_valid);
            assert(0.0 == rig.out.capture_distance.start_m &&
                   0.0 == rig.out.capture_distance.end_m);
            assert(0U == rig.nav.barcode.count);
            anchor = rig.nav.rearm_anchor_m;
            /* Oscillations accumulate travel, but never sufficient NET
             * displacement. */
            for (index = 0U; index < ODOMETRY_JITTER_TICK_COUNT; index++)
            {
                rig.odometry.centre_m =
                    anchor + ((index % 2U) ? 0.015 : -0.015);
                tick(&rig, 150U, 202U, 202U);
                assert(rig.out.b_is_rearming);
            }

            scan(&rig, 'B', false);
            assert(1U == rig.barcodes);
            rig.odometry.centre_m = anchor + (direction ? -0.021 : 0.021);
            hold(&rig, 100000U, 150U, 202U, 202U);
            assert(!rig.out.b_is_rearming);
            scan(&rig, 'Z', direction != 0U);
            assert(2U == rig.barcodes && 'Z' == rig.letter &&
                   (unsigned)S3_NAV_NONE == rig.action);
            assert(rig.out.b_is_rearming);
            /* Bad odometry during rearming cannot turn it into time-only
             * rearming. */
            prepare_frame(&rig, 150U, 202U, 202U);
            process(&rig, false);
            assert(rig.out.b_is_stop_requested && rig.out.b_is_rearming);
        }
    }
    /* Moving synthetic input records actual endpoint distances. */
    {
        rig_t rig;
        init_rig(&rig, 10000U, true);
        rig.velocity = -0.1;
        scan(&rig, 'C', true);
        assert(1U == rig.barcodes && (unsigned)S3_NAV_STRAIGHT == rig.action);
        assert(rig.out.capture_distance.end_m <
               rig.out.capture_distance.start_m);
        hold(&rig, 500000U, 150U, 202U, 202U);
        assert(!rig.out.b_is_rearming);
    }
    /* A malformed capture also requires clearance; it is not a critical fault.
     */
    {
        rig_t rig;
        init_rig(&rig, 10000U, true);
        hold(&rig, 500000U, 150U, 202U, 202U);
        hold(&rig, 1100000U, 280U, 202U, 202U);
        assert(1U == rig.rejects && rig.out.b_is_rearming &&
               !rig.out.b_is_stop_requested);
        rig.odometry.centre_m = 0.025;
        hold(&rig, 100000U, 150U, 150U, 150U);
        assert(
            rig.out
                .b_is_rearming); /* Distance alone cannot clear absent line. */
        hold(&rig, 100000U, 280U, 202U, 202U);
        assert(rig.out.b_is_rearming); /* Or black barcode input. */
        hold(&rig, 100000U, 150U, 202U, 202U);
        assert(!rig.out.b_is_rearming);
        scan(&rig, 'B', false);
        assert(1U == rig.barcodes && (unsigned)S3_NAV_RIGHT == rig.action);
    }
}

/** @brief Exercise normal and diagnostic line processing and check steering
 * validity, stop requests and barcode actions.
 */
static void test_line_modes_and_diagnostics (void)
{
    rig_t                    rig;
    line_navigation_action_t map[26] = {0};
    init_rig(&rig, 10000U, true);
    hold(&rig, 250000U, 150U, 150U, 150U);
    assert(1U == rig.lost && !rig.out.b_is_stop_requested &&
           !rig.out.steering.b_is_valid);
    hold(&rig, 50000U, 150U, 202U, 202U);
    assert(1U == rig.acquired && rig.out.steering.b_is_valid);
    hold(&rig, 500000U, 150U, 202U, 202U);
    hold(&rig, 200000U, 280U, 150U, 150U);
    assert(rig.nav.barcode.b_is_collecting && 2U == rig.lost &&
           !rig.out.b_is_stop_requested);
    assert(line_navigation_set_mode(&rig.nav, S3_MODE_SUPPRESSED));
    assert(!rig.nav.barcode.b_is_collecting);
    scan(&rig, 'A', false);
    hold(&rig, 150000U, 150U, 280U, 280U);
    assert(0U == rig.barcodes && 0U == rig.junctions &&
           !rig.out.steering.b_is_valid);
    assert(line_navigation_set_mode(&rig.nav, S3_MODE_TRACK));
    hold(&rig, 100000U, 150U, 202U, 202U);
    assert(0U == rig.barcodes && rig.out.b_is_rearming);
    rig.odometry.centre_m = 0.025;
    hold(&rig, 100000U, 150U, 202U, 202U);
    scan(&rig, 'D', false);
    assert(1U == rig.barcodes && (unsigned)S3_NAV_UTURN == rig.action);
    hold(&rig, 50000U, 150U, 280U, 280U);
    assert(1U == rig.junctions && !rig.out.steering.b_is_valid);
    assert(line_navigation_end_run(&rig.nav));
    map[25] = S3_NAV_LEFT;
    assert(line_navigation_set_mapping(&rig.nav, map));
    tick(&rig, 150U, 202U, 202U);
    assert(line_navigation_start(&rig.nav, rig.now));
    scan(&rig, 'Z', false);
    assert((unsigned)S3_NAV_LEFT == rig.action);
    /* Inactive diagnostic frames need no odometry and cannot emit actions. */
    init_rig(&rig, 10000U, false);
    rig.b_should_omit_odometry = true;
    scan(&rig, 'A', false);
    assert(0U == rig.barcodes && rig.out.b_is_diagnostic &&
           (unsigned)S3_NAV_NONE == rig.out.action);
    assert(1U == rig.diagnostic_letters && !rig.out.b_has_odometry);
    assert(!rig.nav.capture_distance.b_is_start_valid &&
           !rig.nav.capture_distance.b_is_end_valid);
    /* Wraparound clocks remain supported through lifecycle and recovery. */
    init_rig(&rig, 5000U, false);
    rig.now = 0xFFFF0000U;
    accept_calibrations(&rig);
    tick(&rig, 150U, 202U, 202U);
    assert(line_navigation_start(&rig.nav, rig.now));
    hold(&rig, 150000U, 150U, 202U, 202U);
    assert(rig.out.steering.b_is_valid && !rig.out.b_is_stop_requested);
}

/** @brief Check how calibration and navigation transitions affect clearance
 * and whether the running owner accepts subsequent requests.
 */
static void test_clearance_epochs (void)
{
    rig_t                      rig;
    line_navigation_config_t   cfg       = line_navigation_defaults();
    ir_calibration_collector_t collector = calibration();
    init_rig(&rig, 10000U, false);
    cfg.acquisition.enabled_mask = S3_ALL_SENSORS;
    cfg.rearm_stable_us          = 60000U;
    cfg.rearm_distance_m         = 0.050;
    cfg.fault_clear_us           = 200000U;
    assert(line_navigation_init(&rig.nav, &cfg));
    accept_calibrations(&rig);
    tick(&rig, 150U, 202U, 202U);
    assert(line_navigation_start(&rig.nav, rig.now));
    scan(&rig, 'A', false);
    rig.odometry.centre_m = 0.025;
    hold(&rig, 100000U, 150U, 202U, 202U);
    assert(rig.out.b_is_rearming); /* Configured clearance is 50 mm. */
    tick(&rig, 280U, 202U, 202U);  /* Interrupt the white interval. */
    rig.odometry.centre_m = 0.060;
    hold(&rig, 50000U, 150U, 202U, 202U);
    assert(
        rig.out
            .b_is_rearming); /* Still short of configured 60 ms persistence. */
    tick(&rig, 280U, 202U, 202U);
    hold(&rig, 50000U, 150U, 202U, 202U);
    assert(rig.out.b_is_rearming);
    hold(&rig, 30000U, 150U, 202U, 202U);
    assert(!rig.out.b_is_rearming);
    scan(&rig, 'B', false);
    assert(2U == rig.barcodes && rig.out.b_is_rearming);
    rig.odometry.epoch++;
    rig.odometry.centre_m = 10.0;
    tick(&rig, 150U, 202U, 202U);
    assert(rig.out.b_is_stop_requested && rig.out.b_is_rearming &&
           2U == rig.barcodes);
    assert((unsigned)rig.out.latched_faults &
           (unsigned)S3_FAULT_ODOMETRY_EPOCH);
    hold(&rig, 120000U, 150U, 202U, 202U);
    assert(!line_navigation_reset_fault(&rig.nav, rig.now));
    hold(&rig, 100000U, 150U, 202U, 202U);
    assert(line_navigation_reset_fault(&rig.nav, rig.now));
    assert(line_navigation_start(&rig.nav, rig.now));
    assert(line_navigation_set_mode(&rig.nav, S3_MODE_SUPPRESSED));
    prepare_frame(&rig, 150U, 202U, 202U);
    rig.frame.sensor[0].b_is_valid = false;
    process(&rig, true);
    assert(2U == rig.faults && rig.out.b_is_stop_requested);
    /* Faulted calibration updates cannot clear the latch. */
    assert(!line_navigation_set_calibration(&rig.nav, S3_SENSOR_BARCODE,
                                            &collector));
}

/* "make compare" replays only the pre-recorder tests. */
#ifndef S3_BASELINE_TRACE
/** @brief Check that a small barcode produces one event when thin bars read
 * near 550 and gaps near 300, below the fixed black threshold.
 */
static void test_faint_barcode (void)
{
    static const uint16_t sample_darkness = 0x190U;
    uint16_t              symbols[3]      = {0x094U, sample_darkness, 0x094U};
    unsigned int          letter_index;
    unsigned int          index;
    unsigned int          direction;

    for (direction = 0U; direction < SCAN_DIRECTION_COUNT; direction++)
    {
        rig_t    rig;
        uint32_t runs[29];
        init_rig(&rig, 10000U, true);
        rig.b_has_jitter = true;

        for (letter_index = 0U;
             letter_index < (sizeof symbols / sizeof symbols[0]);
             letter_index++)
        {
            for (index = 0U; index < CODE39_ELEMENT_COUNT; index++)
            {
                runs[(letter_index * 10U) + index] =
                    (symbols[letter_index] & (1U << (8U - index))) ? 130000U
                                                                   : 50000U;
            }

            if (letter_index < 2U)
            {
                runs[(letter_index * 10U) + 9U] = 50000U;
            }
        }

        hold(&rig, 1500000U, 150U, 202U, 202U);

        for (index = 0U; index < FIXTURE_FRAME_RUN_COUNT; index++)
        {
            /* Raw 222 maps to darkness 553 and raw 189 to 300. */
            hold(&rig, runs[direction ? 28U - index : index],
                 (index % 2U) ? 189U : 222U, 202U, 202U);
        }

        hold(&rig, 1500000U, 150U, 202U, 202U);
        assert(1U == rig.barcodes && 'Y' == rig.letter && 0U == rig.rejects);
        assert(rig.b_was_last_reversed == (direction != 0U));
        assert(rig.out.b_is_rearming);
    }
}
#endif

/* End of file. */

