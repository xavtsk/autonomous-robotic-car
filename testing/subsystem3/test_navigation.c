/* Synthetic integration inputs. No hardware results are implied by these tests.
 */
#include <assert.h>
#include <math.h>
#include <string.h>
#include "line_navigation.h"

typedef struct
{
    s3_navigation_t nav;
    s3_navigation_output_t out;
    s3_sensor_frame_t frame;
    s3_odometry_sample_t odometry;
    uint32_t now, step;
    double velocity;
    unsigned int barcodes, lost, acquired, junctions, faults, rejects,
        diagnostic_letters;
    char letter;
    s3_nav_action_t action;
    bool jitter, omit_odometry, last_reversed;
} rig_t;
static s3_calibration_collector_t calibration(void)
{
    s3_calibration_collector_t c;
    unsigned int i;
    s3_calibration_begin(&c);
    for (i = 0U; i < 10U; i++)
    {
        assert(s3_calibration_add(&c, IR_SURFACE_WHITE, 150U));
        assert(s3_calibration_add(&c, IR_SURFACE_BLACK, 280U));
    }
    return c;
}
static void accept_calibrations(rig_t *rig)
{
    s3_calibration_collector_t c = calibration();
    unsigned int i;
    for (i = 0U; i < S3_SENSOR_COUNT; i++)
    {
        assert(
            s3_navigation_apply_calibration(&rig->nav, (s3_sensor_id_t)i, &c));
    }
}
static void prepare_frame(rig_t *rig, uint16_t barcode, uint16_t left,
                          uint16_t right)
{
    unsigned int i;
    rig->frame = (s3_sensor_frame_t){0};
    rig->frame.timestamp_us = rig->now;
    for (i = 0U; i < S3_SENSOR_COUNT; i++)
    {
        rig->frame.sensor[i] = (s3_raw_sample_t){rig->now, 0U, 0, true, true};
    }
    rig->frame.sensor[0].raw = barcode;
    rig->frame.sensor[1].raw = left;
    rig->frame.sensor[2].raw = right;
    rig->odometry.timestamp_us = rig->now;
    rig->odometry.valid = true;
}
static void process(rig_t *rig, bool with_odometry)
{
    uint32_t step = rig->step;
    assert(s3_navigation_process(
        &rig->nav, &rig->frame, with_odometry ? &rig->odometry : 0, &rig->out));
    if (rig->out.barcode_status == S3_BARCODE_DECODED)
    {
        if (rig->out.diagnostic)
        {
            rig->diagnostic_letters++;
        }
        rig->last_reversed = rig->out.barcode.reversed;
    }
    if (rig->out.events & S3_EVENT_BARCODE)
    {
        rig->barcodes++;
        rig->letter = rig->out.barcode.letter;
        rig->action = rig->out.action;
    }
    if (rig->out.events & S3_EVENT_LINE_LOST)
    {
        rig->lost++;
    }
    if (rig->out.events & S3_EVENT_LINE_REACQUIRED)
    {
        rig->acquired++;
    }
    if (rig->out.events & S3_EVENT_JUNCTION)
    {
        rig->junctions++;
    }
    if (rig->out.events & S3_EVENT_SENSOR_FAULT)
    {
        rig->faults++;
    }
    if (rig->out.barcode_status == S3_BARCODE_REJECTED)
    {
        rig->rejects++;
    }
    if (rig->out.lifecycle != S3_RUN_ACTIVE)
    {
        assert(!rig->out.steering.valid && rig->out.action == S3_NAV_NONE);
        assert((rig->out.events & ~(uint32_t)S3_EVENT_SENSOR_FAULT) == 0U);
    }
    if (rig->jitter)
    {
        step = (rig->now % 2U == 0U) ? step + 1001U : step - 1001U;
    }
    rig->now += step;
    rig->odometry.centre_m += rig->velocity * (double)step / 1000000.0;
}
static void tick(rig_t *rig, uint16_t barcode, uint16_t left, uint16_t right)
{
    prepare_frame(rig, barcode, left, right);
    process(rig, !rig->omit_odometry);
}
static void hold(rig_t *rig, uint32_t duration, uint16_t barcode, uint16_t left,
                 uint16_t right)
{
    uint32_t start = rig->now;
    do
    {
        tick(rig, barcode, left, right);
    } while ((uint32_t)(rig->now - start) < duration);
}
static void init_rig(rig_t *rig, uint32_t step, bool active)
{
    s3_navigation_config_t cfg = s3_navigation_defaults();
    *rig = (rig_t){0};
    rig->step = step;
    cfg.acquisition.enabled_mask = S3_ALL_SENSORS;
    assert(s3_navigation_init(&rig->nav, &cfg));
    if (active)
    {
        accept_calibrations(rig);
        tick(rig, 150U, 202U, 202U);
        assert(s3_navigation_start(&rig->nav, rig->now));
    }
}
static void scan(rig_t *rig, char letter, bool reverse)
{
    static const uint16_t patterns[26] = {
        0x109, 0x049, 0x148, 0x019, 0x118, 0x058, 0x00D, 0x10C, 0x04C,
        0x01C, 0x103, 0x043, 0x142, 0x013, 0x112, 0x052, 0x007, 0x106,
        0x046, 0x016, 0x181, 0x0C1, 0x1C0, 0x091, 0x190, 0x0D0};
    uint32_t runs[29];
    uint16_t symbols[3] = {0x094U, patterns[(unsigned int)(letter - 'A')],
                           0x094U};
    unsigned int c, i;
    for (c = 0U; c < 3U; c++)
    {
        for (i = 0U; i < 9U; i++)
        {
            runs[c * 10U + i] =
                (symbols[c] & (1U << (8U - i))) ? 120000U : 40000U;
        }
        if (c < 2U)
        {
            runs[c * 10U + 9U] = 40000U;
        }
    }
    hold(rig, 600000U, 150U, 202U, 202U);
    for (i = 0U; i < 29U; i++)
    {
        hold(rig, runs[reverse ? 28U - i : i], (i % 2U) ? 150U : 280U, 202U,
             202U);
    }
    hold(rig, 600000U, 150U, 202U, 202U);
}
static void test_readiness_and_updates(void)
{
    rig_t rig;
    s3_navigation_config_t cfg = s3_navigation_defaults();
    s3_calibration_collector_t c = calibration(), empty;
    s3_nav_action_t map[26] = {S3_NAV_RIGHT};
    unsigned int i;
    init_rig(&rig, 10000U, false);
    assert(rig.nav.lifecycle == S3_RUN_INACTIVE);
    tick(&rig, 150U, 202U, 202U);
    assert(rig.out.readiness_blockers == S3_READY_CALIBRATION);
    assert(!s3_navigation_start(&rig.nav, rig.now));
    assert(rig.nav.accepted_calibration == 0U);
    assert(s3_navigation_apply_calibration(&rig.nav, S3_SENSOR_LEFT, &c));
    assert(rig.nav.accepted_calibration == S3_SENSOR_BIT(S3_SENSOR_LEFT));
    s3_calibration_begin(&empty);
    assert(!s3_navigation_apply_calibration(&rig.nav, S3_SENSOR_LEFT, &empty));
    assert(rig.nav.accepted_calibration == S3_SENSOR_BIT(S3_SENSOR_LEFT));
    assert(rig.nav.config.calibration[1].white_raw == 150U);
    accept_calibrations(&rig);
    assert(!s3_navigation_start(
        &rig.nav, rig.now)); /* New calibration needs fresh inputs. */
    prepare_frame(&rig, 150U, 202U, 202U);
    process(&rig, false);
    assert(rig.out.readiness_blockers == S3_READY_ODOMETRY);
    assert(!s3_navigation_start(&rig.nav, rig.now));
    tick(&rig, 150U, 202U, 202U);
    assert(s3_navigation_readiness(&rig.nav, rig.now) == 0U);
    assert(!s3_navigation_start(&rig.nav, rig.now + 40000U));
    map[25] = S3_NAV_UTURN;
    assert(s3_navigation_set_mapping(&rig.nav, map));
    map[0] = S3_NAV_LEFT;
    map[25] = S3_NAV_ACTION_COUNT;
    assert(!s3_navigation_set_mapping(&rig.nav, map));
    assert(rig.nav.config.map[0] == S3_NAV_RIGHT &&
           rig.nav.config.map[25] == S3_NAV_UTURN);
    map[25] = (s3_nav_action_t)-1;
    assert(!s3_navigation_set_mapping(&rig.nav, map));
    map[25] = S3_NAV_NONE;
    assert(!s3_navigation_set_mapping(&rig.nav, 0));
    assert(s3_navigation_start(&rig.nav, rig.now));
    assert(!s3_navigation_start(&rig.nav, rig.now));
    assert(!s3_navigation_apply_calibration(&rig.nav, S3_SENSOR_LEFT, &c));
    assert(!s3_navigation_set_mapping(&rig.nav, map));
    assert(rig.nav.config.map[0] == S3_NAV_RIGHT);
    hold(&rig, 50000U, 150U, 202U, 202U);
    assert(rig.out.steering.valid && rig.acquired == 1U);
    assert(s3_navigation_end_run(&rig.nav));
    hold(&rig, 50000U, 150U, 202U, 202U);
    assert(rig.out.steering_preview.valid && !rig.out.steering.valid);
    assert(s3_navigation_set_mapping(&rig.nav, map));
    assert(s3_navigation_apply_calibration(&rig.nav, S3_SENSOR_LEFT, &c));
    /* Each disabled channel independently prevents start. */
    for (i = 0U; i < S3_SENSOR_COUNT; i++)
    {
        cfg.acquisition.enabled_mask =
            (uint8_t)(S3_ALL_SENSORS & ~S3_SENSOR_BIT(i));
        assert(s3_navigation_init(&rig.nav, &cfg));
        accept_calibrations(&rig);
        tick(&rig, 150U, 202U, 202U);
        assert(rig.out.readiness_blockers & S3_READY_CHANNELS);
        assert(!s3_navigation_start(&rig.nav, rig.now));
    }
    cfg = s3_navigation_defaults();
    assert(s3_navigation_init(&rig.nav, &cfg));
    tick(&rig, 150U, 202U, 202U);
    assert(rig.out.sensor[0].valid && !rig.out.sensor[1].valid &&
           rig.out.fault_mask == 0U);
    cfg.rearm_distance_m = NAN;
    assert(!s3_navigation_init(&rig.nav, &cfg));
    cfg = s3_navigation_defaults();
    cfg.fault_clear_us = 0U;
    assert(!s3_navigation_init(&rig.nav, &cfg));
    cfg = s3_navigation_defaults();
    cfg.map[0] = S3_NAV_ACTION_COUNT;
    assert(!s3_navigation_init(&rig.nav, &cfg));
}
static void test_faults(void)
{
    unsigned int kind;
    for (kind = 0U; kind < 8U; kind++)
    {
        rig_t rig;
        uint32_t reason;
        s3_calibration_collector_t c = calibration();
        init_rig(&rig, 10000U, true);
        hold(&rig, 50000U, 150U, 202U, 202U);
        if (kind == 1U)
        {
            rig.now += 40000U;
        }
        prepare_frame(&rig, 150U, 202U, 202U);
        switch (kind)
        {
            case 0U:
                rig.frame.sensor[1].valid = false;
                reason = S3_FAULT_SENSOR;
                break;
            case 1U:
                reason = S3_FAULT_TIMING;
                break;
            case 2U:
                rig.odometry.timestamp_us -= 40000U;
                reason = S3_FAULT_ODOMETRY;
                break;
            case 3U:
                rig.odometry.epoch++;
                reason = S3_FAULT_ODOMETRY_EPOCH;
                break;
            case 4U:
                rig.odometry.centre_m = NAN;
                reason = S3_FAULT_ODOMETRY;
                break;
            case 5U:
                reason = S3_FAULT_ODOMETRY;
                break;
            case 6U:
                rig.frame.sensor[2].timestamp_us -= 40000U;
                reason = S3_FAULT_SENSOR;
                break;
            default:
                rig.odometry.valid = false;
                reason = S3_FAULT_ODOMETRY;
                break;
        }
        process(&rig, kind != 5U);
        assert(rig.out.stop_requested &&
               rig.out.lifecycle == S3_RUN_FAULT_LATCHED);
        assert(rig.out.latched_faults & reason);
        assert(rig.faults == 1U && !rig.out.steering.valid);
        assert(!s3_navigation_reset_fault(&rig.nav, rig.now));
        assert(!s3_navigation_end_run(&rig.nav));
        assert(!s3_navigation_start(&rig.nav, rig.now));
        assert(!s3_navigation_apply_calibration(&rig.nav, S3_SENSOR_LEFT, &c));
        assert(!s3_navigation_set_mapping(&rig.nav, rig.nav.config.map));
        rig.odometry.centre_m = 0.0;
        hold(&rig, 50000U, 150U, 202U, 202U);
        assert(rig.out.stop_requested && rig.faults == 1U &&
               rig.out.fault_mask == 0U);
        assert(!s3_navigation_reset_fault(&rig.nav, rig.now));
        hold(&rig, 100000U, 150U, 202U, 202U);
        assert(rig.out.stop_requested && !rig.out.steering.valid);
        assert(!s3_navigation_reset_fault(
            &rig.nav, rig.now + 40000U)); /* Stale at reset call. */
        assert(s3_navigation_reset_fault(&rig.nav, rig.now));
        tick(&rig, 150U, 202U, 202U);
        assert(!rig.out.stop_requested && rig.out.latched_faults == 0U &&
               !rig.out.steering.valid);
        assert(s3_navigation_start(&rig.nav, rig.now));
        hold(&rig, 50000U, 150U, 202U, 202U);
        assert(rig.out.steering.valid && rig.faults == 1U);
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
        assert(!s3_navigation_reset_fault(&rig.nav, rig.now + 100000U));
        hold(&rig, 80000U, 150U, 202U, 202U);
        prepare_frame(&rig, 150U, 202U, 202U);
        rig.odometry.epoch++;
        process(&rig, true);
        hold(&rig, 50000U, 150U, 202U, 202U);
        assert(!s3_navigation_reset_fault(&rig.nav, rig.now));
    }
}
static void test_rearming_and_timing(void)
{
    unsigned int cadence, direction;
    for (cadence = 0U; cadence < 2U; cadence++)
    {
        for (direction = 0U; direction < 2U; direction++)
        {
            rig_t rig;
            double anchor;
            unsigned int i;
            init_rig(&rig, cadence ? 10000U : 5000U, true);
            rig.jitter = true;
            scan(&rig, 'A', direction != 0U);
            assert(rig.barcodes == 1U && rig.letter == 'A' &&
                   rig.action == S3_NAV_LEFT);
            assert(rig.last_reversed == (direction != 0U));
            assert(rig.out.rearming && rig.out.capture_distance.start_valid &&
                   rig.out.capture_distance.end_valid);
            assert(rig.out.capture_distance.start_m == 0.0 &&
                   rig.out.capture_distance.end_m == 0.0);
            assert(rig.nav.barcode.count == 0U);
            anchor = rig.nav.rearm_anchor_m;
            /* Oscillations accumulate travel, but never sufficient NET
             * displacement. */
            for (i = 0U; i < 40U; i++)
            {
                rig.odometry.centre_m = anchor + ((i % 2U) ? 0.015 : -0.015);
                tick(&rig, 150U, 202U, 202U);
                assert(rig.out.rearming);
            }
            scan(&rig, 'B', false);
            assert(rig.barcodes == 1U);
            rig.odometry.centre_m = anchor + (direction ? -0.021 : 0.021);
            hold(&rig, 100000U, 150U, 202U, 202U);
            assert(!rig.out.rearming);
            scan(&rig, 'Z', direction != 0U);
            assert(rig.barcodes == 2U && rig.letter == 'Z' &&
                   rig.action == S3_NAV_NONE);
            assert(rig.out.rearming);
            /* Bad odometry during rearming cannot turn it into time-only
             * rearming. */
            prepare_frame(&rig, 150U, 202U, 202U);
            process(&rig, false);
            assert(rig.out.stop_requested && rig.out.rearming);
        }
    }
    /* Moving synthetic input records actual endpoint distances. */
    {
        rig_t rig;
        init_rig(&rig, 10000U, true);
        rig.velocity = -0.1;
        scan(&rig, 'C', true);
        assert(rig.barcodes == 1U && rig.action == S3_NAV_STRAIGHT);
        assert(rig.out.capture_distance.end_m <
               rig.out.capture_distance.start_m);
        hold(&rig, 500000U, 150U, 202U, 202U);
        assert(!rig.out.rearming);
    }
    /* A malformed capture also requires clearance; it is not a critical fault.
     */
    {
        rig_t rig;
        init_rig(&rig, 10000U, true);
        hold(&rig, 500000U, 150U, 202U, 202U);
        hold(&rig, 1100000U, 280U, 202U, 202U);
        assert(rig.rejects == 1U && rig.out.rearming &&
               !rig.out.stop_requested);
        rig.odometry.centre_m = 0.025;
        hold(&rig, 100000U, 150U, 150U, 150U);
        assert(rig.out.rearming); /* Distance alone cannot clear absent line. */
        hold(&rig, 100000U, 280U, 202U, 202U);
        assert(rig.out.rearming); /* Or black barcode input. */
        hold(&rig, 100000U, 150U, 202U, 202U);
        assert(!rig.out.rearming);
        scan(&rig, 'B', false);
        assert(rig.barcodes == 1U && rig.action == S3_NAV_RIGHT);
    }
}
static void test_line_modes_and_diagnostics(void)
{
    rig_t rig;
    s3_nav_action_t map[26] = {0};
    init_rig(&rig, 10000U, true);
    hold(&rig, 250000U, 150U, 150U, 150U);
    assert(rig.lost == 1U && !rig.out.stop_requested &&
           !rig.out.steering.valid);
    hold(&rig, 50000U, 150U, 202U, 202U);
    assert(rig.acquired == 1U && rig.out.steering.valid);
    hold(&rig, 500000U, 150U, 202U, 202U);
    hold(&rig, 200000U, 280U, 150U, 150U);
    assert(rig.nav.barcode.collecting && rig.lost == 2U &&
           !rig.out.stop_requested);
    assert(s3_navigation_set_mode(&rig.nav, S3_MODE_SUPPRESSED));
    assert(!rig.nav.barcode.collecting);
    scan(&rig, 'A', false);
    hold(&rig, 150000U, 150U, 280U, 280U);
    assert(rig.barcodes == 0U && rig.junctions == 0U &&
           !rig.out.steering.valid);
    assert(s3_navigation_set_mode(&rig.nav, S3_MODE_TRACK));
    hold(&rig, 100000U, 150U, 202U, 202U);
    assert(rig.barcodes == 0U && rig.out.rearming);
    rig.odometry.centre_m = 0.025;
    hold(&rig, 100000U, 150U, 202U, 202U);
    scan(&rig, 'D', false);
    assert(rig.barcodes == 1U && rig.action == S3_NAV_UTURN);
    hold(&rig, 50000U, 150U, 280U, 280U);
    assert(rig.junctions == 1U && !rig.out.steering.valid);
    assert(s3_navigation_end_run(&rig.nav));
    map[25] = S3_NAV_LEFT;
    assert(s3_navigation_set_mapping(&rig.nav, map));
    tick(&rig, 150U, 202U, 202U);
    assert(s3_navigation_start(&rig.nav, rig.now));
    scan(&rig, 'Z', false);
    assert(rig.action == S3_NAV_LEFT);
    /* Inactive diagnostic frames need no odometry and cannot emit actions. */
    init_rig(&rig, 10000U, false);
    rig.omit_odometry = true;
    scan(&rig, 'A', false);
    assert(rig.barcodes == 0U && rig.out.diagnostic &&
           rig.out.action == S3_NAV_NONE);
    assert(rig.diagnostic_letters == 1U && !rig.out.odometry_available);
    assert(!rig.nav.capture_distance.start_valid &&
           !rig.nav.capture_distance.end_valid);
    /* Wraparound clocks remain supported through lifecycle and recovery. */
    init_rig(&rig, 5000U, false);
    rig.now = 0xFFFF0000U;
    accept_calibrations(&rig);
    tick(&rig, 150U, 202U, 202U);
    assert(s3_navigation_start(&rig.nav, rig.now));
    hold(&rig, 150000U, 150U, 202U, 202U);
    assert(rig.out.steering.valid && !rig.out.stop_requested);
}
static void test_clearance_persistence_and_epochs(void)
{
    rig_t rig;
    s3_navigation_config_t cfg = s3_navigation_defaults();
    s3_calibration_collector_t c = calibration();
    init_rig(&rig, 10000U, false);
    cfg.acquisition.enabled_mask = S3_ALL_SENSORS;
    cfg.rearm_stable_us = 60000U;
    cfg.rearm_distance_m = 0.050;
    cfg.fault_clear_us = 200000U;
    assert(s3_navigation_init(&rig.nav, &cfg));
    accept_calibrations(&rig);
    tick(&rig, 150U, 202U, 202U);
    assert(s3_navigation_start(&rig.nav, rig.now));
    scan(&rig, 'A', false);
    rig.odometry.centre_m = 0.025;
    hold(&rig, 100000U, 150U, 202U, 202U);
    assert(rig.out.rearming);     /* Configured clearance is 50 mm. */
    tick(&rig, 280U, 202U, 202U); /* Interrupt the white interval. */
    rig.odometry.centre_m = 0.060;
    hold(&rig, 50000U, 150U, 202U, 202U);
    assert(rig.out.rearming); /* Still short of configured 60 ms persistence. */
    tick(&rig, 280U, 202U, 202U);
    hold(&rig, 50000U, 150U, 202U, 202U);
    assert(rig.out.rearming);
    hold(&rig, 30000U, 150U, 202U, 202U);
    assert(!rig.out.rearming);
    scan(&rig, 'B', false);
    assert(rig.barcodes == 2U && rig.out.rearming);
    rig.odometry.epoch++;
    rig.odometry.centre_m = 10.0;
    tick(&rig, 150U, 202U, 202U);
    assert(rig.out.stop_requested && rig.out.rearming && rig.barcodes == 2U);
    assert(rig.out.latched_faults & S3_FAULT_ODOMETRY_EPOCH);
    hold(&rig, 120000U, 150U, 202U, 202U);
    assert(!s3_navigation_reset_fault(&rig.nav, rig.now));
    hold(&rig, 100000U, 150U, 202U, 202U);
    assert(s3_navigation_reset_fault(&rig.nav, rig.now));
    assert(s3_navigation_start(&rig.nav, rig.now));
    assert(s3_navigation_set_mode(&rig.nav, S3_MODE_SUPPRESSED));
    prepare_frame(&rig, 150U, 202U, 202U);
    rig.frame.sensor[0].valid = false;
    process(&rig, true);
    assert(rig.faults == 2U && rig.out.stop_requested);
    /* Faulted calibration updates cannot clear the latch. */
    assert(!s3_navigation_apply_calibration(&rig.nav, S3_SENSOR_BARCODE, &c));
}
void test_navigation(void)
{
    test_readiness_and_updates();
    test_faults();
    test_rearming_and_timing();
    test_line_modes_and_diagnostics();
    test_clearance_persistence_and_epochs();
}
