/** @file slope_link_main.c
 * @brief Execute the actual sampling/controller runtime with mocked IMU
 * transport.
 *
 * Existing runtime harness provides ADC, RTOS and motion boundaries.
 * Unsigned casts spell out the original modulo conversion.
 * The selected narrow integer operands fit in int.
 * Packet fields and fixture masks are interpreted as unsigned bits.
 */
#include "slope_link_main.h"
#include "../integration/runtime_main.h"
#include "../../app_program/controller/terr_link.h"
#include "../../app_program/subsystem4/buddy4.h"
#include "../../app_program/app_types.h"
#include <assert.h>
#include <stdio.h>

#define IR_CALIBRATION_SAMPLE_COUNT 12U
#define ADC_WARMUP_TICKS 12U
#define EXTERNAL_HOLD_CHECK_TICKS 5U
#define DUPLICATE_INVALIDATION_CHECKS 100U
#define DELIVERY_RETRY_CHECKS 3U

#define STALE_IMU_WAIT_US 160000U
#define RESET_CONFIRM_WAIT_US 510000U
#define COMMAND_WAIT_US 50000U
#define IR_CHANNEL_COUNT 3U
#define LIVE_CYCLES (200U / S3_SAMPLE_PERIOD_MS)

static terr_data_result_t g_imu;
static bool               gb_is_readable = true;
static bool               gb_is_reset_pending;
static bool               gb_should_reject_begin;
static bool               gb_is_event_pending;
static bool               gb_is_event_lost;
static uint16_t           g_run_id;
static uint16_t           g_count;
static uint16_t           g_sent_count;
static uint16_t           g_sent_height;
static unsigned           g_begins;
static unsigned           g_reports;
static unsigned           g_invalidations;
static ER                 g_hump_report_result = E_OK;
static unsigned           g_accepted_reports;
static uint16_t           g_accepted_count;
static uint16_t           g_accepted_height;
static UB                 g_faults =
    BUDDY1_FAULT_WIFI | BUDDY1_FAULT_STALL | BUDDY1_FAULT_NO_ECHO;
static terr_data_result_t g_queued;

static unsigned                  g_tilts;
static telemetry_codec_terrain_t g_last_terrain;
static int                       g_tilt_pitch;
static int                       g_tilt_roll;
static int                       g_tilt_state;

static void fresh (void);
static void step (void);
static void command (s3_integration_kind_t kind);
static void reset_fault (void);
static void begin (void);
static void ack (void);
static void peak_sample (uint16_t humps, float32_t height);
static void test_hump_report_retry (void);
static void test_pickup_and_overturn_stop (void);

/** @brief Return false while unreadable or awaiting reset acknowledgement.
 * Otherwise copy the fixture sample, run ID and hump count outside
 * the sampling lock.
 */
bool buddy4_read (terr_data_result_t * p_out, uint16_t * p_run,
                  uint16_t * p_humps)
{
    assert(!runtime_main_view().lock_depth);

    if ((!gb_is_readable) || gb_is_reset_pending)
    {
        return false;
    }

    *p_out   = g_imu;
    *p_run   = g_run_id;
    *p_humps = g_count;
    return true;
}

/** @brief Reject an injected start failure without writing the token. On
 * success, increment the run ID and mark reset acknowledgement
 * pending.
 */
bool buddy4_begin_run (uint16_t * p_token)
{
    assert(!runtime_main_view().lock_depth);

    if (gb_should_reject_begin)
    {
        return false;
    }

    g_begins++;
    *p_token            = ++g_run_id;
    gb_is_reset_pending = true;
    return true;
}

/** @brief Pop a pending terrain event and its loss flag. When empty, return
 * false and leave both outputs untouched.
 */
bool buddy4_event (terr_data_result_t * p_out, bool * pb_did_lose)
{
    assert(!runtime_main_view().lock_depth);

    if (!gb_is_event_pending)
    {
        return false;
    }

    *p_out              = g_queued;
    *pb_did_lose        = gb_is_event_lost;
    gb_is_event_pending = false;
    return true;
}

/** @brief Require an IMU-only fault update and apply the changed bit to the
 * fixture fault mask.
 */
ER buddy1_update_faults (UB changed, UB active)
{
    assert(BUDDY1_FAULT_IMU == changed &&
           !((unsigned)active & ~(unsigned)changed));
    g_faults = ((unsigned)g_faults & ~(unsigned)changed) | (unsigned)active;
    return E_OK;
}

/** @brief Record the attempted count and height in millimetres. Update
 * accepted values only on a successful publication.
 */
ER buddy1_report_hump (UH humps, UH height_mm)
{
    g_reports++;
    g_sent_count  = humps;
    g_sent_height = height_mm;

    if (E_OK == g_hump_report_result)
    {
        g_accepted_reports++;
        g_accepted_count  = humps;
        g_accepted_height = height_mm;
    }

    return g_hump_report_result;
}

/** @brief Check tilt and hump field validity and retain the reported terrain
 * state and tilt for the fixture.
 */
ER buddy1_report_terrain (const telemetry_codec_terrain_t * p_time)
{
    assert(p_time && p_time->state < 8U &&
           (p_time->tilt_valid ||
            (0 == p_time->pitch_tenths_deg && 0 == p_time->roll_tenths_deg)));
    assert((p_time->on_hump || 0 == (int)p_time->climb_mm) &&
           (p_time->peak_valid || 0 == (int)p_time->peak_mm));
    g_tilts++;
    g_last_terrain = *p_time;
    g_tilt_pitch   = p_time->tilt_valid ? p_time->pitch_tenths_deg : 9999;
    g_tilt_roll    = p_time->tilt_valid ? p_time->roll_tenths_deg : 9999;
    g_tilt_state   = p_time->state;
    return E_OK;
}

/** @brief Require a hump-topic invalidation, count it and report success.
 */
ER buddy1_invalidate (UB topic)
{
    assert(BUDDY1_TOPIC_HUMP == (int)topic);
    g_invalidations++;
    return E_OK;
}

/** @brief Drive the controller through simulated IMU runs and unsafe events.
 * Check hump publication, reset acknowledgement and stop precedence.
 */
int main (void)
{
    ir_calibration_collector_t cal;
    s3_integration_status_t    status;
    terr_link_init();
    assert(s3_integration_init() == E_OK);
    ir_calibration_begin(&cal);

    for (unsigned sample_index = 0; sample_index < IR_CALIBRATION_SAMPLE_COUNT;
         sample_index++)
    {
        assert(ir_calibration_add(&cal, IR_SURFACE_WHITE, 300));
        assert(ir_calibration_add(&cal, IR_SURFACE_BLACK, 2500));
    }

    for (unsigned sensor_index = 0; sensor_index < IR_CHANNEL_COUNT;
         sensor_index++)
    {
        assert(runtime_main_apply_calibration(
            (ir_acquisition_sensor_id_t)sensor_index, &cal));
    }

    runtime_main_set_adc(1400);

    for (unsigned sample_index = 0; sample_index < ADC_WARMUP_TICKS;
         sample_index++)
    {
        step();
    }

    g_imu.flags        = 0;
    g_imu.timestamp_us = runtime_main_view().clock_us + S3_SAMPLE_PERIOD_US;
    runtime_main_cycle();
    assert(!runtime_main_start_now());
    assert(0U == g_begins && 0U == runtime_main_view().freeze_calls);
    assert(s3_integration_submit(
        &(s3_integration_request_t){.kind = S3_CONTROL_START}));
    fresh();
    g_imu.safety = TERRAIN_SAFETY_OVERTURNED;
    runtime_main_cycle();
    assert((unsigned)S3_DRIVE_OFF == runtime_main_view().p_drive->state &&
           0U == g_begins && 0U == runtime_main_view().freeze_calls &&
           0U == runtime_main_view().command_id);
    step();
    gb_should_reject_begin = true;
    command(S3_CONTROL_START);
    assert((unsigned)S3_DRIVE_OFF == runtime_main_view().p_drive->state &&
           1U == runtime_main_view().cancel_calls && 0U == g_begins);
    gb_should_reject_begin = false;
    begin();

    for (unsigned sample_index = 0; sample_index < EXTERNAL_HOLD_CHECK_TICKS;
         sample_index++)
    {
        step();
        assert(runtime_main_view().p_drive->b_has_external_hold &&
               0U == runtime_main_view().command_id);
    }

    ack();
    /* Cumulative snapshot includes missed delivery of earlier hump events. */
    fresh();
    g_imu.flags |= TERRAIN_PEAK_VALID;
    g_imu.peak_m = 0.123f;
    g_count      = 3;
    runtime_main_cycle();
    terr_link_report();
    assert(1U == g_reports && 3 == (int)g_sent_count &&
           123 == (int)g_sent_height);
    terr_link_report();
    assert(1U == g_reports);
    test_hump_report_retry();
    test_pickup_and_overturn_stop();
    /* An impact event survives even when the latest sample is normal. */
    g_queued            = (terr_data_result_t){.b_has_impact = true};
    gb_is_event_pending = true;
    step();
    assert((unsigned)S3_DRIVE_FAULT == runtime_main_view().p_drive->state &&
           (unsigned)S3_DRIVE_TERRAIN_FAULT ==
               runtime_main_view().p_drive->reason);
    assert(s3_integration_status(&status) &&
           (unsigned)S3_DRIVE_TERRAIN_FAULT == status.reason);
    assert((g_faults & (BUDDY1_FAULT_WIFI | BUDDY1_FAULT_STALL |
                        BUDDY1_FAULT_NO_ECHO | BUDDY1_FAULT_IMU)) == g_faults);
    assert(g_faults & BUDDY1_FAULT_IMU);
    command(S3_CONTROL_START);
    assert((unsigned)S3_DRIVE_FAULT == runtime_main_view().p_drive->state);
    reset_fault();
    begin();
    ack();
    assert(g_invalidations > 0U && !(g_faults & BUDDY1_FAULT_IMU));
    {
        unsigned before = g_invalidations;

        for (unsigned report_index = 0;
             report_index < DUPLICATE_INVALIDATION_CHECKS; ++report_index)
        {
            terr_link_report();
        }

        assert(g_invalidations ==
               before); /* Skip a redundant lock and write while unavailable. */
    }
    assert(g_faults ==
           (BUDDY1_FAULT_WIFI | BUDDY1_FAULT_STALL | BUDDY1_FAULT_NO_ECHO));
    /* One missed nonblocking read uses a still-fresh cached sample. */
    gb_is_readable = false;
    runtime_main_cycle();
    assert((unsigned)S3_DRIVE_FOLLOW == runtime_main_view().p_drive->state);

    for (unsigned sample_index = 0;
         sample_index < (STALE_IMU_WAIT_US / S3_SAMPLE_PERIOD_US);
         sample_index++)
    {
        runtime_main_cycle();
    }

    assert((unsigned)S3_DRIVE_FAULT == runtime_main_view().p_drive->state);
    gb_is_readable = true;
    reset_fault();
    begin();
    ack();
    g_queued            = (terr_data_result_t){0};
    gb_is_event_pending = gb_is_event_lost = true;
    step();
    assert((unsigned)S3_DRIVE_FAULT == runtime_main_view().p_drive->state);
    gb_is_event_lost = false;
    reset_fault();
    begin();

    for (unsigned sample_index = 0;
         sample_index < (RESET_CONFIRM_WAIT_US / S3_SAMPLE_PERIOD_US);
         sample_index++)
    {
        step();
    }

    assert((unsigned)S3_DRIVE_FAULT == runtime_main_view().p_drive->state);
    gb_is_reset_pending = false;
    reset_fault();
    begin();
    command(S3_CONTROL_STOP);
    assert((unsigned)S3_DRIVE_OFF == runtime_main_view().p_drive->state &&
           !runtime_main_view().p_drive->b_has_external_hold &&
           0U == runtime_main_view().command_id);
    gb_is_reset_pending = false;
    step();
    begin();
    ack();
    g_run_id++;
    step();
    assert((unsigned)S3_DRIVE_FAULT == runtime_main_view().p_drive->state);
    /* Live tilt: one report per 200 ms, run or no run; stale or
     * invalid IMU data is "unavailable", never 0 degrees. */
    {
        unsigned before = g_tilts;
        fresh();
        g_imu.pitch_deg = 12.34f;
        g_imu.roll_deg  = -5.06f;
        g_imu.state     = TERRAIN_CLIMBING;
        runtime_main_cycle();

        for (unsigned cycle_index = 0; cycle_index < (2U * LIVE_CYCLES);
             ++cycle_index)
        {
            terr_link_report();
        }

        assert(g_tilts == before + 2U && 123 == g_tilt_pitch &&
               g_tilt_roll == -51 && TERRAIN_CLIMBING == g_tilt_state);
        g_imu.flags &= ~TERRAIN_TILT_VALID;
        runtime_main_cycle();

        for (unsigned cycle_index = 0; cycle_index < LIVE_CYCLES; ++cycle_index)
        {
            terr_link_report();
        }

        assert(g_tilts == before + 3U && 9999 == g_tilt_pitch &&
               TERRAIN_CLIMBING == g_tilt_state);
        fresh();
        g_imu.pitch_deg = 95.0f;
        runtime_main_cycle();

        for (unsigned cycle_index = 0; cycle_index < LIVE_CYCLES; ++cycle_index)
        {
            terr_link_report();
        }

        assert(
            g_tilts == before + 4U &&
            9999 ==
                g_tilt_pitch); /* The sample is outside the accepted range. */
        fresh();
        g_imu.pitch_deg = -90.0f;
        g_imu.roll_deg  = 90.0f;
        runtime_main_cycle();

        for (unsigned cycle_index = 0; cycle_index < LIVE_CYCLES; ++cycle_index)
        {
            terr_link_report();
        }

        assert(g_tilt_pitch == -900 && 900 == g_tilt_roll);
        gb_is_readable = false;

        for (unsigned cycle_index = 0; cycle_index < LIVE_CYCLES; ++cycle_index)
        {
            runtime_main_cycle();
        }

        gb_is_readable = true;

        for (unsigned cycle_index = 0; cycle_index < LIVE_CYCLES; ++cycle_index)
        {
            terr_link_report();
        }

        assert(9999 == g_tilt_pitch && TERRAIN_UNKNOWN == g_tilt_state &&
               !g_last_terrain.on_hump); /* stale */
        /* Live climb height and this run's highest hump, with or without a run.
         */
        fresh();
        g_imu.b_is_hump_active = true;
        g_imu.hump_height_m    = 0.0234f;
        g_imu.flags           |= TERRAIN_PEAK_VALID;
        g_imu.peak_m           = 0.0411f;
        runtime_main_cycle();

        for (unsigned cycle_index = 0; cycle_index < LIVE_CYCLES; ++cycle_index)
        {
            terr_link_report();
        }

        assert(g_last_terrain.on_hump && 23U == g_last_terrain.climb_mm &&
               g_last_terrain.peak_valid && 41U == g_last_terrain.peak_mm);
        fresh();
        runtime_main_cycle();

        for (unsigned cycle_index = 0; cycle_index < LIVE_CYCLES; ++cycle_index)
        {
            terr_link_report();
        }

        assert(!g_last_terrain.on_hump && 0U == g_last_terrain.climb_mm &&
               !g_last_terrain.peak_valid);
        fresh();
        g_imu.flags |= TERRAIN_PEAK_VALID;
        g_imu.peak_m = 0.7f;
        runtime_main_cycle(); /* The sample is outside the accepted range. */
        for (unsigned cycle_index = 0; cycle_index < LIVE_CYCLES; ++cycle_index)
        {
            terr_link_report();
        }

        assert(!g_last_terrain.peak_valid && 0U == g_last_terrain.peak_mm);
        /* A one-sample impact between reports is still reported. */
        g_queued            = (terr_data_result_t){.b_has_impact = true};
        gb_is_event_pending = true;
        fresh();
        runtime_main_cycle();
        fresh();
        runtime_main_cycle();

        for (unsigned cycle_index = 0; cycle_index < LIVE_CYCLES; ++cycle_index)
        {
            terr_link_report();
        }

        assert(TERRAIN_IMPACT == g_tilt_state);

        for (unsigned cycle_index = 0; cycle_index < LIVE_CYCLES; ++cycle_index)
        {
            terr_link_report();
        }

        assert(TERRAIN_STATIONARY == g_tilt_state);
    }
    /* Align to a report, then advance the actual sampling clock: faster
     * ADC sampling must not double the live terrain traffic to 10 Hz. */
    {
        unsigned before = g_tilts;

        for (unsigned cycle_index = 0;
             (cycle_index < LIVE_CYCLES) && (g_tilts == before); ++cycle_index)
        {
            step();
        }

        assert(g_tilts == before + 1U);
        uint32_t started = runtime_main_view().clock_us;

        for (unsigned cycle_index = 0; cycle_index < (LIVE_CYCLES / 2U);
             ++cycle_index)
        {
            step();
        }

        assert(g_tilts == before + 1U &&
               runtime_main_view().clock_us - started == 100000U);

        for (unsigned cycle_index = 0;
             cycle_index < (LIVE_CYCLES - (LIVE_CYCLES / 2U) - 1U);
             ++cycle_index)
        {
            step();
        }

        assert(g_tilts == before + 1U);
        step();
        assert(g_tilts == before + 2U &&
               runtime_main_view().clock_us - started == 200000U);
    }
    puts("PASS: live terrain every 200 ms: tilt "
         "valid/invalid/out-of-range/stale, state, climb height, run peak, "
         "latched impact");
    puts("PASS: hump reports retry repeated timeouts with latest "
         "count/height, stop retrying on acceptance, survive STOP and "
         "discard pending prior-run data on START reset");
    puts("PASS: pickup and overturn brake before further velocity commands "
         "and require safe IMU data, RESET and START");
    puts("PASS: IMU START/reset acknowledgement and timeout, STOP priority, "
         "stale data, lost/impact events, latched reset, cumulative humps "
         "and owned fault bit");
    return 0;
}

/** @brief Prepare a calibrated stationary IMU sample timestamped one
 * integration period ahead of the current clock.
 */
static void fresh (void)
{
    g_imu = (terr_data_result_t){
        .timestamp_us = runtime_main_view().clock_us + S3_SAMPLE_PERIOD_US,
        .flags        = TERRAIN_CALIBRATED | TERRAIN_TILT_VALID,
        .state        = TERRAIN_STATIONARY};
}

/** @brief Supply a fresh sample, execute one sampling cycle and service
 * terrain reports.
 */
static void step (void)
{
    fresh();
    runtime_main_cycle();
    terr_link_report();
}

/** @brief Submit the supplied integration control command and advance the
 * terrain fixture one step.
 */
static void command (s3_integration_kind_t kind)
{
    s3_integration_request_t request = {.kind = kind};
    assert(s3_integration_submit(&request));
    step();
}

/** @brief Request reset and check that the drive reaches its off state.
 */
static void reset_fault (void)
{
    command(S3_CONTROL_RESET);
    step();
    assert((unsigned)S3_DRIVE_OFF == runtime_main_view().p_drive->state);
}

/** @brief Request start and check that following begins with the new terrain
 * run still awaiting acknowledgement.
 */
static void begin (void)
{
    command(S3_CONTROL_START);
    assert((unsigned)S3_DRIVE_FOLLOW == runtime_main_view().p_drive->state &&
           gb_is_reset_pending);
    assert(0U == runtime_main_view()
                     .command_id); /* START has not requested motor velocity. */
}

/** @brief Acknowledge the pending terrain reset, clear the hump count and
 * step until velocity control resumes.
 */
static void ack (void)
{
    gb_is_reset_pending = false;
    g_count             = 0;
    step();

    /* Every command ID fits int before comparing the zero sentinel. */
    for (unsigned sample_index = 0;
         (sample_index < (COMMAND_WAIT_US / S3_SAMPLE_PERIOD_US)) &&
         (0 == (int)runtime_main_view().command_id);
         sample_index++)
    {
        step();
    }

    assert((unsigned)S3_DRIVE_FOLLOW == runtime_main_view().p_drive->state &&
           !runtime_main_view().p_drive->b_has_external_hold &&
           1U == runtime_main_view().command_id);
}

/** @brief Supply a valid peak in metres and the requested run count, then
 * process and report the sample.
 */
static void peak_sample (uint16_t humps, float32_t height)
{
    fresh();
    g_imu.flags |= TERRAIN_PEAK_VALID;
    g_imu.peak_m = height;
    g_count      = humps;
    runtime_main_cycle();
    terr_link_report();
}

/** @brief Inject a publication timeout and check retries, accepted height in
 * millimetres and count ownership across runs.
 */
static void test_hump_report_retry (void)
{
    unsigned attempts    = g_reports;
    unsigned accepted    = g_accepted_reports;
    g_hump_report_result = E_TMOUT;
    peak_sample(4, 0.130f);
    assert(g_reports == attempts + 1U && g_accepted_reports == accepted);
    assert(4U == g_sent_count && 130U == g_sent_height);
    /* A rejected nonblocking write is not delivered, including repeated
     * timeouts with no new IMU sample or changed count to trigger a retry. */
    for (unsigned report_index = 0; report_index < DELIVERY_RETRY_CHECKS;
         report_index++)
    {
        terr_link_report();
        assert(g_reports == attempts + 2U + report_index &&
               g_accepted_reports == accepted);
        assert(4U == g_sent_count && 130U == g_sent_height);
    }

    peak_sample(5, 0.140f);
    assert(g_reports == attempts + 5U && 5U == g_sent_count &&
           140U == g_sent_height);
    /* Only the latest cumulative count/height is offered when space returns. */
    g_hump_report_result = E_OK;
    terr_link_report();
    assert(g_reports == attempts + 6U && g_accepted_reports == accepted + 1U);
    assert(5U == g_accepted_count && 140U == g_accepted_height);

    for (unsigned report_index = 0; report_index < DELIVERY_RETRY_CHECKS;
         report_index++)
    {
        terr_link_report();
    }

    assert(g_reports ==
           attempts + 6U); /* Successful delivery clears the retry. */

    g_hump_report_result = E_TMOUT;
    peak_sample(6, 0.160f);
    attempts = g_reports;
    accepted = g_accepted_reports;
    assert(s3_integration_submit(
        &(s3_integration_request_t){.kind = S3_CONTROL_STOP}));
    peak_sample(6, 0.160f);
    assert((unsigned)S3_DRIVE_OFF == runtime_main_view().p_drive->state &&
           0U == runtime_main_view().command_id);
    assert(g_reports == attempts + 1U && g_accepted_reports == accepted);
    g_hump_report_result = E_OK;
    terr_link_report();
    assert(g_reports == attempts + 2U && g_accepted_reports == accepted + 1U);
    assert(6U == g_accepted_count && 160U == g_accepted_height);
    terr_link_report();
    assert(g_reports == attempts + 2U);

    /* START requests a new-run reset: a blocked old run must not leak into
     * that run while waiting for acknowledgement or after its empty snapshot.
     */
    g_hump_report_result = E_TMOUT;
    peak_sample(7, 0.190f);
    attempts = g_reports;
    accepted = g_accepted_reports;
    begin();
    g_hump_report_result = E_OK;

    for (unsigned report_index = 0; report_index < DELIVERY_RETRY_CHECKS;
         report_index++)
    {
        terr_link_report();
    }

    assert(g_reports == attempts && g_accepted_reports == accepted);
    ack();
    assert(g_reports == attempts && g_accepted_reports == accepted);
    peak_sample(1, 0.009f);
    assert(g_reports == attempts + 1U && g_accepted_reports == accepted + 1U);
    assert(1U == g_accepted_count && 9U == g_accepted_height);
}

/** @brief Exercise pickup and overturn handling and verify that motion stays
 * stopped until reset and a fresh start.
 */
static void test_pickup_and_overturn_stop (void)
{
    unsigned previous_stops      = runtime_main_view().stops;
    unsigned previous_velocities = runtime_main_view().velocity_requests;
    fresh();
    g_imu.safety = TERRAIN_SAFETY_LIFTED;
    runtime_main_cycle();
    terr_link_report();
    assert((unsigned)S3_DRIVE_FAULT == runtime_main_view().p_drive->state &&
           (unsigned)S3_DRIVE_TERRAIN_FAULT ==
               runtime_main_view().p_drive->reason);
    assert(runtime_main_view().stops > previous_stops &&
           0U == runtime_main_view().command_id &&
           runtime_main_view().velocity_requests == previous_velocities);
    assert(g_faults & BUDDY1_FAULT_IMU);

    /* While still unsafe, RESET cannot clear the fault. Setting the car down
     * also cannot restart the run by itself or by sending START alone. */
    assert(s3_integration_submit(
        &(s3_integration_request_t){.kind = S3_CONTROL_RESET}));
    fresh();
    g_imu.safety = TERRAIN_SAFETY_LIFTED;
    runtime_main_cycle();
    assert((unsigned)S3_DRIVE_FAULT == runtime_main_view().p_drive->state &&
           s3_drive_reset_result(runtime_main_view().p_drive) !=
               (unsigned)S3_RESET_PENDING);
    step();
    command(S3_CONTROL_START);
    assert((unsigned)S3_DRIVE_FAULT == runtime_main_view().p_drive->state &&
           0U == runtime_main_view().command_id);
    assert(runtime_main_view().velocity_requests == previous_velocities);

    /* A new unsafe sample during the motor reset handshake aborts it. */
    command(S3_CONTROL_RESET);
    assert(s3_drive_reset_result(runtime_main_view().p_drive) ==
           (unsigned)S3_RESET_PENDING);
    fresh();
    g_imu.safety = TERRAIN_SAFETY_LIFTED;
    runtime_main_cycle();
    assert((unsigned)S3_DRIVE_FAULT == runtime_main_view().p_drive->state &&
           s3_drive_reset_result(runtime_main_view().p_drive) ==
               (unsigned)S3_RESET_FAILED);
    reset_fault();
    assert(0U == runtime_main_view().command_id);
    begin();
    ack();

    /* Even if the current snapshot is already clear, an observed safety
     * event stops the drive before the next velocity request. */
    previous_stops      = runtime_main_view().stops;
    previous_velocities = runtime_main_view().velocity_requests;

    /* clang-format off */
    g_queued            =
        (terr_data_result_t){.safety = TERRAIN_SAFETY_OVERTURNED};
    gb_is_event_pending = true;
    /* clang-format on */
    step();
    assert((unsigned)S3_DRIVE_FAULT == runtime_main_view().p_drive->state &&
           (unsigned)S3_DRIVE_TERRAIN_FAULT ==
               runtime_main_view().p_drive->reason);
    assert(runtime_main_view().stops > previous_stops &&
           0U == runtime_main_view().command_id &&
           runtime_main_view().velocity_requests == previous_velocities);
    reset_fault();
    begin();
    ack();
}

/* End of file. */

