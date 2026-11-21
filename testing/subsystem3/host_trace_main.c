/** @file host_trace_main.c
 * @brief Observe navigation and barcode semantic fields for differential
 * harness comparisons.
 */
/* The SAME harness is compiled against saved original and revised modules.
 * Compare semantic fields and exact float bits, never padded C struct bytes. */
#include "line_navigation.h"
#include "../../app_program/app_types.h"
#include <assert.h>
#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include "host_trace_main.h"
#include "test_core_main.h"

#define MAX_STREAM_SAMPLE_COUNT 65535U

static unsigned trace_rows;

/** @brief Copy a binary32 value into a word for exact trace output.
 */
static uint32_t float_bits (float32_t value)
{
    uint32_t bits;
    memcpy(&bits, &value, sizeof(bits));
    return bits;
}

/** @brief Print a supplied result's letter code, orientation and timestamp.
 * A null result emits no fields.
 */
static void barcode_fields (const bc_decoder_result_t * p_value)
{
    if (p_value)
    {
        printf(" %u %u %" PRIu32, (uint8_t)p_value->letter,
               (unsigned)p_value->b_is_reversed, p_value->timestamp_us);
    }
}

/** @brief Emit one navigation result and the exact steering float bits. */
void host_trace_navigation (bool                             b_did_succeed,
                            const line_navigation_output_t * p_out)
{
    printf("N %u %u", trace_rows++, (unsigned)b_did_succeed);
    if (p_out)
    {
        printf(" %" PRIu32 " %" PRIu32 " %" PRIu32 " %u", p_out->timestamp_us,
               p_out->sample_gap_us, p_out->events,
               (unsigned)p_out->fault_mask);
        for (unsigned sensor_index = 0;
             sensor_index < (unsigned int)S3_SENSOR_COUNT; ++sensor_index)
        {
            printf(" %u %u %u",
                   (unsigned)p_out->sensor[sensor_index].b_is_valid,
                   (unsigned)p_out->sensor[sensor_index].darkness,
                   (unsigned)p_out->sensor[sensor_index].surface);
        }
        printf(" %u %u %u %d %u %u %u %u %08" PRIx32 " %08" PRIx32 " %u",
               (unsigned)p_out->line.b_is_valid,
               (unsigned)p_out->line.b_is_line_present,
               (unsigned)p_out->line.b_has_junction_candidate,
               (int)p_out->line.error, (unsigned)p_out->line.b_has_lost_event,
               (unsigned)p_out->line.b_has_reacquired_event,
               (unsigned)p_out->line.b_has_junction_event,
               (unsigned)p_out->steering.b_is_valid,
               float_bits(p_out->steering.left_mps),
               float_bits(p_out->steering.right_mps),
               (unsigned)p_out->barcode_status);
        barcode_fields(&p_out->barcode);
        printf(" %u", (unsigned)p_out->action);
    }
    putchar('\n');
}

/** @brief Emit the post-update status, decoder state and barcode fields. */
void host_trace_barcode (bc_decoder_status_t  status,
                         const bc_decoder_t * p_decoder, uint32_t now,
                         bool b_is_valid, bool b_is_black,
                         const bc_decoder_result_t * p_out)
{
    printf("B %u %" PRIu32 " %u %u %u", trace_rows++, now, (unsigned)b_is_valid,
           (unsigned)b_is_black, (unsigned)status);
    barcode_fields(p_out);
    if (p_decoder)
    {
        printf(" %u %u %u %u %u %u", (unsigned)p_decoder->count,
               (unsigned)p_decoder->b_is_configured,
               (unsigned)p_decoder->b_has_sample,
               (unsigned)p_decoder->b_is_black,
               (unsigned)p_decoder->b_is_collecting,
               (unsigned)p_decoder->b_is_pending);
    }
    putchar('\n');
}

/** @brief Emit the result of decoding one array of measured run widths. */
void host_trace_runs (bool b_did_succeed, unsigned count,
                      const bc_decoder_result_t * p_out)
{
    printf("R %u %u %u", trace_rows++, count, (unsigned)b_did_succeed);
    barcode_fields(p_out);
    putchar('\n');
}

typedef struct
{
    uint16_t raw;
    int32_t  error;
    uint32_t time;
} raw_source_t;

/** @brief Write the fixture ADC count and return the configured error
 * independently of the channel.
 */
static int32_t raw_read (void * p_context, uint8_t channel, uint16_t * p_raw)
{
    raw_source_t * p_source = p_context;
    (void)channel;
    *p_raw = p_source->raw;
    return p_source->error;
}

/** @brief Advance the timestamp source by one and return it.
 */
static uint32_t raw_clock (void * p_context)
{
    raw_source_t * p_source = p_context;
    return ++p_source->time;
}

/** @brief Exercise all 65,536 candidate raw counts and verify the
 * acquisition boundary and propagated error.
 */
static void all_adc_values (void)
{
    ir_acquisition_config_t config = {{0, 1, 2}, 1};
    raw_source_t            source = {0};
    ir_acquisition_frame_t  frame;
    for (uint32_t raw = 0; raw <= (uint32_t)UINT16_MAX; ++raw)
    {
        source.raw = (uint16_t)raw;
        assert(ir_acquisition_capture(&config, raw_read, raw_clock, &source,
                                      &frame));
        assert(frame.sensor[0].b_is_valid == (raw <= 4095U));
        assert(frame.sensor[0].raw == (raw <= 4095U ? raw : 0U));
        assert(frame.sensor[0].error == (raw <= 4095U ? 0 : -1));
    }
    source.raw   = 4095;
    source.error = -1234567;
    assert(
        ir_acquisition_capture(&config, raw_read, raw_clock, &source, &frame));
    assert(!frame.sensor[0].b_is_valid && 0 == (int)frame.sensor[0].raw);
    assert(frame.sensor[0].error ==
           -1234567); /* Preserve the full diagnostic error. */
    puts("ADC_ALL 65536 values: original invalid-input rejection preserved");
}

/** @brief Check the calibration collector's capacity boundary and emit the
 * retained summary and exact average bits.
 */
static void calibration_limit (void)
{
    ir_calibration_collector_t   collector;
    ir_reflectance_calibration_t active = {150, 280, 400, 600};
    ir_calibration_begin(&collector);
    for (unsigned sample_index = 0; sample_index < MAX_STREAM_SAMPLE_COUNT;
         ++sample_index)
    {
        assert(ir_calibration_add(&collector, IR_SURFACE_WHITE, 4095));
        assert(ir_calibration_add(&collector, IR_SURFACE_BLACK, 0));
    }
    assert(65535U == collector.white.count);
    assert(268365825U == collector.white.sum);
    assert(65535U == collector.black.count && 0U == collector.black.sum);
    assert(!ir_calibration_add(&collector, IR_SURFACE_WHITE, 4095));
    assert(65535U == collector.white.count &&
           268365825U == collector.white.sum);
    assert(ir_calibration_finish(&collector, &active));
    assert(4095U == active.white_raw && 0U == active.black_raw);
    ir_calibration_summary_t summary = ir_calibration_summary(&collector.white);
    assert(summary.b_is_available && 4095.0f == summary.average);
    printf("CAL_MAX %u %" PRIu32 " %u %u %08" PRIx32 "\n",
           (unsigned)summary.count, (uint32_t)collector.white.sum,
           (unsigned)summary.minimum, (unsigned)summary.maximum,
           float_bits(summary.average));
}

/** @brief Exercise maximum accepted navigation configuration values and
 * verify rejection of out-of-range durations.
 * @details Drive synthetic sample timestamps across rollover at the
 * largest allowed elapsed interval. Decode extended run widths with the
 * resulting configuration. Check the first rejected duration on each
 * half-range and quiet-zone-product boundary.
 */
static void large_configuration_limits (void)
{
    line_navigation_config_t config = line_navigation_defaults();
    line_navigation_t        navigation;
    line_navigation_output_t output;
    ir_acquisition_frame_t   frame;
    uint32_t                 runs[S3_BARCODE_RUNS];
    bc_decoder_result_t      result;
    config.acquisition.enabled_mask = S3_ALL_SENSORS;
    config.max_sample_age_us        = 0x7fffffffU;
    config.line.lost_us             = 0x7fffffffU;
    config.line.reacquire_us        = 0x7ffffffeU;
    config.line.junction_us         = 0x7ffffffdU;
    config.line.rearm_us            = 0x7ffffffcU;
    config.line.max_gap_us          = 0x7fffffffU;
    config.line.signal_floor        = 2000;
    config.line.junction_dark       = 1000;
    config.barcode.max_gap_us       = 42949672U;
    config.barcode.min_narrow_us    = 1000000U;
    config.barcode.max_run_us       = 42949672U;
    config.barcode.max_frame_us     = 0x7fffffffU;
    config.barcode.quiet_modules    = 50;
    assert(line_navigation_init(&navigation, &config));
    assert(navigation.config.max_sample_age_us == config.max_sample_age_us);
    assert(navigation.line.config.lost_us == config.line.lost_us);
    assert(navigation.line.config.reacquire_us == config.line.reacquire_us);
    assert(navigation.line.config.junction_us == config.line.junction_us);
    assert(navigation.line.config.rearm_us == config.line.rearm_us);
    assert(navigation.barcode.config.max_frame_us ==
           config.barcode.max_frame_us);
    assert(navigation.barcode.config.max_run_us == config.barcode.max_run_us);
    assert(navigation.barcode.config.max_gap_us == config.barcode.max_gap_us);
    assert(50U == navigation.barcode.config.quiet_modules);
    frame = (ir_acquisition_frame_t){.timestamp_us = 0x90000000U};
    for (unsigned sensor_index = 0U;
         sensor_index < (unsigned int)S3_SENSOR_COUNT; ++sensor_index)
    {
        frame.sensor[sensor_index] = (ir_acquisition_sample_t){
            frame.timestamp_us, 0U == sensor_index ? 150U : 280U, 0, true,
            true};
    }
    assert(test_nav_step(&navigation, &frame, NULL, &output));
    frame.timestamp_us =
        0x0fffffffU; /* Use the maximum accepted positive gap. */
    for (unsigned sensor_index = 0U;
         sensor_index < (unsigned int)S3_SENSOR_COUNT; ++sensor_index)
    {
        frame.sensor[sensor_index].timestamp_us = frame.timestamp_us;
    }
    assert(test_nav_step(&navigation, &frame, NULL, &output));
    test_core_make_runs('M', runs);
    for (unsigned run_index = 0; run_index < S3_BARCODE_RUNS; ++run_index)
    {
        runs[run_index] *= 100U;
    }
    assert(test_runs(runs, S3_BARCODE_RUNS, 50000000U, 50000000U, 1000U,
                     &config.barcode, &result));
    assert('M' == result.letter);
    config.line.lost_us = 0x80000000U;
    assert(!line_navigation_init(&navigation, &config));
    config.line.lost_us         = 0x7fffffffU;
    config.barcode.max_frame_us = 0x80000000U;
    assert(!line_navigation_init(&navigation, &config));
    /* The quiet-zone product, not the usual defaults, bounds these three
     * retained durations. Preserve the exact largest accepted values. */
    config.barcode = (bc_decoder_config_t){214748364U, 214748364U, 214748364U,
                                           214748365U, 10U};
    assert(line_navigation_init(&navigation, &config));
    assert(214748364U == navigation.barcode.config.max_gap_us);
    assert(214748364U == navigation.barcode.config.min_narrow_us);
    assert(214748364U == navigation.barcode.config.max_run_us);
    assert(214748365U == navigation.barcode.config.max_frame_us);
    assert(10U == navigation.barcode.config.quiet_modules);
    assert(214748364U == navigation.config.barcode.max_gap_us);
    assert(214748364U == navigation.config.barcode.min_narrow_us);
    assert(214748364U == navigation.config.barcode.max_run_us);
    assert(214748365U == navigation.config.barcode.max_frame_us);
    assert(10U == navigation.config.barcode.quiet_modules);
    config.barcode.max_run_us   = 214748365U;
    config.barcode.max_frame_us = 214748366U;
    assert(!line_navigation_init(&navigation, &config));
    config.barcode.max_run_us = 214748364U;
    config.barcode.max_gap_us = 214748365U;
    assert(!line_navigation_init(&navigation, &config));
    config.barcode.max_gap_us    = 214748364U;
    config.barcode.min_narrow_us = 214748365U;
    assert(!line_navigation_init(&navigation, &config));
    puts("CONFIG_MAX accepted ranges retained; 2^31 durations still rejected");
}

/** @brief Run the portable core fixture and emit navigation observations
 * over all ADC inputs and selected calibration limits.
 */
int main (void)
{
    assert(test_core_run() == 0);
    all_adc_values();
    calibration_limit();
    large_configuration_limits();
    fprintf(stderr,
            "PASS: %u deterministic field-wise navigation/barcode trace rows; "
            "all 65536 ADC inputs; calibration maximum\n",
            trace_rows);
    return 0;
}

/* End of file. */

