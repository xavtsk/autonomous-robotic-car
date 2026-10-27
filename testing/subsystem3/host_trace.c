/* The SAME harness is compiled against saved original and revised modules.
 * Compare semantic fields and exact float bits, never padded C struct bytes. */
#include "line_navigation.h"
#include <assert.h>
#include <inttypes.h>
#include <stdio.h>
#include <string.h>

static unsigned trace_rows;
static bool trace_navigation(s3_navigation_t *, const s3_sensor_frame_t *, const s3_odometry_sample_t *, s3_navigation_output_t *);
static s3_barcode_status_t trace_barcode(s3_barcode_decoder_t *, uint32_t, bool, bool, s3_barcode_result_t *);
static bool trace_runs(const uint32_t *, unsigned int, uint32_t, uint32_t, uint32_t,
                       const s3_barcode_config_t *, s3_barcode_result_t *);

/* Keep the pre-existing suites intact; observe their public results. */
#define s3_navigation_process trace_navigation
#define s3_barcode_update trace_barcode
#define s3_barcode_decode_runs trace_runs
#define main original_core_main
#include "test_core.c"
#include "test_navigation.c"
#undef main
#undef s3_navigation_process
#undef s3_barcode_update
#undef s3_barcode_decode_runs

static uint32_t float_bits(float value)
{ uint32_t bits; memcpy(&bits, &value, sizeof(bits)); return bits; }

static void barcode_fields(const s3_barcode_result_t *value)
{
    if (value) printf(" %u %u %" PRIu32, (unsigned char)value->letter,
                      (unsigned)value->reversed, value->timestamp_us);
}

static bool trace_navigation(s3_navigation_t *context, const s3_sensor_frame_t *frame,
                              const s3_odometry_sample_t *odometry,
                              s3_navigation_output_t *out)
{
    bool okay = s3_navigation_process(context, frame, odometry, out);
    printf("N %u %u", trace_rows++, (unsigned)okay);
    if (out) {
        printf(" %" PRIu32 " %" PRIu32 " %" PRIu32 " %u", out->timestamp_us,
               out->sample_gap_us, out->events, (unsigned)out->fault_mask);
        for (unsigned i = 0; i < S3_SENSOR_COUNT; ++i)
            printf(" %u %u %u", (unsigned)out->sensor[i].valid,
                   (unsigned)out->sensor[i].darkness, (unsigned)out->sensor[i].surface);
        printf(" %u %u %u %d %u %u %u %u %08" PRIx32 " %08" PRIx32 " %u",
               (unsigned)out->line.valid, (unsigned)out->line.present,
               (unsigned)out->line.junction_candidate, (int)out->line.error,
               (unsigned)out->line.lost_event, (unsigned)out->line.reacquired_event,
               (unsigned)out->line.junction_event, (unsigned)out->steering.valid,
               float_bits(out->steering.left_mps), float_bits(out->steering.right_mps),
               (unsigned)out->barcode_status);
        barcode_fields(&out->barcode);
        printf(" %u", (unsigned)out->action);
    }
    putchar('\n');
    return okay;
}

static s3_barcode_status_t trace_barcode(s3_barcode_decoder_t *decoder, uint32_t now,
                                        bool valid, bool black, s3_barcode_result_t *out)
{
    s3_barcode_status_t status = s3_barcode_update(decoder, now, valid, black, out);
    printf("B %u %" PRIu32 " %u %u %u", trace_rows++, now, (unsigned)valid,
           (unsigned)black, (unsigned)status);
    barcode_fields(out);
    if (decoder) printf(" %u %u %u %u %u %u", (unsigned)decoder->count,
        (unsigned)decoder->configured, (unsigned)decoder->seen, (unsigned)decoder->black,
        (unsigned)decoder->collecting, (unsigned)decoder->pending);
    putchar('\n');
    return status;
}

static bool trace_runs(const uint32_t *runs, unsigned int count, uint32_t leading,
                       uint32_t trailing, uint32_t gap, const s3_barcode_config_t *config,
                       s3_barcode_result_t *out)
{
    bool okay = s3_barcode_decode_runs(runs, count, leading, trailing, gap, config, out);
    printf("R %u %u %u", trace_rows++, count, (unsigned)okay);
    barcode_fields(out); putchar('\n');
    return okay;
}

typedef struct { uint16_t raw; int32_t error; uint32_t time; } raw_source_t;
static int32_t raw_read(void *context, uint8_t channel, uint16_t *raw)
{
    raw_source_t *source = context; (void)channel; *raw = source->raw; return source->error;
}
static uint32_t raw_clock(void *context)
{ raw_source_t *source = context; return ++source->time; }

static void all_adc_values(void)
{
    s3_acquisition_config_t config = {{0, 1, 2}, 1};
    raw_source_t source = {0};
    s3_sensor_frame_t frame;
    for (uint32_t raw = 0; raw <= UINT16_MAX; ++raw) {
        source.raw = (uint16_t)raw;
        assert(s3_acquisition_capture(&config, raw_read, raw_clock, &source, &frame));
        assert(frame.sensor[0].valid == (raw <= 4095U));
        assert(frame.sensor[0].raw == (raw <= 4095U ? raw : 0U));
        assert(frame.sensor[0].error == (raw <= 4095U ? 0 : -1));
    }
    source.raw = 4095; source.error = -1234567;
    assert(s3_acquisition_capture(&config, raw_read, raw_clock, &source, &frame));
    assert(!frame.sensor[0].valid && frame.sensor[0].raw == 0);
    assert(frame.sensor[0].error == -1234567); /* full diagnostic error preserved */
    puts("ADC_ALL 65536 values: original invalid-input rejection preserved");
}

static void calibration_limit(void)
{
    s3_calibration_collector_t collector;
    ir_calibration_t active = {150, 280, 400, 600};
    s3_calibration_begin(&collector);
    for (unsigned i = 0; i < 65535U; ++i) {
        assert(s3_calibration_add(&collector, IR_SURFACE_WHITE, 4095));
        assert(s3_calibration_add(&collector, IR_SURFACE_BLACK, 0));
    }
    assert(collector.white.count == 65535U);
    assert(collector.white.sum == 268365825U);
    assert(collector.black.count == 65535U && collector.black.sum == 0U);
    assert(!s3_calibration_add(&collector, IR_SURFACE_WHITE, 4095));
    assert(collector.white.count == 65535U && collector.white.sum == 268365825U);
    assert(s3_calibration_finish(&collector, &active));
    assert(active.white_raw == 4095U && active.black_raw == 0U);
    s3_calibration_summary_t summary = s3_calibration_summary(&collector.white);
    assert(summary.available && summary.average == 4095.0f);
    printf("CAL_MAX %u %" PRIu32 " %u %u %08" PRIx32 "\n",
           (unsigned)summary.count, (uint32_t)collector.white.sum,
           (unsigned)summary.minimum, (unsigned)summary.maximum, float_bits(summary.average));
}

static void large_configuration_limits(void)
{
    s3_navigation_config_t config = s3_navigation_defaults();
    s3_navigation_t navigation;
    s3_navigation_output_t output;
    s3_sensor_frame_t frame;
    uint32_t runs[S3_BARCODE_RUNS];
    s3_barcode_result_t result;
    config.acquisition.enabled_mask = S3_ALL_SENSORS;
    config.max_sample_age_us = 0x7fffffffU;
    config.line.lost_us = 0x7fffffffU;
    config.line.reacquire_us = 0x7ffffffeU;
    config.line.junction_us = 0x7ffffffdU;
    config.line.rearm_us = 0x7ffffffcU;
    config.line.max_gap_us = 0x7fffffffU;
    config.line.signal_floor = 2000;
    config.line.junction_dark = 1000;
    config.barcode.max_gap_us = 42949672U;
    config.barcode.min_narrow_us = 1000000U;
    config.barcode.max_run_us = 42949672U;
    config.barcode.max_frame_us = 0x7fffffffU;
    config.barcode.quiet_modules = 50;
    assert(s3_navigation_init(&navigation, &config));
    assert(navigation.config.max_sample_age_us == config.max_sample_age_us);
    assert(navigation.line.config.lost_us == config.line.lost_us);
    assert(navigation.line.config.reacquire_us == config.line.reacquire_us);
    assert(navigation.line.config.junction_us == config.line.junction_us);
    assert(navigation.line.config.rearm_us == config.line.rearm_us);
    assert(navigation.barcode.config.max_frame_us == config.barcode.max_frame_us);
    assert(navigation.barcode.config.max_run_us == config.barcode.max_run_us);
    assert(navigation.barcode.config.max_gap_us == config.barcode.max_gap_us);
    assert(navigation.barcode.config.quiet_modules == 50);
    frame = (s3_sensor_frame_t){.timestamp_us = 0x90000000U};
    for (unsigned i = 0U; i < S3_SENSOR_COUNT; ++i)
        frame.sensor[i] = (s3_raw_sample_t){frame.timestamp_us,
            i == 0U ? 150U : 280U, 0, true, true};
    assert(trace_navigation(&navigation,&frame,NULL,&output));
    frame.timestamp_us = 0x0fffffffU; /* largest accepted positive gap */
    for (unsigned i = 0U; i < S3_SENSOR_COUNT; ++i)
        frame.sensor[i].timestamp_us = frame.timestamp_us;
    assert(trace_navigation(&navigation,&frame,NULL,&output));
    make_runs('M',runs);
    for (unsigned i = 0; i < S3_BARCODE_RUNS; ++i) runs[i] *= 100U;
    assert(trace_runs(runs,S3_BARCODE_RUNS,50000000U,50000000U,1000U,
                       &config.barcode,&result));
    assert(result.letter == 'M');
    config.line.lost_us = 0x80000000U;
    assert(!s3_navigation_init(&navigation,&config));
    config.line.lost_us = 0x7fffffffU;
    config.barcode.max_frame_us = 0x80000000U;
    assert(!s3_navigation_init(&navigation,&config));
    /* The quiet-zone product, not the usual defaults, bounds these three
     * retained durations. Preserve the exact largest accepted values. */
    config.barcode = (s3_barcode_config_t){214748364U, 214748364U,
        214748364U, 214748365U, 10U};
    assert(s3_navigation_init(&navigation,&config));
    assert(navigation.barcode.config.max_gap_us == 214748364U);
    assert(navigation.barcode.config.min_narrow_us == 214748364U);
    assert(navigation.barcode.config.max_run_us == 214748364U);
    assert(navigation.barcode.config.max_frame_us == 214748365U);
    assert(navigation.barcode.config.quiet_modules == 10U);
    assert(navigation.config.barcode.max_gap_us == 214748364U);
    assert(navigation.config.barcode.min_narrow_us == 214748364U);
    assert(navigation.config.barcode.max_run_us == 214748364U);
    assert(navigation.config.barcode.max_frame_us == 214748365U);
    assert(navigation.config.barcode.quiet_modules == 10U);
    config.barcode.max_run_us = 214748365U;
    config.barcode.max_frame_us = 214748366U;
    assert(!s3_navigation_init(&navigation,&config));
    config.barcode.max_run_us = 214748364U;
    config.barcode.max_gap_us = 214748365U;
    assert(!s3_navigation_init(&navigation,&config));
    config.barcode.max_gap_us = 214748364U;
    config.barcode.min_narrow_us = 214748365U;
    assert(!s3_navigation_init(&navigation,&config));
    puts("CONFIG_MAX accepted ranges retained; 2^31 durations still rejected");
}

int main(void)
{
    assert(original_core_main() == 0);
    all_adc_values(); calibration_limit(); large_configuration_limits();
    fprintf(stderr, "PASS: %u deterministic field-wise navigation/barcode trace rows; all 65536 ADC inputs; calibration maximum\n", trace_rows);
    return 0;
}
