/* Physical GPIO routing regression; independent of historical algorithm traces. */
#include <assert.h>
#include <stdio.h>
#include "ir_acquisition.h"
#include "ir_bench_config.h"
#include "line_navigation.h"

typedef struct { uint16_t adc[3]; uint32_t now; } physical_adc_t;
static int32_t physical_read(void *context, uint8_t channel, uint16_t *raw)
{
    physical_adc_t *input = context;
    assert(channel < 3U);
    *raw = input->adc[channel];
    return 0;
}
static uint32_t physical_clock(void *context)
{
    return ((physical_adc_t *)context)->now;
}
static s3_navigation_output_t physical_observation(uint16_t gp26,
                                                  uint16_t gp27,
                                                  uint16_t gp28)
{
    s3_navigation_config_t cfg = s3_navigation_defaults();
    s3_navigation_t nav;
    s3_navigation_output_t out;
    s3_sensor_frame_t frame;
    physical_adc_t input = {{gp26, gp27, gp28}, 10000U};
    cfg.acquisition.enabled_mask = S3_ALL_SENSORS;
    assert(s3_navigation_init(&nav, &cfg));
    for (unsigned i = 0U; i < 5U; i++, input.now += 10000U) {
        assert(s3_acquisition_capture(&cfg.acquisition, physical_read,
                                     physical_clock, &input, &frame));
        assert(s3_navigation_process(&nav, &frame, NULL, &out));
    }
    return out;
}
static void test_physical_mapping(void)
{
    s3_navigation_config_t cfg = s3_navigation_defaults();
    s3_sensor_frame_t frame;
    physical_adc_t input = {{1010U, 2020U, 3030U}, 10000U};
    s3_navigation_output_t left, barcode, right;
    /* Independent wiring oracle: Grove 5=right ADC0, Grove 6=left ADC1,
     * Grove 7=barcode ADC2. Do not derive these expectations from macros. */
    assert(cfg.acquisition.channel[S3_SENSOR_LEFT] == 1U);
    assert(cfg.acquisition.channel[S3_SENSOR_RIGHT] == 0U);
    assert(cfg.acquisition.channel[S3_SENSOR_BARCODE] == 2U);
    assert(IR_BENCH_LEFT_CHANNEL == 1U && IR_BENCH_RIGHT_CHANNEL == 0U &&
           IR_BENCH_BARCODE_CHANNEL == 2U);
    cfg.acquisition.enabled_mask = S3_ALL_SENSORS;
    assert(s3_acquisition_capture(&cfg.acquisition, physical_read,
                                 physical_clock, &input, &frame));
    assert(frame.sensor[S3_SENSOR_LEFT].raw == 2020U);
    assert(frame.sensor[S3_SENSOR_RIGHT].raw == 1010U);
    assert(frame.sensor[S3_SENSOR_BARCODE].raw == 3030U);
    left = physical_observation(150U, 280U, 150U);
    barcode = physical_observation(150U, 280U, 280U);
    right = physical_observation(280U, 150U, 150U);
    assert(left.line.valid && left.line.present && left.line.error == -100);
    assert(right.line.valid && right.line.present && right.line.error == 100);
    assert(left.sensor[S3_SENSOR_BARCODE].darkness == 0U);
    assert(barcode.sensor[S3_SENSOR_BARCODE].darkness == 1000U);
    assert(left.line.valid == barcode.line.valid);
    assert(left.line.present == barcode.line.present);
    assert(left.line.error == barcode.line.error);
    assert(left.line.junction_candidate == barcode.line.junction_candidate);
    assert(left.line.lost_event == barcode.line.lost_event);
    assert(left.line.reacquired_event == barcode.line.reacquired_event);
    assert(left.line.junction_event == barcode.line.junction_event);
    assert(left.line.touching == barcode.line.touching);
}

int main(void)
{
    test_physical_mapping();
    puts("PASS: physical GP27 left, GP26 right, GP28 barcode routing and barcode-independent line observations");
    return 0;
}
