/** @file test_mapping_main.c
 * @brief Physical GPIO routing regression; independent of historical algorithm
 * traces.
 */
#include "test_mapping_main.h"
#include <assert.h>
#include <stdio.h>
#include "ir_acquisition.h"
#include "ir_bench_config.h"
#include "line_navigation.h"

#define OBSERVATION_SAMPLE_COUNT (5U)

typedef struct
{
    uint16_t adc[3];
    uint32_t now;
} physical_adc_t;

static int32_t  physical_read (void * p_context, uint8_t channel,
                               uint16_t * p_raw);
static uint32_t physical_clock (void * p_context);
static line_navigation_output_t physical_observation (uint16_t right_adc,
                                                      uint16_t left_adc,
                                                      uint16_t barcode_adc);
static void                     test_physical_mapping (void);

/** @brief Check that GP27, GP26 and GP28 feed the intended physical IR
 * channels and that line observations are independent of the barcode
 * input.
 */
int main (void)
{
    test_physical_mapping();
    puts("PASS: physical GP27 left, GP26 right, GP28 barcode routing and "
         "barcode-independent line observations");
    return 0;
}

/** @brief Return the fixture ADC value for a channel after checking the
 * index is below three.
 */
static int32_t physical_read (void * p_context, uint8_t channel,
                              uint16_t * p_raw)
{
    physical_adc_t * p_input = p_context;
    assert(channel < 3U);
    *p_raw = p_input->adc[channel];
    return 0;
}

/** @brief Return the current timestamp from the simulated ADC context.
 */
static uint32_t physical_clock (void * p_context)
{
    return ((physical_adc_t *)p_context)->now;
}

/** @brief Run navigation on timestamped fixture readings for the selected
 * physical channel and return the resulting observation.
 * @param right_adc Raw GP26 reading from the right sensor.
 * @param left_adc Raw GP27 reading from the left sensor.
 * @param barcode_adc Raw GP28 reading from the barcode sensor.
 */
static line_navigation_output_t physical_observation (uint16_t right_adc,
                                                      uint16_t left_adc,
                                                      uint16_t barcode_adc)
{
    line_navigation_config_t cfg = line_navigation_defaults();
    line_navigation_t        nav;
    line_navigation_output_t out;
    ir_acquisition_frame_t   frame;
    physical_adc_t input         = {{right_adc, left_adc, barcode_adc}, 10000U};
    cfg.acquisition.enabled_mask = S3_ALL_SENSORS;
    assert(line_navigation_init(&nav, &cfg));
    unsigned sample_index = 0U;

    while (sample_index < OBSERVATION_SAMPLE_COUNT)
    {
        assert(ir_acquisition_capture(&cfg.acquisition, physical_read,
                                      physical_clock, &input, &frame));
        assert(line_navigation_process(&nav, &frame, NULL, &out));
        sample_index++;
        input.now += 10000U;
    }

    return out;
}

/** @brief Check ADC-to-sensor channel mapping and verify the line detector's
 * response to each physical input.
 */
static void test_physical_mapping (void)
{
    line_navigation_config_t cfg = line_navigation_defaults();
    ir_acquisition_frame_t   frame;
    physical_adc_t           input = {{1010U, 2020U, 3030U}, 10000U};
    line_navigation_output_t left;
    line_navigation_output_t barcode;
    line_navigation_output_t right;
    /* Independent wiring oracle: Grove 5=right ADC0, Grove 6=left ADC1,
     * Grove 7=barcode ADC2. Do not derive these expectations from macros. */
    assert(1U == cfg.acquisition.channel[S3_SENSOR_LEFT]);
    assert(0U == cfg.acquisition.channel[S3_SENSOR_RIGHT]);
    assert(2U == cfg.acquisition.channel[S3_SENSOR_BARCODE]);
    assert(IR_BENCH_LEFT_CHANNEL == 1U && IR_BENCH_RIGHT_CHANNEL == 0U &&
           IR_BENCH_BARCODE_CHANNEL == 2U);
    cfg.acquisition.enabled_mask = S3_ALL_SENSORS;
    assert(ir_acquisition_capture(&cfg.acquisition, physical_read,
                                  physical_clock, &input, &frame));
    assert(2020U == frame.sensor[S3_SENSOR_LEFT].raw);
    assert(1010U == frame.sensor[S3_SENSOR_RIGHT].raw);
    assert(3030U == frame.sensor[S3_SENSOR_BARCODE].raw);
    left    = physical_observation(150U, 280U, 150U);
    barcode = physical_observation(150U, 280U, 280U);
    right   = physical_observation(280U, 150U, 150U);
    assert(left.line.b_is_valid && left.line.b_is_line_present &&
           left.line.error == -100);
    assert(right.line.b_is_valid && right.line.b_is_line_present &&
           100 == right.line.error);
    assert(0U == left.sensor[S3_SENSOR_BARCODE].darkness);
    assert(1000U == barcode.sensor[S3_SENSOR_BARCODE].darkness);
    assert(left.line.b_is_valid == barcode.line.b_is_valid);
    assert(left.line.b_is_line_present == barcode.line.b_is_line_present);
    assert(left.line.error == barcode.line.error);
    assert(left.line.b_has_junction_candidate ==
           barcode.line.b_has_junction_candidate);
    assert(left.line.b_has_lost_event == barcode.line.b_has_lost_event);
    assert(left.line.b_has_reacquired_event ==
           barcode.line.b_has_reacquired_event);
    assert(left.line.b_has_junction_event == barcode.line.b_has_junction_event);
    assert(left.line.b_is_touching_tape == barcode.line.b_is_touching_tape);
}

/* End of file. */

