/* Host tests: synthetic inputs verify software, not physical sensor accuracy. */
#include <assert.h>
#include <stdio.h>
#include "ir_reflectance.h"
#include "ir_bench_config.h"

static void expect_sample(ir_reflectance_t *p_sensor, uint16_t raw,
                          uint16_t expected_darkness, ir_surface_t expected)
{
    uint16_t darkness = 65535U;
    assert(ir_reflectance_update(p_sensor, raw, &darkness));
    assert(darkness == expected_darkness);
    assert(p_sensor->surface == expected);
}

int main(void)
{
    ir_reflectance_t sensor = {0};
    const ir_calibration_t bench_calibration = {
        IR_BENCH_WHITE_RAW, IR_BENCH_BLACK_RAW,
        IR_BENCH_WHITE_LIMIT, IR_BENCH_BLACK_LIMIT
    };
    ir_calibration_t calibration = {150U, 280U, 400U, 600U};
    uint16_t darkness = 123U;

    assert(!ir_reflectance_update(&sensor, 150U, &darkness));
    assert(darkness == 123U);
    assert(ir_reflectance_init(&sensor, &bench_calibration));
    assert(ir_reflectance_init(&sensor, &calibration));

    /* Start between thresholds; do not invent a starting colour. */
    expect_sample(&sensor, 215U, 500U, IR_SURFACE_UNKNOWN);
    expect_sample(&sensor, 150U, 0U, IR_SURFACE_WHITE);
    expect_sample(&sensor, 215U, 500U, IR_SURFACE_WHITE);
    expect_sample(&sensor, 227U, 592U, IR_SURFACE_WHITE);
    expect_sample(&sensor, 228U, 600U, IR_SURFACE_BLACK);
    expect_sample(&sensor, 215U, 500U, IR_SURFACE_BLACK);
    expect_sample(&sensor, 203U, 407U, IR_SURFACE_BLACK);
    expect_sample(&sensor, 202U, 400U, IR_SURFACE_WHITE);
    expect_sample(&sensor, 280U, 1000U, IR_SURFACE_BLACK);
    expect_sample(&sensor, 4095U, 1000U, IR_SURFACE_BLACK);
    expect_sample(&sensor, 0U, 0U, IR_SURFACE_WHITE);

    /* Failure/recovery must not preserve a stale colour through an ADC error. */
    assert(!ir_reflectance_update(&sensor, 4096U, &darkness));
    assert(sensor.surface == IR_SURFACE_UNKNOWN);
    expect_sample(&sensor, 215U, 500U, IR_SURFACE_UNKNOWN);
    expect_sample(&sensor, 280U, 1000U, IR_SURFACE_BLACK);
    ir_reflectance_reset(&sensor);
    expect_sample(&sensor, 215U, 500U, IR_SURFACE_UNKNOWN);

    /* Calibration can be replaced without rewriting the algorithm. */
    calibration.white_raw = 3800U;
    calibration.black_raw = 280U;
    assert(ir_reflectance_init(&sensor, &calibration));
    expect_sample(&sensor, 2040U, 500U, IR_SURFACE_UNKNOWN);
    expect_sample(&sensor, 3800U, 0U, IR_SURFACE_WHITE);
    expect_sample(&sensor, 280U, 1000U, IR_SURFACE_BLACK);
    expect_sample(&sensor, 0U, 1000U, IR_SURFACE_BLACK);
    expect_sample(&sensor, 4095U, 0U, IR_SURFACE_WHITE);

    calibration.black_raw = calibration.white_raw;
    assert(!ir_reflectance_init(&sensor, &calibration));
    assert(!sensor.configured && sensor.surface == IR_SURFACE_UNKNOWN);
    calibration.black_raw = 4096U;
    assert(!ir_reflectance_init(&sensor, &calibration));
    calibration.black_raw = 280U;
    calibration.white_raw = 4096U;
    assert(!ir_reflectance_init(&sensor, &calibration));
    calibration.white_raw = 150U;
    calibration.white_limit = calibration.black_limit;
    assert(!ir_reflectance_init(&sensor, &calibration));
    calibration.white_limit = 400U;
    calibration.black_limit = 1001U;
    assert(!ir_reflectance_init(&sensor, &calibration));
    assert(!ir_reflectance_init(&sensor, 0));
    assert(!ir_reflectance_init(0, &calibration));
    assert(!ir_reflectance_update(0, 150U, &darkness));
    assert(!ir_reflectance_update(&sensor, 150U, 0));
    ir_reflectance_reset(0);

    puts("PASS: provisional thresholds, hysteresis, polarity, clamping, invalid inputs and recovery");
    return 0;
}
