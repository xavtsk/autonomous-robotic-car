/** @file test_core_main.c
 * @brief Exercise IR acquisition, calibration, line, barcode and navigation
 * logic.
 * Unsigned casts spell out the original modulo conversion.
 * The selected narrow integer operands fit in int.
 */
#include "test_core_main.h"
#include "test_navigation.h"
#include "host_trace_main.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "ir_acquisition.h"
#include "ir_calibration.h"
#include "line_controller.h"
#ifdef S3_LEGACY_MODULE_NAMES
#include "barcode_decoder.h"
#else
#include "bc_decoder.h"
#endif
#include "line_navigation.h"
#include "pdf_patterns.h"

#define CALIBRATION_SAMPLES_PER_SURFACE 10U
#define TRACK_REACQUIRE_SAMPLE_COUNT 4U
#define LINE_LOSS_SAMPLE_COUNT 20U
#define SUSTAINED_JUNCTION_SAMPLE_COUNT 20U
#define JUNCTION_CLEAR_SAMPLE_COUNT 12U
#define JUNCTION_CONFIRM_SAMPLE_COUNT 4U
#define SCAN_DIRECTION_COUNT 2U
#define DUPLICATE_FRAME_COUNT 2U

#define CODE39_ELEMENT_COUNT 9U
#define FIXTURE_ALPHABET_COUNT 26U
#define FIXTURE_FRAME_RUN_COUNT 29U

typedef struct
{
    unsigned int calls;
    uint32_t     now;
    uint8_t      fail;
} mock_adc_t;

/* Run widths extracted from the original PDF. See pdf_patterns.h for
 * provenance. */
static const uint32_t g_pdf_page_1[PDF_PATTERN_RUNS] = {
    39U,  119U, 40U, 40U, 118U, 40U,  119U, 40U, 39U,  40U,
    119U, 39U,  40U, 40U, 39U,  119U, 40U,  39U, 119U, 40U,
    39U,  119U, 40U, 40U, 118U, 40U,  119U, 40U, 39U};
static const uint32_t g_pdf_page_2[PDF_PATTERN_RUNS] = {
    20U, 60U, 20U, 20U, 59U, 20U, 60U, 20U, 20U, 20U, 59U, 20U, 20U, 20U, 20U,
    60U, 20U, 19U, 60U, 20U, 20U, 60U, 20U, 19U, 60U, 20U, 60U, 20U, 20U};
static const uint32_t g_pdf_page_3[PDF_PATTERN_RUNS] = {
    39U, 119U, 40U,  40U, 118U, 40U, 119U, 40U, 39U, 40U,
    39U, 119U, 119U, 40U, 119U, 39U, 40U,  40U, 39U, 40U,
    39U, 119U, 40U,  40U, 118U, 40U, 119U, 40U, 39U};
static const uint32_t g_pdf_page_4[PDF_PATTERN_RUNS] = {
    20U, 60U, 20U, 20U, 59U, 20U, 60U, 20U, 20U, 20U, 20U, 59U, 60U, 20U, 59U,
    20U, 20U, 20U, 20U, 20U, 20U, 59U, 20U, 20U, 60U, 20U, 60U, 19U, 20U};

static const uint16_t g_test_letters[26] = {
    0x109, 0x049, 0x148, 0x019, 0x118, 0x058, 0x00D, 0x10C, 0x04C,
    0x01C, 0x103, 0x043, 0x142, 0x013, 0x112, 0x052, 0x007, 0x106,
    0x046, 0x016, 0x181, 0x0C1, 0x1C0, 0x091, 0x190, 0x0D0};

static void test_acquisition (void);

static void test_calibration (void);

static void test_line (void);

static void make_runs (char letter, uint32_t * p_runs);

static void test_barcode (void);

#ifdef TEST_NAV_TRACE
/** @brief Generate the same framed Code 39 widths used by the core fixture. */
void test_core_make_runs (char letter, uint32_t * runs)
{
    make_runs(letter, runs);
}
#endif

/** @brief Run the core assertions and return zero after successful completion.
 */
int test_core_run (void)
{
    test_acquisition();
    test_calibration();
    test_line();
    test_barcode();
    test_navigation();
    puts("PASS: acquisition, calibration, line detection, steering, Code 39 "
         "and simulated navigation");
    return 0;
}

#ifndef TEST_NAV_TRACE
/** @brief Run the portable Buddy 3 core checks and return their outcome.
 */
int main (void)
{
    return test_core_run();
}
#endif

/** @brief Count the ADC callback, write the channel's fixture value and
 * return the injected channel-specific error.
 */
static int32_t mock_read (void * p_context, uint8_t channel, uint16_t * p_raw)
{
    mock_adc_t * p_mock = p_context;
    p_mock->calls++;
    *p_raw = (uint16_t)(100U + channel);
    return (p_mock->fail == channel) ? -9 : 0;
}

/** @brief Advance the simulated timestamp by ten and return the new value in
 * microseconds.
 */
static uint32_t mock_clock (void * p_context)
{
    mock_adc_t * p_mock = p_context;
    p_mock->now        += 10U;
    return p_mock->now;
}

/** @brief Exercise ADC channel selection, timestamp capture and read
 * failures, including validity of failed readings.
 */
static void test_acquisition (void)
{
    ir_acquisition_config_t cfg = {{0U, 1U, 2U}, 1U};
    ir_acquisition_frame_t  frame;
    mock_adc_t              mock = {0U, 0U, 255U};
    assert(ir_acquisition_capture(&cfg, mock_read, mock_clock, &mock, &frame));
    assert(1U == mock.calls && 100U == frame.sensor[S3_SENSOR_BARCODE].raw);
    assert(!frame.sensor[S3_SENSOR_LEFT].b_is_enabled &&
           !frame.sensor[S3_SENSOR_LEFT].b_is_valid);
    cfg.enabled_mask             = 7U;
    cfg.channel[S3_SENSOR_LEFT]  = 2U;
    cfg.channel[S3_SENSOR_RIGHT] = 1U;
    mock.fail                    = 2U;
    assert(ir_acquisition_capture(&cfg, mock_read, mock_clock, &mock, &frame));
    assert(frame.sensor[S3_SENSOR_BARCODE].b_is_valid);
    assert(!frame.sensor[S3_SENSOR_LEFT].b_is_valid &&
           frame.sensor[S3_SENSOR_LEFT].error == -9);
    assert(0U == frame.sensor[S3_SENSOR_LEFT].raw);
    assert(101U == frame.sensor[S3_SENSOR_RIGHT].raw);
    assert(frame.sensor[S3_SENSOR_BARCODE].timestamp_us <
           frame.sensor[S3_SENSOR_RIGHT].timestamp_us);
    cfg.channel[1] = cfg.channel[0];
    assert(!ir_acquisition_config_valid(&cfg));
    cfg.channel[1] = 3U;
    assert(!ir_acquisition_config_valid(&cfg));
    cfg.channel[1]   = 2U;
    cfg.enabled_mask = 8U;
    assert(!ir_acquisition_config_valid(&cfg));
    cfg.enabled_mask = 0U;
    assert(!ir_acquisition_capture(&cfg, mock_read, mock_clock, &mock, &frame));
    assert(!frame.sensor[0].b_is_valid);
}

/** @brief Check calibration collection, application and invalid or saturated
 * measurements.
 */
static void test_calibration (void)
{
    ir_calibration_collector_t   collector;
    ir_reflectance_calibration_t active = {150U, 280U, 400U, 600U};
    ir_reflectance_t             left;
    ir_reflectance_t             right;
    uint16_t                     dark;
    unsigned int                 index;
    ir_calibration_begin(&collector);
    assert(!ir_calibration_finish(&collector, &active));
    assert(!ir_calibration_summary(&collector.white).b_is_available);
    assert(!ir_calibration_add(&collector, IR_SURFACE_UNKNOWN, 10U));
    assert(!ir_calibration_add(&collector, IR_SURFACE_WHITE, 4096U));

    for (index = 0U; index < CALIBRATION_SAMPLES_PER_SURFACE; index++)
    {
        assert(ir_calibration_add(&collector, IR_SURFACE_WHITE,
                                  (uint16_t)(3000U + index)));
        assert(ir_calibration_add(&collector, IR_SURFACE_BLACK,
                                  (uint16_t)(200U + index)));
    }

    assert(ir_calibration_finish(&collector, &active));
    assert(3004U == active.white_raw && 204U == active.black_raw);
    assert(3000U == collector.white.minimum &&
           3009U == collector.white.maximum);
    assert(ir_calibration_summary(&collector.white).average == 3004.5f);
    assert(ir_reflectance_init(&left, &active));
    active = (ir_reflectance_calibration_t){150U, 280U, 400U, 600U};
    assert(ir_reflectance_init(&right, &active));
    assert(ir_reflectance_update(&left, 3004U, &dark) && 0U == dark);
    assert(ir_reflectance_update(&right, 280U, &dark) && 1000U == dark);
    assert(IR_SURFACE_WHITE == (int)left.surface &&
           IR_SURFACE_BLACK == (int)right.surface);
    ir_calibration_begin(&collector);

    for (index = 0U; index < CALIBRATION_SAMPLES_PER_SURFACE; index++)
    {
        assert(ir_calibration_add(&collector, IR_SURFACE_WHITE, 250U));
        assert(ir_calibration_add(&collector, IR_SURFACE_BLACK, 250U));
    }

    assert(!ir_calibration_finish(&collector, &active));
    assert(150U == active.white_raw && 280U == active.black_raw);
    assert(ir_calibration_add(&collector, IR_SURFACE_WHITE, 300U));
    assert(!ir_calibration_finish(&collector, &active));
    collector.white.count = 65535U;
    assert(!ir_calibration_add(&collector, IR_SURFACE_WHITE, 300U));
}

/** @brief Exercise line tracking, loss and reacquisition and reject invalid
 * steering gain.
 * @details Feed synthetic samples at 10 ms intervals. Check the steering
 * direction and speed clamps, then count one loss and one junction event
 * through sustained changes. A single dark spike can suppress steering
 * without producing a junction event.
 */
static void test_line (void)
{
    line_detection_t             detector;
    line_detection_config_t      cfg      = line_detection_defaults();
    line_controller_config_t     steering = line_controller_defaults();
    line_detection_observation_t line;
    line_controller_request_t    request;
    uint32_t                     sample_time_us = 0U;
    unsigned int                 sample_index;
    unsigned int                 events;
    assert(line_detection_init(&detector, &cfg));
    sample_index = 0;

    while (sample_index < TRACK_REACQUIRE_SAMPLE_COUNT)
    {
        line =
            line_detection_update(&detector, sample_time_us, true, 400U, 400U);
        assert(line.b_has_reacquired_event == (3U == sample_index));
        sample_index++;
        sample_time_us += 10000U;
    }

    assert(line.b_is_valid && line.b_is_line_present && 0 == line.error);
    request = line_controller_compute(&steering, &line);
    assert(request.b_is_valid && request.left_mps == request.right_mps);
    /* clang-format off */
    line            =
        line_detection_update(&detector, sample_time_us, true, 1000U, 0U);
    sample_time_us += 10000U;
    /* clang-format on */
    assert(line.error == -100);
    request = line_controller_compute(&steering, &line);
    assert(request.b_is_valid && request.left_mps < request.right_mps);
    /* clang-format off */
    line            =
        line_detection_update(&detector, sample_time_us, true, 0U, 1000U);
    sample_time_us += 10000U;
    request         = line_controller_compute(&steering, &line);
    /* clang-format on */
    assert(100 == line.error && request.left_mps > request.right_mps);
    steering.kp_mps = 1.0f;
    request         = line_controller_compute(&steering, &line);
    assert(request.left_mps == steering.maximum_mps &&
           0.0f == request.right_mps);
    events       = 0U;
    sample_index = 0;

    while (sample_index < LINE_LOSS_SAMPLE_COUNT)
    {
        line = line_detection_update(&detector, sample_time_us, true, 0U, 0U);

        if (line.b_has_lost_event)
        {
            events++;
        }

        assert(!line_controller_compute(&steering, &line).b_is_valid);
        sample_index++;
        sample_time_us += 10000U;
    }

    assert(1U == events);
    sample_index = 0;

    while (sample_index < TRACK_REACQUIRE_SAMPLE_COUNT)
    {
        line =
            line_detection_update(&detector, sample_time_us, true, 400U, 400U);
        sample_index++;
        sample_time_us += 10000U;
    }

    assert(line.b_has_reacquired_event);
    /* A one-sample black spike does not emit a junction. */
    /* clang-format off */
    line            =
        line_detection_update(&detector, sample_time_us, true, 1000U, 1000U);
    sample_time_us += 10000U;
    /* clang-format on */
    assert(!line.b_has_junction_event &&
           !line_controller_compute(&steering, &line).b_is_valid);
    /* clang-format off */
    line            =
        line_detection_update(&detector, sample_time_us, true, 400U, 400U);
    sample_time_us += 10000U;
    events          = 0U;
    sample_index    = 0;
    /* clang-format on */

    while (sample_index < SUSTAINED_JUNCTION_SAMPLE_COUNT)
    {
        line = line_detection_update(&detector, sample_time_us, true, 1000U,
                                     1000U);

        if (line.b_has_junction_event)
        {
            events++;
        }

        sample_index++;
        sample_time_us += 10000U;
    }

    assert(1U == events);
    sample_index = 0;

    while (sample_index < JUNCTION_CLEAR_SAMPLE_COUNT)
    {
        line =
            line_detection_update(&detector, sample_time_us, true, 400U, 400U);
        sample_index++;
        sample_time_us += 10000U;
    }

    assert(!line.b_has_junction_candidate);
    sample_index = 0;

    while (sample_index < JUNCTION_CONFIRM_SAMPLE_COUNT)
    {
        line = line_detection_update(&detector, sample_time_us, true, 1000U,
                                     1000U);
        sample_index++;
        sample_time_us += 10000U;
    }

    assert(line.b_has_junction_event);
    /* clang-format off */
    line            =
        line_detection_update(&detector, sample_time_us, false, 400U, 400U);
    sample_time_us += 10000U;
    /* clang-format on */
    assert(!line.b_is_valid &&
           !line_controller_compute(&steering, &line).b_is_valid);
    line = line_detection_update(&detector, sample_time_us, true, 400U, 400U);
    line = line_detection_update(&detector, sample_time_us + 100000U, true,
                                 400U, 400U);
    assert(!line.b_is_valid);
    /* Unsigned microsecond wrap remains continuous. */
    line_detection_reset(&detector);
    sample_time_us = 0xFFFFC000U;
    sample_index   = 0;

    while (sample_index < TRACK_REACQUIRE_SAMPLE_COUNT)
    {
        line =
            line_detection_update(&detector, sample_time_us, true, 400U, 400U);
        sample_index++;
        sample_time_us += 10000U;
    }

    assert(line.b_is_valid && line.b_has_reacquired_event);
    steering.kp_mps = NAN;
    assert(!line_controller_compute(&steering, &line).b_is_valid);
}

/** @brief Encode a requested letter between two stars using 10 ms narrow and
 * 30 ms wide element durations.
 */
static void make_runs (char letter, uint32_t * p_runs)
{
    unsigned int symbol_index;
    unsigned int index;
    uint16_t patterns[3] = {0x094, g_test_letters[(unsigned int)(letter - 'A')],
                            0x094};

    for (symbol_index = 0;
         symbol_index < (sizeof patterns / sizeof patterns[0]); symbol_index++)
    {
        for (index = 0; index < CODE39_ELEMENT_COUNT; index++)
        {
            p_runs[(symbol_index * 10U) + index] =
                (patterns[symbol_index] & (1U << (8U - index))) ? 30000U
                                                                : 10000U;
        }

        if (symbol_index < 2U)
        {
            p_runs[(symbol_index * 10U) + 9U] = 10000U;
        }
    }
}

/** @brief Supply a surface level every millisecond, advance the caller's
 * clock and retain the last successful barcode result.
 */
static unsigned int feed_duration (bc_decoder_t * p_data, uint32_t * p_time,
                                   bool b_is_black, uint32_t duration,
                                   bc_decoder_result_t * p_last)
{
    unsigned int count = 0U;
    uint32_t     elapsed;

    for (elapsed = 0U; elapsed < duration; elapsed += 1000U)
    {
        bc_decoder_result_t result;
        bc_decoder_status_t status =
            test_bar_step(p_data, *p_time, true, b_is_black, &result);

        if ((unsigned)S3_BARCODE_DECODED == status)
        {
            count++;
            *p_last = result;
        }

        *p_time += 1000U;
    }

    return count;
}

/** @brief Exercise direct run decoding and streaming updates, including
 * malformed frames and collection timeout.
 */
static void test_barcode (void)
{
    bc_decoder_config_t cfg    = {3000U, 3000U, 1000000U, 10000000U, 10U};
    bc_decoder_result_t result = {0};
    bc_decoder_t        decoder;
    uint32_t            runs[29];
    uint32_t            reverse[29];
    unsigned int        letter_index;
    unsigned int        index;
    unsigned int        trial_index;
    unsigned int        count;
    const uint32_t * fixtures[PDF_PATTERN_PAGES] = {g_pdf_page_1, g_pdf_page_2,
                                                    g_pdf_page_3, g_pdf_page_4};
    uint32_t         sample_time_us              = 0U;

    for (letter_index = 0U; letter_index < FIXTURE_ALPHABET_COUNT;
         letter_index++)
    {
        make_runs((char)((unsigned)('A') + letter_index), runs);

        for (index = 0; index < (sizeof reverse / sizeof reverse[0]); index++)
        {
            reverse[index] = runs[28U - index];
        }

        assert(test_runs(runs, 29U, 150000U, 150000U, 1000U, &cfg, &result));
        assert(result.letter == (char)((unsigned)('A') + letter_index) &&
               !result.b_is_reversed);
        assert(test_runs(reverse, 29U, 150000U, 150000U, 1000U, &cfg, &result));
        assert(result.letter == (char)((unsigned)('A') + letter_index) &&
               result.b_is_reversed);
        /* Exercise every symbol through timestamped transitions in both
         * directions. */
        for (trial_index = 0U; trial_index < SCAN_DIRECTION_COUNT;
             trial_index++)
        {
            const uint32_t * p_trace = trial_index ? reverse : runs;
            assert(bc_decoder_init(&decoder, &cfg));
            /* clang-format off */
            sample_time_us = 0xFFFF0000U;
            count          = 0U;
            count         +=
                feed_duration(&decoder, &sample_time_us, false, 150000U,
                                   &result);
            /* clang-format on */

            for (index = 0U; index < FIXTURE_FRAME_RUN_COUNT; index++)
            {
                count +=
                    feed_duration(&decoder, &sample_time_us, (index % 2U) == 0U,
                                  p_trace[index], &result);
            }

            count += feed_duration(&decoder, &sample_time_us, false, 250000U,
                                   &result);
            assert(1U == count &&
                   result.letter == (char)((unsigned)('A') + letter_index) &&
                   result.b_is_reversed == (trial_index != 0U));
        }
        /* Scaled, jittered elements with locally consistent character speeds.
         */
        for (index = 0U; index < (sizeof runs / sizeof runs[0]); index++)
        {
            unsigned int character_index = index / 10U;
            runs[index] = runs[index] * (80U + (character_index * 20U)) / 100U;
            runs[index] = runs[index] * ((index % 2U) ? 110U : 90U) / 100U;
        }

        assert(test_runs(runs, 29U, 200000U, 200000U, 1000U, &cfg, &result));
        assert(result.letter == (char)((unsigned)('A') + letter_index));
    }

    for (letter_index = 0U; letter_index < PDF_PATTERN_PAGES; letter_index++)
    {
        for (index = 0U; index < PDF_PATTERN_RUNS; index++)
        {
            /* clang-format off */
            runs[index]                              =
                fixtures[letter_index][index] * 500U;
            reverse[(PDF_PATTERN_RUNS - 1U) - index] = runs[index];
            /* clang-format on */
        }

        assert(test_runs(runs, PDF_PATTERN_RUNS, 600000U, 600000U, 1000U, &cfg,
                         &result));
        assert(result.letter == ((letter_index < 2U) ? 'A' : 'Z'));
        assert(test_runs(reverse, PDF_PATTERN_RUNS, 600000U, 600000U, 1000U,
                         &cfg, &result));
        assert(result.b_is_reversed &&
               result.letter == ((letter_index < 2U) ? 'A' : 'Z'));
        assert(!test_runs(runs, PDF_PATTERN_RUNS, 1000U, 600000U, 1000U, &cfg,
                          &result));
    }

    make_runs('A', runs);
    assert(!test_runs(runs, 28U, 150000U, 150000U, 1000U, &cfg, &result));
    assert(!test_runs(runs, 30U, 150000U, 150000U, 1000U, &cfg, &result));
    assert(!test_runs(runs, 29U, 150000U, 100U, 1000U, &cfg, &result));
    assert(!test_runs(runs, 29U, 150000U, 150000U, 10000U, &cfg, &result));
    runs[0] = 30000U;
    assert(!test_runs(runs, 29U, 150000U, 150000U, 1000U, &cfg, &result));
    make_runs('A', runs);
    runs[20] = 30000U; /* Invalid stop marker. */
    assert(!test_runs(runs, 29U, 150000U, 150000U, 1000U, &cfg, &result));
    make_runs('A', runs);
    runs[9] = 90000U;
    assert(!test_runs(runs, 29U, 150000U, 150000U, 1000U, &cfg, &result));
    /* Sampled trace, duplicate suppression, then a fresh frame. */
    assert(bc_decoder_init(&decoder, &cfg));
    make_runs('Z', runs);
    count  = 0U;
    count += feed_duration(&decoder, &sample_time_us, false, 150000U, &result);

    for (trial_index = 0U; trial_index < DUPLICATE_FRAME_COUNT; trial_index++)
    {
        for (index = 0U; index < (sizeof runs / sizeof runs[0]); index++)
        {
            count += feed_duration(&decoder, &sample_time_us,
                                   (index % 2U) == 0U, runs[index], &result);
        }

        count +=
            feed_duration(&decoder, &sample_time_us, false, 250000U, &result);
        assert('Z' == result.letter && count == trial_index + 1U);
    }
    /* A broken scan is reset; a full new quiet zone/frame can recover. */
    bc_decoder_reset(&decoder);
    sample_time_us = 0U;
    feed_duration(&decoder, &sample_time_us, false, 150000U, &result);
    feed_duration(&decoder, &sample_time_us, true, 10000U, &result);
    assert(test_bar_step(&decoder, sample_time_us + 100000U, true, false,
                         &result) == (unsigned)S3_BARCODE_TIMING_ERROR);
    /* clang-format off */
    sample_time_us += 101000U;
    count           =
        feed_duration(&decoder, &sample_time_us, false, 150000U, &result);
    /* clang-format on */
    make_runs('A', runs);

    for (index = 0; index < (sizeof runs / sizeof runs[0]); index++)
    {
        count += feed_duration(&decoder, &sample_time_us, (index % 2U) == 0U,
                               runs[index], &result);
    }

    count += feed_duration(&decoder, &sample_time_us, false, 150000U, &result);
    assert(1U == count && 'A' == result.letter);
    assert(test_bar_step(&decoder, sample_time_us, false, false, &result) ==
           (unsigned)S3_BARCODE_REJECTED);
    /* A short trailing quiet zone cannot complete a frame. */
    bc_decoder_reset(&decoder);
    /* clang-format off */
    sample_time_us = 0U;
    count          =
        feed_duration(&decoder, &sample_time_us, false, 150000U, &result);
    /* clang-format on */

    for (index = 0U; index < (sizeof runs / sizeof runs[0]); index++)
    {
        count += feed_duration(&decoder, &sample_time_us, (index % 2U) == 0U,
                               runs[index], &result);
    }

    count += feed_duration(&decoder, &sample_time_us, false, 50000U, &result);
    assert(0U == count &&
           test_bar_step(&decoder, sample_time_us, true, true, &result) ==
               (unsigned)S3_BARCODE_REJECTED);
    /* Resolution gate also rejects widths with fewer than three samples. */
    assert(test_runs(runs, 29U, 150000U, 150000U, 2900U, &cfg, &result));

    for (index = 0U; index < (sizeof runs / sizeof runs[0]); index++)
    {
        runs[index] /= 2U;
    }

    assert(!test_runs(runs, 29U, 150000U, 150000U, 2900U, &cfg, &result));
    /* Defensive overflow guard, including corrupted producer state. */
    bc_decoder_reset(&decoder);
    sample_time_us = 0U;
    feed_duration(&decoder, &sample_time_us, false, 150000U, &result);
    feed_duration(&decoder, &sample_time_us, true, 10000U, &result);
    decoder.count = S3_BARCODE_RUNS;
    assert(test_bar_step(&decoder, sample_time_us, true, false, &result) ==
           (unsigned)S3_BARCODE_OVERFLOW);
    /* Truncated frame timeout, not a fabricated character. */
    bc_decoder_reset(&decoder);
    sample_time_us = 0U;
    feed_duration(&decoder, &sample_time_us, false, 150000U, &result);
    feed_duration(&decoder, &sample_time_us, true, 10000U, &result);
    assert(feed_duration(&decoder, &sample_time_us, false, 1100000U, &result) ==
           0U);
    assert(!decoder.b_is_collecting);
}

/* End of file. */

