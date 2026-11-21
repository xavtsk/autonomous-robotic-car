/** @file bend_filter_check_main.c
 * @brief Change Requirement 1: the controller's bend detector moved from float
 * to minimum-width integers (14 Nov 2026).
 *
 * This replays 20000 random line-follower
 * wheel-speed sequences through the OLD float filter and the NEW fixed-point
 * filter (copied from s3_drive.c) and compares the bend decision Buddy 5 makes
 * (|turn| >= 20 %). Build and run the replay as follows.
 * @code{.sh}
 *   cc -O2 testing/change_requirement_1/bend_filter_check_main.c -o /tmp/bend
 *   /tmp/bend
 * @endcode
 */
#include "bend_filter_check_main.h"
#include "../../app_program/app_types.h"
#include <inttypes.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>

#define BEND_RANDOM_CASES 20000
static float32_t g_reference_turn = 0;
static int16_t   g_fixed_turn     = 0;

/** @brief Update the reference floating-point bend filter from the two wheel
 * readings and elapsed microseconds when the sample is valid.
 */
static void step_old (float32_t left_mps, float32_t right_mps,
                      uint32_t elapsed_us)
{
    float32_t sum_mps = left_mps + right_mps;
    if (sum_mps < 0.01f || 0U == elapsed_us || elapsed_us > 200000U)
    {
        return;
    }
    float32_t weight =
        (float32_t)elapsed_us / (500000.0f + (float32_t)elapsed_us);
    g_reference_turn +=
        weight * ((right_mps - left_mps) / sum_mps - g_reference_turn);
}

/** @brief Update the bounded integer comparison filter using the same wheel
 * readings and timing checks as the reference.
 */
static void step_new (float32_t left_mps, float32_t right_mps,
                      uint32_t elapsed_us)
{
    float32_t sum_mps = left_mps + right_mps;
    if (sum_mps < 0.01f || 0U == elapsed_us || elapsed_us > 200000U)
    {
        return;
    }
    float32_t ratio     = (right_mps - left_mps) / sum_mps;
    ratio               = ratio > 1 ? 1 : (ratio < -1 ? -1 : ratio);
    int32_t input_units = (int32_t)(ratio * 10000.0f);
    int32_t smoothed    = g_fixed_turn;
    /* clang-format off */
    smoothed    +=
        (int32_t)(((int64_t)(input_units - smoothed) * (int64_t)elapsed_us) /
                  (int64_t)(500000U + elapsed_us));
    g_fixed_turn =
        (int16_t)(smoothed > 10000 ? 10000
                                  : (smoothed < -10000 ? -10000 : smoothed));
    /* clang-format on */
}

/** @brief Convert the fixed-point fixture value to a signed integer
 * percentage by division by 100.
 */
static int8_t pct (int16_t fixed_units)
{
    return (int8_t)(fixed_units / 100);
}

/** @brief Replay seeded wheel-speed sequences through both bend filters.
 * Return one if disagreement or the maximum error exceeds the test
 * limits.
 */
int main (void)
{
    srand(7);
    int32_t   runs      = 0;
    int32_t   agree     = 0;
    int32_t   near      = 0;
    float64_t max_error = 0;
    for (int run = 0; run < BEND_RANDOM_CASES; run++)
    {
        g_reference_turn       = 0;
        g_fixed_turn           = 0;
        float32_t bias         = ((rand() % 2001) - 1000) / 1000.0f * 0.6f;
        float32_t wobble       = (rand() % 300) / 1000.0f;
        int       sample_count = 20 + rand() % 300;
        for (int sample_index = 0; sample_index < sample_count; sample_index++)
        {
            float32_t base = 0.08f + (rand() % 40) / 1000.0f;
            float32_t turn_ratio =
                bias + wobble * ((rand() % 2001) - 1000) / 1000.0f;
            float32_t left_mps  = base * (1 - turn_ratio);
            float32_t right_mps = base * (1 + turn_ratio);
            if (left_mps < 0)
            {
                left_mps = 0;
            }
            if (right_mps < 0)
            {
                right_mps = 0;
            }
            uint32_t elapsed_us = 8000 + rand() % 4000;
            step_old(left_mps, right_mps, elapsed_us);
            step_new(left_mps, right_mps, elapsed_us);
        }
        float64_t error = g_reference_turn - g_fixed_turn / 10000.0;
        if (error < 0)
        {
            error = -error;
        }
        if (error > max_error)
        {
            max_error = error;
        }
        int reference_bend =
            g_reference_turn >= 0.20f || g_reference_turn <= -0.20f;
        int fixed_percent = pct(g_fixed_turn);
        int fixed_bend    = fixed_percent >= 20 || fixed_percent <= -20;
        runs++;
        if (reference_bend == fixed_bend)
        {
            agree++;
        }
        else
        {
            float64_t magnitude =
                g_reference_turn < 0 ? -g_reference_turn : g_reference_turn;
            if (magnitude > 0.19 && magnitude < 0.21)
            {
                near++;
            }
            else
            {
                printf("MISMATCH fold=%f pct=%d\n", g_reference_turn,
                       fixed_percent);
            }
        }
    }
    printf("runs=%" PRId32 " same bend decision=%" PRId32 " differ=%" PRId32
           " (all within 0.19..0.21 of threshold: %" PRId32
           ") max |float-fixed|=%.5f\n",
           runs, agree, runs - agree, near, max_error);
    printf("RESULT: %s\n",
           (runs - agree) == near && max_error < 0.01 ? "PASS" : "FAIL");
    return (runs - agree) == near && max_error < 0.01 ? 0 : 1;
}

/* End of file. */

