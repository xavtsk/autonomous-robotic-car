/** @file replay_barcode_record_main.c
 * @brief Diagnostic stdin: elapsed_us darkness, one sample per line.
 *
 * No padding by
 * default. Optional arguments add SYNTHETIC leading/trailing white milliseconds
 * to investigate clipped captures, never to claim an observed successful run.
 */
#include "replay_barcode_record_main.h"
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include "bc_decoder.h"

#define PADDING_SAMPLE_MS 10U
#define PADDING_SAMPLE_US 10000U
#define TRACE_COLUMN_COUNT 2

static bc_decoder_record_t record;
static uint32_t            now;
static unsigned int        decoded;
static unsigned int        rejected;
static unsigned int        timing;
static unsigned int        overflow;

/** @brief Advance replay time, feed one recorded sample and accumulate
 * decode, rejection, timing and overflow outcomes.
 */
static void feed (unsigned int elapsed_us, unsigned int darkness)
{
    const bc_decoder_config_t config = bc_decoder_defaults();
    bc_decoder_result_t       result;
    bc_decoder_status_t       status;
    now   += elapsed_us;
    status = bc_decoder_record_update(&record, &config, now, true,
                                      (uint16_t)darkness, &result);
    if ((bc_decoder_status_t)S3_BARCODE_DECODED == status)
    {
        decoded++;
        printf("letter=%c reversed=%u timestamp_us=%" PRIu32 "\n",
               result.letter, result.b_is_reversed, now);
    }
    else if ((bc_decoder_status_t)S3_BARCODE_REJECTED == status)
    {
        rejected++;
    }
    else if ((bc_decoder_status_t)S3_BARCODE_TIMING_ERROR == status)
    {
        timing++;
    }
    else if ((bc_decoder_status_t)S3_BARCODE_OVERFLOW == status)
    {
        overflow++;
    }
    else
    {
        /* The sample contributes no terminal barcode result. */
    }
}

/** @brief Read sample timing and darkness from standard input and report
 * decoder events and the final recording state using any requested
 * padding.
 */
int main (int argument_count, char * pp_arguments[])
{
    unsigned int elapsed_us;
    unsigned int dark;
    unsigned int padding_index;
    uintmax_t lead  = argument_count > 1 ? strtoul(pp_arguments[1], 0, 10) : 0U;
    uintmax_t trail = argument_count > 2 ? strtoul(pp_arguments[2], 0, 10) : 0U;
    if (lead > 10000U || trail > 10000U)
    {
        return 2;
    }
    printf("synthetic_leading_ms=%" PRIuMAX " synthetic_trailing_ms=%" PRIuMAX
           "\n",
           lead, trail);
    for (padding_index = 0U; padding_index < lead / PADDING_SAMPLE_MS;
         padding_index++)
    {
        feed(PADDING_SAMPLE_US, 0U);
    }
    while (scanf("%u %u", &elapsed_us, &dark) == TRACE_COLUMN_COUNT)
    {
        if (dark > 1000U || 0U == elapsed_us || elapsed_us > 1000000U)
        {
            return 2;
        }
        feed(elapsed_us, dark);
    }
    for (padding_index = 0U; padding_index < trail / PADDING_SAMPLE_MS;
         padding_index++)
    {
        feed(PADDING_SAMPLE_US, 0U);
    }
    printf("decoded=%u rejected=%u timing=%u overflow=%u state=%u count=%u\n",
           decoded, rejected, timing, overflow, record.state, record.count);
    return 0;
}

/* End of file. */

