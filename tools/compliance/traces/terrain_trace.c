/** @file terrain_trace.c
 * @brief Existing terrain scenarios/assertions, with exact result values and
 * packet bytes.
 */
#include "terrain_trace.h"
#include "../../../app_program/app_types.h"
#include <inttypes.h>
#include <stdio.h>
#include <string.h>

/** @brief Read the exact floating-point representation for comparison. */
static uint32_t trace_bits (float32_t value)
{
    uint32_t encoded;
    memcpy(&encoded, &value, sizeof(encoded));
    return encoded;
}

/** @brief Print the result fields and the exact terrain packet bytes. */
void terrain_trace_result (const terr_data_result_t * p_result)
{
    uint8_t packet[TERRAIN_PACKET_SIZE];
    terr_data_pack(p_result, packet);
    printf("T %" PRIu32 " %u %u %u %u %u %u %08" PRIx32 " %08" PRIx32
           " %08" PRIx32 " %08" PRIx32 " %08" PRIx32 " ",
           p_result->timestamp_us, p_result->state, p_result->safety,
           p_result->flags, p_result->b_has_impact,
           p_result->b_is_hump_complete, p_result->b_is_hump_active,
           trace_bits(p_result->pitch_deg), trace_bits(p_result->roll_deg),
           trace_bits(p_result->turn_dps), trace_bits(p_result->peak_m),
           trace_bits(p_result->hump_height_m));
    for (unsigned index = 0; index < TERRAIN_PACKET_SIZE; ++index)
    {
        printf("%02x", packet[index]);
    }
    putchar('\n');
}

/* End of file. */

