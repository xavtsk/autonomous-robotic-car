/** @file nav_sizes_main.c
 * @brief Report navigation storage sizes and assert copied packet layouts.
 */
/* Compile with short enums to match the current Arm ABI. Full linker/static
 * RAM figures are separate from these individual structure sizes. */
#include "nav_sizes_main.h"
#include "../../app_program/build_assert.h"
#include "navigation_codec.h"
#include <stdio.h>

BUILD_ASSERT(sizeof(navigation_codec_packet_t) == 60U,
             navigation_packet_size_t);
BUILD_ASSERT(sizeof(navigation_codec_barcode_t) == 5U, barcode_packet_size_t);
BUILD_ASSERT(sizeof(navigation_codec_raw_frame_t) == 34U, raw_frame_size_t);

/** @brief Print compiled navigation storage sizes alongside their wide
 * calculation and validation types.
 */
int main (void)
{
    printf("Decoded navigation / copied packet: %zu / %zu bytes\n",
           sizeof(line_navigation_output_t), sizeof(navigation_codec_packet_t));
    printf("Decoded barcode / copied packet: %zu / %zu bytes\n",
           sizeof(bc_decoder_result_t), sizeof(navigation_codec_barcode_t));
    printf("Decoded raw frame / copied packet: %zu / %zu bytes\n",
           sizeof(ir_acquisition_frame_t),
           sizeof(navigation_codec_raw_frame_t));
    printf("Private navigation context: %zu bytes\n",
           sizeof(line_navigation_t));
    printf("Calibration collector: %zu bytes; per surface %zu bytes\n",
           sizeof(ir_calibration_collector_t), sizeof(ir_calibration_range_t));
    printf("Reflectance state per sensor: %zu bytes\n",
           sizeof(ir_reflectance_t));
    puts("Decoded/raw validation types intentionally remain wide; no ADC "
         "precision is discarded.");
    return 0;
}

/* End of file. */

