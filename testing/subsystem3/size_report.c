/* Compile with short enums to match the current Arm ABI. Full linker/static
 * RAM figures are separate from these individual structure sizes. */
#include "navigation_codec.h"
#include <stdio.h>

_Static_assert(sizeof(s3_navigation_packet_t) == 60U, "copied navigation layout");
_Static_assert(sizeof(s3_barcode_packet_t) == 5U, "copied barcode layout");
_Static_assert(sizeof(s3_raw_frame_packet_t) == 34U, "copied raw-frame layout");

int main(void)
{
    printf("Decoded navigation / copied packet: %zu / %zu bytes\n",
           sizeof(s3_navigation_output_t), sizeof(s3_navigation_packet_t));
    printf("Decoded barcode / copied packet: %zu / %zu bytes\n",
           sizeof(s3_barcode_result_t), sizeof(s3_barcode_packet_t));
    printf("Decoded raw frame / copied packet: %zu / %zu bytes\n",
           sizeof(s3_sensor_frame_t), sizeof(s3_raw_frame_packet_t));
    printf("Private navigation context: %zu bytes\n", sizeof(s3_navigation_t));
    printf("Calibration collector: %zu bytes; per surface %zu bytes\n",
           sizeof(s3_calibration_collector_t), sizeof(s3_calibration_range_t));
    printf("Reflectance state per sensor: %zu bytes\n", sizeof(ir_reflectance_t));
    puts("Decoded/raw validation types intentionally remain wide; no ADC precision is discarded.");
    return 0;
}
