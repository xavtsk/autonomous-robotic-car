/** @file cmd_wire_main.c
 * @brief Exercise controller debug packet fields and malformed-input rejection.
 * Packet fields and fixture masks are interpreted as unsigned bits.
 */
#include "cmd_wire_main.h"
#include "../../app_program/controller/control_debug.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

#define MIN_TEST_PERCENT (-100)
#define MAX_TEST_PERCENT 100

/** @brief Round-trip the controller debug packet and check rejection of
 * invalid field widths, enum values and padding bits.
 */
int main (void)
{
    control_debug_sample_t sample = {0};
    control_debug_sample_t debug_sample;
    control_debug_packet_t packet;
    control_debug_packet_t debug_packet;
    assert(CONTROL_DEBUG_BITS == 551 && sizeof packet == 70U);
    sample.timestamp_us          = 0xfffffff0U;
    sample.motion_timestamp_us   = 0xffffffe0U;
    sample.sequence              = 255;
    sample.b_has_motion_snapshot = 1;
    sample.b_is_motion_valid     = 1;
    sample.motion_state          = 1;
    sample.setpoint_left         = 40;
    sample.setpoint_right        = -50;
    sample.measured_left         = 35;
    sample.measured_right        = -42;
    sample.duty_left             = 100;
    sample.duty_right            = -100;
    sample.encoder_left          = -524288;
    sample.encoder_right         = 524287;
    sample.drive_state           = 1;
    sample.line_error            = -50;
    sample.line_flags            = 3;
    sample.accepted_calibration  = 7;
    sample.left_white            = 150;
    sample.left_black            = 3300;
    sample.left_white_limit      = 400;
    sample.left_black_limit      = 600;
    sample.sample_gap_us         = 10000;
    sample.white_count           = 65535;
    sample.junction_events       = 65535;
    assert(control_debug_pack(&sample, &packet) &&
           control_debug_unpack(&packet, &debug_sample));
    assert(debug_sample.measured_right == -42 &&
           debug_sample.encoder_left == -524288 &&
           524287 == debug_sample.encoder_right);
    assert(control_debug_pack(&debug_sample, &debug_packet) &&
           !memcmp(&packet, &debug_packet, sizeof packet));
    printf("CONTROL_VECTOR ");

    for (unsigned byte_index = 0; byte_index < sizeof packet; ++byte_index)
    {
        printf("%02x", packet.bytes[byte_index]);
    }

    puts("");
    sample.reason      = 10;
    sample.drive_state = 6;
    assert(control_debug_pack(&sample, &debug_packet) &&
           control_debug_unpack(&debug_packet, &debug_sample) &&
           10U == debug_sample.reason);
    printf("TERRAIN_VECTOR ");

    for (unsigned byte_index = 0; byte_index < sizeof debug_packet;
         ++byte_index)
    {
        printf("%02x", debug_packet.bytes[byte_index]);
    }

    puts("");
    sample.reason = 11;
    assert(!control_debug_pack(&sample, &debug_packet));
    sample.reason      = 0;
    sample.drive_state = 1;

    for (int test_value = MIN_TEST_PERCENT; test_value <= MAX_TEST_PERCENT;
         ++test_value)
    {
        sample.line_error = test_value;
        sample.duty_left  = test_value;
        assert(control_debug_pack(&sample, &debug_packet) &&
               control_debug_unpack(&debug_packet, &debug_sample) &&
               debug_sample.duty_left == test_value &&
               debug_sample.line_error == test_value);
    }

    sample.line_error = -101;
    assert(!control_debug_pack(&sample, &debug_packet));
    sample.line_error   = 0;
    sample.encoder_left = -524289;
    assert(!control_debug_pack(&sample, &debug_packet));
    sample.encoder_left          = 0;
    sample.b_has_motion_snapshot = 0;
    assert(!control_debug_pack(&sample, &debug_packet));
    /* clang-format off */
    debug_packet           = packet;
    debug_packet.bytes[69] =
        (uint8_t)((unsigned)debug_packet.bytes[69] | 0x80U);
    /* clang-format on */
    assert(!control_debug_unpack(&debug_packet, &debug_sample));
    debug_packet          = packet;
    debug_packet.bytes[0] = 2;
    assert(!control_debug_unpack(&debug_packet, &debug_sample));
    puts("PASS: 70-byte controller codec, independent signed boundaries, "
         "validity, enum and padding rejection");
    return 0;
}

/* End of file. */

