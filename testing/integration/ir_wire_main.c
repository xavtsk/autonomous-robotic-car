/** @file ir_wire_main.c
 * @brief Exercise IR debug packet encoding, decoding and validation.
 */
#include "ir_wire_main.h"
#include "../../app_program/controller/ir_debug.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

#define GENERATED_IR_PACKET_CASES 2000U

/** @brief Encode and decode IR preview packets. Check field limits, padding,
 * wraparound and the publication rate guard.
 */
int main (void)
{
    ir_debug_sample_t sample = {0};
    ir_debug_sample_t decoded;
    ir_debug_packet_t packet;
    uint32_t          last       = 0U;
    bool              b_was_seen = false;
    assert(sizeof packet == 16U);
    assert(ir_debug_due(0xfffffff0U, &last, &b_was_seen));
    assert(!ir_debug_due(0x100U, &last, &b_was_seen));
    assert(ir_debug_due(0x10000U, &last, &b_was_seen));

    for (unsigned sample_index = 0U; sample_index < GENERATED_IR_PACKET_CASES;
         ++sample_index)
    {
        sample.timestamp_us     = 0xffffff00U + (sample_index * 50000U);
        sample.sequence         = (uint8_t)sample_index;
        sample.raw_valid        = 7U;
        sample.normalised_valid = (uint8_t)(sample_index & 7U);

        for (unsigned sensor_index = 0U;
             sensor_index < (sizeof sample.raw / sizeof sample.raw[0]);
             ++sensor_index)
        {
            /* clang-format off */
            sample.raw[sensor_index]      =
                (uint16_t)(((sample_index * 37U) + (sensor_index * 1023U)) %
                           4096U);
            sample.darkness[sensor_index] =
                (uint16_t)(((sample_index * 13U) + sensor_index) % 1001U);
            sample.surface[sensor_index]  = (uint8_t)(sample_index % 3U);
            /* clang-format on */
        }

        assert(ir_debug_pack(&sample, &packet));
        assert(ir_debug_unpack(&packet, &decoded));
        assert(decoded.timestamp_us == sample.timestamp_us &&
               decoded.sequence == sample.sequence);

        for (unsigned sensor_index = 0U;
             sensor_index < (sizeof sample.raw / sizeof sample.raw[0]);
             ++sensor_index)
        {
            assert(decoded.raw[sensor_index] == sample.raw[sensor_index]);
            assert(decoded.darkness[sensor_index] ==
                   ((sample.normalised_valid & (1U << sensor_index))
                        ? sample.darkness[sensor_index]
                        : 0U));
        }
    }

    sample = (ir_debug_sample_t){.timestamp_us     = 0xfffffff0U,
                                 .raw              = {100, 4095, 2300},
                                 .darkness         = {0, 1000, 555},
                                 .raw_valid        = 7,
                                 .normalised_valid = 7,
                                 .surface          = {1, 2, 2},
                                 .sequence         = 255};
    assert(ir_debug_pack(&sample, &packet));
    printf("IR_VECTOR ");

    for (unsigned byte_index = 0; byte_index < sizeof packet.bytes;
         ++byte_index)
    {
        printf("%02x", packet.bytes[byte_index]);
    }

    puts("");
    packet.bytes[15] |= 0x80U;
    assert(!ir_debug_unpack(&packet, &decoded));
    sample.raw[0] = 4096U;
    assert(!ir_debug_pack(&sample, &packet));
    sample.raw[0]      = 0U;
    sample.darkness[1] = 1001U;
    assert(!ir_debug_pack(&sample, &packet));
    sample.darkness[1] = 1000U;
    sample.surface[1]  = 3U;
    assert(!ir_debug_pack(&sample, &packet));
    sample.surface[1] = 2U;
    sample.raw_valid  = 3U;
    assert(!ir_debug_pack(&sample, &packet));
    puts("PASS: IR debug 16-byte explicit fields, independent validity, "
         "boundaries, padding, sequence and clock wrap, 20 Hz rate guard");
    return 0;
}

/* End of file. */

