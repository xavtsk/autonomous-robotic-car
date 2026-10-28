#include "../../app_program/controller/ir_debug.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

int main(void)
{
    ir_debug_sample_t s = {0}, decoded;
    ir_debug_packet_t packet;
    uint32_t last = 0U;
    bool seen = false;
    assert(sizeof packet == 16U);
    assert(ir_debug_due(0xfffffff0U, &last, &seen));
    assert(!ir_debug_due(0x100U, &last, &seen));
    assert(ir_debug_due(0x10000U, &last, &seen));
    for (unsigned k = 0U; k < 2000U; ++k)
    {
        s.timestamp_us = 0xffffff00U + k * 50000U;
        s.sequence = (uint8_t)k;
        s.raw_valid = 7U;
        s.normalised_valid = (uint8_t)(k & 7U);
        for (unsigned i = 0U; i < 3U; ++i) {
            s.raw[i] = (uint16_t)((k * 37U + i * 1023U) % 4096U);
            s.darkness[i] = (uint16_t)((k * 13U + i) % 1001U);
            s.surface[i] = (uint8_t)(k % 3U);
        }
        assert(ir_debug_pack(&s, &packet));
        assert(ir_debug_unpack(&packet, &decoded));
        assert(decoded.timestamp_us == s.timestamp_us && decoded.sequence == s.sequence);
        for (unsigned i = 0U; i < 3U; ++i) {
            assert(decoded.raw[i] == s.raw[i]);
            assert(decoded.darkness[i] == ((s.normalised_valid & (1U << i)) ? s.darkness[i] : 0U));
        }
    }
    s = (ir_debug_sample_t){.timestamp_us=0xfffffff0U, .raw={100,4095,2300},
        .darkness={0,1000,555}, .raw_valid=7, .normalised_valid=7,
        .surface={1,2,2}, .sequence=255};
    assert(ir_debug_pack(&s, &packet));
    printf("IR_VECTOR ");
    for (unsigned i = 0; i < sizeof packet.bytes; ++i) printf("%02x", packet.bytes[i]);
    puts("");
    packet.bytes[15] |= 0x80U;
    assert(!ir_debug_unpack(&packet, &decoded));
    s.raw[0] = 4096U; assert(!ir_debug_pack(&s, &packet));
    s.raw[0] = 0U; s.darkness[1] = 1001U; assert(!ir_debug_pack(&s, &packet));
    s.darkness[1] = 1000U; s.surface[1] = 3U; assert(!ir_debug_pack(&s, &packet));
    s.surface[1] = 2U; s.raw_valid = 3U; assert(!ir_debug_pack(&s, &packet));
    puts("PASS: IR debug 16-byte explicit fields, independent validity, boundaries, padding, sequence and clock wrap, 20 Hz rate guard");
    return 0;
}
