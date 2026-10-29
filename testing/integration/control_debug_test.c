#include "../../app_program/controller/control_debug.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
int main(void)
{
    control_debug_sample_t s={0},d;
    control_debug_packet_t p,q;
    assert(CONTROL_DEBUG_BITS==551 && sizeof p==70);
    s.timestamp_us=0xfffffff0U;s.motion_timestamp_us=0xffffffe0U;s.sequence=255;
    s.motion_available=1;s.motion_valid=1;s.motion_state=1;s.setpoint_left=40;
    s.setpoint_right=-50;s.measured_left=35;s.measured_right=-42;
    s.duty_left=100;s.duty_right=-100;s.encoder_left=-524288;s.encoder_right=524287;
    s.drive_state=1;s.line_error=-50;s.line_flags=3;s.accepted_calibration=7;
    s.left_white=150;s.left_black=3300;s.left_white_limit=400;s.left_black_limit=600;
    s.sample_gap_us=10000;s.white_count=65535;s.junction_events=65535;
    assert(control_debug_pack(&s,&p) && control_debug_unpack(&p,&d));
    assert(d.measured_right==-42 && d.encoder_left==-524288 && d.encoder_right==524287);
    assert(control_debug_pack(&d,&q) && !memcmp(&p,&q,sizeof p));
    printf("CONTROL_VECTOR ");for(unsigned i=0;i<sizeof p;++i)printf("%02x",p.bytes[i]);puts("");
    s.reason=10;s.drive_state=6;
    assert(control_debug_pack(&s,&q)&&control_debug_unpack(&q,&d)&&d.reason==10);
    printf("TERRAIN_VECTOR ");for(unsigned i=0;i<sizeof q;++i)printf("%02x",q.bytes[i]);puts("");
    s.reason=11;assert(!control_debug_pack(&s,&q));s.reason=0;s.drive_state=1;
    for(int i=-100;i<=100;++i){s.line_error=i;s.duty_left=i;assert(control_debug_pack(&s,&q)&&control_debug_unpack(&q,&d)&&d.duty_left==i&&d.line_error==i);}
    s.line_error=-101;assert(!control_debug_pack(&s,&q));s.line_error=0;
    s.encoder_left=-524289;assert(!control_debug_pack(&s,&q));s.encoder_left=0;
    s.motion_available=0;assert(!control_debug_pack(&s,&q));
    q=p;q.bytes[69]|=0x80;assert(!control_debug_unpack(&q,&d));
    q=p;q.bytes[0]=2;assert(!control_debug_unpack(&q,&d));
    puts("PASS: 70-byte controller codec, independent signed boundaries, validity, enum and padding rejection");
    return 0;
}
