/* Current teammate contract, not the superseded GPIO-IRQ implementation. */
#include "motion_pack.h"
#include "motion_codec.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

static motion_status_t current;
static bool available;
bool motion_get_status(motion_status_t *out)
{ if (!available) return false; *out=current; return true; }
static float distance(float a,float b) { return a>b?a-b:b-a; }
int main(void)
{
    motion_cmd_t command;
    motion_status_t decoded;
    motion_report_t report, restored;
    motion_snapshot_t legacy;
    for (int speed=-1022;speed<=1022;speed++) {
        float v=speed/1000.0f;
        assert(motion_unpack_cmd(motion_cmd_velocity(v,-v),&command));
        assert(distance(command.left_mps,v)<=0.00101f);
        assert(distance(command.right_mps,-v)<=0.00101f);
        current=(motion_status_t){.state=MOTION_VELOCITY,.b_valid=true,
            .command_id=255,.speed_left_mps=v,.speed_right_mps=-v,
            .distance_m=-30.123f,.heading_deg=359};
        motion_unpack_status(motion_pack_status(&current),&decoded);
        assert(distance(decoded.speed_left_mps,v)<=0.00101f);
        assert(distance(decoded.speed_right_mps,-v)<=0.00101f);
        assert(distance(decoded.distance_m,-30.123f)<=0.00051f);
        assert(decoded.command_id==255 && decoded.b_valid);
    }
    for (unsigned id=0;id<256;id++) for(unsigned fault=0;fault<=MOTION_FAULT_TIMEOUT;fault++) {
        report=(motion_report_t){.command_id=(uint8_t)id,.outcome=MOTION_OUTCOME_FAULT,
            .fault=(motion_fault_t)fault,.b_lost=true};
        motion_unpack_report(motion_pack_report(&report),&restored);
        assert(restored.command_id==id&&restored.fault==fault&&restored.b_lost);
        assert(restored.outcome==MOTION_OUTCOME_FAULT);
    }
    assert(!motion_unpack_cmd(motion_cmd_velocity(1.1f,0),&command));
    assert(!motion_unpack_cmd(motion_cmd_velocity(NAN,0),&command));
    assert(!motion_unpack_cmd(motion_cmd_velocity(INFINITY,0),&command));
    assert(!motion_unpack_cmd(motion_cmd_drive(9.0f,0.1f),&command));
    assert(!motion_unpack_cmd(motion_cmd_turn(90.0f,0),&command));
    assert(!motion_read_snapshot(&legacy));
    available=true;current.b_valid=true;current.b_busy=false;
    current.speed_left_mps=0.884f;current.speed_right_mps=-0.884f;
    assert(motion_read_snapshot(&legacy)&&!motion_snapshot_speed_valid(&legacy));
    current.speed_left_mps=0.120f;current.speed_right_mps=-0.122f;
    assert(motion_read_snapshot(&legacy)&&motion_snapshot_speed_valid(&legacy));
    assert(motion_snapshot_left_mms(&legacy)==120&&motion_snapshot_right_mms(&legacy)==-122);
    current.b_busy=true;
    assert(motion_read_snapshot(&legacy)&&!motion_snapshot_speed_valid(&legacy));
    puts("PASS: current 64-bit status/16-bit reports/32-bit commands, every mm/s, invalid commands, safe legacy telemetry range");
    return 0;
}
