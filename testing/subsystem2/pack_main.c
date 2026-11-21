/** @file pack_main.c
 * @brief Exercise packed motion commands, status, reports and the legacy
 * snapshot adapter.
 * Unsigned casts spell out the original modulo conversion.
 * The selected narrow integer operands fit in int.
 */
/* Current teammate contract, not the superseded GPIO-IRQ implementation. */
#include "motion_pack.h"
#include "../../app_program/app_types.h"
#include "codec_motion.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "pack_main.h"

#define MAX_TARGET_SPEED_MMS 1022
#define COMMAND_ID_CODE_COUNT 256U
static motion_status_t g_current;
static bool            gb_is_available;

static float32_t distance (float32_t left_value, float32_t right_value);

/** @brief Copy the fixture status only when available. Otherwise return
 * false with the output untouched.
 */
bool motion_get_status (motion_status_t * p_out)
{
    if (!gb_is_available)
    {
        return false;
    }

    *p_out = g_current;
    return true;
}

/** @brief Check motion command and report encoding across wheel speeds and
 * verify rejection of invalid or unrepresentable inputs.
 */
int main (void)
{
    motion_pack_cmd_t       command;
    motion_status_t         decoded;
    motion_report_t         report;
    motion_report_t         restored;
    codec_motion_snapshot_t legacy;
    /* Function arguments are evaluated once; code zero remains no fault. */
    {
        unsigned calls = 0U;
        assert(motion_pack_fault_bit((motion_fault_t)calls++) == 0U);
        assert(1U == calls);

        for (unsigned fault = 1U; fault <= (unsigned)MOTION_FAULT_TIMEOUT;
             ++fault)
        {
            assert(motion_pack_fault_bit((motion_fault_t)fault) ==
                   (1U << (fault - 1U)));
        }
    }

    for (int speed = -MAX_TARGET_SPEED_MMS; speed <= MAX_TARGET_SPEED_MMS;
         speed++)
    {
        float32_t speed_mps = speed / 1000.0f;
        assert(motion_pack_decode(motion_pack_velocity(speed_mps, -speed_mps),
                                  &command));
        assert(distance(command.left_mps, speed_mps) <= 0.00101f);
        assert(distance(command.right_mps, -speed_mps) <= 0.00101f);
        g_current = (motion_status_t){.state             = MOTION_VELOCITY,
                                      .b_is_status_valid = true,
                                      .command_id        = 255,
                                      .speed_left_mps    = speed_mps,
                                      .speed_right_mps   = -speed_mps,
                                      .distance_m        = -30.123f,
                                      .heading_deg       = 359};
        motion_pack_read_status(motion_pack_status(&g_current), &decoded);
        assert(distance(decoded.speed_left_mps, speed_mps) <= 0.00101f);
        assert(distance(decoded.speed_right_mps, -speed_mps) <= 0.00101f);
        assert(distance(decoded.distance_m, -30.123f) <= 0.00051f);
        assert(255 == (int)decoded.command_id && decoded.b_is_status_valid);
    }

    for (unsigned test_command_id = 0; test_command_id < COMMAND_ID_CODE_COUNT;
         test_command_id++)
    {
        for (unsigned fault = 0; fault <= (unsigned)MOTION_FAULT_TIMEOUT;
             fault++)
        {
            report = (motion_report_t){.command_id = (uint8_t)test_command_id,
                                       .outcome    = MOTION_OUTCOME_FAULT,
                                       .fault      = (motion_fault_t)fault,
                                       .b_was_lost = true};
            motion_pack_read_report(motion_pack_report(&report), &restored);
            assert(restored.command_id == test_command_id &&
                   restored.fault == fault && restored.b_was_lost);
            assert((unsigned)MOTION_OUTCOME_FAULT == restored.outcome);
        }
    }

    assert(!motion_pack_decode(motion_pack_velocity(1.1f, 0), &command));
    assert(!motion_pack_decode(motion_pack_velocity(NAN, 0), &command));
    assert(!motion_pack_decode(motion_pack_velocity(INFINITY, 0), &command));
    assert(!motion_pack_decode(motion_pack_drive(9.0f, 0.1f), &command));
    assert(!motion_pack_decode(motion_pack_turn(90.0f, 0), &command));
    assert(!snapshot_motion_read(&legacy));
    gb_is_available             = true;
    g_current.b_is_status_valid = true;
    g_current.b_is_busy         = false;
    g_current.speed_left_mps    = 0.884f;
    g_current.speed_right_mps   = -0.884f;
    assert(snapshot_motion_read(&legacy) && !codec_motion_speed_valid(&legacy));
    g_current.speed_left_mps  = 0.120f;
    g_current.speed_right_mps = -0.122f;
    assert(snapshot_motion_read(&legacy) && codec_motion_speed_valid(&legacy));
    assert(codec_motion_left_mms(&legacy) == 120 &&
           codec_motion_right_mms(&legacy) == -122);
    g_current.b_is_busy = true;
    assert(snapshot_motion_read(&legacy) && !codec_motion_speed_valid(&legacy));
    puts("PASS: current 64-bit status/16-bit reports/32-bit commands, every "
         "mm/s, invalid commands, safe legacy telemetry range");
    return 0;
}

/** @brief Return the nonnegative absolute difference between the two
 * supplied values.
 */
static float32_t distance (float32_t left_value, float32_t right_value)
{
    return left_value > right_value ? left_value - right_value
                                    : right_value - left_value;
}

/* End of file. */

