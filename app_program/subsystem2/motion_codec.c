#include "motion_codec.h"

_Static_assert(sizeof(motion_snapshot_t) == 4U, "motion snapshot layout");
_Static_assert(MOTION_FAULT < 8 && MOTION_FAULT_TIMEOUT < 8,
               "motion state/fault no longer fits three bits");
_Static_assert(MANOEUVRE_U_TURN < 4, "manoeuvre no longer fits two bits");

static uint32_t word(const motion_snapshot_t *value)
{
    return (uint32_t)value->bytes[0] | ((uint32_t)value->bytes[1] << 8)
         | ((uint32_t)value->bytes[2] << 16) | ((uint32_t)value->bytes[3] << 24);
}

static uint32_t speed_code(int32_t mms)
{
    uint32_t mag = (uint32_t)((mms < 0) ? -mms : mms);
    mag = (mag + 1U) / 2U; /* nearest 2 mm/s, ties away from zero */
    return mag | ((uint32_t)(mms < 0) << 8);
}

static int16_t speed_value(uint32_t code)
{
    int32_t mag = (int32_t)(code & 255U) * 2;
    return (int16_t)((code & 256U) ? -mag : mag);
}

bool motion_snapshot_pack(motion_snapshot_t *out, motion_state_t state,
                          motion_fault_t fault, bool busy, bool speed_valid,
                          int32_t left_mms, int32_t right_mms)
{
    uint32_t packed;
    unsigned i;
    if (!out || (unsigned)state > MOTION_FAULT ||
        (unsigned)fault > MOTION_FAULT_TIMEOUT) return false;
    if (speed_valid && (left_mms < -MOTION_REPORT_MAX_MMS ||
        left_mms > MOTION_REPORT_MAX_MMS || right_mms < -MOTION_REPORT_MAX_MMS ||
        right_mms > MOTION_REPORT_MAX_MMS)) return false;
    packed = speed_valid ? speed_code(left_mms) | (speed_code(right_mms) << 9) : 0U;
    packed |= ((uint32_t)state << 18) | ((uint32_t)fault << 21)
            | ((uint32_t)busy << 24) | ((uint32_t)speed_valid << 25);
    for (i = 0; i < MOTION_SNAPSHOT_BYTES; ++i)
        out->bytes[i] = (uint8_t)(packed >> (i * 8U));
    return true;
}

bool motion_snapshot_valid(const motion_snapshot_t *value)
{
    uint32_t packed, left, right;
    if (!value) return false;
    packed = word(value);
    left = packed & 511U; right = (packed >> 9) & 511U;
    if ((packed >> 26) != 0U || ((packed >> 18) & 7U) > MOTION_FAULT ||
        ((packed >> 21) & 7U) > MOTION_FAULT_TIMEOUT) return false;
    if ((packed & (1U << 25)) == 0U) return (left == 0U && right == 0U);
    return left != 256U && right != 256U; /* reject negative zero */
}

motion_state_t motion_snapshot_state(const motion_snapshot_t *value)
{ return (motion_state_t)((word(value) >> 18) & 7U); }
motion_fault_t motion_snapshot_fault(const motion_snapshot_t *value)
{ return (motion_fault_t)((word(value) >> 21) & 7U); }
bool motion_snapshot_busy(const motion_snapshot_t *value)
{ return (word(value) & (1U << 24)) != 0U; }
bool motion_snapshot_speed_valid(const motion_snapshot_t *value)
{ return (word(value) & (1U << 25)) != 0U; }
int16_t motion_snapshot_left_mms(const motion_snapshot_t *value)
{ return speed_value(word(value) & 511U); }
int16_t motion_snapshot_right_mms(const motion_snapshot_t *value)
{ return speed_value((word(value) >> 9) & 511U); }
