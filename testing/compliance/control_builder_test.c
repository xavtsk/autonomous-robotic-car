/** @file control_builder_test.c
 * @brief Verify compact builder acceptance/bytes against the wide API.
 * All invalid inputs retain their wide values until each interface validates.
 */
#include "control_builder_test.h"
#include "app_program/controller/control_debug.h"
#include <inttypes.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

typedef struct
{
    control_debug_field_t selector;
    size_t offset;
    uint8_t width;
    bool signed_value;
} builder_test_field_t;

static const builder_test_field_t g_builder_fields[] = {
    {CD_TIMESTAMP_US, offsetof(control_debug_sample_t, timestamp_us), 32U,
     false},
    {CD_MOTION_TIMESTAMP_US,
     offsetof(control_debug_sample_t, motion_timestamp_us), 32U, false},
    {CD_SEQUENCE, offsetof(control_debug_sample_t, sequence), 8U, false},
    {CD_MOTION_AVAILABLE, offsetof(control_debug_sample_t, motion_available),
     1U, false},
    {CD_MOTION_VALID, offsetof(control_debug_sample_t, motion_valid), 1U,
     false},
    {CD_BRAKED, offsetof(control_debug_sample_t, braked), 1U, false},
    {CD_BUSY, offsetof(control_debug_sample_t, busy), 1U, false},
    {CD_MOTION_STATE, offsetof(control_debug_sample_t, motion_state), 3U,
     false},
    {CD_MOTION_FAULT, offsetof(control_debug_sample_t, motion_fault), 3U,
     false},
    {CD_SETPOINT_LEFT, offsetof(control_debug_sample_t, setpoint_left), 13U,
     true},
    {CD_SETPOINT_RIGHT, offsetof(control_debug_sample_t, setpoint_right), 13U,
     true},
    {CD_MEASURED_LEFT, offsetof(control_debug_sample_t, measured_left), 13U,
     true},
    {CD_MEASURED_RIGHT, offsetof(control_debug_sample_t, measured_right), 13U,
     true},
    {CD_DUTY_LEFT, offsetof(control_debug_sample_t, duty_left), 8U, true},
    {CD_DUTY_RIGHT, offsetof(control_debug_sample_t, duty_right), 8U, true},
    {CD_ENCODER_LEFT, offsetof(control_debug_sample_t, encoder_left), 20U,
     true},
    {CD_ENCODER_RIGHT, offsetof(control_debug_sample_t, encoder_right), 20U,
     true},
    {CD_DRIVE_STATE, offsetof(control_debug_sample_t, drive_state), 3U, false},
    {CD_REASON, offsetof(control_debug_sample_t, reason), 4U, false},
    {CD_PENDING, offsetof(control_debug_sample_t, pending), 3U, false},
    {CD_LINE_ERROR, offsetof(control_debug_sample_t, line_error), 8U, true},
    {CD_LINE_FLAGS, offsetof(control_debug_sample_t, line_flags), 5U, false},
    {CD_BARCODE_STATUS, offsetof(control_debug_sample_t, barcode_status), 3U,
     false},
    {CD_ACCEPTED_CALIBRATION,
     offsetof(control_debug_sample_t, accepted_calibration), 3U, false},
    {CD_READINESS, offsetof(control_debug_sample_t, readiness), 5U, false},
    {CD_BARCODE_EVENTS, offsetof(control_debug_sample_t, barcode_events), 16U,
     false},
    {CD_JUNCTION_EVENTS, offsetof(control_debug_sample_t, junction_events), 16U,
     false},
    {CD_LOST_EVENTS, offsetof(control_debug_sample_t, lost_events), 16U, false},
    {CD_SAMPLE_GAP_US, offsetof(control_debug_sample_t, sample_gap_us), 32U,
     false},
    {CD_RUN_MAX_GAP_US, offsetof(control_debug_sample_t, run_max_gap_us), 32U,
     false},
    {CD_LEFT_WHITE, offsetof(control_debug_sample_t, left_white), 12U, false},
    {CD_LEFT_BLACK, offsetof(control_debug_sample_t, left_black), 12U, false},
    {CD_LEFT_WHITE_LIMIT, offsetof(control_debug_sample_t, left_white_limit),
     10U, false},
    {CD_LEFT_BLACK_LIMIT, offsetof(control_debug_sample_t, left_black_limit),
     10U, false},
    {CD_RIGHT_WHITE, offsetof(control_debug_sample_t, right_white), 12U, false},
    {CD_RIGHT_BLACK, offsetof(control_debug_sample_t, right_black), 12U, false},
    {CD_RIGHT_WHITE_LIMIT, offsetof(control_debug_sample_t, right_white_limit),
     10U, false},
    {CD_RIGHT_BLACK_LIMIT, offsetof(control_debug_sample_t, right_black_limit),
     10U, false},
    {CD_BARCODE_WHITE, offsetof(control_debug_sample_t, barcode_white), 12U,
     false},
    {CD_BARCODE_BLACK, offsetof(control_debug_sample_t, barcode_black), 12U,
     false},
    {CD_BARCODE_WHITE_LIMIT,
     offsetof(control_debug_sample_t, barcode_white_limit), 10U, false},
    {CD_BARCODE_BLACK_LIMIT,
     offsetof(control_debug_sample_t, barcode_black_limit), 10U, false},
    {CD_CAL_SENSOR, offsetof(control_debug_sample_t, cal_sensor), 2U, false},
    {CD_COLLECTING, offsetof(control_debug_sample_t, collecting), 1U, false},
    {CD_WHITE_MIN, offsetof(control_debug_sample_t, white_min), 12U, false},
    {CD_WHITE_MAX, offsetof(control_debug_sample_t, white_max), 12U, false},
    {CD_BLACK_MIN, offsetof(control_debug_sample_t, black_min), 12U, false},
    {CD_BLACK_MAX, offsetof(control_debug_sample_t, black_max), 12U, false},
    {CD_WHITE_COUNT, offsetof(control_debug_sample_t, white_count), 16U, false},
    {CD_BLACK_COUNT, offsetof(control_debug_sample_t, black_count), 16U, false},
};

/** Feed every wide public field to its matching validated builder operation. */
static bool builder_test_pack (const control_debug_sample_t * p_sample,
                               control_debug_packet_t * p_packet)
{
    control_debug_builder_t builder;
    size_t index = 0U;
    const unsigned char * p_bytes = (const unsigned char *)p_sample;
    control_debug_builder_init(&builder);
    for (index = 0U; index < CD_FIELD_COUNT; ++index)
    {
        const builder_test_field_t * p_field = &g_builder_fields[index];
        if (p_field->signed_value)
        {
            int32_t value = 0;
            (void)memcpy(&value, &p_bytes[p_field->offset], sizeof(value));
            control_debug_builder_s(&builder, p_field->selector, value);
        }
        else
        {
            uint32_t value = 0U;
            (void)memcpy(&value, &p_bytes[p_field->offset], sizeof(value));
            control_debug_builder_u(&builder, p_field->selector, value);
        }
    }
    return control_debug_builder_end(&builder, p_packet);
}

/** Compare both acceptance and unchanged-on-failure destination semantics. */
static bool builder_test_compare (const control_debug_sample_t * p_sample)
{
    control_debug_packet_t reference;
    control_debug_packet_t candidate;
    bool reference_ok = false;
    bool candidate_ok = false;
    (void)memset(&reference, 0xA5, sizeof(reference));
    candidate = reference;
    reference_ok = control_debug_pack(p_sample, &reference);
    candidate_ok = builder_test_pack(p_sample, &candidate);
    return (reference_ok == candidate_ok) &&
           (0 == memcmp(&reference, &candidate, sizeof(reference)));
}

/** Exercise field-width boundaries and narrower semantic-domain boundaries. */
int main (void)
{
    size_t index = 0U;
    uint32_t cases = 0U;
    control_debug_sample_t sample = {0};
    control_debug_builder_t builder;
    control_debug_packet_t packet;
    control_debug_packet_t zero;
    if (!builder_test_compare(&sample))
    {
        return 1;
    }
    for (index = 0U; index < CD_FIELD_COUNT; ++index)
    {
        const builder_test_field_t * p_field = &g_builder_fields[index];
        const int64_t limit = INT64_C(1) << p_field->width;
        const int64_t minimum = p_field->signed_value ? -(limit / 2) : 0;
        const int64_t maximum =
            p_field->signed_value ? ((limit / 2) - 1) : (limit - 1);
        const int64_t probes[] = {
            minimum - 1, minimum, minimum + 1, -1,        0,          1,
            maximum - 1, maximum, maximum + 1, INT32_MIN, UINT32_MAX, -101,
            -100,        -99,     4,           5,         6,          9,
            10,          11,      99,          100,       101,        999,
            1000,        1001,    4095,        4096};
        size_t probe = 0U;
        for (probe = 0U; probe < (sizeof(probes) / sizeof(probes[0])); ++probe)
        {
            unsigned char * p_bytes = (unsigned char *)&sample;
            bool representable = false;
            sample = (control_debug_sample_t){0};
            sample.motion_available = 1U;
            if (p_field->signed_value)
            {
                if ((probes[probe] >= INT32_MIN) &&
                    (probes[probe] <= INT32_MAX))
                {
                    /* Explicit domain bounds make this conversion exact. */
                    const int32_t value = (int32_t)probes[probe];
                    (void)memcpy(&p_bytes[p_field->offset], &value,
                                 sizeof(value));
                    representable = true;
                }
            }
            else if ((probes[probe] >= 0) && (probes[probe] <= UINT32_MAX))
            {
                /* Explicit domain bounds make this conversion exact. */
                const uint32_t value = (uint32_t)probes[probe];
                (void)memcpy(&p_bytes[p_field->offset], &value, sizeof(value));
                representable = true;
            }
            else
            {
                /* Values outside the public input type are not narrowed. */
            }
            if (representable)
            {
                ++cases;
                if (!builder_test_compare(&sample))
                {
                    (void)fprintf(stderr,
                                  "field=%zu value=%" PRId64 " differs\n",
                                  index, probes[probe]);
                    return 1;
                }
            }
        }
    }
    sample = (control_debug_sample_t){0};
    sample.motion_valid = 1U;
    if (!builder_test_compare(&sample))
    {
        return 1;
    }
    sample.motion_valid = 0U;
    (void)control_debug_pack(&sample, &zero);
    control_debug_builder_init(&builder);
    control_debug_builder_s(&builder, CD_DUTY_LEFT, -100);
    control_debug_builder_s(&builder, CD_DUTY_LEFT, 0);
    if (!control_debug_builder_end(&builder, &packet) ||
        (0 != memcmp(&packet, &zero, sizeof(packet))))
    {
        return 1;
    }
    control_debug_builder_s(&builder, CD_ENCODER_LEFT, -524289);
    packet = zero;
    if (control_debug_builder_end(&builder, &packet) ||
        (0 != memcmp(&packet, &zero, sizeof(packet))))
    {
        return 1;
    }
    control_debug_builder_init(&builder);
    control_debug_builder_u(&builder, CD_DUTY_LEFT, 0U);
    if (control_debug_builder_end(&builder, &packet))
    {
        return 1;
    }
    control_debug_builder_init(&builder);
    control_debug_builder_s(&builder, CD_TIMESTAMP_US, 0);
    if (control_debug_builder_end(&builder, &packet))
    {
        return 1;
    }
    control_debug_builder_init(&builder);
    /* Deliberate invalid selector tests rejection before array indexing. */
    control_debug_builder_u(&builder, (control_debug_field_t)-1, 0U);
    if (control_debug_builder_end(&builder, &packet) ||
        control_debug_builder_end(NULL, &packet) ||
        control_debug_builder_end(&builder, NULL))
    {
        return 1;
    }
    control_debug_builder_init(NULL);
    control_debug_builder_u(NULL, CD_SEQUENCE, 0U);
    control_debug_builder_s(NULL, CD_DUTY_LEFT, 0);
    (void)printf(
        "builder boundary cases=%" PRIu32
        " accepted/bytes/failure outputs match; scratch=%zu wide=%zu\n",
        cases, sizeof(builder), sizeof(sample));
    return 0;
}

/* End of file. */

