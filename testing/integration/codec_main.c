/** @file codec_main.c
 * @brief Exercise round-trip preservation of copied controller packets.
 * Unsigned casts spell out the original modulo conversion.
 * Packet fields and fixture masks are interpreted as unsigned bits.
 */
#include "codec_main.h"
#include "../../app_program/app_types.h"
#include "../../app_program/controller/integration_codec.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

#define CHECKED_LETTER_COUNT 26
#define SYNTHETIC_PACKET_CASES 20000

static void check_control_commands (void);
static void roundtrip (const s3_integration_status_t * p_input);

/** @brief Round-trip the controller status packet and exercise malformed
 * bytes, padding and exact floating-point representations.
 */
int main (void)
{
    s3_integration_status_t status = {0};
    /* clang-format off */
    status.navigation.lifecycle            = S3_RUN_FAULT_LATCHED;
    status.navigation.readiness_blockers   = 31;
    status.navigation.latched_faults       = 15;
    status.navigation.accepted_calibration = 7;
    status.navigation.b_is_stop_requested  = status.navigation.b_has_odometry =
        status.navigation.b_is_rearming    = status.navigation.b_is_diagnostic =
            true;
    status.navigation.steering_preview     =
        (line_controller_request_t){true, -0.0f, 0.1234567f};
    status.navigation.capture_distance     = (line_navigation_distance_t){
        -1234.56789, 9876.54321, UINT32_MAX, true, true};
    status.motion                          = (motion_feedback_t){
        -0.0, UINT32_MAX, UINT32_MAX, MOTION_FAULT, MOTION_FAULT_TIMEOUT,
        true, true,       255,        UINT32_MAX};
    status.white                           =
        (ir_calibration_summary_t){65535, 4095, 4095, 4095.0f, true};
    status.black                           = status.white;
    status.samples                         =
        status.max_gap_us = status.above_target =
        status.command_sequence        = UINT32_MAX;
    /* clang-format on */

    status.run_timing = (run_timing_t){UINT32_MAX, UINT32_MAX, UINT32_MAX,
                                       UINT32_MAX, UINT32_MAX, UINT32_MAX};

    status.adc_status         = INT32_MIN;
    status.calibration_sensor = S3_SENSOR_RIGHT;

    for (unsigned state = 0; state <= (unsigned)S3_DRIVE_FAULT; state++)
    {
        for (unsigned reason = 0; reason <= (unsigned)S3_DRIVE_BAD_INPUT;
             reason++)
        {
            status.state  = (s3_drive_state_t)state;
            status.reason = (s3_drive_reason_t)reason;
            roundtrip(&status);
        }
    }

    integration_codec_status_packet_t packet;
    integration_codec_status_packet_t copy;
    assert(integration_codec_pack_status(&status, &packet));
    copy               = packet;
    status.white.count = 65536;
    assert(!integration_codec_pack_status(&status, &packet) &&
           memcmp(&packet, &copy, sizeof(packet)) == 0);
    status.white.count  = 65535;
    packet.metadata[63] = (uint8_t)((unsigned)packet.metadata[63] | 0x80U);
    assert(!integration_codec_unpack_status(&packet, &status));

    check_control_commands();

    uint32_t rng      = 0x7238a1U;
    unsigned accepted = 0;

    for (unsigned packet_index = 0;
         packet_index < (unsigned)SYNTHETIC_PACKET_CASES; packet_index++)
    {
        uint8_t * p_bytes = (uint8_t *)&packet;

        for (unsigned byte_index = 0; byte_index < sizeof(packet); byte_index++)
        {
            rng                 = (rng * 1664525U) + 1013904223U;
            p_bytes[byte_index] = (uint8_t)(rng >> 24);
        }

        if (integration_codec_unpack_status(&packet, &status))
        {
            assert(integration_codec_pack_status(&status, &copy) &&
                   memcmp(&packet, &copy, sizeof(packet)) == 0);
            accepted++;
        }
    }

    printf("PASS: full 60-byte nav/146-byte shared status/11-byte control, "
           "exact doubles/floats and run timing, boundaries/padding; hostile "
           "status packets accepted=%u\n",
           accepted);
}

/** @brief Round-trip each control kind and sensor choice, then compare the
 * encoded bytes after decoding the request.
 */
static void check_control_commands (void)
{
    s3_integration_request_t           request = {0};
    s3_integration_request_t           decoded;
    integration_codec_control_packet_t encoded_request;
    integration_codec_control_packet_t other_request;

    for (unsigned kind = 0; kind <= (unsigned)S3_CONTROL_RESET; kind++)
    {
        for (unsigned sensor = 0; sensor < (unsigned)S3_SENSOR_COUNT; sensor++)
        {
            request.kind   = (s3_integration_kind_t)kind;
            request.sensor = (ir_acquisition_sensor_id_t)sensor;

            for (unsigned letter_index = 0;
                 letter_index < (unsigned)CHECKED_LETTER_COUNT; letter_index++)
            {
                request.map[letter_index] =
                    (line_navigation_action_t)(letter_index %
                                               (unsigned)S3_NAV_ACTION_COUNT);
            }

            assert(integration_codec_pack_cmd(&request, &encoded_request) &&
                   integration_codec_unpack_cmd(&encoded_request, &decoded) &&
                   integration_codec_pack_cmd(&decoded, &other_request));
            assert(memcmp(&encoded_request, &other_request,
                          sizeof(encoded_request)) == 0);
        }
    }
}

/** @brief Pack and unpack integration status, compare its fields and verify
 * the re-encoded packet and exact steering representation.
 */
static void roundtrip (const s3_integration_status_t * p_input)
{
    integration_codec_status_packet_t packet;
    integration_codec_status_packet_t again;
    s3_integration_status_t           out;
    assert(integration_codec_pack_status(p_input, &packet));
    assert(integration_codec_unpack_status(&packet, &out));
    assert(integration_codec_pack_status(&out, &again) &&
           memcmp(&packet, &again, sizeof(packet)) == 0);
    assert(memcmp(&p_input->motion.centre_m, &out.motion.centre_m,
                  sizeof(float64_t)) == 0);
    assert(out.motion.command_id == p_input->motion.command_id &&
           out.motion.report_loss_count == p_input->motion.report_loss_count);
    assert(out.adc_status == p_input->adc_status &&
           out.navigation.latched_faults == p_input->navigation.latched_faults);
    assert(memcmp(&p_input->run_timing, &out.run_timing,
                  sizeof(p_input->run_timing)) == 0);
    assert(memcmp(&p_input->navigation.steering_preview.left_mps,
                  &out.navigation.steering_preview.left_mps,
                  sizeof(float32_t)) == 0);
}

/* End of file. */

