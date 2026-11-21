/** @file buddy5_test.c
 * @brief On-device self-check entry point for Buddy 5.
 */
#include "buddy5_test.h"
#include "ultrasonic_sensor.h"
#if APP_TEST
/** @brief Check pulse conversion and obstacle packet round-trip handling on the
 * target. Report one result on the test console. */
void subsystem5_test (void)
{
    uint16_t               range_mm    = 0;
    s5_codec_observation_t observation = {0};
    s5_codec_observation_t out         = {0};
    uint8_t                packet[S5_PACKET_BYTES];
    bool b_is_ok = ultrasonic_sensor_pulse_mm(2000U, &range_mm) &&
                   343U == range_mm &&
                   !ultrasonic_sensor_pulse_mm(25000U, &range_mm) &&
                   s5_codec_observation_pack(&observation, packet) &&
                   s5_codec_observation_unpack(packet, &out);
    tm_printf(
        (UB *)"[TEST] subsystem5 codec/range %s; physical tests NOT RUN\n",
        b_is_ok ? "PASS" : "FAIL");
}
#endif

/* End of file. */

