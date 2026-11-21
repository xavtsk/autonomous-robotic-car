/** @file ultrasonic_sensor.h
 * @brief Trigger and poll PIO ultrasonic echo capture and convert pulses to
 * range.
 */
#ifndef BUDDY5_ULTRASONIC_SENSOR_H
#define BUDDY5_ULTRASONIC_SENSOR_H
#include "s5_config.h"
#include <stdbool.h>
#include <stdint.h>
/** @brief Configure PIO0 SM2 and GP1/GP0 for ultrasonic measurements.
 * @return True after setup; false if hardware is unconfirmed, PIO0 is held
 * in reset, SM2 is enabled, a pin setup fails, or APP_S5=0.
 * @details Requires PIO0 already released by the motion owner. Loads eight
 * instructions at 24..31 and configures SM2 for a 1 MHz counter. Success sets
 * ready and clears active. Failure does not undo earlier writes or reset
 * readiness from a previous call. The trigger-low write's error is ignored.
 * @pre The obstacle task exclusively owns this module's state, SM2/program
 * slots and pins. Serialize all init/trigger/poll/cancel calls; no lock or
 * resource claim is performed. Other PIO state machines must retain ownership.
 */
bool ultrasonic_sensor_init (void);
/** @brief Start one capture and issue a nominal 12 us trigger pulse.
 * @param[in] now Current timestamp in microseconds modulo 2^32.
 * @return False without starting if uninitialized, already active, ECHO is
 * high, or APP_S5=0; otherwise true after the trigger pulse.
 * @details The driver clears the SM2 FIFOs, restarts SM2 and stores now
 * before raising TRIG. It waits on the hardware timer for 12 us and lowers
 * TRIG. It ignores GPIO write failures and leaves echo polling and spacing
 * to the caller.
 * @pre The module is exclusively owned as described for init; the hardware
 * timer advances normally. Caller enforces measurement spacing and settling.
 */
bool ultrasonic_sensor_trigger (uint32_t now);
/** @brief Poll once for a completed echo or expired capture timeout.
 * @param[in] now Timestamp in microseconds from the trigger's clock.
 * @param[out] p_mm Caller-owned millimetres; unchanged when returning false.
 * @param[out] pb_is_valid Caller-owned usability flag; unchanged on false.
 * @return True once for a completed result, including an invalid measurement;
 * false when inactive, either output is null, still waiting, or APP_S5=0.
 * @details Reads at most one FIFO result, which takes precedence over timeout.
 * Converts its two-us counter units through ultrasonic_sensor_echo_mm();
 * oversized counts become clear range. With no FIFO result, elapsed time at
 * least S5_ECHO_TIMEOUT_US completes the capture: ECHO high means valid maximum
 * range, ECHO low means invalid zero. Every completed result cancels SM2.
 * Null outputs leave an active capture running. No echo wait occurs.
 * @pre Writable outputs are distinct; serialize module access. Elapsed time
 * since trigger is less than one uint32_t cycle; unsigned subtraction handles
 * timestamp wrap. The caller must inspect pb_is_valid as well as the return
 * value. A valid distance below the maximum is an obstacle reading. A valid
 * maximum is the clear-range sentinel. An invalid result may mean a missing
 * or too-short echo.
 */
bool ultrasonic_sensor_poll (uint32_t now, uint16_t * p_mm, bool * pb_is_valid);
/** @brief Disable SM2 and clear active, preserving readiness and start time.
 * @details Does not drain FIFOs or change the trigger pin. No result is
 * delivered for the cancelled capture. APP_S5=0 does nothing.
 * @pre Serialize with all other module operations; no lock is acquired.
 */
void ultrasonic_sensor_cancel (void);
/** @brief Convert a pulse to whole millimetres if it is within usable range.
 * @param[in] pulse_us Pulse duration in us; values above 24000 are rejected.
 * @param[out] p_mm Caller-owned result, unchanged on failure.
 * @return False for null output or converted range outside
 * S5_RANGE_MIN_MM..S5_RANGE_MAX_MM inclusive; otherwise true.
 * @details Computes (pulse_us * 343 + 1000) / 2000 with integer rounding.
 * No hardware, module state, locks or retained pointers are involved.
 */
bool ultrasonic_sensor_pulse_mm (uint32_t pulse_us, uint16_t * p_mm);
/** @brief Convert an echo, mapping every over-range pulse to clear distance.
 * @param[in] pulse_us Any uint32_t pulse duration in us.
 * @param[out] p_mm Caller-owned result; valid range, maximum if over-range,
 * or zero for too-short pulses. Null is rejected without a write.
 * @return True for a usable or over-range pulse; false for null output or a
 * too-short pulse. The latter failure writes zero, unlike
 * ultrasonic_sensor_pulse_mm.
 * @details Uses the pulse converter's rounding; a duration above 24000 or
 * converted distance above S5_RANGE_MAX_MM becomes maximum range. The long
 * duration check short-circuits before multiplication. No hardware/state or
 * synchronization is involved; no pointer is retained.
 */
bool ultrasonic_sensor_echo_mm (uint32_t pulse_us, uint16_t * p_mm);
#endif

/* End of file. */

