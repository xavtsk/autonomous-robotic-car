/** @file wheel_adapter_main.h
 * @brief Legacy telemetry fault adapter fixture.
 *
 * The fixture uses the earlier controller snapshot interface. These kernel
 * declarations preserve the original fake task and mutex types.
 */
#ifndef TESTING_SUBSYSTEM2_WHEEL_ADAPTER_MAIN_H
#define TESTING_SUBSYSTEM2_WHEEL_ADAPTER_MAIN_H

#include "wheel_kernel.h"
#include "../../app_program/subsystem3/line_navigation.h"

/** @brief Acknowledge a navigation mode request in the legacy fixture. */
ER subsystem3_set_mode (line_navigation_mode_t mode);

#endif /* TESTING_SUBSYSTEM2_WHEEL_ADAPTER_MAIN_H */

/* End of file. */

