/** @file controller_main.h
 * @brief Legacy controller task fixture.
 *
 * The fixture starts a simulated controller task through the fake kernel and
 * checks the earlier task priority and status interface.
 */
#ifndef TESTING_INTEGRATION_CONTROLLER_MAIN_H
#define TESTING_INTEGRATION_CONTROLLER_MAIN_H

#include "ctrl_test_kernel.h"
#include "../../app_program/subsystem3/line_navigation.h"

/** @brief Record the navigation mode requested by the legacy controller. */
ER subsystem3_set_mode (line_navigation_mode_t mode);

#endif /* TESTING_INTEGRATION_CONTROLLER_MAIN_H */

/* End of file. */

