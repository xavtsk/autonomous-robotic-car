/** @file app_main.h
 * @brief Application initialization and RTOS task creation interface.
 */
#ifndef APP_MAIN_H
#define APP_MAIN_H

#include "app.h"

/** @brief Run the kernel's initial user task under the selected car profile.
 * The kernel calls this entry once. A normal startup creates subsystem
 * objects before starting their tasks, then sleeps or reports diagnostics.
 * @return Zero if control returns to the kernel.
 */
INT usermain (void);

#endif /* APP_MAIN_H */

/* End of file. */

