/** @file tmonitor.h
 * @brief Declare the fake monitor output function used by Wi-Fi tests.
 */
#ifndef TESTING_NETWORK_WIFI_FAKES_TM_TMONITOR_H
#define TESTING_NETWORK_WIFI_FAKES_TM_TMONITOR_H

#include <stdint.h>

typedef uint8_t UB;
int             tm_printf (const UB * p_format, ...);

#endif /* TESTING_NETWORK_WIFI_FAKES_TM_TMONITOR_H */

/* End of file. */

