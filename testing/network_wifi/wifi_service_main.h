/** @file wifi_service_main.h
 * @brief Radio interface implemented by the Wi-Fi owner fixture.
 *
 * The simulated radio reports join and link outcomes to the owner task. The
 * include search selects the host CYW43 declarations.
 */
#ifndef TESTING_NETWORK_WIFI_WIFI_SERVICE_MAIN_H
#define TESTING_NETWORK_WIFI_WIFI_SERVICE_MAIN_H

#include "cyw43.h"
#include "lwip/init.h"
#include "lwip/timeouts.h"
#include "mqtt_utk.h"
#include "tm/tmonitor.h"

/* The polling task uses virtual time and a fixed kernel processor here. */
int32_t  tk_get_prc (void);
int32_t  tk_dly_tsk (uint32_t delay_ms);
void     mp_memory_barrir (void);
uint64_t time_us_64 (void);

#endif /* TESTING_NETWORK_WIFI_WIFI_SERVICE_MAIN_H */

/* End of file. */

