/** @file mqtt_utk.h
 * @brief Declare fake MQTT lifecycle hooks used by Wi-Fi host tests.
 */
#ifndef TESTING_NETWORK_WIFI_FAKES_MQTT_UTK_H
#define TESTING_NETWORK_WIFI_FAKES_MQTT_UTK_H

int  mqtt_utk_app_link (int b_is_up);
void mqtt_utk_start (void);
void mqtt_utk_poll (void);

#endif /* TESTING_NETWORK_WIFI_FAKES_MQTT_UTK_H */

/* End of file. */

