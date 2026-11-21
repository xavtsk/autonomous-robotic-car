/** @file remote_bridge_main.h
 * @brief Remote owner callbacks implemented by the MQTT bridge fixture.
 *
 * The bridge delivers link events and commands to the fixture and takes
 * status and reply packets through the remote control declarations.
 */
#ifndef TESTING_INTEGRATION_REMOTE_BRIDGE_MAIN_H
#define TESTING_INTEGRATION_REMOTE_BRIDGE_MAIN_H

#include "../../app_program/controller/remote_control.h"

/** @brief Provide the fixture seed for a new remote-control epoch. */
unsigned int lwip_utk_rand (void);

#endif /* TESTING_INTEGRATION_REMOTE_BRIDGE_MAIN_H */

/* End of file. */

