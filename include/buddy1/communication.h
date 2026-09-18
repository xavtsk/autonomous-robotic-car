/*
 * File: communication.h
 * Owner: Buddy 1 - WiFi Communication, Command and Telemetry
 * Purpose: Public communication subsystem interface.
 */

#ifndef BUDDY1_COMMUNICATION_H
#define BUDDY1_COMMUNICATION_H

#include <stdbool.h>
#include <stdint.h>

#include "shared/robot_config.h"

/* The integrated application owns config. Buddy 1 may update it only while
 * its shared phase is CONFIGURING. */
bool communication_init(robot_config_t * config);

/* Non-blocking service function. The future lowest-priority network task
 * should call this about every 10 ms. */
void communication_process(void);

bool communication_is_online(void);
uint32_t communication_reconnect_count(void);

#endif /* BUDDY1_COMMUNICATION_H */
