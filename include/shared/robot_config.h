/*
 * robot_config.h - shared run lifecycle and barcode navigation mapping.
 *
 * This is deliberately not part of the WiFi module. Buddy 1 transports and
 * validates configuration received over MQTT; the integrated robot owns the
 * accepted configuration and decides when it becomes immutable.
 */
#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#include <stdbool.h>
#include <stdint.h>

#include "shared/robot_types.h"

#define ROBOT_BARCODE_COUNT  26u

typedef enum
{
    RUN_PHASE_CONFIGURING = 0,
    RUN_PHASE_ARMED,
    RUN_PHASE_AUTONOMOUS,
    RUN_PHASE_FINISHED,
    RUN_PHASE_COUNT
} run_phase_t;

typedef enum
{
    ROBOT_CONFIG_APPLIED = 0,
    ROBOT_CONFIG_LOCKED,
    ROBOT_CONFIG_INVALID_ARGUMENT,
    ROBOT_CONFIG_INVALID_MAP
} robot_config_result_t;

typedef struct
{
    run_phase_t phase;
    uint32_t    version;
    nav_action_t barcode_map[ROBOT_BARCODE_COUNT];
} robot_config_t;

/* Development defaults from the briefing: A=straight, B=left, C=right,
 * D=U-turn. Demo-day configuration replaces the whole table before arming. */
void robot_config_init(robot_config_t * config);

/* Replace the complete mapping. Only legal while CONFIGURING. */
robot_config_result_t robot_config_apply_barcode_map(
    robot_config_t * config,
    const nav_action_t map[ROBOT_BARCODE_COUNT]);

/* Advance CONFIGURING -> ARMED -> AUTONOMOUS -> FINISHED. Repeating the
 * current phase is harmless; going backwards requires a fresh init/reset. */
bool robot_config_set_phase(robot_config_t * config, run_phase_t next_phase);

nav_action_t robot_config_action_for_letter(const robot_config_t * config,
                                            char letter);

uint32_t robot_config_assigned_count(const robot_config_t * config);
const char * run_phase_name(run_phase_t phase);

#endif /* ROBOT_CONFIG_H */
