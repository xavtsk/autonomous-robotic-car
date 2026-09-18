/* robot_config.c - pure, host-testable shared configuration logic. */

#include "shared/robot_config.h"

#include <stddef.h>

static bool action_is_valid(nav_action_t action)
{
    return ((unsigned)action < (unsigned)NAV_COUNT);
}

void robot_config_init(robot_config_t * config)
{
    unsigned i;

    if (NULL == config)
    {
        return;
    }

    config->phase   = RUN_PHASE_CONFIGURING;
    config->version = 0u;

    for (i = 0u; i < ROBOT_BARCODE_COUNT; i++)
    {
        config->barcode_map[i] = NAV_NONE;
    }

    config->barcode_map[(unsigned)('A' - 'A')] = NAV_STRAIGHT;
    config->barcode_map[(unsigned)('B' - 'A')] = NAV_LEFT;
    config->barcode_map[(unsigned)('C' - 'A')] = NAV_RIGHT;
    config->barcode_map[(unsigned)('D' - 'A')] = NAV_UTURN;
}

robot_config_result_t robot_config_apply_barcode_map(
    robot_config_t * config,
    const nav_action_t map[ROBOT_BARCODE_COUNT])
{
    unsigned i;
    bool     has_assignment = false;

    if ((NULL == config) || (NULL == map))
    {
        return ROBOT_CONFIG_INVALID_ARGUMENT;
    }

    if (RUN_PHASE_CONFIGURING != config->phase)
    {
        return ROBOT_CONFIG_LOCKED;
    }

    for (i = 0u; i < ROBOT_BARCODE_COUNT; i++)
    {
        if (!action_is_valid(map[i]))
        {
            return ROBOT_CONFIG_INVALID_MAP;
        }
        if (NAV_NONE != map[i])
        {
            has_assignment = true;
        }
    }

    if (!has_assignment)
    {
        return ROBOT_CONFIG_INVALID_MAP;
    }

    for (i = 0u; i < ROBOT_BARCODE_COUNT; i++)
    {
        config->barcode_map[i] = map[i];
    }
    config->version++;

    return ROBOT_CONFIG_APPLIED;
}

bool robot_config_set_phase(robot_config_t * config, run_phase_t next_phase)
{
    run_phase_t expected;

    if ((NULL == config) ||
        ((unsigned)config->phase >= (unsigned)RUN_PHASE_COUNT) ||
        ((unsigned)next_phase >= (unsigned)RUN_PHASE_COUNT))
    {
        return false;
    }

    if (next_phase == config->phase)
    {
        return true;
    }

    expected = (run_phase_t)((int)config->phase + 1);
    if (next_phase != expected)
    {
        return false;
    }

    config->phase = next_phase;
    return true;
}

nav_action_t robot_config_action_for_letter(const robot_config_t * config,
                                            char letter)
{
    if ((NULL == config) || (letter < 'A') || (letter > 'Z'))
    {
        return NAV_NONE;
    }

    return config->barcode_map[(unsigned)(letter - 'A')];
}

uint32_t robot_config_assigned_count(const robot_config_t * config)
{
    unsigned i;
    uint32_t count = 0u;

    if (NULL == config)
    {
        return 0u;
    }

    for (i = 0u; i < ROBOT_BARCODE_COUNT; i++)
    {
        if (NAV_NONE != config->barcode_map[i])
        {
            count++;
        }
    }

    return count;
}

const char * run_phase_name(run_phase_t phase)
{
    static const char * const names[] =
    {
        "CONFIGURING",
        "ARMED",
        "AUTONOMOUS",
        "FINISHED"
    };

    if ((unsigned)phase >= (unsigned)RUN_PHASE_COUNT)
    {
        return "INVALID";
    }

    return names[phase];
}
