/** @file terrain_trace.c
 * @brief Existing terrain scenarios/assertions, with exact result values and
 * packet bytes.
 */
/* Existing terrain scenarios/assertions, with exact result values and packet bytes. */
#include "terrain_trace.h"
#include "terr_data.h"
#include <inttypes.h>
#include <stdio.h>
#include <string.h>
static void trace_terrain (terrain_monitor_t *, const imu_sample_t *,
                           const terrain_odometry_t *);
#define terrain_monitor_process trace_terrain
#include "host_test.c"
#undef terrain_monitor_process

static uint32_t trace_bits (float value)
{
    uint32_t encoded;
    memcpy(&encoded, &value, sizeof(encoded));
    return encoded;
}

static void trace_terrain (terrain_monitor_t * monitor,
                           const imu_sample_t * sample,
                           const terrain_odometry_t * odometry)
{
    uint8_t packet[TERRAIN_PACKET_SIZE];
    terrain_monitor_process(monitor, sample, odometry);
    terrain_pack(&monitor->result, packet);
    printf("T %" PRIu32 " %u %u %u %u %u %u %08" PRIx32 " %08" PRIx32
           " %08" PRIx32 " %08" PRIx32 " %08" PRIx32 " ",
           monitor->result.timestamp_us, monitor->result.state,
           monitor->result.safety, monitor->result.flags,
           monitor->result.impact, monitor->result.hump_completed,
           monitor->result.hump_active, trace_bits(monitor->result.pitch_deg),
           trace_bits(monitor->result.roll_deg),
           trace_bits(monitor->result.turn_dps),
           trace_bits(monitor->result.peak_m),
           trace_bits(monitor->result.hump_height_m));
    for (unsigned index = 0; index < TERRAIN_PACKET_SIZE; ++index)
    {
        printf("%02x", packet[index]);
    }
    putchar('\n');
}

/* End of file. */

