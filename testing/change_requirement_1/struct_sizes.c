/** @file struct_sizes.c
 * @brief Change Requirement 1 evidence: storage sizes measured by the TARGET
 * compiler (arm-none-eabi-gcc, the firmware's own flags).
 */
/* Change Requirement 1 evidence: storage sizes measured by the TARGET compiler
 * (arm-none-eabi-gcc, the firmware's own flags). Each array below has the size
 * of one type, so `arm-none-eabi-nm -S` on the object prints the bytes.
 * Nothing here is linked into the firmware.
 *
 * "unpacked" = the same information in ordinary C types. For Buddy 4 and 5
 * these are the real decoded-view structs the code still uses for arithmetic
 * (terrain_result_t, s5_observation_t); they are what the shared/queued data
 * would occupy without the packed formats. Buddy 1's real pre-change layouts
 * are in testing/subsystem1/size_report.c. Buddy 2/3: their own EVIDENCE.md. */
#include "terr_data.h"
#include "s5_codec.h"
#include "scan_obstacle.h"
#include <stdint.h>
#include <stdbool.h>

#define SIZE(name, type) char size_##name[sizeof(type)]

/* Buddy 4: one shared measurement / queued event. */
SIZE(b4_unpacked_result, terrain_result_t);
char size_b4_packed_packet[TERRAIN_PACKET_SIZE];
SIZE(b4_unpacked_queue8, terrain_result_t[8]);
char size_b4_packed_queue8[8U * TERRAIN_PACKET_SIZE];

/* Buddy 5: shared task snapshot (forward reading + scan profile). */
SIZE(b5_unpacked_observation, s5_observation_t);
char size_b5_packed_observation[S5_PACKET_BYTES];

/* Buddy 5: one scan sample: distance, angle, validity. */
typedef struct
{
    uint16_t distance_mm;
    uint8_t angle_deg;
    bool valid;
} ref_sample_t;

SIZE(b5_unpacked_sample, ref_sample_t);
SIZE(b5_packed_sample, s5_sample_t);
SIZE(b5_unpacked_samples16, ref_sample_t[S5_SAMPLES_MAX]);
SIZE(b5_packed_samples16, s5_sample_t[S5_SAMPLES_MAX]);

/* Controller -> telemetry Buddy 5 status: state, fault, route, leg, pass step,
 * side swapped. Unpacked = one enum (state/fault/route) or byte per field. */
typedef enum
{
    REF_A,
    REF_B
} ref_enum_t;

typedef struct
{
    ref_enum_t state, fault, route;
    uint8_t leg, pass;
    bool swapped;
} ref_status_t;

SIZE(ctrl_unpacked_s5_status, ref_status_t);
SIZE(ctrl_packed_s5_status, uint16_t);

/* Controller bend detector (14 Nov 2026): float average + float latched value
 * before; int16 1/10000 average + int8 percent after (timestamp unchanged). */
typedef struct
{
    float turn_ratio, obstacle_turn;
    uint32_t turn_us;
} ref_bend_before_t;

typedef struct
{
    int16_t turn_q;
    int8_t obstacle_turn_pct;
    uint32_t turn_us;
} ref_bend_after_t;

SIZE(ctrl_bend_before, ref_bend_before_t);
SIZE(ctrl_bend_after, ref_bend_after_t);

/* End of file. */

