/** @file struct_sizes.c
 * @brief Change Requirement 1 evidence: storage sizes measured by the TARGET
 * compiler (arm-none-eabi-gcc, the firmware's own flags).
 *
 * Each array below has the size
 * of one type, so `arm-none-eabi-nm -S` on the object prints the bytes.
 * Nothing here is linked into the firmware.
 *
 * "unpacked" = the same information in ordinary C types. For Buddy 4 and 5
 * these are the real decoded-view structs the code still uses for arithmetic
 * (terr_data_result_t, s5_codec_observation_t); they are what the shared/queued
 * data would occupy without the packed formats. Buddy 1's real pre-change
 * layouts are in testing/subsystem1/size_report_main.c. Buddy 2 and Buddy 3
 * each record their measurements in EVIDENCE.md.
 */
#include "terr_data.h"
#include "../../app_program/app_types.h"
#include "s5_codec.h"
#include "profile_obstacle.h"
#include <stdint.h>
#include <stdbool.h>
#include "struct_sizes.h"

/* Buddy 4: one shared measurement / queued event. */
uint8_t size_b4_unpacked_result[sizeof(terr_data_result_t)];
uint8_t size_b4_packed_packet[TERRAIN_PACKET_SIZE];
uint8_t size_b4_unpacked_queue8[sizeof(terr_data_result_t[8])];
uint8_t size_b4_packed_queue8[8U * TERRAIN_PACKET_SIZE];

/* Buddy 5: shared task snapshot (forward reading + scan profile). */
uint8_t size_b5_unpacked_observation[sizeof(s5_codec_observation_t)];
uint8_t size_b5_packed_observation[S5_PACKET_BYTES];

/* Buddy 5: one scan sample: distance, angle, validity. */
typedef struct
{
    uint16_t distance_mm;
    uint8_t  angle_deg;
    bool     b_is_valid;
} ref_sample_t;

uint8_t size_b5_unpacked_sample[sizeof(ref_sample_t)];
uint8_t size_b5_packed_sample[sizeof(profile_obstacle_sample_t)];
uint8_t size_b5_unpacked_samples16[sizeof(ref_sample_t[S5_SAMPLES_MAX])];
uint8_t
    size_b5_packed_samples16[sizeof(profile_obstacle_sample_t[S5_SAMPLES_MAX])];

/* Controller -> telemetry Buddy 5 status: state, fault, route, leg, pass step,
 * side swapped. Unpacked = one enum (state/fault/route) or byte per field. */
typedef enum
{
    REF_A,
    REF_B
} ref_enum_t;

typedef struct
{
    ref_enum_t state;
    ref_enum_t fault;
    ref_enum_t route;
    uint8_t    leg;
    uint8_t    pass;
    bool       b_is_swapped;
} ref_status_t;

uint8_t size_ctrl_unpacked_s5_status[sizeof(ref_status_t)];
uint8_t size_ctrl_packed_s5_status[sizeof(uint16_t)];

/* Controller bend detector (14 Nov 2026): float average + float latched value
 * before; int16 1/10000 average + int8 percent after (timestamp unchanged). */
typedef struct
{
    float32_t turn_ratio;
    float32_t obstacle_turn;
    uint32_t  turn_us;
} ref_bend_before_t;

typedef struct
{
    int16_t  turn_q;
    int8_t   obstacle_turn_pct;
    uint32_t turn_us;
} ref_bend_after_t;

uint8_t size_ctrl_bend_before[sizeof(ref_bend_before_t)];
uint8_t size_ctrl_bend_after[sizeof(ref_bend_after_t)];

/* End of file. */

