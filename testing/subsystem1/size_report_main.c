/** @file size_report_main.c
 * @brief Compare retained baseline telemetry storage and payload sizes with
 * current representations.
 */
/* Baseline definitions copied from Buddy 1 immediately before this change.
 * Keep them here so the comparison is reproducible after the old JSON code
 * is replaced. These are measurements, not production storage. */
#include "size_report_main.h"
#include "../../app_program/build_assert.h"
#include "buddy1.h"
#include "mqtt_utk.h"
#include <stdio.h>
#include <string.h>

typedef struct
{
    UB topic[64];
    UB payload[384];
    UH length;
    UB qos;
    UB retain;
} baseline_outgoing_t;

typedef struct
{
    UB topic[64];
    UB payload[384];
    UH length;
} baseline_incoming_t;

typedef struct
{
    BOOL                     motion_valid;
    H                        left_mms;
    H                        right_mms;
    BOOL                     barcode_valid;
    UB                       barcode_letter;
    telemetry_codec_action_t barcode_action;
    BOOL                     hump_valid;
    UH                       hump_count;
    UH                       hump_height_mm;
    BOOL                     heading_valid;
    H                        heading_deg;
    BOOL                     obstacle_valid;
    UH                       distance_mm;
    UH                       width_mm;
    telemetry_codec_route_t  route;
    BOOL                     state_valid;
    telemetry_codec_state_t  state;
} baseline_snapshot_t;

#define SLOT_BYTES(type)                                                       \
    (sizeof(INT) + ((sizeof(type) + sizeof(INT) - 1U) & ~(sizeof(INT) - 1U)))
/* These also compile under the actual Arm compiler to check host layout. */
BUILD_ASSERT(sizeof(baseline_outgoing_t) == 452U, outgoing_layout_t);
BUILD_ASSERT(sizeof(baseline_incoming_t) == 450U, command_layout_t);
BUILD_ASSERT(sizeof(telemetry_codec_action_t) == 1U, action_enum_size_t);
BUILD_ASSERT(sizeof(baseline_snapshot_t) == 52U, snapshot_layout_t);

/* Current (after) sizes are printed, not asserted, so this report keeps
 * working as topics are added (14 Nov 2026: terrain, avoidance, scan;
 * 19 Nov 2026: terrain turn rate/safety, scan_sample). */

/** @brief Print representative JSON and binary telemetry sizes and compare
 * their computed storage totals.
 */
int main (void)
{
    /* Before = the text the original Buddy 1 code published (first six, copied
     * from it) or, for topics added after the change, the same values written
     * as JSON (marked *): what a text design would have had to send. */
    static const char * const json[BUDDY1_TOPIC_COUNT] = {
        "{\"left_mms\":120,\"right_mms\":-118}",
        "{\"letter\":\"W\",\"action\":\"left\"}",
        "{\"count\":2,\"height_mm\":45}",
        "{\"heading_deg\":359}",
        "{\"distance_mm\":180,\"width_mm\":140,\"route\":\"left\"}",
        "{\"state\":\"line_following\"}",
        "{\"stall\":0,\"encoder\":0,\"imu\":0,\"no_echo\":1,\"wifi\":0,"
        "\"ir\":0}",
        "{\"motion\":{\"left_mms\":120,\"right_mms\":-118},"
        "\"barcode\":{\"letter\":\"W\",\"action\":\"left\"},"
        "\"hump\":{\"count\":2,\"height_mm\":45},\"heading_deg\":359,"
        "\"obstacle\":{\"distance_mm\":180,"
        "\"width_mm\":140,\"route\":\"left\"},\"state\":\"line_following\","
        "\"faults\":8,"
        "\"distance_cm\":1234,\"heading_since_start\":359}",
        "{\"result\":\"ok\"}",
        "{\"alive\":1,\"faults_known\":1,\"faults\":8}",
        "{\"distance_cm\":1234,\"heading_deg\":359}",
        "{\"tilt_valid\":1,\"pitch_deg\":12.3,\"roll_deg\":-4.5,"
        "\"state\":\"climbing\",\"on_hump\":1,"
        "\"climb_mm\":37,\"peak_valid\":1,\"peak_mm\":45,\"turn_valid\":1,"
        "\"turn_dps\":-87,\"safety\":\"none\"}",
        "{\"range_mm\":180,\"no_echo\":0,\"servo_deg\":90,"
        "\"state\":\"moving\",\"fault\":\"none\","
        "\"route\":\"left\",\"leg\":3,\"pass\":\"peek\",\"swapped\":0}",
        "{\"valid\":1,\"scan_id\":12,\"closest_mm\":196,\"width_mm\":180,"
        "\"left_clear_mm\":420,"
        "\"right_clear_mm\":380,\"forward_mm\":250,\"left_edge_mm\":90,"
        "\"right_edge_mm\":-90}",
        "{\"scan_id\":12,\"peek\":0,\"index\":3,\"count\":12,"
        "\"angle_deg\":75,\"valid\":1,\"distance_mm\":412}"};
    unsigned topic_index;
    unsigned before = 0U;
    unsigned after  = 0U;
    printf("Outgoing slot: %zu -> %zu bytes\n", sizeof(baseline_outgoing_t),
           sizeof(buddy1_telemetry_msg_t));
    printf("Command slot: %zu -> %zu bytes\n", sizeof(baseline_incoming_t),
           sizeof(buddy1_command_msg_t));
    printf("4-slot outbox including RTOS per-message headers/alignment: %zu "
           "-> %zu bytes\n",
           4U * SLOT_BYTES(baseline_outgoing_t),
           4U * SLOT_BYTES(buddy1_telemetry_msg_t));
    printf("2-slot commands including RTOS per-message headers/alignment: %zu "
           "-> %zu bytes\n",
           2U * SLOT_BYTES(baseline_incoming_t),
           2U * SLOT_BYTES(buddy1_command_msg_t));
    printf("Combined message-buffer storage requested at startup: %zu -> %zu "
           "bytes\n",
           4U * SLOT_BYTES(baseline_outgoing_t) +
               2U * SLOT_BYTES(baseline_incoming_t),
           4U * SLOT_BYTES(buddy1_telemetry_msg_t) +
               2U * SLOT_BYTES(buddy1_command_msg_t));
    printf("Shared snapshot cache: %zu -> %u bytes\n",
           sizeof(baseline_snapshot_t), BUDDY1_PAYLOAD_MAX);
    printf("Network staging message (T_MQTT_UTK_APP_MSG): %zu bytes\n",
           sizeof(T_MQTT_UTK_APP_MSG));
    puts("MQTT application payloads (exclude topic/TCP/MQTT headers); * = "
         "JSON equivalent:");
    puts("| Topic | Text bytes | Binary bits | Binary bytes (incl. version "
         "byte) |");
    puts("|---|---:|---:|---:|");
    for (topic_index = 0; topic_index < (unsigned int)BUDDY1_TOPIC_COUNT;
         ++topic_index)
    {
        printf("| %s%s | %zu | %u | %u |\n",
               telemetry_codec_topic_name((UB)topic_index),
               topic_index >= 6U ? " *" : "", strlen(json[topic_index]),
               telemetry_codec_payload_bits((UB)topic_index),
               telemetry_codec_payload_size((UB)topic_index));
        before += (unsigned)strlen(json[topic_index]);
        after  += telemetry_codec_payload_size((UB)topic_index);
    }
    printf("| all %u topics, one message each | %u | | %u |\n",
           (unsigned)BUDDY1_TOPIC_COUNT, before, after);
    return 0;
}

/* End of file. */

