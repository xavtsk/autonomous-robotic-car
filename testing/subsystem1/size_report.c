/** @file size_report.c
 * @brief Compare retained baseline telemetry storage and payload sizes with
 * current representations.
 */
/* Baseline definitions copied from Buddy 1 immediately before this change.
 * Keep them here so the comparison is reproducible after the old JSON code
 * is replaced. These are measurements, not production storage. */
#include "buddy1.h"
#include "mqtt_utk.h"
#include <stdio.h>
#include <string.h>

typedef struct
{
    UB topic[64], payload[384];
    UH length;
    UB qos, retain;
} BEFORE_TX;

typedef struct
{
    UB topic[64], payload[384];
    UH length;
} BEFORE_RX;

typedef struct
{
    BOOL motion_valid;
    H left_mms, right_mms;
    BOOL barcode_valid;
    UB barcode_letter;
    buddy1_barcode_action_t barcode_action;
    BOOL hump_valid;
    UH hump_count, hump_height_mm;
    BOOL heading_valid;
    H heading_deg;
    BOOL obstacle_valid;
    UH distance_mm, width_mm;
    buddy1_obstacle_route_t route;
    BOOL state_valid;
    buddy1_robot_state_t state;
} BEFORE_SNAPSHOT;

#define SLOT_BYTES(type)                                                       \
    (sizeof(INT) + ((sizeof(type) + sizeof(INT) - 1) & ~(sizeof(INT) - 1)))
/* These also compile under the actual Arm compiler to check host layout. */
_Static_assert(sizeof(BEFORE_TX) == 452, "baseline outgoing layout");
_Static_assert(sizeof(BEFORE_RX) == 450, "baseline command layout");
_Static_assert(sizeof(buddy1_barcode_action_t) == 1,
               "match Arm minimal-enum ABI for sizes");
_Static_assert(sizeof(BEFORE_SNAPSHOT) == 52, "baseline snapshot layout");

/* Current (after) sizes are printed, not asserted, so this report keeps
 * working as topics are added (14 Nov 2026: terrain, avoidance, scan;
 * 19 Nov 2026: terrain turn rate/safety, scan_sample). */

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
        "{\"stall\":0,\"encoder\":0,\"imu\":0,\"no_echo\":1,\"wifi\":0,\"ir\":0}",
        "{\"motion\":{\"left_mms\":120,\"right_mms\":-118},\"barcode\":{\"letter\":\"W\",\"action\":\"left\"},"
        "\"hump\":{\"count\":2,\"height_mm\":45},\"heading_deg\":359,\"obstacle\":{\"distance_mm\":180,"
        "\"width_mm\":140,\"route\":\"left\"},\"state\":\"line_following\",\"faults\":8,"
        "\"distance_cm\":1234,\"heading_since_start\":359}",
        "{\"result\":\"ok\"}",
        "{\"alive\":1,\"faults_known\":1,\"faults\":8}",
        "{\"distance_cm\":1234,\"heading_deg\":359}",
        "{\"tilt_valid\":1,\"pitch_deg\":12.3,\"roll_deg\":-4.5,\"state\":\"climbing\",\"on_hump\":1,"
        "\"climb_mm\":37,\"peak_valid\":1,\"peak_mm\":45,\"turn_valid\":1,\"turn_dps\":-87,\"safety\":\"none\"}",
        "{\"range_mm\":180,\"no_echo\":0,\"servo_deg\":90,\"state\":\"moving\",\"fault\":\"none\","
        "\"route\":\"left\",\"leg\":3,\"pass\":\"peek\",\"swapped\":0}",
        "{\"valid\":1,\"scan_id\":12,\"closest_mm\":196,\"width_mm\":180,\"left_clear_mm\":420,"
        "\"right_clear_mm\":380,\"forward_mm\":250,\"left_edge_mm\":90,\"right_edge_mm\":-90}",
        "{\"scan_id\":12,\"peek\":0,\"index\":3,\"count\":12,\"angle_deg\":75,\"valid\":1,\"distance_mm\":412}"};
    unsigned i, before = 0U, after = 0U;
    printf("Outgoing slot: %zu -> %zu bytes\n", sizeof(BEFORE_TX),
           sizeof(buddy1_telemetry_msg_t));
    printf("Command slot: %zu -> %zu bytes\n", sizeof(BEFORE_RX),
           sizeof(buddy1_command_msg_t));
    printf(
        "4-slot outbox including RTOS per-message headers/alignment: %zu -> %zu bytes\n",
        4 * SLOT_BYTES(BEFORE_TX), 4 * SLOT_BYTES(buddy1_telemetry_msg_t));
    printf(
        "2-slot commands including RTOS per-message headers/alignment: %zu -> %zu bytes\n",
        2 * SLOT_BYTES(BEFORE_RX), 2 * SLOT_BYTES(buddy1_command_msg_t));
    printf(
        "Combined message-buffer storage requested at startup: %zu -> %zu bytes\n",
        4 * SLOT_BYTES(BEFORE_TX) + 2 * SLOT_BYTES(BEFORE_RX),
        4 * SLOT_BYTES(buddy1_telemetry_msg_t) +
            2 * SLOT_BYTES(buddy1_command_msg_t));
    printf("Shared snapshot cache: %zu -> %u bytes\n", sizeof(BEFORE_SNAPSHOT),
           BUDDY1_PAYLOAD_MAX);
    printf("Network staging message (T_MQTT_UTK_APP_MSG): %zu bytes\n",
           sizeof(T_MQTT_UTK_APP_MSG));
    puts(
        "MQTT application payloads (exclude topic/TCP/MQTT headers); * = JSON equivalent:");
    puts(
        "| Topic | Text bytes | Binary bits | Binary bytes (incl. version byte) |");
    puts("|---|---:|---:|---:|");
    for (i = 0; i < BUDDY1_TOPIC_COUNT; ++i)
    {
        printf("| %s%s | %zu | %u | %u |\n", buddy1_topic_name((UB)i),
               i >= 6U ? " *" : "", strlen(json[i]), buddy1_payload_bits((UB)i),
               buddy1_payload_size((UB)i));
        before += (unsigned)strlen(json[i]);
        after += buddy1_payload_size((UB)i);
    }
    printf("| all %u topics, one message each | %u | | %u |\n",
           (unsigned)BUDDY1_TOPIC_COUNT, before, after);
    return 0;
}

/* End of file. */

