/* Baseline definitions copied from Buddy 1 immediately before this change.
 * Keep them here so the comparison is reproducible after the old JSON code
 * is replaced. These are measurements, not production storage. */
#include "subsystem1.h"
#include "mqtt_utk.h"
#include <stdio.h>
#include <string.h>

typedef struct { UB topic[64], payload[384]; UH length; UB qos, retain; } BEFORE_TX;
typedef struct { UB topic[64], payload[384]; UH length; } BEFORE_RX;
typedef struct {
    BOOL motion_valid; H left_mms, right_mms;
    BOOL barcode_valid; UB barcode_letter; BUDDY1_BARCODE_ACTION barcode_action;
    BOOL hump_valid; UH hump_count, hump_height_mm;
    BOOL heading_valid; H heading_deg;
    BOOL obstacle_valid; UH distance_mm, width_mm; BUDDY1_OBSTACLE_ROUTE route;
    BOOL state_valid; BUDDY1_ROBOT_STATE state;
} BEFORE_SNAPSHOT;

#define SLOT_BYTES(type) (sizeof(INT) + ((sizeof(type) + sizeof(INT) - 1) & ~(sizeof(INT) - 1)))
/* These also compile under the actual Arm compiler to check host layout. */
_Static_assert(sizeof(BEFORE_TX) == 452, "baseline outgoing layout");
_Static_assert(sizeof(BEFORE_RX) == 450, "baseline command layout");
_Static_assert(sizeof(BUDDY1_BARCODE_ACTION) == 1, "match Arm minimal-enum ABI for sizes");
_Static_assert(sizeof(BEFORE_SNAPSHOT) == 52, "baseline snapshot layout");
_Static_assert(sizeof(BUDDY1_TELEMETRY_MSG) == 17, "binary outgoing layout");
_Static_assert(sizeof(BUDDY1_COMMAND_MSG) == 5, "binary command layout");
_Static_assert(sizeof(T_MQTT_UTK_APP_MSG) == 84, "network staging layout");

int main(void)
{
    static const char *json[] = {
        "{\"left_mms\":120,\"right_mms\":-118}",
        "{\"letter\":\"W\",\"action\":\"left\"}",
        "{\"count\":2,\"height_mm\":45}", "{\"heading_deg\":359}",
        "{\"distance_mm\":180,\"width_mm\":140,\"route\":\"left\"}",
        "{\"state\":\"line_following\"}"
    };
    unsigned i;
    printf("Outgoing slot: %zu -> %zu bytes\n", sizeof(BEFORE_TX), sizeof(BUDDY1_TELEMETRY_MSG));
    printf("Command slot: %zu -> %zu bytes\n", sizeof(BEFORE_RX), sizeof(BUDDY1_COMMAND_MSG));
    printf("4-slot outbox including RTOS per-message headers/alignment: %zu -> %zu bytes\n",
           4 * SLOT_BYTES(BEFORE_TX), 4 * SLOT_BYTES(BUDDY1_TELEMETRY_MSG));
    printf("2-slot commands including RTOS per-message headers/alignment: %zu -> %zu bytes\n",
           2 * SLOT_BYTES(BEFORE_RX), 2 * SLOT_BYTES(BUDDY1_COMMAND_MSG));
    printf("Combined message-buffer storage requested at startup: %zu -> %zu bytes\n",
           4 * SLOT_BYTES(BEFORE_TX) + 2 * SLOT_BYTES(BEFORE_RX),
           4 * SLOT_BYTES(BUDDY1_TELEMETRY_MSG) + 2 * SLOT_BYTES(BUDDY1_COMMAND_MSG));
    printf("Shared snapshot cache: %zu -> %u bytes\n", sizeof(BEFORE_SNAPSHOT), BUDDY1_PAYLOAD_MAX);
    puts("Example MQTT application payloads (exclude topic/TCP/MQTT headers):");
    for (i = 0; i < 6; ++i)
        printf("%s: %zu -> %u bytes\n", buddy1_topic_name((UB)i), strlen(json[i]), buddy1_payload_size((UB)i));
    return 0;
}
