/* Buddy 1: task-context reporting API. The RTOS copies complete binary
 * messages; only the kernel-owned network task calls lwIP/MQTT. */
#ifndef SUBSYSTEM1_H
#define SUBSYSTEM1_H

#include "../app.h"
#include "telemetry_codec.h"

#define BUDDY1_QUEUE_MESSAGES      4
#define BUDDY1_COMMAND_MESSAGES    2
#define S1_PRI_COMMAND           22

/* Four-bit topic ID; upper bits zero. Fixed slots preserve the four-message
 * capacity. MQTT sends only payload_size(type) bytes, never the whole slot. */
typedef struct {
    UB type;
    UB payload[BUDDY1_PAYLOAD_MAX];
} BUDDY1_TELEMETRY_MSG;
typedef enum {
    BUDDY1_COMMAND_UNKNOWN = 0, BUDDY1_COMMAND_SNAPSHOT, BUDDY1_COMMAND_BARCODE_MAP
} BUDDY1_COMMAND_KIND;
typedef struct {
    UB kind; /* two bits; length is fixed by kind */
    UB payload[BUDDY1_MAP_BYTES];
} BUDDY1_COMMAND_MSG;

IMPORT ID mbf_buddy1_outbox;
IMPORT ID mbf_buddy1_commands;
IMPORT ID mtx_buddy1_state;
IMPORT ER buddy1_report_motion(H left_mms, H right_mms);
IMPORT ER buddy1_report_barcode(UB decoded_letter);
IMPORT ER buddy1_report_hump(UH hump_count, UH height_mm);
IMPORT ER buddy1_report_heading(H heading_deg);
IMPORT ER buddy1_report_obstacle(UH distance_mm, UH width_mm, BUDDY1_OBSTACLE_ROUTE route);
IMPORT ER buddy1_report_robot_state(BUDDY1_ROBOT_STATE state);
/* Central protected telemetry fault mirror. Each caller updates only its
 * owned bits; active is a subset of changed. Task context only. Controller
 * safety decisions remain separate from this best-effort telemetry mirror. */
IMPORT ER buddy1_update_faults(UB changed, UB active);
IMPORT ER buddy1_invalidate(UB telemetry_type);
IMPORT ER buddy1_queue_publish(UB type, const UB *payload, UH payload_len);
IMPORT BOOL buddy1_outbox_take(BUDDY1_TELEMETRY_MSG *out);
IMPORT ER buddy1_command_enqueue(const char *topic, const UB *payload, UH payload_len);
IMPORT ER buddy1_command_process(const BUDDY1_COMMAND_MSG *command);
IMPORT BOOL buddy1_motion_snapshot(H *left_mms, H *right_mms);
IMPORT BOOL buddy1_snapshot(BUDDY1_SNAPSHOT *out);
IMPORT BOOL buddy1_snapshot_packet(UB packet[BUDDY1_PAYLOAD_MAX]);
IMPORT BUDDY1_COMMAND_KIND buddy1_command_kind(const char *topic);
IMPORT BOOL buddy1_parse_barcode_map(const UB *payload, UH length, BUDDY1_BARCODE_MAP *out);
IMPORT ER buddy1_apply_barcode_map(const BUDDY1_BARCODE_MAP *map);
IMPORT BUDDY1_BARCODE_ACTION buddy1_barcode_action(UB decoded_letter);
/* Close once before the first run; the map remains frozen until reboot. */
IMPORT BOOL buddy1_configuration_allowed(void);
IMPORT ER buddy1_configuration_close(void);
IMPORT ER buddy1_configuration_snapshot_and_close(BUDDY1_BARCODE_MAP *map);
/* Same, and reports whether this call froze the map (TRUE) or it was already
 * frozen by an earlier run (FALSE). newly_closed may be NULL. */
IMPORT ER buddy1_configuration_claim(BUDDY1_BARCODE_MAP *map, BOOL *newly_closed);
/* Undo only a close that the caller's own rejected START newly took (claim
 * reported TRUE), so no run ever used the map. Never undo an earlier freeze. */
IMPORT ER buddy1_configuration_reopen(void);
IMPORT void buddy1_command_task(INT stacd, void *exinf);
#endif
