/** @file buddy1.h
 * @brief Buddy 1: task-context reporting API.
 *
 * The RTOS copies complete binary
 * messages; only the kernel-owned network task calls lwIP/MQTT.
 */
#ifndef SUBSYSTEM1_H
#define SUBSYSTEM1_H

#include "../app.h"
#include "telemetry_codec.h"

#define BUDDY1_QUEUE_MESSAGES 4
#define BUDDY1_COMMAND_MESSAGES 2
#define S1_PRI_COMMAND 22

/* Four-bit topic ID; upper bits zero. Fixed slots preserve the four-message
 * capacity. MQTT sends only payload_size(type) bytes, never the whole slot. */
typedef struct
{
    UB type;
    UB payload[BUDDY1_PAYLOAD_MAX];
} buddy1_telemetry_msg_t;

typedef enum
{
    BUDDY1_COMMAND_UNKNOWN = 0,
    BUDDY1_COMMAND_SNAPSHOT,
    BUDDY1_COMMAND_BARCODE_MAP
} buddy1_command_kind_t;

typedef struct
{
    UB kind; /* Two bits select the fixed packet length. */
    UB payload[BUDDY1_MAP_BYTES];
} buddy1_command_msg_t;

extern ID gh_buddy1_commands;
extern ID gh_buddy1_state_mutex;
extern ER buddy1_report_motion (H left_mms, H right_mms);
extern ER buddy1_report_barcode (UB decoded_letter);
extern ER buddy1_report_hump (UH hump_count, UH height_mm);
extern ER buddy1_report_heading (H heading_deg);
extern ER buddy1_report_obstacle (UH distance_mm, UH width_mm,
                                  telemetry_codec_route_t route);
/* Live route pose: distance since START modulo 4096 cm, heading 0-359 deg
 * since START (+ anticlockwise). Not cached in the snapshot. */
extern ER buddy1_report_pose (UH distance_cm_mod, H heading_deg);
/* Live Buddy 4 terrain: tilt, terrain state, live climb height and this
 * run's highest hump (see MQTT_PROTOCOL.md). Unavailable values must be zero;
 * out-of-range values are rejected (E_PAR). Not cached. */
extern ER buddy1_report_terrain (const telemetry_codec_terrain_t * p_terrain);
/* Live Buddy 5 status (5 Hz) and obstacle profile (per scan/peek); see
 * MQTT_PROTOCOL.md. Out-of-range values are rejected (E_PAR). Not cached. */
extern ER
buddy1_report_avoidance (const telemetry_codec_avoidance_t * p_avoidance);
extern ER buddy1_report_scan (const telemetry_codec_scan_t * p_scan);
/* One reading of the last scan; call once per reading after its profile. */
extern ER
buddy1_report_scan_sample (const telemetry_codec_scan_sample_t * p_sample);
/* TRUE when nothing is waiting in the outbox. Optional bulk data (raw scan
 * readings) is sent only then, so it never takes a slot an event needs. */
extern BOOL buddy1_outbox_idle (void);
extern ER   buddy1_report_robot_state (telemetry_codec_state_t state);
/* Central protected telemetry fault mirror. Each caller updates only its
 * owned bits; active is a subset of changed. Task context only. Controller
 * safety decisions remain separate from this best-effort telemetry mirror. */
extern ER buddy1_update_faults (UB changed, UB active);
extern ER buddy1_invalidate (UB telemetry_type);
extern ER buddy1_queue_publish (UB type, const UB * p_payload, UH payload_len);
extern BOOL buddy1_outbox_take (buddy1_telemetry_msg_t * p_out);
extern ER   buddy1_command_enqueue (const char * p_topic, const UB * p_payload,
                                    UH payload_len);
extern ER   buddy1_command_process (const buddy1_command_msg_t * p_command);
/* Same, but a full outbox is retried every BUDDY1_REPLY_RETRY_MS, up to
 * `tries` attempts (the command task uses BUDDY1_REPLY_TRIES = 2 s, well
 * inside the dashboard's 8 s reply wait). Task context only (it may sleep). */
#define BUDDY1_REPLY_RETRY_MS 10
#define BUDDY1_REPLY_TRIES 200U
extern ER buddy1_command_process_retry (const buddy1_command_msg_t * p_command,
                                        UINT                         tries);
extern BOOL buddy1_motion_snapshot (H * p_left_mms, H * p_right_mms);
extern BOOL buddy1_snapshot (telemetry_codec_snapshot_t * p_out);
extern BOOL buddy1_snapshot_packet (UB p_packet[BUDDY1_PAYLOAD_MAX]);
extern buddy1_command_kind_t buddy1_command_kind (const char * p_topic);
extern BOOL buddy1_parse_barcode_map (const UB * p_payload, UH length,
                                      telemetry_codec_map_t * p_out);
extern ER   buddy1_apply_barcode_map (const telemetry_codec_map_t * p_map);
extern telemetry_codec_action_t buddy1_barcode_action (UB decoded_letter);
/* Close once before the first run; the map remains frozen until reboot. */
extern BOOL buddy1_configuration_allowed (void);
extern ER   buddy1_configuration_close (void);
extern ER   buddy1_config_snapshot_close (telemetry_codec_map_t * p_map);
/* Same, and reports whether this call froze the map (TRUE) or it was already
 * frozen by an earlier run (FALSE). newly_closed may be NULL. */
extern ER buddy1_configuration_claim (telemetry_codec_map_t * p_map,
                                      BOOL *                  pb_did_close);
/* Undo only a close that the caller's own rejected START newly took (claim
 * reported TRUE), so no run ever used the map. Never undo an earlier freeze. */
extern ER   buddy1_configuration_reopen (void);
extern void buddy1_command_task (INT stacd, void * p_exinf);
#endif

/* End of file. */

