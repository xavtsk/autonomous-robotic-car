#include "subsystem1.h"

#define ALIGN_TO_INT(value) (((value) + sizeof(INT) - 1) & ~(sizeof(INT) - 1))
#define MESSAGE_STORAGE(type) (sizeof(INT) + ALIGN_TO_INT(sizeof(type)))

EXPORT ID mbf_buddy1_outbox;
EXPORT ID mbf_buddy1_commands;
EXPORT ID mtx_buddy1_state;
/* Same compact layout as the snapshot reply. All access holds the mutex. */
LOCAL UB latest[BUDDY1_PAYLOAD_MAX] = {BUDDY1_WIRE_VERSION};

EXPORT ER subsystem1_init(void)
{
    T_CMBF outbox = {0}, commands = {0};
    T_CMTX state_mutex = {0};
    if (mbf_buddy1_outbox > 0 || mbf_buddy1_commands > 0 || mtx_buddy1_state > 0) return E_OBJ;
    outbox.mbfatr = TA_TFIFO;
    outbox.bufsz = BUDDY1_QUEUE_MESSAGES * MESSAGE_STORAGE(BUDDY1_TELEMETRY_MSG);
    outbox.maxmsz = sizeof(BUDDY1_TELEMETRY_MSG);
    mbf_buddy1_outbox = tk_cre_mbf(&outbox);
    if (mbf_buddy1_outbox < E_OK) return mbf_buddy1_outbox;
    commands.mbfatr = TA_TFIFO;
    commands.bufsz = BUDDY1_COMMAND_MESSAGES * MESSAGE_STORAGE(BUDDY1_COMMAND_MSG);
    commands.maxmsz = sizeof(BUDDY1_COMMAND_MSG);
    mbf_buddy1_commands = tk_cre_mbf(&commands);
    if (mbf_buddy1_commands < E_OK) {
        ER er = mbf_buddy1_commands;
        (void)tk_del_mbf(mbf_buddy1_outbox);
        mbf_buddy1_outbox = mbf_buddy1_commands = 0;
        return er;
    }
    state_mutex.mtxatr = TA_INHERIT;
    mtx_buddy1_state = tk_cre_mtx(&state_mutex);
    if (mtx_buddy1_state < E_OK) {
        ER er = mtx_buddy1_state;
        (void)tk_del_mbf(mbf_buddy1_commands);
        (void)tk_del_mbf(mbf_buddy1_outbox);
        mbf_buddy1_commands = mbf_buddy1_outbox = mtx_buddy1_state = 0;
        return er;
    }
    return E_OK;
}

EXPORT ER subsystem1_start(void)
{
#if defined(TM_MQTT) && TM_MQTT
    ID tid = app_start_task(buddy1_command_task, S1_PRI_COMMAND, 0, "buddy1_command_task");
    return tid < E_OK ? tid : E_OK;
#else
    return E_OK;
#endif
}

EXPORT ER buddy1_queue_publish(UB type, const UB *payload, UH payload_len)
{
    BUDDY1_TELEMETRY_MSG message = {0};
    UH i;
    if (!buddy1_payload_valid(type, payload, payload_len)) return E_PAR;
    message.type = type;
    for (i = 0; i < payload_len; ++i) message.payload[i] = payload[i];
    /* TMO_POL: the caller never waits for buffer space or for MQTT. */
    return tk_snd_mbf(mbf_buddy1_outbox, &message, sizeof(message), TMO_POL);
}

LOCAL ER store_section(UB type, const BUDDY1_SNAPSHOT *v)
{
    BUDDY1_SNAPSHOT cache;
    if (!buddy1_decode(BUDDY1_TOPIC_SNAPSHOT, latest, sizeof(latest), &cache)) return E_SYS;
    switch (type) {
    case BUDDY1_TOPIC_MOTION:
        cache.motion_valid = 1; cache.left_mms = v->left_mms; cache.right_mms = v->right_mms; break;
    case BUDDY1_TOPIC_BARCODE:
        cache.barcode_valid = 1; cache.barcode_letter = v->barcode_letter;
        cache.barcode_action = v->barcode_action; break;
    case BUDDY1_TOPIC_HUMP:
        cache.hump_valid = 1; cache.hump_count = v->hump_count;
        cache.hump_height_mm = v->hump_height_mm; break;
    case BUDDY1_TOPIC_HEADING:
        cache.heading_valid = 1; cache.heading_deg = v->heading_deg; break;
    case BUDDY1_TOPIC_OBSTACLE:
        cache.obstacle_valid = 1; cache.obstacle_distance_mm = v->obstacle_distance_mm;
        cache.obstacle_width_mm = v->obstacle_width_mm; cache.obstacle_route = v->obstacle_route; break;
    case BUDDY1_TOPIC_STATE:
        cache.robot_state_valid = 1; cache.robot_state = v->robot_state; break;
    default: return E_PAR;
    }
    return buddy1_encode(BUDDY1_TOPIC_SNAPSHOT, &cache, latest, sizeof(latest)) ? E_OK : E_PAR;
}

LOCAL ER report(UB type, const BUDDY1_SNAPSHOT *v)
{
    UB payload[BUDDY1_PAYLOAD_MAX];
    unsigned length = buddy1_encode(type, v, payload, sizeof(payload));
    ER er;
    if (!length) return E_PAR;
    /* Pose is live-only: it has no snapshot section, so skip the cache. */
    if (type != BUDDY1_TOPIC_POSE) {
        er = tk_loc_mtx(mtx_buddy1_state, TMO_FEVR);
        if (er < E_OK) return er;
        er = store_section(type, v);
        (void)tk_unl_mtx(mtx_buddy1_state);
        if (er < E_OK) return er;
    }
    return buddy1_queue_publish(type, payload, (UH)length);
}

EXPORT ER buddy1_report_motion(H left_mms, H right_mms)
{
    BUDDY1_SNAPSHOT v = {0}; v.left_mms = left_mms; v.right_mms = right_mms;
    return report(BUDDY1_TOPIC_MOTION, &v);
}
EXPORT ER buddy1_report_barcode(UB letter)
{
    BUDDY1_SNAPSHOT v = {0};
    v.barcode_letter = letter; v.barcode_action = buddy1_barcode_action(letter);
    return report(BUDDY1_TOPIC_BARCODE, &v);
}
EXPORT ER buddy1_report_hump(UH count, UH height_mm)
{
    BUDDY1_SNAPSHOT v = {0}; v.hump_count = count; v.hump_height_mm = height_mm;
    return report(BUDDY1_TOPIC_HUMP, &v);
}
EXPORT ER buddy1_report_heading(H heading_deg)
{
    BUDDY1_SNAPSHOT v = {0}; v.heading_deg = heading_deg;
    return report(BUDDY1_TOPIC_HEADING, &v);
}
EXPORT ER buddy1_report_obstacle(UH distance_mm, UH width_mm, BUDDY1_OBSTACLE_ROUTE route)
{
    BUDDY1_SNAPSHOT v = {0}; v.obstacle_distance_mm = distance_mm;
    v.obstacle_width_mm = width_mm; v.obstacle_route = route;
    return report(BUDDY1_TOPIC_OBSTACLE, &v);
}
EXPORT ER buddy1_report_pose(UH distance_cm_mod, H heading_deg)
{
    BUDDY1_SNAPSHOT v = {0};
    v.pose_distance_cm = distance_cm_mod; v.pose_heading_deg = heading_deg;
    return report(BUDDY1_TOPIC_POSE, &v);
}
EXPORT ER buddy1_report_robot_state(BUDDY1_ROBOT_STATE state)
{
    BUDDY1_SNAPSHOT v = {0}; v.robot_state = state;
    return report(BUDDY1_TOPIC_STATE, &v);
}

EXPORT ER buddy1_update_faults(UB changed, UB active)
{
    BUDDY1_SNAPSHOT v;
    UB packet[2];
    ER er;
    if (changed == 0U || (changed & ~BUDDY1_FAULT_MASK) != 0U ||
        (active & ~changed) != 0U) return E_PAR;
    if (mtx_buddy1_state <= 0) return E_OBJ;
    er = tk_loc_mtx(mtx_buddy1_state, TMO_FEVR);
    if (er < E_OK) return er;
    if (!buddy1_decode(BUDDY1_TOPIC_SNAPSHOT, latest, sizeof(latest), &v)) {
        (void)tk_unl_mtx(mtx_buddy1_state); return E_SYS;
    }
    v.faults = (UB)((v.faults & (UB)~changed) | active);
    v.faults_valid = 1;
    (void)buddy1_encode(BUDDY1_TOPIC_SNAPSHOT, &v, latest, sizeof(latest));
    (void)buddy1_encode(BUDDY1_TOPIC_FAULTS, &v, packet, sizeof(packet));
    (void)tk_unl_mtx(mtx_buddy1_state);
    return buddy1_queue_publish(BUDDY1_TOPIC_FAULTS, packet, sizeof(packet));
}

EXPORT ER buddy1_invalidate(UB type)
{
    unsigned i, offset = 7;
    ER er;
    if (type > BUDDY1_TOPIC_FAULTS) return E_PAR;
    er = tk_loc_mtx(mtx_buddy1_state, TMO_FEVR);
    if (er < E_OK) return er;
    latest[1] &= (UB)~(1U << type);
    for (i = 0; i < type; ++i) offset += buddy1_payload_bits((UB)i);
    for (i = 0; i < buddy1_payload_bits(type); ++i) {
        unsigned bit = offset + i;
        latest[1 + bit / 8] &= (UB)~(1U << (bit % 8));
    }
    (void)tk_unl_mtx(mtx_buddy1_state);
    return E_OK;
}

EXPORT BOOL buddy1_snapshot_packet(UB packet[BUDDY1_PAYLOAD_MAX])
{
    unsigned i;
    if (!packet || tk_loc_mtx(mtx_buddy1_state, TMO_FEVR) < E_OK) return FALSE;
    for (i = 0; i < sizeof(latest); ++i) packet[i] = latest[i];
    (void)tk_unl_mtx(mtx_buddy1_state);
    return TRUE;
}
EXPORT BOOL buddy1_snapshot(BUDDY1_SNAPSHOT *out)
{
    UB packet[BUDDY1_PAYLOAD_MAX];
    return out && buddy1_snapshot_packet(packet) &&
           buddy1_decode(BUDDY1_TOPIC_SNAPSHOT, packet, sizeof(packet), out);
}
EXPORT BOOL buddy1_motion_snapshot(H *left_mms, H *right_mms)
{
    BUDDY1_SNAPSHOT v;
    if (!left_mms || !right_mms || !buddy1_snapshot(&v) || !v.motion_valid) return FALSE;
    *left_mms = v.left_mms; *right_mms = v.right_mms; return TRUE;
}
EXPORT BOOL buddy1_outbox_take(BUDDY1_TELEMETRY_MSG *out)
{
    return out && tk_rcv_mbf(mbf_buddy1_outbox, out, TMO_POL) == (INT)sizeof(*out);
}
