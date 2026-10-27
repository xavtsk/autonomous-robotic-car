#include "subsystem1.h"

/* Packed default map A/B/C/D, produced by buddy1_map_pack(). */
LOCAL UB active_map[BUDDY1_MAP_BYTES] = {BUDDY1_WIRE_VERSION, 0x20, 0x88, 0x01};

/* Bit 7 of the cached version byte is a LOCAL configuration latch. It is
 * masked off before decoding; it is never transmitted as protocol data. */
EXPORT BOOL buddy1_configuration_allowed(void)
{
    BOOL allowed;
    if (tk_loc_mtx(mtx_buddy1_state, TMO_FEVR) < E_OK) return FALSE;
    allowed = (active_map[0] & 0x80U) == 0U;
    (void)tk_unl_mtx(mtx_buddy1_state);
    return allowed;
}

EXPORT ER buddy1_configuration_close(void)
{
    ER er = tk_loc_mtx(mtx_buddy1_state, TMO_FEVR);
    if (er < E_OK) return er;
    active_map[0] |= 0x80U;
    (void)tk_unl_mtx(mtx_buddy1_state);
    return E_OK;
}

EXPORT ER buddy1_configuration_claim(BUDDY1_BARCODE_MAP *map, BOOL *newly_closed)
{
    UB packed[BUDDY1_MAP_BYTES];
    ER error;
    if (!map) return E_PAR;
    error=tk_loc_mtx(mtx_buddy1_state,TMO_FEVR);
    if (error < E_OK) return error;
    for (unsigned i=0U;i<sizeof(packed);i++) packed[i]=active_map[i];
    packed[0]&=0x7fU;
    if (!buddy1_map_unpack(packed,sizeof(packed),map)) {
        (void)tk_unl_mtx(mtx_buddy1_state); return E_SYS;
    }
    /* Read and set the latch under the same lock: the caller learns whether
     * THIS call froze the map or it was already frozen by an earlier run. */
    if (newly_closed) *newly_closed = (active_map[0] & 0x80U) == 0U;
    active_map[0]|=0x80U;
    (void)tk_unl_mtx(mtx_buddy1_state);
    return E_OK;
}

EXPORT ER buddy1_configuration_snapshot_and_close(BUDDY1_BARCODE_MAP *map)
{
    return buddy1_configuration_claim(map, NULL);
}

/* Only the sampling owner calls this, and only to undo a close that its own
 * rejected START newly took (claim reported newly_closed) before any run
 * began. Never call it to undo an earlier run's freeze. Map contents unchanged. */
EXPORT ER buddy1_configuration_reopen(void)
{
    ER er = tk_loc_mtx(mtx_buddy1_state, TMO_FEVR);
    if (er < E_OK) return er;
    active_map[0] &= 0x7fU;
    (void)tk_unl_mtx(mtx_buddy1_state);
    return E_OK;
}

LOCAL BOOL equal(const char *a, const char *b)
{
    while (*a && *a == *b) { ++a; ++b; }
    return *a == *b;
}

EXPORT BUDDY1_COMMAND_KIND buddy1_command_kind(const char *topic)
{
    if (!topic) return BUDDY1_COMMAND_UNKNOWN;
    if (equal(topic, "car/cmd/snapshot")) return BUDDY1_COMMAND_SNAPSHOT;
    if (equal(topic, "car/cmd/barcode_map")) return BUDDY1_COMMAND_BARCODE_MAP;
    return BUDDY1_COMMAND_UNKNOWN;
}

EXPORT BOOL buddy1_parse_barcode_map(const UB *payload, UH length, BUDDY1_BARCODE_MAP *out)
{
    return buddy1_map_unpack(payload, length, out);
}

EXPORT ER buddy1_apply_barcode_map(const BUDDY1_BARCODE_MAP *map)
{
    UB packed[BUDDY1_MAP_BYTES];
    unsigned i;
    ER er;
    if (!buddy1_map_pack(map, packed)) return E_PAR;
    er = tk_loc_mtx(mtx_buddy1_state, TMO_FEVR);
    if (er < E_OK) return er;
    if ((active_map[0] & 0x80U) != 0U) {
        (void)tk_unl_mtx(mtx_buddy1_state); return E_OBJ;
    }
    for (i = 0; i < sizeof(active_map); ++i) active_map[i] = packed[i];
    (void)tk_unl_mtx(mtx_buddy1_state);
    return E_OK;
}

EXPORT BUDDY1_BARCODE_ACTION buddy1_barcode_action(UB letter)
{
    UB packed[BUDDY1_MAP_BYTES];
    BUDDY1_BARCODE_MAP map;
    unsigned i;
    if (tk_loc_mtx(mtx_buddy1_state, TMO_FEVR) < E_OK) return BUDDY1_ACTION_UNKNOWN;
    for (i = 0; i < sizeof(active_map); ++i) packed[i] = active_map[i];
    (void)tk_unl_mtx(mtx_buddy1_state);
    packed[0] &= 0x7fU;
    if (!buddy1_map_unpack(packed, sizeof(packed), &map)) return BUDDY1_ACTION_UNKNOWN;
    if (letter == map.forward) return BUDDY1_ACTION_FORWARD;
    if (letter == map.left) return BUDDY1_ACTION_LEFT;
    if (letter == map.right) return BUDDY1_ACTION_RIGHT;
    if (letter == map.uturn) return BUDDY1_ACTION_UTURN;
    return BUDDY1_ACTION_UNKNOWN;
}

EXPORT ER buddy1_command_enqueue(const char *topic, const UB *payload, UH length)
{
    BUDDY1_COMMAND_MSG message = {0};
    BUDDY1_BARCODE_MAP parsed;
    unsigned i;
    message.kind = (UB)buddy1_command_kind(topic);
    if (!payload || message.kind == BUDDY1_COMMAND_UNKNOWN) return E_PAR;
    if (message.kind == BUDDY1_COMMAND_SNAPSHOT) {
        if (length != 1 || payload[0] != BUDDY1_WIRE_VERSION) return E_PAR;
    } else if (!buddy1_map_unpack(payload, length, &parsed)) return E_PAR;
    for (i = 0; i < length; ++i) message.payload[i] = payload[i];
    return tk_snd_mbf(mbf_buddy1_commands, &message, sizeof(message), TMO_POL);
}

EXPORT ER buddy1_command_process(const BUDDY1_COMMAND_MSG *command)
{
    UB reply[BUDDY1_PAYLOAD_MAX];
    BUDDY1_BARCODE_MAP map;
    ER er;
    if (!command) return E_PAR;
    if (command->kind == BUDDY1_COMMAND_SNAPSHOT) {
        if (command->payload[0] != BUDDY1_WIRE_VERSION) return E_PAR;
        if (!buddy1_snapshot_packet(reply)) return E_SYS;
        return buddy1_queue_publish(BUDDY1_TOPIC_SNAPSHOT, reply, sizeof(reply));
    }
    if (command->kind != BUDDY1_COMMAND_BARCODE_MAP) return E_PAR;
    reply[0] = BUDDY1_WIRE_VERSION;
    if (!buddy1_map_unpack(command->payload, sizeof(command->payload), &map)) {
        reply[1] = BUDDY1_REPLY_INVALID;
    } else {
        er = buddy1_apply_barcode_map(&map);
        reply[1] = er == E_OK ? BUDDY1_REPLY_OK :
                   er == E_OBJ ? BUDDY1_REPLY_BUSY : BUDDY1_REPLY_INTERNAL;
    }
    return buddy1_queue_publish(BUDDY1_TOPIC_MAP_REPLY, reply, 2);
}

EXPORT void buddy1_command_task(INT stacd, void *exinf)
{
    BUDDY1_COMMAND_MSG command;
    (void)stacd; (void)exinf;
    for (;;) {
        if (tk_rcv_mbf(mbf_buddy1_commands, &command, TMO_FEVR) == (INT)sizeof(command))
            (void)buddy1_command_process(&command);
    }
}
