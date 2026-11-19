#include "command.h"

/* Packed default map A/B/C/D, produced by buddy1_map_pack(). */
LOCAL UB g_active_map[BUDDY1_MAP_BYTES] = {BUDDY1_WIRE_VERSION, 0x20, 0x88,
                                           0x01};

/* Bit 7 of the cached version byte is a LOCAL configuration latch. It is
 * masked off before decoding; it is never transmitted as protocol data. */
EXPORT BOOL buddy1_configuration_allowed (void)
{
    BOOL allowed;
    if (tk_loc_mtx(g_mtx_buddy1_state, TMO_FEVR) < E_OK)
    {
        return FALSE;
    }
    allowed = (g_active_map[0] & 0x80U) == 0U;
    (void)tk_unl_mtx(g_mtx_buddy1_state);
    return allowed;
}

EXPORT ER buddy1_configuration_close (void)
{
    ER value_er = tk_loc_mtx(g_mtx_buddy1_state, TMO_FEVR);
    if (value_er < E_OK)
    {
        return value_er;
    }
    g_active_map[0] |= 0x80U;
    (void)tk_unl_mtx(g_mtx_buddy1_state);
    return E_OK;
}

EXPORT ER buddy1_configuration_claim (buddy1_barcode_map_t * p_map,
                                      BOOL * p_newly_closed)
{
    UB packed[BUDDY1_MAP_BYTES];
    ER error;
    if (!p_map)
    {
        return E_PAR;
    }
    error = tk_loc_mtx(g_mtx_buddy1_state, TMO_FEVR);
    if (error < E_OK)
    {
        return error;
    }
    for (unsigned value_i = 0U; value_i < sizeof(packed); value_i++)
    {
        packed[value_i] = g_active_map[value_i];
    }
    packed[0] &= 0x7fU;
    if (!buddy1_map_unpack(packed, sizeof(packed), p_map))
    {
        (void)tk_unl_mtx(g_mtx_buddy1_state);
        return E_SYS;
    }
    /* Read and set the latch under the same lock: the caller learns whether
     * THIS call froze the map or it was already frozen by an earlier run. */
    if (p_newly_closed)
    {
        *p_newly_closed = (g_active_map[0] & 0x80U) == 0U;
    }
    g_active_map[0] |= 0x80U;
    (void)tk_unl_mtx(g_mtx_buddy1_state);
    return E_OK;
}

EXPORT ER buddy1_config_snapshot_close (buddy1_barcode_map_t * p_map)
{
    return buddy1_configuration_claim(p_map, NULL);
}

/* Only the sampling owner calls this, and only to undo a close that its own
 * rejected START newly took (claim reported newly_closed) before any run
 * began. Never call it to undo an earlier run's freeze. Map contents unchanged.
 */
EXPORT ER buddy1_configuration_reopen (void)
{
    ER value_er = tk_loc_mtx(g_mtx_buddy1_state, TMO_FEVR);
    if (value_er < E_OK)
    {
        return value_er;
    }
    g_active_map[0] &= 0x7fU;
    (void)tk_unl_mtx(g_mtx_buddy1_state);
    return E_OK;
}

LOCAL BOOL equal (const char * p_a, const char * p_b)
{
    while (*p_a && *p_a == *p_b)
    {
        ++p_a;
        ++p_b;
    }
    return *p_a == *p_b;
}

EXPORT buddy1_command_kind_t buddy1_command_kind (const char * p_topic)
{
    if (!p_topic)
    {
        return BUDDY1_COMMAND_UNKNOWN;
    }
    if (equal(p_topic, "car/cmd/snapshot"))
    {
        return BUDDY1_COMMAND_SNAPSHOT;
    }
    if (equal(p_topic, "car/cmd/barcode_map"))
    {
        return BUDDY1_COMMAND_BARCODE_MAP;
    }
    return BUDDY1_COMMAND_UNKNOWN;
}

EXPORT BOOL buddy1_parse_barcode_map (const UB * p_payload, UH length,
                                      buddy1_barcode_map_t * p_out)
{
    return buddy1_map_unpack(p_payload, length, p_out);
}

EXPORT ER buddy1_apply_barcode_map (const buddy1_barcode_map_t * p_map)
{
    UB packed[BUDDY1_MAP_BYTES];
    unsigned value_i;
    ER value_er;
    if (!buddy1_map_pack(p_map, packed))
    {
        return E_PAR;
    }
    value_er = tk_loc_mtx(g_mtx_buddy1_state, TMO_FEVR);
    if (value_er < E_OK)
    {
        return value_er;
    }
    if ((g_active_map[0] & 0x80U) != 0U)
    {
        (void)tk_unl_mtx(g_mtx_buddy1_state);
        return E_OBJ;
    }
    for (value_i = 0; value_i < sizeof(g_active_map); ++value_i)
    {
        g_active_map[value_i] = packed[value_i];
    }
    (void)tk_unl_mtx(g_mtx_buddy1_state);
    return E_OK;
}

EXPORT buddy1_barcode_action_t buddy1_barcode_action (UB letter)
{
    UB packed[BUDDY1_MAP_BYTES];
    buddy1_barcode_map_t map;
    unsigned value_i;
    if (tk_loc_mtx(g_mtx_buddy1_state, TMO_FEVR) < E_OK)
    {
        return BUDDY1_ACTION_UNKNOWN;
    }
    for (value_i = 0; value_i < sizeof(g_active_map); ++value_i)
    {
        packed[value_i] = g_active_map[value_i];
    }
    (void)tk_unl_mtx(g_mtx_buddy1_state);
    packed[0] &= 0x7fU;
    if (!buddy1_map_unpack(packed, sizeof(packed), &map))
    {
        return BUDDY1_ACTION_UNKNOWN;
    }
    if (letter == map.forward)
    {
        return BUDDY1_ACTION_FORWARD;
    }
    if (letter == map.left)
    {
        return BUDDY1_ACTION_LEFT;
    }
    if (letter == map.right)
    {
        return BUDDY1_ACTION_RIGHT;
    }
    if (letter == map.uturn)
    {
        return BUDDY1_ACTION_UTURN;
    }
    return BUDDY1_ACTION_UNKNOWN;
}

EXPORT ER buddy1_command_enqueue (const char * p_topic, const UB * p_payload,
                                  UH length)
{
    buddy1_command_msg_t message = {0};
    buddy1_barcode_map_t parsed;
    unsigned value_i;
    message.kind = (UB)buddy1_command_kind(p_topic);
    if (!p_payload || message.kind == BUDDY1_COMMAND_UNKNOWN)
    {
        return E_PAR;
    }
    if (message.kind == BUDDY1_COMMAND_SNAPSHOT)
    {
        if (length != 1 || p_payload[0] != BUDDY1_WIRE_VERSION)
        {
            return E_PAR;
        }
    }
    else if (!buddy1_map_unpack(p_payload, length, &parsed))
    {
        return E_PAR;
    }
    for (value_i = 0; value_i < length; ++value_i)
    {
        message.payload[value_i] = p_payload[value_i];
    }
    return tk_snd_mbf(g_mbf_buddy1_commands, &message, sizeof(message),
                      TMO_POL);
}

/* Carry out one command (apply the map / take the snapshot) exactly once and
 * build its reply. */
LOCAL ER command_reply (const buddy1_command_msg_t * p_command,
                        UB p_reply[BUDDY1_PAYLOAD_MAX], UB * p_type,
                        UH * p_length)
{
    buddy1_barcode_map_t map;
    ER value_er;
    if (!p_command)
    {
        return E_PAR;
    }
    if (p_command->kind == BUDDY1_COMMAND_SNAPSHOT)
    {
        if (p_command->payload[0] != BUDDY1_WIRE_VERSION)
        {
            return E_PAR;
        }
        if (!buddy1_snapshot_packet(p_reply))
        {
            return E_SYS;
        }
        *p_type = BUDDY1_TOPIC_SNAPSHOT;
        *p_length = BUDDY1_PAYLOAD_MAX;
        return E_OK;
    }
    if (p_command->kind != BUDDY1_COMMAND_BARCODE_MAP)
    {
        return E_PAR;
    }
    p_reply[0] = BUDDY1_WIRE_VERSION;
    if (!buddy1_map_unpack(p_command->payload, sizeof(p_command->payload),
                           &map))
    {
        p_reply[1] = BUDDY1_REPLY_INVALID;
    }
    else
    {
        value_er = buddy1_apply_barcode_map(&map);
        p_reply[1] = value_er == E_OK    ? BUDDY1_REPLY_OK
                     : value_er == E_OBJ ? BUDDY1_REPLY_BUSY
                                         : BUDDY1_REPLY_INTERNAL;
    }
    *p_type = BUDDY1_TOPIC_MAP_REPLY;
    *p_length = 2;
    return E_OK;
}

EXPORT ER buddy1_command_process (const buddy1_command_msg_t * p_command)
{
    return buddy1_command_process_retry(p_command, 1U);
}

/* The reply shares the 4-slot outbox with live telemetry. While it is full
 * (E_TMOUT), wait BUDDY1_REPLY_RETRY_MS and try again, up to `tries` attempts,
 * instead of dropping the reply. The command itself is never repeated. */
EXPORT ER buddy1_command_process_retry (const buddy1_command_msg_t * p_command,
                                        UINT tries)
{
    UB reply[BUDDY1_PAYLOAD_MAX];
    UB type = 0;
    UH length = 0;
    UINT attempt;
    ER value_er = command_reply(p_command, reply, &type, &length);
    if (value_er != E_OK)
    {
        return value_er;
    }
    for (attempt = 1U;; ++attempt)
    {
        value_er = buddy1_queue_publish(type, reply, length);
        if (value_er != E_TMOUT || attempt >= tries)
        {
            return value_er;
        }
        (void)tk_dly_tsk(BUDDY1_REPLY_RETRY_MS);
    }
}

EXPORT void buddy1_command_task (INT stacd, void * p_exinf)
{
    buddy1_command_msg_t command;
    (void)stacd;
    (void)p_exinf;
    for (;;)
    {
        if (tk_rcv_mbf(g_mbf_buddy1_commands, &command, TMO_FEVR) ==
            (INT)sizeof(command))
        {
            (void)buddy1_command_process_retry(&command, BUDDY1_REPLY_TRIES);
        }
    }
}
