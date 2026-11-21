/** @file command.c
 * @brief Barcode configuration and queued command processing.
 * The selected byte and half-word operands fit a 32-bit int. The casts
 * spell out the signed promotion already used by these comparisons.
 * Command and map operations release a mutex acquired in the same call.
 * Release status is discarded so it cannot replace the operation result.
 */
#include "command.h"

/* Packed default map A/B/C/D, produced by buddy1_map_pack(). */
static UB g_active_map[BUDDY1_MAP_BYTES] = {BUDDY1_WIRE_VERSION, 0x20, 0x88,
                                            0x01};

static BOOL topics_equal (const char * p_first, const char * p_second);
static ER   command_reply (const buddy1_command_msg_t * p_command,
                           UB p_reply[BUDDY1_PAYLOAD_MAX], UB * p_type,
                           UH * p_length);

/* Bit 7 of the cached version byte is a LOCAL configuration latch. It is
 * masked off before decoding; it is never transmitted as protocol data. */
/** @brief Read the map freeze latch under the state mutex. A failed lock
 * returns FALSE.
 */
BOOL buddy1_configuration_allowed (void)
{

    if (tk_loc_mtx(gh_buddy1_state_mutex, TMO_FEVR) < E_OK)
    {
        return FALSE;
    }

    BOOL b_is_allowed = (g_active_map[0] & 0x80U) == 0U;
    (void)tk_unl_mtx(gh_buddy1_state_mutex);
    return b_is_allowed;
}

/** @brief Set the local map freeze latch under the state mutex. Return the
 * lock error or E_OK.
 */
ER buddy1_configuration_close (void)
{
    ER value_er = tk_loc_mtx(gh_buddy1_state_mutex, TMO_FEVR);

    if (value_er < E_OK)
    {
        return value_er;
    }

    g_active_map[0] |= 0x80U;
    (void)tk_unl_mtx(gh_buddy1_state_mutex);
    return E_OK;
}

/** @brief Copy and decode the current map and freeze it under the same lock.
 * Optionally report whether this call acquired a new freeze. Null map,
 * lock failure and decode failure return an error.
 */
ER buddy1_configuration_claim (telemetry_codec_map_t * p_map,
                               BOOL *                  pb_did_close)
{
    UB packed[BUDDY1_MAP_BYTES];

    if (0 == p_map)
    {
        return E_PAR;
    }

    ER error = tk_loc_mtx(gh_buddy1_state_mutex, TMO_FEVR);

    if (error < E_OK)
    {
        return error;
    }

    for (unsigned byte_index = 0U; byte_index < sizeof(packed); byte_index++)
    {
        packed[byte_index] = g_active_map[byte_index];
    }

    packed[0] &= 0x7fU;

    if (!telemetry_codec_map_unpack(packed, sizeof(packed), p_map))
    {
        (void)tk_unl_mtx(gh_buddy1_state_mutex);
        return E_SYS;
    }
    /* Read and set the latch under the same lock: the caller learns whether
     * THIS call froze the map or it was already frozen by an earlier run. */
    if (pb_did_close)
    {
        *pb_did_close = (g_active_map[0] & 0x80U) == 0U;
    }

    g_active_map[0] |= 0x80U;
    (void)tk_unl_mtx(gh_buddy1_state_mutex);
    return E_OK;
}

/** @brief Read and freeze the current map atomically. Return the claim result
 * and discard the optional ownership flag.
 */
ER buddy1_config_snapshot_close (telemetry_codec_map_t * p_map)
{
    return buddy1_configuration_claim(p_map, NULL);
}

/* Only the sampling owner calls this, and only to undo a close that its own
 * rejected START newly took (claim reported newly_closed) before any run
 * began. Never call it to undo an earlier run's freeze. Map contents unchanged.
 */
/** @brief Release a new freeze owned by a rejected START before a run begins.
 * The sampling owner calls this only for its own claim. Return the
 * lock result.
 */
ER buddy1_configuration_reopen (void)
{
    ER value_er = tk_loc_mtx(gh_buddy1_state_mutex, TMO_FEVR);

    if (value_er < E_OK)
    {
        return value_er;
    }

    g_active_map[0] &= 0x7fU;
    (void)tk_unl_mtx(gh_buddy1_state_mutex);
    return E_OK;
}

/** @brief Classify a terminated MQTT topic as a snapshot or barcode-map
 * request. Return UNKNOWN for a null or unrecognized topic.
 */
buddy1_command_kind_t buddy1_command_kind (const char * p_topic)
{
    if (0 == p_topic)
    {
        return BUDDY1_COMMAND_UNKNOWN;
    }

    if (topics_equal(p_topic, "car/cmd/snapshot"))
    {
        return BUDDY1_COMMAND_SNAPSHOT;
    }

    if (topics_equal(p_topic, "car/cmd/barcode_map"))
    {
        return BUDDY1_COMMAND_BARCODE_MAP;
    }

    return BUDDY1_COMMAND_UNKNOWN;
}

/** @brief Decode the supplied payload of length bytes into the caller's map
 * and return the map codec's validation result.
 */
BOOL buddy1_parse_barcode_map (const UB * p_payload, UH length,
                               telemetry_codec_map_t * p_out)
{
    return telemetry_codec_map_unpack(p_payload, length, p_out);
}

/** @brief Encode and validate the proposed map before locking. A frozen
 * configuration returns E_OBJ. Only an accepted map replaces the
 * cached bytes.
 */
ER buddy1_apply_barcode_map (const telemetry_codec_map_t * p_map)
{
    UB packed[BUDDY1_MAP_BYTES];

    if (!buddy1_map_pack(p_map, packed))
    {
        return E_PAR;
    }

    ER value_er = tk_loc_mtx(gh_buddy1_state_mutex, TMO_FEVR);

    if (value_er < E_OK)
    {
        return value_er;
    }

    if ((g_active_map[0] & 0x80U) != 0U)
    {
        (void)tk_unl_mtx(gh_buddy1_state_mutex);
        return E_OBJ;
    }

    unsigned byte_index;
    for (byte_index = 0; byte_index < sizeof(g_active_map); ++byte_index)
    {
        g_active_map[byte_index] = packed[byte_index];
    }

    (void)tk_unl_mtx(gh_buddy1_state_mutex);
    return E_OK;
}

/** @brief Copy the map under the mutex, then resolve the letter in forward,
 * left, right and U-turn order. Unmapped letters, decode failure or
 * lock failure give UNKNOWN.
 */
telemetry_codec_action_t buddy1_barcode_action (UB decoded_letter)
{
    UB packed[BUDDY1_MAP_BYTES];

    if (tk_loc_mtx(gh_buddy1_state_mutex, TMO_FEVR) < E_OK)
    {
        return BUDDY1_ACTION_UNKNOWN;
    }

    unsigned byte_index;
    for (byte_index = 0; byte_index < sizeof(g_active_map); ++byte_index)
    {
        packed[byte_index] = g_active_map[byte_index];
    }

    (void)tk_unl_mtx(gh_buddy1_state_mutex);
    packed[0] &= 0x7fU;

    telemetry_codec_map_t map;
    if (!telemetry_codec_map_unpack(packed, sizeof(packed), &map))
    {
        return BUDDY1_ACTION_UNKNOWN;
    }

    if (decoded_letter == map.forward)
    {
        return BUDDY1_ACTION_FORWARD;
    }

    if (decoded_letter == map.left)
    {
        return BUDDY1_ACTION_LEFT;
    }

    if (decoded_letter == map.right)
    {
        return BUDDY1_ACTION_RIGHT;
    }

    if (decoded_letter == map.uturn)
    {
        return BUDDY1_ACTION_UTURN;
    }

    return BUDDY1_ACTION_UNKNOWN;
}

/** @brief Validate a received topic and payload and poll the bounded command
 * buffer. Invalid data returns E_PAR before queuing.
 */
ER buddy1_command_enqueue (const char * p_topic, const UB * p_payload,
                           UH payload_len)
{
    buddy1_command_msg_t message = {0};
    message.kind                 = (UB)buddy1_command_kind(p_topic);

    if ((0 == p_payload) || (BUDDY1_COMMAND_UNKNOWN == (int)message.kind))
    {
        return E_PAR;
    }

    telemetry_codec_map_t parsed;
    if (BUDDY1_COMMAND_SNAPSHOT == (int)message.kind)
    {
        if ((payload_len != 1U) || (p_payload[0] != BUDDY1_WIRE_VERSION))
        {
            return E_PAR;
        }
    }
    else if (!telemetry_codec_map_unpack(p_payload, payload_len, &parsed))
    {
        return E_PAR;
    }
    else
    {
        /* The decoded map can now be copied into the command queue. */
    }

    unsigned byte_index;
    for (byte_index = 0; byte_index < payload_len; ++byte_index)
    {
        message.payload[byte_index] = p_payload[byte_index];
    }

    return tk_snd_mbf(gh_buddy1_commands, &message, sizeof(message), TMO_POL);
}

/** @brief Execute one command and attempt a single reply publication. Return
 * the processing or queue result.
 */
ER buddy1_command_process (const buddy1_command_msg_t * p_command)
{
    return buddy1_command_process_retry(p_command, 1U);
}

/* The reply shares the 4-slot outbox with live telemetry. While it is full
 * (E_TMOUT), wait BUDDY1_REPLY_RETRY_MS and try again, up to `tries` attempts,
 * instead of dropping the reply. The command itself is never repeated. */
/** @brief Build the reply once. Retry only queue timeouts, delaying between
 * attempts until the given limit. Always attempt a first send,
 * including when tries is zero.
 */
ER buddy1_command_process_retry (const buddy1_command_msg_t * p_command,
                                 UINT                         tries)
{
    UB   reply[BUDDY1_PAYLOAD_MAX];
    UB   type     = 0;
    UH   length   = 0;
    UINT attempt  = 1U;
    ER   value_er = command_reply(p_command, reply, &type, &length);

    if (value_er != E_OK)
    {
        return value_er;
    }

    for (;;)
    {
        value_er = buddy1_queue_publish(type, reply, length);

        if ((value_er != E_TMOUT) || (attempt >= tries))
        {
            return value_er;
        }

        /* The delay status does not change the bounded attempt count. */
        (void)tk_dly_tsk(BUDDY1_REPLY_RETRY_MS);
        ++attempt;
    }
}

/** @brief Wait for a full command message and process it with the configured
 * reply retry limit. The kernel start code and argument are unused.
 */
void buddy1_command_task (INT stacd, void * p_exinf)
{
    (void)stacd;
    (void)p_exinf;

    buddy1_command_msg_t command;
    for (;;)
    {
        if (tk_rcv_mbf(gh_buddy1_commands, &command, TMO_FEVR) ==
            (INT)sizeof(command))
        {
            /* Retry is handled within the helper. Resume receiving for
             * every result it returns. */
            (void)buddy1_command_process_retry(&command, BUDDY1_REPLY_TRIES);
        }
    }
}

/** @brief Compare two non-null, terminated topic strings, byte by byte,
 * through the first mismatch or terminator.
 */
static BOOL topics_equal (const char * p_first, const char * p_second)
{
    while ((*p_first) && (*p_first == *p_second))
    {
        ++p_first;
        ++p_second;
    }

    return *p_first == *p_second;
}

/* Carry out one command (apply the map / take the snapshot) exactly once and
 * build its reply. */
/** @brief Execute one dequeued command and construct its reply in caller
 * storage. Snapshot and map replies have distinct lengths and topics.
 * Invalid commands return an error before publication.
 */
static ER command_reply (const buddy1_command_msg_t * p_command,
                         UB p_reply[BUDDY1_PAYLOAD_MAX], UB * p_type,
                         UH * p_length)
{

    if (0 == p_command)
    {
        return E_PAR;
    }

    if (BUDDY1_COMMAND_SNAPSHOT == (int)p_command->kind)
    {
        if (p_command->payload[0] != BUDDY1_WIRE_VERSION)
        {
            return E_PAR;
        }

        if (!buddy1_snapshot_packet(p_reply))
        {
            return E_SYS;
        }

        *p_type   = BUDDY1_TOPIC_SNAPSHOT;
        *p_length = BUDDY1_PAYLOAD_MAX;
        return E_OK;
    }

    if ((int)p_command->kind != BUDDY1_COMMAND_BARCODE_MAP)
    {
        return E_PAR;
    }

    p_reply[0] = BUDDY1_WIRE_VERSION;

    telemetry_codec_map_t map;
    ER                    value_er;
    if (!telemetry_codec_map_unpack(p_command->payload,
                                    sizeof(p_command->payload), &map))
    {
        p_reply[1] = BUDDY1_REPLY_INVALID;
    }
    else
    {
        value_er   = buddy1_apply_barcode_map(&map);
        p_reply[1] = E_OK == value_er ? BUDDY1_REPLY_OK
                     : E_OBJ == value_er ? BUDDY1_REPLY_BUSY
                                         : BUDDY1_REPLY_INTERNAL;
    }

    *p_type   = BUDDY1_TOPIC_MAP_REPLY;
    *p_length = 2;
    return E_OK;
}

/* End of file. */

