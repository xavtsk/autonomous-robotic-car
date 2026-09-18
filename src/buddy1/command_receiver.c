/*
 * command_receiver.c - parsing inbound MQTT commands.
 *
 * The barcode-map parser below is deliberately NOT a general JSON parser.
 * A real one is thousands of lines and we would have to justify every one
 * of them. This reads exactly the one message shape our schema defines,
 * in about sixty lines that can be explained in a viva.
 */

#include "buddy1/command_receiver.h"

#include <string.h>
#include <stddef.h>
#include <stdio.h>
#include <inttypes.h>

/* Longest action name we accept ("STRAIGHT" is 8 characters). */
#define ACTION_NAME_MAX  8u

/* -------------------------------------------------------------------------
 * Topic matching
 * ------------------------------------------------------------------------- */

/* Return a pointer to the text after the final '/' in a topic. */
static const char * topic_last_segment(const char * topic)
{
    const char * slash = strrchr(topic, '/');

    return (NULL != slash) ? (slash + 1) : topic;
}

/*
 * True if the segment immediately before the last one is "cmd".
 * This stops us acting on "grp07/car/telemetry/stop" if someone ever
 * creates such a topic by mistake.
 */
static bool topic_is_under_cmd(const char * topic)
{
    const char * last;
    const char * prev_slash;
    size_t       prev_len;

    last = strrchr(topic, '/');
    if ((NULL == last) || (last == topic))
    {
        return false;
    }

    /* Walk back from the last slash to find the one before it. */
    prev_slash = last - 1;
    while ((prev_slash > topic) && ('/' != *prev_slash))
    {
        prev_slash--;
    }

    if ('/' == *prev_slash)
    {
        prev_slash++;              /* step past the slash onto the segment */
    }

    prev_len = (size_t)(last - prev_slash);

    return ((3u == prev_len) && (0 == strncmp(prev_slash, "cmd", 3u)));
}

cmd_kind_t cmd_kind_from_topic(const char * topic)
{
    const char * segment;

    if (NULL == topic)
    {
        return CMD_NONE;
    }

    if (!topic_is_under_cmd(topic))
    {
        return CMD_NONE;
    }

    segment = topic_last_segment(topic);

    /* A topic like "grp07/car/cmd/" names no command. That is structurally
       malformed rather than unrecognised, so it is CMD_NONE - CMD_UNKNOWN is
       reserved for "you named a command, but not one I know", which is a
       useful thing to report and this is not. */
    if ('\0' == segment[0])
    {
        return CMD_NONE;
    }

    if (0 == strcmp(segment, "barcode_map")) { return CMD_BARCODE_MAP; }
    if (0 == strcmp(segment, "snapshot"))    { return CMD_SNAPSHOT;    }
    /* Under cmd/ but not a command we know. Worth reporting rather than
       ignoring silently - it usually means a typo at the sending end. */
    return CMD_UNKNOWN;
}

/* -------------------------------------------------------------------------
 * Action names
 * ------------------------------------------------------------------------- */

nav_action_t nav_action_from_name(const char * name)
{
    unsigned index;

    if (NULL == name)
    {
        return NAV_NONE;
    }

    /* Walk the same names tele_state_name() prints, so the two directions
       can never disagree. Start at 1 because index 0 is NAV_NONE. */
    for (index = 1u; index < (unsigned)NAV_COUNT; index++)
    {
        if (0 == strcmp(name, nav_action_name((nav_action_t)index)))
        {
            return (nav_action_t)index;
        }
    }

    return NAV_NONE;
}

/* -------------------------------------------------------------------------
 * The barcode map parser
 * ------------------------------------------------------------------------- */

/* Advance past spaces, tabs, newlines and carriage returns. */
static const char * skip_space(const char * p)
{
    while ((' ' == *p) || ('\t' == *p) || ('\n' == *p) || ('\r' == *p))
    {
        p++;
    }
    return p;
}

bool cmd_parse_barcode_map(const char * payload,
                           nav_action_t map_out[ROBOT_BARCODE_COUNT])
{
    nav_action_t scratch[ROBOT_BARCODE_COUNT];
    const char * p;
    unsigned     pairs_found = 0u;
    unsigned     i;

    if ((NULL == payload) || (NULL == map_out))
    {
        return false;
    }

    /* Parse into scratch. map_out is only touched if everything succeeds. */
    for (i = 0u; i < ROBOT_BARCODE_COUNT; i++)
    {
        scratch[i] = NAV_NONE;
    }

    /*
     * The payload must be a complete JSON object: { ... }
     *
     * Checking the braces is not pedantry. If a payload ever arrives
     * truncated, the fragment "{"A":"LEFT"" still contains one perfectly
     * well-formed pair - and without this check we would apply it as if it
     * were the whole mapping, mid-demo, and never know.
     */
    p = skip_space(payload);
    if ('{' != *p)
    {
        return false;
    }
    p++;

    for (;;)
    {
        char         letter;
        char         name[ACTION_NAME_MAX + 1u];
        unsigned     name_len = 0u;
        nav_action_t action;

        p = skip_space(p);

        if ('}' == *p)
        {
            p++;
            break;                 /* object closed - we are done */
        }

        /* Every pair after the first must be separated by a comma. */
        if (pairs_found > 0u)
        {
            if (',' != *p)
            {
                return false;
            }
            p++;
            p = skip_space(p);
        }

        /* ---- "X" : the key ---- */
        if ('"' != *p)
        {
            return false;          /* expected a quoted key */
        }
        p++;

        letter = *p;
        if ((letter < 'A') || (letter > 'Z'))
        {
            return false;          /* keys must be a single capital letter */
        }
        p++;

        if ('"' != *p)
        {
            return false;          /* key was longer than one character */
        }
        p++;

        /* ---- the colon ---- */
        while ((' ' == *p) || ('\t' == *p))
        {
            p++;
        }
        if (':' != *p)
        {
            return false;
        }
        p++;
        while ((' ' == *p) || ('\t' == *p))
        {
            p++;
        }

        /* ---- "ACTION" : the value ---- */
        if ('"' != *p)
        {
            return false;
        }
        p++;

        while (('\0' != *p) && ('"' != *p))
        {
            if (name_len >= ACTION_NAME_MAX)
            {
                return false;      /* longer than any name we know */
            }
            name[name_len] = *p;
            name_len++;
            p++;
        }

        if ('"' != *p)
        {
            return false;          /* ran off the end with no closing quote */
        }
        p++;
        name[name_len] = '\0';

        action = nav_action_from_name(name);
        if (NAV_NONE == action)
        {
            return false;          /* not an action we recognise */
        }

        scratch[(unsigned)(letter - 'A')] = action;
        pairs_found++;
    }

    /* Nothing may follow the closing brace except whitespace. */
    p = skip_space(p);
    if ('\0' != *p)
    {
        return false;
    }

    if (0u == pairs_found)
    {
        return false;              /* an empty map is almost certainly a mistake */
    }

    /* Everything parsed. Commit. */
    for (i = 0u; i < ROBOT_BARCODE_COUNT; i++)
    {
        map_out[i] = scratch[i];
    }

    return true;
}

bool cmd_format_barcode_map_reply(char * out,
                                  size_t out_size,
                                  uint32_t timestamp_ms,
                                  const robot_config_t * config,
                                  cmd_map_reply_t result)
{
    static const char * const reasons[] =
    {
        "",
        "malformed_payload",
        "configuration_locked",
        "invalid_map"
    };
    int written;

    if ((NULL == out) || (0u == out_size) || (NULL == config) ||
        ((unsigned)result >= (unsigned)CMD_MAP_REPLY_COUNT))
    {
        return false;
    }

    if (CMD_MAP_REPLY_ACCEPTED == result)
    {
        written = snprintf(out, out_size,
                           "{\"t\":%" PRIu32 ",\"accepted\":true,"
                           "\"version\":%" PRIu32 ",\"entries\":%" PRIu32 ","
                           "\"phase\":\"%s\"}",
                           timestamp_ms,
                           config->version,
                           robot_config_assigned_count(config),
                           run_phase_name(config->phase));
    }
    else
    {
        written = snprintf(out, out_size,
                           "{\"t\":%" PRIu32 ",\"accepted\":false,"
                           "\"reason\":\"%s\",\"version\":%" PRIu32 ","
                           "\"phase\":\"%s\"}",
                           timestamp_ms,
                           reasons[result],
                           config->version,
                           run_phase_name(config->phase));
    }

    return ((written >= 0) && ((size_t)written < out_size));
}
