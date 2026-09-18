/*
 * command_receiver.h - inbound commands for the INF2004 robot car.
 * Buddy 1: WiFi communication, command and telemetry.
 *
 * The brief requires two-way communication: the car publishes telemetry AND
 * accepts commands. This module turns an arriving MQTT message into
 * something the rest of the firmware can act on.
 *
 * The command that matters most:
 *
 *   On demo day the lecturer announces a NEW mapping from barcode letters to
 *   navigation actions - e.g. D now means turn left. He then requires us to
 *   apply it over WiFi, live. Recompiling is explicitly not allowed.
 *
 *   This module validates the wire format. The accepted table is stored by
 *   shared robot_config code, not by the network subsystem.
 *
 * Like telemetry.c, this file contains no Pico SDK calls and is fully
 * testable on a laptop.
 *
 * Style: Barr-C:2018.  Language: C99.
 */
#ifndef BUDDY1_COMMAND_RECEIVER_H
#define BUDDY1_COMMAND_RECEIVER_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "shared/robot_config.h"

/* Which command arrived. */
typedef enum
{
    CMD_NONE = 0,        /* topic was not a command at all      */
    CMD_BARCODE_MAP,     /* reassign barcode letters to actions */
    CMD_SNAPSHOT,        /* send the current state right now    */
    CMD_UNKNOWN          /* under cmd/, but we do not know it   */
} cmd_kind_t;

typedef enum
{
    CMD_MAP_REPLY_ACCEPTED = 0,
    CMD_MAP_REPLY_MALFORMED,
    CMD_MAP_REPLY_LOCKED,
    CMD_MAP_REPLY_INVALID,
    CMD_MAP_REPLY_COUNT
} cmd_map_reply_t;

/*
 * Identify a command from its full MQTT topic.
 *
 * Matches on the last path segment, so it works whatever the group prefix
 * is: "grp07/car/cmd/snapshot" and "grp12/car/cmd/snapshot" both give
 * CMD_SNAPSHOT.
 *
 * Returns CMD_NONE if the topic is not under a cmd/ branch at all.
 */
cmd_kind_t cmd_kind_from_topic(const char * topic);

/*
 * Turn an action name into the enum. Case-sensitive, exact match.
 * Returns NAV_NONE if the name is not recognised.
 */
nav_action_t nav_action_from_name(const char * name);

/*
 * Parse a barcode remap payload such as:
 *
 *     {"A":"LEFT","B":"RIGHT","C":"STRAIGHT","D":"UTURN"}
 *
 * Rules this parser follows, and why:
 *
 *  - NO malloc, NO strtok. Same constraints as the LAB2 cmdparse exercise.
 *    strtok keeps hidden state between calls and is not safe to use from
 *    more than one task.
 *
 *  - The input is read-only and is not modified.
 *
 *  - It is ALL OR NOTHING. Parsing happens into a scratch table, and
 *    map_out is only overwritten once the entire payload has been read
 *    successfully. A malformed command therefore leaves the car running on
 *    the mapping it already had, instead of a half-applied one. Halfway
 *    through a demo, that distinction matters.
 *
 *  - Letters not mentioned in the payload are set to NAV_NONE, meaning
 *    "this barcode has no assigned action". The payload defines the
 *    complete mapping, so what you see on the wire is what the car is using.
 *
 * map_out must have room for ROBOT_BARCODE_COUNT entries, indexed by
 * (letter - 'A').
 *
 * Returns true if the payload parsed and map_out was updated.
 */
bool cmd_parse_barcode_map(const char * payload,
                           nav_action_t map_out[ROBOT_BARCODE_COUNT]);

/* Build the structured acknowledgement published on reply/barcode_map. */
bool cmd_format_barcode_map_reply(char * out,
                                  size_t out_size,
                                  uint32_t timestamp_ms,
                                  const robot_config_t * config,
                                  cmd_map_reply_t result);

#endif /* BUDDY1_COMMAND_RECEIVER_H */
