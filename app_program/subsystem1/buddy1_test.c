/** @file buddy1_test.c
 * @brief Check Buddy 1 map locking and telemetry on the device.
 */
#include "buddy1_test.h"
#include "../test_report.h"
#if APP_TEST
#define SNAPSHOT_TOPIC "car/cmd/snapshot"
#define TEST_MOTION BUDDY1_TOPIC_MOTION
static void drain (void);

/** @brief Verify packet rejection, RTOS queue limits and command replies. */
void subsystem1_test (void)
{
    buddy1_telemetry_msg_t     message;
    telemetry_codec_snapshot_t src = {0};
    telemetry_codec_snapshot_t decoded;
    telemetry_codec_map_t      map      = {'A', 'W', 'F', 'Z'};
    BOOL                       b_closed = FALSE;
    buddy1_command_msg_t       command  = {0};
    UB                         buf[BUDDY1_PAYLOAD_MAX];
    UB                         mapping[BUDDY1_MAP_BYTES];
    UB                         bad[BUDDY1_PAYLOAD_MAX];
    unsigned                   slot;
    H                          left;
    H                          right;
    const UB expected_w = (UB)(22U | ((unsigned)BUDDY1_ACTION_LEFT << 5U));
    TEST_INIT();
    drain();
    TEST_CHECK("S1-B01", sizeof(message) == 18U && sizeof(command) == 5U,
               "outgoing slot is 18 bytes; command slot is 5 bytes");
    TEST_CHECK("S1-B02",
               buddy1_report_motion(120, -118) == E_OK &&
                   buddy1_outbox_take(&message),
               "signed wheel values enter and leave the copied FIFO");
    TEST_CHECK("S1-B03",
               BUDDY1_TOPIC_MOTION == (int)message.type &&
                   buddy1_decode(message.type, message.payload, 4, &decoded) &&
                   120 == decoded.left_mms && decoded.right_mms == -118,
               "motion binary payload keeps wheel direction");
    TEST_CHECK("S1-B04",
               buddy1_motion_snapshot(&left, &right) && 120 == left &&
                   right == -118,
               "protected packed cache returns the reported values");
    TEST_CHECK("S1-B05",
               buddy1_report_motion(511, 0) == E_PAR &&
                   buddy1_report_motion(-32768, 0) == E_PAR &&
                   buddy1_motion_snapshot(&left, &right) && 120 == left,
               "overflow rejected before cache update");
    src.left_mms  = 1;
    src.right_mms = -509;
    TEST_CHECK("S1-B06",
               buddy1_encode(TEST_MOTION, &src, buf, sizeof buf) == 4U &&
                   buddy1_decode(BUDDY1_TOPIC_MOTION, buf, 4, &decoded) &&
                   2 == decoded.left_mms && decoded.right_mms == -510,
               "2 mm/s steps round away from zero at half steps");
    src.left_mms  = 0;
    src.right_mms = 0;
    (void)buddy1_encode(BUDDY1_TOPIC_MOTION, &src, buf, sizeof(buf));
    TEST_CHECK("S1-B07",
               buf[0] == BUDDY1_WIRE_VERSION && 0 == (int)buf[1] &&
                   0 == (int)buf[2] && 0 == (int)buf[3] &&
                   buddy1_queue_publish(BUDDY1_TOPIC_MOTION, buf, 4) == E_OK &&
                   buddy1_outbox_take(&message) && 0 == (int)message.payload[3],
               "embedded zero bytes survive queue copy");
    TEST_CHECK("S1-B08",
               !buddy1_decode(BUDDY1_TOPIC_MOTION, buf, 3, &decoded) &&
                   !buddy1_decode(BUDDY1_TOPIC_MOTION, buf, 5, &decoded),
               "truncated and oversized binary packets rejected");
    buf[0] = 1; /* Reject the previous wire version. */
    TEST_CHECK("S1-B09", !buddy1_decode(BUDDY1_TOPIC_MOTION, buf, 4, &decoded),
               "unknown protocol version rejected");
    buf[0] = BUDDY1_WIRE_VERSION;
    buf[3] = 0x80;
    TEST_CHECK("S1-B10", !buddy1_decode(BUDDY1_TOPIC_MOTION, buf, 4, &decoded),
               "nonzero padding bits rejected");
    TEST_CHECK("S1-B11",
               buddy1_map_pack(&map, mapping) &&
                   buddy1_parse_barcode_map(mapping, sizeof(mapping), &map) &&
                   buddy1_apply_barcode_map(&map) == E_OK &&
                   buddy1_barcode_action('W') == (unsigned)BUDDY1_ACTION_LEFT,
               "20-bit pre-run mapping is parsed and applied");
    map.left = 'A';
    TEST_CHECK("S1-B12",
               !buddy1_map_pack(&map, bad) &&
                   buddy1_apply_barcode_map(&map) == E_PAR &&
                   buddy1_barcode_action('W') == (unsigned)BUDDY1_ACTION_LEFT,
               "duplicate letters rejected without changing active map");
    map.left = 'w';
    TEST_CHECK("S1-B13", !buddy1_map_pack(&map, bad),
               "lowercase mapping rejected");
    TEST_CHECK("S1-B14",
               buddy1_report_barcode('W') == E_OK &&
                   buddy1_outbox_take(&message) &&
                   message.payload[1] == expected_w,
               "barcode and action share eight body bits");
    TEST_CHECK("S1-B15", buddy1_report_barcode('a') == E_PAR,
               "invalid barcode rejected");
    TEST_CHECK("S1-B16",
               buddy1_report_hump(0, 0) == E_OK &&
                   buddy1_outbox_take(&message) && buddy1_snapshot(&decoded) &&
                   decoded.hump_valid && 0 == (int)decoded.hump_height_mm,
               "valid zero height differs from unavailable height");
    TEST_CHECK(
        "S1-B17",
        buddy1_report_hump(65535, 500) == E_OK &&
            buddy1_outbox_take(&message) &&
            buddy1_decode(message.type, message.payload, 5, &decoded) &&
            65535U == decoded.hump_count && 500U == decoded.hump_height_mm &&
            buddy1_report_hump(0, 501) == E_PAR,
        "hump height uses nine bits, rejects above 500 mm without clipping");
    TEST_CHECK(
        "S1-B18",
        buddy1_report_heading(359) == E_OK && buddy1_outbox_take(&message) &&
            buddy1_decode(message.type, message.payload, 3, &decoded) &&
            359 == decoded.heading_deg && buddy1_report_heading(360) == E_PAR &&
            buddy1_report_heading(-1) == E_PAR,
        "heading uses nine bits and checks limits");
    TEST_CHECK(
        "S1-B19",
        buddy1_report_obstacle(4000, 4095, BUDDY1_ROUTE_RIGHT) == E_OK &&
            buddy1_outbox_take(&message) &&
            buddy1_decode(message.type, message.payload, 5, &decoded) &&
            4000 == (int)decoded.obstacle_distance_mm &&
            4095 == (int)decoded.obstacle_width_mm &&
            (unsigned)BUDDY1_ROUTE_RIGHT == decoded.obstacle_route &&
            buddy1_report_obstacle(4001, 0, BUDDY1_ROUTE_UNKNOWN) == E_PAR &&
            buddy1_report_obstacle(30, 4096, BUDDY1_ROUTE_UNKNOWN) == E_PAR,
        "obstacle fields use twelve bits and reject out-of-range values");
    TEST_CHECK("S1-B20",
               buddy1_report_obstacle(0, 0, (telemetry_codec_route_t)3) ==
                   E_PAR,
               "reserved obstacle route rejected");
    TEST_CHECK("S1-B21",
               buddy1_report_robot_state(BUDDY1_STATE_ERROR) == E_OK &&
                   buddy1_outbox_take(&message) && 7U == message.payload[1] &&
                   buddy1_report_robot_state(BUDDY1_STATE_UNKNOWN) == E_PAR,
               "eight valid robot states use three bits");
    TEST_CHECK("S1-B22",
               buddy1_update_faults(BUDDY1_FAULT_STALL, BUDDY1_FAULT_STALL) ==
                       E_OK &&
                   buddy1_outbox_take(&message) &&
                   buddy1_update_faults(BUDDY1_FAULT_WIFI, BUDDY1_FAULT_WIFI) ==
                       E_OK &&
                   buddy1_outbox_take(&message) && buddy1_snapshot(&decoded) &&
                   decoded.faults == (BUDDY1_FAULT_STALL | BUDDY1_FAULT_WIFI),
               "separate fault updates preserve both bits");
    TEST_CHECK("S1-B23",
               buddy1_update_faults(BUDDY1_FAULT_WIFI, 0) == E_OK &&
                   buddy1_outbox_take(&message) && buddy1_snapshot(&decoded) &&
                   BUDDY1_FAULT_STALL == decoded.faults,
               "Wi-Fi recovery clears only its owned fault bit");
    TEST_CHECK("S1-B24",
               buddy1_update_faults(0x80, 0x80) == E_PAR &&
                   buddy1_update_faults(1, 2) == E_PAR,
               "invalid fault-mask updates rejected");
    TEST_CHECK("S1-B35",
               buddy1_update_faults(BUDDY1_FAULT_IR, BUDDY1_FAULT_IR) == E_OK &&
                   buddy1_outbox_take(&message) && buddy1_snapshot(&decoded) &&
                   decoded.faults == (BUDDY1_FAULT_STALL | BUDDY1_FAULT_IR) &&
                   buddy1_decode(message.type, message.payload, 2, &decoded) &&
                   decoded.faults == (BUDDY1_FAULT_STALL | BUDDY1_FAULT_IR),
               "IR fault uses bit five without erasing the motor fault");
    TEST_CHECK("S1-B36",
               buddy1_update_faults(BUDDY1_FAULT_IR, 0) == E_OK &&
                   buddy1_outbox_take(&message) && buddy1_snapshot(&decoded) &&
                   BUDDY1_FAULT_STALL == decoded.faults,
               "IR recovery clears only its owned bit");
    TEST_CHECK("S1-B37", buddy1_update_faults(0x40, 0x40) == E_PAR,
               "six-bit mask rejects the first reserved bit");
    TEST_CHECK(
        "S1-B25",
        buddy1_snapshot_packet(buf) &&
            buddy1_decode(BUDDY1_TOPIC_SNAPSHOT, buf, sizeof(buf), &decoded) &&
            decoded.motion_valid && decoded.barcode_valid &&
            decoded.hump_valid && decoded.heading_valid &&
            decoded.obstacle_valid && decoded.robot_state_valid &&
            decoded.faults_valid,
        "all seven sections and pose fit a 17-byte snapshot");
    TEST_CHECK("S1-B26",
               buddy1_invalidate(BUDDY1_TOPIC_HUMP) == E_OK &&
                   buddy1_snapshot(&decoded) && !decoded.hump_valid &&
                   decoded.heading_valid,
               "invalidation preserves neighbouring packed fields");
    for (slot = 0; slot < (unsigned)BUDDY1_QUEUE_MESSAGES; ++slot)
    {
        if (buddy1_report_heading((H)slot) != E_OK)
        {
            break;
        }
    }
    TEST_CHECK(
        "S1-B27",
        BUDDY1_QUEUE_MESSAGES == slot && buddy1_report_heading(42) == E_TMOUT &&
            buddy1_snapshot(&decoded) && 42 == decoded.heading_deg,
        "full four-slot outbox rejects live message but retains latest value");
    unsigned count = 0;
    while (buddy1_outbox_take(&message))
    {
        TEST_CHECK("S1-B28",
                   buddy1_decode(message.type, message.payload, 3, &decoded) &&
                       decoded.heading_deg == (H)count,
                   "FIFO preserves message order");
        ++count;
    }
    TEST_CHECK("S1-B29", 4U == count,
               "only four messages remained after overflow");
    buf[0] = BUDDY1_WIRE_VERSION;
    TEST_CHECK("S1-B30",
               buddy1_command_enqueue(SNAPSHOT_TOPIC, buf, 1) == E_OK &&
                   buddy1_command_enqueue(SNAPSHOT_TOPIC, buf, 1) == E_OK &&
                   buddy1_command_enqueue(SNAPSHOT_TOPIC, buf, 1) == E_TMOUT,
               "incoming command buffer is bounded to two messages");
    while (tk_rcv_mbf(gh_buddy1_commands, &command, TMO_POL) > 0)
    { /* Each nonblocking receive drains one command from the full mailbox. */
    }
    TEST_CHECK(
        "S1-B31",
        buddy1_command_enqueue(SNAPSHOT_TOPIC, buf, 0) == E_PAR &&
            buddy1_command_enqueue("car/cmd/move_forward", buf, 1) == E_PAR &&
            buddy1_command_enqueue("car/cmd/barcode_map", mapping, 3) == E_PAR,
        "bad command lengths and remote movement rejected");
    command.kind       = BUDDY1_COMMAND_SNAPSHOT;
    command.payload[0] = BUDDY1_WIRE_VERSION;
    TEST_CHECK("S1-B32",
               buddy1_command_process(&command) == E_OK &&
                   buddy1_outbox_take(&message) &&
                   BUDDY1_TOPIC_SNAPSHOT == (int)message.type,
               "snapshot command produces the binary summary");
    for (slot = 0; slot < sizeof(mapping); ++slot)
    {
        command.payload[slot] = mapping[slot];
    }
    command.kind = BUDDY1_COMMAND_BARCODE_MAP;
    TEST_CHECK("S1-B33",
               buddy1_command_process(&command) == E_OK &&
                   buddy1_outbox_take(&message) &&
                   BUDDY1_TOPIC_MAP_REPLY == (int)message.type &&
                   BUDDY1_REPLY_OK == (int)message.payload[1],
               "valid setup command produces a compact success reply");
    TEST_CHECK("S1-B34",
               buddy1_configuration_close() == E_OK &&
                   !buddy1_configuration_allowed() &&
                   buddy1_command_process(&command) == E_OK &&
                   buddy1_outbox_take(&message) &&
                   BUDDY1_REPLY_BUSY == (int)message.payload[1] &&
                   buddy1_barcode_action('W') == (unsigned)BUDDY1_ACTION_LEFT,
               "configuration freezes before run; existing map remains usable");
    TEST_CHECK("S1-B38",
               buddy1_configuration_reopen() == E_OK &&
                   buddy1_configuration_allowed() &&
                   buddy1_command_process(&command) == E_OK &&
                   buddy1_outbox_take(&message) &&
                   BUDDY1_REPLY_OK == (int)message.payload[1] &&
                   buddy1_configuration_claim(&map, &b_closed) == E_OK &&
                   b_closed && 'W' == (int)map.left &&
                   !buddy1_configuration_allowed() &&
                   buddy1_configuration_claim(&map, &b_closed) == E_OK &&
                   !b_closed && !buddy1_configuration_allowed(),
               "claim reports a new freeze once; an existing freeze is "
               "reported as not new");

    TEST_SUMMARY("subsystem1-binary");
}

/** @brief Consume queued publications until the test outbox is empty. */
static void drain (void)
{
    buddy1_telemetry_msg_t discarded;

    while (buddy1_outbox_take(&discarded))
    {
        /* Each successful take discards one queued publication. */
    }
}
#endif

/* End of file. */

