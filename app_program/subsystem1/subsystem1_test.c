#include "subsystem1.h"
#include "../test_report.h"

#if APP_TEST
LOCAL void drain(void)
{
    BUDDY1_TELEMETRY_MSG discarded;
    while (buddy1_outbox_take(&discarded)) { }
}

EXPORT void subsystem1_test(void)
{
    BUDDY1_TELEMETRY_MSG message;
    BUDDY1_SNAPSHOT v = {0}, decoded;
    BUDDY1_BARCODE_MAP map = {'A', 'W', 'F', 'Z'};
    BOOL newly = FALSE;
    BUDDY1_COMMAND_MSG command = {0};
    UB p[BUDDY1_PAYLOAD_MAX], mapping[BUDDY1_MAP_BYTES], bad[BUDDY1_PAYLOAD_MAX];
    unsigned i, count;
    H left, right;
    TEST_INIT();
    drain();
    TEST_CHECK("S1-B01", sizeof(message) == 15 && sizeof(command) == 5,
               "outgoing slot is 15 bytes; command slot is 5 bytes");
    TEST_CHECK("S1-B02", buddy1_report_motion(120, -118) == E_OK && buddy1_outbox_take(&message),
               "signed wheel values enter and leave the copied FIFO");
    TEST_CHECK("S1-B03", message.type == BUDDY1_TOPIC_MOTION &&
               buddy1_decode(message.type, message.payload, 4, &decoded) &&
               decoded.left_mms == 120 && decoded.right_mms == -118,
               "motion binary payload keeps wheel direction");
    TEST_CHECK("S1-B04", buddy1_motion_snapshot(&left, &right) && left == 120 && right == -118,
               "protected packed cache returns the reported values");
    TEST_CHECK("S1-B05", buddy1_report_motion(511, 0) == E_PAR &&
               buddy1_report_motion(-32768, 0) == E_PAR &&
               buddy1_motion_snapshot(&left, &right) && left == 120,
               "overflow rejected before cache update");
    v.left_mms = 1; v.right_mms = -509;
    TEST_CHECK("S1-B06", buddy1_encode(BUDDY1_TOPIC_MOTION, &v, p, sizeof(p)) == 4 &&
               buddy1_decode(BUDDY1_TOPIC_MOTION, p, 4, &decoded) &&
               decoded.left_mms == 2 && decoded.right_mms == -510,
               "2 mm/s steps round away from zero at half steps");
    v.left_mms = 0; v.right_mms = 0;
    (void)buddy1_encode(BUDDY1_TOPIC_MOTION, &v, p, sizeof(p));
    TEST_CHECK("S1-B07", p[0] == BUDDY1_WIRE_VERSION && p[1] == 0 && p[2] == 0 && p[3] == 0 &&
               buddy1_queue_publish(BUDDY1_TOPIC_MOTION, p, 4) == E_OK &&
               buddy1_outbox_take(&message) && message.payload[3] == 0,
               "embedded zero bytes survive queue copy");
    TEST_CHECK("S1-B08", !buddy1_decode(BUDDY1_TOPIC_MOTION, p, 3, &decoded) &&
               !buddy1_decode(BUDDY1_TOPIC_MOTION, p, 5, &decoded),
               "truncated and oversized binary packets rejected");
    p[0] = 1; /* previous version must not be interpreted as v2 */
    TEST_CHECK("S1-B09", !buddy1_decode(BUDDY1_TOPIC_MOTION, p, 4, &decoded),
               "unknown protocol version rejected");
    p[0] = BUDDY1_WIRE_VERSION; p[3] = 0x80;
    TEST_CHECK("S1-B10", !buddy1_decode(BUDDY1_TOPIC_MOTION, p, 4, &decoded),
               "nonzero padding bits rejected");
    TEST_CHECK("S1-B11", buddy1_map_pack(&map, mapping) &&
               buddy1_parse_barcode_map(mapping, sizeof(mapping), &map) &&
               buddy1_apply_barcode_map(&map) == E_OK &&
               buddy1_barcode_action('W') == BUDDY1_ACTION_LEFT,
               "20-bit pre-run mapping is parsed and applied");
    map.left = 'A';
    TEST_CHECK("S1-B12", !buddy1_map_pack(&map, bad) && buddy1_apply_barcode_map(&map) == E_PAR &&
               buddy1_barcode_action('W') == BUDDY1_ACTION_LEFT,
               "duplicate letters rejected without changing active map");
    map.left = 'w';
    TEST_CHECK("S1-B13", !buddy1_map_pack(&map, bad), "lowercase mapping rejected");
    TEST_CHECK("S1-B14", buddy1_report_barcode('W') == E_OK && buddy1_outbox_take(&message) &&
               message.payload[1] == (UB)(22 | (BUDDY1_ACTION_LEFT << 5)),
               "barcode and action share eight body bits");
    TEST_CHECK("S1-B15", buddy1_report_barcode('a') == E_PAR,
               "invalid barcode rejected");
    TEST_CHECK("S1-B16", buddy1_report_hump(0, 0) == E_OK && buddy1_outbox_take(&message) &&
               buddy1_snapshot(&decoded) && decoded.hump_valid && decoded.hump_height_mm == 0,
               "valid zero height differs from unavailable height");
    TEST_CHECK("S1-B17", buddy1_report_hump(65535, 500) == E_OK && buddy1_outbox_take(&message) &&
               buddy1_decode(message.type, message.payload, 5, &decoded) &&
               decoded.hump_count == 65535 && decoded.hump_height_mm == 500 &&
               buddy1_report_hump(0, 501) == E_PAR,
               "hump height uses nine bits, rejects above 500 mm without clipping");
    TEST_CHECK("S1-B18", buddy1_report_heading(359) == E_OK && buddy1_outbox_take(&message) &&
               buddy1_decode(message.type, message.payload, 3, &decoded) && decoded.heading_deg == 359 &&
               buddy1_report_heading(360) == E_PAR && buddy1_report_heading(-1) == E_PAR,
               "heading uses nine bits and checks limits");
    TEST_CHECK("S1-B19", buddy1_report_obstacle(4000, 4095, BUDDY1_ROUTE_RIGHT) == E_OK &&
               buddy1_outbox_take(&message) && buddy1_decode(message.type, message.payload, 5, &decoded) &&
               decoded.obstacle_distance_mm == 4000 && decoded.obstacle_width_mm == 4095 &&
               decoded.obstacle_route == BUDDY1_ROUTE_RIGHT &&
               buddy1_report_obstacle(4001, 0, BUDDY1_ROUTE_UNKNOWN) == E_PAR &&
               buddy1_report_obstacle(30, 4096, BUDDY1_ROUTE_UNKNOWN) == E_PAR,
               "obstacle fields use twelve bits and reject out-of-range values");
    TEST_CHECK("S1-B20", buddy1_report_obstacle(0, 0, (BUDDY1_OBSTACLE_ROUTE)3) == E_PAR,
               "reserved obstacle route rejected");
    TEST_CHECK("S1-B21", buddy1_report_robot_state(BUDDY1_STATE_ERROR) == E_OK &&
               buddy1_outbox_take(&message) && message.payload[1] == 7 &&
               buddy1_report_robot_state(BUDDY1_STATE_UNKNOWN) == E_PAR,
               "eight valid robot states use three bits");
    TEST_CHECK("S1-B22", buddy1_update_faults(BUDDY1_FAULT_STALL, BUDDY1_FAULT_STALL) == E_OK &&
               buddy1_outbox_take(&message) &&
               buddy1_update_faults(BUDDY1_FAULT_WIFI, BUDDY1_FAULT_WIFI) == E_OK &&
               buddy1_outbox_take(&message) && buddy1_snapshot(&decoded) &&
               decoded.faults == (BUDDY1_FAULT_STALL | BUDDY1_FAULT_WIFI),
               "separate fault updates preserve both bits");
    TEST_CHECK("S1-B23", buddy1_update_faults(BUDDY1_FAULT_WIFI, 0) == E_OK &&
               buddy1_outbox_take(&message) && buddy1_snapshot(&decoded) &&
               decoded.faults == BUDDY1_FAULT_STALL,
               "Wi-Fi recovery clears only its owned fault bit");
    TEST_CHECK("S1-B24", buddy1_update_faults(0x80, 0x80) == E_PAR &&
               buddy1_update_faults(1, 2) == E_PAR, "invalid fault-mask updates rejected");
    TEST_CHECK("S1-B35", buddy1_update_faults(BUDDY1_FAULT_IR, BUDDY1_FAULT_IR) == E_OK &&
               buddy1_outbox_take(&message) && buddy1_snapshot(&decoded) &&
               decoded.faults == (BUDDY1_FAULT_STALL | BUDDY1_FAULT_IR) &&
               buddy1_decode(message.type, message.payload, 2, &decoded) &&
               decoded.faults == (BUDDY1_FAULT_STALL | BUDDY1_FAULT_IR),
               "IR fault uses bit five without erasing the motor fault");
    TEST_CHECK("S1-B36", buddy1_update_faults(BUDDY1_FAULT_IR, 0) == E_OK &&
               buddy1_outbox_take(&message) && buddy1_snapshot(&decoded) &&
               decoded.faults == BUDDY1_FAULT_STALL,
               "IR recovery clears only its owned bit");
    TEST_CHECK("S1-B37", buddy1_update_faults(0x40, 0x40) == E_PAR,
               "six-bit mask rejects the first reserved bit");
    TEST_CHECK("S1-B25", buddy1_snapshot_packet(p) &&
               buddy1_decode(BUDDY1_TOPIC_SNAPSHOT, p, sizeof(p), &decoded) &&
               decoded.motion_valid && decoded.barcode_valid && decoded.hump_valid &&
               decoded.heading_valid && decoded.obstacle_valid && decoded.robot_state_valid && decoded.faults_valid,
               "all seven sections fit a 14-byte snapshot");
    TEST_CHECK("S1-B26", buddy1_invalidate(BUDDY1_TOPIC_HUMP) == E_OK &&
               buddy1_snapshot(&decoded) && !decoded.hump_valid && decoded.heading_valid,
               "invalidation preserves neighbouring packed fields");
    for (i = 0; i < BUDDY1_QUEUE_MESSAGES; ++i)
        if (buddy1_report_heading((H)i) != E_OK) break;
    TEST_CHECK("S1-B27", i == BUDDY1_QUEUE_MESSAGES && buddy1_report_heading(42) == E_TMOUT &&
               buddy1_snapshot(&decoded) && decoded.heading_deg == 42,
               "full four-slot outbox rejects live message but retains latest value");
    count = 0;
    while (buddy1_outbox_take(&message)) {
        TEST_CHECK("S1-B28", buddy1_decode(message.type, message.payload, 3, &decoded) &&
                   decoded.heading_deg == (H)count, "FIFO preserves message order");
        ++count;
    }
    TEST_CHECK("S1-B29", count == 4, "only four messages remained after overflow");
    p[0] = BUDDY1_WIRE_VERSION;
    TEST_CHECK("S1-B30", buddy1_command_enqueue("car/cmd/snapshot", p, 1) == E_OK &&
               buddy1_command_enqueue("car/cmd/snapshot", p, 1) == E_OK &&
               buddy1_command_enqueue("car/cmd/snapshot", p, 1) == E_TMOUT,
               "incoming command buffer is bounded to two messages");
    while (tk_rcv_mbf(mbf_buddy1_commands, &command, TMO_POL) > 0) { }
    TEST_CHECK("S1-B31", buddy1_command_enqueue("car/cmd/snapshot", p, 0) == E_PAR &&
               buddy1_command_enqueue("car/cmd/move_forward", p, 1) == E_PAR &&
               buddy1_command_enqueue("car/cmd/barcode_map", mapping, 3) == E_PAR,
               "bad command lengths and remote movement rejected");
    command.kind = BUDDY1_COMMAND_SNAPSHOT; command.payload[0] = BUDDY1_WIRE_VERSION;
    TEST_CHECK("S1-B32", buddy1_command_process(&command) == E_OK && buddy1_outbox_take(&message) &&
               message.type == BUDDY1_TOPIC_SNAPSHOT,
               "snapshot command produces the binary summary");
    for (i = 0; i < sizeof(mapping); ++i) command.payload[i] = mapping[i];
    command.kind = BUDDY1_COMMAND_BARCODE_MAP;
    TEST_CHECK("S1-B33", buddy1_command_process(&command) == E_OK && buddy1_outbox_take(&message) &&
               message.type == BUDDY1_TOPIC_MAP_REPLY && message.payload[1] == BUDDY1_REPLY_OK,
               "valid setup command produces a compact success reply");
    TEST_CHECK("S1-B34", buddy1_configuration_close() == E_OK &&
               !buddy1_configuration_allowed() && buddy1_command_process(&command) == E_OK &&
               buddy1_outbox_take(&message) && message.payload[1] == BUDDY1_REPLY_BUSY &&
               buddy1_barcode_action('W') == BUDDY1_ACTION_LEFT,
               "configuration freezes before run; existing map remains usable");
    TEST_CHECK("S1-B38", buddy1_configuration_reopen() == E_OK &&
               buddy1_configuration_allowed() && buddy1_command_process(&command) == E_OK &&
               buddy1_outbox_take(&message) && message.payload[1] == BUDDY1_REPLY_OK &&
               buddy1_configuration_claim(&map, &newly) == E_OK && newly && map.left == 'W' &&
               !buddy1_configuration_allowed() &&
               buddy1_configuration_claim(&map, &newly) == E_OK && !newly &&
               !buddy1_configuration_allowed(),
               "claim reports a new freeze once; an existing freeze is reported as not new");
    TEST_SUMMARY("subsystem1-binary");
}
#endif
