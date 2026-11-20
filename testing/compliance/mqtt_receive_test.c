/** @file mqtt_receive_test.c
 * @brief Characterize MQTT fragment admission before compacting private state.
 *
 * Runs the unchanged subsystem fixture, then drives its real registered MQTT
 * callbacks. The subject mqtt_utk.c remains separately compiled. No hardware,
 * socket or motor is used. All observations occur on one host test thread.
 */
#define main mqtt_receive_test_original_main
#include "../subsystem1/host_test.c"
#undef main

#include <inttypes.h>
#include <stdbool.h>

/** @brief Submit two fragments and observe the exact command queue result.
 * @param case_id Sequential observation identifier, used only in the trace.
 * @param announced MQTT's wide advertised payload length in bytes.
 * @param actual Number of addressable bytes submitted, in the range 0..65535.
 * @param split First fragment length, in the range 0..actual.
 * @param p_bytes Buffer containing at least actual readable bytes.
 * @param p_expected Four bytes of the valid map used in this test.
 * @return Nothing. An assertion failure aborts this host-only test.
 * @note The caller has initialized callbacks and emptied the command queue.
 */
static void mqtt_receive_test_case (uint32_t case_id, uint32_t announced,
                                    uint16_t actual, uint16_t split,
                                    const uint8_t * p_bytes,
                                    const uint8_t * p_expected)
{
    buddy1_command_msg_t command;
    INT result;
    bool b_expected = (announced == 4U) && (actual == 4U);

    memset(&command, 0xA5, sizeof(command));
    incoming(NULL, "car/cmd/barcode_map", announced);
    fragment(NULL, p_bytes, split, 0U);
    /* The subtraction is nonnegative and bounded by the uint16_t actual. */
    fragment(NULL, p_bytes + split, (uint16_t)(actual - split),
             MQTT_DATA_FLAG_LAST);
    result = tk_rcv_mbf(g_mbf_buddy1_commands, &command, TMO_POL);
    if (b_expected)
    {
        assert(result == (INT)sizeof(command));
        assert(command.kind == BUDDY1_COMMAND_BARCODE_MAP);
        assert(memcmp(command.payload, p_expected, 4U) == 0);
    }
    else
    {
        assert(result == E_TMOUT);
    }
    printf("MQSTORE RX %" PRIu32 " %" PRIu32 " %u %u %d %u", case_id, announced,
           (unsigned)actual, (unsigned)split, (int)result,
           (unsigned)command.kind);
    for (unsigned index = 0U; index < sizeof(command.payload); ++index)
    {
        printf(" %02x", command.payload[index]);
    }
    putchar('\n');
    assert(tk_rcv_mbf(g_mbf_buddy1_commands, &command, TMO_POL) == E_TMOUT);
}

/** @brief Exercise exact-size, fragmented, saturated and reset receive paths.
 * @return Zero after all original and added assertions pass.
 * @note Standard C entry point; no command is sent to physical hardware.
 */
int main (void)
{
    static uint8_t bytes[UINT16_MAX];
    static const uint32_t announced[] = {
        0U, 1U, 3U, 4U, 5U, 16U, 17U, 18U, 31U, UINT16_MAX, UINT32_MAX};
    buddy1_barcode_map_t map = {'A', 'B', 'C', 'D'};
    uint8_t expected[4];
    uint32_t case_id = 0U;

    assert(mqtt_receive_test_original_main() == 0);
    assert(buddy1_map_pack(&map, expected));
    memcpy(bytes, expected, sizeof(expected));
    for (unsigned index = 0U; index < sizeof(announced) / sizeof(announced[0]);
         ++index)
    {
        for (uint16_t actual = 0U; actual <= 18U; ++actual)
        {
            for (uint16_t split = 0U; split <= actual; ++split)
            {
                mqtt_receive_test_case(case_id++, announced[index], actual,
                                       split, bytes, expected);
            }
        }
    }
    mqtt_receive_test_case(case_id++, 4U, UINT16_MAX, 0U, bytes, expected);
    mqtt_receive_test_case(case_id++, UINT32_MAX, UINT16_MAX, 1U, bytes,
                           expected);
    /* A new publish must clear the previous oversize/truncated state. */
    mqtt_receive_test_case(case_id++, 4U, 4U, 2U, bytes, expected);
    fprintf(stderr, "MQSTORE receive cases=%" PRIu32 "\n", case_id);
    assert(case_id == 2093U);
    return 0;
}

/* End of file. */

