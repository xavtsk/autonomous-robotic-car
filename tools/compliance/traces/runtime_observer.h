/** @file runtime_observer.h
 * @brief Print controller state and queued replies from the generated runtime
 * harness.
 */
/* Included only in the generated host harness, after its production C include. */
static void trace_runtime (const char * point)
{
    printf(
        "RUNTIME %s %u lock=%u state=%u reason=%u stop=%u velocity=%u freeze=%u cancel=%u reset=%u shared=%u%u%u%u waiting=%u remote=%u,%u,%u,%u,%u pending=%u,%u,%u,%u ring=",
        point, clock_value, locked, (unsigned)g_drive.state,
        (unsigned)g_drive.reason, stops, velocity_requests, freeze_calls,
        cancel_calls, reset_calls, g_shared.pending, g_shared.stop,
        g_shared.status_ready, g_shared.reset_pending, TEST_S3_RESET_WAITING,
        g_remote.epoch, g_remote.last_id, g_remote.stop_sequence, g_remote.head,
        g_remote.count, g_remote.pending.valid, g_remote.pending.action,
        g_remote.pending.id, g_remote.pending.epoch);
    for (unsigned item = 0; item < g_remote.count; ++item)
    {
        unsigned index = (g_remote.head + item) % REMOTE_REPLY_RING_SIZE;
        for (unsigned byte = 0; byte < REMOTE_REPLY_SIZE; ++byte)
        {
            printf("%02x", g_remote.reply[index][byte]);
        }
        putchar(',');
    }
#if APP_IR_DEBUG
    printf(" control=%u,%u,", TEST_S3_CONTROL_SEEN, TEST_S3_CONTROL_READY);
    for (unsigned byte = 0; byte < CONTROL_DEBUG_PACKET_SIZE; ++byte)
    {
        printf("%02x", g_control_debug.bytes[byte]);
    }
#endif
    putchar('\n');
}

/* End of file. */

