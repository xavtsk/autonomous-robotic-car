/** @file cycle_observer.h
 * @brief Print controller state and queued replies from the fixture snapshot.
 */
#ifndef TESTING_RUNTIME_OBSERVER_H
#define TESTING_RUNTIME_OBSERVER_H

/** @brief Print a snapshot of controller state, queued replies and enabled
 * debug bytes after a named fixture event. Reading the ring leaves it intact.
 */
static void trace_runtime (const char * p_point)
{
    const s3_drive_t * p_drive = s3_integration_runtime_test_drive();
    const runtime_main_private_view_t view = s3_integration_runtime_test_view();
    printf("RUNTIME %s %u lock=%u state=%u reason=%u stop=%u velocity=%u "
           "freeze=%u cancel=%u reset=%u shared=%u%u%u%u waiting=%u "
           "remote=%u,%u,%u,%u,%u pending=%u,%u,%u,%u ring=",
           p_point, g_clock_us, g_locks, (unsigned)p_drive->state,
           (unsigned)p_drive->reason, g_stops, g_velocity_requests,
           g_freeze_calls, g_cancel_calls, g_reset_calls, view.b_has_command,
           view.b_has_stop, view.b_has_status, view.b_has_reset,
           view.b_waits_for_reset, view.epoch, view.last_id, view.stop_sequence,
           view.head, view.count, view.b_has_pending_request,
           view.pending_action, view.pending_id, view.pending_epoch);
    for (unsigned item = 0; item < view.count; ++item)
    {
        unsigned index = (view.head + item) % REMOTE_REPLY_RING_SIZE;
        for (unsigned byte = 0; byte < REMOTE_REPLY_SIZE; ++byte)
        {
            printf("%02x", view.reply[index][byte]);
        }
        putchar(',');
    }
#if APP_IR_DEBUG
    printf(" control=%u,%u,", view.b_has_control_tick,
           view.b_has_control_debug);
    for (unsigned byte = 0; byte < CONTROL_DEBUG_PACKET_SIZE; ++byte)
    {
        printf("%02x", view.control_packet[byte]);
    }
#endif
    putchar('\n');
}
#endif /* TESTING_RUNTIME_OBSERVER_H */
/* End of file. */

