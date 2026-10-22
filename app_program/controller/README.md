# Temporary Buddy 2/3 controller

This folder is the integration layer, not Buddy 3's sensor subsystem and not the
finished shared Vehicle Controller. Buddy 1's September draft has not been imported.

```text
IR acquisition -> Buddy 3 line_navigation -> this controller -> Buddy 2 motion
                                         <- feedback/reports <-
                     <- odometry + processing mode <-
```

`s3_drive.c` implements the current two-buddy policy with injected motion callbacks.
`s3_integration.c` supplies those callbacks, serialises commands and samples, and
publishes copied status. The `s3_` names are retained for existing callers. These
files moved out of `subsystem3`; include `controller/s3_integration.h` for the
temporary control API. Only the explicit `S3_MOTION=1` profile starts this runtime.

## Contract for the future Vehicle Controller

1. Replace this runtime's controller ownership; do not start two motion callers or
   two report consumers. Keep a single ADC owner and serialise `line_navigation`
   context access. The current subsystem3 lifecycle delegates to this runtime only
   in the opt-in profile; change that delegation when installing the team controller.
2. Supply sensor frames and timestamped signed centre odometry to
   `s3_navigation_process`. Obtain odometry from Buddy 2's `motion_get_feedback`
   through the controller. The sensor core calls no motion functions. Distance
   is used for barcode rearming, not to decide whether a line is black or white.
3. Consume observations, steering proposals, barcode/junction/loss events and the
   persistent stop request. Arbitrate these against remote commands, obstacles and
   terrain. The temporary pending-turn policy is replaceable and is not part of
   Buddy 3's public sensing contract.
4. Issue Buddy 2 requests, save `motion_command_id()` immediately after acceptance,
   and drain `motion_get_report()` each cycle. Match DONE/CANCELLED/FAULT to IDs;
   `motion_poll()` does not drain the queue and must not be a second consumer.
   IDs wrap at 255: only compare outstanding IDs, never retain old reports across
   new runs. Queue loss, unexpected current-command cancellation or stale feedback
   must stop the run. The feedback loss counter detects overflow even without a
   subsequent successfully queued report.
5. Wait for fresh IDLE + `braked` before manoeuvring/restarting. Here `braked`
   means Buddy 2 has applied its brake and observed still wheels for 100 ms.
   Completion alone does not prove stillness. Fault reset requires explicit START
   afterwards. Keep the 30 ms navigation-owner lease, or agree an equivalent
   controller watchdog with Buddy 2; only healthy cycles refresh it.
6. Set suppressed processing mode during manoeuvres and restore TRACK afterwards.
   Raw diagnostics and safety checks continue. Keep explicit calibration/start
   gating; never drive from `steering_preview` or inactive diagnostic letters.

Current requests use Buddy 2's retained typed function API. Its new packed command,
report and telemetry formats remain available and unchanged. The packed telemetry
word lacks source time/epoch and saturates distance; do not reconstruct control
odometry from it or stamp old data with the current time.

See [operation, calibration and limitations](../subsystem3/MOTION_INTEGRATION.md).
Obstacle arbitration, MQTT, operator controls, full-team timing and moving-car
validation are still outstanding. No code here establishes 5 ms timing compliance.
