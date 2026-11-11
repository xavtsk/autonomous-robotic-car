# Dashboard run controls

This optional protocol is available in the `APP_S3_MOTION` firmware profile.
It uses the existing single navigation owner and its calibration, fault,
motion-feedback and terrain readiness gates. GP20/GP21 remain available.
All integers below are unsigned little-endian. Version is `3`. Commands and
replies are non-retained MQTT messages. There is no automatic START retry.

| Topic | Python struct | Fields in order |
| --- | --- | --- |
| `car/cmd/control` | `<BBIII` (14 bytes) | version, action, request ID, link epoch, observed robot time in microseconds |
| `car/reply/control` | `<BBIBBI` (12 bytes) | version, action, request ID, result, reason, request epoch (STOP uses current epoch) |
| `car/telemetry/control` | `<BIIIB` (14 bytes) | version, link epoch, robot time in microseconds, last request ID, flags |

Actions: START=1, STOP=2, RESET=3. Results: QUEUED=0, EXECUTED=1,
REJECTED=2, CANCELLED=3. Reasons: NONE=0, INVALID=1, STALE=2, REPLAY=3,
BUSY=4, NOT_READY=5, SUPERSEDED=6, UNAVAILABLE=7. Malformed/unknown packets
are ignored, because a reliable request identity cannot be assumed.

Flags: bit0 AVAILABLE, bit1 CAN_START, bit2 CAN_RESET, bit3 RUNNING;
bits4–7 are zero. Flags are a current readiness hint, not a promise that a
later command will execute. The controller repeats all gates at execution.
Status is latest-only, at most 2 Hz, and uses routine telemetry priority.
When ordinary telemetry is pending, at least one ordinary packet is selected
between these status packets, even on a link slower than the status period.
If the sampling owner has not updated for 200 ms, all flags are cleared.
Replies have event priority and QoS1; a bounded six-result ring retains the
most recent replies if a client floods commands. Missing replies must be
shown as unconfirmed, never inferred successful.

Clients take the current epoch/time from fresh control status (at most 2 s
old), allocate IDs after `last_request_id`, and increment serially. Firmware
uses modulo-2^32 serial ordering (positive difference below 2^31). It consumes
newer IDs even on refusal. Within an epoch, duplicate or older START/RESET
cannot execute. A newer STOP advances that watermark before cancelling a
pending START/RESET, so late delivery of the older START cannot revive it.

Every MQTT connection receives a nonzero transport-PRNG epoch. Reconnect
cancels queued remote START/RESET and changes the epoch. Both receipt and
actual execution require a matching epoch and an echoed robot time no more
than 2 s old; uint32 timer wrap is handled by unsigned subtraction. This is
operational freshness protection, not authentication or cryptographic replay
protection. Use the trusted local broker/network. A remote STOP is accepted
even with epoch0 or an expired time, and its reply carries the current epoch.

QUEUED means only accepted by the controller mailbox. EXECUTED means the
sampling owner accepted START, performed STOP, or observed successful RESET
completion. RESET cannot start the car and is separate from START. Dashboard
state confirmation additionally uses a newer controller telemetry sample;
QoS delivery or a QUEUED reply alone must never be labelled as moving/stopped.
STOP bypasses the ordinary snapshot/map command queue and cancels pending
remote commands. It never directly calls a motor driver from a network task.
Dashboard STOP also cancels a GP20 countdown which has not yet submitted its
START. A held GP21 blocks remote START and RESET until released. A cancelled
remote RESET can finish its safe housekeeping without acknowledging a GP
request or producing a success under a different connection epoch.
Wi-Fi controls are not an emergency-stop guarantee; physical GP21 still
works without the dashboard/network.

Example vectors (hex): START ID `12345678`, epoch `11223344`, time `aabbccdd`:
`03017856341244332211ddccbbaa`; EXECUTED/NONE reply:
`030178563412010044332211`; status flags3 with the same values:
`0344332211ddccbbaa7856341203`.
