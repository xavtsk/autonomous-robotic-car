# Buddy 1 binary MQTT protocol version 3

Tribunal 1 replaces JSON application payloads with explicit binary fields.
Topics remain readable. Robot tasks call the existing `buddy1_report_*()` APIs;
Buddy 1 packs the values, copies a complete message into its RTOS outbox, and
the existing network task publishes the bytes. The laptop decodes them with
`tools/buddy1_mqtt_codec.py`.

Robot and laptop must both use version 3. Versions 1/2 and JSON are rejected,
not guessed by packet length. Version 3 adopts Buddy 4's 0–500 mm peak range
and Buddy 5's 30–4000 mm distance / 0–4095 mm width ranges. Hump count remains
16 bits for the sensor owner's saturating 0–65535 count. Wheel/control precision
and all navigation algorithms are unchanged. No raw C struct/enum layout is sent.

## Common encoding

- Byte 0 is protocol version `0x03`. It is framing metadata.
- The body starts at byte 1. Bit 0 is its least significant bit; bits continue
  through successive bytes. Multi-byte fields are little endian.
- Unused high bits in the last byte must be zero. Unknown versions, invalid
  field codes and incorrect lengths are rejected.
- Zero bytes are ordinary binary data. Always use the explicit payload length;
  never `strlen()`, a terminating zero, or `%s` on a binary payload.
- Field ranges, resolution and outstanding owner decisions are listed in
  [DATA_REPRESENTATION.md](DATA_REPRESENTATION.md).
- Live telemetry uses QoS 0. Replies use QoS 1. Retain is always zero.
- QoS 1 is not an application-level delivery guarantee. Buffer-full or publish
  failure can still lose a live report or reply; the cache remains available.

## Robot to laptop packets

Bit positions below are relative to the body, excluding the version byte.

| Topic | Body fields, low bit first | Payload bytes including version |
|---|---|---:|
| `car/telemetry/motion` | left magnitude bits 0–7; left reverse bit 8; right magnitude bits 9–16; right reverse bit 17 | 4 |
| `car/telemetry/barcode` | letter bits 0–4; action bits 5–7 | 2 |
| `car/telemetry/hump` | count bits 0–15; height bits 16–24 (0–500 mm); seven zero padding bits | 5 |
| `car/telemetry/heading` | heading bits 0–8 | 3 |
| `car/telemetry/obstacle` | distance bits 0–11 (30–4000 mm); width bits 12–23 (0–4095 mm); route bits 24–25 | 5 |
| `car/telemetry/state` | state bits 0–2 | 2 |
| `car/telemetry/faults` | shared fault mask bits 0–5 | 2 |
| `car/telemetry/pose` | distance since START, cm modulo 4096, bits 0–11; heading since START bits 12–20 | 4 |
| `car/reply/snapshot` | validity and all seven sections, described below | 14 |
| `car/reply/barcode_map` | result bits 0–1: 0 OK, 1 invalid, 2 configuration closed, 3 internal error | 2 |
| `car/status` | alive bit 0; faults-valid bit 1; fault mask bits 2–7 | 2 |

The periodic heartbeat uses the topic configured by `MQTT_TOPIC` in the local
Wi-Fi configuration, which may be `mtk3smp-pico/heartbeat` rather than `car/status`.
Both have the status layout above. The interval remains 5 seconds. Alive means
the network task reached its heartbeat callback; it does not prove every task or
sensor is healthy. A missing heartbeat alone cannot distinguish radio, broker,
laptop or robot failure.

Magnitude codes represent 2 mm/s each, covering 0–510 mm/s. The reverse bit is
one for a negative speed. Odd integer speeds round to the nearest 2 mm/s, with
half steps away from zero (119 → 120; -119 → -120). Error is at most 1 mm/s.
Values outside ±510 are rejected before the cache changes. Negative zero is
noncanonical and rejected. This quantisation applies to telemetry and its cache,
not Buddy 2's PID calculations or control requests. Validate the proposed range
against calibrated speed before freezing the team contract.

### Route pose (`car/telemetry/pose`, added 28 Oct 2026)

Live only: it has no snapshot section and is never cached. The controller
sends one pose at START (0, 0), one every 200 ms while a run is active
(following, braking, turning, reacquiring or avoiding) and a final one when the
run ends. Both values come from Buddy 2's encoder estimate
(`motion_status_t.distance_m`, `.heading_deg`) relative to their values at START.

- **Distance**: robot-centre distance in cm, rounded, signed, then modulo 4096.
  Reversing gives a smaller value; spinning on the spot leaves it unchanged.
  The laptop unwraps it (`unwrap_distance()` in `tools/buddy1_mqtt_codec.py`):
  each step is taken as the nearest value in -2048..+2047 cm. At most ~18 cm is
  travelled per sample at 0.884 m/s, so ~23 s of lost samples are tolerated.
- **Heading**: degrees 0-359, + = anticlockwise, 0 = direction at START. Same
  9-bit encoding as `car/telemetry/heading`, which stays reserved for Buddy 4.
- Samples are skipped while Buddy 2's estimate is invalid or calibrating. Encoder
  dead reckoning drifts with wheel slip; it is a route sketch, not a survey.

Letter is A=0 through Z=25. Action is unknown=0, forward=1, left=2, right=3,
uturn=4. Route is unknown=0, left=1, right=2. State is stopped=0,
line_following=1, turning_left=2, turning_right=3, uturn=4, avoiding_obstacle=5,
complete=6, error=7. `BUDDY1_STATE_UNKNOWN` is a local API value; it is not a
ninth transmitted state. Snapshot validity represents an unavailable state.

### Snapshot

| Body bits | Meaning |
|---|---|
| 0–6 | validity bits: motion, barcode, hump, heading, obstacle, state, faults |
| 7–24 | motion: same 18 bits as live motion |
| 25–32 | barcode: same 8 bits as live barcode |
| 33–57 | hump: same 25 bits as live hump |
| 58–66 | heading: same 9 bits as live heading |
| 67–92 | obstacle: same 26 bits as live obstacle |
| 93–95 | state: same 3 bits as live state |
| 96–101 | fault mask: same 6 bits as live faults |
| 102–103 | zero padding |

An invalid section has its validity bit clear and canonical zero data bits.
A valid zero height is different from an invalid height. The snapshot holds
the most recently reported values; it is not a history or a simultaneous sample
of all sensors. Validity does not expire automatically. Owners should call
`buddy1_invalidate(type)` when their cached result is no longer usable. Per-field
age limits/timestamps remain a team integration decision.

### Fault ownership

| Mask bit | Meaning | Intended source |
|---|---|---|
| 0 | motor stall | Buddy 2 / Vehicle Controller adapter |
| 1 | encoder fault | Buddy 2 / Vehicle Controller adapter |
| 2 | IMU fault | Buddy 4 / Vehicle Controller adapter |
| 3 | confirmed no-echo failure | Buddy 5 / Vehicle Controller adapter |
| 4 | Wi-Fi/IP link unavailable | network task hook |
| 5 | one or more enabled IR sensors faulted | Buddy 3 / Vehicle Controller adapter |

`buddy1_update_faults(changed, active)` changes only the caller's agreed bits.
For example `(0x10, 0)` clears Wi-Fi loss without clearing motor stall. All
read-modify-write access uses Buddy 1's priority-inheritance mutex. Queue entries
are independent copies. Do not call this API from an ISR.

This is a protected telemetry mirror, not the controller's authoritative safety
state. The radio hook is connected; the controller owns the motor/IR reporting
adapters, including Buddy 4/5. The aggregate IR bit does
not identify the failed channel: Buddy 3 retains its own three-channel mask.
Zero bits do not prove unconnected sensors passed.
`faults_valid` means at least one source supplied a fault update. It is not a
separate health certificate for each of the six fault categories. Broker-only failure
does not set the Wi-Fi link bit; heartbeat absence and network logs show it.

## Laptop to robot commands

| Topic | Exact binary payload | Behaviour |
|---|---|---|
| `car/cmd/snapshot` | `03` (version only) | enqueue a protected snapshot reply |
| `car/cmd/barcode_map` | version + four 5-bit letters: forward at 0–4, left at 5–9, right at 10–14, uturn at 15–19; 4 bytes total | validate and atomically replace the map before first run |

All four letters must be different uppercase A–Z. `03 20 88 01` maps A/B/C/D.
The high four body padding bits are zero. Old JSON, malformed packets, unknown
topics and unsupported commands are rejected before enqueueing. Those packets
receive no reply. A valid queued mapping receives an OK/closed/internal result
when processed, provided the outgoing reply fits the outbox.

`vehicle_controller_begin()` closes configuration using the same mutex as map
updates before posting its start request. The map stays frozen until reboot,
including after a stop. This prevents a mapping change between barcode detection
and its later junction. Snapshot requests remain available during a run. Remote
motor-driving commands are unsupported.

## Buffering and task behaviour

- Outbox: four fixed 15-byte slots, each containing a one-byte topic ID and a
  14-byte maximum payload. Topic IDs 0–10 use four bits; the rest of the byte is
  zero. The slot's unused payload tail is zero and is not published.
- Incoming commands: two fixed 5-byte slots, each containing a two-bit command
  kind in one byte and four payload bytes. Length is implicit in the kind.
- The RTOS owns copies, so producers may reuse their local variables on return.
- `E_OK` means queued, not delivered. `E_TMOUT` means full; the new live report
  was rejected. The protected latest-value cache was updated first.
- Message-buffer send/receive uses `TMO_POL` for the producer and network task.
  Producers can briefly wait for the cache mutex; they never wait for network
  transmission. The command task blocks on its incoming buffer with `TMO_FEVR`.
- Network service attempts at most one packet per 20 ms paced slot. A fixed
  in-flight/retry slot retains transiently rejected packets; later FIFO entries
  remain queued. Retries can duplicate a packet; QoS 0 is not guaranteed delivery.
- Suggested periodic telemetry rate remains at most 5 Hz per source. Events
  should be reported once per event/change, not every control iteration.
- A burst may drop an important event. Safety actions must use the controller
  path, not successful delivery of a telemetry packet. Fault status remains in
  the cache and later heartbeats; event history is not guaranteed.

Buddy 1 command priority remains 22. The kernel radio/lwIP task still runs at
priority 4, above motion at 14. A lower-priority application command task does
not by itself prove the Week 6 low-priority networking claim. Full-load control
jitter and reconnection timing remain physical validation items; do not claim
they are solved by compact packets.

## Laptop examples

From the repository root:

```sh
python3 tools/buddy1_mqtt_codec.py decode --topic car/telemetry/motion --hex 033c7602
python3 tools/buddy1_mqtt_codec.py encode-map --forward A --left W --right F --uturn Z
python3 tools/buddy1_mqtt_codec.py encode-snapshot
```

`decode()` accepts the original bytes from any MQTT subscriber, e.g.
`decode(message.topic, message.payload)`. `--file` reads a raw payload file.
The encoder commands above PRINT HEX FOR INSPECTION. The MQTT sender must send
`bytes.fromhex(hex_output)` (or the bytes from `encode_map()`), not the printable
hex characters. A snapshot request is one byte `0x03`, not the text `"03"`.

## Optional recorded barcode diagnostics

`car/debug/barcode` now carries binary BCAP v1 packets, **not ASCII**. This is
a separate diagnostic version, not telemetry version 3. See
`docs/BARCODE_WIRE_PROTOCOL.md`. USB dumps and laptop `.txt` imports/exports stay
readable and unchanged. The source owner packs capture data under its existing
mutex; the network side uses `TMO_POL` and never waits on the capture lock.
Core telemetry remains ahead of capture uploads; optional controller/IR live
previews remain latest-only, lower priority and disabled unless `IR_DEBUG=1`.

For a heartbeat configured on a different topic, decode it with the `car/status`
layout. The tool is a codec; broker connection/subscription is handled by the
laptop's chosen MQTT client.
