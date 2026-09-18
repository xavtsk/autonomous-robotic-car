# Buddy 1 MQTT Protocol

This document is the wire contract between the robot and the monitoring
laptop. It does **not** replace the in-robot interfaces between Buddies 2-5
and the shared Vehicle Controller.

## Operating rule

The barcode mapping is sent while the robot is in `CONFIGURING`. The
integrated application then advances through `ARMED` to `AUTONOMOUS`, which
freezes the accepted mapping. During the assessed autonomous run, the laptop
is a passive observer: the robot keeps sensing and driving if WiFi, MQTT or
the laptop disappears. No MQTT message commands the motors, steering,
obstacle bypass or mission state.

`cmd/snapshot` is a development and demonstration diagnostic. Do not send it
during an assessment that forbids all laptop input after the start.

## Transport stack

From lowest to highest level:

1. WiFi carries radio frames between the Pico W and the access point.
2. IP gives the robot and laptop addresses on that network.
3. TCP provides MQTT with an ordered, reliable byte stream.
4. MQTT routes messages by topic through the broker on the laptop.
5. JSON defines the fields inside each MQTT payload.

The current broker endpoint is an IPv4 address supplied at build time and
port `1883`. Development uses Mosquitto. Authentication and TLS are not
enabled in the supplied local development configuration, so use a private
hotspot or isolated lab network rather than a public network.

## Topic root

Every topic begins with a team-specific root:

```text
grp07/car
```

Replace `grp07` with the real group number. It is supplied to CMake as
`-DTEAM_ROOT=grp07/car`; no source edit is required.

## Topic table

| Topic suffix | Direction | QoS | Retained | Purpose |
|---|---|---:|---:|---|
| `status` | robot to laptop | 1 | yes | Heartbeat, link health and run phase |
| `telemetry/state` | robot to laptop | 0 | no | Shared controller state |
| `telemetry/motion` | robot to laptop | 0 | no | Wheel speed, distance and encoders |
| `telemetry/line` | robot to laptop | 0 | no | Line position, junction and IR readings |
| `telemetry/barcode` | robot to laptop | 0 | no | Decoded letter and frozen action |
| `telemetry/hump` | robot to laptop | 0 | no | Hump height and count |
| `telemetry/imu` | robot to laptop | 0 | no | Heading, pitch, estimated turn rate and event |
| `telemetry/obstacle` | robot to laptop | 0 | no | Obstacle profile and selected plan |
| `telemetry/log` | robot to laptop | 0 | no | Bounded diagnostic text |
| `cmd/barcode_map` | laptop to robot | 1 | no | Pre-run letter-to-action configuration |
| `reply/barcode_map` | robot to laptop | 1 | no | Acceptance or rejection of that configuration |
| `cmd/snapshot` | laptop to robot | 1 | no | Development request for current cached values |
| `reply/snapshot` | robot to laptop | 1 | no | Number of values replayed |

The robot subscribes to `<root>/cmd/#`. Publishers should not retain command
messages: an old retained mapping must not be silently replayed to a robot
that reconnects later.

## Common telemetry envelope

Every telemetry payload contains:

```json
{"t":128450,"seq":412,"peak_mm":43,"count":2}
```

- `t`: milliseconds since the Pico booted.
- `seq`: one robot-wide sequence number. A gap means that a generated
  telemetry message was not observed.
- All remaining fields belong to that topic.
- Snapshot replays additionally contain `"snap":true`.

Integers are used throughout. A field name carries its unit: `_mm` is
millimetres, `_mms` is millimetres per second, `_dg` is degrees, `_ddg` is
deci-degrees, and `_dps` is degrees per second.

## Telemetry payloads

### Status and heartbeat

```json
{"up":true,"uptime_s":42,"rssi":-58,"phase":"AUTONOMOUS","dropped":0,"coalesced":12,"qdepth":1,"reconnects":0,"cmd_dropped":0}
```

This is published every two seconds and retained. When the connection is
lost unexpectedly, the broker publishes the robot's Last Will:

```json
{"up":false,"reason":"link lost"}
```

`dropped` counts important events displaced because the event FIFO was full.
`coalesced` counts stale periodic samples replaced by newer values.
`cmd_dropped` counts inbound commands discarded because a previous command
was still awaiting main-context processing.

### State

```json
{"t":128450,"seq":400,"state":"LINE_FOLLOW"}
```

### Motion — Buddy 2

```json
{"t":128451,"seq":401,"spd_l_mms":220,"spd_r_mms":218,"dist_mm":1840,"enc_l":4821,"enc_r":4805}
```

### Line and barcode — Buddy 3

```json
{"t":128452,"seq":402,"pos":-3,"junction":false,"ir_l":2100,"ir_r":1980}
```

```json
{"t":128470,"seq":403,"char":"C","action":"RIGHT","reversed":true}
```

`action` is looked up from the shared, accepted `robot_config`; Buddy 1 does
not decide the direction.

### IMU and hump — Buddy 4

```json
{"t":128500,"seq":404,"heading_dg":90,"pitch_ddg":-125,"turn_dps":42,"event":"CLIMBING"}
```

```json
{"t":128900,"seq":405,"peak_mm":43,"max_mm":51,"count":2}
```

### Obstacle — Buddy 5

```json
{"t":129000,"seq":406,"angle_dg":15,"closest_mm":250,"width_mm":120,"clear_l_mm":300,"clear_r_mm":90,"plan":"LEFT"}
```

### Log

```json
{"t":129010,"seq":407,"msg":"recalibrated IR sensors"}
```

## Barcode-map transaction

Publish a complete mapping object to `<root>/cmd/barcode_map` while the robot
reports phase `CONFIGURING`:

```json
{"D":"LEFT","C":"RIGHT","W":"STRAIGHT","F":"UTURN"}
```

Rules:

- Keys are one uppercase letter from `A` to `Z`.
- Values are `LEFT`, `RIGHT`, `STRAIGHT` or `UTURN`.
- The object must contain at least one entry.
- Letters omitted from the object become unassigned.
- Parsing and application are all-or-nothing.
- A duplicate key is allowed; its last value wins.
- Each accepted map increments `version`.
- `ARMED`, `AUTONOMOUS` and `FINISHED` reject changes.

Accepted reply:

```json
{"t":1234,"accepted":true,"version":1,"entries":4,"phase":"CONFIGURING"}
```

Rejected replies use one of `malformed_payload`, `configuration_locked` or
`invalid_map`:

```json
{"t":1300,"accepted":false,"reason":"configuration_locked","version":1,"phase":"AUTONOMOUS"}
```

The laptop must wait for an accepted reply and verify `version`, `entries`
and `phase` before the physical start/arm step. A publish being sent does not
prove the robot accepted it. The robot holds one critical reply in a bounded
slot until lwIP completes its QoS 1 request; local output congestion or a
reconnection therefore delays the reply instead of silently discarding it.

## Snapshot diagnostic

Publishing any payload to `<root>/cmd/snapshot` replays the most recent value
of each state-like telemetry topic. Each replay stays on its normal topic and
includes `"snap":true`. Old log events are excluded. The acknowledgement is:

```json
{"t":1400,"replayed":7}
```

## Reconnection behavior

The networking state machine never blocks navigation. After a failure it
retries with backoff delays of 1, 2, 4, 8 and 16 seconds, capped at 16
seconds. A successful MQTT connection resets the delay and re-subscribes to
`cmd/#`. Telemetry generated while offline remains subject to the same
bounded event FIFO and latest-value rules; communication failure must not
consume unbounded RAM.

## Local demonstration

With the development broker running, monitor everything:

```bash
mosquitto_sub -h localhost -v -t 'grp07/car/#'
```

Configure the pre-run map:

```bash
mosquitto_pub -h localhost -q 1 -t 'grp07/car/cmd/barcode_map' \
  -m '{"D":"LEFT","C":"RIGHT","W":"STRAIGHT","F":"UTURN"}'
```

Request a development snapshot:

```bash
mosquitto_pub -h localhost -q 1 -t 'grp07/car/cmd/snapshot' -m ''
```

For the actual evaluation, replace `localhost` with the broker laptop's
address as needed and stop after the accepted pre-run map and physical start.
