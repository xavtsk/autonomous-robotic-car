# Local robot dashboard

The laptop dashboard replaces the separate terminal viewer and map/snapshot
commands with one browser window. It uses Buddy 1's existing binary protocol v2
and the original `tools/buddy1_mqtt_codec.py`. Existing telemetry needs no firmware
rebuild. The new live three-sensor graphs require the optional `IR_DEBUG=1`
firmware described below.

## Start without the car

From the repository root:

```sh
python3 tools/robot_dashboard.py --demo --open
```

The browser opens `http://127.0.0.1:8765`. The blue **SIMULATED ROBOT** banner
identifies generated readings. Select Ready, Following line, Barcode decoded,
Line lost, Connection lost or Motor fault, then press Load. Ready simulates a
fresh robot boot so another map can be tried. Demo commands and captures remain
separate from live mode and never connect to an MQTT broker.

Appearance in the header selects System, Light or Dark and remembers the choice
in this browser. Blue marks navigation and active motion, green marks confirmed
connections/replies, amber marks stale/pending data, and red marks reported
faults or rejected commands. The simulation banner uses an informational blue.

## Start with the robot

Keep the existing Mosquitto broker running, with the Mac and Pico on the same
hotspot. If the broker is running on this Mac:

```sh
python3 tools/robot_dashboard.py --connect --open
```

Or supply the broker address explicitly:

```sh
python3 tools/robot_dashboard.py --broker 172.20.10.2 --connect --open
```

Use the Mac/broker address, rather than the Pico's address. A changing hotspot
address can be edited under **Broker connection**. The default port is 1883;
`--mqtt-port` changes it. Mosquitto's `mosquitto_sub` and `mosquitto_pub` must be
installed and available on PATH (`brew install mosquitto` on macOS). The Python
server uses only the standard library; no Python package installation is needed.

If no broker is already running, the existing lab configuration can be started
in another terminal using `mosquitto -c mosquitto-dev.conf -v`. Do not start a
second broker on a port already in use. The dashboard combines monitoring and
setup commands; it does not start or reconfigure the broker itself.

Wait for **Broker connected** and recent car traffic. In Overview, enter four
different uppercase letters, one each for Forward, Left, Right and U-turn.
Press Send map and wait for the robot's OK reply. The browser's draft letters
are not a readback of the active map. Use one command sender during setup:
the existing protocol has no request ID or echoed map, so concurrent terminal
commands and delayed replies cannot be conclusively correlated.

GP20 and GP21 remain the physical calibration/start and software-stop controls.
The physical power switch is the independent cutoff. The dashboard supports
only the existing map and snapshot commands. Test the first live dashboard
session with the car stationary, using a snapshot and map reply before a run.

The firmware freezes mapping after the first accepted START until a physical
robot reboot. A broker reconnect does not clear that rule. If the dashboard has
seen an active run or a BUSY/closed reply, it labels the map previously locked.
After a robot reboot, its explicit Retry button lets the firmware decide whether
the next map is allowed; the dashboard cannot identify a reboot automatically.

## Pages

**Overview** shows robot state, left/right speeds, last barcode/action and
reported faults. Heading, hump and obstacle summaries appear when actually
received. Broker connection and recent robot traffic have separate indicators.
Motion becomes stale after 3 seconds without a report. Robot traffic becomes
stale after 15 seconds. State, faults and barcode are event/last-report values,
not periodic sensor measurements. Ages are laptop receive ages.

**Route** (Overview) draws the run from `car/telemetry/pose`: distance and
heading since START from Buddy 2's wheel encoders, every 200 ms. Barcode, hump,
obstacle, turn and run-end events are placed at the latest pose when they arrive
and listed with their distance from START. The last five runs can be selected.
The drawing is dead reckoning: wheel slip during spins and on humps bends it, so
it is a route sketch, not a surveyed map. Needs firmware from 28 Oct 2026 or
later; older firmware leaves the section empty. Demo mode draws a simulated oval.

Snapshot values are firmware-cached values without source timestamps. A newly
received snapshot is labelled Cached / age unknown, including valid zeroes.
Invalid values remain unavailable. No reported fault bits does not establish
health for every subsystem; the firmware does not publish fault-source coverage.

**Debug** has one view per buddy and a download of the recent received log:

Buddy 1 also exposes a collapsible **Received MQTT packets** inspector with the
latest eight validated binary payloads, their byte lengths, topics and decoded
fields. Pause the packet view to inspect it while telemetry keeps arriving.
Decode failures are recorded in Related activity. Payload byte totals exclude
MQTT framing and do not measure the Pico's RAM or network bandwidth.

| Buddy | Current MQTT evidence | Further firmware work |
|---|---|---|
| 1 | Broker connection, heartbeat, decoded packet count/errors, map and snapshot replies | Actual MCU queue usage, RAM and Wi-Fi association/DHCP/retry diagnostics |
| 2 | Wheel speeds/fault bits; updated optional diagnostics add ramped PID setpoints, measured speed, signed motor duty, encoder counts, motion state and brake acknowledgement | PID component terms, measured stop latency |
| 3 | Barcode/action, state, IR fault, recorded captures and 20 Hz IR preview; updated diagnostics add calibration references/ranges, line error, pending action, event counts and controller reason | Exact per-event timestamps, original-line identity after recovery |
| 4 | With `S4=1`: hump count, run peak height, IMU fault bit. The heading topic stays reserved (empty) | Tilt, raw IMU and motion events are USB-only |
| 5 | With `S5=1 S5_HARDWARE_CONFIRMED=1`: closest distance, width and route per completed scan, no-echo fault, "avoiding obstacle" state | Servo angles, scan samples and left/right clearance are not published |

The debug pages include rolling histories, graph pause/window selection and
**Save debug evidence**, which exports bounded JSON with source/validity metadata.
Pausing freezes graphs, not the latest-value table or the robot. Cached snapshot
values are excluded from telemetry histories; they must not look like new sensor
measurements. Individual hump/obstacle reports are plotted as points, not a
fabricated continuous signal. Heading plots break at a 360°/0° wrap.

Buddy 1 adds messages/second and application bytes/second (completed laptop
receive-time buckets), plus laptop/broker connection history. These include
diagnostic traffic and do not measure the Pico's queue depth or RAM. Buddy 2
overlays ramped PID setpoint and measured speed for each wheel; without the
new controller stream it falls back to measured-speed-only plots. Buddy 3
includes a line-error graph and a per-sensor calibration table. Buddy 4 plots
heading/hump reports only when supplied, and Buddy 5 similarly plots obstacle
distance/width. Pitch/roll and distance-versus-scan-angle are explicitly marked
not published: the dashboard cannot derive them from summary values.

Controller decisions and observed IR sampling gaps are available with the
updated diagnostic firmware, but task execution times, stop latency and memory/
queue measurements remain unavailable. MCU motion/PID, line-following and sensor
algorithms are unchanged; Buddy 2 gained read-only accessors under its existing
mutex, not a new motor writer or controller task.

**Sensors** plots the front-left (GP27), front-right (GP26) and right-side barcode
(GP28) channels on aligned time axes. Choose normalized darkness (0–1000) or raw
ADC (0–4095), and a 5/15/30/60-second window. The strip under each curve shows
the firmware's actual white/black classification; the browser does not invent
a threshold. Valid zero readings are retained, unavailable readings are gaps,
and a stream older than one second is marked stale without replacing it with
zeroes. Normalized values/classification do not themselves prove that physical
calibration has been accepted; use the operator calibration prompts for that.

Pause freezes the visible graphs while reception continues. Save visible samples
CSV exports that window, source clocks, validity, classification and sequence
information. The laptop retains at most 1200 frames (about a minute at 20 Hz).
Unobserved sequence numbers include frames superseded by the latest-only mailbox
or skipped under load; they are not a measurement of Wi-Fi packet loss alone.
The separately received last decoded letter is shown below the barcode graph;
it does not prove that a turn completed.

### Enable wireless sensor graphs on the car

Build from the repository root, with the existing private Wi-Fi configuration:

```sh
export PICO_SDK_PATH="/Users/zhaobao/pico/pico-sdk"
make -C build_make CONSOLE=usb_cdc WIFI=cyw43 NET=lwip MQTT=1 TEST=0 IR_BENCH=0 S3_MOTION=1 IR_DEBUG=1 DIAGNOSTICS=0 MOTION_SMOKE=0 S4=0 -j8
```

The output is **`build_make/mtk3pico_smp1_usb_cdc_wifi_lwip_mqtt_s3_motion_ir_debug.uf2`**.
The suffix distinguishes it from the ordinary course build. To install:

1. Stop the car, switch off/disconnect motor power, and raise both wheels on a
   stable support. Close any serial viewer before reconnecting the Pico.
2. Unplug the Pico's micro-USB cable. Hold BOOTSEL on the green Pico W, connect
   its micro-USB cable to the laptop, then release BOOTSEL. Confirm RPI-RP2 appears.
3. Copy only the diagnostic course firmware:

   ```sh
   cp -X build_make/mtk3pico_smp1_usb_cdc_wifi_lwip_mqtt_s3_motion_ir_debug.uf2 /Volumes/RPI-RP2/
   ```

4. The drive normally disappears as the Pico restarts. Keep the existing broker
   running, start the dashboard in live mode, connect it to that broker and open
   Sensors. The graphs must say **Wireless**, not Simulated. No motor power is
   needed just to observe sensors when the sensor supply is powered correctly.
5. With the car stationary, check that moving a white/black surface under each
   physical sensor changes the matching graph. Complete the physical GP20
   calibration as usual. These graphs do not start the car or bypass readiness.
6. Only after the stationary check, prepare a supervised course run using the
   existing GP20 start and GP21 software stop; keep the power cutoff accessible.
   Measure control timing with diagnostics enabled in a separate validation run
   before relying on this preview for a demo. Flashing restarts calibration/map
   state as defined by the existing firmware.

The stream is optional and off by default. Rebuild without `IR_DEBUG=1` and
flash the ordinary course UF2 to disable it. No remote debug enable/start/stop
command was added.

The updated build also sends a **70-byte controller snapshot at up to 5 Hz**
on `car/debug/controller`. Flash this newly rebuilt UF2 even if you previously
flashed the IR-only diagnostic build: the filename is unchanged but the contents
now include the buddy-debug fields. Its motion read uses a nonblocking lock;
busy, stale or out-of-range feedback is unavailable, not zero-valued evidence.
The dashboard remains usable with ordinary firmware; the extra fields then
remain unreceived. No firmware was flashed automatically.

The wire preview is capped at **20 Hz**, in 16-byte QoS-0, non-retained packets.
Normal telemetry and recorded barcode uploads take precedence, followed by the
5 Hz controller snapshot and then IR preview. Each has one latest-only mailbox;
congested or stale previews are dropped, not replayed.
The motor and line/barcode algorithms still run at their existing 10 ms cadence.
Added packing/network work still needs on-car timing measurement: host tests do
not establish real-time deadlines. See [IR_DEBUG_PROTOCOL.md](IR_DEBUG_PROTOCOL.md).

**Use the preview to check mounting, calibration contrast, sensor polarity and
line behaviour. Do not use it as a complete barcode waveform:** narrow bars can
fall between 50 ms preview frames. The higher-rate saved barcode capture in Runs
remains the evidence for individual bar widths and decoding failures. Simulation
curves demonstrate the UI and are not real sensor or barcode-decoder evidence.

**Runs** accepts saved `barcode_trace_*.txt` files and automatically saves
incoming `car/debug/barcode` captures. Inspect a run to plot raw ADC or normalized
darkness by episode and save the original text. Files up to 2 MiB are accepted.
The recent list holds 40 captures in memory; older capture files stay on disk.
The archive location is printed at startup; the default is `.dashboard-data/live`
or `.dashboard-data/demo` in the repository, both ignored by Git. Original
imported files are never modified. Use `--data-dir /path/to/folder` for another
location.

A complete transfer requires the summary, advertised episodes/samples and exact
end marker. Firmware `not_stored` capture opportunities are reported separately
from missing transfer records, decoder overflow and dashboard size limits.
Retransmitted sample rows are deduplicated by episode and sample offset. Sample
time is approximate because firmware intervals are integer milliseconds;
saturated intervals or missing samples make later time unknown, and the plot
breaks rather than inventing continuity. Wait for upload completion before the
next GP20 START, which can interrupt the firmware's upload.

## Verification

Run the independent dashboard suite:

```sh
python3 -m unittest discover -s testing/dashboard -v
node --check tools/dashboard/static/app.js
node --check tools/dashboard/static/sensor_graphs.js
node --check tools/dashboard/static/buddy_graphs.js
node --check tools/dashboard/static/route_map.js
```

The service tests bind loopback ports and launch a disposable Mosquitto broker on
a separate ephemeral port. They never contact the hotspot broker or the car.
They cover exact binary command bytes and replies, reverse wheel speeds,
malformed and retained packets, freshness, cached snapshot validity, command
timeouts, map closure, HTTP origin/session checks, trace imports/downloads and
bounded memory. The optional real-capture test checks the successful local
`barcode_trace_20261027_221613.txt`: ZZ, two decodes and 1927 samples.

The IR tests compile the production C packet codec and compare its bytes with
the independent Python decoder. They cover validity, reserved values, bounded
history, source-clock/sequence wrap, gaps and an isolated real-MQTT IR transfer.
Controller/network host tests also cover the latest-only mailbox and priority
behind normal telemetry and barcode uploads. No real robot timing or wireless
performance result is implied by these tests.

The server binds only to 127.0.0.1, validates Host/Origin and requires its session
token for changes. Robot payloads retain their original compact encoding;
browser JSON exists only on the laptop. MQTT authentication/TLS and remote
hosting are outside this lab dashboard's current configuration.

Close it with Control+C in its terminal. `--port 8766` selects another web port
if needed. The web server currently targets macOS/Linux; its Mosquitto bridge
uses a private terminal descriptor to receive connection acknowledgements
without waiting for stdout buffering.
