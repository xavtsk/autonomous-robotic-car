# Team and subsystems

Subsystem numbers correspond to Buddy numbers. Motor/encoder and Buddy 5 resource
claims were reconciled with code on 27 October 2026. Physical validation and
remaining owner/interface sign-offs are still required.

Fill this in at the start of the project and keep it up to date. It is the
single place that says **who owns what**, so that five people can work in
one firmware image without clashing over pins, priorities, or kernel objects.

A **subsystem** is one student's part of the product, with its own folder.
It may contain several tasks and own several peripherals (see section 3).

Rule: **only the owner edits a subsystem's folders.** To change another
subsystem, ask its owner or open a pull request for them to review.

## 1. Members and subsystems

| # | Subsystem (what it does) | Student ID | GitHub username | Code | Tests |
|---|---|---|---|---|---|
| 1 | WiFi communication, command and telemetry | | | `app_program/subsystem1/` | `testing/subsystem1/` |
| 2 | Motion control using motors and wheel encoders | | | `app_program/subsystem2/` | `testing/subsystem2/` |
| 3 | IR line following and barcode decoding | | | `app_program/subsystem3/` | `testing/subsystem3/` |
| 4 | IMU-based motion and terrain monitoring | | | `app_program/subsystem4/` | `testing/subsystem4/` |
| 5 | Ultrasonic scanning and obstacle avoidance | | | `app_program/subsystem5/` | `testing/subsystem5/` |

Shared files (`app_program/app_main.c`, `app_program/app.h`, this file)
are changed only by agreement of the team. The exception is claiming an empty
row in this file (a pin, a peripheral, an interface ID), which anyone may do
(section 2).

## 2. Pins

One owner per pin. Claim a pin here **before** you wire or code it.

**Why:** two subsystems using the same pin is one of the most common and
hardest-to-find mistakes in a team project. The code builds without any
error, but the hardware misbehaves. Each subsystem can pass its own tests
and then fail once they are combined, and two outputs driving the same pin
against each other can damage it. Writing the claim down first means a clash
is spotted when someone edits this table, not hours later on the bench.

**No instructor check needed.** The team keeps this table itself. You do not
need the instructor, or the rest of the team, to check or approve a claim:
if the row is empty, fill it in and commit. If two people claim the same
pin, they agree between themselves who moves. Only the *reserved* rows are
fixed.

| Pin | Owner | Used for |
|---|---|---|
| GP23, GP24, GP25, GP29 | *reserved* | Pico W radio |
| GP0 | 5 | HC-SR04 ECHO, user-confirmed 2026-10-27; verify 3.3 V-compatible signal conditioning; conflicts with UART/crash console |
| GP1 | 5 | HC-SR04 TRIG, user-confirmed 2026-10-27; S5 requires USB console, CRASH_REPORT=0 |
| GP2 | 4 | Grove 2 IMU SDA, I2C1 |
| GP3 | 4 | Grove 2 IMU SCL, I2C1 |
| GP4 | 2 | Right encoder A; existing Buddy 2 implementation |
| GP5 | 2 | Right encoder B; existing Buddy 2 implementation |
| GP6 | | |
| GP7 | | |
| GP8 | 2 | Left motor A; existing Buddy 2 implementation |
| GP9 | 2 | Left motor B; existing Buddy 2 implementation |
| GP10 | 2 | Right motor A; existing Buddy 2 implementation |
| GP11 | 2 | Right motor B; existing Buddy 2 implementation |
| GP12 | 5 | Servo signal, PWM6A; user-confirmed 2026-10-27; pulse/travel calibration pending |
| GP13 | | |
| GP14 | | |
| GP15 | | |
| GP16 | 2 | Left encoder A; existing Buddy 2 implementation |
| GP17 | 2 | Left encoder B; existing Buddy 2 implementation |
| GP18 | | |
| GP19 | | |
| GP20 | MOTION_SMOKE and S3_MOTION operator panel | Next calibration step / START (3 s countdown); hold 2 s = recalibrate; no automatic start |
| GP21 | MOTION_SMOKE and S3_MOTION operator panel | Software STOP button; power switch remains the independent fallback |
| GP22 | | |
| GP26 | 3 | Front-RIGHT line IR analogue input (AO), ADC0, Grove 5; wiring confirmed 2026-10-27 |
| GP27 | 3 | Front-LEFT line IR analogue input (AO), ADC1, Grove 6; wiring confirmed 2026-10-27 |
| GP28 | 3 | BARCODE IR analogue input (AO), ADC2, Grove 7 yellow wire; wiring confirmed 2026-10-27; check VBAT jumper |

## 3. Peripherals

**Subsystems, tasks and peripherals are three different things:**

| Term | What it is | Example |
|---|---|---|
| **Subsystem** | one student's part of the product, one folder | "environment sensing", "motor control", "user interface" |
| **Task** | one thread of execution. A subsystem may have **as many tasks as it needs** | a sampling task and a filtering task, both in "environment sensing" |
| **Peripheral** | one hardware block on the RP2040 | I2C0, UART1, the ADC |

A driver is **not** a subsystem. It is code *inside* the subsystem that
needs that hardware: the "environment sensing" subsystem contains the I2C
sensor code, and owns I2C0. Write it as `drv_<name>.c` / `drv_<name>.h` in
that subsystem's folder, never under `device/`, and call it only from the
peripheral's owning task. The kernel's own drivers in `device/` (serial,
I2C, ADC) may be used instead; switching one on in `config/config_device.h`
changes shared configuration, so record it in the table below first.

**The rule: each peripheral is driven by exactly one task, its *owning
task*.** That task can be in any subsystem, and one subsystem may own
several peripherals (one owning task each, or one task owning all of them).
Every other task, in the same subsystem or another one, asks the owning task
through a kernel object (section 5) and never touches that peripheral itself.

*Why:* this port runs tasks on both cores at the same time. If two tasks
drive the same I2C or UART block at once, their transfers get mixed up, and
the drivers have no locking to stop it.

| Peripheral | Owner subsystem | Owning task | Pins |
|---|---|---|---|
| UART1 | | | |
| I2C0 | | | |
| I2C1 | 4 | Buddy 4 sensing task, S4=1 | GP2/GP3 |
| SPI0 | | | |
| SPI1 | | | |
| ADC | 3 | `ir-sample` in bench or `s3-motion` in integrated profile; exclusive kernel `adca` driver | GP28 barcode only in default bench; GP27 left/GP26 right enabled with IR_CHANNELS=3 or the integrated profile. Mapping: `S3_ADC_CHANNEL_*` in `ir_acquisition.h` |
| PWM4/5 | 2 | motion task | GP8–11; 10 kHz motor PWM |
| PWM6A | 5 | obstacle-scan task | GP12; 50 Hz servo, never reset other slices |
| PIO0 SM0/SM1, words 0–23 | 2 | motion task | GP16/17, GP4/5 encoders; initialises before S5 |
| PIO0 SM2, words 24–31 | 5 | obstacle-scan task | GP0 ECHO; 1 MHz pulse counter; atomic per-SM control |
| PIO1 | kernel | CYW43 owner | Wi-Fi; Buddy 5 does not touch it |
| WiFi radio, TCP/IP, MQTT | *kernel* | `cyw43_utk_task` (built in) | GP23–25, GP29 |

### WiFi and MQTT are special

The radio and the network stack belong to the kernel's built-in WiFi service
task, `cyw43_utk_task` (`lib/libwifi/sysdepend/pico_rp2040/`). **No student
task may call `cyw43_...`, lwIP or MQTT functions, not even from a
"WiFi subsystem".** The calls are not safe from any other task, on either core.

What students *can* do:

- **Read the connection state** from any task, which is always safe:
  ```c
  #include "cyw43_utk.h"
  T_CYW43_UTK_STATUS st;
  cyw43_utk_get_status(&st);   /* st.ready, st.link_status, st.ip_addr, ... */
  ```
- **Send or receive data over the network** by adding code that *runs
  inside the service task*. The MQTT client is in
  `lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c`. It publishes a heartbeat
  today. A "cloud / reporting" subsystem would add a kernel object
  (e.g. a message buffer) that application tasks fill. The MQTT code then
  reads it with `TMO_POL` (never blocking) and publishes it.
  Because this changes shared library code, **agree it with the team and
  the instructor first**, and record it as an interface in section 5.
  It needs the `NET=lwip MQTT=1` build (docs/UPSTREAM_README.md section 3).

## 4. Task priorities

1 is the highest. 1–4 are used by the kernel (USB console 3, WiFi 4). Each subsystem gets its own band by default;
change it only by team agreement. Write down every task you create.

| Subsystem | Priority band | Tasks (name – priority – core) |
|---|---|---|
| 1 | Command service 22 | `buddy1-cmd` - 22 - core 0; radio/lwIP/MQTT remain on kernel task 4 |
| 2 | 14–15 | motion task - 14 - core 0; 10 ms cyclic; sole motor writer |
| 3 | 16–17 | `ir-sample` - 16 - core 0; `ir-report` - 17 - core 0; bench with `IR_BENCH=1`; alternatively `s3-motion`/`s3-status` at the same priorities/core with `S3_MOTION=1` |
| 4 | 18–19 | IMU sensing - 18 - core 0, when S4=1 |
| 5 | 20–21 | obstacle-scan - 20 - core 0, when S5=1 and hardware configuration enabled |
| Controller operator panel | 24 | `operator` - 24 - core 0; `S3_MOTION=1` only; polls GP20/GP21 every 10 ms; talks only to `s3_integration` |

Core: `TP_PRC1` = core 0, `TP_PRC2` = core 1, or "any".

The optional diagnostics reporter runs in the initial task lowered to priority
23; it adds one mutex. The old priority-15 Vehicle Controller task is not started:
`s3-motion` is the sole navigation/motion-request/report owner in the integrated
profile. Application tasks retain 2048-byte stacks; runtime high-water marks and
radio-induced latency still require on-car measurements.

## 5. Interfaces between subsystems

Current integration, 25 October 2026: Buddy 1's binary v2 service is connected
to the single `s3_integration`/`s3_drive` controller through the taskless
`vehicle_controller` facade. Its pre-run A=forward/B=left/C=right/D=U-turn map
(or a validated remote replacement) is atomically frozen/copied on explicit
start. The standalone sensor default mapping below is not authoritative at
team start. Runtime requests/status use 11/146-byte explicit packets under
the existing snapshot mutex. The older two-buddy-only description below is
the imported handoff history; see `docs/INTEGRATION_2026-10-25.md` for current
ownership, tests and open issues. Buddy 5 arbitration now extends this same owner when S5=1; Buddy 4 run/event/fault arbitration is also connected through this owner when S4=1.

Every arrow between two subsystems is listed here. The **provider** creates
the kernel object and publishes its ID in its own `subsystemN.h`; the **user** only
sends to it or waits on it. Agree on the data format before coding.

| ID | From → To | Mechanism | Object ID name | Data / meaning | Agreed by |
|---|---|---|---|---|---|
| S4-I1 | S4 sensing → controller | 11-byte packet + 16-bit run/count metadata, protected snapshot; eight-event queue | existing S4 mutex | Terrain/tilt/rate, peak/count, validity, impact and loss | Integration policy 28 Oct; physical validation pending |
| S4-I2 | Controller → S4 | Bounded reset request with 16-bit generation | existing S4 mutex | Clear run statistics; wait for matching publication before driving | Same |
| S4-I3 | Controller → Buddy 1 | Existing hump telemetry and masked IMU fault API | existing Buddy 1 mutex/outbox | Count/highest peak; update only IMU fault bit | Binary MQTT v2 unchanged |
| S5-I1 | S5 sensing → controller | 23-byte protected snapshot | private S5 mutex | Source-timed forward range, validity/no-echo, identified scan result and profile | Implementation in Xavier branch; physical validation pending |
| S5-I2 | Controller → S5 sensing | Bounded 16-bit scan token + cancel flag | same S5 mutex | Request scan only after stopped ACK; sensing owner independently checks stopped feedback | Same |
| S5-I3 | Controller → Buddy 2 | Existing Motion API and identified reports | existing motion objects | One requester/report consumer; S5 task never commands motors | Existing ownership preserved |
| S5-I4 | Buddy 3 → controller recovery | Existing fresh line observations | same controller context | Tape evidence near predicted original corridor; suppress barcode/junction actions during detour | Physical original-line identity still needs proof |
| S5-I5 | Controller → Buddy 1 | Existing telemetry API | Buddy 1 mutex/outbox | Closest distance, conservative width, chosen route, state; only owned NO_ECHO bit modified | Binary v2 unchanged |

Buddy 3's pure context API in `app_program/subsystem3/line_navigation.h` emits
observations, steering proposals in m/s and edge events. The controller supplies
source-timestamped signed centre distance and reset epochs, serialises accepted
calibration/start controls and turns persistent stop requests into Buddy 2
braking. The sensing core calls no motors. Buddy 5 now extends this owner under S5=1. Buddy 4 uses the same owner for reset acknowledgement and fault handling; no
competing motion callers/report consumers are added.

Upstream Buddy 2 `a29f973` uses PIO0 SM0/SM1 for encoders and retains its
measured settings. Imported `71193e2` adds timestamp/epoch feedback, wrap-safe
centre displacement, report loss and the opt-in navigation lease. Buddy 1's
v2 telemetry range is narrower; the adapter invalidates out-of-range speeds.
See `app_program/controller/README.md` for the current handoff.

The car's wiring confirmed on 2026-10-27 differs from the Week 6 L/C/R pin order:
GP27 front-left, GP26 front-right, GP28 barcode. The code follows this wiring
via `S3_ADC_CHANNEL_*` in `ir_acquisition.h`. The
bench's 10 ms target also differs from Week 6's 5 ms Line+Barcode schedule; no
shared tick or priority changes are made here. See the handoff traceability table
for scheduling, mapping delivery, distance tuning and fault-to-brake decisions.

Each interface is tested by at least one integration test
(`testing/integration/TEST_LOG.md`).

## 6. Kernel object budget

Limits are set in `config/config.h`. If the team needs more, raise the
limit there. Buddy 2's task, mutex and cyclic handler are now recorded from its implementation;
this is still not a full kernel/library resource audit.
Buddy 3 uses two tasks, one snapshot/control mutex and one 10 ms cyclic handler
in either opt-in profile (bench or integrated; never both). The existing ADC driver owns its completion flag; Buddy 3
does not create another flag. The kernel has its own usage.

| Object | Limit | S1 | S2 | S3 | S4 | S5 |
|---|---|---|---|---|---|---|
| Task | 32 | 1 | 1 | 2 in either opt-in S3 profile | 1 when S4=1 | 1 when S5=1 |
| Semaphore | 16 | 0 | 0 | 0 | 0 | 0 |
| Event flag | 16 | 0 | 0 | 0 | 0 | 0 |
| Mailbox | 12 | 0 | 0 | 0 | 0 | 0 |
| Mutex | 12 | 1 | 1 | 1 in either opt-in S3 profile | 1 when S4=1 | 1 when S5=1 |
| Message buffer | 12 | 2 (4 outbox / 2 commands) | 1 (8 packed completion reports) | 0 | 0 | 0 |
| Fixed memory pool | 12 | 0 | 0 | 0 | 0 | 0 |
| Variable memory pool | 12 | 0 | 0 | 0 | 0 | 0 |
| Cyclic handler | 12 | 0 | 1 | 1 in either opt-in S3 profile | 0 | 0 |
| Alarm handler | 12 | 0 | 0 | 0 | 0 | 0 |

(The kernel's USB and WiFi service tasks also use a few of these.)

## Buddy 4 implementation, 27 October 2026

S4=1 enables the priority-18, core-0 sensing task (4096-byte stack) and one
priority-inheritance mutex. S4_BENCH=1 is the isolated stationary sensor build.
The task exclusively owns iicb. Shared snapshot and eight event slots use explicit
11-byte packets; overflow drops the oldest event and reports loss.
Integrated 28 October: the existing sampling/controller owner consumes S4 events,
waits for a post-reset run token before motion, and faults/stops on stale data,
IMU fault, impact candidate or queue loss. S4 itself never commands motors.
Buddy 1 receives cumulative completed-hump count/highest valid peak and masked IMU
fault updates. USB reports terrain/tilt/turn rate. Internal packets now use 83
meaningful bits in 11 bytes; run/count are protected 16-bit metadata. No new task
or mutex was added for the controller adapter. See the Buddy 4 integration guide.
Calibration, magnetic interference, impact sensitivity, height accuracy and
full-load timing remain physical validation work.

## Buddy 5 implementation, 27 October 2026

Pins confirmed by Xavier: HC-SR04 ECHO GP0, TRIG GP1, servo signal GP12.
`S5=1` requires `S3_MOTION=1`, USB console and `CRASH_REPORT=0`.
Hardware activation remains separately opt-in (`S5_HARDWARE_CONFIRMED=1`) after
checking level protection, servo pulse/travel calibration and route geometry.
`S5=0` is the default. No firmware was flashed in this implementation session.
The S5 task waits for valid Buddy 2 feedback before initialising PIO0 SM2; do not
reinitialise the whole PIO0 block after either owner starts. Buddy 2 source is
unchanged; its historical “PIO0 ours alone” comment predates this allocation.
See `app_program/subsystem5/README.md` and `DATA_FORMAT.md` for limits and tests.
