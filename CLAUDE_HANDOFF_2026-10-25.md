# Claude handoff: autonomous robot, Buddies 1–3

Prepared by Codex for Zhaobao on **25 October 2026**, Asia/Singapore.
This is a continuation briefing, not proof that the robot is physically ready.
Read the current files and recheck Git status before acting: the user may have
made further changes after this document was written.

## 1. Start here: what the user wants

The user is handing this project from Codex to Claude in Work mode, with access
to the same local folders. They want Claude to understand what has already been
done and continue without starting over, losing changes, or confusing old design
claims with actual implementation/test evidence.

The immediate work just completed was:

> “My teammate updated the GitHub repo, I just git pulled it, verify and help me
> integrate to our branch.”

That integration is now complete **locally**, tested in software, and documented.
The next practical step is to test the **rebuilt isolated motor-test image with
the wheels raised**, with the user's explicit participation. It has not been
flashed or physically tested after the integration. Do not automatically flash,
start motion, commit, push, merge more branches or implement a new autonomous
control UI merely because this handoff describes them. Confirm the user's next
requested action and the current physical setup.

### The most important facts

1. Work in the **Project** repository below, not a similarly named older folder.
2. Branch is `integration/buddies-1-3`; HEAD is `71193e2`, with significant
   **unstaged and untracked local work** that is part of the intended project.
3. All merge conflicts were resolved; no new commit or push was made.
4. A recoverable pre-integration stash remains. **Do not blindly apply it again.**
5. Use the teammate's latest motion/PID implementation; do not replace it with
   old local copies or retune it without user agreement/evidence.
6. Buddy 2 alone writes motor outputs. There is one navigation/motion-request/
   completion-report owner, not two competing controllers.
7. Default firmware does not run navigation. `S3_MOTION=1` is opt-in and starts
   disarmed. It still needs an operator-facing calibration/start adapter.
8. GP20/GP21 start/stop work in the **isolated smoke-test profile only**.
9. The new motor-test firmware includes the teammate's PWM-reset fix, but actual
   movement/stopping on this car is **not yet confirmed**.
10. The network image labelled `_COMPILE_ONLY` uses public dummy settings.
    **Do not flash it.**

## 2. Working folders and authoritative references

Workspace root:

```text
/Users/zhaobao/Documents/SIT/SIT-Y2-Tri1/Embedded System
```

**Active repository:**

```text
/Users/zhaobao/Documents/SIT/SIT-Y2-Tri1/Embedded System/Project/autonomous-robotic-car
```

There is ALSO an `autonomous-robotic-car` folder directly under the workspace
root, and a `Project/buddy1` folder. They are not the active integrated checkout.
Do not edit/build the wrong repository based on name alone.

Unless a path is absolute, paths below are relative to the active repository.

### Read these first

1. `docs/INTEGRATION_2026-10-25.md` — current integration provenance, architecture,
   tests, builds, compatibility limits and remaining work.
2. `app_program/controller/README.md` — current single-owner controller contract.
3. `testing/diagnostics/MOTION_SMOKE.md` — safe isolated motor-test procedure.
4. `testing/diagnostics/ROBOT_TEST_SESSION.md` — evidence collection/test stages.
5. `TEAM.md` — resource, pin, task and object ownership; some imported historical
   handoff wording is explicitly superseded by the current integration notes.
6. `app_program/subsystem1/MQTT_PROTOCOL.md`, `subsystem1.h` and the live facade
   implementation — Buddy 1 wire/API contract.
7. `app_program/subsystem2/DATA_FORMAT.md`, `motion.h`, `wheel_encoder.h` — latest
   teammate formats and motion/PIO contract.
8. `app_program/subsystem3/MOTION_INTEGRATION.md`, `INTEGRATION.md`,
   `line_navigation.h` — sensor/control API, calibration, state and timing gates.

### Course/project documents in the workspace root

These files were located on disk when preparing this handoff:

```text
AAI07_Week_6_Design_Review.pdf
Week_6_Design_Diagrams.drawio.pdf
Change Requirement 1.docx
AAI07_ImmTri1.pdf
AAI07_PreTri1.pdf
AAI07_Tribunal1_VersionB_Updated_Draft.docx
Week8_Tribunal1_Template.docx
```

Relevant files under `Project reference files/`:

```text
Project Write-up.pdf
Project Briefing.pdf
briefing transcript.md
Barcode & Line Specification.pdf
Barcode Sample.pdf
Robo_Pico.pdf
Robo Pico Board Datasheet.pdf
RP-008312-DS-2-pico-w-datasheet.pdf
Infrared-Reflective-Sensor-UserManual.pdf
tcrt5000.pdf
barr_c_coding_standard_2018.pdf
```

Read the actual relevant documents when checking requirements. Do not treat a
generated draft, this handoff, or an old assistant claim as the lecturer's exact
wording. Skip unrelated lectures/labs/files unless they help answer the task.
Temporary/older exports under `tmp/` are not automatically the authoritative
submitted Week 6 design. Office `~$` files are lock files, not alternate designs.

The user previously attached tribunal PDFs and photos from Downloads or Codex
attachments. Some paths may no longer exist; the root documents above are the
stable starting points. If hardware identification matters, ask for a fresh
photo rather than inferring from an old board description.

## 3. How to work with this user

- Explain in plain language, briefly first. The user is learning hardware and
  prefers one concrete next action over a large abstract explanation.
- For physical setup/flashing, **do not skip steps**. State which green Pico W,
  which USB socket, which board/button, which file, what to expect, and whether
  power should be disconnected. Do not assume they know USB vs motor power.
- Give exact copyable terminal commands with quoted paths containing spaces.
- They became frustrated by long preparation while waiting to see wheels move.
  Prioritise a small, safe, observable motor test before full autonomous work.
- Preserve unrelated work. Review before delivery and explicitly say what was
  tested, what was changed and what remains unverified.
- They explicitly wanted minimum-bit changes **without changing Buddy 2 motion
  logic**. The recent integration imports the teammate's newer implementation;
  additional changes must not silently tune PID, calibration, steering, state
  transitions or timing policies.
- They want Buddies **1, 2 and 3 first**. Buddy 4 then Buddy 5 should be developed/
  checked independently and integrated one at a time for easier debugging.
- Do not accept every AI suggestion. Challenge assumptions and explain why a
  recommendation fits their actual design and what evidence would validate it.
- Keep physical observation separate from logs, inference and host simulation.
- This document is context, not authority to perform new external/destructive
  actions. Do not force-reset, clean away untracked files, push or flash by default.

## 4. Project, roles and course requirements

INF2004 embedded-systems project: a Raspberry Pi Pico W on a Cytron Robo Pico
robot car, application code in C on **micro T-Kernel 3.0**, based on the supplied
project template. This is not a drop-in standalone Pico SDK/CMake application
and not a FreeRTOS project. Use the existing kernel/BSP/build structure.

### Buddy roles

| Buddy | Role | Main deliverables from the user's project brief |
|---|---|---|
| 1 (user) | Wi-Fi/MQTT, commands and telemetry | Communication API, telemetry framework, MQTT documentation, demonstrated telemetry |
| 2 | Motion/motors/encoders | Motion library, PID tuning report, motion accuracy evaluation |
| 3 | IR line/barcode sensing | Line-following module, barcode decoder, navigation command interface |
| 4 | IMU/terrain | IMU processing, hump detection, terrain analysis report |
| 5 | Ultrasonic/servo obstacle scanning/avoidance | Scan subsystem, obstacle profile, avoidance and recovery algorithms |

Important user clarification: **two IR sensors for line detection, one separate
IR sensor for barcode reading**. Buddy 3 does not own motor PWM. It produces
sensing/navigation information, including a steering proposal in the current
code; the controller decides/requests motion from Buddy 2. Do not remove/relocate
that proposal calculation merely to make the implementation sound more
“sensing only”; that would be a separate functional change.

The lecturer advised working on each buddy's own drivers/modules first, then
gradual integration. Tribunal 1 changes **must be implemented and demonstrated
in Week 13 Final Demo + Code/Test Review**. Software compilation alone does not
meet the physical demonstration or accuracy-evidence requirements.

### Tribunal background and what the requirement changed

The injected requirement is in `Change Requirement 1.docx`: minimise bits for
application data, agreed packed/binary representations and safe concurrent
access, with resource/correctness/timing reasoning and validation. Check the
document for exact scope/wording; do not reduce it to “make MQTT smaller only.”

The team discussed valid ranges, units, resolution, direction, validity,
overflow, packing races, memory savings and timing costs. Their earlier wheel
speed proposal was eight magnitude bits at 2 mm/s plus a reverse bit. The
teammate later measured a higher maximum speed and widened Buddy 2's own packed
speed magnitude to nine bits. Buddy 1 v2 remains narrower — see the open issue.

The user explicitly excluded **electrical energy optimisation** from the
software-budget discussion. They mean CPU scheduling/load, RAM/stacks/heap,
buffers, processing time and communication bandwidth. Physical power diagnosis
is still relevant if motors will not move; do not confuse it with an electrical
power-budget redesign.

In earlier Version A/frozen-architecture analysis, the user wanted the Week 6
design used as the baseline and no current code used to retroactively justify
Week 6 claims. That was a **report-analysis scope**, not a permanent ban on code:
the user subsequently explicitly authorised current implementation and integration.
Keep original design claims, proposed decisions, implemented code and measured
evidence distinguished in reports.

The lecturer's forum feedback: AI may identify possible impacts, but students
must ask why/where in their system, magnitude, trade-offs, dependencies, what to
change, whether AI's assumptions hold, and how to verify. They may accept AI
when they understand and agree; no need for jargon or 90 minutes of scripted
content. Version B records real discussion/decisions; Version C critiques AI
instead of accepting every suggestion. Do not fabricate Zoom events or test data.

## 5. Git state and the latest integration

Verified when writing this file:

```text
branch: integration/buddies-1-3
HEAD:   71193e22a8e8ce08f71c1ec94ff928bd84f3123f
tracks: origin/buddy3
```

Relevant revisions:

```text
71193e2 Update Buddy 3 integration for latest Buddy 2 motion interface
9be6b6e Integrate Buddy 3 navigation with Buddy 2 motion control
477230c Implement Buddy 3 sensing and navigation core with stationary diagnostics
816d7f0 earlier Buddy 2 implementation

origin/Buddy2 = a29f973 (latest motion branch fetched at integration time)
```

The user's pull fetched new references, but the old working checkout initially
remained at `477230c` with lots of local work. Codex:

1. Saved tracked AND untracked local work in a stash.
2. Fast-forwarded the integration branch to `origin/buddy3` at `71193e2`.
3. Applied the saved work and resolved overlapping/conflicting files.
4. Adopted the teammate's current motion/navigation core, adapted our codecs,
   reporting facade, profiles and tests, and preserved original local tools.
5. Marked all conflicts resolved and restored the originally unstaged working
   pattern. Nothing remains staged; nothing was committed or pushed.

### Recovery snapshot — retain it

```text
cfde7c4c7d4631f184d7e76c305a69eca2fd57ab
stash@{0}: On integration/buddies-1-3: Safety snapshot before integrating Buddy2
a29f973 and buddy3 71193e2 2026-10-25
```

Use its **hash**, not an assumed stash index, if more stashes have been added.
Do not `stash pop/apply` it blindly: its old motion/controller API would conflict
with the completed integration. Use read-only `git show` to inspect a specific
file; the stash third parent contains the saved untracked files. The old
untracked `app_program/controller/README.md` collided with the incoming tracked
README; a current merged README is present and the old one remains recoverable.

All saved untracked paths were checked to exist after integration. Many new
source files still appear as `??` — they are essential, not disposable artifacts.
Several deleted Buddy 1 scaffold `.c/.h` files were already deleted in our
pre-integration work and replaced by the real service implementation. Do not
restore those stubs as a “fix” or interpret all working-tree changes as work
newly introduced by the teammate integration.

Current logic files were checked against imported HEAD: `motion.c/.h`,
`motion_controller.c/.h`, `motor_control.c`, `wheel_encoder.c`,
`motion_estimation.c`, `speed_pid.c`, `s3_drive.c`, `line_navigation.c` and
`line_detection.c` have no local algorithm changes relative to `71193e2`.
`subsystem2.c` has optional timing-observer hooks. Buddy 3 headers/leaf modules
retain representation-only storage changes, verified against upstream behaviour.
The imported feedback/lease additions differ from the original `origin/Buddy2`
snapshot; they are already part of `71193e2`, not a new PID rewrite by Codex.

Safe initial checks:

```sh
cd "/Users/zhaobao/Documents/SIT/SIT-Y2-Tri1/Embedded System/Project/autonomous-robotic-car"
git status --short --branch
git log -n 4 --oneline --decorate
git diff --check
git diff --name-only --diff-filter=U
git stash list
```

Expected at handoff: dirty intended working tree, no unresolved conflict files,
no staged changes, backup stash retained. Inspect changes made after handoff
before assuming this is still true. Do not use `git reset --hard`, `git clean`
or `git checkout --` to make this workspace “clean.”

## 6. Current architecture: do not duplicate the controller

```text
                            pre-run four-letter map
Laptop MQTT -> Buddy 1 command/cache ---------------------+
                                                        |
IR ADC -> Buddy 3 line/navigation -> s3_drive controller <-+
                       ^              |        ^
                       | odometry     |        | source-timestamp feedback
                       +--------------|--------+ and command-ID reports
                                      v
                               Buddy 2 motion -> PWM/motors

s3_integration is the single sampling/control/runtime owner.
vehicle_controller is a taskless facade -> Buddy 1 telemetry/outbox -> MQTT.
```

### Entry point and profiles

`app_program/app_main.c` chooses the startup path:

| Profile | Meaning |
|---|---|
| Default `S3_MOTION=0 IR_BENCH=0 MOTION_SMOKE=0` | Normal subsystem hooks; Buddy 1/2 service available, but Buddy 3 does not start autonomous navigation |
| `S3_MOTION=1` | Buddy 1 + Buddy 2 + single sensor/controller owner and status reporter; all three IR channels enabled; boots disarmed |
| `IR_BENCH=1` | Stationary IR acquisition/reporting only; motor subsystem not initialised; GP26 only unless `IR_CHANNELS=3` |
| `MOTION_SMOKE=1` | Isolated manually armed one-shot motor test; Buddy 2 only, no navigation, Wi-Fi, MQTT, IMU or obstacle task |
| `TEST=1` | On-target self-tests; movement/calibration cases have separate guarded switches; compiling is not executing them |
| `DIAGNOSTICS=1` | Optional low-priority USB observer, not movement control; use appropriate normal/integrated profile |

Build guards reject incompatible smoke/bench/navigation/test combinations.
Pass the complete flags on every build; build-profile changes invalidate stale
objects. Do not flash a file just because it exists or has a recent timestamp.

### Runtime/controller files

- `controller/s3_drive.c/.h`: teammate's portable drive/state policy. The sole
  ordinary motion-request and motion-report consumer.
- `controller/s3_integration.c/.h`: one sampling/controller task serialises ADC,
  navigation context, calibration and control requests; separate status reporter.
- `controller/vehicle_controller.c/.h`: **no task**, no direct motion request,
  no completion consumer. Routes queued START/STOP, freezes/copies mapping and
  mirrors telemetry/faults. Its legacy external navigation submission returns
  `E_NOSPT` so a second producer cannot sneak in.
- `controller/integration_codec.c/.h`: explicit packed shared request/status.

Do not restart the old priority-15 Vehicle Controller task alongside this
runtime. Do not have a UI/network/diagnostic task consume `motion_get_report()`
or `motion_poll()` while the integrated owner is running.

### Agreed navigation behaviour

- Detect a barcode and remember its mapped action; do **not** immediately turn.
- At a persistent junction, request braking, wait for a newer stopped
  acknowledgement, then request the manoeuvre.
- Match completion by the accepted command ID. Stale/incorrect/cancelled/lost
  reports cannot complete a newer manoeuvre.
- Suppress line/barcode control capture during the manoeuvre; faults/freshness
  still matter. Rearm and reacquire a stable line before resuming tracking.
- Unmapped/ambiguous pending codes, unassigned junctions, persistent line loss,
  stale feedback, rejected requests and timeouts cause halt/fault rather than
  blind motion or automatic restart. Read `s3_drive.c` for exact distinctions.
- Current development policies include 5 s pending action, 500 ms brake ack,
  10 s manoeuvre and 1 s reacquisition timeouts; physical validation is open.
- Barcode rearming uses fresh signed centre odometry and reset epoch, including
  20 mm net clearance plus 30 ms stable clear line. These are tuning values, not
  physically demonstrated recognition guarantees.

**Original-line identity is NOT solved.** Two front IR sensors can confirm a
stable line but cannot identify one of several otherwise identical lines. The
user explicitly worried about a course with several possible lines. Route-aware
search/corridor/position/marker constraints must be considered with Buddy 5
integration later. Do not rename “stable line reacquired” to “original line
confirmed” or claim packing solves navigation identity.

## 7. Buddy 1 current behaviour and limitations

Main files:

```text
app_program/subsystem1/subsystem1.c/.h
app_program/subsystem1/command.c
app_program/subsystem1/telemetry_codec.c/.h
app_program/subsystem1/mqtt_bridge.c
app_program/subsystem1/MQTT_PROTOCOL.md
app_program/subsystem1/DATA_REPRESENTATION.md
lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c/.h
lib/libwifi/sysdepend/pico_rp2040/cyw43_utk.c
tools/buddy1_mqtt_codec.py
```

### Message flow

1. Application owners call `buddy1_report_*` with normal units/validated values.
2. Buddy 1 packs explicit binary bytes and updates a protected latest cache.
3. It copies a complete message into a bounded RTOS outbox.
4. The existing kernel-owned radio/lwIP/MQTT task drains messages and publishes.
   Other tasks do not call lwIP/MQTT directly.
5. Laptop code decodes using the agreed protocol, not raw C struct layout.

Protocol **v2**: first byte `0x02`, explicit LSB-first fields, little-endian,
exact lengths and canonical zero padding. Reject wrong versions/codes/lengths.
Zero bytes are normal: never `strlen`/`%s` a binary payload.

Topics remain readable. Motion 4 bytes including version, barcode 2, hump 5,
heading 3, obstacle 6, state 2, faults 2, full snapshot 16, mapping reply 2,
heartbeat/status 2. Consult the protocol file for exact fields rather than
hand-coding them from this summary.

- Outbox: **4 fixed 17-byte application slots** (topic ID + up to 16 payload).
- Incoming commands: **2 fixed 5-byte slots** (kind + up to 4 payload).
- RTOS message-buffer bookkeeping/rounding is additional RAM; 4×17 is not the
  whole allocated queue cost. See implementation/size tests.
- Sends/pops use `TMO_POL`; full outbox returns `E_TMOUT` and rejects the new
  report, but the latest-value cache was updated first.
- Cache mutex access can briefly block; “non-blocking networking” is not a claim
  that every API is wait-free. No task waits for a network delivery.
- Network drains at most 4 messages per round. A publish rejection can drop a
  popped message. No reliable event-history/retransmission mechanism was added.
- An important event can be dropped in a burst. Safety/control must not depend
  on telemetry delivery. Snapshot/fault cache helps recover latest state, not
  every historical event. Periodic reporting is roughly 5 Hz, not every cycle.

### Commands and mapping

Only snapshot and pre-run barcode mapping commands are supported; remote motor
driving commands are intentionally unsupported.

```text
car/cmd/snapshot:     one raw byte 0x02
car/cmd/barcode_map:  4 bytes, v2 plus four 5-bit A-Z letter fields
```

Exactly four distinct uppercase letters correspond to forward/left/right/U-turn.
Buddy 1 default: **A=forward, B=left, C=right, D=U-turn**. This differs from the
standalone Buddy 3 development default. At team START, the owner installs Buddy
1's authoritative mapping into its private 26-entry sensor map.

`vehicle_controller_begin()` now only queues START. On the sampling owner,
readiness is checked and `buddy1_configuration_snapshot_and_close()` atomically
copies/freezes the map before starting. No network task directly edits the
navigation context. Map remains frozen until reboot, including after stop.

Known documentation nuance: an older paragraph in `MQTT_PROTOCOL.md` still says
`vehicle_controller_begin()` closes the map before posting. The current live
facade/runtime code and current controller README supersede that exact ordering:
freezing happens when the owner processes an eligible explicit START.

Laptop helper examples (print/decode only, not publish/move):

```sh
python3 tools/buddy1_mqtt_codec.py decode --topic car/telemetry/motion --hex 023c7602
python3 tools/buddy1_mqtt_codec.py encode-map --forward A --left W --right F --uturn Z
python3 tools/buddy1_mqtt_codec.py encode-snapshot
```

The encoder prints hex for inspection. An MQTT client must send actual bytes
(`bytes.fromhex(...)`), not the ASCII characters `"02"`.

### Shared telemetry faults

| Bit | Meaning |
|---:|---|
| 0 | Motor stall |
| 1 | Encoder fault |
| 2 | IMU fault |
| 3 | Confirmed no-echo failure |
| 4 | Wi-Fi/IP link unavailable |
| 5 | Enabled IR sensor fault(s) |

`buddy1_update_faults(changed, active)` changes only agreed owner bits under the
mutex. The facade updates stall/encoder/IR bits; unknown motion status does not
clear previously known motor faults. Radio owns Wi-Fi. Buddy 4/5 adapters are
not yet integrated. This is a telemetry mirror, not authoritative safety state.
Aggregate `faults_valid` does not certify that every sensor was tested.

Heartbeat is every **5 seconds** on the configured `MQTT_TOPIC` (which may not
literally be `car/status`). Alive means the network callback ran, not that all
tasks/sensors are healthy. Broker-only failure is not necessarily a Wi-Fi fault.
Cache validity does not auto-expire; per-field age policy is still a decision.

### Important OPEN range issue

Buddy 1 wire v2 speed: 8 magnitude bits × 2 mm/s plus direction = **±510 mm/s**.
New Buddy 2 own packed status: 9 magnitude bits × 2 mm/s plus direction =
**±1022 mm/s**. Configured measured top speed is **0.884 m/s**.

The integration does NOT silently widen v2 or clip/wrap high speeds. Above
±510, nonfinite values, busy calibration or invalid status, the facade/legacy
adapter marks motion unavailable. Normal PID inputs remain full precision.

Before relying on high-speed telemetry, agree either an explicitly versioned
wider wire contract and laptop decoder or another justified representation.
Do not claim full speed telemetry is solved by software tests that cover only
the old wire range. Do not change speed targets to hide the incompatibility.

## 8. Buddy 2 current behaviour and why the old motor attempt failed

### Adopted teammate implementation

`origin/Buddy2` at `a29f973` supplied:

- PWM peripheral reset release in `motor_control_init()`.
- New measured geometry/calibration/PID data and saved physical logs.
- PIO quadrature acquisition, replacing the old GPIO edge ISR decoder.
- Minimum-bit commands/status/reports, stop latch, report IDs/queue/loss handling.
- Stop-aware calibration and explicit aborted/invalid result semantics.

`71193e2` additionally includes coherent source-time/epoch feedback and the
opt-in navigation-owner lease needed by the teammate's Buddy 3 controller.

Current configured values in `motion.c`:

```text
wheel diameter:       0.0654 m
track width:          0.116 m
counts/revolution:    2534
maximum wheel speed:  0.884 m/s
M1/M2 inversion:      false / false
encoder inversion:   false / false
default drive:        0.20 m/s
default turn:         0.15 m/s
motion period:        10 ms
```

These are teammate-provided values with teammate evidence; they have not been
revalidated on the user's current car by Codex. Do not tune them based only on
the historical no-movement attempt or a hand-spun speed log.

Main files: `motion.c/.h`, `motion_controller.c/.h`, `motor_control.c/.h`,
`wheel_encoder.c/.h`, `motion_estimation.c/.h`, `speed_pid.c/.h`,
`motion_pack.c/.h`, `motion_lease.h`, `subsystem2.c/.h`.

### Motor/encoder ownership

- M1 = **left wheel**, M2 = **right wheel**, confirmed by the user.
- Motor pins GP8/GP9/GP10/GP11; encoder pairs GP4/GP5 and GP16/GP17, subject to
  checking current `TEAM.md`/source/actual wiring before changing connections.
- Motion task is sole PWM writer. Other tasks submit requests, including stop.
- Only motion task drains PIO FIFOs with `wheel_encoder_sample()`; getters use
  the latest count snapshot. Do not add a second FIFO reader.
- PIO0 SM0/SM1 and instruction memory 0..23 are reserved for encoders.
- Current local SDK radio allocator searches PIO1 first; kernel radio starts
  before application motion. Encoder driver refuses already-running PIO0 SMs.
  Raw-register encoder ownership is not registered in SDK claim bookkeeping;
  preserve allocation with future PIO users and verify concurrency on hardware.
- Count storage is **20-bit modular**, interpreted signed, with wrap-safe deltas.
  Header justifies about ±42.5 m/side against a declared 30 m longest run.
  Treat that as teammate design justification, not Codex-measured course length.
- `wheel_encoder_glitches()` always returns zero with PIO because invalid
  transitions are ignored. Diagnostics print `glitches_available=0`. Zero is
  NOT evidence of clean signals; older IRQ glitch logs are a different backend.

### Motion reports, feedback and stop

- Motion states: IDLE, VELOCITY, DRIVE, TURN, COMPLETE, FAULT.
- Fault codes: none, stall, stale command, encoder, direction, timeout.
- Status is 64 bits; reports 16 bits; commands 32 bits. Read `DATA_FORMAT.md`.
- Report buffer holds 8 entries. Command IDs wrap 255→1. Reports identify the
  command and outcome; loss is explicit and observed by the controller.
- `motion_get_status()` / `motion_get_feedback()` are coherent non-consuming
  views. Feedback has signed centre distance, original source timestamp,
  reset/calibration epoch, validity, state/fault, command ID, braked ack and loss.
- Stopped acknowledgement includes applied brakes plus about 100 ms observed
  stillness, not merely receipt of a stop request.
- `stop()` has priority/latch semantics and wakes the motion task; new commands
  cannot immediately overwrite an unapplied stop.
- Earlier concern “calibration ignores stop” was valid for the old version.
  The imported version checks stop during calibration and marks abort invalid.
  Revalidate on hardware; this is still software, not a universal hardware E-stop.
- Navigation-owner heartbeat/lease is off by default; healthy active controller
  refreshes it. Motion latches stale-command and brakes after 30 ms without it,
  including finite moves. Calibration is excluded while lease is armed.
- Software lease/stop depends on a functioning kernel/motion task and cannot
  substitute for an accessible physical motor power switch.

### Legacy reporting adapter

`motion_snapshot_adapter.c` creates the old 4-byte compact snapshot from the new
coherent status. It is for diagnostics/compatibility, not PID control. It does
not consume completion, issue motion or change algorithms. Out-of-range speeds
become invalid as described in Buddy 1.

## 9. Buddy 3 current calibration/start gates

- Barcode GP26/ADC0; front left GP27/ADC1; front right GP28/ADC2.
- This role assignment differs from Week 6 L/C/R proposal. User confirmed two
  line sensors plus one barcode sensor. Verify the assembled wiring/mounting;
  GP28's board VBAT jumper configuration matters (see sensor/board docs).
- Standalone IR bench defaults to GP26 only; “left/right DISABLED” there is
  deliberate, not proof that hardware is missing/broken.
- Provisional white/black defaults 150/280 are NOT actual-track calibration and
  do not authorise navigation START, even if bench labels look sensible.
- Each sensor needs independently collected/accepted white and black data at
  its final height/mounting. Preserve raw 12-bit ADC precision before processing;
  do not reduce raw calibration data to one black/white bit.
- `s3_navigation_process()` now has **four arguments**: context, frame,
  odometry pointer (or NULL diagnostic case), output. Old three-argument callers
  and old output packets are incompatible.
- Accepted calibrations, fresh valid all-channel frames, fresh valid odometry,
  healthy fault/reset state and braked/IDLE motion are start gates.
- New state tracks lifecycle/readiness, accepted-cal mask, persistent faults,
  diagnostic steering preview, stop request, rearm and exact capture distances.
- Calibration is RAM-only; after reboot collect/apply again. No persistent
  storage or automatic calibration bypass has been added.

### Operator API is implemented; operator UI is NOT yet connected

`s3_integration_submit()` supports:

```text
S3_CONTROL_WHITE
S3_CONTROL_BLACK
S3_CONTROL_APPLY_CALIBRATION
S3_CONTROL_MAPPING
S3_CONTROL_START
S3_CONTROL_STOP
S3_CONTROL_RESET
```

There is one pending command slot. STOP takes precedence and cancels queued
work. True means queued, not successful execution. Poll copied status for a new
`command_sequence` and `command_ok`; inspect `collecting` and calibration
summary/count/min/max. Do not overwrite pending dependent commands.

While disarmed, hold real white under a selected sensor, submit WHITE and wait
for the one-second collection to complete. Repeat BLACK, APPLY_CALIBRATION and
check acceptance. Repeat for all three. Exact procedure/API is in
`subsystem3/MOTION_INTEGRATION.md`.

The current USB console discards input characters. Typing `start`, a C enum or
JSON into `screen` does nothing. There is no normal-firmware button/calibration
input adapter yet. Adding one is a separate scoped implementation choice with
the user; it must route through the owner, retain calibration gates and avoid
remote motor-control scope expansion. The immediate motor smoke test does not
need this UI and should come first.

## 10. Minimum-bit integration and RAM/CPU reasoning

Packing is an agreed data representation, not a set of GPIO pin states. Each
module can have private state; only genuinely shared interfaces need a shared
format. Bit allocation in one word does not mean all tasks globally “share 8
bits.” Protect shared read-modify-write updates; a C bit field alone is not
atomic or a concurrency solution.

Current explicit copied formats:

| Value | Stored size | Notes |
|---|---:|---|
| Full Buddy 3 navigation result | 479 bits / 60 bytes | Includes all new API fields, exact steering floats and binary64 capture data; one padding bit |
| Runtime status | 122 bytes | 60 navigation + 22 feedback + 40 metadata |
| Pending runtime control | 83 bits / 11 bytes | 3-bit kind, 2-bit sensor, 26×3-bit map; five padding bits |
| Stationary raw-frame snapshot | 266 bits / 34 bytes | Per-channel time, raw 12-bit count, signed error and enabled/valid flags |
| Saved barcode | 38 bits / 5 bytes | Letter/orientation/time; two padding bits |
| Buddy 2 own status / report / command | 8 / 2 / 4 bytes | Teammate's agreed module format |
| Legacy motion reporting snapshot | 4 bytes | Narrower speed range; invalidated when necessary |

The earlier local navigation packet was 234 bits/30 bytes. Keeping that size
after the teammate API update would silently drop readiness/odometry information,
so it was extended. Do not “optimise back” to 30 bytes without considering the
new fields. Local internal letter coding differs from MQTT letter coding;
codecs/facade intentionally translate decoded values at boundaries.

Private retained small masks/booleans/configuration have representation changes;
decoded inputs and working arithmetic remain wide enough to validate ranges and
preserve behaviour. Float/double retention is not proof of a universal minimum;
future quantisation needs agreed error/range analysis and tests, not truncation.
Do not claim every Tribunal requirement is closed simply because these codecs
exist or enum counts fit in a few bits.

Application task stacks are 2048 bytes. Current priorities (lower number higher):
USB 3, kernel radio/lwIP/MQTT 4, motion 14, sampling/controller 16, S3 reporter 17,
Buddy 1 command 22, optional diagnostic initial-task reporter 23. Core affinity
is core 0 for these current services. No priority/rate retuning was silently done.

Network priority still outranks motion. Compact packets do not prove motion
deadlines under publishing/reconnection load. Measure jitter/execution, stack/
heap usage and resource contention on the board. Lock ownership helps races;
priority alone does not bound all blocking or computation.

Current sampling and motion targets are both **10 ms**. Week 6 proposed 5 ms
Line+Barcode. User discussed whether faster/Nyquist helps; no automatic 5 ms
change was accepted. Recognition depends on speed/bar width/samples/noise/cost;
faster is not automatically better. If changing IR to 5 ms later, preserve
control logic and telemetry cadence (currently 20 cycles ≈5 Hz; 40 at 5 ms).

New host short-enum structure observations: decoded navigation 128 vs packed60,
raw frame52 vs34, barcode8 vs5, navigation context488, calibration collector18.
These are host structure sizes, not total target RAM savings.

Linker figures from final verification:

| Image | text | data | BSS |
|---|---:|---:|---:|
| Combined offline S3_MOTION + diagnostics | 117064 | 92 | 24092 |
| Combined network compile-only | 384928 | 152 | 84024 |
| IR bench, 1 or 3 channels | 115496 | 92 | 23900 |
| Offline self-test | 124352 | 92 | 22460 |
| Isolated motion smoke | 114568 | 92 | 22460 |

Linker data/BSS is not peak total RAM. Kernel allocations, task stacks, network
runtime use, heap and stack high-water still need measurement. Codec processing
and console reporting also cost CPU time. Historical RAM-savings reports in
`testing/subsystem2/EVIDENCE.md`, `subsystem3/EVIDENCE.md`, `BUILD_AND_RAM.md` and
`DATA_REPRESENTATION.md` are labelled historical/superseded where applicable.

## 11. What actually happened on the robot so far

This section is **user-reported observations and pasted logs**, not independent
post-integration measurements by Codex.

### Physical setup history

- Initially the user had a green Pico W mounted on a separate purple carrier,
  plus a robot with a purple board/socket but no green Pico installed.
- Prior photo discussion identified the separate carrier as Maker Pi Pico base
  and the car board as Robo Pico. Verify current hardware with photos if unsure.
- The user safely removed the green Pico W from the separate carrier and seated
  it on the robot's socket. Do not tell them to repeat removal or use force.
- **Power off/disconnect before inserting/removing boards or changing wiring.**
  Never assume orientation/alignment from colour alone; use actual silkscreen,
  photos and board docs.
- The laptop data cable goes to the **micro-USB socket on the green Pico W**.
  They also have a supplied power bank for the assembled car's power path.
- USB-only/indicator behaviour was confusing; motor supply diagnosis was
  explicitly deferred at the user's request because teammates had a working
  power arrangement. Do not claim the full power path is proved or silently
  rewire it. If no movement persists after the new test, revisit motor power/
  switch/driver enable safely. The physical power switch must remain reachable.
- The user says the manual M1/M2 hardware motor buttons worked and confirmed
  M1 left/M2 right. Do not mix those buttons with the software smoke test.

The current physical power/cable state is not known just from this file. Ask
before issuing a motion instruction. Do not assume both wheels are still raised.

### Flashing and USB did eventually work

- Holding BOOTSEL while connecting green Pico micro-USB showed `RPI-RP2` in
  Finder. This is bootloader storage, not the normal serial console.
- Finder copy initially hung/asked Touch ID; terminal `cp -X` subsequently
  worked. Do not infer a defective Pico from the earlier Finder UI alone.
- After successful UF2 copy the Pico reboots, the drive normally disappears and
  the firmware's USB serial port appears. No need to eject it as normal storage.
- Last successful console device: `/dev/cu.usbmodemMTK3PICO1` at 115200 baud.
  Re-enumerate it after replugging; do not assume that exact name always exists.
- Several early `screen is terminating` / no-device errors occurred before the
  serial device was available. Later logs prove serial was working.
- Shell history sometimes still showed an earlier IR UF2 copy command even
  when current logs were `[DIAG]` or `[SMOKE]`. Use a new exact command/source
  path and current log banner/hash; don't deduce installed firmware from old
  scrollback alone.

The user needed detailed `screen` exit guidance: **Control**+A, release, then
lowercase `k`, then `y` if prompted. It is a sequence, not all keys held at once,
not Command+A, and it does not open a new window. “No other window” was a screen
key-command message, not a new firmware error. Close the existing serial viewer
before opening a logger/another viewer; do not kill unrelated processes.

### Stationary IR observation

Original `mtk3pico_smp1_usb_cdc_ir_bench.uf2` showed e.g.:

```text
[S3 BENCH] enabled=1 cal=PROVISIONAL ... target=10ms
barcode GP26 valid=1 raw≈3380/4095 ... BLACK white=150 black=280
left GP27 DISABLED/unavailable
right GP28 DISABLED/unavailable
```

The user said their black pants were under the sensor; high black classification
was plausible, not calibrated track evidence. Earlier raw values near40–50 were
also observed in a different setup. No actual white/black calibration or moving
barcode/line-following success was established in this conversation.

### Encoder/diagnostic observations before the new PIO firmware

- Initially counts/speeds were zero with the robot stopped; this was expected,
  not proof of motor requests being sent.
- User then operated buttons/wheels; screenshots showed left and right counts
  changing independently, sometimes very high positive/negative speed estimates
  and increasing glitch counts in the **old GPIO IRQ backend**.
- Those observations do not establish calibrated scaling, correct forward signs,
  clean signals or new PIO behaviour. Invalid compact speed could simply mean
  outside ±510 mm/s; don't decode it as “wheel stopped.”
- Old timing logs had strange sample gaps (IR ~233 us while target10ms, earlier
  maxima around141580 us). The imported `monotonic_clock.h` explicitly starts
  hardware TIMER/watchdog tick correctly; new timing must be measured before
  claiming the anomaly is fixed. Host-arrival times are not MCU sample times.

### Most recent attempted movement — BEFORE integration

The isolated smoke image ran and accepted GP20:

```text
[SMOKE] READY: GP20 starts once; GP21 cancels.
[SMOKE] START accepted: 3 second countdown. GP21 cancels.
[SMOKE] Buddy 2 fault; left=1 right=-1 fault=1
[SMOKE] FINISHED: no automatic retry; power off before repositioning.
```

User explicitly said **wheels did not move and were raised off the surface**.
Fault1 is STALL. Codex found the old motor code did not release PWM from reset
in this kernel configuration. Before applying a separate fix, the user asked to
integrate the teammate's GitHub updates. The incoming teammate code already has
the PWM reset release; we adopted it instead of adding a competing fix.

Do NOT say the no-movement fault is physically resolved. The rebuilt smoke image
has not been flashed/tried after this import. If the new attempt still stalls,
collect exact logs/physical observations and check the power/driver/PWM path
before tuning PID, changing encoder width or modifying control algorithms.

## 12. Exact next motor-test procedure: guide, do not run automatically

Read `testing/diagnostics/MOTION_SMOKE.md` and actual smoke source first.

Current rebuilt firmware:

```text
/Users/zhaobao/Documents/SIT/SIT-Y2-Tri1/Embedded System/Project/autonomous-robotic-car/build_make/mtk3pico_smp1_usb_cdc_motion_smoke.uf2
```

Validated RP2040 UF2, 229376 bytes at handoff, SHA-256:

```text
3af00dfb91def3e299d5b205f2f55f77c3473099e8e4b41aaeb77bae84913dd5
```

A rebuild/source/toolchain change may change the hash; verify the current file.
No file is copied to the Pico by the commands below until the user deliberately
performs the copy. Building/flashing overwrites the selected firmware on that
Pico, so retain a known-good teammate UF2 separately if available.

### Test semantics

- Boots without motion and waits for an explicit fresh/debounced start.
- Release **both** GP20 and GP21 first; a held-at-boot start cannot arm.
- GP20 = START; GP21 = software STOP/cancel. M1/M2 are not these buttons.
- GP20 gives a 3-second countdown. GP21 can cancel before/during countdown.
- Requests forward **0.10 m at 0.05 m/s**, with **3-second software time limit**.
- No console printing in the active movement loop. Stop, timeout, completion,
  fault/rejection or unexpected state ends the attempt; no automatic retry/reset.
- One attempt per boot. Power off before repositioning. Do not rapidly repeat
  starts to work around a fault or bypass calibration checks.
- These are requests, not a guarantee of actual distance/speed. First test
  raised wheels on a stable support, not the floor/table edge. Keep hair/fingers/
  leads away and the physical power switch accessible.
- GP21/time limit require software/kernel/motion task to run; not an independent
  emergency cutoff. Do not touch manual motor buttons while this test runs.

### Build if needed (no flashing)

```sh
cd "/Users/zhaobao/Documents/SIT/SIT-Y2-Tri1/Embedded System/Project/autonomous-robotic-car"
export PICO_SDK_PATH="/Users/zhaobao/pico/pico-sdk"
make -C build_make CONSOLE=usb_cdc WIFI=none NET=none MQTT=0 TEST=0 IR_BENCH=0 S3_MOTION=0 DIAGNOSTICS=0 MOTION_SMOKE=1 -j8
```

### Flash only when the user is physically ready

Have the user first close the serial viewer, secure raised wheels, switch off
motor power and enter BOOTSEL with the green Pico's micro-USB data cable. Hold
BOOTSEL while connecting USB, then release when `RPI-RP2` appears. Confirm this
is the intended Pico and the mount exists; do not blindly copy to a missing
mount path or to the whole build folder.

```sh
ls -ld "/Volumes/RPI-RP2"
ls -l "/Users/zhaobao/Documents/SIT/SIT-Y2-Tri1/Embedded System/Project/autonomous-robotic-car/build_make/mtk3pico_smp1_usb_cdc_motion_smoke.uf2"
cp -X "/Users/zhaobao/Documents/SIT/SIT-Y2-Tri1/Embedded System/Project/autonomous-robotic-car/build_make/mtk3pico_smp1_usb_cdc_motion_smoke.uf2" "/Volumes/RPI-RP2/"
```

After reboot verify port, then use **one** viewer/logger:

```sh
ls /dev/cu.usbmodem*
screen /dev/cu.usbmodemMTK3PICO1 115200
```

The glob may report no matches until normal firmware enumerates. Verify actual
port rather than repeatedly opening a non-existent old name. Use the team's
verified motor power arrangement/physical switch only once the car is secured;
do not change wiring or connect arbitrary supply pins from these instructions.
Wait for READY, then guide GP20 once. Ask specifically whether each wheel moved
in physical forward direction and both stopped, and request the full final log.

Do not escalate to floor distance tests until raised-wheel direction and stop
are confirmed. For floor testing use clear floor area, never a tabletop; record
measured travel rather than claiming an encoder target proves 10 cm.

## 13. Logging and how to send evidence back

Files:

```text
tools/capture_robot_log.py
tools/requirements-robot-log.txt
testing/diagnostics/SESSION_NOTES_TEMPLATE.md
testing/diagnostics/test_capture_robot_log.py
build/robot-tools/          # prepared local Python virtual environment
build/robot-logs/           # ignored local recordings, if created
```

The laptop logger reads serial bytes, never sends robot commands. It saves
`console.log`, `metadata.json`, optional `hostarrival.csv`, refuses to overwrite
an existing recording directory and labels replay as `REPLAY_NOT_HARDWARE`.
Opening a serial port can still affect USB control lines; secure the robot first.
Do not have `screen` hold the same port while the logger is trying to open it.

List ports:

```sh
source build/robot-tools/bin/activate
python tools/capture_robot_log.py --list-ports
```

Example live capture — choose a **new** output name and confirm firmware/current
port before opening; do not auto-run this as part of reading the handoff:

```sh
python tools/capture_robot_log.py --port /dev/cu.usbmodemMTK3PICO1 --baud 115200 --output build/robot-logs/2026-10-25-smoke-after-merge-01 --firmware 71193e2-local-motion-smoke --test raised-wheel-first-check --line-csv
```

Record: branch/commit plus local changes, UF2 hash/profile, actual power/wiring,
which wheel was moved or commanded, physical direction, counts before/after,
actual wheel turns/time/distance, fault/stopped response, calibration parameters,
surface/lighting/sensor height and video if useful. For encoder settings, collect
known physical wheel turns and measured travel, not counts alone. For CPU/RAM,
collect MCU timing and measured heap/stack metrics, not laptop line-arrival times.

## 14. Software verification completed after integration

No physical robot/serial/network connection was opened by the integration tests.
No UF2 was flashed. Saved teammate hardware logs retain their own provenance.

Commands from the active repo:

```sh
make -C testing/subsystem1 test sanitize
make -C testing/subsystem2 test adapter regression sizes sanitize
make -C testing/subsystem3 test codec drive feedback regression sizes sanitize
make -C testing/integration -f controller_test.mk test sanitize
make -C testing/diagnostics -f firmware_test.mk test smoke disabled
python3 -m unittest discover -s testing/diagnostics -p test_capture_robot_log.py
```

Observed PASS results include:

- Buddy 1 on-target-style C checks under host stubs, independent Python decoder,
  full wire speed range, all64 fault masks, eight concurrent producers, bounded
  FIFO, actual MQTT callbacks/bridge and frozen mapping copy.
- New Buddy 2 pack boundary tests at every integer mm/s across ±1022, command/
  report IDs, rejected invalid/out-of-range/nonfinite inputs and narrow-legacy
  telemetry invalidation.
- Teammate's real production feedback tests: original timestamps, epochs,
  modular wrap, report loss/queue overflow, command-ID wrap, cancellation,
  calibration abort/exclusion and navigation watchdog.
- Real Buddy 2 controller/PID with Buddy 3 drive bridge under fake hardware.
- Buddy 3 core/navigation/reflectance/calibration/line/Code39 tests, forward/
  reversed barcode fixtures, error/timing/rearming/readiness cases.
- Regression against **71193e2**: 58,915 observed field-wise navigation/barcode
  trace rows, all65,536 ADC input values and calibration maxima; current and
  imported traces match. Actual stationary wrapper diagnostic output also matches.
- New 60-byte navigation, 122-byte shared status and 11-byte control roundtrip/
  range/padding/hostile-packet tests; exact float/double/capture information.
- Actual runtime test of STOP replacing a pending request, full-slot rejection,
  no status before publication, rejected uncalibrated start, collection, signed
  ADC errors and no ADC/motor/facade callbacks while snapshot mutex is held.
- Facade ownership, mapping freeze/validation, fault-bit preservation, unknown
  motion-status handling, cache invalidation and state/barcode reporting.
- Diagnostic timing/wrap/saturation/publication/startup/failure and non-consuming
  observations; one-shot smoke start/cancel/timeout/fault/no-autoretry tests.
- ASan/UBSan runs for the software suites above; logger 25 unit tests pass.

These do not prove ISR/PIO electrical behaviour, real scheduling, physical
braking latency, actual track recognition, autonomous route correctness or PID
accuracy. Synthetic 5/10ms traces are not a measured 5ms scheduling guarantee.

### Current versus historical test harnesses

The pre-pull Buddy 2 host harness assumes the old GPIO IRQ layout and is kept as
historical `legacy-test`. Default `make test` now runs current pack/feedback/drive
checks; `adapter` runs current facade/runtime tests. `regression` exports
`71193e2` without switching the checkout. Do not use old `816d7f0`/`477230c`
comparisons to claim the newer PIO/odometry implementation is unchanged from
the ancient one. Compare representation work to the appropriate current baseline.

Current C-test sources include `testing/integration/team_adapter_test.c`,
`codec_test.c`, `runtime_test.c`, `testing/subsystem2/pack_test.c`,
`testing/subsystem3/test_drive.c`, `test_motion_feedback.c`, `test_navigation.c`,
`test_codec.c`, `host_trace.c`, `bench_trace.c`.
Old `testing/integration/controller_test.c` describes the superseded controller;
the current makefile selects `team_adapter_test.c`, not that old task harness.

Temporary captured outputs (may disappear after cleanup/reboot):

```text
/tmp/integrate-final-buddy1.log
/tmp/integrate-final-buddy2.log
/tmp/integrate-final-buddy2-regression.log
/tmp/integrate-final-buddy3.log
/tmp/integrate-bench-regression.log
/tmp/integrate-final-adapter.log
/tmp/integrate-final-diag.log
/tmp/integrate-final-logger.log
/tmp/integrate-final-combined.log
/tmp/integrate-final-network.log
/tmp/integrate-final-bench1.log
/tmp/integrate-final-bench3.log
/tmp/integrate-final-selftest.log
/tmp/integrate-final-smoke.log
```

Reproduce if these no longer exist. Persistent summary:
`docs/INTEGRATION_2026-10-25.md` and `testing/integration/TEST_LOG.md`.

## 15. Build environment and output files

Working macOS setup:

```text
shell: zsh
Arm compiler: arm-none-eabi-gcc (found at /opt/homebrew/bin/arm-none-eabi-gcc)
PICO_SDK_PATH: /Users/zhaobao/pico/pico-sdk
make-based BSP build: build_make/Makefile + pico_rp2040.mk
Python logger venv: build/robot-tools
```

Do not delete `build_make/`: it contains tracked source Makefiles, not merely
disposable CMake output. Do not change runtime/SDK/kernel architecture to get a
standalone example building. Use the existing flags and profile machinery.

Built successfully after integration:

```text
build_make/mtk3pico_smp1_usb_cdc_motion_smoke.uf2
build_make/mtk3pico_smp1_usb_cdc_s3_motion_diag.uf2
build_make/mtk3pico_smp1_usb_cdc_ir_bench.uf2
build_make/mtk3pico_smp1_usb_cdc_ir_bench_3sensor.uf2
build_make/mtk3pico_smp1_usb_cdc_test.uf2
build_make/mtk3pico_smp1_usb_cdc_wifi_lwip_mqtt_s3_motion_diag_COMPILE_ONLY.uf2
```

The first four were checked for RP2040 UF2 block headers/family/sequence.
The final build objects/profile were left in **offline isolated smoke** mode.
Generated binaries are ignored; the source changes remain uncommitted.

Offline combined/disarmed diagnostic build:

```sh
make -C build_make CONSOLE=usb_cdc WIFI=none NET=none MQTT=0 TEST=0 IR_BENCH=0 S3_MOTION=1 DIAGNOSTICS=1 MOTION_SMOKE=0 -j8
```

Stationary sensor build, one channel (use `IR_CHANNELS=3` for all three only
when physically appropriate):

```sh
make -C build_make CONSOLE=usb_cdc WIFI=none NET=none MQTT=0 TEST=0 IR_BENCH=1 IR_CHANNELS=1 S3_MOTION=0 DIAGNOSTICS=0 MOTION_SMOKE=0 -j8
```

### Network compile-only warning

No real ignored `config/wifi_secrets.h` was present at handoff. The network
compile initially failed for that missing header, then passed using the public
`testing/subsystem1/fakes/wifi_secrets.h` via `CPATH`, with dummy `host-test`,
`unused`, loopback broker settings. No real credentials were read or created.

That ELF/map/UF2 were deliberately renamed `_COMPILE_ONLY`. **Do not flash them
and do not tell the user their Wi-Fi is configured.** Build success is not a
broker connection test. The final subsequent offline build invalidated those
network objects via profile changes.

For real networking later, the user must configure the local ignored secrets
file from the example without exposing passwords in logs/chat/commits, then
rebuild the full network profile with real settings. Ensure dummy-config objects
are invalidated if a future session changes only include paths/config location.
Do not paste the real secrets contents into a handoff or test evidence.

## 16. Remaining work and recommended order

### A. Next: actual raised-wheel motor result

Confirm current support/power/cables and explicit readiness, use the rebuilt
isolated smoke firmware, collect full log/video/physical wheel directions and
stopping. If it fails, isolate motor supply/enable/PWM/wiring and observed
encoder behaviour; do not immediately change PID or minimum-bit widths.

### B. Independently validate Buddy 2 and Buddy 3

Buddy 2: actual encoder signs/counts per known turn, calibrated distance/speed,
both turns/reverse, stop including calibration, known-length floor moves, PID
and accuracy evidence. Saved teammate results are useful but confirm current
hardware matches them.

Buddy 3: each actual sensor/wiring, mounted-height white/black calibration,
two-sensor line/junction detection, moving barcode recognition at controlled
speed, gap/timing/rearming and fault/invalid data. Do not equate a black label
from pants with barcode decoding.

### C. Connect explicit operator calibration/start to the integrated owner

Discuss minimal UI choice with the user; preserve no-motion-on-boot, STOP
precedence, accepted-calibration gates, sole ownership and frozen pre-run map.
Do not add remote driving or bypass readiness for convenience.

### D. Validate Buddies 1–3 together

Show line following, barcode remembered until junction, stop-before-manoeuvre,
matching completion, reacquisition, binary telemetry/snapshot/mapping/heartbeat,
safe behaviour under network loss and actual timing/resource measurements.
Resolve the high-speed telemetry range mismatch explicitly/version it.

### E. Close tribunal/deliverable evidence gaps

Update representation justifications with agreed ranges/resolution, actual
message/RAM results, pack/unpack/boundary tests, heap/stack/timing measurements,
physical videos/logs, PID tuning and motion accuracy, moving barcode/navigation
evidence. Keep unproven items open/Amber. Do not state “all Tribunal requirements
met” merely because builds and unit tests pass.

### F. Later: Buddy 4, then Buddy 5

Validate each independently, agree their ranges/units/validity/fault bits,
integrate through the same controller owner. With Buddy 5 address route-correct
original-line recovery, obstacle scan stop acknowledgement, timing and safe
fallback. Do not expand into these now without the user's request.

## 17. Suggested first response to the user after reading this

Keep it short and concrete. For example:

> I have the handoff. We are on the integrated Buddies 1–3 branch, software
> checks pass, and the new motor-test image includes your teammate's PWM fix.
> It has not been tried on the car yet. Are both wheels still securely raised,
> and what is currently connected to the Pico and the car's power input?

Then guide one step at a time. Do not re-ask the user to explain the whole
project, rebuild the old design or repeat already completed integration work.
Recheck facts that may have changed, and use the current source as authority
when a historical assistant explanation/document contradicts it.
