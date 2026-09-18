# Robot Integration Contract - Draft 1

This document defines the boundary between the five buddy subsystems and the
central mission controller. It is deliberately a draft: the team must confirm
the open decisions near the end before hardware integration.

The central controller decides **what behaviour is active**. Each subsystem
still owns **how its behaviour is implemented**. For example, the controller
may request a right turn, but Buddy 2 owns the encoder feedback, PID and PWM
needed to perform it.

## 1. Ownership

| Owner | Owns | Does not own |
|---|---|---|
| Buddy 1 - communications | WiFi, TCP/IP, MQTT, JSON telemetry, command validation and acknowledgements | Mission decisions, motor control or sensor processing |
| Shared Vehicle Controller | Mission state, behaviour arbitration, pending barcode action and safety transitions | Sensor drivers, PID or MQTT callbacks |
| Buddy 2 | Motors, PWM, encoders, speed/distance control and completion reports | Choosing the mission behaviour |
| Buddy 3 | IR sampling, line position, junction detection, barcode decoding and navigation-command generation using the accepted shared map | Driving PWM directly |
| Buddy 4 | IMU sampling, heading, tilt, hump measurement and impact detection | Choosing the overall mission state |
| Buddy 5 | Ultrasonic/servo scanning, obstacle profiling and avoidance-plan generation | Direct ownership of motor PWM |

The written brief presents the Vehicle Controller separately and does not
assign it to Buddy 1. The controller and shared vocabulary therefore live in
`include/controller/`, `src/controller/`, `include/shared/` and `src/shared/`; the team must agree their interfaces and record
who contributes each change. A network delay must never delay navigation.

## 2. Mission states

| State | Meaning | Active subsystem behaviour |
|---|---|---|
| `BOOT` | Drivers and tasks are starting | Motors stopped |
| `LINE_FOLLOW` | Normal course navigation | Buddy 3 guides Buddy 2 |
| `AT_JUNCTION` | A stored barcode action is being executed | Buddy 2 performs the requested turn |
| `HUMP` | The car is crossing and measuring a hump | Buddy 3 still follows the line; Buddy 2 slows; Buddy 4 measures |
| `OBSTACLE_STOPPING` | An obstacle was detected and braking is in progress | Buddy 2 stops and confirms zero motion |
| `OBSTACLE_SCAN` | The car is stopped while an obstacle is profiled | Buddy 5 scans using servo and ultrasonic sensor |
| `BYPASS` | The selected avoidance plan is being executed | Buddy 2 moves; Buddy 4 checks heading; Buddy 5 checks clearance |
| `LINE_SEARCH` | The original line is being recovered | Buddy 2 searches; Buddy 3 detects reacquisition |
| `STOPPED` | A planned or safety stop is active | Motors stopped |
| `FAULT` | An impact, timeout or unsafe condition occurred | Motors stopped; operator intervention required |

A decoded barcode is not a mission state. The robot continues line following,
stores the decoded action, and executes it only when the junction is detected.

```text
BOOT --START--> LINE_FOLLOW
LINE_FOLLOW --barcode--> LINE_FOLLOW (store the action)
LINE_FOLLOW --junction--> AT_JUNCTION --turn done--> LINE_FOLLOW
LINE_FOLLOW --hump--> HUMP --hump done--> LINE_FOLLOW
moving state --obstacle--> OBSTACLE_STOPPING --motor stopped--> OBSTACLE_SCAN
OBSTACLE_SCAN --plan--> BYPASS
BYPASS --done--> LINE_SEARCH --line found--> LINE_FOLLOW
line-follow/hump/junction --line lost--> LINE_SEARCH
any moving state --safety stop--> STOPPED
any state --impact--> FAULT
timed manoeuvre --matching timeout--> FAULT
```

The detailed transition rules in `src/controller/mission_controller.c` are the executable
version of this overview; the diagram intentionally omits edge-case recovery.

The mission state is separate from the run lifecycle in `include/shared/robot_config.h`:
`CONFIGURING -> ARMED -> AUTONOMOUS -> FINISHED`. Barcode mappings may only be
replaced while `CONFIGURING`; arming freezes the accepted table for the run.

## 3. Events into the mission controller

| Event | Source | Payload | Kind |
|---|---|---|---|
| `START` | System startup | None | One-off event |
| `BARCODE_DECODED` | Buddy 3 navigation module, using the accepted shared mapping | Letter, navigation action, reverse-scan flag | One-off event; must not be lost |
| `JUNCTION_DETECTED` | Buddy 3 | None | One-off event; must not be lost |
| `TURN_COMPLETED` | Buddy 2 | None | One-off event; must not be lost |
| `HUMP_DETECTED` | Buddy 4 | None | One-off event |
| `HUMP_COMPLETED` | Buddy 4 | Peak height in millimetres for telemetry; state machine needs only completion | One-off event |
| `OBSTACLE_DETECTED` | Buddy 5 | None | One-off event; higher priority than normal navigation |
| `MOTION_STOPPED` | Buddy 2 | None | Handshake proving the car is stationary before Buddy 5 scans |
| `OBSTACLE_PROFILE_READY` | Buddy 5 | Avoidance plan and profile data | One-off event; must not be lost |
| `BYPASS_COMPLETED` | Buddy 2/Buddy 5 agreement | None | One-off event |
| `LINE_LOST` | Buddy 3 | None | One-off event |
| `LINE_REACQUIRED` | Buddy 3 | None | One-off event |
| `STOP_REQUESTED` | Physical safety input or system safety logic | None | Safety event; highest priority |
| `RESUME_REQUESTED` | Local system/start logic, if the team permits resume | None | Operator event |
| `IMPACT_DETECTED` | Buddy 4 | None | Safety event; highest priority |
| `TIMEOUT` | RTOS timer/controller adapter | State that armed the timer | Safety event while a manoeuvre is incomplete; a stale timer for an old state is ignored |

Periodic sensor sampling remains inside each buddy's task. The mission
controller receives significant events and may inspect the latest processed
values; it does not process every ADC, encoder or IMU sample.

## 4. Decisions produced by the controller

Every processed event produces a `mission_decision_t`. It contains:

- the previous and current mission states;
- whether the state changed;
- at most one motion command for Buddy 2;
- an optional navigation-action parameter for a turn;
- an optional avoidance-plan parameter for a bypass; and
- a flag requesting Buddy 5 to start obstacle scanning.

Motion commands replace the previous Buddy 2 behaviour. For example,
`START_LINE_SEARCH` cancels a partly completed turn; it is not an additional
movement layered on top of that turn. Buddy 2 must acknowledge a requested
obstacle stop with `MOTION_STOPPED` before the scan flag can be produced.

The controller never calls GPIO, PWM, ADC, I2C, MQTT or RTOS APIs. A later
integration adapter converts its decisions into calls or RTOS messages for the
appropriate buddy.

## 5. Safety and arbitration order

When more than one condition exists, use this order:

1. Impact/fault and safety stop.
2. Obstacle detection and collision avoidance.
3. Hump handling.
4. Junction navigation.
5. Line recovery.
6. Normal line following.
7. Telemetry and logging.

STOP and impact events must not wait behind ordinary telemetry in the same
queue. The RTOS adapter must provide a safety-priority path.

## 6. Interfaces and units

| Data | Unit/convention |
|---|---|
| Distance and obstacle dimensions | Integer millimetres |
| Wheel speed | Signed integer millimetres per second |
| Heading | `0..359` degrees |
| Pitch | Signed deci-degrees; `-125` means `-12.5` degrees |
| Line position | Normalised `-100..100`; negative means line is left |
| Barcode | Capital `A..Z`; decoder also reports whether scanning was reversed |
| Avoidance bearing | Degrees; negative means left |

Internal hardware interfaces are owned by each buddy: IR uses ADC, encoders use
GPIO interrupts, motors and servo use PWM, IMU uses I2C, and ultrasonic ranging
uses GPIO plus a timer. MQTT is only for robot-to-laptop communication; buddies
on the same Pico exchange typed events and decisions through memory/RTOS
mechanisms.

## 7. RTOS mapping - not implemented yet

The pure controller will later run inside a mission task. Buddy tasks post
events to it through a kernel queue, mailbox or equivalent mechanism supplied
by the lecturer's Pico μT-Kernel port. Decisions are then forwarded to the
relevant subsystem task.

The exact μT-Kernel APIs, task priorities, stack sizes and synchronization
objects must be chosen only after the team has the starter port.

## 8. Open decisions for the team

1. **Resolved in the draft implementation:** Buddy 1 parses and transports the
   MQTT payload, but `src/shared/robot_config.c` owns the accepted map and locks it
   when the robot is armed. Buddy 3/navigation uses that shared configuration;
   it never calls `net.c` to interpret a barcode.
2. Who confirms `BYPASS_COMPLETED`: Buddy 2, Buddy 5, or both through a small
   handshake?
3. On RESUME after an interrupted turn or bypass, should the car search for the
   line, restart the manoeuvre, or remain stopped? Draft 1 uses line search.
4. Is an unexpected junction with no pending barcode a fault or a straight-on
   fallback? Draft 1 treats it as a fault because the route is unknown.
5. What are the control-loop rates and the much lower telemetry publish rates
   for Buddies 2, 3 and 4?
6. What timeout applies to turns, obstacle scans, bypasses and line searches?
7. Which events require a reliable RTOS queue and MQTT QoS 1 reporting?
8. **Should `LINE_LOST` be able to cancel a manoeuvre that is already in
   progress?** Draft 1 says yes: the diagram in section 2 routes line loss
   from `AT_JUNCTION` and `HUMP` straight to `LINE_SEARCH`.

   The physical problem is that while the car pivots through a junction its
   line sensors *will* leave the line, and while one wheel is on a hump they
   may. If Buddy 3 reports either as `LINE_LOST`, the turn or hump is
   abandoned midway and its own completion event is then discarded — so no
   turn could ever finish.

   Two ways to settle it, and the team must pick one:
   - Buddy 3 suppresses `LINE_LOST` while the controller is in `AT_JUNCTION`
     or `HUMP` (the knowledge lives with the sensor owner); or
   - the controller ignores `LINE_LOST` in those states (the knowledge lives
     with the arbiter), which mirrors how an early `LINE_REACQUIRED` is
     already prevented from cancelling an unfinished `BYPASS`.

   Current behaviour is pinned by `test_line_lost_during_a_manoeuvre()` in
   `tests/controller/mission_test.c`, which is written to fail loudly if it changes.

9. **Who is responsible for restoring normal speed after a hump?** Entering
   `HUMP` issues `SET_SLOW_SPEED`. Only the `HUMP_COMPLETED` path issues
   `RESTORE_NORMAL_SPEED`; leaving the hump via a safety stop or via an
   obstacle never does.

   Whether that is a defect depends on an unanswered ownership question:
   - if `START_LINE_FOLLOW` already implies normal speed, then
     `RESTORE_NORMAL_SPEED` is redundant everywhere and should be removed; but
   - if speed is a separate axis that Buddy 2 keeps until told otherwise, the
     car finishes the course at hump speed after either of those paths.

   Section 4's "motion commands replace the previous Buddy 2 behaviour" is
   suggestive of the first reading but does not settle it. Buddy 2 owns the
   answer. Current behaviour is pinned by
   `test_slow_speed_is_not_restored_on_every_exit()`.

These decisions are intentionally visible instead of being hidden as arbitrary
constants in source code.

### How the open decisions are enforced

Each open decision that affects runtime behaviour has a characterisation test
that records what the code does **today**. Those tests assert current
behaviour, not correct behaviour. When the team settles a decision, the
corresponding test fails and points at the decision — which is the intended
outcome, and is why they are written that way.

## 9. Big picture for the team discussion

The data path is:

```text
Buddy sensor/command task
        |
        | typed mission event (not JSON)
        v
central mission task ---- decision ----> Buddy 2 motion task
        |                    |
        |                    +----------> Buddy 5 scan task, when required
        |
        +---- state and result ----> Buddy 1 telemetry queue
                                           |
                                           v
                                      MQTT over TCP/IP
                                           |
                                           v
                                      laptop broker
```

The buddies do not communicate with one another through MQTT, JSON or I2C.
They are software modules on the same Pico, so the future μT-Kernel integration
uses typed C structures in queues/mailboxes. MQTT and JSON are only the
robot-to-laptop boundary. The complete external wire contract is in
`buddy1/MQTT_PROTOCOL.md`.

### Normal line following

1. Buddy 3 samples two line sensors using the ADC and calculates line error.
2. Buddy 2 uses that error plus encoder feedback from GPIO interrupts to adjust
   both motor PWM outputs.
3. Buddy 4 may compare encoder-derived motion with IMU heading read over I2C.
4. Buddy 1 publishes slower status samples as JSON over MQTT; telemetry must
   never control or delay the steering loop.

### Barcode followed by a junction

1. Buddy 3 samples the third IR sensor using the ADC and decodes Code 39 in
   either direction.
2. Buddy 3's navigation layer resolves the letter through the accepted shared
   barcode map and sends `BARCODE_DECODED(letter, action)` to the mission task.
3. The mission controller stores the action but remains in `LINE_FOLLOW`.
4. When Buddy 3 sends `JUNCTION_DETECTED`, the controller sends
   `EXECUTE_TURN(action)` to Buddy 2.
5. Buddy 2 uses encoder feedback and optionally Buddy 4's I2C IMU heading to
   complete the turn, then sends `TURN_COMPLETED`.

### Hump

1. Buddy 4 detects the climb from accelerometer samples read over I2C and sends
   `HUMP_DETECTED`.
2. The controller tells Buddy 2 to use the slower hump speed; Buddy 3 keeps the
   line centred while one wheel is raised.
3. Buddy 4 measures the peak tilt/height while the car is moving and sends
   `HUMP_COMPLETED`; Buddy 1 reports this hump and the maximum for the run.
4. The controller restores normal speed.

### Obstacle, bypass and line recovery

1. Buddy 5 uses a timed GPIO trigger/echo measurement to detect an obstacle.
2. The controller enters `OBSTACLE_STOPPING` and tells Buddy 2 to stop.
3. Buddy 2 reports `MOTION_STOPPED`; only then does the controller request the
   scan, so Buddy 5 never profiles while the car is still rolling.
4. Buddy 5 moves the ultrasonic sensor with servo PWM, profiles the obstacle,
   and sends an avoidance plan.
5. The controller sends that plan to Buddy 2. Encoders keep the bypass distance
   repeatable; Buddy 4 can cross-check heading over I2C; Buddy 5 checks clearance.
6. Buddy 2 reports `BYPASS_COMPLETED`. Only then does the controller enter
   `LINE_SEARCH`; Buddy 3 reports `LINE_REACQUIRED` when the ADC readings confirm
   the original line.
7. The controller returns to `LINE_FOLLOW`, and Buddy 1 publishes the profile,
   chosen plan and state changes over MQTT.

### Safety stop, resume and impact

1. MQTT accepts barcode configuration while `CONFIGURING` and snapshot
   requests; it does not accept remote motion control.
2. A physical safety input or system safety rule may generate
   `STOP_REQUESTED`; Buddy 1 never changes motor PWM.
3. STOP and Buddy 4's `IMPACT_DETECTED` take priority over ordinary behaviour.
   An impact enters `FAULT`, which an ordinary RESUME cannot clear.
4. If the team permits a local resume, an interrupted turn or bypass starts
   `LINE_SEARCH` rather than replaying a partly completed movement.

### Why I2C is used here

I2C is not the link between buddies. It is the hardware bus for the
LSM303DLHC because that sensor provides native I2C addresses for its
accelerometer and magnetometer, needs only SDA and SCL, and its data rate is
well within I2C capacity. UART is not an interface offered by that sensor.
SPI can be faster, but speed is unnecessary here and it needs more signal
wires/chip-select handling. Thus I2C is the practical choice for this IMU,
not a universal claim that I2C is always better.

## 10. Recommended build sequence from here

1. Freeze the event names, units and open choices in this contract as a team.
2. Each buddy builds and host-tests their driver/algorithm behind that boundary.
3. Keep the shared pure mission controller host-tested while drivers are
   developed; do not wait for all hardware before testing scenarios.
4. Integrate Buddy 2 + Buddy 3 first for line following and junction turns.
5. Add Buddy 4 for hump/heading, then Buddy 5 for obstacle recovery.
6. Add the lecturer's μT-Kernel port and replace direct test calls with RTOS
   queues/tasks only after its actual APIs are available.
7. Add MQTT last so network timing cannot hide navigation defects, then run
   complete-course and failure-injection tests.
