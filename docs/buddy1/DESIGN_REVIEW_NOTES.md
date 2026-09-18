# Week 6 Design Review — Buddy 1 working notes

> **This is raw material, not an answer to paste.**
>
> The template says it outright: *"Do not paste AI output directly as the final
> design. Summarise, challenge, correct, and justify AI suggestions using
> engineering reasoning."* And your lecturer will ask you to explain any line
> of it.
>
> So: read each item, disagree with the ones you disagree with, rewrite them in
> your own words, and delete anything you cannot defend. The numbers here are
> real — they come from code that builds and tests that pass — but the
> reasoning has to become yours.

Covers only **Buddy 1's** sections. The other four buddies fill in theirs.

---

## §1.1 Functional requirements — communications

Written with "shall", numbered, and each one testable with a yes/no outcome —
the four characteristics from the W2.1 Requirements lecture.

| ID | Requirement |
|---|---|
| FR-C1 | The system shall associate with a configured WiFi access point on power-up. |
| FR-C2 | The system shall publish telemetry to a configured MQTT broker. |
| FR-C3 | The system shall subscribe to the permitted barcode-configuration and snapshot command topics. |
| FR-C4 | The system shall validate and forward a new barcode mapping, **without recompilation or restart**, while the robot is in `CONFIGURING`. |
| FR-C5 | On request, the system shall publish the current value of every subsystem's telemetry. |
| FR-C6 | The system shall notify subscribers when it loses its connection unexpectedly. |
| FR-C7 | The system shall re-establish a lost connection automatically. |
| FR-C8 | The system shall acknowledge every barcode-map request with accepted/rejected status, configuration version and a rejection reason. |

**Traceability** — FR-C4 comes from the briefing, not the write-up: *"you need
to be able to send a command through the Wi-Fi to reprogram this thing on the
fly, and not recompile the whole code."* FR-C5 likewise: *"give me a summary…
the car should be able to send back a snapshot of what's happening at that
point of time."* Worth citing the source, because neither appears in the
written brief and a marker may not remember saying them.

## §1.2 Non-functional requirements

Numeric, with tolerances, so each maps to a measurement rather than an opinion.

| ID | Requirement | How it will be verified |
|---|---|---|
| NFR-C1 | A telemetry call shall return within **100 µs** and shall never block the caller. | Toggle a GPIO around `tele_report_*()`, measure worst case on a scope |
| NFR-C2 | A status message shall be published every **2 s ± 200 ms** while connected. | Timestamp deltas in the subscriber log |
| NFR-C3 | Telemetry loss shall be **detectable**: every message carries a sequence number, and the count of dropped messages is published. | Force an overflow; confirm the gap and the counter agree |
| NFR-C4 | Reconnection shall be retried with backoff between **1 s and 16 s**. | Kill the access point, time the retries |
| NFR-C5 | The communications subsystem shall consume **zero GPIO pins**. | Inspection of the pin table |
| NFR-C6 | The event FIFO, periodic latest-value slots and snapshot cache shall occupy **≤ 7 kB** of RAM. | ARM symbols: **5,535 bytes** including their counters/metadata. **Passed.** |

NFR-C1 is the one that matters most, and it is the reason the whole subsystem
is shaped the way it is. A telemetry call that could block would stall
line-following, and the car would leave the track.

> NFR-C6 deliberately includes all three stores. The original draft counted
> only the FIFO and snapshot cache; the topic-aware design also has three
> latest-value slots. The ARM image reports 5,484 bytes for the three data
> arrays and 51 bytes of supporting counters/metadata, for 5,535 bytes total.
> This is target evidence, not a host `sizeof` estimate.

---

## §2 Hardware integration — Buddy 1's rows

| Device | Type | Signal | Pico GPIO | Interface | Notes / task owner |
|---|---|---|---|---|---|
| CYW43439 WiFi | Comms | — | **none external** | internal SPI | On the Pico W module itself. Owned by `net_task`. |
| (reserved) | — | wireless SPI | GP23, GP24, GP25, GP29 | internal | **Consumed by the Pico W. Not available to anyone.** |
| On-board LED | Indicator | — | `CYW43_WL_GPIO_LED` | via CYW43 | Link status. Not an RP2040 pin — driving it is an SPI transaction taking microseconds. |

**The communications subsystem uses no external pins at all.** Worth stating
explicitly: it means Buddy 1 never competes for the pin budget, and the whole
budget is available to the other four.

### A pin-budget problem the team should resolve in this document

Taken by the Robo Pico before anyone chooses anything: **GP8–11** motors,
**GP12–15** servos, GP18 NeoPixel, GP20/21 buttons, GP22 buzzer. Taken by the
Pico W: **GP23/24/25/29**.

That leaves GP0–7, GP16, GP17, GP19, GP26, GP27, GP28.

**GP26, GP27 and GP28 are the only ADC-capable pins.** Three IR sensors
reading analogue values would consume all three — which also costs the VBAT
sense on GP28. Either the barcode sensor runs on its digital output (losing
software-adjustable thresholds, which works against "robust against lighting
changes"), or the team accepts losing battery monitoring. **This is exactly
the kind of thing an architecture freeze exists to settle.**

---

## §3 RTOS task table — Buddy 1's task

| Task | Purpose | Inputs | Outputs | Trigger | Priority |
|---|---|---|---|---|---|
| `net_task` | Maintain the WiFi/MQTT link; drain telemetry; handle permitted inbound messages | Telemetry stores, MQTT socket | MQTT publishes and validated configuration requests | Periodic, 10 ms | **Lowest** |

> **Watch the priority numbering when you fill this in.** In **μT-Kernel 3.0,
> 1 is the HIGHEST priority** and `CNF_MAX_TSKPRI` is the lowest — the inverse
> of FreeRTOS, where a larger number means higher. So "lowest priority" here
> means a **large** number, not a small one. Copying Lab 5's numbers across
> unchanged would invert the whole scheduling design and make the most
> critical task the least important. Verified against the TRON Forum
> specification; state the convention explicitly in your submission so a
> marker can see you knew.

### Priority justification

**`net_task` is deliberately the lowest-priority task in the system.**

A late telemetry message costs nothing — it arrives a few milliseconds later
and nobody notices. A late steering correction costs the track. Under
rate-monotonic reasoning (Lab 5: *shorter deadline, higher priority*),
communications has the longest deadline of anything on the robot and therefore
belongs at the bottom.

This is only safe because **the telemetry API never performs network I/O.**
Reporting updates bounded telemetry storage and returns; a low-priority task drains it later. If
reporting published directly, a low-priority network task would mean sensing
tasks waiting behind it, and the priority assignment would be actively
dangerous instead of merely sensible.

### One task or two?

Command handling could be its own task. It is not, because commands arrive a
handful of times per run and handling one takes microseconds; a second task
would cost a second stack for no measurable benefit. **State this as a decision
with a reason, not an omission** — the template asks for justification.

---

## §4 Task interaction — Buddy 1's edges

```
  every other task ──tele_report_*()──▶ [ event FIFO + latest values ]
                        bounded/no network I/O     │
                                                  ▼
                                            ┌───────────┐
                                            │ net_task  │──▶ MQTT ──▶ broker
                                            │  (low)    │
                                            └───────────┘
                                                  ▲
   lwIP IRQ ──buffer + flag──▶ [ pending command ]┘
   (deferred, never acts in interrupt context)
                                                  │
                        validated map request     ▼
                                  shared robot_config
```

Two things in that diagram are the design, and both are worth a sentence in
your explanation:

1. **The telemetry stores are a gatekeeper** (Lab 5 Part 4). One task owns the
   network; everyone else posts and walks away. No mutex on the network, so no
   priority inversion and no deadlock on it.
2. **The IRQ defers** (Lab 5 Part 5). lwIP's callbacks run in a low-priority
   interrupt. They buffer the bytes and raise a flag; `net_task` does the work
   in ordinary context. Acting on a command inside the callback would mean
   touching the queue from an interrupt while the task is midway through
   touching it — a race that shows up as intermittent lost telemetry.

---

## §6 Subsystem statechart — communications

This overview combines the two implemented non-blocking state machines:
`wifi_connection_process()` owns WiFi association/recovery and
`buddy1_mqtt_process()` owns the broker session/recovery.

```
        ┌──────┐
        │ BOOT │◀────────────────────────────┐
        └───┬──┘                             │
            │ start association              │ backoff expires
            ▼                                │
    ┌───────────────┐                  ┌─────┴─────┐
    │ WIFI_JOINING  │──join failed────▶│  BACKOFF  │
    └───────┬───────┘                  └─────▲─────┘
            │ link up                        │
            ▼                                │ connect refused,
    ┌──────────────────┐                     │ or link lost
    │ MQTT_CONNECTING  │─────────────────────┤
    └───────┬──────────┘                     │
            │ CONNACK accepted               │
            ▼                                │
       ┌─────────┐                           │
       │ ONLINE  │───────────────────────────┘
       └─────────┘
        publishes telemetry, status every 2 s,
        handles deferred commands
```

- **Entering ONLINE**: subscribe to `cmd/#` at QoS 1, publish retained status.
- **Backoff** doubles 1 s → 16 s and resets on success, so a broker that is
  down is not hammered but a brief glitch recovers quickly.
- **The Last Will** is registered at connect time, so an *unexpected* exit from
  ONLINE — a crash, a flat battery — is announced by the broker on our behalf.
  That is the difference between telemetry that stops and telemetry that says
  why it stopped.

---

## §7 Assumptions — Buddy 1's

At least one per buddy is required. These are mine; each needs a risk, a
validation method and an owner.

| ID | Assumption | Risk if wrong | Validation | Status |
|---|---|---|---|---|
| A-C1 | A phone hotspot is stable enough that reconnects are rare during a 5-minute run. | Telemetry gaps mid-demo; time lost to reconnects. | Run 10 minutes, count reconnects from the status message. Target: zero. | Open |
| A-C2 | A 16-slot event FIFO is deep enough for the team's burst rate. | Important events are dropped during an extreme burst. | Flood periodic telemetry and burst events separately; `dropped` must remain 0 in a full run. | Open |
| A-C3 | Publishing never delays a control task. | Line-following jitter; the car leaves the track. | GPIO trace pin around `tele_report_*()`, worst case on a scope. Must be < 100 µs (NFR-C1). | Open |
| A-C4 | **The demo-day broker accepts anonymous connections on port 1883.** | **The car cannot connect at all on the day.** Everything else works and nothing is visible. | Ask the lecturer now. Test against a second machine's broker before the demo, not on it. | Open |
| A-C5 | A 26-letter barcode remap fits one MQTT message. | The demo-day command is truncated and rejected. | Measured: 391 bytes against a 512-byte buffer. **Validated.** | Closed |

**A-C4 is the one nobody thinks of.** Mosquitto 2.x refuses anonymous clients
and non-local connections *by default*. If the examiner's broker ships with
stock settings, every group fails identically — and finding that out at 9am on
demo day is the worst possible time. Ask early; it costs one email.

---

## §8 AI-supported design evidence

The template wants an honest record: useful ideas, **weak ideas**, and the
team's decision. Weak ideas are graded material, not an admission — §8.1's
third column asks for them by name.

### §8.1 What actually happened

| Useful | Weak or wrong | Your decision |
|---|---|---|
| Proposed a non-blocking queue with a single draining task (the gatekeeper pattern) so telemetry can never stall control. | — | *(yours)* |
| Proposed splitting pure logic from hardware code so most of it tests on a laptop. | — | *(yours)* |
| — | **The first API draft missed requirements that were in the brief**: encoder counts, IMU motion events, turn rate, two of the five obstacle profile fields. It was written from a sketch instead of from the write-up. | *(yours)* |
| — | **Forced C99 to satisfy Barr-C rule 1.1a, which broke the Pico SDK** — the SDK's headers use C11's `static_assert`. | *(yours)* |
| — | **Wrote a barcode parser that accepted truncated payloads.** `{"A":"LEFT"` parsed as a complete one-letter mapping. On demo day that would silently apply a partial map. | *(yours)* |
| — | **Missed JSON escaping entirely.** A quote, backslash or newline in a log message broke the output — the backslash case *silently*, since `\t` means TAB in JSON. | *(yours)* |
| — | **Two concurrency bugs**: acted on MQTT commands inside an lwIP interrupt callback while the main loop touched the same queue, and called lwIP unbracketed from main context, contrary to the SDK's stated contract. | *(yours)* |

The pattern worth writing up in your own words: **the AI was good at structure
and bad at requirements.** Architecture suggestions held up; anything that
needed checking against the actual brief, the actual datasheet or the actual
SDK documentation had to be verified, and roughly half of it was wrong until it
was.

### §8.3 Gap check — acting as a strict reviewer

Gaps that are still open, with the template's response vocabulary:

| Gap | Response | Action / evidence |
|---|---|---|
| The hardware-facing communications modules have never executed on the Pico. They cross-compile; that is not the same thing. | **Needs validation** | Flash and run before the review. Both concurrency bugs above were found by reading documentation, not by testing — which indicates how much could still be hiding. |
| The telemetry lock hooks are no-ops in the current bare-metal build. | **Deferred** | Correct while only the main loop touches telemetry. Replace the hooks with a short, measured, non-sleeping μT-Kernel critical section before several tasks report concurrently. |
| Event FIFO depth of 16 still needs hardware-load evidence. | **Accepted as risk** | Periodic traffic is now coalesced separately; validate the remaining event burst capacity in a full run. |
| High-rate telemetry could evict a rare, critical message. | **Fixed in host logic** | Motion, line and IMU now use one latest-value slot each; important events use a separate FIFO and drain first. Hardware timing remains to be validated. |
| MQTT STOP/RESUME conflicted with the no-remote-control rule. | **Removed** | The assessed command surface now contains barcode configuration and snapshot only. Safety stop events remain available to physical/system safety logic. |
| A temporary lwIP output-buffer shortage could discard an accepted map's acknowledgement. | **Fixed in target logic** | A single bounded QoS 1 reply slot retains and retries the reply across congestion/reconnection. Hardware broker testing remains required. |
| The mission state machine was presented as Buddy 1 work although the brief shows a separate Vehicle Controller. | **Fixed for this branch** | Shared vocabulary/configuration live in project-level `include/shared/` and `src/shared/`; the controller itself is deliberately not hidden inside Buddy 1. |

---

## §9 Design consistency — Buddy 1's answers

**Which task owns each device?** `net_task` owns the CYW43 WiFi chip. A future
link-status LED hook can remain in that task. It owns no external pins.

**Which task has the highest priority, and why?** Not this one. `net_task` is
deliberately lowest: the longest deadline on the robot, and safe there only
because the telemetry API never blocks.

**Which statechart corresponds to which task?** The combined communications
statechart (§6) belongs to `net_task`; `communication_process()` advances its
WiFi and MQTT state machines once per invocation.

**How are the requirements reflected in the design?** FR-C1/C2 → the
BOOT→JOINING→CONNECTING→ONLINE path. FR-C3 → deferred permitted-message handling.
FR-C4/C8 → parsing, shared lifecycle validation and structured acknowledgement. FR-C5 → the snapshot
cache in `src/buddy1/telemetry.c`. FR-C6 → the MQTT Last Will. FR-C7 → the BACKOFF state.

**What is still expected to change after Week 6?** The RTOS port to μT-Kernel,
which calls `communication_process()` from a task body without altering the
module. Further queue sizing changes are needed only if physical measurement
shows the 16-event FIFO is insufficient.

---

## What you still have to supply

- Your group ID and student IDs.
- §5, draw the **system-level** statechart from the implemented draft in
  the future shared mission controller, after the team confirms the open transitions.
- §6.2 and §6.3, two more subsystem statecharts from other buddies.
- One assumption per buddy from the other four.
- Your own words in every "your decision" cell in §8.
