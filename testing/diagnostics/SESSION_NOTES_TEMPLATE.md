# Robot test session notes

Copy into the recording folder after the logger creates it. Leave unavailable
measurements marked `Not measured`, never invented. Do not include Wi-Fi/broker
passwords or other credentials. A replay is not a hardware test.

## Identify this run

| Item | Entry |
|---|---|
| Date/time/timezone and tester | |
| Hardware test or replay | |
| Test purpose | |
| Session folder / raw log | |
| Firmware UF2 name and build identifier | |
| Branch/revision and relevant uncommitted settings | |
| Build options: TEST, IR_BENCH, DIAGNOSTICS, WIFI, NET, MQTT | |
| Teammate's working baseline retained at | |
| Startup method / autonomous start invoked? | |
| Reset/reflash/disconnection during capture | |

## Conditions and settings

| Item | Entry |
|---|---|
| Track surface, lighting, sensor mounting height | |
| Enabled sensors and calibrated white/black ranges | |
| Battery/power arrangement and condition | |
| Wheels raised or on floor; clear test area | |
| Wheel diameter, track width, encoder counts/revolution | |
| Motor/encoder inversion and speed calibration values | |
| Motion/IR periods and task priorities | |
| MQTT client/robot protocol versions; network test condition | |
| Maximum intended continuous operation before counter reset | |

## Actions and measurements

Record each attempt, including failures. Use signed counts/speeds and state the
units. The physical result must be measured independently of the encoder estimate.

| Attempt | Requested action / target | Duration | Start L/R counts | End L/R counts | Robot-reported result | Independently measured result | Fault/observation |
|---|---|---|---|---|---|---|---|
| 1 | | | | | | | |
| 2 | | | | | | | |
| 3 | | | | | | | |

## Timing, resources and communication

| Measurement | Result / evidence |
|---|---|
| MCU motion start-gap min/max and max execution interval | |
| MCU IR start-gap min/max and max execution interval | |
| Diagnostic publication skips / recording interruptions | |
| Network loss/recovery and heartbeat recovery duration | |
| Queue-full rejection / lost reports / snapshot behaviour | |
| Stack/heap high-water | Not measured |
| Physical distance/turn/barcode video or photo | |

Host-arrival CSV times are not MCU task timing. A timing summary includes
preemption/blocking and does not prove WCET. Identify which workload was active.

## Conclusion

Result (PASS / FAIL / INCOMPLETE / NOT RUN):

Acceptance criterion used:

What differed from the baseline:

What the evidence supports, and what it does not:

Proposed next change (one setting at a time):

Unresolved question / teammate to confirm:
