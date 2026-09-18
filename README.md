# Autonomous Robotic Car

University Embedded Systems project developed by five team members (Buddies).
All five subsystems will integrate into one autonomous robotic car.

## Project requirements

- Raspberry Pi Pico + Robo Pico platform
- C language and Pico C SDK
- micro T-Kernel RTOS
- Barr C Coding Standard
- Clean, modular subsystem interfaces

## Subsystem ownership

| Buddy | Responsibility |
| --- | --- |
| 1 | WiFi communication, command and telemetry |
| 2 | Motion control using motors and wheel encoders |
| 3 | IR line following and barcode decoding using 3 IR sensors |
| 4 | IMU-based motion and terrain monitoring |
| 5 | Adaptive ultrasonic scanning and obstacle profiling using an ultrasonic sensor and servo |

## Repository structure

```text
src/
  main.c              # Placeholder for the final system entry point
  buddy1/             # Buddy 1 subsystem source
  buddy2/             # Buddy 2 subsystem source
  buddy3/             # Buddy 3 subsystem source
  buddy4/             # Buddy 4 subsystem source
  buddy5/             # Buddy 5 subsystem source
include/
  system_config.h     # Placeholder for agreed shared configuration
tests/
  buddy1/             # Buddy 1 tests
  buddy2/             # Buddy 2 tests
  buddy3/             # Buddy 3 tests
  buddy4/             # Buddy 4 tests
  buddy5/             # Buddy 5 tests
docs/
  buddy1/             # Buddy 1 documentation
  buddy2/             # Buddy 2 documentation
  buddy3/             # Buddy 3 documentation
  buddy4/             # Buddy 4 documentation
  buddy5/             # Buddy 5 documentation
```

Each Buddy directory contains a short README so Git tracks it.

## Collaboration

- Work on a separate branch and submit a GitHub pull request for review.
- Keep subsystem implementation in the assigned `src/buddyN/` directory,
  with corresponding tests and documentation in `tests/buddyN/` and `docs/buddyN/`.
- Agree on subsystem interfaces before integration. Document inputs, outputs,
  ownership of shared data, and RTOS interactions in the relevant documentation.
- Add public interface headers under `include/` as interfaces are agreed;
  keep private implementation details within each subsystem.
- Coordinate changes to shared files such as `src/main.c` and
  `include/system_config.h` through pull requests.
- Follow the Barr C Coding Standard when implementation begins.

## Current status

Buddy 1 now has an implemented and host-tested communications draft covering
telemetry, command validation, WiFi/MQTT state machines, heartbeat and
reconnection. Its host suite is under `tests/buddy1/`, and its wire contract
is under `docs/buddy1/`.

The shared, hardware-independent mission controller is under
`include/controller/` and `src/controller/`, with its scenario tests under
`tests/controller/`. It is intentionally separate from Buddy 1.

Buddies 2–5 remain scaffolding. `src/main.c` and `include/system_config.h` are
still shared placeholders. Team-level Pico C SDK build configuration,
micro T-Kernel integration and physical hardware validation remain pending.
