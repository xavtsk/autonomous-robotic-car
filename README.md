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

This repository contains structure and placeholders only. No subsystem logic,
hardware drivers, GPIO assignments, or specific sensor models are defined.
`src/main.c` is a comment-only placeholder, and `include/system_config.h`
contains only an include guard and a comment.

Build configuration, Pico C SDK setup, and micro T-Kernel integration will be
added later. There is currently no runnable firmware or test suite.
