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
  main.c              # Minimal Pico USB serial smoke test
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

The build currently compiles only `src/main.c`, which prints a USB serial greeting
once per second. Buddy 1–5 remain scaffolding and are not included in the build.
No GPIO assignments or micro T-Kernel integration have been added.
`include/system_config.h` remains a placeholder. There is no automated test suite.

## Build the Pico smoke test

Required tools: CMake, an ARM embedded GCC toolchain (`arm-none-eabi-gcc` and
`arm-none-eabi-g++`), Make, Python 3, Git, and a native C/C++ compiler. Install the
Pico SDK with its submodules, including TinyUSB for USB serial support.
`pico_sdk_import.cmake` is copied unchanged from Pico SDK 2.3.0.

From the repository root, using the original RP2040 Raspberry Pi Pico:

```sh
cmake -S . -B build -G "Unix Makefiles" \
  -DPICO_SDK_PATH="$HOME/pico/pico-sdk" \
  -DPICO_BOARD=pico
cmake --build build --parallel
```

Adjust the SDK path to your installation. For a different Pico model, select its
matching SDK board instead. SDK 2.3.0 can download and build picotool automatically
for UF2 generation, so the first configuration may require internet access.
Generated files stay in the ignored `build/` directory.

## Flash and verify

1. Hold BOOTSEL while connecting the Pico to your computer with a USB data cable.
2. Release BOOTSEL once the `RPI-RP2` drive appears.
3. Copy `build/pico_hello.uf2` onto that drive. The Pico reboots automatically.
4. On macOS, find the USB serial port with `ls /dev/cu.usbmodem*` and open the
   matching port with `screen /dev/cu.usbmodemXXXX 115200`, replacing the example
   device name with the actual one.
5. Verify that `Hello from Raspberry Pi Pico!` appears once per second.

To exit screen, press Control-A, then K, and confirm with Y.
The smoke test uses USB serial with UART serial disabled and does not configure
any application GPIO pins. Flashing instructions follow the
[official Pico SDK documentation](https://www.raspberrypi.com/documentation/microcontrollers/c_sdk.html).
