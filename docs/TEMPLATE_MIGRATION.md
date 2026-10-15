# Migration to the INF2004 RTOS template

Source: https://github.com/sirfonzie/INF2004_ProjectTemplate
Upstream revision: `c3b09544cb1fd6dac6ad4b8f523be0af59b44482`
Original car skeleton: `e6f533437127e5660cc64b44bdbcfe5399a4a212`

## Path changes

| Previous path | New path |
|---|---|
| `src/buddyN/*.c` | `app_program/subsystemN/*.c` |
| `include/buddyN/*.h` | `app_program/subsystemN/*.h` |
| `src/buddyN/README.md` | `app_program/subsystemN/README.md` |
| `include/system_config.h` | `app_program/system_config.h` |
| `tests/buddyN/` | `testing/subsystemN/` |
| Comment-only `src/main.c` | RTOS entry point `app_program/app_main.c` |

All original Buddy module source/header basenames are preserved. Their relative
header includes were adjusted for co-location; no module functionality was added.
The top-level `include/` is now the imported kernel/BSP include tree.

## Adaptation

The upstream kernel, libraries, device drivers, configuration, linker scripts,
host tools and Make infrastructure are imported without functional changes.
Upstream subsystem demos are replaced with empty init/start hooks for all five
Buddies. No application tasks, car GPIO assignments, or control loops are added.
Tests report SKIP, and example test cases/results have been cleared from the
new logs so they cannot be mistaken for car verification.

The shared `app_main.c` retains the template's two-phase startup and task helper.
`app.h` also includes the preserved shared configuration placeholder. `TEAM.md`
maps subsystem numbers to car responsibilities; student owners and resource
claims remain for the team to fill in.

This is a source-tree migration on `xavier/development`, not a merge into `main`.

## Validation

Both Make profiles compiled and linked successfully with Pico SDK 2.3.0,
Arm GNU Toolchain 15.3.1, and macOS Make 3.81:

- Default dual-core USB/WiFi profile (`TEST=0`).
- The same profile with `TEST=1`, including all five placeholder test hooks.

Both generated UF2 files passed block-format and RP2040 family-ID checks.
All 58 original Buddy C/header files were checked against the clean skeleton;
only their header include paths changed. The imported framework files were
checked against the recorded upstream revision. Generated outputs and local
credentials are ignored by Git. No hardware/runtime verification was performed.
