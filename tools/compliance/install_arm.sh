#!/usr/bin/env bash
set -euo pipefail
# Official Arm 14.3.Rel1 Linux x86_64 archive; compiler reports 14.3.1.
# Checksum retrieved from the adjacent official .sha256asc on 2026-11-20.
destination=${1:?Supply an empty installation directory}
archive=arm-gnu-toolchain-14.3.rel1-x86_64-arm-none-eabi.tar.xz
checksum=8f6903f8ceb084d9227b9ef991490413014d991874a1e34074443c2a72b14dbd
mkdir -p "$destination"
curl --fail --location --retry 3 "https://developer.arm.com/-/media/Files/downloads/gnu/14.3.rel1/binrel/$archive" -o "$destination/$archive"
printf '%s  %s\n' "$checksum" "$destination/$archive" | sha256sum --check -
tar -xJf "$destination/$archive" -C "$destination"
compiler_bin="$destination/arm-gnu-toolchain-14.3.rel1-x86_64-arm-none-eabi/bin"
test "$("$compiler_bin/arm-none-eabi-gcc" -dumpfullversion)" = 14.3.1
"$compiler_bin/arm-none-eabi-gcc" --version
printf '%s\n' "$compiler_bin" >> "${GITHUB_PATH:?Run from GitHub Actions}"
