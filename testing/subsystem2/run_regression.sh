#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
# Compare against the imported teammate contract, not the superseded IRQ driver.
baseline_ref=${BASELINE_REF:-71193e2}
baseline_dir=$(mktemp -d /private/tmp/buddy2-baseline.XXXXXX)
git -C ../.. archive "$baseline_ref" app_program |
    tar -x -C "$baseline_dir"
mkdir -p ../../build/buddy2
make -s -C ../subsystem3 feedback MOTION="$baseline_dir/app_program/subsystem2" > ../../build/buddy2/before.trace
make -s -C ../subsystem3 feedback > ../../build/buddy2/after.trace
cmp ../../build/buddy2/before.trace ../../build/buddy2/after.trace
printf 'PASS: imported and integrated motion feedback/report/stop/calibration/watchdog assertions\n'
shasum -a 256 ../../build/buddy2/before.trace ../../build/buddy2/after.trace
