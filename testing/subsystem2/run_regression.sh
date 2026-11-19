#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
# Compare against the imported teammate contract, not the superseded IRQ driver.
baseline_ref=${BASELINE_REF:-4ad2cc42595722cedb47102e4343298339040d40}
baseline_dir=$(mktemp -d "${TMPDIR:-/tmp}/buddy2-baseline.XXXXXX")
trap 'rm -rf -- "$baseline_dir"' EXIT
git -C ../.. archive "$baseline_ref" app_program |
    tar -x -C "$baseline_dir"
mkdir -p ../../build/buddy2
make -s -C ../subsystem3 feedback MOTION="$baseline_dir/app_program/subsystem2" > ../../build/buddy2/before.trace
make -s -C ../subsystem3 feedback > ../../build/buddy2/after.trace
cmp ../../build/buddy2/before.trace ../../build/buddy2/after.trace
printf 'PASS: baseline and candidate motion feedback/report/stop/calibration/watchdog assertions\n'
printf 'LIMIT: equal PASS summaries are not complete behavioral traces; use tools/compliance/differential.py.\n'
python3 -c 'import hashlib,sys; [print(hashlib.sha256(open(p,"rb").read()).hexdigest(), p) for p in sys.argv[1:]]' ../../build/buddy2/before.trace ../../build/buddy2/after.trace
