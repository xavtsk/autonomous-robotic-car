#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
# Archive the original Buddy 3 sources without switching/resetting the checkout.
baseline_ref=${BASELINE_REF:-4ad2cc42595722cedb47102e4343298339040d40}
baseline_dir=$(mktemp -d "${TMPDIR:-/tmp}/buddy3-baseline.XXXXXX")
trap 'rm -rf -- "$baseline_dir"' EXIT
git -C ../.. archive "$baseline_ref" app_program |
    tar -x -C "$baseline_dir"
make -s compare bench-compare BASELINE_DIR="$baseline_dir/app_program/subsystem3"
