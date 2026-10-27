#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
# Archive the original Buddy 3 sources without switching/resetting the checkout.
baseline_ref=${BASELINE_REF:-71193e2}
baseline_dir=$(mktemp -d "${TMPDIR:-/tmp}/buddy3-baseline.XXXXXX")
git -C ../.. archive "$baseline_ref" app_program |
    tar -x -C "$baseline_dir"
make -s compare bench-compare BASELINE_DIR="$baseline_dir/app_program/subsystem3"
