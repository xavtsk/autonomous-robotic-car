#!/bin/sh
# Desk-side checks. Real timing/reconnection needs the separate physical run.
# Generated logs never overwrite historical checked-in evidence.
set -u
ROOT=$(CDPATH= cd -- "$(dirname -- "$0")/../.." && pwd)
cd "$ROOT" || exit 1
DAY=$(date -u +%Y-%m-%dT%H%M%SZ)
REV=$(git rev-parse --short=12 HEAD) || exit 1
OUT_BASE=${CR1_OUTPUT_DIR:-build/change-requirement-1}
mkdir -p "$OUT_BASE" || exit 1
OUT=$(mktemp -d "$OUT_BASE/${DAY}_${REV}_XXXXXX") || exit 1
OUT=$(CDPATH= cd -- "$OUT" && pwd)
WORK=$(mktemp -d "${TMPDIR:-/tmp}/cr1.XXXXXX") || exit 1
trap 'rm -rf -- "$WORK"' EXIT HUP INT TERM
FAIL=0
INCOMPLETE=0
printf 'revision=%s\nrecorded_at=%s\n' "$(git rev-parse HEAD)" "$DAY" > "$OUT/metadata.txt"
git status --short >> "$OUT/metadata.txt"
printf 'step\tstatus\texit_code\n' > "$OUT/results.tsv"
step() {
    name=$1
    shift
    "$@" > "$OUT/$name.txt" 2>&1
    code=$?
    if python3 -c 'import pathlib,sys; from tools.compliance.run_host import classify_output; result=classify_output(int(sys.argv[1]),pathlib.Path(sys.argv[2]).read_text(errors="replace")); sys.exit(result["status"]!="PASS")' "$code" "$OUT/$name.txt"; then
        printf 'PASS  %s\n' "$name"
        printf '%s\tPASS\t0\n' "$name" >> "$OUT/results.tsv"
    else
        printf 'FAIL  %s (exit %s)\n' "$name" "$code"
        printf '%s\tFAIL\t%s\n' "$name" "$code" >> "$OUT/results.tsv"
        FAIL=1
    fi
}
step widths python3 testing/change_requirement_1/check_widths.py
step style python3 tools/check_c_style.py
step bend_filter cc -O2 -Wall testing/change_requirement_1/bend_filter_check.c -o "$WORK/bend_filter"
step bend_filter_run "$WORK/bend_filter"
for n in 1 2 3; do
    step "sizes_buddy$n" make -s -C "testing/subsystem$n" sizes "BUILD=$WORK/sizes$n"
done
for n in 1 2 3 4 5; do
    step "tests_buddy$n" make -s -C "testing/subsystem$n" test "BUILD=$WORK/tests$n"
    step "sanitize_buddy$n" make -s -C "testing/subsystem$n" sanitize "BUILD=$WORK/sanitize$n"
done
step tests_s3_extra make -s -C testing/subsystem3 codec drive feedback replay "BUILD=$WORK/s3extra"
step tests_integration make -s -C testing/integration -f controller_test.mk test "BUILD=$WORK/integration"
step sanitize_integration make -s -C testing/integration -f controller_test.mk sanitize "BUILD=$WORK/integration-sanitize"
step diagnostics make -s -C testing/diagnostics -f firmware_test.mk test smoke disabled "BUILD=$WORK/diagnostics"
step network_wifi make -s -C testing/network_wifi test sanitize "BUILD=$WORK/network"
step tests_laptop_codec python3 testing/subsystem1/test_laptop_codec.py
for directory in dashboard change_requirement_1 diagnostics; do
    step "python_$directory" python3 -m unittest discover -v -s "testing/$directory" -p 'test_*.py'
done
step frontend node --test testing/dashboard/test_frontend_controls.js
if command -v arm-none-eabi-gcc >/dev/null && [ -f config/wifi_secrets.h ] && [ -n "${PICO_SDK_PATH:-}" ]; then
    step ram_target python3 testing/change_requirement_1/measure.py
else
    printf 'INCOMPLETE ram_target: needs Arm toolchain, PICO_SDK_PATH and local build configuration\n'
    printf 'ram_target\tINCOMPLETE\t2\n' >> "$OUT/results.tsv"
    INCOMPLETE=1
fi
printf 'Logs: %s\n' "$OUT"
if [ "$FAIL" -ne 0 ]; then exit 1; fi
if [ "$INCOMPLETE" -ne 0 ]; then exit 2; fi
exit 0
