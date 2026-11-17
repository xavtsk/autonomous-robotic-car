#!/bin/sh
# Change Requirement 1: regenerate every desk-side measurement and test log.
# Run from the repository root:  sh testing/change_requirement_1/run_all.sh
# Needs a host C compiler and python3. The firmware/RAM part also needs the Arm
# toolchain on PATH, PICO_SDK_PATH and config/wifi_secrets.h (skipped if absent).
# The on-car timing test is separate (see docs/CHANGE_REQUIREMENT_1_EVIDENCE.md).
set -u
DAY=$(date +%Y-%m-%d)
OUT=testing/change_requirement_1/logs
mkdir -p "$OUT"
FAIL=0
step() { # name, command...
    name=$1; shift
    if "$@" > "$OUT/${DAY}_$name.txt" 2>&1; then echo "PASS  $name"; else echo "FAIL  $name"; FAIL=1; fi
}
step widths       python3 testing/change_requirement_1/check_widths.py
step bend_filter  sh -c 'cc -O2 -Wall testing/change_requirement_1/bend_filter_check.c -o /tmp/cr1_bend && /tmp/cr1_bend'
step sizes_buddy1 make -s -C testing/subsystem1 sizes BUILD=/tmp/cr1_b1
step sizes_buddy2 make -s -C testing/subsystem2 sizes BUILD=/tmp/cr1_b2
step sizes_buddy3 make -s -C testing/subsystem3 sizes BUILD=/tmp/cr1_b3
for n in 1 2 3 4 5; do
    step "tests_buddy$n" make -s -C "testing/subsystem$n" test BUILD="/tmp/cr1_t$n"
done
step tests_integration make -s -C testing/integration -f controller_test.mk
step tests_laptop_codec python3 testing/subsystem1/test_laptop_codec.py
step tests_dashboard sh -c 'for f in testing/dashboard/test_*.py; do python3 -m unittest -q "$f" || exit 1; done'
if command -v arm-none-eabi-gcc > /dev/null && [ -f config/wifi_secrets.h ]; then
    step ram_target python3 testing/change_requirement_1/measure.py
else
    echo "SKIP  ram_target (needs the Arm toolchain and config/wifi_secrets.h)"
fi
echo "Logs: $OUT/${DAY}_*.txt"
exit $FAIL
