# Buddy 3 representation regression evidence

**Historical measurements below:** these refer to the earlier navigation API.
The current baseline is `71193e2`, navigation packets are **60 bytes / 479 bits**,
and the shared runtime snapshot/command are **122/11 bytes**. Regression now
embeds the current core and navigation suites, compares 58,915 observed rows
plus stationary bench output, and retains the full new odometry/readiness data.
See [current integration review](../../docs/INTEGRATION_2026-10-25.md).

Checked on 2026-10-23 in the local `integration/buddies-1-3` working tree.
No hardware was accessed, firmware flashed, or repository commit/push made by
these checks. These results do not prove track calibration, moving barcode
recognition, correct-original-line recovery or real RTOS/interrupt timing.

## Reproduce

```
make -C testing/subsystem3 test
make -C testing/subsystem3 regression
make -C testing/subsystem3 codec
make -C testing/subsystem3 sanitize
make -C testing/subsystem3 sizes
```

`regression` archives revision `477230c5069b49f6113f5a43f3cb3def2595178b`
without checking it out. Its portable Buddy 3 algorithm sources match the saved
pre-change working tree. Runtime integration differences are not hidden in this
comparison: this harness tests the portable acquisition/calibration/classifier,
line/steering/barcode/navigation modules, not the controller task or GPIO driver.
The same regression command separately exercises the real stationary
`APP_IR_BENCH=1` wrapper with a small fake RTOS/ADC/console boundary and compares
its printed diagnostic fields and counters against the original wrapper.
The baseline can be overridden with `BASELINE_REF`. `make compare` additionally
accepts an explicit `BASELINE_DIR` pointing to a source folder.

`host_trace.c` embeds the original `test_core.c` unchanged and observes public
results. It does not compare C structure bytes, padding or compiler-dependent
bit-field layout. Every navigation field and exact steering float bit pattern
is printed, together with direct streaming/batch barcode results and semantic
decoder flags. The same harness compiles against both implementations.

## Results

| Check | Result |
|---|---|
| Existing core suite | PASS: acquisition, calibration, line detection, steering, A-Z Code 39 forward/reverse and simulated navigation |
| Existing reflectance suite | PASS: provisional thresholds, UNKNOWN/WHITE/BLACK hysteresis, both polarities, clamping, invalid inputs and recovery |
| Before/after semantic trace | 53,660 rows, byte-identical |
| Actual bench sampler/reporter wrapper | Identical console diagnostics and counters before/after for not-ready/empty state, startup failure, valid and saturated ADC, read failure retaining prior sample, invalid sensor with INT32_MIN error, timestamp wrap/long gap, and valid recovery |
| Acquisition boundary | All 65,536 possible `uint16_t` ADC callback results checked: 0..4095 preserved; 4096..65535 rejected before packing |
| Diagnostic error | A callback error of -1234567 remains unchanged; it is not narrowed into a boolean or small invented error code |
| Calibration bound | 65,535 samples for each surface, maximum sum 268,365,825; next sample rejected without count/sum mutation; reference and average preserved |
| Configurable timing bounds | Near-2^31 line timings/sample ages and large barcode durations with quiet_modules=50 remain accepted/copied exactly; quiet_modules=10 also preserves maximum accepted run/gap/minimum-narrow214748364 with frame214748365; one-above boundaries and 2^31 durations remain rejected |
| Main packet combinations | 51,456 combinations: 201 signed line errors x 32 event masks x 8 sensor-fault masks |
| Other round trips | Every darkness value 0..1000; UNKNOWN/WHITE/BLACK; all A-Z/none letters, reverse flag and action meanings; timestamp wrap-domain values |
| Floating-point preservation | Exact binary32 steering bits, including non-round decimal values, negative zero and inactive nonfinite diagnostics |
| Known layout | Independently constructed explicit 234-bit expected packet agrees byte-for-byte with the encoder |
| Bench raw-frame packet | All 4096 valid ADC values, all enabled/valid flag combinations and full signed 32-bit driver errors preserved; independently constructed 266-bit packet agrees with the encoder |
| Raw-frame boundaries | Each sensor rejects every raw value 4096..65535 without changing the packet; reserved padding and null inputs rejected without changing decoded output |
| Rejection | Bad ranges/enums, reserved padding, invalid letters and nulls rejected; tested failure paths preserve caller output |
| Hostile bytes | 20,000 deterministic arbitrary navigation packets (1019 accepted) and 20,000 raw frames (10167 accepted); every accepted packet re-encodes identically |
| Memory/undefined behaviour | Original suites and packet tests pass AddressSanitizer and UndefinedBehaviorSanitizer |

Before/after trace SHA-256:
`d2c4464b3313e717e75bba910c4f831ca046e3b79b044dec16bddcb343f8a6b9`.
The generated traces remain in `build/buddy3/before.trace` and `after.trace`.

`bench_trace.c` includes the actual subsystem wrapper. Its fake sleep/delay ends
each task loop after one controlled iteration. Fake mutex calls also assert
that ADC and console operations occur outside the snapshot lock. This checks
the retained bench behaviour without claiming to simulate scheduling,
concurrent preemption, real task stacks or physical ADC timing. The current
wrapper also passes AddressSanitizer/UndefinedBehaviorSanitizer. Its diagnostic
comparison is recorded separately in `before-bench.trace`/`after-bench.trace`.
Both bench traces have SHA-256
`6bb7072e75a471fa344e52989d9f2e8e5e42727888c25fd7a4c0f83a0d58bfcf`.

## Individual structure measurements

`make sizes` uses short enums to match the current Arm ABI; root verification
also compiled `size_report.c` with `arm-none-eabi-gcc` for Cortex-M0+.

| Item | Original | Revised |
|---|---:|---:|
| Decoded validation/calculation navigation view | 72 B | 72 B |
| Copied inter-task navigation format | raw 72 B view | explicit 30 B packet |
| Decoded barcode result | 8 B | 8 B |
| Copied pending-barcode format | raw 8 B result | explicit 5 B packet |
| Copied existing bench raw-frame snapshot | raw 52 B view | explicit 34 B packet |
| Private navigation context | 412 B | 344 B |
| Calibration collector, two surfaces | 24 B | 18 B |
| Classifier state, per sensor | 10 B | 7 B |

Decoded/raw types intentionally remain wide so callers can submit invalid
values for validation and diagnostics retain their original error information.
Keeping the full 12-bit ADC precision is not the same as accepting invalid
4096 as a wrapped zero. Byte rounding and ABI alignment mean logical bit
savings must not be directly summed into linker/static-RAM savings.

Controller-mailbox/full-image RAM and integration tests are recorded by the
integration work; no task-stack high-water or physical performance claim is
made by this structure-size table.

The 34-byte raw-frame codec is for the existing bench-mode shared snapshot.
It does not introduce a new normal-mode raw-frame queue. Its public input
remains wide and validates the ADC range before storing the 12-bit value.
