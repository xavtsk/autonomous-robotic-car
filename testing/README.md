# Test evidence

The team completed the physical demonstration and dismantled the car. The
post-demo submission was checked with host tests, recorded barcode waveforms,
course simulation and matched Arm builds. The [mission results](../docs/CURRENT_STATUS.md#mission-requirements)
identify the five simulated bypass misses. The [barcode endpoint review](subsystem3/fixtures/barcode/README.md#submission-review-of-two-recorded-endpoints)
explains the two corrected expectations. Run `python3 tools/verify_submission.py`
from the repository root on Linux. [Prerequisites and commands](../docs/BUILD_AND_TEST.md)

| Area | Evidence |
| --- | --- |
| Buddy 1 — telemetry and commands | [Host checks and measured data sizes](subsystem1/EVIDENCE.md) |
| Buddy 2 — motors and encoders | [Test log](subsystem2/TEST_LOG.md) and [saved calibration, PID and movement records](subsystem2/logs/) |
| Buddy 3 — line sensing and barcodes | [Mounted sensor calibration](subsystem3/IR_CALIBRATION_2026-10-27.md), [recorded waveforms](subsystem3/fixtures/barcode/README.md) and [host tests](subsystem3/README.md#repeatable-host-tests) |
| Buddy 4 — terrain sensing | [IMU and controller tests](subsystem4/TEST_LOG.md) |
| Buddy 5 — obstacle avoidance | [Scan, planning and bypass tests](subsystem5/TEST_LOG.md) |
| Shared controller | [Integration cases and dated results](integration/TEST_LOG.md) |
| Whole-car demo | [Scope of the system record](system/TEST_LOG.md) |

Each dated run describes the firmware and setup used at the time. A host test
can verify a controller decision or detect a change in decoding behaviour.
A hardware accuracy or timing result requires a measurement on the corresponding
image. Post-demo measurements of physical timing and hotspot recovery are
unavailable. The [hardware comparison procedure](../docs/PHYSICAL_VALIDATION.md)
records how those measurements were defined.
