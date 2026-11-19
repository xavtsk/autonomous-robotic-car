# IMU acquisition and terrain detector contracts

This comments-only batch documents ten definitions and seven public prototypes
in `imu_sensor`, `hump_detection` and `events_motion`. The
[source and compiler receipt](SENSOR_TERRAIN_FUNCTION_CONTRACTS.json) records
the exact files and pre-batch revision. Existing comments, C tokens,
preprocessing directives and function interfaces are unchanged.

The sensor contracts distinguish failed initialization after partial device
writes from failed acquisition after output initialization. An acceleration
sample can succeed without valid magnetic data. Units, vehicle axes, callback
lifetime, bus serialization, null handling and the absence of age validation
are stated explicitly.

The hump contracts describe both pitch integration and right-wheel geometry.
They retain the completion pulse, highest completed peak, rejection effects,
single-sample acceleration hold, level rearming and wrapping timeout arithmetic.
Unknown quality enum values retain their existing path through the algorithm;
the cleanup adds no validation. Optional debug publication follows the same
state-machine step and uses its saved pre-step state.

The motion-classification contract states the existing category priority and
the absence of finite/range validation. It introduces no thresholds or changes
to floating-point comparisons.

Libclang 18.1.1 checked 24 recorded production/host source and header contexts
before and after the edit, including both host `APP_IMU_DEBUG` settings, with
zero documentation warnings or errors. All parameters have attached contract
documentation, and canonical clang-format 18.1.3 leaves the six files unchanged.
These are compiler observations and a bounded source proof, not local C test
execution, target instruction identity or physical validation.

```sh
python tools/compliance/verify_comment_batch.py --manifest docs/compliance/SENSOR_TERRAIN_FUNCTION_CONTRACTS.json --candidate WORKTREE
```

Use the published batch commit for historical verification. This batch does
not close the whole-repository documentation rule; the ledger and compiler
context union retain the remaining work.
