# Baseline failures discovered by Linux execution

Comparison baseline: `4ad2cc42595722cedb47102e4343298339040d40`.
Initial infrastructure revision: `72ddba71a2f25573a05fd77e0cf5af565b040710`.
Run: [44565047810](https://github.com/xavtsk/autonomous-robotic-car/actions/runs/44565047810).
Production sources were unchanged in this run.

## Recorded barcode replay expectation

The unmodified `testing/subsystem3/test_barcode_replay.py` fails for
`20261103-152854-successful-z.txt`, midpoint timing, 2000 ms synthetic leading
white and no synthetic trailing white. Both revisions report:

```text
decoded=0 rejected=0 timing=0 overflow=0 state=2 count=987
```

`S3_REC_QUIET` is state 2. The test expects a published Z or, for this exact
fixture variant, an unfinished worker in state 3 (`S3_REC_PROCESSING`).
The candidate has the same production source as the baseline, so this is an
existing discrepancy, not a cleanup regression. A quiet-state candidate still
has to satisfy the measured trailing-quiet and corrected-frame-time conditions
before publication. Merely treating state 2 as success would weaken the test.

The original fixture, sample data, expectations and production decision logic
remain intact. `tools/compliance/replay_audit.py` separately accumulates every
original predicate result so this first failure does not conceal later cases.
Its false predicates remain failures; matching baseline/candidate outputs are
reported separately from passing fixture expectations. Additional per-update
trace evidence will locate the unfulfilled condition and enumerate all failures.
Barcode algorithm changes remain outside this behavior-preserving cleanup.

## Test-runner infrastructure

The original laptop codec test assumes `build/buddy1/host_test`, while the new
runner puts each binary in a unique evidence directory. This is a test-runner
path mismatch. The repair must select the actual isolated binary and leave all
codec assertions intact. The baseline checkout needs a disposable compatibility
path rather than a source edit.

The dashboard suite also reports an optional local capture that is absent from
a clean clone. Optional-data absence must be named as unavailable evidence;
missing required dependencies and all other skips remain failures.

## Evidence already obtained

The initial differential job matched 487 debug-codec boundary/output-mutation
rows and 60,007 navigation/barcode semantic rows exactly. These results apply
only to the exercised inputs. The full replay gate remains failed and must not
be summarized as a passing baseline. The raw Linux logs are retained in the
linked Actions run and locally under `build/compliance-ci/44565047810/`.
