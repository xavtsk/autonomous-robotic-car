# Estimator module filename alignment

`estimate_motion.c/.h` become `motion_estimation.c/.h`, matching the ten existing
public `motion_estimation_*` functions under BARR 6.1.i. Their API names,
parameter types, data declarations and control operations remain unchanged.
The `motion_e` first-eight-character prefix has no competing owned module.

The [manifest](MOTION_MODULE_RENAME.json) binds 16 files to exact pre-rename
revision `ba4f3391261e47ede38649b231324269d86c60bb`. Nine C/header files either
move or update filename references. After reversing the exact header include,
their preprocessing tokens match. Existing source comments change only filename
references. The remaining files update build/test paths and the module README.

The target Makefile discovers source files alphabetically. It therefore retains
`estimate_motion.c` as a historical sorting key, then maps back to the real new
path before compiling. This preserves the estimator's old object position instead
of moving it after `motion.c`. The verifier checks the resulting logical order;
actual target-image equality still requires the matched Linux cross-build.

```sh
python tools/compliance/verify_module_rename.py
python -m unittest testing.compliance.test_module_rename testing.compliance.test_ci_runner
```

Once published, pin `--candidate <rename-commit>` for historical verification.
Target bytes must be compared with `ba4f339`, which already contains the separately
committed motion-storage change, rather than assuming the older image hash applies.
At preparation time the inversion checks and 11 focused tests pass; the target
and Linux execution checks for this filename batch remain pending.

The shared runners select exactly one complete old or new C/header pair for each
revision and reject mixed, missing or duplicate pairs. Old baseline sources keep
their original files. Published alias, formatting, function-documentation and
lexical-review manifests likewise retain their historical paths and revisions;
rewriting them would invalidate their evidence. Generated current field inventory
paths are refreshed separately after concurrent source batches are frozen.
