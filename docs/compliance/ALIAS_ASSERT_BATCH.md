# C99 assertion and floating-type alias batch

Pre-batch revision: `2e44326dabcb862d4028aa52676cbc507bf0924e`.
This is a bounded mechanical batch after the debug-builder change, not a new
control algorithm or a claim of full C99/BARR compliance.

[ALIAS_ASSERT_MANIFEST.json](ALIAS_ASSERT_MANIFEST.json) records 51 existing
application files, 447 replacements of actual `float`/`double` keyword tokens,
17 preserved assertion conditions, required includes and before/after hashes.
The root-owned `s3_integration.c` contributes 11 replacements. The debug codec
source/header contain no floating-point keywords and are unchanged by this batch.
String literals, comments, system-header names and floating constants are retained.

Two headers supply the mechanical replacements:

- `app_types.h` defines `float32_t` as the existing `float` type and `float64_t`
  as the existing `double` type. Compile-time checks establish eight-bit bytes
  and the existing binary32/binary64 size, radix, mantissa and exponent limits.
  No reduced-precision representation, arithmetic regrouping or conversion is
  introduced. There is no unsupported `float128_t` alias.
- `build_assert.h` replaces C11 `_Static_assert` declarations with C99 typedef
  declarations containing a checked bit-field width. C requires that width to
  be an integer constant expression; false conditions produce a negative width.
  The declarations allocate no objects or runtime work. Each original condition
  is preserved and its diagnostic text is retained in a nearby comment/manifest.
  A function cannot implement this compile-time declaration, which is why the
  parameterized macro is necessary.

The helper's C99 compile-only probes used Arm GCC with
`-std=c99 -pedantic-errors -Wall -Wextra -Werror -fsyntax-only`. Exact alias/pointer
compatibility and a true condition compiled; false and nonconstant conditions
were correctly rejected. The probe sources and full logs are local under
`build/compliance-review-20261120/`. No generated test executable was run.

The independent inverse-token verifier reads the original files from Git,
removes only the recorded includes, restores original assertion declarations,
and reverses only the declared type aliases. It reports identical original token
streams for all 51 files. It also binds the new header semantics by token hash.

```sh
python tools/compliance/verify_alias_batch.py
python -m unittest discover -s testing/compliance -p 'test_alias*.py'
```

Six verifier regression tests cover accepted substitutions, untouched comments/
literals, accidental system-header renaming, changed numeric literals, weakened
conditions and missing assertions. After later batches, pass the recorded alias
batch commit using `--candidate <commit>`; an unrelated later source change must
not silently be counted as part of this mechanical batch.

This evidence establishes the declared source transformation and compile-time
helper behavior. Full local Make stopped before compilation because MSYS could
not create its Windows shared-object namespace (`0xC0000022`). The Linux runner
must therefore provide the exact pre/post target build, loadable-image/RAM
comparison and executed regressions. Do not report these pending checks as done.
Existing barcode fixture expectation failures remain separate from equality of
baseline/candidate outputs.

The shared imported build flags are unchanged. Closing BARR 1.1.a still requires
the owned-source/profile C99 audit and explicit imported-interface boundaries.
Float equality/finite checks, zero behavior, rounding, saturation, PID values,
sensor thresholds, protocol fields and synchronization are unchanged here.
