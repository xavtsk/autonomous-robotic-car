# Generated source reference and documentation evidence

The Linux `documentation` job generates HTML and XML from a disposable copy of
every owned C/header file. Its artifact contains `reference/html/index.html`,
XML, the exact Doxyfile, complete warning log, generation log and source hashes.
The supplied standard and assignment documents are not inputs to this generator
and are unchanged.

The tool is Doxygen 1.9.8, using the exact Ubuntu package
`1.9.8+ds-2ubuntu0.1` in [the tool lock](../../tools/compliance/toolchain.lock.json).
This package version is listed by [Ubuntu's package archive](https://packages.ubuntu.com/en/noble/amd64/doxygen).
Private and static functions are included. `EXTRACT_ALL` remains off, since
enabling it disables missing-documentation warnings, as explained in
[Doxygen's configuration reference](https://www.doxygen.nl/manual/config.html).
All warnings are retained and keep this evidence gate incomplete. Diagnostics
written to `generation.log` also block completion: configuration warnings can
appear before Doxygen opens its configured warning file. With `QUIET = YES`,
any nonblank generator output requires review.

Mixed imported files are masked using the same ownership selectors as the
compiler inventory; their original line positions are retained. Imported files
and local Wi-Fi secrets are not copied. Source and staged bytes are hashed before
and after generation. The command refuses an existing output directory.

```sh
python tools/compliance/build_docs.py \
  --function-report <current-function-inventory.json> \
  --output <new-documentation-output-directory>
```

The compiler report is generated separately by `barr_functions.py` using the
current target commands and recorded host/profile contexts. Its owned-file set
and source-location hashes must match the generator's inputs exactly. Full
normalized source-text hashes also bind comment prose; a same-line comment edit
cannot reuse an old documentation result. The compiler collector checks these
hashes before and after observation. Functions
from error-free compiler observations are compared against XML definition names,
files and body locations. Recovery observations, missing definitions and
undocumented comments remain separate counts. Missing/stale reports fail closed.

Doxygen preprocessing is disabled so alternative source branches can appear in
the reference. This is deliberately a documentation view, not a compilation
profile or evidence that a branch was built. The separate conditional-coverage
gate still applies. Macros or alternate declarations that Doxygen cannot resolve
remain visible as warnings or missing compiler-observed sites.

The generator reports **INCOMPLETE** while any warning, missing definition,
recovery observation, undocumented function or incomplete compiler coverage
remains. It does not certify the meaning of a comment, infer that a documented
signature has a complete contract, or approve an exception. The HTML is reference
material; the rule ledger and reviewed contracts remain the compliance evidence.

The first Linux generation completed at revision `55af85c`, in
[run 44604115125](https://github.com/xavtsk/autonomous-robotic-car/actions/runs/44604115125).
Its source hashes were unchanged. The result remains **INCOMPLETE**: the warning
file contains 4,101 lines, and the supplied compiler report contains 1,708
definition variants, including 980 recovery observations and 1,512 undocumented
variants. These are observations, not counts of distinct missing contracts.
The target-only compiler context used for that report is being corrected; its
recovery counts must not be compared with the broader local field inventory as
if they had identical inputs. The historical artifact is retained.

That run also reported an obsolete `HTML_TIMESTAMP` setting in `generation.log`.
The setting has been removed and both diagnostic streams now gate completion.
Nine focused Python tests cover binding, invalid XML, configuration diagnostics,
quiet successful generation and source preservation after failed generation.
These checks use mocked generation; no local Doxygen execution is claimed.
