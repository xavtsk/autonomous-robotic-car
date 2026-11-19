# Arm type-alias optimization sensitivity

The C99 alias batch `c37570c` preserves source expressions and actual C types.
Linux run [44575110705](https://github.com/xavtsk/autonomous-robotic-car/actions/runs/44575110705)
also matches all exercised baseline semantic traces and packet bytes, with the
same two pre-existing replay expectation failures. This is not proof of identical
Arm instructions or physical timing.

The matched Arm 14.3.1 production builds exposed a different implementation of
`terrain_monitor_process`: 4,316 bytes before the alias batch, 4,112 afterward.
Its compiler stack estimate changed from 200 to 192 bytes. The whole-image code
size fell by 200 bytes including layout effects, while static RAM was unchanged.
The same fixed `4ad2cc4` baseline image was byte-identical across both runs, ruling
out a different baseline build or toolchain drift as the explanation.

Isolated builds reproduce the difference. After reversing only typedef spellings
and normalizing compiler-generated declaration IDs, early GCC intermediate code
and control flow are identical. Optimization later diverges. This demonstrates a
compiler optimization sensitivity; it does not establish a compiler defect or a
functional failure.

The difference is isolated to the local acceleration norm `value_n`. Retaining
only that local as primitive `float` restores **every allocated ELF object
section and its normalized relocations exactly**. `dynamic_g` and all other
floating-point declarations keep the aliases. Separating the two alias
declarations, adding an initialization, moving the declaration to its existing
assignment, and adding const at that assignment did not restore the bytes.

| Variant | Function bytes | Function code SHA-256 |
| --- | ---: | --- |
| Before aliases | 4316 | `12e2f65531d44d9a7245805d5481de8da99656a2ec817a8c72f74c2a4125e74c` |
| All aliases | 4112 | `ec1569e8af8480b6923ec846a577f9a89efd5cc8cdb9079ca3944ad014c371a0` |
| Only `value_n` retains primitive spelling | 4316 | `12e2f65531d44d9a7245805d5481de8da99656a2ec817a8c72f74c2a4125e74c` |

The instruction/relocation listing hash for both the original and repaired
variant is `00a2c7bc85315a940406725815f3cecd53746d3c25850969e9e81523657cdfba`.
The complete comparison is retained in [the checked object evidence](alias-target-object.json).
Detailed compiler dumps and build commands remain locally under
`build/alias-investigation/`; no generated application executable was run on
Windows.

The subsequent matched [Linux run 44579354977](https://github.com/xavtsk/autonomous-robotic-car/actions/runs/44579354977)
confirms the **complete repaired production flash payload is byte-identical**
to the pre-alias `2e44326` candidate. Both have SHA-256
`f09ff492fe30fdfb14a8bc259b0f7fbc386fee81290cbff707580422040ac2f8`,
465,980 text bytes and 129,568 static RAM bytes. The fixed `4ad2cc4` baseline
payload also matches across both runs. See the
[matched image record](alias-repaired-target.json) for exact revisions, artifact
IDs, public configuration hash, toolchain, SDK and sizes. This comparison is
between otherwise identical public-example builds; it is not the private
hotspot-configured rollback UF2's hash.

The repaired revision also matches all exercised semantic traces, including
1,782 production-layout terrain observations and 1,916 debug-layout observations.
Its CI run remains failed for the two unchanged barcode fixture expectations
and the explicitly incomplete standards audits. Byte identity does not approve
the exception or establish new hardware validation.

This single primitive spelling is **BARR-P12, pending approval** under rule
5.4.b.i. Its precision is still the same checked binary32 type. It protects the
requested executable preservation while avoiding a premature claim of full
BARR compliance. No approver or approval is inferred. Retaining the all-alias
compiler output would instead require its own timing/physical evidence before
deployment. The standard's named types are supplied by project typedefs; they
are not ordinary built-in C99 type names.
