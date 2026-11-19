# Imported template boundary correction

The current ownership audit found 15 files that earlier cleanup scope lists
mistakenly treated as team-owned adapters or helpers. Thirteen have identical
Git blobs in upstream template revision `c3b09544cb1fd6dac6ad4b8f523be0af59b44482`,
adoption revision `437f46eb91b3dab0ad3febe5ebfab235abe7ce04` and the fixed
`4ad2cc4` baseline. The other two (`test_report.h` and the public Wi-Fi example)
have only baseline formatting differences and identical preprocessing tokens.

The [complete restoration receipt](evidence/RESTORED_TEMPLATE_BOUNDARIES.json)
records all paths, blobs and hashes. Each file is restored to its fixed-baseline
text, including formatting that already existed in that baseline. The
preprocessing fingerprints also match the preceding cleanup revision `3fda4d5`.
Imported files are pinned in the import manifest; subsequent edits fail the
inventory integrity check. This is an ownership correction, not a compliance
exception or permission to omit team-written code.

The affected files provide unchanged lwIP platform declarations/options, USB
console/TinyUSB implementations, CYW43 compatibility/kernel interfaces and two
template helper headers. The modified `mqtt_utk.c`, `mqtt_utk.h` and
`cyw43_utk.c` remain in scope, along with project retry policy and application
integration. Their changes from the imported template are retained.

The historical 239-file formatting receipt still describes what was done at
that revision. It must not be read as proof that all 239 files were team-owned;
the present provenance evidence corrects that claim. Similarly, the historical
`sys_arch.h` compiler-resolution probe remains factual, but supplies no reason
to edit its imported implementation. Current header/naming work excludes those
unchanged imports. Historical reports and original supplied documents remain
unchanged.

This restoration changes no C preprocessing operations. Exact whole-image
comparison remains a separate target-build check; compiler source locations and
debug metadata may change when the baseline text is restored.
