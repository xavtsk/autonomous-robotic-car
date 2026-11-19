# Navigation and telemetry codec contracts

This comments-only batch documents all 28 function definitions and 14 public
prototypes in `subsystem3/navigation_codec.c/.h` and
`subsystem1/telemetry_codec.c/.h`. Public headers carry complete parameter,
return, mutation, ownership and serialization contracts. Implementation
comments link to those contracts; private bit/field helpers document their
cursor, capacity and validation preconditions directly.

The contracts describe existing behavior. Navigation uses 479 meaningful bits
in 60 bytes; barcode and raw frames use 38 bits in five bytes and 266 bits in
34 bytes. Floating-point diagnostic representations are copied without
scaling or finite-value validation. All public navigation codecs validate a
local result before replacing the caller's output.

Telemetry motion rounds to 2 mm/s steps, with halfway values rounded away
from zero. Other telemetry fields retain their stated integer units. Snapshot
validity controls which sections are serialized, while single-section topics
encode their fields regardless of the top-level validity flag. The current
version-3 snapshot includes pose; an added note clarifies this while preserving
the older live-only comments in their original order.

`buddy1_encode` can return failure after clearing and partially filling its
packet. Its null/topic/capacity rejection happens before writes. In contrast,
`buddy1_decode` and map decoding leave caller output unchanged on failure.
These distinctions follow the implementation and reviewed callers/tests;
they do not add validation, locking or any new runtime behavior.

The [bound evidence](CODEC_FUNCTION_CONTRACTS.json) uses pre-batch revision
`07affd0d70f460786a1e81a746ab150ae2d0d481`. Verification confirms identical
preprocessing tokens/directive semantics, identical function interfaces and
all original comments retained byte-for-byte in order. The canonical
clang-format 18.1.3 wrapper leaves all four resulting files unchanged.

Libclang 18.1.1 reports zero errors or warnings with `-Wdocumentation` and
`-Wdocumentation-unknown-command` for the four files before and after the
batch. Arguments come from each source's actual Linux production command in
run `44595295824`, artifact `14566022142`, with explicit local repository/SDK
path mapping and Arm GCC headers. Headers are checked in their corresponding
source context. Public parameter coverage is checked against the function's
own or canonical declaration comment.

```sh
python tools/compliance/verify_comment_batch.py --manifest docs/compliance/CODEC_FUNCTION_CONTRACTS.json --candidate WORKTREE
```

After publication, replace `WORKTREE` with this batch's immutable commit for
historical verification. The command recomputes the comment/token proof; it
does not repeat compiler observations or execute firmware. Doxygen generation
and the repository-wide documentation coverage gate are separate checks.
No compiled C program was executed locally. This evidence covers only these
four files and does not close the whole BARR documentation rule.
