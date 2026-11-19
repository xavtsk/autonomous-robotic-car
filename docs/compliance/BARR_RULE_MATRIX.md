# BARR-C 2018 rule matrix

The canonical ledger is [BARR_RULES.json](BARR_RULES.json). This matrix covers
chapters 1-8, including explicit nested subclauses. Example rows preserve the
standard's structure; they are not additional mandatory requirements. The number
of rows is a coverage count, never a count of compliant requirements.

Baseline for the review: `4ad2cc42595722cedb47102e4343298339040d40`.
The original PDF is the authority. The local extraction used during review is
`build/compliance-review-20261119/barr-c-2018.txt` (not a tracked deliverable).
Owned C/header scope comes from
[inventory_scope.json](../../testing/compliance/inventory_scope.json).
Imported source and externally fixed ABI boundaries must remain explicit.

## Reading status and strength

- **mandatory:** a direct prohibition or requirement.
- **qualified / conditional:** an obligation qualified by applicability,
  reasonable effort, necessity or feasibility; it still needs evidence.
- **preferred:** a stated preference or recommendation; do not change behavior
  merely to force it. **mixed** rows identify both a preference and an obligation.
- **permission / guidance / example:** permitted syntax, explanatory guidance
  or an illustration; do not count these as independently mandatory rules.
- **unreviewed:** no completed source-wide review. **verified:** named evidence
  establishes the applicable rule across the recorded scope/profiles.
- **not_applicable:** requires a concrete scope/applicability reason.
- **pending_deviation:** a specific unresolved conflict, not a waiver.
  **approved_deviation:** requires named project approval, date and rationale.

The original deviation procedure requires documented and approved exceptions.
See [pending exceptions](BARR_EXCEPTIONS.md). No exception in the initial review
is approved, and preserving behavior does not silently approve one.

## Enforcement boundaries

`python tools/compliance/barr_check.py` scans owned source without changing it.
It reports observed violations, review findings and missing evidence separately.
Exit 1 means violations; exit 2 means incomplete review or unavailable evidence.
A clean partial scan does not certify full BARR compliance. The existing
`tools/check_c_style.py` remains an application formatter check only.

The lexical pass excludes strings and comments when recognizing C keywords,
checks filenames/includes/text/comment tokens and identifies missing module
documentation. The AST pass consumes Clang JSON and checks actual function and
variable declarations, public function prefixes, function documentation and
length, floating equality operand types and conditional-depth review candidates.
It does not implement every rule marked `ast-review`; that label identifies the
required enforcement method, not a claim that its implementation is complete.
Compiler/profile coverage, promotions, casts, ownership, ISR classification,
function organization, documentation meaning and other manual rows remain open.

For AST input, pass `--ast-manifest path/to/manifest.json`. The manifest has
`schema_version: 1`, a `source_sha256` mapping for **every owned C/header file**,
and `translation_units` entries with `source`, relative `ast` JSON path,
`profile`, `compiler` and exact `command`. Generate JSON using Clang's
`-Xclang -ast-dump=json -fsyntax-only` under the actual compile definitions.
Use native documentation comments; do not use `-fparse-all-comments` to make
ordinary comments appear documented. The checker rejects stale hashes and
reports missing units/ambiguous include provenance. Conditional profiles and
header-only definitions still require a coverage review. An observed denominator
is never represented as complete function documentation coverage.

## Documentation completion procedure

1. Inventory definitions from all supported translation units and profile
   branches, including private, inline, test and adapter functions.
2. Add Doxygen-compatible module and function comments covering purpose,
   parameters, units/ranges, outputs, mutation, ownership, concurrency and
   timing assumptions as applicable. Document exact retained behavior.
3. Generate Doxygen with warnings for undocumented members/parameters; compare
   its definition inventory with the compiler inventory. A warning-free run
   alone cannot prove all conditional branches were included.
4. Review meaning and complexity coverage under 2.2.a-g; mark 2.2.h verified
   only when all owned definitions and modules are accounted for.

Templates and abbreviation definitions are in [templates](templates/README.md)
and [ABBREVIATIONS.md](ABBREVIATIONS.md). Generate this matrix with
`python tools/compliance/barr_matrix.py --write`; check it with `--check`.

## Rule-by-rule ledger

| Rule | Strength | Enforcement needed | Requirement or explanatory item | Review status |
| --- | --- | --- | --- | --- |
| 1.1.a | mandatory | compiler-review | Use ISO C99-compatible owned C code. | unreviewed |
| 1.1.b | mandatory | compiler-review | Restrict a C++ compiler to the selected ISO C language when compiling C. | unreviewed |
| 1.1.c | qualified | ast-review | Minimize proprietary extensions and localize necessary ones to hardware driver modules. | unreviewed |
| 1.1.d | mandatory | lexical-review | Do not redefine language keywords or language constructs with macros. | unreviewed |
| 1.2.a | mandatory | lexical | Limit source lines to 80 characters. | unreviewed |
| 1.3.a | mandatory | ast-review | Brace all control-statement bodies, including empty bodies. | unreviewed |
| 1.3.b | mandatory | formatter-review | Place opening and closing block braces on separate aligned lines. | unreviewed |
| 1.4.a | mandatory | ast-review | Make expression evaluation order clear without depending on remembered precedence. | unreviewed |
| 1.4.b | mandatory | ast-review | Parenthesize compound operands of logical AND and OR. | unreviewed |
| 1.5.a | preferred | manual | Avoid abbreviations whose meanings are not broadly understood. | unreviewed |
| 1.5.b | mandatory | manual | Maintain a version-controlled project abbreviation table. | unreviewed |
| 1.6.a | mandatory | ast-review | Associate every cast with a comment explaining its valid input range and behavior. | unreviewed |
| 1.7.a | mandatory | lexical | Do not use auto. | unreviewed |
| 1.7.b | mandatory | lexical | Do not use register. | unreviewed |
| 1.7.c | mixed | ast-review | Prefer no goto; any goto must target a later label in the same or enclosing block. | unreviewed |
| 1.7.d | preferred | lexical-review | Prefer avoiding continue. | unreviewed |
| 1.8.a | mandatory | ast-review | Make functions and objects static unless external visibility is necessary. | unreviewed |
| 1.8.b | qualified | ast-review | Use const wherever appropriate. | unreviewed |
| 1.8.b.i | example | manual | Consider const for objects unchanged after initialization. | unreviewed |
| 1.8.b.ii | example | ast-review | Consider const for pointed-to function arguments that are not modified. | unreviewed |
| 1.8.b.iii | example | manual | Consider const for immutable structure or union members, including hardware overlays. | unreviewed |
| 1.8.b.iv | example | manual | Consider typed const values instead of numerical macros where language semantics allow. | unreviewed |
| 1.8.c | qualified | manual | Use volatile wherever appropriate. | unreviewed |
| 1.8.c.i | example | manual | Review objects accessible to interrupt handlers for volatile qualification. | unreviewed |
| 1.8.c.ii | example | manual | Review objects accessible to multiple threads for volatile qualification. | unreviewed |
| 1.8.c.iii | example | manual | Review memory-mapped peripheral pointers for volatile qualification. | unreviewed |
| 1.8.c.iv | example | manual | Review delay-loop counters for volatile qualification. | unreviewed |
| 2.1.a | permission | manual | C++-style single-line comments are permitted. | unreviewed |
| 2.1.b | mandatory | lexical | Do not place nested comment tokens or backslashes inside comment content. | unreviewed |
| 2.1.c | mandatory | manual | Do not retain commented-out code. | unreviewed |
| 2.1.c.i | guidance | manual | Use conditional compilation for temporarily disabled code. | unreviewed |
| 2.1.c.ii | mandatory | manual | Place code existing specifically for increased debug output inside NDEBUG guards. | unreviewed |
| 2.2.a | mandatory | manual | Write comments as clear complete sentences with spelling, grammar and punctuation. | unreviewed |
| 2.2.b | mixed | manual | Place useful step comments before code; follow the described block with a blank line. | unreviewed |
| 2.2.c | guidance | manual | Avoid comments that merely repeat obvious code. | unreviewed |
| 2.2.d | mandatory | manual | Scale documentation detail to code complexity. | unreviewed |
| 2.2.e | mandatory | manual | Cite external algorithm and technical references sufficiently to locate them. | unreviewed |
| 2.2.f | qualified | manual | Version-control necessary diagrams and refer to them from code. | unreviewed |
| 2.2.g | mandatory | manual | Document all assumptions. | unreviewed |
| 2.2.h | mandatory | ast-documentation | Document every module and function for automatic documentation generation. | unreviewed |
| 2.2.i | guidance | manual | Use the standard capitalized issue markers consistently. | unreviewed |
| 2.2.i.i | guidance | manual | Use WARNING to explain maintenance risks. | unreviewed |
| 2.2.i.ii | guidance | manual | Use NOTE to explain rationale and relevant assumptions. | unreviewed |
| 2.2.i.iii | guidance | manual | Use TODO to explain unfinished work. | unreviewed |
| 3.1.a | mandatory | formatter-review | Use one space after control and return keywords when followed by text. | unreviewed |
| 3.1.b | mandatory | formatter-review | Space assignment operators on both sides. | unreviewed |
| 3.1.c | mandatory | formatter-review | Space binary operators on both sides. | unreviewed |
| 3.1.d | mandatory | formatter-review | Keep unary operators adjacent to their operands. | unreviewed |
| 3.1.e | mandatory | formatter-review | Space pointer operators in declarations; keep unary uses adjacent to operands. | unreviewed |
| 3.1.f | mandatory | formatter-review | Space both ternary operators on both sides. | unreviewed |
| 3.1.g | mandatory | formatter-review | Do not surround member-access operators with spaces. | unreviewed |
| 3.1.h | mandatory | formatter-review | Do not add spaces around array subscripts except where another rule requires. | unreviewed |
| 3.1.i | mandatory | formatter-review | Do not pad the inside of parentheses. | unreviewed |
| 3.1.j | mandatory | formatter-review | Keep calls tight and place one space before declaration parameter parentheses. | unreviewed |
| 3.1.k | mandatory | formatter-review | Follow parameter commas with one space unless at end of line. | unreviewed |
| 3.1.l | mandatory | formatter-review | Follow for-clause semicolons with one space. | unreviewed |
| 3.1.m | mandatory | formatter-review | Do not put a space immediately before statement semicolons. | unreviewed |
| 3.2.a | mandatory | formatter-review | Align names in consecutive variable declarations. | unreviewed |
| 3.2.b | mandatory | formatter-review | Align structure and union member names. | unreviewed |
| 3.2.c | mandatory | formatter-review | Align consecutive assignment operators. | unreviewed |
| 3.2.d | mandatory | lexical | Place every preprocessor hash at the start of its line. | unreviewed |
| 3.3.a | mandatory | ast-review | Place no more than one statement on a line. | unreviewed |
| 3.3.b | mandatory | manual | Separate natural code blocks with blank lines. | unreviewed |
| 3.3.c | mandatory | lexical | End each C source file with an end-of-file comment and a blank line. | unreviewed |
| 3.4.a | preferred | formatter-review | Prefer indentation in multiples of four columns. | unreviewed |
| 3.4.b | mandatory | formatter-review | Align switch case labels and indent case contents one level. | unreviewed |
| 3.4.c | guidance | manual | Indent continued lines for readability. | unreviewed |
| 3.5.a | mandatory | lexical | Do not use literal tab characters in source files. | unreviewed |
| 3.6.a | qualified | lexical | Use LF line endings whenever possible. | unreviewed |
| 3.6.b | mandatory | lexical | Permit no other control character except form feed. | unreviewed |
| 4.1.a | mandatory | lexical | Use lowercase letters, digits and underscores in module filenames. | unreviewed |
| 4.1.b | mandatory | lexical | Make module basenames unique in their first eight characters and use .c/.h suffixes. | unreviewed |
| 4.1.c | mandatory | lexical | Do not reuse a C or C++ standard-library header filename. | unreviewed |
| 4.1.d | mandatory | ast-review | Include main in the filename of a module defining main. | unreviewed |
| 4.2.a | mandatory | lexical | Give each C source exactly one same-basename header. | unreviewed |
| 4.2.b | mandatory | lexical-review | Protect every header from repeated inclusion. | unreviewed |
| 4.2.c | mandatory | manual | Expose only the interface necessary to other modules. | unreviewed |
| 4.2.c.i | preferred | ast-review | Prefer no extern object declarations in headers. | unreviewed |
| 4.2.c.ii | mandatory | ast-review | Allocate no object storage in headers. | unreviewed |
| 4.2.d | mandatory | manual | Do not include private headers from public headers. | unreviewed |
| 4.3.a | mandatory | manual | Limit each module to behaviors belonging to one entity. | unreviewed |
| 4.3.b | mandatory | ast-review | Order source sections: comment, includes, types/constants/macros, static data, private prototypes, public bodies, private bodies. | unreviewed |
| 4.3.c | mandatory | lexical | Include the corresponding same-basename header in each C source. | unreviewed |
| 4.3.d | mandatory | lexical | Do not use absolute include paths. | unreviewed |
| 4.3.e | mandatory | compiler-review | Remove unused includes after checking every supported profile. | unreviewed |
| 4.3.f | mandatory | lexical | Do not include C source files. | unreviewed |
| 4.4.a | mandatory | manual | Maintain project source and header templates. | unreviewed |
| 5.1.a | mandatory | ast-review | Use lowercase type names with internal underscores ending in _t. | unreviewed |
| 5.1.b | mandatory | ast-review | Name new structures, unions and enumerations with typedefs. | unreviewed |
| 5.1.c | mandatory | ast-review | Prefix public type names with the module name and underscore. | unreviewed |
| 5.2.a | qualified | ast-review | Use fixed-width integers whenever the number of bits or bytes matters. | unreviewed |
| 5.2.b | mandatory | lexical | Do not use the short or long keywords in owned declarations or expressions. | unreviewed |
| 5.2.c | mandatory | ast-review | Reserve char for strings and character-string operations. | unreviewed |
| 5.3.a | mandatory | ast-review | Do not define signed integer bit-fields. | unreviewed |
| 5.3.b | mandatory | ast-review | Do not manipulate signed integer data with bitwise operators. | unreviewed |
| 5.3.c | mandatory | ast-review | Do not combine signed and unsigned integers in expressions or comparisons. | unreviewed |
| 5.4.a | preferred | manual | Avoid floating-point data where its precision and behavior are not required. | unreviewed |
| 5.4.b | conditional | manual | Apply the floating-point rules wherever floating-point calculation is necessary. | unreviewed |
| 5.4.b.i | mandatory | ast-review | Use float32_t, float64_t and float128_t names for necessary floating-point data. | unreviewed |
| 5.4.b.ii | mandatory | ast-review | Suffix single-precision constants with f. | unreviewed |
| 5.4.b.iii | conditional | compiler-review | Verify adequate double precision whenever calculations depend on it. | unreviewed |
| 5.4.b.iv | mandatory | ast-review | Do not compare floating-point values for equality or inequality. | unreviewed |
| 5.4.b.v | mandatory | manual | Use isfinite to check calculations for infinity and NaN. | unreviewed |
| 5.5.a | mandatory | compiler-review | Verify padding and layout of structures or unions crossing hardware or processor boundaries. | unreviewed |
| 5.5.b | mandatory | compiler-review | Verify intended bit-field ordering. | unreviewed |
| 5.6.a | mandatory | ast-review | Declare Boolean variables with bool. | unreviewed |
| 5.6.b | mandatory | ast-review | Convert non-Boolean values with relational operations rather than casts. | unreviewed |
| 6.1.a | mandatory | ast-review | Do not name procedures with C or C++ language keywords. | unreviewed |
| 6.1.b | mandatory | ast-review | Do not reuse C standard-library function names. | unreviewed |
| 6.1.c | mandatory | ast | Do not begin procedure names with an underscore. | unreviewed |
| 6.1.d | mandatory | ast | Limit procedure names to 31 characters. | unreviewed |
| 6.1.e | mandatory | ast | Do not use uppercase letters in function names. | unreviewed |
| 6.1.f | mandatory | lexical | Do not use lowercase letters in macro names. | unreviewed |
| 6.1.g | mandatory | manual | Separate words in procedure names with underscores. | unreviewed |
| 6.1.h | mixed | manual | Use descriptive procedure names; noun-verb ordering is recommended. | unreviewed |
| 6.1.i | mandatory | ast | Prefix public function names with their module name and underscore. | unreviewed |
| 6.2.a | qualified | ast-review | Make reasonable efforts to limit functions to one printed page or 100 lines. | unreviewed |
| 6.2.b | qualified | manual | Start functions at the top of a printed page where practicable. | unreviewed |
| 6.2.c | preferred | ast-review | Prefer a single return exit at the bottom of a function. | unreviewed |
| 6.2.d | mandatory | ast-review | Declare every public function in its module header. | unreviewed |
| 6.2.e | mandatory | ast-review | Declare private functions static. | unreviewed |
| 6.2.f | mandatory | ast-review | Explicitly declare and meaningfully name all parameters. | unreviewed |
| 6.3.a | qualified | manual | Replace parameterized macros with functions when functions can accomplish the same behavior. | unreviewed |
| 6.3.b | conditional | manual | Apply the macro safeguards to remaining parameterized macros. | unreviewed |
| 6.3.b.i | mandatory | manual | Parenthesize the entire macro body. | unreviewed |
| 6.3.b.ii | mandatory | manual | Parenthesize every use of a macro parameter. | unreviewed |
| 6.3.b.iii | mandatory | manual | Use each macro parameter at most once. | unreviewed |
| 6.3.b.iv | mandatory | lexical-review | Do not place control transfers in macros. | unreviewed |
| 6.4.a | mandatory | manual | End thread entry-point names with _thread, _task or _process. | unreviewed |
| 6.5.a | mandatory | manual | Declare actual ISRs through the compiler and platform's required mechanism. | unreviewed |
| 6.5.b | mandatory | manual | End actual ISR names with _isr. | unreviewed |
| 6.5.c | mandatory | manual | Keep ISRs static or at the end of their driver as the platform permits. | unreviewed |
| 6.5.d | mandatory | manual | Provide default handling for unexpected or unhandled interrupt sources. | unreviewed |
| 7.1.a | mandatory | ast-review | Do not use language keywords as variable names. | unreviewed |
| 7.1.b | mandatory | ast-review | Do not reuse standard-library variable names. | unreviewed |
| 7.1.c | mandatory | ast | Do not begin variable names with an underscore. | unreviewed |
| 7.1.d | mandatory | ast | Limit variable names to 31 characters. | unreviewed |
| 7.1.e | mandatory | ast | Use at least three characters for all variable names, including loop counters. | unreviewed |
| 7.1.f | mandatory | ast | Do not use uppercase letters in variable names. | unreviewed |
| 7.1.g | mandatory | manual | Do not encode duplicated sizes or bit counts in variable names. | unreviewed |
| 7.1.h | mandatory | manual | Separate words in variable names with underscores. | unreviewed |
| 7.1.i | mandatory | manual | Make variable names descriptive. | unreviewed |
| 7.1.j | mandatory | ast-review | Prefix global variable names with g. | unreviewed |
| 7.1.k | mandatory | ast-review | Prefix pointer variable names with p. | unreviewed |
| 7.1.l | mandatory | ast-review | Prefix pointer-to-pointer names with pp. | unreviewed |
| 7.1.m | mandatory | ast-review | Prefix Boolean-information names with b and phrase them as questions. | unreviewed |
| 7.1.n | mandatory | manual | Prefix non-pointer object handles with h. | unreviewed |
| 7.1.o | mandatory | ast-review | Order combined prefixes as global, pointer depth, then Boolean or handle. | unreviewed |
| 7.2.a | mandatory | compiler-review | Initialize every variable before use. | unreviewed |
| 7.2.b | preferred | manual | Prefer declaring locals near their first use. | unreviewed |
| 7.2.c | mandatory | ast-review | Group file/project globals near the top of a source file. | unreviewed |
| 7.2.d | mandatory | ast-review | Initialize pointers without an initial address to NULL. | unreviewed |
| 8.1.a | mandatory | ast-review | Use a separate declaration for every variable rather than comma declarations. | unreviewed |
| 8.2.a | preferred | manual | Prefer the shortest conditional branch first when it preserves evaluation behavior. | unreviewed |
| 8.2.b | mandatory | ast-review | Limit nested if/else statements to two levels. | unreviewed |
| 8.2.c | mandatory | ast-review | Do not assign within if or else-if tests. | unreviewed |
| 8.2.d | mandatory | ast-review | Finish every else-if chain with an else. | unreviewed |
| 8.3.a | mandatory | formatter-review | Align a case's break with its case label. | unreviewed |
| 8.3.b | mandatory | ast-review | Give every switch a default block. | unreviewed |
| 8.3.c | mandatory | manual | Document deliberate case fallthrough. | unreviewed |
| 8.4.a | mandatory | ast-review | Use named loop bounds instead of magic numbers. | unreviewed |
| 8.4.b | mandatory | ast-review | Limit loop-control assignments to initializing and updating its loop counter. | unreviewed |
| 8.4.c | mandatory | ast-review | Spell infinite loops as for (;;). | unreviewed |
| 8.4.d | mandatory | ast-review | Use braces and an explanatory comment for every empty loop body. | unreviewed |
| 8.5.a | mandatory | ast-review | Apply the restricted forward-target goto rule. | unreviewed |
| 8.5.b | mandatory | ast-review | Do not call abort, exit, setjmp or longjmp. | unreviewed |
| 8.6.a | mandatory | ast-review | Put the constant on the left of a variable-to-constant equality test. | unreviewed |
