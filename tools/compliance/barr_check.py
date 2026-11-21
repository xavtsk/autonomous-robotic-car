#!/usr/bin/env python3
"""Read-only, partial BARR-C audit. Coverage is never a compliance certificate.

Lexical checks inspect every owned C/header file from inventory_scope.json.
Optional Clang JSON ASTs add type-aware findings and function documentation
coverage. Missing/stale ASTs and unreviewed rules make the result INCOMPLETE.
Exit 1: observed violations; exit 2: incomplete/error; exit 0: fully reviewed.
See docs/compliance/BARR_RULE_MATRIX.md for the precise coverage boundary.
"""
from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
import hashlib
import json
from pathlib import Path
import re
import sys

ROOT = Path(__file__).resolve().parents[2]
LEDGER = ROOT / "docs/compliance/BARR_RULES.json"
SCOPE = ROOT / "testing/compliance/inventory_scope.json"
TOKEN = re.compile(
    r'(?P<comment>/\*[\s\S]*?\*/|//[^\n]*)|'
    r'(?P<string>(?:u8|[LuU])?"(?:\\[\s\S]|[^"\\])*"|'
    r"(?:[LuU])?'(?:\\[\s\S]|[^'\\])*')|"
    r'(?P<identifier>[A-Za-z_]\w*)|(?P<other>.)', re.DOTALL)
GROUPED_POINTER_DECLARATOR = re.compile(
    r"\b(?:void|bool|char|short|int|long|float|double|[A-Za-z_]\w*_t)\b"
    r"(?P<space>[ \t]+)(?P<group>\([ \t]*\*[ \t]*)"
    r"[A-Za-z_]\w*[ \t]*\)(?=[ \t]*\()"
)
STANDARD_HEADERS = {
    "assert", "complex", "ctype", "errno", "fenv", "float", "inttypes",
    "iso646", "limits", "locale", "math", "setjmp", "signal", "stdarg",
    "stdbool", "stddef", "stdint", "stdio", "stdlib", "string", "tgmath",
    "time", "wchar", "wctype", "algorithm", "array", "atomic", "bitset",
    "chrono", "deque", "exception", "fstream", "functional", "iostream",
    "iterator", "list", "map", "memory", "mutex", "new", "numeric",
    "queue", "random", "regex", "set", "sstream", "stack", "stdexcept",
    "thread", "tuple", "type_traits", "unordered_map", "unordered_set",
    "utility", "valarray", "vector",
}
KEYWORDS = set("auto break case char const continue default do double else enum "
               "extern float for goto if inline int long register restrict "
               "return short signed sizeof static struct switch typedef union "
               "unsigned void volatile while _Bool _Complex _Imaginary".split())
RESERVED_IDENTIFIERS = KEYWORDS | set(
    "alignas alignof and and_eq asm "
    "bitand bitor bool catch char8_t char16_t char32_t class compl concept consteval "
    "constexpr constinit const_cast co_await co_return co_yield decltype delete "
    "dynamic_cast explicit export false friend interrupt mutable namespace "
    "new noexcept not not_eq nullptr operator or or_eq private protected public "
    "reinterpret_cast requires static_assert static_cast template this thread_local "
    "throw true try typeid typename using virtual wchar_t xor xor_eq typeof typeof_unqual "
    "_Alignas _Alignof _Atomic _BitInt _Decimal32 _Decimal64 _Decimal128 _Generic "
    "_Noreturn _Static_assert _Thread_local".split())
STANDARD_OBJECT_NAMES = {"errno", "stdin", "stdout", "stderr"}
STRENGTHS = {"mandatory", "qualified", "preferred", "guidance", "permission",
             "example", "mixed", "conditional"}
STATUSES = {"unreviewed", "verified", "not_applicable", "pending_deviation",
            "approved_deviation"}
SECTIONS = {
    "1.1": "abcd", "1.2": "a", "1.3": "ab", "1.4": "ab", "1.5": "ab", "1.6": "a", "1.7": "abcd", "1.8": "abc",
    "2.1": "abc", "2.2": "abcdefghi", "3.1": "abcdefghijklm", "3.2": "abcd", "3.3": "abc", "3.4": "abc", "3.5": "a", "3.6": "ab",
    "4.1": "abcd", "4.2": "abcd", "4.3": "abcdef", "4.4": "a", "5.1": "abc", "5.2": "abc", "5.3": "abc", "5.4": "ab", "5.5": "ab", "5.6": "ab",
    "6.1": "abcdefghi", "6.2": "abcdef", "6.3": "ab", "6.4": "a", "6.5": "abcd", "7.1": "abcdefghijklmno", "7.2": "abcd",
    "8.1": "a", "8.2": "abcd", "8.3": "abc", "8.4": "abcd", "8.5": "ab", "8.6": "a"}
EXPECTED_RULES = {f"{section}.{letter}" for section, letters in SECTIONS.items() for letter in letters}
for _section, _count in {"1.8.b": 4, "1.8.c": 4, "2.1.c": 2, "2.2.i": 3,
                         "4.2.c": 2, "5.4.b": 5, "6.3.b": 4}.items():
    EXPECTED_RULES.update(_section + "." + roman for roman in ("i", "ii", "iii", "iv", "v")[:_count])


@dataclass(frozen=True)
class Finding:
    rule: str
    path: str
    line: int
    message: str
    severity: str = "violation"


def relative(path: Path, root: Path) -> str:
    return path.resolve().relative_to(root.resolve()).as_posix()


def line_at(text: str, offset: int) -> int:
    return text.count("\n", 0, offset) + 1


def mask_noncode(text: str) -> str:
    """Preserve offsets/newlines while excluding comments and string literals."""
    return TOKEN.sub(lambda match: re.sub(r"[^\n]", " ", match.group())
                     if match.lastgroup in {"comment", "string"}
                     else match.group(), text)


def without_comments(text: str) -> str:
    return TOKEN.sub(lambda match: re.sub(r"[^\n]", " ", match.group())
                     if match.lastgroup == "comment" else match.group(), text)


def logical_source_lines(text: str) -> list[str]:
    """Return nonempty logical lines for structural preprocessor checks."""
    # C99 translates trigraphs and splices lines before removing comments.
    text = text.lstrip("\ufeff").replace("\r\n", "\n").replace("\r", "\n")
    spliced = text.replace("??=", "#").replace("??/", "\\").replace("\\\n", "")
    return [line.strip() for line in without_comments(spliced).splitlines() if line.strip()]


def macro_control_transfers(text: str):
    """Locate explicit control transfers in parameterized macro replacements."""
    normalized = text.lstrip("\ufeff").replace("\r\n", "\n").replace("\r", "\n")
    translated = normalized.replace("??=", "#").replace("??/", "\\")
    # Preserve the physical start of each directive, even when a token spans
    # a continuation. Comments and literals are removed after line splicing.
    starts = [1]
    for number, line in enumerate(translated.split("\n"), 1):
        if not line.endswith("\\"):
            starts.append(number + 1)
    code = mask_noncode(translated.replace("\\\n", ""))
    directive = re.compile(r"^[ \t]*(?:#|%:)\s*define\s+(\w+)\(([^)]*)\)(.*)$")
    for number, line in zip(starts, code.split("\n")):
        match = directive.match(line)
        if not match or not match[2].strip():
            continue
        parameters = set(re.findall(r"\w+", match[2]))
        for token in re.findall(r"\b(?:break|continue|goto|return|longjmp|exit)\b", match[3]):
            if token not in parameters:
                yield number, match[1], token


def enclosing_header_guard(text: str) -> str | None:
    """Return the conventional guard if it encloses every declaration."""
    lines = logical_source_lines(text)
    if len(lines) < 3:
        return None
    directive = r"(?:#|%:)\s*"
    opening = re.fullmatch(directive + r"ifndef\s+([A-Za-z_]\w*)", lines[0])
    if not opening:
        return None
    guard = opening[1]
    if not re.fullmatch(directive + r"define\s+" + guard + r"(?:\s+.*)?", lines[1]):
        return None
    depth = 1
    for index, line in enumerate(lines[2:], start=2):
        match = re.match(directive + r"(\w+)\b(.*)", line)
        if not match:
            continue
        keyword, argument = match[1], match[2].strip()
        if keyword in ("if", "ifdef", "ifndef"):
            depth += 1
        elif keyword == "endif":
            depth -= 1
            if depth == 0:
                return guard if index == len(lines) - 1 else None
        elif depth == 1 and keyword in ("else", "elif"):
            return None
        elif keyword == "undef" and argument == guard:
            return None
    return None


def includes_local_header_unconditionally(source: Path, text: str) -> bool:
    """Check for a direct, unconditional quoted include of the local pair."""
    header = source.with_suffix(".h").resolve()
    depth = 0
    for line in logical_source_lines(text):
        match = re.fullmatch(r"(?:#|%:)\s*(\w+)\b(.*)", line)
        if not match:
            continue
        directive, argument = match[1], match[2].strip()
        if directive in ("if", "ifdef", "ifndef"):
            depth += 1
        elif directive == "endif":
            depth -= 1
        elif directive == "include" and depth == 0:
            quoted = re.fullmatch(r'"([^"]+)"', argument)
            if quoted and (source.parent / quoted[1]).resolve() == header:
                return True
    return False


def owned_files(root: Path, scope: dict) -> list[Path]:
    """Enumerate manifest includes, including untracked files; never guess scope."""
    if scope.get("schema_version") not in {1, 2} or not scope.get("include_globs"):
        raise ValueError("Scope must have schema_version 1/2 and include_globs")
    # Reuse the representation inventory's enumerator so the two audits cannot
    # silently disagree about newly added files or explicit exclusions.
    if str(ROOT) not in sys.path:
        sys.path.insert(0, str(ROOT))
    from testing.compliance.inventory import owned_files as inventory_owned_files
    return [path for path in inventory_owned_files(root, scope)
            if path.suffix.lower() in {".c", ".h"}]


def lexical_checks(path: Path, root: Path, source_override=None, file_level=True) -> list[Finding]:
    data = path.read_bytes() if source_override is None else source_override.encode("utf-8")
    text = data.decode("utf-8")
    name = relative(path, root)
    findings = []

    def add(rule, number, message, severity="violation"):
        findings.append(Finding(rule, name, number, message, severity))

    for number, line in enumerate(text.splitlines(), 1):
        if len(line) > 80:
            add("1.2.a", number, f"Line has {len(line)} characters; maximum is 80")
        if "\t" in line:
            add("3.5.a", number, "Literal tab character")
    if b"\r" in data:
        add("3.6.a", 1, "Non-LF line endings need normalization or review", "review")
    for offset, character in enumerate(text):
        # Tabs and CR have their own diagnostics above. UTF-8 also permits
        # controls outside ASCII, such as U+0085 and zero-width format marks.
        if not character.isprintable() and character not in "\n\r\t\f":
            add("3.6.b", line_at(text, offset), "Forbidden nonprinting character")
    code = mask_noncode(text)
    for number, macro, token in macro_control_transfers(text):
        add("6.3.b.iv", number, f"Parameterized macro {macro} contains {token}")
    # Use the original whitespace. Masked literals and comments occupy spaces
    # in "code" even when the closing quote touches the punctuation.
    declarator_parens = {
        pos for item in GROUPED_POINTER_DECLARATOR.finditer(code)
        for pos in (item.start("group"), item.end() - 1)
    }
    for match in re.finditer(r"[();]", code):
        pos = match.start()
        before = text[pos - 1:pos] if pos else ""
        after = text[pos + 1:pos + 2]
        if match[0] == ";" and before in (" ", "\t"):
            add("3.1.m", line_at(text, pos), "Space immediately before semicolon")
        if match[0] == "(" and pos not in declarator_parens and after in (" ", "\t"):
            add("3.1.i", line_at(text, pos), "Horizontal padding inside opening parenthesis")
        if match[0] == ")" and pos not in declarator_parens and before in (" ", "\t"):
            line_start = text.rfind("\n", 0, pos) + 1
            if text[line_start:pos].strip():
                add("3.1.i", line_at(text, pos), "Horizontal padding inside closing parenthesis")
    for match in re.finditer(r"\b(auto|register|short|long)\b", code):
        rule = {"auto": "1.7.a", "register": "1.7.b"}.get(match[0], "5.2.b")
        add(rule, line_at(text, match.start()), f"Forbidden keyword {match[0]}")
    for match in re.finditer(r"\bcontinue\b", code):
        add("1.7.d", line_at(text, match.start()), "Review preferred avoidance of continue", "review")
    for match in re.finditer(r"(?m)^[ \t]+#", code):
        add("3.2.d", line_at(text, match.start()), "Preprocessor hash is indented")
    for match in re.finditer(r"(?m)^[ \t]*#[ \t]*define[ \t]+(\w+)", code):
        macro = match[1]
        if macro in KEYWORDS:
            add("1.1.d", line_at(text, match.start()), f"Macro renames language keyword {macro}")
        if re.search(r"[a-z]", macro):
            add("6.1.f", line_at(text, match.start()), f"Macro {macro} contains lowercase letters")
    for match in TOKEN.finditer(text):
        if match.lastgroup == "comment":
            body = match[0][2:-2] if match[0].startswith("/*") else match[0][2:]
            if re.search(r"/\*|//|\\", body):
                add("2.1.b", line_at(text, match.start()), "Comment content contains a forbidden comment token or backslash")
    includes = list(re.finditer(r'(?m)^[ \t]*#[ \t]*include[ \t]*[<"]([^>"\n]+)[>"]', without_comments(text)))
    for match in includes:
        target = match[1]
        if target.startswith(("/", "\\")) or re.match(r"[A-Za-z]:", target):
            add("4.3.d", line_at(text, match.start()), "Absolute include path")
        if target.endswith(".c"):
            add("4.3.f", line_at(text, match.start()), "Source includes another C source")
    if file_level and not re.fullmatch(r"[a-z0-9_]+", path.stem):
        add("4.1.a", 1, "Module basename contains disallowed characters")
    if file_level and path.suffix == ".h" and path.stem in STANDARD_HEADERS:
        add("4.1.c", 1, "Module reuses a standard-library header name")
    comments = [match for match in TOKEN.finditer(text) if match.lastgroup == "comment"]
    if file_level and path.suffix == ".c":
        if not path.with_suffix(".h").is_file():
            # The standard requires the same root name, not the same directory.
            # Include-search paths can bind a header in a separate include tree.
            add("4.2.a", 1, "No colocated same-root header; resolve the module's header in its compiler context", "review")
        own_includes = [item for item in includes
                        if Path(item[1]).name == path.with_suffix(".h").name]
        if not own_includes:
            macro_include = re.search(r"(?m)^\s*#\s*include\s+[^<\"\s]", without_comments(text))
            add("4.3.c", 1, "No literal same-root header include; macro includes require compiler resolution"
                if macro_include else "Same-root header is not directly included",
                "review" if macro_include else "violation")
        elif not any((path.parent / item[1]).resolve() == path.with_suffix(".h").resolve()
                     for item in own_includes):
            add("4.3.c", 1, "Same-root include uses a separate directory; confirm its resolved module header", "review")
        elif not includes_local_header_unconditionally(path, text):
            add("4.3.c", 1, "Check which configurations and search paths include the module header", "review")
        last = comments[-1] if comments else None
        if not (last and re.search(r"\b(?:end of file|eof)\b", last[0], re.IGNORECASE)
                and re.fullmatch(r"\n\n[\n]*", text[last.end():])):
            add("3.3.c", max(1, len(text.splitlines())), "Missing final end-of-file comment followed by a blank line")
    if file_level and path.suffix == ".h" and enclosing_header_guard(text) is None:
        add("4.2.b", 1, "Review header without an enclosing ifndef/define guard", "review")
    if file_level and not any(re.match(r"/\*\*|/\*!|///|//!", match[0]) and
               re.search(r"(?:@file|\\file)\b", match[0]) for match in comments):
        add("2.2.h", 1, "No automatic-documentation module @file comment found")
    return findings


def location(node: dict, fallback: str) -> tuple[str, int]:
    loc = node.get("loc", node.get("range", {}).get("begin", {}))
    loc = loc.get("spellingLoc", loc)
    return loc.get("file", fallback), loc.get("line", 0)


def float_type(node: dict) -> bool:
    types = node.get("type", {})
    spelling = types.get("desugaredQualType", types.get("qualType", ""))
    return bool(re.search(r"\b(?:float|double|_Float\d+)\b", spelling))


def contains_assignment(expression: dict) -> bool:
    """Check only the conditional expression, including nested operands."""
    if expression.get("kind") == "CompoundAssignOperator" or (
            expression.get("kind") == "BinaryOperator" and expression.get("opcode") == "="):
        return True
    return any(contains_assignment(child) for child in expression.get("inner", []))


def has_own_default(statement: dict) -> bool:
    """A default inside a nested switch belongs to that switch alone."""
    kind = statement.get("kind")
    if kind == "SwitchStmt":
        return False
    if kind == "DefaultStmt":
        return True
    return any(has_own_default(child) for child in statement.get("inner", []))


def control_bodies(node: dict) -> list[dict]:
    """Return the governed statements in Clang's C AST, excluding else-if links."""
    children = node.get("inner", [])
    kind = node["kind"]
    if not children:
        return []
    if kind == "IfStmt" and node.get("hasElse") and len(children) >= 3:
        then, otherwise = children[-2:]
        return [then] if otherwise.get("kind") == "IfStmt" else [then, otherwise]
    if kind == "DoStmt":
        return children[:1]
    if kind in {"IfStmt", "WhileStmt", "ForStmt", "SwitchStmt"}:
        return children[-1:]
    return []


def ast_checks(tree: dict, source: Path, root: Path, owned: set[Path], allowed_lines=None):
    """Inspect Clang-declared types/definitions, never regex-infer C types.

    Results cover this translation unit/profile only. Missing file provenance
    in included locations is skipped and reported as incomplete by the caller.
    """
    findings = []
    documented = []
    functions = []
    unknown_locations = 0

    def walk(node, inherited, parents=()):
        nonlocal unknown_locations
        filename, number = location(node, inherited)
        path = Path(filename)
        if not path.is_absolute():
            path = root / path
        path = path.resolve()
        kind = node.get("kind")
        loc = node.get("loc", node.get("range", {}).get("begin", {}))
        ambiguous = "includedFrom" in loc and "file" not in loc
        line_ranges = (allowed_lines or {}).get(path)
        relevant = path in owned and not ambiguous and (line_ranges is None or
                    any(start <= number <= end for start, end in line_ranges))
        if ambiguous and kind in {"FunctionDecl", "VarDecl", "ParmVarDecl", "FieldDecl"}:
            unknown_locations += 1

        def add(rule, message, severity="violation"):
            findings.append(Finding(rule, relative(path, root), number, message, severity))

        if relevant and kind == "TypedefDecl" and not node.get("isImplicit"):
            name = node.get("name", "")
            if name and not re.fullmatch(r"[a-z][a-z0-9_]*_t", name):
                add("5.1.a", f"Type {name} needs lowercase spelling and a _t suffix")
        if relevant and kind == "DeclStmt":
            variables = [child.get("name", "") for child in node.get("inner", [])
                         if child.get("kind") == "VarDecl"]
            if len(variables) > 1:
                add("8.1.a", "Variables share one local declaration: " + ", ".join(variables))
        if relevant and kind == "FunctionDecl" and not node.get("isImplicit"):
            name = node.get("name", "")
            if name in RESERVED_IDENTIFIERS:
                add("6.1.a", f"Function {name} reuses a reserved language name")
            if name.startswith("_"):
                add("6.1.c", f"Function {name} starts with underscore")
            if len(name) > 31:
                add("6.1.d", f"Function {name} exceeds 31 characters")
            if re.search(r"[A-Z]", name):
                add("6.1.e", f"Function {name} contains uppercase letters")
            if node.get("storageClass") != "static" and not name.startswith(path.stem + "_"):
                add("6.1.i", f"Public function {name} lacks {path.stem}_ prefix; external ABI exceptions require approval")
            body = next((item for item in node.get("inner", []) if item.get("kind") == "CompoundStmt"), None)
            if body:
                functions.append((relative(path, root), number, name))
                comment = any(item.get("kind") == "FullComment" for item in node.get("inner", []))
                if comment:
                    documented.append((relative(path, root), number, name))
                else:
                    add("2.2.h", f"Function definition {name} has no Clang-recognized documentation comment")
                end = body.get("range", {}).get("end", {}).get("line", number)
                if end and number and end - number + 1 > 100:
                    add("6.2.a", f"Function {name} spans {end - number + 1} lines; review decomposition or an explicit deviation", "review")
                if name == "main" and "main" not in path.stem:
                    add("4.1.d", "Module defining main lacks main in its basename")
        if relevant and kind in {"VarDecl", "ParmVarDecl", "FieldDecl"} and not node.get("isImplicit"):
            name = node.get("name", "")
            if name:
                if name in RESERVED_IDENTIFIERS:
                    add("7.1.a", f"Variable {name} reuses a reserved language name")
                if name in STANDARD_OBJECT_NAMES:
                    add("7.1.b", f"Variable {name} reuses a standard library name")
                if name.startswith("_"):
                    add("7.1.c", f"Variable {name} starts with underscore")
                if len(name) > 31:
                    add("7.1.d", f"Variable {name} exceeds 31 characters")
                if len(name) < 3:
                    add("7.1.e", f"Variable {name} is shorter than three characters")
                if re.search(r"[A-Z]", name):
                    add("7.1.f", f"Variable {name} contains uppercase letters")
        if relevant and kind in {"IfStmt", "DoStmt", "WhileStmt", "ForStmt", "SwitchStmt"}:
            for body in control_bodies(node):
                if body.get("kind") != "CompoundStmt":
                    add("1.3.a", f"{kind} body has no braces")
        if relevant and kind == "BinaryOperator":
            children = node.get("inner", [])
            if node.get("opcode") in {"==", "!="} and any(float_type(item) for item in children):
                add("5.4.b.iv", "Equality/inequality operates on floating-point values")
        if relevant and kind == "IfStmt":
            if node.get("inner") and contains_assignment(node["inner"][0]):
                add("8.2.c", "Assignment in if condition")
            depth = 1 + sum(parent == "IfStmt" for parent in parents)
            if depth > 2:
                add("8.2.b", f"Conditional nesting depth {depth}; else-if chains need manual structural review", "review")
        if relevant and kind == "SwitchStmt" and node.get("inner") and not has_own_default(node["inner"][-1]):
            add("8.3.b", "Switch has no default label")
        for child in node.get("inner", []):
            walk(child, filename, parents + (kind,))

    walk(tree, str(source))
    return findings, functions, documented, unknown_locations


def validate_ledger(ledger: dict) -> list[str]:
    errors = []
    seen = set()
    for row in ledger.get("rules", []):
        key = row.get("id")
        if key in seen:
            errors.append(f"Duplicate rule {key}")
        seen.add(key)
        if row.get("strength") not in STRENGTHS or row.get("status") not in STATUSES:
            errors.append(f"Invalid strength/status for {key}")
        if row.get("status") != "unreviewed" and not row.get("evidence"):
            errors.append(f"Rule {key} has no evidence for its reviewed status")
        if row.get("status") == "approved_deviation" and not all(row.get(field) for field in ("approver", "approved_at", "rationale")):
            errors.append(f"Rule {key} claims approval without approver, date and rationale")
    if {str(number) for number in range(1, 9)} != {str(key).split(".")[0] for key in seen}:
        errors.append("Ledger does not cover all eight chapters")
    if seen != EXPECTED_RULES:
        errors.append(f"Ledger rule IDs differ: missing {sorted(EXPECTED_RULES - seen)}, unexpected {sorted(seen - EXPECTED_RULES)}")
    return errors


def audit(root: Path, scope: dict, ledger: dict, ast_manifest: Path | None = None) -> dict:
    paths = owned_files(root, scope)
    if not paths:
        raise ValueError("Scope selected no C/header files")
    findings = []
    allowed_lines = {}
    mixed = scope.get("mixed_files", {})
    from testing.compliance.inventory import mixed_owned_text, mixed_region_spans
    for path in paths:
        region = mixed.get(relative(path, root))
        if region:
            text = path.read_text(encoding="utf-8")
            spans = mixed_region_spans(text, region)
            masked = mixed_owned_text(text, region)
            # The shared mask preserves offsets with spaces. Those spaces are
            # imported text, so they must not create false line-width findings.
            masked = re.sub(r"(?m)^[ \t]+(?=\n|$)", "", masked)
            allowed_lines[path] = [(line_at(text, start), line_at(text, end - 1))
                                   for start, end in spans]
            findings.extend(lexical_checks(path, root, masked, file_level=False))
        else:
            findings.extend(lexical_checks(path, root))
    prefixes = {}
    for path in paths:
        if relative(path, root) in mixed:
            continue
        key = path.stem[:8]
        module = path.with_suffix("")
        previous = prefixes.setdefault(key, module)
        if previous != module:
            split_pair = previous.name == module.name
            message = ("Repeated root name needs module-pair resolution with " if split_pair
                       else "First-eight-character collision with ")
            findings.append(Finding("4.1.b", relative(path, root), 1,
                                    message + relative(previous, root),
                                    "review" if split_pair else "violation"))
    gaps = validate_ledger(ledger)
    from testing.compliance.inventory import repository_errors
    gaps.extend(repository_errors(root, scope))
    if mixed:
        gaps.append(f"{len(mixed)} mixed-ownership file(s): only recorded owned regions scanned; surrounding imported bodies are not BARR edit targets")
    functions, documented, covered = set(), set(), set()
    if ast_manifest is None:
        gaps.append("No Clang AST manifest supplied; function/type/profile coverage is unknown")
    else:
        manifest = json.loads(ast_manifest.read_text(encoding="utf-8"))
        if manifest.get("schema_version") != 1:
            raise ValueError("Unsupported AST manifest schema")
        hashes = manifest.get("source_sha256", {})
        for path in paths:
            name = relative(path, root)
            if hashes.get(name) != hashlib.sha256(path.read_bytes()).hexdigest():
                raise ValueError(f"AST manifest missing or stale owned-source hash: {name}")
        for unit in manifest.get("translation_units", []):
            if not all(unit.get(key) for key in ("source", "ast", "profile", "compiler", "command")):
                raise ValueError("AST unit needs source, ast, profile, compiler and command")
            source = (root / unit["source"]).resolve()
            if source not in paths:
                raise ValueError(f"AST primary source outside owned scope: {unit['source']}")
            tree = json.loads((ast_manifest.parent / unit["ast"]).read_text(encoding="utf-8"))
            if tree.get("kind") != "TranslationUnitDecl":
                raise ValueError("AST root is not a Clang TranslationUnitDecl")
            result, found, docs, unknown = ast_checks(tree, source, root, set(paths), allowed_lines)
            findings.extend(result)
            functions.update(found)
            documented.update(docs)
            covered.add(source)
            if unknown:
                gaps.append(f"{unit['source']}: {unknown} AST declarations have ambiguous include provenance")
        for path in paths:
            if path.suffix == ".c" and path not in covered:
                gaps.append(f"No AST translation unit for {relative(path, root)}")
        gaps.append("AST coverage is limited to supplied profiles; supported conditional-build coverage requires review")
    unreviewed = [row["id"] for row in ledger["rules"] if row["status"] in {"unreviewed", "pending_deviation"}]
    if unreviewed:
        gaps.append(f"{len(unreviewed)} ledger entries remain unreviewed or pending deviation")
    # Findings can repeat through includes; do not inflate observed violation counts.
    findings = sorted(set(findings), key=lambda row: (row.path, row.line, row.rule))
    violations = sum(row.severity == "violation" for row in findings)
    reviews = sum(row.severity == "review" for row in findings)
    status = "FAIL" if violations else "INCOMPLETE" if gaps or reviews else "PASS"
    return {"schema_version": 1, "status": status, "full_compliance_certified": False,
            "files_scanned": len(paths), "violations": violations, "review_findings": reviews,
            "function_definitions_observed": len(functions),
            "documented_definitions_observed": len(documented),
            "documentation_coverage_complete": False,
            "findings": [asdict(row) for row in findings], "gaps": gaps}


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scope", type=Path, default=SCOPE)
    parser.add_argument("--ledger", type=Path, default=LEDGER)
    parser.add_argument("--ast-manifest", type=Path)
    parser.add_argument("--json", action="store_true", help="Print full JSON report")
    parser.add_argument("--limit", type=int, default=40, help="Maximum displayed findings; JSON is never truncated")
    args = parser.parse_args(argv)
    try:
        report = audit(ROOT, json.loads(args.scope.read_text(encoding="utf-8")),
                       json.loads(args.ledger.read_text(encoding="utf-8")), args.ast_manifest)
    except (OSError, UnicodeError, ValueError, KeyError, TypeError) as error:
        print(json.dumps({"status": "INCOMPLETE", "error": str(error)}) if args.json
              else f"INCOMPLETE: {error}")
        return 2
    if args.json:
        print(json.dumps(report, indent=2))
    else:
        for row in report["findings"][:max(0, args.limit)]:
            print(f"{row['path']}:{row['line']}: {row['rule']} {row['severity']}: {row['message']}")
        for gap in report["gaps"]:
            print(f"INCOMPLETE: {gap}")
        print(f"{report['status']}: {report['files_scanned']} files; {report['violations']} observed violations; "
              f"{report['review_findings']} review findings. Full BARR compliance is not certified.")
    return 1 if report["status"] == "FAIL" else 2 if report["status"] == "INCOMPLETE" else 0


if __name__ == "__main__":
    sys.exit(main())
