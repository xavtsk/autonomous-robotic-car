#!/usr/bin/env python3
"""Bind the CR1 field ledger to current owned sources without compiling firmware.

The lexical pass preserves every conditional branch and finds ordinary record
fields. It is deliberately NOT a C parser or proof of whole-source coverage.
Compiler observations are collected separately by inventory_ast.py. Unknown
domains and unobserved files remain explicitly unresolved. --check checks ledger
integrity, whereas --require-complete additionally gates CR1 closure.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import io
import json
import re
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent
STATUSES = {"unresolved", "gap", "packed", "preserve", "boundary", "test"}
TOKEN = re.compile(r'"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\'|[A-Za-z_]\w*|'
                   r'0[xX][0-9A-Fa-f]+[uUlL]*|\d+(?:\.\d*)?[uUlLfF]*|'
                   r'>>=|<<=|->|==|!=|<=|>=|&&|\|\||<<|>>|\+\+|--|\S')
COMMENTS = re.compile(r'"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\'|'
                      r'/\*.*?\*/|//[^\n]*', re.DOTALL)
IDENTIFIER = re.compile(r"^[A-Za-z_]\w*$")


def read_json(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def owned_files(root=ROOT, scope=None):
    """Enumerate the explicit scope, including new/untracked source files."""
    scope = scope or read_json(HERE / "inventory_scope.json")
    files = {p.resolve() for pattern in scope["include_globs"]
             for p in Path(root).glob(pattern) if p.is_file()}
    files.update((Path(root) / path).resolve() for path in scope.get("mixed_files", {})
                 if (Path(root) / path).is_file())
    excluded = {p.resolve() for pattern in scope.get("exclude_globs", [])
                for p in Path(root).glob(pattern) if p.is_file()}
    return sorted(files - excluded)


def repository_sources(root):
    """Tracked plus nonignored untracked C/H files, including unknown folders."""
    result = subprocess.run(["git", "-C", str(root), "ls-files", "-z", "--cached",
                             "--others", "--exclude-standard"], capture_output=True,
                            check=False)
    if result.returncode == 0:
        names = {name.decode("utf-8") for name in result.stdout.split(b"\0") if name}
        return sorted(name for name in names if Path(name).suffix.lower() in {".c", ".h"})
    # Isolated unit-test fixtures and exported source trees lack a Git index.
    return sorted(p.relative_to(root).as_posix() for p in root.rglob("*")
                  if p.is_file() and p.suffix.lower() in {".c", ".h"})


def imported_files(root, scope):
    name = scope.get("imports_manifest")
    return read_json(root / name).get("files", {}) if name else {}


def mixed_region_spans(text, entry):
    """Resolve explicitly owned directives; reject ambiguous or nested overlap."""
    lines = text.splitlines(keepends=True)
    offsets, offset = [], 0
    for line in lines:
        offsets.append(offset)
        offset += len(line)
    spans = []
    for region in entry["owned_regions"]:
        if region["kind"] == "macro":
            pattern = r"^\s*#\s*define\s+" + re.escape(region["name"]) + r"\b"
            found = [i for i, line in enumerate(lines) if re.match(pattern, line)]
        elif region["kind"] == "conditional":
            expected = TOKEN.findall(region["directive"])
            found = [i for i, line in enumerate(lines)
                     if TOKEN.findall(uncomment(line)) == expected]
        else:
            raise ValueError("Unknown mixed-region selector kind")
        if len(found) != 1:
            raise ValueError(f"Mixed-region selector must match once: {region}")
        start = end = found[0]
        if region["kind"] == "macro":
            while lines[end].rstrip().endswith("\\"):
                end += 1
                if end >= len(lines):
                    raise ValueError("Unterminated owned macro")
        else:
            depth = 0
            for end in range(start, len(lines)):
                directive = re.match(r"^\s*#\s*(if|ifdef|ifndef|endif)\b", uncomment(lines[end]))
                if directive:
                    depth += -1 if directive[1] == "endif" else 1
                    if depth == 0:
                        break
            if depth != 0:
                raise ValueError("Unterminated owned conditional")
        spans.append((offsets[start], offsets[end] + len(lines[end])))
    spans.sort()
    if any(left[1] > right[0] for left, right in zip(spans, spans[1:])):
        raise ValueError("Overlapping owned regions")
    return spans


def mixed_owned_text(text, entry):
    """Return only project-owned spans, preserving source offsets and lines."""
    chars = list(re.sub(r"[^\n]", " ", text))
    for start, end in mixed_region_spans(text, entry):
        chars[start:end] = text[start:end]
    return "".join(chars)


def mixed_preserved_body(text, entry):
    """Replace owned spans while retaining exact imported bytes around them."""
    for index, (start, end) in reversed(list(enumerate(mixed_region_spans(text, entry)))):
        text = text[:start] + f"<PROJECT_REGION:{index}>\n" + text[end:]
    return text.replace("\r\n", "\n")


def repository_errors(root, scope):
    owned = {p.relative_to(root).as_posix() for p in owned_files(root, scope)}
    imports = imported_files(root, scope)
    present = set(repository_sources(root))
    errors = [f"UNCLASSIFIED REPOSITORY FILE: {path}" for path in sorted(present - owned - imports.keys())]
    for path in sorted(imports.keys() - present):
        errors.append(f"IMPORTED FILE MISSING: {path}")
    for path, region in scope.get("mixed_files", {}).items():
        file = root / path
        if not file.is_file():
            errors.append(f"MIXED-OWNERSHIP FILE MISSING: {path}")
            continue
        text = file.read_text(encoding="utf-8-sig")
        try:
            body = mixed_preserved_body(text, region)
            digest = hashlib.sha256(body.encode()).hexdigest()
        except ValueError:
            digest = None
        if digest != region.get("preserved_body_sha256"):
            errors.append(f"MIXED IMPORTED BODY CHANGED: {path}; only the recorded project integration region is owned")
    # Compare content independently of checkout filters. Historical imported
    # blobs can contain CRLF while the current attributes request LF. The blob
    # IDs remain provenance evidence; only line-ending normalization is allowed.
    for path in sorted(present & imports.keys()):
        file = root / path
        if not file.is_file():
            errors.append(f"IMPORTED FILE MISSING: {path}")
            continue
        digest = hashlib.sha256(file.read_bytes().replace(b"\r\n", b"\n")).hexdigest()
        if digest != imports[path].get("baseline_content_sha256_lf"):
            errors.append(f"IMPORTED BOUNDARY CHANGED: {path}")
    return errors


def uncomment(text):
    def replace(match):
        value = match[0]
        return re.sub(r"[^\n]", " ", value) if value.startswith(("/*", "//")) else value
    return COMMENTS.sub(replace, text)


def semantic_digest(text):
    """Ignore comments/whitespace, but bind directives, literals and all tokens."""
    return hashlib.sha256("\n".join(TOKEN.findall(uncomment(text))).encode()).hexdigest()


def source_location_digest(text):
    """Bind token line positions without treating comment prose as C data."""
    clean = uncomment(text)
    tokens, line, previous = [], 1, 0
    for match in TOKEN.finditer(clean):
        line += clean.count("\n", previous, match.start())
        tokens.append((match[0], line))
        line += match[0].count("\n")
        previous = match.end()
    return semantic_json_digest(tokens)


def without_directives(text):
    lines, continued = [], False
    for line in text.splitlines(keepends=True):
        directive = continued or line.lstrip().startswith("#")
        continued = directive and line.rstrip().endswith("\\")
        lines.append(re.sub(r"[^\n]", " ", line) if directive else line)
    return "".join(lines)


def split_top(tokens, separator):
    result, chunk, depth = [], [], 0
    for token in tokens:
        if token[0] in ("(", "[", "{"):
            depth += 1
        elif token[0] in (")", "]", "}"):
            depth -= 1
        if token[0] == separator and depth == 0:
            result.append(chunk)
            chunk = []
        else:
            chunk.append(token)
    result.append(chunk)
    return result


def matching(tokens, start, left="{", right="}"):
    depth = 0
    for index in range(start, len(tokens)):
        depth += tokens[index][0] == left
        depth -= tokens[index][0] == right
        if depth == 0:
            return index
    return None


def declarator(tokens):
    """Recognize an ordinary field declarator; refuse unfamiliar forms."""
    tokens = split_top(tokens, "=")[0]
    values = [t[0] for t in tokens]
    if not values or values[0] in ("typedef", "return"):
        return None
    for index in range(len(values) - 3):
        if values[index:index + 2] == ["(", "*"] and IDENTIFIER.fullmatch(values[index + 2]):
            return index + 2
    stop = next((i for i, v in enumerate(values) if v in ("[", ":")), len(values))
    candidates = [i for i in range(stop) if IDENTIFIER.fullmatch(values[i])]
    return candidates[-1] if candidates else None


def lexical_fields(text, path):
    """Conditional-union fallback for record members, not whole-C coverage."""
    clean = without_directives(uncomment(text))
    tokens = [(m[0], m.start(), m.end()) for m in TOKEN.finditer(clean)]
    rows, unsupported, owner_occurrences = [], [], {}
    for index, token in enumerate(tokens):
        if token[0] not in ("struct", "union"):
            continue
        start = index + 1
        while start < len(tokens) and tokens[start][0] not in ("{", ";", "=", "}"):
            start += 1
        if start == len(tokens) or tokens[start][0] != "{":
            continue
        end = matching(tokens, start)
        if end is None:
            unsupported.append({"line": text.count("\n", 0, token[1]) + 1, "reason": "unclosed record"})
            continue
        tail = []
        cursor = end + 1
        while cursor < len(tokens) and tokens[cursor][0] not in (";", "=", "{"):
            tail.append(tokens[cursor])
            cursor += 1
        names = [t[0] for t in tail if IDENTIFIER.fullmatch(t[0])]
        tag = tokens[index + 1][0] if index + 1 < start else ""
        owner = names[0] if names else tag if IDENTIFIER.fullmatch(tag) else f"anonymous_{token[1]}"
        # A struct use in a function declaration can precede that function's
        # opening brace. Require an actual record header, never a ')' header.
        header = [t[0] for t in tokens[index + 1:start]]
        if ")" in header and "__attribute__" not in header:
            continue
        # Different function-local records can reuse the same variable/tag
        # name. Keep them distinct; their widths are not profile variants.
        owner_occurrences[owner] = owner_occurrences.get(owner, 0) + 1
        if owner_occurrences[owner] > 1:
            owner += f"@{owner_occurrences[owner]}"
        for statement in split_top(tokens[start + 1:end], ";"):
            if not statement:
                continue
            if any(t[0] in ("{", "}") for t in statement):
                unsupported.append({"line": text.count("\n", 0, statement[0][1]) + 1,
                                    "reason": f"nested/anonymous member in {owner}; compiler observation required"})
                continue
            base_type = ""
            for part_index, part in enumerate(split_top(statement, ",")):
                name_at = declarator(part)
                if name_at is None:
                    continue
                name = part[name_at][0]
                if name in {"bool", "unsigned", "int", "uint32_t", "uint16_t", "uint8_t"}:
                    continue  # unnamed allocation-unit padding
                if "(" in [t[0] for t in part] and "*" not in [t[0] for t in part]:
                    unsupported.append({"line": text.count("\n", 0, part[0][1]) + 1,
                                        "reason": f"macro-generated fields in {owner}"})
                    continue
                if part_index == 0:
                    base_type = " ".join(t[0] for t in part[:name_at])
                values = [t[0] for t in part]
                width = " ".join(values[values.index(":") + 1:]) if ":" in values else None
                rows.append({"path": path, "owner": owner, "name": name, "kind": "FIELD_DECL",
                             "line": text.count("\n", 0, part[name_at][1]) + 1,
                             "column": part[name_at][1] - text.rfind("\n", 0, part[name_at][1]),
                             "type": base_type, "bit_width_expression": width,
                             "stored_bits": int(width) if width and width.isdecimal() else None,
                             "method": "lexical-conditional-union", "profiles": []})
    # The control debug X-macro is intentionally retained as a schema. Bind
    # its names and declared widths, but not those widths as in-memory widths.
    if "CONTROL_DEBUG_FIELDS(U, S)" in text:
        for match in re.finditer(r"\b([US])\((\w+),\s*(\d+)\)", uncomment(text)):
            rows.append({"path": path, "owner": "control_debug_sample_t", "name": match[2],
                         "kind": "FIELD_DECL", "line": text.count("\n", 0, match.start()) + 1,
                         "type": "uint32_t" if match[1] == "U" else "int32_t",
                         "stored_bits": 32, "schema_bits": int(match[3]),
                         "method": "lexical-x-macro-schema", "profiles": []})
    return rows, unsupported


def row_id(row):
    return "::".join((row["path"], row["kind"], row.get("owner", ""), row["name"]))


def minimum_bits(lo, hi):
    if hi < lo:
        raise ValueError("reversed domain")
    return max(1, (hi - lo).bit_length())


def reviewed_enum_abi(row, approved):
    """Validate a named public enum boundary against clean, explicit ABIs.

    A width variant is not itself a range proof. Require an approved target,
    compatible integer kind, compiler option and width for every observation;
    each combination must have a clean AST, including any recovery sightings.
    """
    observations = row.get("enum_abi_observations", [])
    if row.get("type_category") != "ENUM" or row.get("method") != "libclang-ast" or not observations:
        return None
    observed_keys, clean_keys, widths, domains = set(), set(), set(), {}
    for item in observations:
        candidates = [abi for abi in approved
                      if item.get("compatible_kind") == abi["compatible_kind"]
                      and item.get("compatible_bits") == abi["bits"]
                      and item.get("stored_bits") == abi["bits"]
                      and item.get("enum_option") == abi["enum_option"]
                      and item.get("compiler_target") in abi["compiler_targets"]]
        if len(candidates) != 1 or item.get("method") not in {"libclang-ast", "libclang-recovery"}:
            return None
        abi = candidates[0]
        # The policy must independently state the complete compatible integer
        # domain, not merely assert that a convenient enum width is supported.
        bits, kind = abi["bits"], abi["compatible_kind"]
        unsigned = kind in {"UCHAR", "USHORT", "UINT", "ULONG", "ULONGLONG"}
        signed = kind in {"SCHAR", "SHORT", "INT", "LONG", "LONGLONG"}
        if not (unsigned or signed) or bits < 1:
            return None
        expected = (0, (1 << bits) - 1) if unsigned else (-(1 << (bits - 1)), (1 << (bits - 1)) - 1)
        if (abi.get("min"), abi.get("max")) != expected:
            return None
        key = (item["compiler_target"], kind, bits, item["enum_option"])
        observed_keys.add(key)
        if item["method"] == "libclang-ast":
            clean_keys.add(key)
        widths.add(bits)
        domains[key] = {"compiler_target": key[0], "compatible_kind": kind,
                        "enum_option": key[3], "min": expected[0], "max": expected[1],
                        "required_bits": bits, "stored_bits": bits}
    reported_widths = set(row.get("profile_width_variants", [])) | {row.get("stored_bits")}
    if observed_keys != clean_keys or widths != reported_widths:
        return None
    return [domains[key] for key in sorted(domains)]


def classify(row, policies):
    """Apply reviewed domain rules; do not infer bounded domains from types."""
    for rule in policies.get("rules", []):
        selectors = ("path", "owner", "name", "kind", "type_category", "storage", "type",
                     "storage_duration", "function_linkage", "linkage", "method")
        selectors += ("stored_bits", "canonical_type")
        if all(re.fullmatch(rule.get(key, ".*"), str(row.get(key, "")))
               for key in selectors):
            result = {key: value for key, value in rule.items()
                      if key not in selectors and key != "id"}
            result["policy"] = rule["id"]
            domain = result.get("domain") or {}
            if domain.get("values"):
                result["required_bits"] = max(1, (len(set(domain["values"])) - 1).bit_length())
            elif "min" in domain and "max" in domain:
                step = domain.get("step", 1)
                if not isinstance(step, int) or step < 1 or (domain["max"] - domain["min"]) % step:
                    raise ValueError("Reviewed discrete domain requires a positive, exact interval step")
                result["required_bits"] = minimum_bits(0, (domain["max"] - domain["min"]) // step)
            else:
                result.setdefault("required_bits", None)
            if "native_enum_abis" in domain:
                observed = reviewed_enum_abi(row, domain["native_enum_abis"])
                if observed is None:
                    result["status"] = "unresolved"
                    result["required_bits"] = None
                    result["reason"] += " Missing or unsupported clean enum ABI/domain evidence; no boundary closure."
                else:
                    result["required_bits"] = row["stored_bits"]
                    result["required_bits_by_abi"] = observed
            if domain.get("minimum_width_proved") is False:
                result["safe_upper_bound_bits"] = result["required_bits"]
                result["required_bits"] = None
                result["status"] = "unresolved"
            if result.get("status") == "compare":
                used, need = row.get("stored_bits"), result["required_bits"]
                result["status"] = "unresolved" if used is None or row.get("method") == "libclang-recovery" or len(row.get("profile_width_variants", [])) > 1 else "packed" if used == need else "gap" if used > need else "unresolved"
            return result
    return {"status": "unresolved", "policy": None, "domain": None, "required_bits": None,
            "ownership": "unreviewed", "constraints": ["Derive accepted, sentinel and terminal values from every producer/consumer before narrowing."],
            "evidence": [], "reason": "No reviewed application-domain classification matches this declaration."}


def collect(root, scope, policies, ast=None):
    files, rows, mixed_lines = {}, {}, {}
    for file in owned_files(root, scope):
        path = file.relative_to(root).as_posix()
        text = file.read_text(encoding="utf-8-sig")
        owned_text = mixed_owned_text(text, scope["mixed_files"][path]) if path in scope.get("mixed_files", {}) else text
        if owned_text != text:
            mixed_lines[path] = {i for i, line in enumerate(owned_text.splitlines(), 1) if line.strip()}
        discovered, unsupported = lexical_fields(owned_text, path)
        files[path] = {"semantic_sha256": semantic_digest(text), "source_location_sha256": source_location_digest(text), "lexical_unsupported": unsupported,
                       "compiler_profiles": [], "compiler_status": "unobserved"}
        for row in discovered:
            rows[row_id(row)] = row
    if ast:
        for row in ast.get("declarations", []):
            if row["path"] not in files:
                continue
            if row["path"] not in ast.get("files", {}):
                raise ValueError(f"Missing compiler input binding for {row['path']}; recollect AST before refresh")
            if row["path"] in mixed_lines and row["line"] not in mixed_lines[row["path"]]:
                continue
            key = row_id(row)
            old = rows.get(key, {})
            row = dict(row)
            if "schema_bits" in old:
                row["schema_bits"] = old["schema_bits"]
            rows[key] = row
        for path, receipt in ast.get("files", {}).items():
            if path in files:
                if receipt.get("semantic_sha256") != files[path]["semantic_sha256"]:
                    raise ValueError(f"Stale compiler observations for {path}; recollect AST before refresh")
                if receipt.get("source_location_sha256") != files[path]["source_location_sha256"]:
                    raise ValueError(f"Stale or missing compiler source-location binding for {path}; recollect AST before refresh")
                bindings = receipt.get("input_bindings", {})
                if path not in bindings:
                    raise ValueError(f"Missing compiler dependency binding for {path}; recollect AST before refresh")
                for dependency, binding in bindings.items():
                    if dependency not in files or any(files[dependency].get(key) != binding.get(key)
                            for key in ("semantic_sha256", "source_location_sha256")):
                        raise ValueError(f"Stale compiler dependency for {path}: {dependency}; recollect affected translation units")
                files[path]["compiler_profiles"] = receipt.get("profiles", [])
                files[path]["compiler_status"] = receipt.get("status", "partial")
                files[path]["compiler_input_bindings"] = bindings
    for key, row in rows.items():
        row.update(classify(row, policies))
    for path, file in files.items():
        keys = sorted(key for key, row in rows.items() if row["path"] == path)
        file["declaration_count"] = len(keys)
        file["declaration_sha256"] = semantic_json_digest(keys)
    return {"schema_version": 1, "baseline_sha": scope["baseline_sha"],
            "scope_sha256": semantic_json_digest(scope),
            "policies_sha256": semantic_json_digest(policies),
            "coverage_statement": "Compiler-observed declarations plus lexical record-field union. Unobserved profiles and unsupported lexical forms remain unresolved; source digests fail closed on all semantic source changes.",
            "files": dict(sorted(files.items())), "fields": [rows[k] for k in sorted(rows)]}


def semantic_json_digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":")).encode()).hexdigest()


def compact_ledger(ledger):
    """Factor repeated explanations into the reviewed policy dictionary."""
    result = dict(ledger)
    result["compiler_inputs"] = {}
    result["files"] = {}
    for path, source in ledger["files"].items():
        file = dict(source)
        bindings = file.pop("compiler_input_bindings", {})
        if bindings:
            file["compiler_input_refs"] = sorted(bindings)
            for dependency, binding in bindings.items():
                if (dependency in result["compiler_inputs"] and
                        result["compiler_inputs"][dependency] != binding):
                    raise ValueError(f"Conflicting compiler input snapshots: {dependency}")
                result["compiler_inputs"][dependency] = binding
        result["files"][path] = file
    result["compiler_inputs"] = dict(sorted(result["compiler_inputs"].items()))
    result["classification_format"] = "Domain/width/status are inline; ownership, constraints, evidence and reason resolve through the policy ID in inventory_policies.json. Null policy is explicitly unresolved."
    omitted = {"id", "ownership", "constraints", "evidence", "reason"}
    result["fields"] = [{key: value for key, value in row.items() if key not in omitted}
                        for row in ledger["fields"]]
    return result


def expand_ledger(ledger, policies):
    result = dict(ledger)
    result["files"] = {}
    for path, source in ledger["files"].items():
        file = dict(source)
        if "compiler_input_refs" in file:
            file["compiler_input_bindings"] = {dependency: ledger.get("compiler_inputs", {}).get(dependency, {})
                                               for dependency in file.pop("compiler_input_refs")}
        result["files"][path] = file
    result["fields"] = []
    for source in ledger["fields"]:
        row = dict(source)
        expected = classify(row, policies)
        for key in ("ownership", "constraints", "evidence", "reason"):
            row[key] = expected[key]
        result["fields"].append(row)
    return result


def validate(root, scope, policies, ledger, require_complete=False):
    errors = repository_errors(root, scope)
    actual = {p.relative_to(root).as_posix(): p for p in owned_files(root, scope)}
    recorded = ledger.get("files", {})
    current_bindings = {}
    for path, file in actual.items():
        text = file.read_text(encoding="utf-8-sig")
        current_bindings[path] = {"semantic_sha256": semantic_digest(text),
                                  "source_location_sha256": source_location_digest(text)}
    for path in sorted(actual.keys() - recorded.keys()):
        errors.append(f"UNCLASSIFIED FILE: {path}")
    for path in sorted(recorded.keys() - actual.keys()):
        errors.append(f"STALE FILE: {path}")
    keys = set()
    for row in ledger.get("fields", []):
        key = row_id(row)
        if key in keys:
            errors.append(f"DUPLICATE FIELD: {key}")
        keys.add(key)
        if row.get("status") not in STATUSES or "domain" not in row or "required_bits" not in row:
            errors.append(f"UNCLASSIFIED FIELD: {key}")
        if row.get("status") in {"packed", "gap"} and (row.get("required_bits") is None or not row.get("evidence")):
            errors.append(f"UNSUPPORTED WIDTH CLAIM: {key}")
        expected = classify(row, policies)
        if any(row.get(k) != v for k, v in expected.items()):
            errors.append(f"STALE CLASSIFICATION: {key}")
        if require_complete and row.get("status") in {"unresolved", "gap"}:
            errors.append(f"OPEN CR1 FIELD ({row.get('status')}): {key}")
    for path in sorted(actual.keys() & recorded.keys()):
        text = actual[path].read_text(encoding="utf-8-sig")
        if semantic_digest(text) != recorded[path].get("semantic_sha256"):
            errors.append(f"SOURCE CHANGED: {path}; rediscover declarations and review domains")
        if source_location_digest(text) != recorded[path].get("source_location_sha256"):
            errors.append(f"SOURCE LOCATIONS CHANGED: {path}; refresh compiler rows and generated field links")
        bindings = recorded[path].get("compiler_input_bindings", {})
        if recorded[path].get("compiler_status") == "observed" and path not in bindings:
            errors.append(f"MISSING COMPILER DEPENDENCY BINDING: {path}")
        for dependency, binding in bindings.items():
            if dependency not in actual:
                errors.append(f"STALE COMPILER DEPENDENCY: {path} includes removed owned input {dependency}")
                continue
            if any(current_bindings[dependency][key] != binding.get(key)
                   for key in ("semantic_sha256", "source_location_sha256")):
                errors.append(f"STALE COMPILER DEPENDENCY: {path} observed with old {dependency}")
        owned_text = mixed_owned_text(text, scope["mixed_files"][path]) if path in scope.get("mixed_files", {}) else text
        fields, unsupported = lexical_fields(owned_text, path)
        for field in fields:
            if row_id(field) not in keys:
                errors.append(f"UNCLASSIFIED FIELD: {row_id(field)}")
        bound = sorted(row_id(row) for row in ledger.get("fields", []) if row["path"] == path)
        if semantic_json_digest(bound) != recorded[path].get("declaration_sha256"):
            errors.append(f"DECLARATION MANIFEST DIFFERS: {path}; a compiler-observed field/variable/parameter was added or removed")
        if require_complete and (recorded[path].get("compiler_status") != "observed" or unsupported):
            errors.append(f"INCOMPLETE COMPILER/CONDITIONAL COVERAGE: {path}")
    if ledger.get("scope_sha256") != semantic_json_digest(scope):
        errors.append("SCOPE CHANGED: refresh after reviewing added/removed owned files")
    if ledger.get("policies_sha256") != semantic_json_digest(policies):
        errors.append("DOMAIN POLICIES CHANGED: regenerate and review the ledger")
    return errors


def render(ledger):
    fields = ledger["fields"]
    counts = {status: sum(r["status"] == status for r in fields) for status in sorted(STATUSES)}
    observed = sum(f["compiler_status"] == "observed" for f in ledger["files"].values())
    lines = ["# Change Requirement 1 field inventory", "",
             "Generated by `python3 testing/compliance/inventory.py --refresh --ast <compiler-observations.json>`; edit reviewed domain policies, not this table.", "",
             f"Baseline: `{ledger['baseline_sha']}`. {len(ledger['files'])} owned C/header files; {len(fields)} data declarations recorded.",
             f"Compiler observations cover {observed} files in the recorded profiles; inactive-branch lexical coverage is supplementary, not a whole-C proof.", "",
             "`--check` verifies source bindings and classification integrity. It can pass while CR1 work remains. `--require-complete` additionally fails on gaps, unresolved domains, unobserved files and unsupported conditional forms.", "",
             "| Disposition | Declarations |", "|---|---:|"]
    lines += [f"| {status} | {count} |" for status, count in counts.items()]
    lines += ["", "Full machine-readable rows, domains and declaration widths are in [`inventory_fields.json`](../../testing/compliance/inventory_fields.json); each row's `policy` links its ownership, API constraints and evidence in [`inventory_policies.json`](../../testing/compliance/inventory_policies.json). The owned-file boundary is [`inventory_scope.json`](../../testing/compliance/inventory_scope.json).", "",
              "Complete generated tables: [`CR1_FIELDS.csv`](CR1_FIELDS.csv) contains every declaration, and [`CR1_RETAINED_FIELDS.md`](CR1_RETAINED_FIELDS.md) focuses on owned retained objects and record members. Unknown ranges, units and synchronization are explicit; neither artifact treats an unknown as compliant.", "",
              "Every tracked/nonignored untracked C/header file is classified. Unclassified paths anywhere in the repository fail the check. [`inventory_imports.json`](../../testing/compliance/inventory_imports.json) records upstream/template/baseline Git blob provenance for the preserved imported files. Mixed kernel/configuration ownership is limited to the explicit macro or balanced conditional regions in the scope manifest; imported declarations outside those regions are excluded and their surrounding content is pinned against changes.", "",
              "Continuous-value precision, invalid-input rejection, sentinels, terminal counters, wire bytes and synchronization remain preservation requirements. Compiler ABI widths and padding are not application minimum-bit proofs.", "",
              "Compiler rows distinguish storage class from C99 storage duration and record external/private function linkage. Automatic arithmetic temporaries and function parameters retain evaluation and validation behavior; these policies do not exempt static locals, globals or record fields. Generic RTOS integer aliases alone do not prove a retained value is an ABI boundary.", "",
              "`address_evidence` records direct address-of expressions seen by the compiler. `not-observed-in-recorded-TUs` is not proof that an object has no aliases or escapes. Before narrowing a retained member, review all owner tasks/ISRs, pointer-compatible API uses and synchronization at the storage-unit level. Callback and retained pointer rules record concrete lifecycle owners where reviewed.", "",
              "Distinct compilation contexts and observed target widths are retained. Width variants cannot establish a minimum-bit pass. Recovery ASTs and headers requiring an enclosing translation unit remain partial observations until a valid context covers them.", "",
              "Each compiler file receipt also binds all owned inputs of its observed translation units. A changed included header invalidates dependent observations, including aggregate sizes in otherwise unchanged C files; refresh affected translation units together. Collection rejects sources changed while parsing.", "",
              "[Historical fixture compilation receipts](evidence/historical-fixture-compilation.json) preserve exact baseline/candidate commands and diagnostics for retained obsolete test interfaces. A historical designation does not turn a failed compiler observation into a pass.", "",
              "## Reviewed packing gaps", "", "| Declaration | Domain | Required / stored bits | Ownership |", "|---|---|---|---|"]
    for row in fields:
        if row["status"] == "gap":
            label = f"{row.get('owner', '')}.{row['name']}".lstrip(".")
            domain = row.get("domain")
            domain_text = f"{domain['min']}..{domain['max']}" if domain and "min" in domain else str(domain)
            lines.append(f"| [{label}](../../{row['path']}#L{row['line']}) | {domain_text} | {row['required_bits']} / {row['stored_bits']} | {row.get('ownership', 'unreviewed')} |")
    lines += ["", "## Source and unresolved-domain coverage", "", "| Owned file | Declarations | Unresolved | Compiler status |", "|---|---:|---:|---|"]
    for path, file in ledger["files"].items():
        selected = [r for r in fields if r["path"] == path]
        lines.append(f"| [{path}](../../{path}) | {len(selected)} | {sum(r['status'] == 'unresolved' for r in selected)} | {file['compiler_status']} |")
    return "\n".join(lines) + "\n"


def table_row(row):
    """Expose reviewed facts without inventing units or interval resolution."""
    domain = row.get("domain") or {}
    numeric = "min" in domain and "max" in domain
    range_text = f"{domain['min']}..{domain['max']}" if numeric else domain.get("description", "unresolved")
    if domain.get("values"):
        range_text += "; reachable values: " + ", ".join(map(str, domain["values"]))
    resolution = domain.get("resolution", "unresolved")
    if numeric and row.get("type_category") not in {"FLOAT", "DOUBLE", "LONGDOUBLE"}:
        resolution = f"exact discrete step {domain.get('step', 1)} in listed units"
    if row.get("type_category") in {"FLOAT", "DOUBLE", "LONGDOUBLE"}:
        resolution = "existing binary floating representation/precision preserved; no fixed-point quantization"
    return {
        "source": row["path"], "line": row["line"], "kind": row["kind"],
        "owner": row.get("owner", "") or "translation unit", "name": row["name"],
        "type": row.get("type", "unresolved"), "range_or_domain": range_text,
        "producer_valid_range": (f"{domain['valid_min']}..{domain['valid_max']}"
                                  if "valid_min" in domain and "valid_max" in domain else "not separately established"),
        "initialized_producer_domain_only": (json.dumps(domain["initialized_domain"], sort_keys=True)
                                             if "initialized_domain" in domain else "not separately established"),
        "units": domain.get("units", "unresolved"), "resolution": resolution,
        "required_encoding": domain.get("encoding", "retain existing numeric representation unless separately reviewed"),
        "required_bits": row.get("required_bits") if row.get("required_bits") is not None else "unresolved / preservation boundary",
        "required_domains_and_bits_by_abi": (json.dumps(row["required_bits_by_abi"], sort_keys=True)
                                             if row.get("required_bits_by_abi") else "not separately applicable / unresolved"),
        "safe_upper_bound_bits_if_minimum_unproved": row.get("safe_upper_bound_bits", "not separately applicable"),
        "stored_bits_per_scalar_or_bitfield": row.get("stored_bits") if row.get("stored_bits") is not None else "unresolved",
        "array_count": row.get("array_count") if row.get("array_count") is not None else "unresolved",
        "declared_type_allocation_bits": row.get("allocation_bits") if row.get("allocation_bits") is not None else "unresolved",
        "observed_storage_width_variants_bits": "; ".join(map(str, row.get("profile_width_variants", []))) or "no differing width observed",
        "compiler_targets": "; ".join(row.get("compiler_targets", [row.get("compiler_target", "unobserved")])),
        "storage_duration": row.get("storage_duration", "unresolved"),
        "ownership": row.get("ownership", "unreviewed"),
        "synchronization": row.get("synchronization", "unreviewed; inspect ownership/constraints before changing storage"),
        "address_observation": row.get("address_taken", "not audited"),
        "address_evidence": "; ".join(row.get("address_evidence", [])),
        "rationale": row.get("reason", "unreviewed"),
        "constraints": "; ".join(row.get("constraints", [])),
        "status": row["status"], "policy": row.get("policy") or "unresolved",
        "method": row.get("method", "unresolved"),
        "profiles": "; ".join(row.get("profiles", [])),
        "evidence": "; ".join(row.get("evidence", [])),
    }


def render_csv(ledger):
    rows = [table_row(row) for row in ledger["fields"]]
    output = io.StringIO(newline="")
    if rows:
        writer = csv.DictWriter(output, fieldnames=list(rows[0]), lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)
    return output.getvalue()


def render_retained(ledger):
    selected = [row for row in ledger["fields"] if not row["path"].startswith(("testing/", "tools/"))
                and (row["kind"] == "FIELD_DECL" or
                     (row["kind"] == "VAR_DECL" and row.get("storage_duration") == "static"))]
    lines = ["# CR1 retained-object and member inventory", "",
             "Generated from the same source-bound ledger as [the inventory summary](CR1_FIELD_INVENTORY.md) and [all declaration rows](CR1_FIELDS.csv). Do not edit this table manually.", "",
             "Record members are listed independently of each instance's lifetime. Static-duration variables include extern declarations and read-only tables; duplicate declarations are not additional allocations. Required bits describe reviewed domains only. Declared-type allocation is not additive for bitfields, unions, arrays or repeated declarations; use target structure/map measurements for actual RAM.", "",
             "A preserved ABI or floating representation does not establish its synchronization. Unreviewed ranges, units and synchronization remain explicit. Direct address observations are positive evidence only; absence does not prove absence of aliases.", "",
             "For reviewed public enum boundaries, the full compatible integer domain and required width are recorded for each clean, explicitly selected ABI. The single required/stored pair is the representative observation; the per-ABI detail preserves all supported widths. Named producer codes alone never establish the domain of caller-mutable storage.", "",
             f"{len(selected)} retained declaration/member rows.", "",
             "| Declaration | Range/domain; units; resolution | Required/stored bits; storage | Ownership and synchronization | Rationale/status |",
             "|---|---|---|---|---|"]
    def escape(value):
        return str(value).replace("|", "&#124;").replace("\n", " ")
    for source in selected:
        row = table_row(source)
        label = f"{row['owner']}.{row['name']}"
        values = [f"[{label}](../../{row['source']}#L{row['line']}) ({row['type']})",
                  f"{row['range_or_domain']}; units: {row['units']}; resolution: {row['resolution']}" +
                  (f"; initialized-only envelope: {row['initialized_producer_domain_only']}"
                   if row['initialized_producer_domain_only'] != "not separately established" else ""),
                  f"{row['required_bits']} / {row['stored_bits_per_scalar_or_bitfield']}; {row['storage_duration']}" +
                  (f"; per-ABI domains/widths: {row['required_domains_and_bits_by_abi']}"
                   if source.get("required_bits_by_abi") else ""),
                  f"{row['ownership']} Sync: {row['synchronization']}",
                  f"{row['rationale']} [{row['status']}; {row['policy']}]"]
        lines.append("| " + " | ".join(escape(value) for value in values) + " |")
    return "\n".join(lines) + "\n"


def generated_artifacts(ledger):
    return {"CR1_FIELD_INVENTORY.md": render(ledger), "CR1_FIELDS.csv": render_csv(ledger),
            "CR1_RETAINED_FIELDS.md": render_retained(ledger)}


def artifact_errors(directory, ledger):
    return [f"GENERATED DOCUMENT STALE: regenerate {name}" for name, content in generated_artifacts(ledger).items()
            if not (directory / name).is_file() or (directory / name).read_text(encoding="utf-8") != content]


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=ROOT)
    parser.add_argument("--check", action="store_true")
    parser.add_argument("--refresh", action="store_true")
    parser.add_argument("--require-complete", action="store_true")
    parser.add_argument("--ast", type=Path, help="compiler observations from inventory_ast.py")
    args = parser.parse_args(argv)
    root = args.root.resolve()
    folder = root / "testing/compliance"
    scope = read_json(folder / "inventory_scope.json")
    policies = read_json(folder / "inventory_policies.json")
    target = folder / "inventory_fields.json"
    document = root / "docs/compliance/CR1_FIELD_INVENTORY.md"
    if args.refresh:
        if target.exists() and args.ast is None and any(
                f.get("compiler_status") == "observed" for f in read_json(target).get("files", {}).values()):
            parser.error("--refresh needs --ast after compiler observations exist; refusing to silently replace them with lexical-only coverage")
        ledger = collect(root, scope, policies, read_json(args.ast) if args.ast else None)
        compact = compact_ledger(ledger)
        # One declaration per line makes the large complete ledger reviewable,
        # while factoring repeated prose avoids multi-megabyte duplication.
        prefix = {k: v for k, v in compact.items() if k != "fields"}
        serialized = json.dumps(prefix, indent=2)[:-2] + ',\n  "fields": [\n'
        serialized += ",\n".join("    " + json.dumps(r, separators=(",", ":")) for r in compact["fields"])
        target.write_text(serialized + "\n  ]\n}\n", encoding="utf-8")
        document.parent.mkdir(parents=True, exist_ok=True)
        for name, content in generated_artifacts(ledger).items():
            (document.parent / name).write_text(content, encoding="utf-8", newline="\n")
    else:
        ledger = expand_ledger(read_json(target), policies)
    errors = validate(root, scope, policies, ledger, args.require_complete)
    errors.extend(artifact_errors(document.parent, ledger))
    for error in errors:
        print(error)
    open_fields = sum(r["status"] in {"unresolved", "gap"} for r in ledger["fields"])
    print(f"{'FAIL' if errors else 'PASS'}: inventory integrity; {len(ledger['files'])} files, {len(ledger['fields'])} declarations; {open_fields} open CR1 fields.")
    if open_fields:
        print("CR1 completion: INCOMPLETE. An integrity pass is not a compliance certificate.")
    return bool(errors)


if __name__ == "__main__":
    raise SystemExit(main())
