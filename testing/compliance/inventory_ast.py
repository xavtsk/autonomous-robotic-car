#!/usr/bin/env python3
"""Collect compiler-derived CR1 declaration observations; never link/run tests.

Requires the optional pinned `libclang==18.1.1` Python package. Feed the actual
compile database to retain ABI flags/includes. Every diagnostic and observed
profile is recorded: recovery ASTs cannot establish complete compiler coverage.
The checked-in ledger remains usable without this dependency via inventory.py.
"""
from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor
import json
import os
from pathlib import Path
import re
import shlex
import shutil
import subprocess
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))
import inventory

PROFILES = {
    "configured": {},
    "diagnostics": {"APP_IR_DEBUG": 1, "APP_IMU_DEBUG": 1, "APP_DIAGNOSTICS": 1},
    "selftest": {"APP_TEST": 1, "APP_S3_MOTION": 0, "APP_IR_BENCH": 0,
                 "APP_MOTION_SMOKE": 0, "APP_S4": 1, "APP_S5": 1},
    "disabled": {"APP_TEST": 0, "APP_S3_MOTION": 0, "APP_IR_BENCH": 0,
                 "APP_MOTION_SMOKE": 0, "APP_S4": 0, "APP_S5": 0,
                 "APP_IR_DEBUG": 0, "APP_IMU_DEBUG": 0, "APP_DIAGNOSTICS": 0},
    "bench": {"APP_TEST": 0, "APP_S3_MOTION": 0, "APP_IR_BENCH": 1},
    "smoke": {"APP_TEST": 0, "APP_S3_MOTION": 0, "APP_MOTION_SMOKE": 1},
}


def clean_arguments(arguments, directory):
    """Strip outputs/source operands, resolve include paths without chdir."""
    output, skip = [], False
    paired = {"-o", "-MF", "-MT", "-MQ"}
    path_flags = {"-I", "-isystem", "-iquote", "-include", "-imacros", "-isysroot"}
    index = 1
    while index < len(arguments):
        arg = arguments[index]
        if arg in paired:
            index += 2
            continue
        if arg in {"-c", "-MMD", "-MD", "-MP", "-pthread"} or re.match(r"^-Wl,|^-l|^-L", arg):
            index += 1
            continue
        if arg in path_flags and index + 1 < len(arguments):
            value = Path(arguments[index + 1])
            output += [arg, str(value if value.is_absolute() else (directory / value).resolve())]
            index += 2
            continue
        if arg.startswith("-I") and len(arg) > 2:
            value = Path(arg[2:])
            output.append("-I" + str(value if value.is_absolute() else (directory / value).resolve()))
        elif not arg.startswith("-") and Path(arg).suffix.lower() in {".c", ".h", ".o", ".obj", ".a"}:
            pass
        else:
            output.append(arg)
        index += 1
    return output


def compiler_include_arguments(compiler):
    """Recover Arm GCC implicit C headers without running a target program."""
    executable = Path(shutil.which(compiler) or compiler)
    result = []
    if "arm-none-eabi" in executable.name:
        prefix = executable.resolve().parent.parent
        result.append("--target=arm-none-eabi")
        candidates = sorted((prefix / "lib/gcc/arm-none-eabi").glob("*/include"))
        candidates += [prefix / "arm-none-eabi/include"]
        for path in candidates:
            if path.is_dir():
                result += ["-isystem", str(path)]
    elif os.name == "nt" and executable.name in {"gcc.exe", "cc.exe"}:
        prefix = executable.resolve().parent.parent
        candidates = sorted((prefix / "lib/gcc").glob("*/*/include"))
        candidates += [prefix / "include"]
        result.append("--target=x86_64-w64-windows-gnu")
        for path in candidates:
            if path.is_dir():
                result += ["-isystem", str(path)]
    clang = shutil.which("clang")
    if clang:
        resource = subprocess.run([clang, "-print-resource-dir"], capture_output=True,
                                  text=True, check=True).stdout.strip()
        if resource:
            result += ["-resource-dir", resource]
    return result


def profile_arguments(arguments, overrides):
    changed = set(overrides)
    result = [arg for arg in arguments
              if not (arg.startswith(("-D", "-U")) and arg[2:].split("=")[0] in changed)]
    result += [f"-D{name}={value}" for name, value in overrides.items()]
    return result


def scalar_type(type_value):
    canonical = type_value.get_canonical()
    count = 1
    while canonical.kind.name in {"CONSTANTARRAY", "INCOMPLETEARRAY", "VARIABLEARRAY"}:
        size = canonical.get_array_size()
        count = count * size if size >= 0 and count is not None else None
        canonical = canonical.get_array_element_type().get_canonical()
    return canonical, count


def storage_duration(kind, parent_kind, storage):
    """C99 storage duration, kept separate from Clang's storage-class token."""
    if kind == "PARM_DECL":
        return "automatic"
    if kind == "VAR_DECL":
        return "automatic" if parent_kind == "FUNCTION_DECL" and storage not in {"STATIC", "EXTERN"} else "static"
    return "enclosing-object" if kind == "FIELD_DECL" else "none"


def parse_one(job, root, owned, lexical_index):
    from clang import cindex
    source, arguments, profile = job
    target = next((arg.split("=", 1)[1] for arg in arguments if arg.startswith("--target=")), "host-default")
    try:
        unit = cindex.Index.create().parse(str(source), args=arguments)
    except cindex.TranslationUnitLoadError as error:
        return {"source": source.relative_to(root).as_posix(), "profile": profile,
                "arguments": arguments, "compiler_target": target,
                "errors": [str(error)], "declarations": [], "seen": []}
    errors = [str(d) for d in unit.diagnostics if d.severity >= cindex.Diagnostic.Error]
    declarations, seen = [], {source.relative_to(root).as_posix()}
    for inclusion in unit.get_includes():
        file = Path(inclusion.include.name).resolve()
        if file in owned:
            seen.add(file.relative_to(root).as_posix())
    location_cache = {}

    def owned_path(cursor):
        if not cursor.location.file:
            return None
        name = cursor.location.file.name
        if name not in location_cache:
            file = Path(name).resolve()
            location_cache[name] = file.relative_to(root).as_posix() if file in owned else None
        return location_cache[name]

    def walk_owned(cursor):
        # Imported declarations cannot contain application definitions. Prune
        # their subtrees instead of traversing every SDK/libc implementation.
        for child in cursor.get_children():
            if owned_path(child) is not None:
                yield child
                yield from walk_owned(child)
    cursors = list(walk_owned(unit.cursor))
    address_uses = {}
    for cursor in cursors:
        if cursor.kind.name != "UNARY_OPERATOR":
            continue
        tokens = iter(cursor.get_tokens())
        first = next(tokens, None)
        if first is None or first.spelling != "&":
            continue
        for child in cursor.walk_preorder():
            if child.kind.name not in {"DECL_REF_EXPR", "MEMBER_REF_EXPR"} or not child.referenced:
                continue
            key = child.referenced.get_usr()
            if key:
                address_uses.setdefault(key, set()).add(
                    f"{owned_path(cursor)}:{cursor.location.line}")
    kinds = {"FIELD_DECL", "VAR_DECL", "PARM_DECL", "ENUM_CONSTANT_DECL"}
    for cursor in cursors:
        path = owned_path(cursor)
        seen.add(path)
        if cursor.kind.name not in kinds or not cursor.spelling:
            continue
        parent = cursor.semantic_parent
        owner = parent.spelling if parent and parent.kind.name != "TRANSLATION_UNIT" else ""
        if cursor.kind.name == "FIELD_DECL":
            fallback = lexical_index.get((path, cursor.spelling, cursor.location.line, cursor.location.column))
            if fallback:
                owner = fallback
        if cursor.kind.name == "VAR_DECL" and parent and parent.kind.name == "FUNCTION_DECL":
            owner += f"#L{cursor.location.line}"
        scalar, count = scalar_type(cursor.type)
        width = cursor.get_bitfield_width() if cursor.kind.name == "FIELD_DECL" and cursor.is_bitfield() else None
        scalar_size = scalar.get_size()
        declared_size = cursor.type.get_size()
        row = {"path": path, "owner": owner, "name": cursor.spelling,
               "kind": cursor.kind.name, "line": cursor.location.line, "column": cursor.location.column,
               "type": cursor.type.spelling, "canonical_type": scalar.spelling,
               "type_category": scalar.kind.name, "array_count": count,
               "stored_bits": width if width is not None else scalar_size * 8 if scalar_size >= 0 else None,
               "allocation_bits": declared_size * 8 if declared_size >= 0 else None,
               "method": "libclang-recovery" if errors else "libclang-ast",
               "profiles": [profile], "storage": cursor.storage_class.name,
               "storage_duration": storage_duration(cursor.kind.name, parent.kind.name if parent else "", cursor.storage_class.name),
               "linkage": cursor.linkage.name,
               "function_linkage": parent.linkage.name if parent and parent.kind.name == "FUNCTION_DECL" else "not-function",
               "compiler_target": target,
               "address_taken": "observed-direct" if cursor.get_usr() in address_uses else "not-observed-in-recorded-TUs",
               "address_evidence": sorted(address_uses.get(cursor.get_usr(), set()))}
        if cursor.kind.name == "ENUM_CONSTANT_DECL":
            row["enum_value"] = cursor.enum_value
        if scalar.kind.name == "ENUM":
            row["enumerators"] = {child.spelling: child.enum_value
                                  for child in scalar.get_declaration().get_children()
                                  if child.kind.name == "ENUM_CONSTANT_DECL"}
        declarations.append(row)
    return {"source": source.relative_to(root).as_posix(), "profile": profile,
            "arguments": arguments, "compiler_target": target,
            "errors": errors, "declarations": declarations, "seen": sorted(seen)}


def collect(root, databases, profiles, jobs_count=4, limit_paths=None):
    root = root.resolve()
    scope = inventory.read_json(root / "testing/compliance/inventory_scope.json")
    owned = set(inventory.owned_files(root, scope))
    # Bind every observation to all owned inputs in its translation unit. A
    # header change can alter a global's layout without changing the .c file.
    input_bindings = {}
    lexical_index = {}
    for file in owned:
        text = file.read_text(encoding="utf-8-sig")
        path = file.relative_to(root).as_posix()
        input_bindings[path] = {"semantic_sha256": inventory.semantic_digest(text),
                                "source_location_sha256": inventory.source_location_digest(text)}
        rows, _ = inventory.lexical_fields(text, path)
        lexical_index.update({(r["path"], r["name"], r["line"], r.get("column")): r["owner"] for r in rows})
    commands, common = {}, None
    for database in databases:
        entries = ([json.loads(line) for line in database.read_text(encoding="utf-8").splitlines() if line.strip()]
                   if database.suffix == ".jsonl" else inventory.read_json(database))
        for entry in entries:
            directory = Path(entry.get("directory", entry.get("cwd", root))).resolve()
            arguments = entry.get("arguments", entry.get("command"))
            if isinstance(arguments, str):
                arguments = shlex.split(arguments)
            if not arguments:
                continue
            clean = clean_arguments(arguments, directory) + compiler_include_arguments(arguments[0])
            if common is None:
                common = clean
            sources = [entry["file"]] if "file" in entry else [arg for arg in arguments if arg.endswith(".c")]
            for name in sources:
                path = Path(name)
                path = (path if path.is_absolute() else directory / path).resolve()
                if path in owned:
                    contexts = commands.setdefault(path, [])
                    if clean not in contexts:
                        contexts.append(clean)
    if common is None:
        raise ValueError("Compile database contained no usable compiler arguments")
    jobs = []
    for source in sorted(owned):
        if limit_paths and not any(source.relative_to(root).as_posix().startswith(p) for p in limit_paths):
            continue
        host_only = source.relative_to(root).parts[0] in {"testing", "tools"}
        contexts = list(commands.get(source, []))
        if not contexts or (not host_only and common not in contexts):
            contexts.insert(0, common)
        for args in contexts:
            selected_profiles = ["configured"] if host_only or args != common else profiles
            for profile in selected_profiles:
                # Headers are parsed as C translation units; no object is produced.
                jobs.append((source, profile_arguments([*args, "-x", "c"], PROFILES[profile]), profile))
    rows, files, receipts = {}, {}, []
    with ThreadPoolExecutor(max_workers=jobs_count) as executor:
        results = executor.map(lambda job: parse_one(job, root, owned, lexical_index), jobs)
        for index, result in enumerate(results, 1):
            receipts.append({k: v for k, v in result.items() if k not in {"declarations", "seen"}})
            receipts[-1]["owned_inputs"] = result["seen"]
            for path in result["seen"]:
                receipt = files.setdefault(path, {"profiles": [], "status": "partial",
                    **input_bindings[path], "input_bindings": {}})
                receipt["input_bindings"].update({dependency: input_bindings[dependency]
                                                  for dependency in result["seen"]})
                if result["profile"] not in receipt["profiles"]:
                    receipt["profiles"].append(result["profile"])
                if not result["errors"]:
                    receipt["status"] = "observed"
            for row in result["declarations"]:
                key = inventory.row_id(row)
                if key not in rows:
                    rows[key] = row
                else:
                    saved = rows[key]
                    merged_profiles = sorted(set(saved["profiles"] + row["profiles"]))
                    merged_addresses = sorted(set(saved.get("address_evidence", []) + row.get("address_evidence", [])))
                    merged_targets = sorted(set(saved.get("compiler_targets", [saved.get("compiler_target")]) + [row.get("compiler_target")]))
                    widths = sorted(set(saved.get("profile_width_variants", []) +
                        [v for v in (saved.get("stored_bits"), row.get("stored_bits")) if v is not None]))
                    if saved["method"] == "libclang-recovery" and row["method"] == "libclang-ast":
                        saved.update({k: v for k, v in row.items() if k != "profiles"})
                    saved["profiles"] = merged_profiles
                    saved["compiler_targets"] = merged_targets
                    saved["address_evidence"] = merged_addresses
                    saved["address_taken"] = "observed-direct" if merged_addresses else "not-observed-in-recorded-TUs"
                    if len(widths) > 1:
                        saved["profile_width_variants"] = widths
            if index % 50 == 0:
                print(f"Parsed {index}/{len(jobs)} translation units", flush=True)
    for path, before in input_bindings.items():
        text = (root / path).read_text(encoding="utf-8-sig")
        after = {"semantic_sha256": inventory.semantic_digest(text),
                 "source_location_sha256": inventory.source_location_digest(text)}
        if after != before:
            raise ValueError(f"Compiler input changed during collection: {path}; recollect frozen sources")
    return {"schema_version": 2, "compiler": "libclang 18.1.1 Python binding",
            "profiles": {p: PROFILES[p] for p in profiles}, "translation_units": receipts,
            "files": files, "declarations": [rows[k] for k in sorted(rows)]}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=inventory.ROOT)
    parser.add_argument("--compile-db", type=Path, action="append", required=True)
    parser.add_argument("--profiles", nargs="+", choices=PROFILES, default=list(PROFILES))
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--jobs", type=int, default=4)
    parser.add_argument("--paths", nargs="+")
    args = parser.parse_args(argv)
    report = collect(args.root, args.compile_db, args.profiles, args.jobs, args.paths)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    failures = sum(bool(t["errors"]) for t in report["translation_units"])
    print(f"Compiler observations: {len(report['declarations'])} declarations; {failures} translation units have diagnostics (explicit partial coverage).")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
