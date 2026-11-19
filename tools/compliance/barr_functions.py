#!/usr/bin/env python3
"""Observe owned function definitions with actual compiler contexts/profiles.

This is an inventory, not proof that all conditional functions are observed or
that a comment states the correct contract. No C object is linked or executed.
"""
from __future__ import annotations

import argparse
from collections import Counter
import importlib.metadata
import json
from pathlib import Path
import re
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from testing.compliance import inventory, inventory_ast


def function_documented(comment):
    """A module-level @file comment is not a function contract."""
    return bool(comment and not re.search(r"(?:@|\\)file\b", comment))


def parse_functions(job, root, owned, lexical_index):
    from clang import cindex
    source, arguments, profile = job
    try:
        unit = cindex.Index.create().parse(str(source), args=arguments)
    except cindex.TranslationUnitLoadError as error:
        return {"source": source.relative_to(root).as_posix(), "profile": profile,
                "errors": [str(error)], "declarations": [], "seen": []}
    errors = [str(item) for item in unit.diagnostics if item.severity >= cindex.Diagnostic.Error]
    seen = {source.relative_to(root).as_posix()}
    for inclusion in unit.get_includes():
        path = Path(inclusion.include.name).resolve()
        if path in owned:
            seen.add(path.relative_to(root).as_posix())
    scope = inventory.read_json(root / "testing/compliance/inventory_scope.json")
    allowed = {}
    for name, region in scope.get("mixed_files", {}).items():
        text = (root / name).read_text(encoding="utf-8-sig")
        masked = inventory.mixed_owned_text(text, region)
        allowed[name] = {line for line, value in enumerate(masked.splitlines(), 1) if value.strip()}
    rows = []
    for cursor in unit.cursor.get_children():
        if cursor.kind.name != "FUNCTION_DECL" or not cursor.is_definition() or not cursor.location.file:
            continue
        path = Path(cursor.location.file.name).resolve()
        if path not in owned:
            continue
        name = path.relative_to(root).as_posix()
        if name in allowed and cursor.location.line not in allowed[name]:
            continue
        canonical = cursor.canonical
        declaration_file = str(canonical.location.file) if canonical.location.file else None
        rows.append({"path": name, "kind": "FUNCTION_DECL", "owner": f"line_{cursor.location.line}",
                     "name": cursor.spelling, "usr": cursor.get_usr(),
                     "line": cursor.location.line, "end_line": cursor.extent.end.line,
                     "linkage": cursor.linkage.name, "storage": cursor.storage_class.name,
                     "method": "libclang-recovery" if errors else "libclang-ast",
                     "profiles": [profile], "parameters": [item.spelling for item in cursor.get_arguments()],
                     "definition_documented": function_documented(cursor.raw_comment),
                     "definition_comment": cursor.raw_comment,
                     "canonical_declaration_file": declaration_file,
                     "canonical_declaration_line": canonical.location.line,
                     "canonical_declaration_documented": function_documented(canonical.raw_comment)})
    return {"source": source.relative_to(root).as_posix(), "profile": profile,
            "errors": errors, "declarations": rows, "seen": sorted(seen)}


def collect(root, databases, profiles, jobs):
    # Reuse the inventory's exact compiler-context and profile enumeration,
    # including its explicit diagnostic receipts and source bindings.
    previous = inventory_ast.parse_one
    inventory_ast.parse_one = parse_functions
    try:
        observed = inventory_ast.collect(root, databases, profiles, jobs)
    finally:
        inventory_ast.parse_one = previous
    functions = observed.pop("declarations")
    observed["functions"] = functions
    observed["definition_count"] = len(functions)
    observed["documented_definition_count"] = sum(row["definition_documented"] for row in functions)
    observed["error_free_definition_count"] = sum(row["method"] == "libclang-ast" for row in functions)
    observed["files_with_definitions"] = len({row["path"] for row in functions})
    observed["libclang_package"] = importlib.metadata.version("libclang")
    observed["coverage_complete"] = False
    observed["limits"] = [
        "Observed definition variants are keyed by path, source line and name; profiles are unioned.",
        "Recovery ASTs, unsupported conditional combinations and unobserved declarations remain open.",
        "Comment recognition does not validate contract meaning or generated Doxygen output.",
        "Mixed imported file bodies are excluded using the shared owned-region selectors.",
        "No compiled C test, target binary or hardware action is executed."]
    return observed


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--compile-db", type=Path, action="append", required=True)
    parser.add_argument("--profiles", nargs="+", choices=inventory_ast.PROFILES, default=list(inventory_ast.PROFILES))
    parser.add_argument("--jobs", type=int, default=4)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    report = collect(ROOT, args.compile_db, args.profiles, args.jobs)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({key: report[key] for key in ("definition_count", "documented_definition_count",
          "error_free_definition_count", "files_with_definitions", "coverage_complete")}, indent=2))
    print("Diagnostic translation units:", sum(bool(row["errors"]) for row in report["translation_units"]))
    print("Definitions by directory:", dict(Counter(str(Path(row["path"]).parent) for row in report["functions"])))
    return 0


if __name__ == "__main__":
    sys.exit(main())
