#!/usr/bin/env python3
"""Observe owned function definitions with actual compiler contexts/profiles.

This is an inventory, not proof that all conditional functions are observed or
that a comment states the correct contract. No C object is linked or executed.
"""
from __future__ import annotations

import argparse
from collections import Counter
from concurrent.futures import ThreadPoolExecutor
import hashlib
import importlib.metadata
import json
from pathlib import Path
import re
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from testing.compliance import inventory, inventory_ast
from tools.compliance import compiler_contexts


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
                "errors": [str(error)], "diagnostics": [str(error)], "declarations": [], "seen": []}
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
                     "canonical_type": cursor.type.get_canonical().spelling,
                     "method": "libclang-recovery" if errors else "libclang-ast",
                     "profiles": [profile], "parameters": [item.spelling for item in cursor.get_arguments()],
                     "definition_documented": function_documented(cursor.raw_comment),
                     "definition_comment": cursor.raw_comment,
                     "canonical_declaration_file": declaration_file,
                     "canonical_declaration_line": canonical.location.line,
                     "canonical_declaration_documented": function_documented(canonical.raw_comment)})
    return {"source": source.relative_to(root).as_posix(), "profile": profile,
            "errors": errors, "diagnostics": [str(item) for item in unit.diagnostics],
            "declarations": rows, "seen": sorted(seen)}


def merge_function(rows, incoming):
    """Choose an observed representation while retaining every context outcome."""
    key = (incoming['path'], incoming['line'], incoming['name'])
    if key not in rows:
        rows[key] = dict(incoming)
        rows[key]['context_observations'] = list(incoming['context_observations'])
        return
    saved = rows[key]
    observations = {item['context_id']: item for item in saved['context_observations']}
    for item in incoming['context_observations']:
        if item['context_id'] in observations and observations[item['context_id']] != item:
            raise ValueError('Conflicting function observations for the same context')
        observations[item['context_id']] = item
    profiles = sorted(set(saved['profiles']+incoming['profiles']))
    priority = {'libclang-recovery': 0, 'libclang-fallback': 1, 'libclang-derived': 2, 'libclang-ast': 3}
    if priority[incoming['method']] > priority[saved['method']]:
        saved.update(incoming)
    saved['profiles'] = profiles
    saved['context_observations'] = [observations[key] for key in sorted(observations)]


def add_counts(report):
    functions = report['functions']
    report['definition_count'] = len(functions)
    report['documented_definition_count'] = sum(row['definition_documented'] for row in functions)
    report['error_free_definition_count'] = sum(row['method'] == 'libclang-ast' for row in functions)
    report['files_with_definitions'] = len({row['path'] for row in functions})
    report['context_counts'] = dict(Counter(row['origin'] for row in report['translation_units']))
    report['diagnostic_translation_unit_count'] = sum(bool(row['errors']) for row in report['translation_units'])
    report['recovery_function_observation_count'] = sum(
        item['method'] == 'libclang-recovery' for row in functions for item in row['context_observations'])
    report['coverage_complete'] = False
    return report


def collect(root, databases, profiles, jobs, *, alternatives=(), recorded_only=False,
            profile_label='local-observation', public_config_output=None):
    root = root.resolve()
    scope_path = root/'testing/compliance/inventory_scope.json'
    tracked_inputs = [scope_path, *databases]
    if public_config_output:
        tracked_inputs.append(root/'config/wifi_secrets.example.h')
    input_bindings = compiler_contexts.input_snapshot(tracked_inputs)
    scope = inventory.read_json(scope_path)
    inputs = set(inventory.owned_files(root, scope))
    bindings = compiler_contexts.source_bindings(root, inputs)
    source_revision = compiler_contexts.revision(root)
    mapped_roots = compiler_contexts.equivalent_roots(root, alternatives, inputs)
    resource, resource_receipt = compiler_contexts.pinned_resource()
    extra, derivations = [], []
    if public_config_output:
        extra, derivation = compiler_contexts.public_configuration(root, public_config_output)
        template_key = str((root/'config/wifi_secrets.example.h').resolve())
        if derivation['template_bytes_sha256'] != input_bindings[template_key]:
            raise ValueError('Public template changed before compiler observation')
        derivations.append(derivation)
        input_bindings.update(compiler_contexts.input_snapshot([Path(derivation['derived_path'])]))
    planned = compiler_contexts.plan(root, databases, inputs, profiles, resource,
        mapped_roots, recorded_only, extra, derivations)
    for _, metadata in planned:
        if metadata['database_sha256'] != input_bindings[metadata['database']]:
            raise ValueError('Compile database changed before context planning')
    compiler_contexts.verify_input_snapshot(input_bindings)
    files = {name: {**binding, 'status': 'unobserved', 'profiles': [], 'input_bindings': {},
                    'context_ids': []} for name, binding in bindings.items()}
    receipts, rows = [], {}
    with ThreadPoolExecutor(max_workers=jobs) as executor:
        parsed = executor.map(lambda item: parse_functions(item[0], root, inputs, {}), planned)
        for index, ((job, metadata), result) in enumerate(zip(planned, parsed), 1):
            context_id = hashlib.sha256((profile_label+metadata['context_id']).encode()).hexdigest()
            origin = metadata['origin']
            context = {**metadata, **{key: value for key, value in result.items() if key not in {'declarations', 'seen'}},
                       'context_id': context_id, 'collector_profile': profile_label,
                       'compiler_target': next((arg.split('=', 1)[1] for arg in job[1] if arg.startswith('--target=')), 'host-default'),
                       'owned_inputs': result['seen']}
            receipts.append(context)
            clean_recorded = not result['errors'] and origin == 'recorded-command'
            method = ('libclang-recovery' if result['errors'] else 'libclang-ast' if clean_recorded
                      else 'libclang-derived' if origin == 'derived-profile' else 'libclang-fallback')
            for name in result['seen']:
                file = files[name]
                file['context_ids'].append(context_id)
                file['input_bindings'].update({path: bindings[path] for path in result['seen']})
                file['profiles'] = sorted(set(file['profiles']+[profile_label+':'+job[2]]))
                if clean_recorded:
                    file['status'] = 'observed'
                elif file['status'] == 'unobserved':
                    file['status'] = 'partial'
            for row in result['declarations']:
                row['method'] = method
                row['context_observations'] = [{'context_id': context_id, 'origin': origin,
                    'profile': profile_label+':'+job[2], 'method': method,
                    'canonical_type': row['canonical_type'],
                    'canonical_declaration_file': row['canonical_declaration_file'],
                    'canonical_declaration_line': row['canonical_declaration_line'],
                    'canonical_declaration_documented': row['canonical_declaration_documented']}]
                merge_function(rows, row)
            if index % 50 == 0:
                print(f'Parsed {index}/{len(planned)} function contexts', flush=True)
    if bindings != compiler_contexts.source_bindings(root, inputs) or source_revision != compiler_contexts.revision(root):
        raise ValueError('Source text changed during function-documentation observation')
    compiler_contexts.verify_input_snapshot(input_bindings)
    if public_config_output and ((root/'config/wifi_secrets.h').exists() or (root/'config/wifi_secrets.h').is_symlink()):
        raise ValueError('Source network settings appeared during compiler observation')
    observed = {'schema_version': 3, 'revision': source_revision, 'source_root': str(root),
        'scope_sha256': input_bindings[str(scope_path.resolve())],
        'context_input_bindings': input_bindings, 'context_inputs_unchanged':True,
        'collector_profile': profile_label,
        'compiler': 'libclang 18.1.1 Python binding', 'libclang_package': importlib.metadata.version('libclang'),
        'resource_headers': resource_receipt, 'recorded_only': recorded_only,
        'public_configuration': derivations, 'profiles': {name: inventory_ast.PROFILES[name] for name in profiles},
        'translation_units': receipts, 'files': files, 'functions': [rows[key] for key in sorted(rows)]}
    observed['limits'] = [
        "Observed definition variants are keyed by path, source line and name; profiles are unioned.",
        "Every failed, derived or fallback context remains explicit even when another recorded context parses cleanly.",
        "Recovery ASTs, synthetic profile combinations, fallback parses and unobserved declarations remain incomplete.",
        "Compiler proxy receipts record invocations without original compiler exit codes; libclang-ast means parser success only.",
        "Comment recognition does not validate contract meaning or generated Doxygen output.",
        "Mixed imported file bodies are excluded using the shared owned-region selectors.",
        "No compiled C test, target binary or hardware action is executed."]
    return add_counts(observed)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=ROOT)
    parser.add_argument('--equivalent-root', type=Path, action='append', default=[])
    parser.add_argument('--recorded-only', action='store_true')
    parser.add_argument('--profile-label', default='local-observation')
    parser.add_argument('--public-config-output', type=Path)
    parser.add_argument("--compile-db", type=Path, action="append", required=True)
    parser.add_argument("--profiles", nargs="+", choices=inventory_ast.PROFILES, default=list(inventory_ast.PROFILES))
    parser.add_argument("--jobs", type=int, default=4)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    report = {'status':'INCOMPLETE', 'collector_profile':args.profile_label,
              'coverage_complete':False, 'errors':[]}
    try:
        report.update(collect(args.root, args.compile_db, args.profiles, args.jobs,
            alternatives=args.equivalent_root, recorded_only=args.recorded_only,
            profile_label=args.profile_label, public_config_output=args.public_config_output))
        report['status'] = 'OBSERVATIONS_COLLECTED'
    except (OSError, ValueError, KeyError, ImportError, subprocess.SubprocessError) as error:
        report['errors'].append(str(error))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({key: report.get(key) for key in ('status','errors',"definition_count", "documented_definition_count",
          "error_free_definition_count", "files_with_definitions", "coverage_complete")}, indent=2))
    if 'translation_units' in report:
        print("Diagnostic translation units:", sum(bool(row["errors"]) for row in report["translation_units"]))
        print("Definitions by directory:", dict(Counter(str(Path(row["path"]).parent) for row in report["functions"])))
    return 0 if report['status']=='OBSERVATIONS_COLLECTED' else 2


if __name__ == "__main__":
    sys.exit(main())
