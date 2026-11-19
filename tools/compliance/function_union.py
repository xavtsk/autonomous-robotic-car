#!/usr/bin/env python3
"""Union exact-revision function contexts; unresolved observations remain visible."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import re
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from testing.compliance import inventory
from tools.compliance import compiler_contexts
from tools.compliance.barr_functions import add_counts, merge_function

EXPECTED = {'target-production', 'host-gcc', 'host-clang'}
BINDINGS = ('semantic_sha256', 'source_location_sha256', 'source_content_sha256')


def context_method(context):
    origin = context['origin']
    if origin == 'recorded-command' and (context.get('recorded_source') is not True or
            context.get('profile') != 'configured' or context.get('profile_overrides') != {}):
        raise ValueError('Recorded context contradicts source/profile derivation metadata')
    if origin == 'derived-profile' and (context.get('recorded_source') is not True or
            context.get('profile') == 'configured' or not context.get('profile_overrides')):
        raise ValueError('Derived profile lacks explicit source/override metadata')
    if origin.startswith('fallback-') and context.get('recorded_source') is not False:
        raise ValueError('Fallback context incorrectly claims a recorded source')
    if context['errors']:
        return 'libclang-recovery'
    if context['origin'] == 'recorded-command':
        return 'libclang-ast'
    if context['origin'] == 'derived-profile':
        return 'libclang-derived'
    if context['origin'] in {'fallback-standalone-header', 'fallback-unrecorded-source'}:
        return 'libclang-fallback'
    raise ValueError('Unknown compiler context origin')


def combine(reports, revision, bindings, scope_hash):
    labels = [report.get('collector_profile') for report in reports]
    if len(labels) != len(EXPECTED) or set(labels) != EXPECTED:
        raise ValueError('Require exactly target-production, host-gcc and host-clang function receipts')
    files = {name: {**binding, 'status': 'unobserved', 'profiles': [],
                    'input_bindings': {}, 'context_ids': []} for name, binding in bindings.items()}
    contexts, rows, evidence = {}, {}, []
    for report in reports:
        label = report['collector_profile']
        if report.get('schema_version') != 3 or report.get('revision') != revision or report.get('scope_sha256') != scope_hash:
            raise ValueError('Compiler receipt schema/revision/scope differs: '+label)
        resource = report.get('resource_headers', {})
        if report.get('libclang_package') != '18.1.1' or not re.search(r'\bversion 18\.1\.', resource.get('version', '')) or not re.fullmatch(r'[0-9a-f]{64}', resource.get('stdint_header_sha256', '')):
            raise ValueError('Compiler receipt lacks pinned parser/resource header provenance')
        if set(report['files']) != set(bindings):
            raise ValueError('Compiler receipt owned file set differs: '+label)
        if label.startswith('host-') and report.get('recorded_only') is not True:
            raise ValueError('Host function observations must use recorded commands only')
        for name, expected in bindings.items():
            incoming = report['files'][name]
            if any(incoming.get(key) != expected[key] for key in BINDINGS):
                raise ValueError('Stale compiler input binding: '+name)
            for dependency, binding in incoming['input_bindings'].items():
                if dependency not in bindings or any(binding.get(key) != bindings[dependency][key] for key in BINDINGS):
                    raise ValueError('Stale included-header binding: '+dependency)
        report_contexts = {}
        for context in report['translation_units']:
            identity = context['context_id']
            if identity in contexts or identity in report_contexts or context['collector_profile'] != label:
                raise ValueError('Duplicate or mismatched compiler context')
            if not context.get('original_arguments') or not context.get('arguments') or not context.get('database_sha256'):
                raise ValueError('Compiler context lacks exact command provenance')
            method = context_method(context)
            if label.startswith('host-') and context['origin'] != 'recorded-command':
                raise ValueError('Synthetic host context cannot be treated as recorded evidence')
            for name in context['owned_inputs']:
                if name not in bindings:
                    raise ValueError('Compiler observation references an unbound input')
                files[name]['context_ids'].append(identity)
                files[name]['profiles'].append(label+':'+context['profile'])
                files[name]['input_bindings'].update({path: bindings[path] for path in context['owned_inputs']})
                if method == 'libclang-ast':
                    files[name]['status'] = 'observed'
                elif files[name]['status'] == 'unobserved':
                    files[name]['status'] = 'partial'
            report_contexts[identity] = context
        for row in report['functions']:
            if not row.get('context_observations'):
                raise ValueError('Function lacks explicit compiler-context observations')
            for observation in row['context_observations']:
                context = report_contexts.get(observation['context_id'])
                if context is None or row['path'] not in context['owned_inputs']:
                    raise ValueError('Function refers to an unknown/unbound context')
                if observation['method'] != context_method(context) or observation['origin'] != context['origin']:
                    raise ValueError('Function context method contradicts diagnostics or provenance')
            # An aggregate method cannot silently upgrade a fallback/recovery row.
            expected = max((item['method'] for item in row['context_observations']),
                key=lambda method: {'libclang-recovery':0, 'libclang-fallback':1, 'libclang-derived':2, 'libclang-ast':3}[method])
            if row['method'] != expected:
                raise ValueError('Aggregate function method is not supported by its contexts')
            merge_function(rows, row)
        contexts.update(report_contexts)
        evidence.append({'collector_profile': label, 'resource_headers': report['resource_headers'],
                         'public_configuration': report.get('public_configuration', []),
                         'definition_count': report['definition_count'], 'context_counts': report['context_counts']})
    for file in files.values():
        file['profiles'] = sorted(set(file['profiles']))
        file['context_ids'] = sorted(set(file['context_ids']))
    return add_counts({'schema_version':3, 'revision':revision, 'scope_sha256':scope_hash,
        'compiler':'libclang 18.1.1 Python binding', 'collector_profiles':sorted(labels),
        'files':files, 'functions':[rows[key] for key in sorted(rows)],
        'translation_units':[contexts[key] for key in sorted(contexts)], 'report_evidence':evidence,
        'limits':['Source-bound union of three required observation reports; not complete conditional or runtime coverage.',
                  'Successful recorded observations do not remove failed, fallback or derived contexts.',
                  'Comment recognition and Doxygen generation do not prove contract correctness.']})


def validate_receipt_path(path, receipt_root, report, revision):
    relative = path.relative_to(receipt_root)
    expected = 'function-inventory-'+report.get('collector_profile', '')+'-'+revision
    if relative.parts != (expected, 'function-inventory.json'):
        raise ValueError('Function receipt directory does not match its profile/revision')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=ROOT)
    parser.add_argument('--receipts', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(argv)
    root = args.root.resolve()
    result = {'status':'INCOMPLETE', 'coverage_complete':False, 'errors':[]}
    try:
        scope_path = root/'testing/compliance/inventory_scope.json'
        paths = inventory.owned_files(root, inventory.read_json(scope_path))
        bindings = compiler_contexts.source_bindings(root, paths)
        revision = compiler_contexts.revision(root)
        receipts = sorted(args.receipts.rglob('function-inventory.json'))
        reports = [json.loads(path.read_text()) for path in receipts]
        for path, report in zip(receipts, reports):
            validate_receipt_path(path, args.receipts, report, revision)
        result.update(combine(reports, revision, bindings, compiler_contexts.sha256(scope_path.read_bytes())))
        result['receipt_files'] = [{'path':str(path), 'bytes_sha256':compiler_contexts.sha256(path.read_bytes())} for path in receipts]
        if compiler_contexts.source_bindings(root, paths) != bindings:
            raise ValueError('Source changed while combining compiler observations')
        result['status'] = 'OBSERVATIONS_MERGED'
    except (OSError, ValueError, KeyError, subprocess.SubprocessError) as error:
        result['errors'].append(str(error))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x', encoding='utf-8') as stream:
        stream.write(json.dumps(result, indent=2)+'\n')
    print(json.dumps({key:result.get(key) for key in ('status','errors','definition_count','error_free_definition_count',
        'diagnostic_translation_unit_count','recovery_function_observation_count','coverage_complete')}, indent=2))
    return 0 if result['status']=='OBSERVATIONS_MERGED' else 2


if __name__ == '__main__':
    raise SystemExit(main())
