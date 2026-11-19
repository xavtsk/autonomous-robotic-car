#!/usr/bin/env python3
"""Observe active owned preprocessor arms in a disposable source overlay.

This preprocesses only: it neither runs C programs nor edits measured sources.
The result distinguishes observed arms from missing coverage. It never treats
compiling a few named profiles as proof that every conditional arm was built.
"""
from __future__ import annotations
import argparse
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
from pathlib import Path
import re
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from testing.compliance import inventory
try:
    from .format_owned import TOKEN
    from .baseline_fixture_coverage import ExternalBaseline, PROFILES as BASELINE_FIXTURE_PROFILES
except ImportError:
    from format_owned import TOKEN
    from baseline_fixture_coverage import ExternalBaseline, PROFILES as BASELINE_FIXTURE_PROFILES

MARKER = 'CODEX_CONDITIONAL_'


def source_view(source):
    """Return the phase-2 text, masked directives view and physical offsets."""
    logical, offsets = [], []
    cursor = 0
    for match in re.finditer(r'\\\r?\n', source):
        logical.append(source[cursor:match.start()])
        offsets.extend(range(cursor, match.start()))
        cursor = match.end()
    logical.append(source[cursor:])
    offsets.extend(range(cursor, len(source)))
    spliced = ''.join(logical)
    masked = TOKEN.sub(lambda item: re.sub(r'[^\n]', ' ', item[0])
                       if item.lastgroup in {'comment', 'literal'} else item[0], spliced)
    return spliced, masked, offsets


def macro_definitions(source):
    """Retain every definition, including redefinitions, without expanding it."""
    spliced, masked, _ = source_view(source)
    definitions = []
    pattern = r'(?m)^[ \t]*(?:#|%:)\s*define\s+([A-Za-z_]\w*)([^\n]*)'
    for match in re.finditer(pattern, masked):
        tail = spliced[match.start(2):match.end(2)]
        parameters = set()
        if tail.startswith('('):
            closing = tail.find(')')
            if closing < 0:
                raise ValueError('Unclosed macro parameter list: '+match[1])
            parameters = set(re.findall(r'[A-Za-z_]\w*', tail[1:closing]))
            parameters.add('__VA_ARGS__')
            tail = tail[closing+1:]
        values = [item[0] for item in TOKEN.finditer(tail)
                  if item.lastgroup not in {'comment', 'literal'}]
        definitions.append(dict(name=match[1],
            dependencies=sorted(set(value for value in values
                if re.fullmatch(r'[A-Za-z_]\w*', value)) - parameters),
            unsupported=any(value in {'##', '%:%:', 'defined'} for value in values),
            definition=spliced[match.start():match.end()].strip()))
    return definitions


def expression_names(expression):
    """defined(NAME) tests existence; it does not expand NAME's value."""
    values = [item[0] for item in TOKEN.finditer(expression)
              if item.lastgroup not in {'comment', 'literal'}]
    names, index = set(), 0
    while index < len(values):
        value = values[index]
        if value == 'defined':
            index += 1
            if index < len(values) and values[index] == '(':
                index += 1
                if index < len(values):
                    index += 1
                if index < len(values) and values[index] == ')':
                    index += 1
            elif index < len(values):
                index += 1
            continue
        if re.fullmatch(r'[A-Za-z_]\w*', value):
            names.add(value)
        index += 1
    return names


def line_safety_errors(sources, observed_definitions=()):
    """Conservatively reject line-dependent conditions and include selection.

    Definitions from pristine -dD runs include active imported-header and -D
    macros. Lexical owned definitions also cover inactive/undefined aliases.
    All definitions are unioned; no ordering or argument-use proof is claimed.
    """
    definitions = list(observed_definitions)
    for source in sources.values():
        definitions.extend(macro_definitions(source))
    dependencies, unsupported = {}, set()
    for row in definitions:
        dependencies.setdefault(row['name'], set()).update(row['dependencies'])
        if row['unsupported']:
            unsupported.add(row['name'])

    def dependency_path(name):
        pending, seen = [(name, [name])], set()
        while pending:
            current, chain = pending.pop()
            if current in seen:
                continue
            seen.add(current)
            if current == '__LINE__':
                return chain, 'line value'
            if current in unsupported:
                return chain, 'token-pasting or generated defined operator'
            pending.extend((value, chain+[value])
                           for value in sorted(dependencies.get(current, ()), reverse=True))
        return None

    errors = []
    for path, source in sources.items():
        spliced, masked, offsets = source_view(source)
        for match in re.finditer(r'(?m)^[ \t]*(?:#|%:)\s*(?:line\b|[0-9])[^\n]*', masked):
            line = source.count('\n', 0, offsets[match.start()])+1
            errors.append(f'{path}:{line}: explicit source line control is unsupported')
        pattern = r'(?m)^[ \t]*(?:#|%:)\s*(if|elif|include|include_next)\b([^\n]*)'
        for match in re.finditer(pattern, masked):
            expression = spliced[match.start(2):match.end(2)].strip()
            if match[1].startswith('include') and expression.startswith(('"', '<')):
                continue
            for name in sorted(expression_names(expression)):
                found = dependency_path(name)
                if found:
                    chain, reason = found
                    line = source.count('\n', 0, offsets[match.start()])+1
                    errors.append(f'{path}:{line}: #{match[1]} depends on {reason}: '+
                                  ' -> '.join(chain))
    return errors


def instrument(source, path):
    """Preserve logical lines; reject unsupported line-dependent directives."""
    if re.search(r'\?\?[=/\'()!<>-]', source):
        raise ValueError('Trigraph source requires an explicit preprocessing review: ' + path)
    sensitive = line_safety_errors({path: source})
    if sensitive:
        raise ValueError('; '.join(sensitive))
    spliced, masked, offsets = source_view(source)
    insertions, branches, stack = [], [], []
    for match in re.finditer(r'(?m)^[ \t]*(?:#|%:)\s*(if|ifdef|ifndef|elif|else|endif)\b[^\n]*', masked):
        kind = match[1]
        if kind == 'endif':
            if not stack:
                raise ValueError('Unmatched #endif: ' + path)
            stack.pop()
            continue
        if kind in {'else', 'elif'}:
            if not stack or stack[-1]['else_seen']:
                raise ValueError('Unmatched or repeated conditional arm: ' + path)
            stack[-1]['else_seen'] = kind == 'else'
        else:
            stack.append(dict(else_seen=False))
        end = offsets[match.end()-1]+1
        if end < len(source) and source[end] == '\n':
            end += 1
        line = source.count('\n', 0, offsets[match.start()])+1
        identifier = hashlib.sha256(f'{path}:{len(branches)}'.encode()).hexdigest()[:24]
        branch = dict(id=identifier, source=path, line=line, kind=kind,
                      directive=spliced[match.start():match.end()].strip(), depth=len(stack))
        branches.append(branch)
        insertions.append((end, f'\n#pragma message ("{MARKER}{identifier}")\n'))
    if stack:
        raise ValueError('Unclosed conditional: ' + path)
    result = source
    for position, addition in reversed(insertions):
        result = result[:position]+addition+result[position:]
    return result, branches


def preprocess_arguments(arguments, source):
    """Remove output/link operands, retaining the real compiler and feature flags."""
    result, index = [arguments[0]], 1
    while index < len(arguments):
        arg = arguments[index]
        if arg in {'-o', '-MF', '-MT', '-MQ'}:
            index += 2
            continue
        if arg.startswith(('-MF', '-MT', '-MQ', '-o')):
            index += 1
            continue
        if arg in {'-I', '-isystem', '-iquote', '-include', '-imacros', '-isysroot', '-D', '-U'}:
            if index+1 >= len(arguments):
                raise ValueError('Missing compiler option argument: '+arg)
            result.extend(arguments[index:index+2])
            index += 2
            continue
        if (arg in {'-c', '-S', '-E', '-MMD', '-MD', '-MP', '-fstack-usage'} or
                arg.startswith(('-Wl,', '-l', '-L')) or
                (not arg.startswith('-') and Path(arg).suffix in {'.c', '.o', '.a', '.obj'})):
            index += 1
            continue
        result.append(arg)
        index += 1
    return [*result, '-E', '-P', str(source)]


def pristine_observation(command, cwd, output, number):
    """Preprocess the untouched overlay and retain macro/failure evidence."""
    command = [*command[:-1], '-dD', command[-1]]
    log = f'pristine-{number}.log'
    macros = f'pristine-{number}.macros'
    receipt = dict(command=command, directory=str(cwd), log=log,
                   macro_definitions_log=macros)
    try:
        result = subprocess.run(command, cwd=cwd, capture_output=True, text=True, timeout=120)
        (output/log).write_text(result.stderr, encoding='utf-8')
        definitions = macro_definitions(result.stdout)
        (output/macros).write_text('\n'.join(row['definition'] for row in definitions)+'\n', encoding='utf-8')
        receipt.update(returncode=result.returncode,
                       stdout_sha256=hashlib.sha256(result.stdout.encode()).hexdigest(),
                       definition_count=len(definitions), definitions=definitions)
        if result.returncode:
            failed_output = f'pristine-{number}.failed.i'
            (output/failed_output).write_text(result.stdout, encoding='utf-8')
            receipt['failed_preprocessing_output'] = failed_output
    except (OSError, subprocess.SubprocessError, ValueError) as exc:
        (output/log).write_text(str(exc), encoding='utf-8')
        receipt.update(returncode=125, definitions=[], error=str(exc))
    return receipt


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, required=True)
    parser.add_argument('--commands', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--profile', required=True)
    parser.add_argument('--external-baseline', type=Path)
    parser.add_argument('--fixture-bindings', type=Path)
    parser.add_argument('--module-bindings', type=Path)
    args = parser.parse_args()
    if sys.platform != 'linux':
        parser.error('Run the full overlay collector in the Linux CI checkout.')
    root, output = args.root.resolve(), args.output.resolve()
    if output.is_relative_to(root):
        parser.error('Keep the disposable overlay outside the subject source tree.')
    output.mkdir(parents=True, exist_ok=False)
    overlay = output/'source-overlay'
    revision = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=root, text=True).strip()
    external, setup_errors = None, []
    if args.external_baseline:
        try:
            if args.profile not in BASELINE_FIXTURE_PROFILES or args.fixture_bindings is None:
                raise ValueError('External baseline requires a distinct fixture profile and same-run binding')
            external = ExternalBaseline(root, args.external_baseline, args.fixture_bindings,
                                        args.module_bindings, revision)
        except (OSError, ValueError, subprocess.SubprocessError) as error:
            setup_errors.append(str(error))
    elif args.profile in BASELINE_FIXTURE_PROFILES or args.fixture_bindings or args.module_bindings:
        setup_errors.append('Baseline-fixture inputs require an explicit external baseline checkout')
    if (root/'config/wifi_secrets.h').exists():
        parser.error('Refusing a checkout containing private or unclassified network settings.')
    # Preserve build-generated headers and original paths. No links back to
    # measured files: writing an overlay file must never mutate the real tree.
    shutil.copytree(root, overlay, symlinks=True,
                    ignore=shutil.ignore_patterns('.git', '*.elf', '*.uf2', '*.o', '*.a', '*.map', '*.su'))
    example = root/'config/wifi_secrets.example.h'
    if example.is_file():
        shutil.copyfile(example, overlay/'config/wifi_secrets.h')
    scope = inventory.read_json(ROOT/'testing/compliance/inventory_scope.json')
    owned = inventory.owned_files(root, scope)
    branches, hashes, errors, sources = [], {}, list(setup_errors), {}
    for file in owned:
        path = file.relative_to(root).as_posix()
        text = file.read_text(encoding='utf-8-sig')
        hashes[path] = hashlib.sha256(text.encode()).hexdigest()
        sources[path] = text
    command_bytes = args.commands.read_bytes()
    command_digest = hashlib.sha256(command_bytes).hexdigest()
    contents = command_bytes.decode('utf-8')
    commands = json.loads(contents) if contents.lstrip().startswith('[') else [json.loads(row) for row in contents.splitlines() if row]
    work, seen = [], set()
    for row in commands if not setup_errors else []:
        arguments = row['arguments']
        directory = Path(row['directory'])
        for value in arguments[1:]:
            if value.startswith('-') or Path(value).suffix != '.c':
                continue
            source = (directory/value).resolve()
            if not source.is_relative_to(root):
                continue
            mapped = overlay/source.relative_to(root)
            command = preprocess_arguments(arguments, mapped)
            try:
                if external:
                    command, cwd = external.map_context(command, directory, root, overlay)
                else:
                    command = [part.replace(str(root), str(overlay)) for part in command]
                    cwd = overlay/directory.relative_to(root)
            except (OSError, ValueError) as error:
                errors.append('Unbound compiler context: '+str(error))
                continue
            key = (tuple(command), str(cwd))
            if key not in seen:
                seen.add(key)
                work.append((command, cwd))
    # Collect active header and command-line definitions before inserting any
    # marker. Failed pristine commands cannot supply a passing coverage union.
    with ThreadPoolExecutor(max_workers=4) as pool:
        pristine = list(pool.map(lambda item: pristine_observation(
            item[1][0], item[1][1], output, item[0]), enumerate(work)))
    definitions = [definition for row in pristine for definition in row.pop('definitions')]
    errors.extend('Pristine preprocessing failed: '+row['log']
                  for row in pristine if row['returncode'])
    dependencies = []
    if external and not errors:
        # Sequential collection also binds a stable first-seen dependency hash.
        dependencies = [external.observe(command, cwd, output, number, overlay)
                        for number, (command, cwd) in enumerate(work)]
        errors.extend('External dependency observation failed: '+row['log']
                      for row in dependencies if row['returncode'] or row['errors'])
    try:
        safety_errors = line_safety_errors(sources, definitions)
    except ValueError as exc:
        safety_errors = ['Line-safety review failed: '+str(exc)]
    errors.extend(safety_errors)
    changed_sources = {}
    for path, text in sources.items():
        try:
            changed, rows = instrument(text, path)
            # Mixed ownership: observe only the explicitly owned conditional.
            if path in scope.get('mixed_files', {}):
                region = scope['mixed_files'][path]
                ranges = [(text.count('\n', 0, start)+1, text.count('\n', 0, end-1)+1)
                          for start, end in inventory.mixed_region_spans(text, region)]
                rows = [row for row in rows if any(first <= row['line'] <= last for first, last in ranges)]
            branches.extend(rows)
            changed_sources[path] = changed
        except ValueError as exc:
            errors.append(str(exc))
    derived_inputs = {}
    if not errors:
        for path, changed in changed_sources.items():
            if external is None or path in external.fixtures:
                (overlay/path).write_text(changed, encoding='utf-8')
        # Both pristine and instrumented passes use the public example alias.
        if example.is_file():
            shutil.copyfile(overlay/'config/wifi_secrets.example.h', overlay/'config/wifi_secrets.h')
            derived_inputs['config/wifi_secrets.h'] = 'config/wifi_secrets.example.h'
    def observe(item):
        number, (command, cwd) = item
        try:
            result = subprocess.run(command, cwd=cwd, capture_output=True, text=True, timeout=120)
            (output/f'preprocess-{number}.log').write_text(result.stderr, encoding='utf-8')
            active = sorted(set(re.findall(MARKER+r'([0-9a-f]{24})', result.stdout)))
            return dict(command=command, directory=str(cwd), returncode=result.returncode,
                        active=active if result.returncode == 0 else [], log=f'preprocess-{number}.log')
        except (OSError, subprocess.SubprocessError) as exc:
            return dict(command=command, directory=str(cwd), returncode=125, active=[], error=str(exc))
    observations = []
    if not errors:
        with ThreadPoolExecutor(max_workers=4) as pool:
            observations = list(pool.map(observe, enumerate(work)))
    active = {value for row in observations for value in row['active']}
    for row in branches:
        row['observed'] = row['id'] in active
    for path, digest in hashes.items():
        current = hashlib.sha256((root/path).read_text(encoding='utf-8-sig').encode()).hexdigest()
        if current != digest:
            errors.append('Measured source changed during preprocessing: '+path)
    if external:
        try:
            external.verify_preservation()
        except (OSError, ValueError) as error:
            errors.append(str(error))
    if hashlib.sha256(args.commands.read_bytes()).hexdigest() != command_digest:
        errors.append('Compiler command receipt changed during preprocessing')
    failures = [row for row in observations if row['returncode']]
    report = dict(profile=args.profile, revision=revision, source_sha256=hashes, branches=branches,
                  derived_inputs=derived_inputs,
                  pristine_observations=pristine, line_safety_errors=safety_errors,
                  external_baseline=external.record if external else None,
                  dependency_observations=dependencies,
                  compile_commands_sha256=command_digest,
                  observations=observations, errors=errors, missing=[row['id'] for row in branches if not row['observed']],
                  measurement_passed=bool(observations) and not errors and not failures,
                  coverage_complete=bool(observations) and not errors and not failures and all(row['observed'] for row in branches),
                  network_configuration='Public example in disposable overlay only.',
                  limitations=['Explicit conditional arms only; implicit empty else arms contain no owned code.',
                               'Host-only files need host compile records; a target profile alone is incomplete.',
                               'Line-safety dependencies conservatively union macro definitions and contexts; harmless redefinitions or unused arguments can be rejected.',
                               'Token-pasting/generated defined macros reaching conditions or include selection require explicit review; no full macro-expansion equivalence is claimed.',
                               'External fixed-baseline dependencies remain uninstrumented; only hash-bound current fixture arms can contribute.',
                               'Preprocessor visibility is not branch execution or full-course validation.'])
    (output/'results.json').write_text(json.dumps(report, indent=2)+'\n', encoding='utf-8')
    if overlay.resolve().parent != output or overlay.name != 'source-overlay':
        raise RuntimeError('Unexpected overlay cleanup path')
    shutil.rmtree(overlay)
    print(f"Observed {sum(row['observed'] for row in branches)}/{len(branches)} explicit owned arms in {len(observations)} preprocess commands; failures={len(failures)}.")
    return not report['measurement_passed']


if __name__ == '__main__':
    sys.exit(main())
