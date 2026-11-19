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
from format_owned import TOKEN

MARKER = 'CODEX_CONDITIONAL_'


def instrument(source, path):
    """Preserve logical-line boundaries; ignore directives inside comments/literals."""
    if re.search(r'\?\?[=/\'()!<>-]', source):
        raise ValueError('Trigraph source requires an explicit preprocessing review: ' + path)
    # Build physical offsets for the C99 phase-2 line-spliced view.
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


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, required=True)
    parser.add_argument('--commands', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--profile', required=True)
    args = parser.parse_args()
    if sys.platform != 'linux':
        parser.error('Run the full overlay collector in the Linux CI checkout.')
    root, output = args.root.resolve(), args.output.resolve()
    if output.is_relative_to(root):
        parser.error('Keep the disposable overlay outside the subject source tree.')
    output.mkdir(parents=True, exist_ok=False)
    overlay = output/'source-overlay'
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
    branches, hashes, errors = [], {}, []
    for file in owned:
        path = file.relative_to(root).as_posix()
        text = file.read_text(encoding='utf-8-sig')
        hashes[path] = hashlib.sha256(text.encode()).hexdigest()
        try:
            changed, rows = instrument(text, path)
            # Mixed ownership: observe only the explicitly owned conditional.
            if path in scope.get('mixed_files', {}):
                region = scope['mixed_files'][path]
                start = text.index(region['project_region_start'])
                end = text.index(region['project_region_end'], start)+len(region['project_region_end'])
                first, last = text.count('\n', 0, start)+1, text.count('\n', 0, end)+1
                rows = [row for row in rows if first <= row['line'] <= last]
            branches.extend(rows)
            (overlay/path).write_text(changed, encoding='utf-8')
        except ValueError as exc:
            errors.append(str(exc))
    contents = args.commands.read_text(encoding='utf-8')
    commands = json.loads(contents) if contents.lstrip().startswith('[') else [json.loads(row) for row in contents.splitlines() if row]
    work, seen = [], set()
    for row in commands:
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
            command = [part.replace(str(root), str(overlay)) for part in command]
            cwd = overlay/directory.relative_to(root)
            key = (tuple(command), str(cwd))
            if key not in seen:
                seen.add(key)
                work.append((command, cwd))
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
    with ThreadPoolExecutor(max_workers=4) as pool:
        observations = list(pool.map(observe, enumerate(work)))
    active = {value for row in observations for value in row['active']}
    for row in branches:
        row['observed'] = row['id'] in active
    for path, digest in hashes.items():
        current = hashlib.sha256((root/path).read_text(encoding='utf-8-sig').encode()).hexdigest()
        if current != digest:
            errors.append('Measured source changed during preprocessing: '+path)
    failures = [row for row in observations if row['returncode']]
    report = dict(profile=args.profile, source_sha256=hashes, branches=branches,
                  observations=observations, errors=errors, missing=[row['id'] for row in branches if not row['observed']],
                  measurement_passed=bool(observations) and not errors and not failures,
                  coverage_complete=bool(observations) and not errors and not failures and all(row['observed'] for row in branches),
                  network_configuration='Public example in disposable overlay only.',
                  limitations=['Explicit conditional arms only; implicit empty else arms contain no owned code.',
                               'Host-only files need host compile records; a target profile alone is incomplete.',
                               'Preprocessor visibility is not branch execution or full-course validation.'])
    (output/'results.json').write_text(json.dumps(report, indent=2)+'\n', encoding='utf-8')
    if overlay.resolve().parent != output or overlay.name != 'source-overlay':
        raise RuntimeError('Unexpected overlay cleanup path')
    shutil.rmtree(overlay)
    print(f"Observed {sum(row['observed'] for row in branches)}/{len(branches)} explicit owned arms in {len(observations)} preprocess commands; failures={len(failures)}.")
    return not report['measurement_passed']


if __name__ == '__main__':
    sys.exit(main())
