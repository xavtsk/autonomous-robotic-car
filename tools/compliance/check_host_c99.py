#!/usr/bin/env python3
"""Check C99 syntax for application translation units exercised by host tests.

Preserve each recorded include path and profile define. Test harnesses, imported
sources and target firmware compiler flags are not modified. Missing host
coverage is listed explicitly; this is not an all-profile firmware proof.
"""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys


def syntax_commands(row, root):
    directory = Path(row['directory'])
    arguments = row['arguments']
    sources, flags = [], []
    skip_next = False
    for argument in arguments[1:]:
        if skip_next:
            skip_next = False
            continue
        if argument in ('-o', '-Xlinker', '-L', '-l'):
            skip_next = True
            continue
        if argument.endswith('.c'):
            source = (directory/argument).resolve()
            if source.is_relative_to(root/'app_program'):
                sources.append(source)
        elif (argument in ('-c', '-S', '-E', '-shared')
              or argument.startswith(('-std=', '-Wl,', '-l', '-L'))
              or argument.endswith(('.o', '.a'))):
            continue
        else:
            flags.append(argument)
    return [(source, [arguments[0], *flags, '-std=c99', '-pedantic-errors',
                      '-fsyntax-only', str(source)]) for source in sources]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--commands', type=Path, required=True)
    parser.add_argument('--root', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if sys.platform != 'linux':
        parser.error('C99 compiler audit runs on Linux only.')
    root, output = args.root.resolve(), args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    results, seen, covered = [], set(), set()
    for line in args.commands.read_text(encoding='utf-8').splitlines():
        row = json.loads(line)
        for source, command in syntax_commands(row, root):
            key = json.dumps([row['directory'], command])
            if key in seen:
                continue
            seen.add(key)
            relative = source.relative_to(root).as_posix()
            covered.add(relative)
            log_name = hashlib.sha256(key.encode()).hexdigest()[:16]+'.log'
            with (output/log_name).open('w', encoding='utf-8') as stream:
                stream.write('COMMAND '+json.dumps(command)+'\n')
                stream.flush()
                try:
                    code = subprocess.run(command, cwd=row['directory'], stdout=stream,
                                          stderr=subprocess.STDOUT, timeout=120).returncode
                except (OSError, subprocess.SubprocessError) as exc:
                    stream.write(str(exc)+'\n')
                    code = 125
            results.append(dict(source=relative, command=command, directory=row['directory'],
                                status='PASS' if code == 0 else 'FAIL', returncode=code, log=log_name))
    uncovered = sorted(path.relative_to(root).as_posix() for path in (root/'app_program').rglob('*.c')
                       if path.relative_to(root).as_posix() not in covered)
    passed = bool(results) and all(row['returncode'] == 0 for row in results)
    report = dict(passed=passed, scope='application translation units in recorded host compiler invocations',
                  checked_variants=len(results), covered_sources=sorted(covered),
                  sources_without_standalone_host_compilation=uncovered, results=results,
                  limits=['Host fake headers differ from target SDK headers.',
                          'Sources embedded by a test include are not standalone translation units here.',
                          'This gate does not claim C99 coverage of every firmware profile.'])
    (output/'results.json').write_text(json.dumps(report, indent=2)+'\n', encoding='utf-8')
    print(f'{"PASS" if passed else "FAIL"}: C99 syntax for {len(results)} variants of {len(covered)} application sources; {len(uncovered)} lack standalone host compilation')
    return not passed


if __name__ == '__main__':
    sys.exit(main())
