#!/usr/bin/env python3
"""Compare actual symbol allocation and compiler stack reports, without execution."""
from collections import defaultdict
import json
from pathlib import Path


def symbol_rows(contents):
    """Keep duplicate local names instead of silently overwriting allocations."""
    rows = []
    for line in contents.splitlines():
        parts = line.split()
        if not parts:
            continue
        if len(parts) not in (3, 4) or len(parts[1]) != 1:
            raise ValueError('Unexpected POSIX nm row: ' + line)
        if len(parts) == 4:
            rows.append(dict(name=parts[0], kind=parts[1],
                             address=int(parts[2], 16), bytes=int(parts[3], 16)))
    return rows


def stack_rows(folder):
    """Source line numbers are metadata, not part of a function's identity."""
    rows = []
    for file in sorted(Path(folder).rglob('*.su')):
        for line in file.read_text(encoding='utf-8').splitlines():
            if not line:
                continue
            parts = line.split('\t')
            if len(parts) != 3:
                raise ValueError(f'Unexpected stack row in {file}: {line}')
            source, line_number, column, function = parts[0].rsplit(':', 3)
            rows.append(dict(report=file.relative_to(folder).as_posix(),
                             source=source, line=int(line_number), column=int(column),
                             function=function, bytes=int(parts[1]), qualifier=parts[2]))
    if not rows:
        raise ValueError('No nonempty compiler stack reports')
    return rows


def compare_rows(before, after, key_fields, value_fields):
    """Compare multisets, recording removals/additions rather than inventing a rename."""
    grouped = []
    for rows in (before, after):
        group = defaultdict(list)
        for row in rows:
            group[tuple(row[name] for name in key_fields)].append(
                tuple(row[name] for name in value_fields))
        grouped.append(group)
    changes = []
    for key in sorted(grouped[0].keys() | grouped[1].keys()):
        left, right = (sorted(group.get(key, [])) for group in grouped)
        if left != right:
            changes.append(dict(zip(key_fields, key), baseline=left, candidate=right))
    return changes


def collect(directory):
    """Retain raw reports alongside a machine-readable interpretation."""
    directory = Path(directory)
    result = dict(symbols=symbol_rows((directory/'symbols.txt').read_text(encoding='utf-8')),
                  stack=stack_rows(directory/'stack-usage'),
                  limitations=[
                      'Symbol sizes include object padding; they are not information-bit counts.',
                      'Linker gaps, shared storage and discarded objects need the full map/size report.',
                      'Compiler stack estimates are per function, not task high-water marks or call-chain maxima.',
                      'Renamed/inlined/removed functions are reported explicitly; no zero-stack claim is inferred.'])
    (directory/'allocation-and-stack.json').write_text(json.dumps(result, indent=2)+'\n', encoding='utf-8')
    return result


def compare(before, after):
    return dict(
        allocation_changes=compare_rows(before['symbols'], after['symbols'], ('name', 'kind'), ('bytes',)),
        stack_changes=compare_rows(before['stack'], after['stack'], ('report', 'function'), ('bytes', 'qualifier')),
        all_changes_reported=True,
        physical_stack_measured=False)
