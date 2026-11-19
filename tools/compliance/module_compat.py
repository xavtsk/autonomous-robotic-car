"""Bind shared fixtures to an explicit historical module filename boundary."""
import hashlib
import json
from pathlib import Path

try:
    from .fixture_binding import FIXED_BASELINE
except ImportError:
    from fixture_binding import FIXED_BASELINE


BASELINE_TILT_HEADER_SHA256 = '1a6bf7f012d2c1b54ff21bb343f492eb85c6c8cd858bb290b9636f234d36d25c'


def tilt_module(root):
    """Accept exactly one complete pair, rejecting stale partial pairs and links."""
    directory = Path(root)/'app_program/subsystem4'
    names = ('tilt_estimation', 'tilt')
    if directory.is_symlink() or directory.parent.is_symlink():
        raise ValueError('Linked tilt module directory')
    present = {}
    for name in names:
        paths = [directory/(name+suffix) for suffix in ('.c', '.h')]
        if any(path.is_symlink() for path in paths):
            raise ValueError('Linked tilt module input')
        if any(not path.resolve().is_relative_to(Path(root).resolve()) for path in paths):
            raise ValueError('Tilt module input escapes subject source')
        present[name] = tuple(path.is_file() for path in paths)
    complete = [name for name, pair in present.items() if pair == (True, True)]
    if len(complete) != 1 or any(any(pair) for name, pair in present.items() if name not in complete):
        raise ValueError('Expected exactly one complete tilt .c/.h pair: '+str(present))
    return complete[0]


def prepare_tilt(root, revision, destination, receipt):
    """Generate only the fixed baseline's missing header, outside measured source."""
    root, destination, receipt = Path(root).resolve(), Path(destination).resolve(), Path(receipt)
    report = dict(subject_root=str(root), subject_revision=revision, status='FAIL', errors=[])
    try:
        name = tilt_module(root)
        report['module'] = name
        expected = 'tilt_estimation' if revision == FIXED_BASELINE else 'tilt'
        if not revision or name != expected:
            raise ValueError('Tilt filename pair does not match the selected revision boundary')
        header = root/'app_program/subsystem4'/(name+'.h')
        report['header'] = str(header)
        report['header_bytes_sha256'] = hashlib.sha256(header.read_bytes()).hexdigest()
        report['source_bytes_sha256'] = hashlib.sha256(header.with_suffix('.c').read_bytes()).hexdigest()
        includes = []
        if revision == FIXED_BASELINE:
            if report['header_bytes_sha256'] != BASELINE_TILT_HEADER_SHA256:
                raise ValueError('Fixed-baseline tilt header bytes differ from the reviewed Git blob')
            if destination.is_relative_to(root):
                raise ValueError('Historical header output must remain outside subject source')
            header_name = header.as_posix()
            if any(character in header_name for character in ('"', '\n', '\r')):
                raise ValueError('Unsupported historical header path quoting')
            destination.mkdir(parents=True, exist_ok=False)
            forwarder = destination/'tilt.h'
            contents = ('/* Derived fixture compatibility for fixed baseline '+FIXED_BASELINE+'. */\n'
                        '#include "'+header_name+'"\n')
            forwarder.write_text(contents, encoding='utf-8', newline='\n')
            report['derived_header'] = dict(path=str(forwarder), contents=contents,
                bytes_sha256=hashlib.sha256(forwarder.read_bytes()).hexdigest(),
                dependency_header=str(header), dependency_bytes_sha256=report['header_bytes_sha256'])
            includes.append(destination)
        report.update(status='PASS', include_directories=[str(path) for path in includes],
                      selection='fixed-baseline-derived-header' if includes else 'candidate-real-header')
        return name, includes
    except (OSError, ValueError) as error:
        report['errors'].append(str(error))
        raise
    finally:
        receipt.write_text(json.dumps(report, indent=2)+'\n', encoding='utf-8')
