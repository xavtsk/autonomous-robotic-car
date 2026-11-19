"""Bind unchanged shared C fixtures to candidate paths, with a retained receipt."""
import hashlib
import json
from pathlib import Path


FIXED_BASELINE = '4ad2cc42595722cedb47102e4343298339040d40'
FIXTURE_TREES = ('testing', 'tools/compliance/traces')


def fixture_hashes(root):
    """Include all C/header fixtures and fakes; reject links instead of guessing."""
    hashes = {}
    for tree in FIXTURE_TREES:
        directory = root/tree
        if not directory.is_dir() or directory.is_symlink():
            raise ValueError('Missing or linked fixture tree: ' + tree)
        for path in sorted(directory.rglob('*')):
            if path.is_symlink():
                raise ValueError('Linked fixture input: ' + path.relative_to(root).as_posix())
            if path.is_file() and path.suffix in ('.c', '.h'):
                if not path.resolve().is_relative_to(root):
                    raise ValueError('Fixture input escapes its source tree')
                hashes[path.relative_to(root).as_posix()] = hashlib.sha256(path.read_bytes()).hexdigest()
    if not hashes:
        raise ValueError('No C/header fixture inputs found')
    return hashes


def select_fixture_root(subject, tooling, subject_revision, tooling_revision, receipt):
    """Only the fixed baseline uses shared tooling; candidates must match bytes."""
    subject, tooling, receipt = Path(subject).resolve(), Path(tooling).resolve(), Path(receipt)
    report = dict(subject_root=str(subject), tooling_root=str(tooling),
                  subject_revision=subject_revision, tooling_revision=tooling_revision,
                  trees=list(FIXTURE_TREES), status='FAIL', errors=[])
    try:
        expected = fixture_hashes(tooling)
        report['tooling_sha256'] = expected
        if subject_revision == FIXED_BASELINE:
            selected = tooling
            report['selection'] = 'fixed-baseline-shared-tooling'
            report['subject_files_compared'] = False
        else:
            if not subject_revision or subject_revision != tooling_revision:
                raise ValueError('Candidate and shared tooling revisions differ')
            actual = fixture_hashes(subject)
            report['subject_sha256'] = actual
            report['subject_files_compared'] = True
            missing, extra = sorted(expected.keys()-actual.keys()), sorted(actual.keys()-expected.keys())
            changed = sorted(name for name in expected.keys() & actual.keys() if expected[name] != actual[name])
            if missing or extra or changed:
                report['differences'] = dict(missing=missing, extra=extra, changed=changed)
                raise ValueError('Candidate C/header fixtures differ from shared tooling')
            selected = subject
            report['selection'] = 'byte-identical-candidate-subject'
        report.update(status='PASS', selected_root=str(selected), selected_sha256=expected)
        return selected
    except (OSError, ValueError) as error:
        report['errors'].append(str(error))
        raise
    finally:
        receipt.write_text(json.dumps(report, indent=2)+'\n', encoding='utf-8')
