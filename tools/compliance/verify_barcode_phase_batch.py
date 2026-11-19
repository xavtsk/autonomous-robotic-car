#!/usr/bin/env python3
"""Verify a bounded barcode helper-extraction batch by reversing its tokens."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys

try:
    from .format_owned import TOKEN
except ImportError:
    from format_owned import TOKEN

ROOT = Path(__file__).resolve().parents[2]


def tokens(source):
    return [match[0] for match in TOKEN.finditer(source) if match.lastgroup != 'comment']


def unique_at(source, pattern):
    hits = [index for index in range(len(source))
            if source[index:index+len(pattern)] == pattern]
    if len(hits) != 1:
        raise ValueError('Expected one token sequence: ' + ' '.join(pattern))
    return hits[0]


def recover(candidate, helpers):
    """Undo only exact pointer bindings, phase calls and void-return exits."""
    recovered = tokens(candidate)
    for item in helpers:
        phase, name = item['phase'], item['helper']
        signature = tokens('static void '+name+' (s3_barcode_record_t * p_r) {')
        start = unique_at(recovered, signature)
        body_start = start+len(signature)
        depth, end = 1, body_start
        while depth and end < len(recovered):
            depth += (recovered[end] == '{') - (recovered[end] == '}')
            end += 1
        if depth:
            raise ValueError('Unbalanced helper body: ' + name)
        body = recovered[body_start:end-1]
        binding = tokens('s3_barcode_work_t * p_w = &p_r->work;')
        if body[:len(binding)] != binding:
            raise ValueError('Unexpected workspace binding: ' + name)
        body = body[len(binding):]
        if body.count('return') != item['switch_exit_count']:
            raise ValueError('Unexpected phase exit count: ' + name)
        for index, token in enumerate(body):
            if token == 'return' and body[index+1:index+2] != [';']:
                raise ValueError('Non-void helper return: ' + name)
        body = [('break' if token == 'return' else token) for token in body]
        recovered[start:end] = []
        call = tokens('case '+phase+': '+name+'(p_r); break;')
        start = unique_at(recovered, call)
        recovered[start:start+len(call)] = tokens('case '+phase+':')+body
    return recovered


def read_source(root, reference, path):
    if reference == 'WORKTREE':
        return (root/path).read_text(encoding='utf-8')
    return subprocess.check_output(['git', 'show', reference+':'+path],
                                   cwd=root, text=True, encoding='utf-8')


def verify(root, manifest, candidate_reference):
    before = read_source(root, manifest['before_revision'], manifest['source'])
    candidate = read_source(root, candidate_reference, manifest['source'])
    for label, source in (('before', before), ('candidate', candidate)):
        digest = hashlib.sha256(source.replace('\r\n', '\n').encode()).hexdigest()
        if digest != manifest[label+'_lf_sha256']:
            raise ValueError(label+' source hash does not match this historical batch')
    original = tokens(before)
    inverted = recover(candidate, manifest['helpers'])
    if original != inverted:
        raise ValueError('Whole-file inverse token comparison differs')
    return {'passed': True, 'before_revision': manifest['before_revision'],
            'candidate_reference': candidate_reference, 'source': manifest['source'],
            'recovered_tokens': len(inverted), 'helpers': len(manifest['helpers']),
            'limit': 'Source transformation proof; machine code and timing require separate evidence.'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest', type=Path,
                        default=ROOT/'docs/compliance/evidence/barcode-phase-extraction.json')
    parser.add_argument('--candidate', default='WORKTREE',
                        help='Pin the extraction commit after publishing; later source changes are a different batch.')
    parser.add_argument('--json', action='store_true')
    args = parser.parse_args()
    try:
        result = verify(ROOT, json.loads(args.manifest.read_text()), args.candidate)
    except (OSError, ValueError, subprocess.SubprocessError) as error:
        result = {'passed': False, 'error': str(error)}
    print(json.dumps(result, indent=2) if args.json else
          ('PASS' if result['passed'] else 'FAIL')+': barcode phase extraction inverse tokens')
    return not result['passed']


if __name__ == '__main__':
    sys.exit(main())
