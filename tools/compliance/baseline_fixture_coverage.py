"""Observe current fixtures with explicitly bound, uninstrumented baseline inputs."""
import hashlib
import json
from pathlib import Path
import re
import subprocess

try:
    from .fixture_binding import FIXED_BASELINE, fixture_hashes
    from .module_compat import BASELINE_TILT_HEADER_SHA256
except ImportError:
    from fixture_binding import FIXED_BASELINE, fixture_hashes
    from module_compat import BASELINE_TILT_HEADER_SHA256

PROFILES = {'host-baseline-fixtures-gcc', 'host-baseline-fixtures-clang'}
SYSTEM_ROOTS = tuple(Path(name) for name in ('/usr/include', '/usr/lib/gcc', '/usr/lib/llvm-18/lib/clang'))
CURRENT_HELPERS = ('app_program/build_assert.h',)


def digest(data):
    return hashlib.sha256(data).hexdigest()


def git_blob(data):
    return hashlib.sha1(b'blob '+str(len(data)).encode()+b'\0'+data).hexdigest()


def reject_links(path):
    if any(item.is_symlink() for item in (path, *path.parents)):
        raise ValueError('Linked coverage dependency: '+str(path))


def baseline_tree(root):
    data = subprocess.check_output(['git', 'ls-tree', '-r', '-z', FIXED_BASELINE], cwd=root)
    files = {}
    for entry in data.split(b'\0'):
        if not entry:
            continue
        metadata, path = entry.split(b'\t', 1)
        mode, kind, blob = metadata.decode().split()
        if kind == 'blob' and mode in ('100644', '100755'):
            files[path.decode('utf-8')] = blob
    return files


class ExternalBaseline:
    """Dependency receipt for one current-tooling/fixed-baseline collection."""
    def __init__(self, root, baseline, fixture_receipt, module_receipt, revision):
        root, baseline = Path(root), Path(baseline)
        reject_links(root)
        reject_links(baseline)
        self.root, self.baseline = root.resolve(), baseline.resolve()
        if self.root.is_relative_to(self.baseline) or self.baseline.is_relative_to(self.root):
            raise ValueError('Current tooling and external baseline must be separate roots')
        self.check_private_configuration()
        observed = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=self.baseline, text=True).strip()
        if observed != FIXED_BASELINE:
            raise ValueError('External dependency checkout is not the fixed baseline')
        self.blobs = baseline_tree(self.baseline)
        binding = json.loads(Path(fixture_receipt).read_text(encoding='utf-8'))
        self.fixtures = fixture_hashes(self.root)
        self.helpers = {}
        for name in CURRENT_HELPERS:
            path = self.root/name
            reject_links(path)
            if path.is_file():
                self.helpers[name] = digest(path.read_bytes())
        if (binding.get('status') != 'PASS' or binding.get('selection') != 'fixed-baseline-shared-tooling' or
                binding.get('subject_revision') != FIXED_BASELINE or binding.get('tooling_revision') != revision or
                Path(binding.get('subject_root', '')).resolve() != self.baseline or
                Path(binding.get('tooling_root', '')).resolve() != self.root or
                Path(binding.get('selected_root', '')).resolve() != self.root or
                binding.get('selected_sha256') != self.fixtures or binding.get('tooling_sha256') != self.fixtures):
            raise ValueError('Missing, stale or inconsistent same-run fixture binding')
        self.derived = {}
        if module_receipt is not None:
            module = json.loads(Path(module_receipt).read_text(encoding='utf-8'))
            header = self.baseline/'app_program/subsystem4/tilt_estimation.h'
            expected = ('/* Derived fixture compatibility for fixed baseline '+FIXED_BASELINE+'. */\n'
                        '#include "'+header.as_posix()+'"\n')
            derived = module.get('derived_header', {})
            if (module.get('status') != 'PASS' or module.get('subject_revision') != FIXED_BASELINE or
                    module.get('selection') != 'fixed-baseline-derived-header' or
                    module.get('subject_root') != str(self.baseline) or
                    derived.get('contents') != expected or derived.get('dependency_header') != str(header) or
                    derived.get('dependency_bytes_sha256') != BASELINE_TILT_HEADER_SHA256 or
                    derived.get('bytes_sha256') != digest(expected.encode())):
                raise ValueError('Unbound historical module forwarding header')
            path = Path(derived['path'])
            reject_links(path)
            if path.name != 'tilt.h' or path.is_relative_to(self.root) or path.is_relative_to(self.baseline):
                raise ValueError('Historical forwarding header must be isolated derived output')
            if digest(path.read_bytes()) != derived['bytes_sha256']:
                raise ValueError('Historical forwarding header bytes changed')
            self.derived[path.resolve()] = derived
        self.snapshots = {}
        self.record = dict(kind='current-fixtures-fixed-baseline', revision=FIXED_BASELINE,
            root=str(self.baseline), tooling_root=str(self.root), tooling_revision=revision,
            fixture_bindings_sha256=digest(Path(fixture_receipt).read_bytes()),
            fixture_sha256=self.fixtures, helper_sha256=self.helpers,
            derived_headers=list(self.derived.values()),
            source_unchanged=False)

    def check_private_configuration(self):
        for root in (self.root, self.baseline):
            if (root/'config/wifi_secrets.h').exists():
                raise ValueError('Refusing private or unclassified external network configuration')

    def map_context(self, arguments, directory, root, overlay):
        """Rebind only actual tooling path operands; preserve external baseline cwd."""
        self.record['overlay_root'] = str(overlay)
        directory = Path(directory).resolve()
        if directory.is_relative_to(root):
            cwd = overlay/directory.relative_to(root)
        elif directory.is_relative_to(self.baseline):
            cwd = directory
        else:
            raise ValueError('Unknown compile-command working directory: '+str(directory))
        result = [arguments[0]]
        separate, literal = False, False
        prefixes = ('-isystem', '-iquote', '-include', '-imacros', '-I')
        for argument in arguments[1:]:
            if argument.startswith(('-Wp,', '-save-temps', '--save-temps', '-fprofile',
                                    '-ftime-trace', '-MJ', '-serialize-diagnostics', '--serialize-diagnostics')):
                raise ValueError('Output-producing external preprocessor option is unsupported: '+argument)
            if literal:
                result.append(argument)
                literal = False
                continue
            if argument in ('-D', '-U', '-x'):
                result.append(argument)
                literal = True
                continue
            if argument in prefixes:
                result.append(argument)
                separate = True
                continue
            prefix = next((item for item in prefixes if argument.startswith(item) and argument != item), '')
            path_operand = separate or bool(prefix) or not argument.startswith('-')
            separate = False
            if argument.startswith(('@', '--sysroot', '-isysroot')):
                raise ValueError('Unsupported external compiler path option: '+argument)
            if path_operand:
                value = argument[len(prefix):] if prefix else argument
                path = (directory/value).resolve()
                if path.is_relative_to(root):
                    path = overlay/path.relative_to(root)
                if not (path.is_relative_to(overlay) or path.is_relative_to(self.baseline) or
                        any(path.is_relative_to(item) for item in SYSTEM_ROOTS) or
                        path in self.derived or any(path == item.parent for item in self.derived)):
                    raise ValueError('Unknown compiler input root: '+str(path))
                argument = prefix+str(path)
            result.append(argument)
        return result, cwd

    def dependency(self, path, overlay):
        reject_links(path)
        path = path.resolve()
        if path.is_relative_to(overlay):
            name = path.relative_to(overlay).as_posix()
            original = self.root/name
            reject_links(original)
            expected = self.fixtures.get(name, self.helpers.get(name))
            if expected is None:
                raise ValueError('Unbound current dependency: '+name)
            kind = 'tooling' if name in self.fixtures else 'tooling-helper'
        elif path.is_relative_to(self.baseline):
            name = path.relative_to(self.baseline).as_posix()
            if name not in self.blobs:
                raise ValueError('Untracked fixed-baseline dependency: '+name)
            kind = 'baseline'
        elif path in self.derived:
            kind, name = 'derived', str(path)
        elif any(path.is_relative_to(item) for item in SYSTEM_ROOTS):
            kind, name = 'system', str(path)
        else:
            raise ValueError('Unknown consumed dependency root: '+str(path))
        # Classify before reading: unknown paths cannot disclose their contents.
        data = path.read_bytes()
        row = dict(kind=kind, path=name, bytes_sha256=digest(data))
        if kind in ('tooling', 'tooling-helper'):
            if digest(data) != expected or original.read_bytes() != data:
                raise ValueError('Stale or unbound current fixture dependency: '+name)
            row['normalized_sha256'] = digest(data.decode('utf-8-sig').replace('\r\n', '\n').encode())
            path = original
        elif kind == 'baseline':
            if self.blobs.get(name) != git_blob(data):
                raise ValueError('Dirty or untracked fixed-baseline dependency: '+name)
            row['blob_sha1'] = git_blob(data)
        elif kind == 'derived':
            if digest(data) != self.derived[path]['bytes_sha256']:
                raise ValueError('Derived forwarding header changed')
        if path in self.snapshots and self.snapshots[path] != digest(data):
            raise ValueError('Consumed dependency changed between observations: '+str(path))
        self.snapshots.setdefault(path, digest(data))
        return row

    def observe(self, command, cwd, output, number, overlay):
        """Retain an additional pristine pass with file markers for dependencies."""
        command = [part for part in command if part != '-P']
        receipt = dict(command=command, directory=str(cwd), returncode=125, dependencies=[], errors=[],
                       log=f'dependencies-{number}.log', output=f'dependencies-{number}.i')
        try:
            process = subprocess.run(command, cwd=cwd, capture_output=True, text=True, timeout=120)
            receipt['returncode'] = process.returncode
            (output/receipt['log']).write_text(process.stderr, encoding='utf-8')
            (output/receipt['output']).write_text(process.stdout, encoding='utf-8')
            receipt['output_sha256'] = digest(process.stdout.encode())
            if process.returncode:
                raise ValueError('External dependency preprocessing failed')
            files = set(re.findall(r'^#\s+\d+\s+("(?:\\.|[^"\\])*")', process.stdout, re.MULTILINE))
            for quoted in sorted(files):
                name = json.loads(quoted)
                if name in ('', '<built-in>', '<command-line>', '<command line>'):
                    continue
                path = (cwd/name)
                if path.resolve() == cwd.resolve() and path.is_dir():
                    continue  # GCC's explicit working-directory marker.
                receipt['dependencies'].append(self.dependency(path, overlay))
            kinds = {row['kind'] for row in receipt['dependencies']}
            if not {'tooling', 'baseline'} <= kinds:
                raise ValueError('No current fixture and fixed-baseline dependency pair observed')
        except (OSError, ValueError, subprocess.SubprocessError) as error:
            receipt['errors'].append(str(error))
            if not (output/receipt['log']).exists():
                (output/receipt['log']).write_text(str(error), encoding='utf-8')
        return receipt

    def verify_preservation(self):
        self.check_private_configuration()
        if fixture_hashes(self.root) != self.fixtures:
            raise ValueError('Current shared fixtures changed during observation')
        for name, expected in self.helpers.items():
            if digest((self.root/name).read_bytes()) != expected:
                raise ValueError('Current uninstrumented helper changed during observation: '+name)
        for path, expected in self.snapshots.items():
            reject_links(path)
            if digest(path.read_bytes()) != expected:
                raise ValueError('External consumed dependency changed during observation: '+str(path))
        self.record['source_unchanged'] = True


def validate_receipt(report, samples, hashes, revision, root):
    """Independently bind external receipts to current fixtures and Git objects."""
    if root is None:
        raise ValueError('External fixture union requires a source checkout for dependency verification')
    root = Path(root)
    binding = report.get('external_baseline') or {}
    fixtures = fixture_hashes(root)
    helpers = {name: digest((root/name).read_bytes()) for name in CURRENT_HELPERS if (root/name).is_file()}
    if (binding.get('kind') != 'current-fixtures-fixed-baseline' or
            binding.get('revision') != FIXED_BASELINE or binding.get('tooling_revision') != revision or
            binding.get('source_unchanged') is not True or binding.get('fixture_sha256') != fixtures or
            binding.get('helper_sha256') != helpers or
            not re.fullmatch(r'[0-9a-f]{64}', binding.get('fixture_bindings_sha256', '')) or
            not re.fullmatch(r'[0-9a-f]{64}', report.get('compile_commands_sha256', ''))):
        raise ValueError('External revision, source preservation or current fixture binding differs')
    baseline_root = Path(binding['root'])
    tooling_root = Path(binding['tooling_root'])
    overlay_root = Path(binding.get('overlay_root', ''))
    if (not all(path.is_absolute() for path in (baseline_root, tooling_root, overlay_root)) or
            baseline_root.is_relative_to(tooling_root) or tooling_root.is_relative_to(baseline_root) or
            overlay_root.is_relative_to(tooling_root) or overlay_root.is_relative_to(baseline_root)):
        raise ValueError('Invalid external source root binding')
    derived = {}
    for entry in binding.get('derived_headers', []):
        header = baseline_root/'app_program/subsystem4/tilt_estimation.h'
        contents = ('/* Derived fixture compatibility for fixed baseline '+FIXED_BASELINE+'. */\n'
                    '#include "'+header.as_posix()+'"\n')
        path = Path(entry['path'])
        if (not path.is_absolute() or path.name != 'tilt.h' or entry.get('contents') != contents or
                entry.get('bytes_sha256') != digest(contents.encode()) or
                entry.get('dependency_header') != str(header) or
                entry.get('dependency_bytes_sha256') != BASELINE_TILT_HEADER_SHA256 or
                path.is_relative_to(baseline_root) or path.is_relative_to(tooling_root)):
            raise ValueError('Unbound derived header in external receipt')
        if str(path) in derived:
            raise ValueError('Duplicate external derived header')
        derived[str(path)] = entry['bytes_sha256']
    observations = report.get('dependency_observations', [])
    if len(observations) != len(samples) or not observations:
        raise ValueError('Missing external dependency observations')
    blobs = baseline_tree(root)
    baseline_hashes = {}
    for observation, sample in zip(observations, samples):
        if (observation.get('returncode') != 0 or observation.get('errors') or
                observation.get('command') != [part for part in sample['command'] if part != '-P'] or
                observation.get('directory') != sample['directory'] or
                not observation.get('dependencies')):
            raise ValueError('Failed or mismatched external dependency context')
        directory, source = Path(sample['directory']), Path(sample['command'][-1])
        if (not source.is_relative_to(overlay_root) or
                source.relative_to(overlay_root).as_posix() not in fixtures or
                not (directory.is_relative_to(overlay_root) or directory.is_relative_to(baseline_root))):
            raise ValueError('External command does not select a current fixture in a bound source root')
        source_name = source.relative_to(overlay_root).as_posix()
        if not any(row.get('kind') == 'tooling' and row.get('path') == source_name
                   for row in observation['dependencies']):
            raise ValueError('Selected current fixture is absent from dependency observations')
        kinds = set()
        for entry in observation['dependencies']:
            kind, path, checksum = entry['kind'], entry['path'], entry['bytes_sha256']
            kinds.add(kind)
            if not re.fullmatch(r'[0-9a-f]{64}', checksum):
                raise ValueError('Invalid external dependency content hash')
            if kind == 'tooling':
                if fixtures.get(path) != checksum or hashes.get(path) != entry.get('normalized_sha256'):
                    raise ValueError('Current fixture dependency differs: '+path)
            elif kind == 'tooling-helper':
                if helpers.get(path) != checksum or hashes.get(path) != entry.get('normalized_sha256'):
                    raise ValueError('Current uninstrumented helper dependency differs: '+path)
            elif kind == 'baseline':
                if blobs.get(path) != entry.get('blob_sha1') or not entry.get('blob_sha1'):
                    raise ValueError('Baseline dependency Git object differs: '+path)
                if path not in baseline_hashes:
                    baseline_hashes[path] = digest(subprocess.check_output(['git', 'show', FIXED_BASELINE+':'+path], cwd=root))
                if baseline_hashes[path] != checksum:
                    raise ValueError('Baseline dependency content differs: '+path)
            elif kind == 'derived':
                if derived.get(path) != checksum:
                    raise ValueError('Derived dependency content differs: '+path)
            elif kind == 'system':
                if not any(Path(path).is_relative_to(directory) for directory in SYSTEM_ROOTS):
                    raise ValueError('Unknown external system-header root: '+path)
            else:
                raise ValueError('Unknown external dependency classification: '+str(kind))
        if not {'tooling', 'baseline'} <= kinds:
            raise ValueError('External context lacks its fixture/baseline dependency pair')
