"""Plan source-bound compiler observations without inventing successful contexts."""
from __future__ import annotations

import hashlib
import json
from pathlib import Path
import re
import shlex
import shutil
import subprocess

from testing.compliance import inventory, inventory_ast


def sha256(data):
    return hashlib.sha256(data).hexdigest()


def revision(root):
    return subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=root, text=True).strip()


def pinned_resource():
    """Use the installed Clang 18 frontend's resource headers with libclang 18."""
    compiler = shutil.which('clang-18')
    if not compiler:
        raise ValueError('Pinned clang-18 is required to locate matching builtin headers')
    version = subprocess.check_output([compiler, '--version'], text=True).splitlines()[0]
    if not re.search(r'\bversion 18\.1\.', version):
        raise ValueError('Expected Clang 18.1 resource headers: '+version)
    resource = Path(subprocess.check_output([compiler, '-print-resource-dir'], text=True).strip()).resolve()
    header = resource/'include/stdint.h'
    if not header.is_file():
        raise ValueError('Clang resource directory lacks stdint.h')
    return resource, {'compiler': compiler, 'version': version, 'resource_directory': str(resource),
                      'stdint_header_sha256': sha256(header.read_bytes())}


def source_bindings(root, paths):
    result = {}
    for path in paths:
        text = path.read_text(encoding='utf-8-sig')
        result[path.relative_to(root).as_posix()] = {
            'semantic_sha256': inventory.semantic_digest(text),
            'source_location_sha256': inventory.source_location_digest(text),
            'source_content_sha256': sha256(text.encode())}
    return result


def equivalent_roots(root, alternatives, paths):
    """Only remap a second checkout of the exact revision and owned contents."""
    expected_revision = revision(root)
    bindings = source_bindings(root, paths)
    accepted = []
    for alternative in alternatives:
        alternative = alternative.resolve()
        if alternative == root or revision(alternative) != expected_revision:
            raise ValueError('Equivalent source root is identical or has a different revision')
        for name, binding in bindings.items():
            other = alternative/name
            if not other.is_file() or sha256(other.read_text(encoding='utf-8-sig').encode()) != binding['source_content_sha256']:
                raise ValueError('Equivalent checkout source differs: '+name)
        # Imported tracked headers also matter to the command. Never remap a
        # dirty tracked source tree merely because its owned inputs agree.
        for checkout in (root, alternative):
            if subprocess.run(['git', 'diff', '--quiet', 'HEAD', '--'], cwd=checkout).returncode:
                raise ValueError('Equivalent-root remapping requires unchanged tracked files')
        accepted.append(alternative)
    return accepted


def remap(value, root, alternatives):
    for alternative in alternatives:
        for spelling in {str(alternative), alternative.as_posix()}:
            value = re.sub(re.escape(spelling)+r'(?=[/\\]|$)', lambda _: str(root), value)
    return value


def public_configuration(root, destination):
    """Reproduce the target builder's public template after its cleanup step."""
    template = root/'config/wifi_secrets.example.h'
    existing = root/'config/wifi_secrets.h'
    if existing.exists() or existing.is_symlink():
        # A local settings file may contain credentials. Presence is enough
        # to reject this CI-only derivation; its contents are never inspected.
        raise ValueError('Public-config derivation requires source settings to be absent')
    if template.is_symlink() or template.parent.is_symlink():
        raise ValueError('Linked public settings template is not supported')
    if any(path.is_symlink() for path in (destination, *destination.parents)):
        raise ValueError('Linked derived settings destination is not supported')
    contents = template.read_bytes()
    destination = destination.resolve()
    if destination.is_relative_to(root):
        raise ValueError('Derived compiler settings must remain outside the source checkout')
    destination.mkdir(parents=True, exist_ok=False)
    derived = destination/'wifi_secrets.h'
    derived.write_bytes(contents)
    return ['-I'+str(destination)], {'kind': 'public-example-config',
        'template': 'config/wifi_secrets.example.h', 'template_bytes_sha256': sha256(contents),
        'derived_path': str(derived), 'derived_bytes_sha256': sha256(derived.read_bytes()),
        'source_settings_modified': False}


def plan(root, databases, paths, profiles, resource, alternatives=(), recorded_only=False,
         extra_arguments=(), derivations=()):
    """Retain actual commands; explicitly label every synthetic/fallback parse."""
    commands, common = {}, None
    for database in databases:
        entries = ([json.loads(line) for line in database.read_text().splitlines() if line.strip()]
                   if database.suffix == '.jsonl' else json.loads(database.read_text()))
        database_hash = sha256(database.read_bytes())
        for index, entry in enumerate(entries):
            original = entry.get('arguments', entry.get('command'))
            if isinstance(original, str):
                original = shlex.split(original)
            if not original:
                continue
            directory = Path(remap(str(entry.get('directory', entry.get('cwd', root))), root, alternatives)).resolve()
            mapped = [remap(value, root, alternatives) for value in original]
            cleaned = inventory_ast.clean_arguments(mapped, directory)
            cleaned = [*extra_arguments, *cleaned,
                       *inventory_ast.compiler_include_arguments(mapped[0], resource), '-x', 'c']
            context = {'database': str(database.resolve()), 'database_sha256': database_hash,
                       'entry_index': index, 'original_directory': str(entry.get('directory', entry.get('cwd', root))),
                       'original_arguments': original, 'mapped_directory': str(directory),
                       'arguments': cleaned, 'argument_derivations': list(derivations),
                       'equivalent_root_remapping': [str(path) for path in alternatives]}
            if common is None:
                common = context
            sources = [entry['file']] if 'file' in entry else [value for value in original if value.endswith('.c')]
            for value in sources:
                source = Path(remap(value, root, alternatives))
                source = (source if source.is_absolute() else directory/source).resolve()
                if source in paths:
                    contexts = commands.setdefault(source, [])
                    if not any(item['arguments'] == cleaned for item in contexts):
                        contexts.append(context)
    if common is None:
        raise ValueError('No usable compiler command was supplied')
    jobs = []
    for source in sorted(paths):
        actual = commands.get(source, [])
        host_only = source.relative_to(root).parts[0] in {'testing', 'tools'}
        # A target build is not a host test's compiler context. Retain that
        # file's unobserved binding until an actual host receipt supplies it.
        if host_only and not actual:
            continue
        if recorded_only and not actual:
            continue
        contexts = actual or [common]
        for context in contexts:
            selected = ['configured'] if recorded_only or host_only else profiles
            for profile in selected:
                origin = ('recorded-command' if profile == 'configured' else 'derived-profile') if actual else (
                    'fallback-standalone-header' if source.suffix == '.h' else 'fallback-unrecorded-source')
                arguments = inventory_ast.profile_arguments(context['arguments'], inventory_ast.PROFILES[profile])
                identity = {'source': source.relative_to(root).as_posix(), 'origin': origin,
                            'profile': profile, 'arguments': arguments}
                metadata = {**context, **identity,
                            'context_id': sha256(json.dumps(identity, sort_keys=True).encode()),
                            'profile_overrides': inventory_ast.PROFILES[profile],
                            'recorded_source': bool(actual)}
                jobs.append(((source, arguments, profile), metadata))
    if not jobs:
        raise ValueError('No owned translation unit matched the recorded compiler commands')
    return jobs
