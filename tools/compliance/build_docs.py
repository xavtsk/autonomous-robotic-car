#!/usr/bin/env python3
"""Generate an owned-source reference and retain incomplete documentation evidence."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from testing.compliance import inventory


def stage_sources(root, destination):
    """Copy only classified ownership; preserve lines in masked imported regions."""
    scope = inventory.read_json(root/'testing/compliance/inventory_scope.json')
    bindings = {}
    for path in inventory.owned_files(root, scope):
        name = path.relative_to(root).as_posix()
        text = path.read_text(encoding='utf-8-sig')
        bindings[name] = dict(source_location_sha256=inventory.source_location_digest(text),
                              source_content_sha256=hashlib.sha256(text.encode()).hexdigest(),
                              original_bytes_sha256=hashlib.sha256(path.read_bytes()).hexdigest())
        if name in scope.get('mixed_files', {}):
            text = inventory.mixed_owned_text(text, scope['mixed_files'][name])
        target = destination/name
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(text, encoding='utf-8', newline='\n')
        bindings[name]['staged_sha256'] = hashlib.sha256(target.read_bytes()).hexdigest()
    if not bindings:
        raise ValueError('No owned documentation inputs')
    return bindings


def validate_functions(report, bindings):
    """Reject a partial/stale file set before comparing observed function sites."""
    files = report.get('files', {})
    if set(files) != set(bindings):
        raise ValueError('Compiler function report has a different owned file set')
    for name, binding in bindings.items():
        if files[name].get('source_location_sha256') != binding['source_location_sha256']:
            raise ValueError('Stale compiler function locations: '+name)
        content = files[name].get('source_content_sha256')
        if not content or content != binding['source_content_sha256']:
            raise ValueError('Stale or missing compiler documentation source content: '+name)
    if not report.get('functions'):
        raise ValueError('No compiler-observed function definitions')


def configuration(source, output):
    """Keep undocumented warnings enabled; XML is observational, not a compiler."""
    def quoted(path):
        value = Path(path).resolve().as_posix()
        if any(char in value for char in ('"', '\n', '\r')):
            raise ValueError('Unsupported Doxygen path quoting')
        return '"'+value+'"'
    return '\n'.join([
        'PROJECT_NAME = "Autonomous robotic car: owned source reference"',
        'OUTPUT_DIRECTORY = '+quoted(output),
        'INPUT = '+quoted(source),
        'STRIP_FROM_PATH = '+quoted(source),
        'RECURSIVE = YES', 'FILE_PATTERNS = *.c *.h',
        'OPTIMIZE_OUTPUT_FOR_C = YES', 'EXTRACT_ALL = NO',
        'EXTRACT_STATIC = YES', 'EXTRACT_PRIVATE = YES',
        'HIDE_UNDOC_MEMBERS = NO', 'HIDE_UNDOC_CLASSES = NO',
        'WARNINGS = YES', 'WARN_IF_UNDOCUMENTED = YES',
        'WARN_IF_DOC_ERROR = YES', 'WARN_NO_PARAMDOC = YES',
        'WARN_AS_ERROR = NO', 'WARN_LOGFILE = '+quoted(output/'warnings.log'),
        'WARN_FORMAT = "$file:$line: $text"',
        'ENABLE_PREPROCESSING = NO', 'SEARCH_INCLUDES = NO',
        'GENERATE_HTML = YES', 'HTML_TIMESTAMP = NO',
        'GENERATE_LATEX = NO', 'GENERATE_XML = YES',
        'XML_PROGRAMLISTING = NO', 'HAVE_DOT = NO', 'QUIET = YES',
        'FULL_PATH_NAMES = YES', 'SOURCE_BROWSER = NO',
        '',
    ])


def observed_functions(xml_directory, staged_source):
    """Use body locations to distinguish repeated names in different C modules."""
    found = set()
    index = ET.parse(xml_directory/'index.xml').getroot()
    for compound in index.findall('compound'):
        refid = compound.get('refid', '')
        if not refid or Path(refid).name != refid or '/' in refid or '\\' in refid:
            raise ValueError('Invalid Doxygen compound reference')
        for member in ET.parse(xml_directory/(refid+'.xml')).iter('memberdef'):
            if member.get('kind') != 'function':
                continue
            location = member.find('location')
            if location is None or not location.get('bodyfile'):
                continue
            function_name = member.findtext('name')
            body_start = location.get('bodystart', '')
            if not function_name or not body_start.isdecimal() or int(body_start) <= 0:
                raise ValueError('Missing or invalid Doxygen function name/body start')
            path = Path(location.get('bodyfile'))
            if not path.is_absolute():
                path = staged_source/path
            try:
                name = path.resolve().relative_to(staged_source.resolve()).as_posix()
            except ValueError:
                raise ValueError('Doxygen body outside owned staged source: '+str(path))
            found.add((name, int(body_start), function_name))
    return found


def compare_functions(report, observed):
    missing = []
    for row in report['functions']:
        if row['method'] != 'libclang-ast':
            continue
        # Doxygen's body start may be the signature line or its opening brace.
        matches = [site for site in observed if site[0] == row['path'] and
                   site[2] == row['name'] and row['line'] <= site[1] <= row['end_line']]
        if not matches:
            missing.append({key: row[key] for key in ('path', 'name', 'line', 'end_line', 'profiles')})
    return dict(missing_observed_definitions=missing,
                compiler_definition_variants=len(report['functions']),
                compiler_recovery_variants=sum(row['method'] != 'libclang-ast' for row in report['functions']),
                compiler_undocumented_variants=sum(not row['definition_documented'] for row in report['functions']),
                doxygen_definition_sites=len(observed))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=ROOT)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--function-report', type=Path, required=True)
    parser.add_argument('--doxygen', default='/usr/bin/doxygen')
    args = parser.parse_args(argv)
    root, output = args.root.resolve(), args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    bindings = {}
    staged = output/'source'
    result = dict(status='INCOMPLETE', coverage_complete=False, errors=[],
                  limits=['Generated documentation does not validate contract meaning.',
                          'Preprocessing is disabled to expose alternative source branches; Doxygen is not a C compiler.',
                          'Compiler observations and conditional coverage are separate, potentially incomplete evidence.'])
    try:
        bindings = stage_sources(root, staged)
        result['source_bindings'] = bindings
        report = json.loads(args.function_report.read_text(encoding='utf-8'))
        validate_functions(report, bindings)
        result['function_report_sha256'] = hashlib.sha256(args.function_report.read_bytes()).hexdigest()
        version = subprocess.check_output([args.doxygen, '--version'], text=True).strip()
        result['version'] = version
        if version != '1.9.8':
            raise ValueError('Expected reviewed Doxygen 1.9.8, found '+version)
        config = output/'Doxyfile'
        (output/'reference').mkdir()
        config.write_text(configuration(staged, output/'reference'), encoding='utf-8')
        command = [args.doxygen, str(config)]
        result['command'] = command
        with (output/'generation.log').open('w', encoding='utf-8') as stream:
            code = subprocess.run(command, cwd=root, stdout=stream, stderr=subprocess.STDOUT, timeout=240).returncode
        result['generation_returncode'] = code
        if code:
            raise ValueError('Doxygen generation failed; see retained generation.log')
        warning_path = output/'reference/warnings.log'
        warnings = warning_path.read_text(encoding='utf-8').splitlines()
        result['warning_lines'] = len(warnings)
        observed = observed_functions(output/'reference/xml', staged)
        result.update(compare_functions(report, observed))
        if not warnings and not result['missing_observed_definitions'] and not result['compiler_recovery_variants'] and not result['compiler_undocumented_variants'] and report.get('coverage_complete') is True:
            result['status'] = 'PASS'
            result['coverage_complete'] = True
    except (OSError, ValueError, KeyError, ET.ParseError, subprocess.SubprocessError) as error:
        result['errors'].append(str(error))
    finally:
        # Retain the preservation result even when generation or XML parsing fails.
        if bindings:
            try:
                for name, binding in bindings.items():
                    if hashlib.sha256((root/name).read_bytes()).hexdigest() != binding['original_bytes_sha256']:
                        raise ValueError('Source changed during documentation measurement: '+name)
                    if hashlib.sha256((staged/name).read_bytes()).hexdigest() != binding['staged_sha256']:
                        raise ValueError('Staged source changed during documentation measurement: '+name)
                result['source_unchanged'] = True
            except (OSError, ValueError) as error:
                result.update(status='INCOMPLETE', coverage_complete=False, source_unchanged=False)
                result['errors'].append(str(error))
    (output/'results.json').write_text(json.dumps(result, indent=2)+'\n', encoding='utf-8')
    print(json.dumps({key: value for key, value in result.items() if key not in ('source_bindings', 'missing_observed_definitions')}, indent=2))
    return 0 if result['status'] == 'PASS' else 2


if __name__ == '__main__':
    sys.exit(main())
