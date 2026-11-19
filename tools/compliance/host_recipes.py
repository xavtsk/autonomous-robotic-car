"""Existing additional host recipes, bound to unchanged shared expectations."""
import hashlib
import json
from pathlib import Path
import re


def embedded_decoder_fixture(subject, fixtures, relative, destination):
    """Only relocate the one subject include when testing the fixed baseline."""
    original = fixtures/relative
    data = original.read_bytes()
    receipt = dict(source=str(original), source_sha256=hashlib.sha256(data).hexdigest())
    if subject == fixtures:
        receipt.update(selection='candidate-subject', selected_source=str(original))
        return original, receipt
    pattern = rb'(?m)^[ \t]*#[ \t]*include[ \t]+"../../app_program/subsystem3/bc_decoder[.]c"[ \t]*(?=\r?$)'
    matches = list(re.finditer(pattern, data))
    if len(matches) != 1:
        raise ValueError('Expected exactly one embedded decoder include: ' + relative)
    target = subject/'app_program/subsystem3/bc_decoder.c'
    if not target.is_file():
        raise ValueError('Missing subject decoder: ' + str(target))
    replacement = ('#include "'+target.as_posix()+'"').encode()
    match = matches[0]
    changed = data[:match.start()] + replacement + data[match.end():]
    assert changed[:match.start()] + match[0] + changed[match.start()+len(replacement):] == data
    destination.write_bytes(changed)
    receipt.update(selection='shared-fixture-subject-include-only', selected_source=str(destination),
                   selected_sha256=hashlib.sha256(changed).hexdigest(),
                   original_include=match[0].decode(), relocated_include=replacement.decode(),
                   inverse_bytes_equal=True)
    return destination, receipt


def additional_host_recipes(subject, fixtures, output, compiler, flags):
    """Return build/execution pairs; all compile through the caller's recorder."""
    subject, fixtures, output = map(lambda value: Path(value).resolve(), (subject, fixtures, output))
    stage = output/'additional-host'
    stage.mkdir(exist_ok=False)
    report = dict(status='FAIL', subject=str(subject), fixtures=str(fixtures),
                  fixture_relocations=[], recipes=[], errors=[])
    try:
        app = subject/'app_program'
        s3 = app/'subsystem3'
        s3_fixtures = fixtures/'testing/subsystem3'

        def add(name, sources, includes=(), extra=()):
            binary = stage/name
            build = [compiler, *flags, *extra, *['-I'+str(p) for p in includes],
                     *map(str, sources), '-o', str(binary)]
            report['recipes'].append(dict(name=name, build=build, execute=[str(binary)]))

        for name in ('fit_error', 'incremental'):
            relative = 'testing/subsystem3/test_barcode_'+name+'.c'
            source, receipt = embedded_decoder_fixture(subject, fixtures, relative,
                                                        stage/('test_barcode_'+name+'.c'))
            report['fixture_relocations'].append(receipt)
            add('barcode_'+name, [source], [s3, s3_fixtures], ['-pedantic'])
        modules = [s3/(name+'.c') for name in ('ir_acquisition', 'ir_calibration', 'ir_reflectance',
                   'line_detection', 'line_controller', 'bc_decoder', 'line_navigation')]
        add('barcode_bench', [s3_fixtures/'bench_trace.c', *modules, s3/'navigation_codec.c'],
            [s3, s3_fixtures], ['-pedantic'])
        s1, f1 = app/'subsystem1', fixtures/'testing/subsystem1'
        mqtt = subject/'lib/liblwip/sysdepend/pico_rp2040'
        add('subsystem1_sizes', [f1/'size_report.c', s1/'telemetry_codec.c'],
            [f1, f1/'fakes', s1, mqtt],
            ['-DAPP_H', '-DAPP_TEST=1', '-DTM_MQTT=1', '-include', str(f1/'host_kernel.h'),
             '-fshort-enums', '-Wno-type-limits'])
        f2 = fixtures/'testing/subsystem2'
        add('subsystem2_sizes', [f2/'size_report.c'], [f2, f2/'fakes', app/'subsystem2'],
            ['-DAPP_H', '-DAPP_TEST=1', '-DTEST_PACKED', '-include', str(f2/'host_kernel.h'), '-fshort-enums'])
        add('subsystem3_sizes', [s3_fixtures/'size_report.c'], [s3, s3_fixtures],
            ['-pedantic', '-fshort-enums'])
        add('course_zero_axle', [fixtures/'testing/integration/course_test.c',
                                app/'controller/s3_drive.c', *modules],
            [app/'subsystem2', s3, app/'controller'], ['-DS3_SENSOR_TO_AXLE_MM=0'])
        report['status'] = 'PREPARED'
        report['limits'] = ['Prepared commands are not executed test results.',
                            'Short-enum host layout reports are not linked target RAM or stack measurements.']
        return report['recipes']
    except (OSError, ValueError) as error:
        report['errors'].append(str(error))
        raise
    finally:
        (output/'additional-host-recipes.json').write_text(json.dumps(report, indent=2)+'\n', encoding='utf-8')
