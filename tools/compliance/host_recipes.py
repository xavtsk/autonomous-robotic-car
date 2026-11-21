"""Existing additional host recipes, bound to unchanged shared expectations."""
import hashlib
import json
from pathlib import Path
import re

try:
    from .module_compat import barcode_work_flags
except ImportError:
    from module_compat import barcode_work_flags


def decoder_with_probes(subject, fixtures, destination, guard, header):
    """Select the decoder with the same host-only probes in either revision."""
    return _source_with_probes(subject, fixtures, destination,
        'app_program/subsystem3/bc_decoder.c', 'decoder', guard, header, b'#endif')


def _source_with_probes(subject, fixtures, destination, relative, kind, guard, header, closing):
    source = subject/relative
    data = source.read_bytes()
    marker = ('#ifdef '+guard).encode()
    receipt = dict(source=str(source), source_sha256=hashlib.sha256(data).hexdigest())
    if data.count(marker) == 1:
        receipt.update(selection='subject-'+kind+'-probes', selected_source=str(source))
        return source, receipt
    if subject == fixtures or marker in data:
        raise ValueError('Expected one '+guard+' probe block in the candidate source')

    # The fixed sources predate the probes. Append only their shared bodies
    # to a copy of that subject. The recipe still compiles its own algorithm.
    shared = (fixtures/relative).read_bytes()
    if shared.count(marker) != 1:
        raise ValueError('Expected one shared '+guard+' probe block')
    start = shared.index(marker)
    end = shared.index(closing, start) + len(closing)
    probe = shared[start:end]
    selected = data + ('\n#include "'+header+'"\n\n').encode() + probe + b'\n'
    destination.write_bytes(selected)
    receipt.update(selection='fixed-'+kind+'-with-probes', selected_source=str(destination),
                   selected_sha256=hashlib.sha256(selected).hexdigest(),
                   probes_sha256=hashlib.sha256(probe).hexdigest())
    return destination, receipt


def motion_storage_source(subject, fixtures, stage):
    """Select motion for the retained-storage fixture and record its source."""
    stage.mkdir(exist_ok=False)
    source, receipt = _source_with_probes(subject, fixtures, stage/'motion.c',
        'app_program/subsystem2/motion.c', 'motion', 'MOTION_STORAGE_PROBE',
        'motion_storage_main.h', b'#endif /* MOTION_STORAGE_PROBE */')
    (stage/'fixture.json').write_text(json.dumps(receipt, indent=2)+'\n', encoding='utf-8')
    return source


def operator_runtime_recipe(subject, fixtures, stage):
    """Select a real sampling owner with the simulated operator boundaries."""
    stage.mkdir(exist_ok=False)
    header = fixtures/'testing/integration/operator_main.h'
    source, receipt = _source_with_probes(subject, fixtures, stage/'runtime.c',
        'app_program/controller/s3_integration.c', 'operator-runtime',
        'S3_OPERATOR_TEST', header.name, b'#endif')
    (stage/'fixture.json').write_text(json.dumps(receipt, indent=2)+'\n', encoding='utf-8')
    return source, ['-DS3_OPERATOR_TEST', '-include', str(header)]


def sampling_runtime_recipe(subject, fixtures, stage):
    """Select the sampling owner with inspection enabled only for this test."""
    stage.mkdir(exist_ok=False)
    header = fixtures/'testing/integration/runtime_main.h'
    source, receipt = _source_with_probes(subject, fixtures, stage/'runtime.c',
        'app_program/controller/s3_integration.c', 'sampling-runtime',
        'S3_RUNTIME_TEST', header.name, b'#endif /* S3_RUNTIME_TEST */')
    (stage/'fixture.json').write_text(json.dumps(receipt, indent=2)+'\n', encoding='utf-8')
    return source, ['-DS3_RUNTIME_TEST', '-include', str(header)]


def decoder_boundary_recipe(subject, fixtures, stage):
    """Prepare the original phase-boundary assertions for this decoder."""
    stage.mkdir(exist_ok=False)
    source_dir = subject/'app_program/subsystem3'
    fixture_dir = fixtures/'testing/compliance'
    guard = 'BC_DECODER_BOUNDARY_TEST'
    decoder, receipt = decoder_with_probes(subject, fixtures, stage/'decoder.c',
                                          guard, 'decode_work_main.h')
    definitions = re.findall(
        rb'\benum\s*(\{\s*RW_PRIMARY\s*=\s*1\s*,\s*'
        rb'(?:RW_[A-Z_]+\s*,\s*)*RW_DONE\s*\})',
        (source_dir/'bc_decoder.c').read_bytes())
    if len(definitions) != 1:
        raise ValueError('Expected one sequence of decoder work phase values')
    phases = b'enum '+definitions[0]+b';\n'
    (stage/'decode_work_phases.h').write_bytes(phases)
    receipt['phase_header_sha256'] = hashlib.sha256(phases).hexdigest()
    (stage/'fixture.json').write_text(json.dumps(receipt, indent=2)+'\n', encoding='utf-8')
    return ([fixture_dir/'decode_work_main.c', decoder], [source_dir, fixture_dir, stage],
            ['-D'+guard, *barcode_work_flags(subject)])


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

        for name, fixture, guard, options in (
            ('fit_error', 'fit_score_main', 'BC_DECODER_FIT_TEST', []),
            ('incremental', 'work_quota_main', 'BC_DECODER_WORK_TEST',
             ['-DS3_BARCODE_SYNC_REFERENCE', *barcode_work_flags(subject)]),
        ):
            decoder, receipt = decoder_with_probes(subject, fixtures,
                stage/(fixture+'_decoder.c'), guard, fixture+'.h')
            report['fixture_relocations'].append(receipt)
            add('barcode_'+name, [s3_fixtures/(fixture+'.c'), decoder], [s3, s3_fixtures],
                ['-pedantic', '-D'+guard, *options])
        modules = [s3/(name+'.c') for name in ('ir_acquisition', 'ir_calibration', 'ir_reflectance',
                   'line_detection', 'line_controller', 'bc_decoder', 'line_navigation')]
        add('barcode_bench', [s3_fixtures/'bench_trace_main.c', s3/'buddy3.c', *modules, s3/'navigation_codec.c'],
            [s3, s3_fixtures], ['-pedantic', '-DAPP_H', '-DAPP_IR_BENCH=1', '-DAPP_TEST=0',
                                '-include', str(s3_fixtures/'bench_trace_main.h')])
        s1, f1 = app/'subsystem1', fixtures/'testing/subsystem1'
        mqtt = subject/'lib/liblwip/sysdepend/pico_rp2040'
        add('subsystem1_sizes', [f1/'size_report_main.c', s1/'telemetry_codec.c'],
            [f1, f1/'fakes', s1, mqtt],
            ['-DAPP_H', '-DAPP_TEST=1', '-DTM_MQTT=1', '-include', str(f1/'host_kernel.h'),
             '-fshort-enums', '-Wno-type-limits'])
        f2 = fixtures/'testing/subsystem2'
        add('subsystem2_sizes', [f2/'wheel_sizes_main.c'], [f2, f2/'fakes', app/'subsystem2'],
            ['-DAPP_H', '-DAPP_TEST=1', '-DTEST_PACKED', '-include', str(f2/'wheel_kernel.h'), '-fshort-enums'])
        add('subsystem3_sizes', [s3_fixtures/'nav_sizes_main.c'], [s3, s3_fixtures],
            ['-pedantic', '-fshort-enums'])
        course_flags = ['-DS3_SENSOR_TO_AXLE_MM=0']
        if subject != fixtures:
            header = (app/'controller/s3_drive.h').read_text()
            callbacks = re.findall(r'^\s*motion_result_t\s*\(\s*\*\s*(drive|p_drive)\s*\)\s*\(',
                                   header, re.MULTILINE)
            if callbacks == ['drive']:
                course_flags.append('-DCOURSE_DRIVE_MEMBER=drive')
            elif callbacks != ['p_drive']:
                raise ValueError('Unrecognized course-options drive callback')
        add('course_zero_axle', [fixtures/'testing/integration/course_main.c',
                                app/'controller/s3_drive.c', *modules],
            [app/'subsystem2', s3, app/'controller'], course_flags)
        report['status'] = 'PREPARED'
        report['limits'] = ['Prepared commands are not executed test results.',
                            'Short-enum host layout reports are not linked target RAM or stack measurements.']
        return report['recipes']
    except (OSError, ValueError) as error:
        report['errors'].append(str(error))
        raise
    finally:
        (output/'additional-host-recipes.json').write_text(json.dumps(report, indent=2)+'\n', encoding='utf-8')
