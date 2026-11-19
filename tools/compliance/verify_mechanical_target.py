#!/usr/bin/env python3
"""Verify one immutable filename-only target pair, without executing firmware."""
from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import re
import shlex
import subprocess
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from tools.compliance import target_metrics


REVISIONS = {'baseline': '55af85ce084fa0fe1fe4f8fe6444cd78e5acf007',
             'candidate': '99b07b777559c0dd5d536f147fae15ea08f0d642'}
SDK = '98a542c1a62fb549ffb5d66a3e5892b06276b670'
IMAGE = 'mtk3pico_smp1_usb_cdc_wifi_lwip_mqtt_s3_motion_s4_s5_ir_debug'
PROFILE = dict(SMP=1, CONSOLE='usb_cdc', CRASH_REPORT=0, WIFI='cyw43', NET='lwip',
    MQTT=1, TEST=0, IR_BENCH=0, IR_CHANNELS=1, S3_MOTION=1, S3_STRAIGHT_TEST=0,
    DIAGNOSTICS=0, MOTION_SMOKE=0, S4=1, S4_BENCH=0, S5=1,
    S5_HARDWARE_CONFIRMED=1, IR_DEBUG=1, IMU_DEBUG=0)


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def rename_path(value):
    """Invert this filename only; the public tilt_estimation symbol is unchanged."""
    return re.sub(r'(?<=/)tilt\.(c|h|o|su)(?=$|\s)', r'tilt_estimation.\1', value)


def link_command(log):
    rows = []
    for line in log.splitlines():
        if line.startswith('arm-none-eabi-gcc ') and ' -o ' in line and '.elf' in line:
            rows.append(shlex.split(line))
    if len(rows) != 1:
        raise ValueError('Expected exactly one actual target link command')
    return [rename_path(argument) for argument in rows[0]]


def verify(evidence, roots):
    evidence = evidence.resolve()
    results = json.loads((evidence/'results.json').read_text())
    if results.get('profile') != PROFILE or results.get('sdk_revision') != SDK:
        raise ValueError('Mechanical pair profile or SDK is not the pinned production configuration')
    if not re.search(r'\b14\.3\.1\b', results.get('compiler', '')):
        raise ValueError('Mechanical pair does not record Arm GCC 14.3.1')
    if results.get('hardware_executed') is not False or set(results.get('builds', {})) != set(REVISIONS):
        raise ValueError('Unexpected mechanical build set or execution claim')
    records, metrics, links = {}, {}, {}
    for label, expected in REVISIONS.items():
        root = roots[label].resolve()
        observed = subprocess.check_output(['git','rev-parse','HEAD'], cwd=root, text=True).strip()
        if observed != expected:
            raise ValueError('Unexpected immutable source revision: '+label)
        if subprocess.run(['git','diff','--quiet','HEAD','--'], cwd=root).returncode:
            raise ValueError('Mechanical source checkout was modified: '+label)
        record = results['builds'][label]
        if record.get('revision') != expected or record.get('returncode') != 0 or record.get('error'):
            raise ValueError('Missing successful bound target build: '+label)
        if record.get('example_configuration_sha256') != digest(root/'config/wifi_secrets.example.h'):
            raise ValueError('Public example configuration binding differs: '+label)
        if (root/'config/wifi_secrets.h').exists() or (root/'config/wifi_secrets.h').is_symlink():
            raise ValueError('Derived source configuration was not removed: '+label)
        directory = evidence/label
        if digest(directory/'flash.bin') != record['flash_sha256']:
            raise ValueError('Flash bytes differ from the build receipt: '+label)
        if set(record.get('artifacts', {})) != {IMAGE+suffix for suffix in ('.elf','.map','.uf2')}:
            raise ValueError('Target artifact set is incomplete: '+label)
        for name, expected_hash in record['artifacts'].items():
            if digest(directory/name) != expected_hash:
                raise ValueError('Target artifact hash differs: '+label+'/'+name)
        metrics[label] = json.loads((directory/'allocation-and-stack.json').read_text())
        actual_symbols = target_metrics.symbol_rows((directory/'symbols.txt').read_text())
        actual_stack = target_metrics.stack_rows(directory/'stack-usage')
        if metrics[label]['symbols'] != actual_symbols or metrics[label]['stack'] != actual_stack:
            raise ValueError('Parsed allocation/stack receipt differs from raw reports: '+label)
        text, data, bss = map(int, (directory/'size.txt').read_text().splitlines()[1].split()[:3])
        if record['sizes'] != dict(text=text,data=data,bss=bss,static_ram=data+bss):
            raise ValueError('Size receipt differs from raw report: '+label)
        links[label] = link_command((directory/'build.log').read_text())
        records[label] = record
    left, right = (records[label] for label in REVISIONS)
    if left['example_configuration_sha256'] != right['example_configuration_sha256']:
        raise ValueError('Public configuration differs between the pair')
    if left['flash_sha256'] != right['flash_sha256']:
        raise ValueError('Filename-only pair changed flashed bytes')
    if left['sizes'] != right['sizes']:
        raise ValueError('Filename-only pair changed target allocation totals')
    if links['baseline'] != links['candidate']:
        raise ValueError('Filename-only pair changed link flags or object order')
    symbols = [Counter((row['name'],row['kind'],row['address'],row['bytes'])
                       for row in metrics[label]['symbols']) for label in REVISIONS]
    stacks = [Counter((rename_path(row['report']),row['function'],row['bytes'],row['qualifier'])
                      for row in metrics[label]['stack']) for label in REVISIONS]
    if not all(symbols) or symbols[0] != symbols[1]:
        raise ValueError('Filename-only pair changed symbol address/size multisets')
    if not all(stacks) or stacks[0] != stacks[1]:
        raise ValueError('Filename-only pair changed compiler stack reports')
    return {'status':'PASS', 'revisions':REVISIONS, 'sdk_revision':SDK,
        'compiler':results['compiler'], 'runner_image':results.get('runner_image'),
        'profile':PROFILE, 'flash_sha256':left['flash_sha256'], 'sizes':left['sizes'],
        'public_configuration_sha256':left['example_configuration_sha256'],
        'flash_bytes_identical':True, 'link_arguments_and_object_order_identical':True,
        'symbol_rows_identical':sum(symbols[0].values()),
        'stack_rows_identical_after_filename_inversion':sum(stacks[0].values()),
        'build_receipt_sha256':digest(evidence/'results.json'),
        'limits':['ELF/debug paths and raw source-location metadata may differ.',
                  'This proves the pinned production image only; no hardware timing or task high-water measurement.',
                  'Later bundled metadata changes are outside this immutable pair.']}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('evidence','baseline','candidate','output'):
        parser.add_argument('--'+name,type=Path,required=True)
    args = parser.parse_args(argv)
    result = {'status':'FAIL','errors':[],'revisions':REVISIONS,'hardware_executed':False}
    try:
        result.update(verify(args.evidence, {label:getattr(args,label) for label in REVISIONS}))
    except (OSError,ValueError,KeyError,subprocess.SubprocessError) as error:
        result['errors'].append(str(error))
    args.output.parent.mkdir(parents=True,exist_ok=True)
    with args.output.open('x',encoding='utf-8') as stream:
        stream.write(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))
    return 0 if result['status']=='PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
