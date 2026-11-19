#!/usr/bin/env python3
"""Build matched target profiles in separate CI checkouts; never flash hardware."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import shlex
import shutil
import subprocess
import sys

SDK_SHA = '98a542c1a62fb549ffb5d66a3e5892b06276b670'
COMMON = dict(SMP=1, CONSOLE='usb_cdc', CRASH_REPORT=0, WIFI='none', NET='none',
              MQTT=0, TEST=0, IR_BENCH=0, IR_CHANNELS=1, S3_MOTION=0,
              S3_STRAIGHT_TEST=0, DIAGNOSTICS=0, MOTION_SMOKE=0, S4=0,
              S4_BENCH=0, S5=0, S5_HARDWARE_CONFIRMED=0, IR_DEBUG=0, IMU_DEBUG=0)
LIVE = dict(WIFI='cyw43', NET='lwip', MQTT=1, S3_MOTION=1,
            S4=1, S5=1, S5_HARDWARE_CONFIRMED=1, IR_DEBUG=1)
PROFILES = {
    'production': LIVE,
    'diagnostics': dict(LIVE, DIAGNOSTICS=1),
    'imu_debug': dict(LIVE, IMU_DEBUG=1),
    'offline_motion': dict(S3_MOTION=1, S4=1, S5=1, S5_HARDWARE_CONFIRMED=1),
    'stationary_1': dict(IR_BENCH=1),
    'stationary_3': dict(IR_BENCH=1, IR_CHANNELS=3),
    'self_tests': dict(TEST=1),
    'motion_smoke': dict(MOTION_SMOKE=1),
    's4_bench': dict(S4=1, S4_BENCH=1),
    'single_core': dict(LIVE, SMP=0),
    'straight_test': dict(LIVE, S3_STRAIGHT_TEST=1),
    's5_unconfirmed': dict(LIVE, S5_HARDWARE_CONFIRMED=0),
}


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def output_of(command, cwd=None):
    return subprocess.check_output(command, cwd=cwd, text=True).strip()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('baseline', 'candidate', 'sdk', 'output'):
        parser.add_argument('--' + name, type=Path, required=True)
    parser.add_argument('--profile', choices=tuple(PROFILES), required=True)
    args = parser.parse_args()
    if sys.platform != 'linux':
        parser.error('This CI build runner requires Linux.')
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    sdk = args.sdk.resolve()
    sdk_revision = output_of(['git', 'rev-parse', 'HEAD'], sdk)
    if sdk_revision != SDK_SHA:
        raise RuntimeError('Pico SDK revision differs from the pinned SDK')
    if output_of(['arm-none-eabi-gcc', '-dumpfullversion']) != '14.3.1':
        raise RuntimeError('Expected Arm compiler 14.3.1')
    profile = dict(COMMON, **PROFILES[args.profile])
    manifest = dict(recorded_at=datetime.now(timezone.utc).isoformat(), profile=profile,
                    sdk_revision=sdk_revision, compiler=output_of(['arm-none-eabi-gcc', '--version']),
                    configurations='Identical checked-in public example; no real credentials.',
                    hardware_executed=False, builds={})
    # Append a compile-only reporting flag without overriding the Makefile's
    # CPU, feature and profile flags. Both revisions use the identical wrapper.
    wrapper = output / 'arm-stack-gcc'
    wrapper.write_text('#!/bin/sh\nexec arm-none-eabi-gcc -fstack-usage "$@"\n', encoding='utf-8')
    wrapper.chmod(0o755)
    manifest['stack_usage'] = 'Compiler .su reports only; not call-chain maxima, measured task high-water or free heap.'
    failed = False
    for label in ('baseline', 'candidate'):
        root = getattr(args, label).resolve()
        dest = output / label
        dest.mkdir()
        record = dict(revision=output_of(['git', 'rev-parse', 'HEAD'], root))
        manifest['builds'][label] = record
        # Only disposable GitHub Actions checkouts may receive this example.
        config = root / 'config/wifi_secrets.h'
        if config.exists():
            raise RuntimeError(f'Refusing to overwrite an existing local configuration in {root}')
        shutil.copyfile(root / 'config/wifi_secrets.example.h', config)
        record['example_configuration_sha256'] = digest(config)
        command = ['make', '-C', str(root / 'build_make'), 'PICO_SDK_PATH=' + str(sdk), 'GCC=' + str(wrapper),
                   *[f'{key}={value}' for key, value in profile.items()],
                   '-j' + str(min(os.cpu_count() or 2, 4))]
        record['command'] = command
        try:
            with (dest / 'build.log').open('w', encoding='utf-8') as log:
                result = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, timeout=1200)
            record['returncode'] = result.returncode
            if result.returncode:
                failed = True
                print((dest / 'build.log').read_text(errors='replace')[-6000:])
                continue
            images = list((root / 'build_make').glob('*.elf'))
            if len(images) != 1:
                raise RuntimeError(f'Expected one fresh ELF, found {len(images)}')
            elf = images[0]
            size_text = output_of(['arm-none-eabi-size', str(elf)])
            (dest / 'size.txt').write_text(size_text + '\n', encoding='utf-8')
            text, data, bss = map(int, size_text.splitlines()[1].split()[:3])
            record['sizes'] = dict(text=text, data=data, bss=bss, static_ram=data+bss)
            record['artifacts'] = {}
            for suffix in ('.elf', '.uf2', '.map'):
                source = elf.with_suffix(suffix)
                shutil.copyfile(source, dest / source.name)
                record['artifacts'][source.name] = digest(source)
            subprocess.run(['arm-none-eabi-objcopy', '-O', 'binary', str(elf), str(dest / 'flash.bin')], check=True)
            record['flash_sha256'] = digest(dest / 'flash.bin')
            stack_files = sorted((root / 'build_make').rglob('*.su'))
            for source in stack_files:
                target = dest / 'stack-usage' / source.relative_to(root / 'build_make')
                target.parent.mkdir(parents=True, exist_ok=True)
                shutil.copyfile(source, target)
            record['stack_usage_files'] = len(stack_files)
            if not stack_files:
                raise RuntimeError('Compiler produced no stack-usage reports')
            # Capture the commands that actually built the image, never make -n:
            # this project's parse-time profile guard can mutate even a dry run.
            commands = []
            for line in (dest / 'build.log').read_text(errors='replace').splitlines():
                if (line.startswith('arm-none-eabi-gcc ') or line.startswith(str(wrapper) + ' ')) and ' -c ' in line:
                    argv = shlex.split(line)
                    if argv[0] == str(wrapper):
                        argv[0] = 'arm-none-eabi-gcc'
                        argv.insert(1, '-fstack-usage')
                    if '-o' not in argv:
                        continue
                    source = next((part for part in reversed(argv) if part.endswith('.c')), None)
                    if source:
                        commands.append(dict(directory=str(root / 'build_make'), arguments=argv,
                                             file=str((root / 'build_make' / source).resolve())))
            (dest / 'compile_commands.json').write_text(json.dumps(commands, indent=2) + '\n', encoding='utf-8')
            print(f'PASS {label} {args.profile}: static RAM {data+bss} B')
        except (OSError, ValueError, RuntimeError, subprocess.SubprocessError) as exc:
            record['error'] = str(exc)
            failed = True
            print(f'FAIL {label}: {exc}')
        finally:
            config.unlink(missing_ok=True)
            (output / 'results.json').write_text(json.dumps(manifest, indent=2) + '\n', encoding='utf-8')
    if all('sizes' in item for item in manifest['builds'].values()):
        before, after = manifest['builds']['baseline'], manifest['builds']['candidate']
        manifest['comparison'] = dict(static_ram_delta=after['sizes']['static_ram']-before['sizes']['static_ram'],
                                      flash_identical=before['flash_sha256']==after['flash_sha256'],
                                      note='Binary equality is reported, not assumed for representation changes.')
    (output / 'results.json').write_text(json.dumps(manifest, indent=2) + '\n', encoding='utf-8')
    return failed


if __name__ == '__main__':
    sys.exit(main())
