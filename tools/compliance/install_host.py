#!/usr/bin/env python3
"""Install reviewed tool versions; record the hosted runner image separately."""
import json
import argparse
import os
from pathlib import Path
import subprocess
import sys

if sys.platform != 'linux':
    sys.exit('This installer is only for the isolated Linux CI runner.')
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--formatter', action='store_true')
args = parser.parse_args()
lock = json.loads(Path(__file__).with_name('toolchain.lock.json').read_text())
image = os.environ.get('ImageVersion')
print('Hosted runner image', image, '(recorded, not selectable by immutable image ID)', flush=True)
subprocess.run(['sudo', 'apt-get', 'update'], check=True)
versions = dict(lock['host_packages'])
if args.formatter:
    versions.update(lock['static_packages'])
packages = [name + '=' + version for name, version in versions.items()]
# Deliberately request these exact reviewed versions even when the hosted
# image has newer tools. No unversioned tool downgrade is requested.
subprocess.run(['sudo', 'apt-get', 'install', '-y', '--allow-downgrades', *packages], check=True)
for name, expected in versions.items():
    actual = subprocess.check_output(['dpkg-query', '-W', '-f=${Version}', name], text=True)
    if actual != expected:
        sys.exit(f'{name}: expected {expected}, found {actual}')
    print(name, actual, flush=True)
print('Runner image', image)
