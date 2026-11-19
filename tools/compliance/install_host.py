#!/usr/bin/env python3
"""Install reviewed tool versions; record the hosted runner image separately."""
import json
import os
from pathlib import Path
import subprocess
import sys

if sys.platform != 'linux':
    sys.exit('This installer is only for the isolated Linux CI runner.')
lock = json.loads(Path(__file__).with_name('toolchain.lock.json').read_text())
image = os.environ.get('ImageVersion')
print('Hosted runner image', image, '(recorded, not selectable by immutable image ID)', flush=True)
subprocess.run(['sudo', 'apt-get', 'update'], check=True)
packages = [name + '=' + version for name, version in lock['host_packages'].items()]
# Deliberately request these exact reviewed versions even when the hosted
# image has newer tools. No unversioned tool downgrade is requested.
subprocess.run(['sudo', 'apt-get', 'install', '-y', '--allow-downgrades', *packages], check=True)
for name, expected in lock['host_packages'].items():
    actual = subprocess.check_output(['dpkg-query', '-W', '-f=${Version}', name], text=True)
    if actual != expected:
        sys.exit(f'{name}: expected {expected}, found {actual}')
    print(name, actual, flush=True)
print('Runner image', image)
