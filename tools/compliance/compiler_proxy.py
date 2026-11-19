#!/usr/bin/env python3
"""Record compiler invocations unchanged, then execute the pinned compiler."""
import json
import os
from pathlib import Path
import sys


def main():
    compiler = os.environ['COMPLIANCE_REAL_CC']
    row = dict(directory=str(Path.cwd()), arguments=[compiler, *sys.argv[1:]])
    descriptor = os.open(os.environ['COMPLIANCE_COMPILE_LOG'],
                         os.O_WRONLY | os.O_CREAT | os.O_APPEND, 0o600)
    try:
        os.write(descriptor, (json.dumps(row)+'\n').encode('utf-8'))
    finally:
        os.close(descriptor)
    os.execv(compiler, [compiler, *sys.argv[1:]])


if __name__ == '__main__':
    main()
