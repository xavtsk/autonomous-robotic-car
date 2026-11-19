"""Check compiler-command isolation without executing generated native code."""
from pathlib import Path
import unittest

from tools.compliance.check_host_c99 import syntax_commands


class C99CommandTests(unittest.TestCase):
    def setUp(self):
        self.root = Path(__file__).resolve().parents[2]
        self.directory = self.root/'testing/subsystem1'

    def test_preserves_profile_and_headers_but_compiles_only_application_source(self):
        row = dict(directory=str(self.directory), arguments=[
            '/usr/bin/clang-18', '-std=c11', '-DAPP_IR_DEBUG=1', '-include', 'host_kernel.h',
            '-Ifakes', '-Wall', '-Werror', 'host_test.c',
            '../../app_program/subsystem1/command.c',
            '../../lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c',
            '-lm', '-pthread', '-o', '/tmp/host_test'])
        commands = syntax_commands(row, self.root)
        self.assertEqual(len(commands), 1)
        source, command = commands[0]
        self.assertEqual(source, self.root/'app_program/subsystem1/command.c')
        self.assertEqual(command, ['/usr/bin/clang-18', '-DAPP_IR_DEBUG=1', '-include',
            'host_kernel.h', '-Ifakes', '-Wall', '-Werror', '-pthread',
            '-std=c99', '-pedantic-errors', '-fsyntax-only', str(source)])

    def test_header_only_probe_or_imported_source_cannot_count_as_application_coverage(self):
        for arguments in (['cc', '--version'], ['cc', '-x', 'c', '-'],
                          ['cc', '../../lib/device/some_driver.c', '-c', '-o', 'driver.o']):
            self.assertEqual(syntax_commands(dict(directory=str(self.directory),
                                                 arguments=arguments), self.root), [])

    def test_each_application_source_gets_its_own_translation_unit(self):
        rows = syntax_commands(dict(directory=str(self.directory), arguments=[
            'cc', '../../app_program/subsystem1/command.c',
            '../../app_program/subsystem1/telemetry_codec.c', '-o', 'host']), self.root)
        self.assertEqual(len(rows), 2)
        for source, command in rows:
            self.assertEqual(command[-1], str(source))
            self.assertEqual(sum(item.endswith('.c') for item in command), 1)


if __name__ == '__main__':
    unittest.main()
