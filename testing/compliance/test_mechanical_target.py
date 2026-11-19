"""The immutable image gate rejects missing, stale or changed evidence."""
import copy
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from tools.compliance import target_metrics
from tools.compliance import verify_mechanical_target as gate


class MechanicalTargetTests(unittest.TestCase):
    def fixture(self, temporary):
        base = Path(temporary)
        evidence = base/'evidence'
        evidence.mkdir()
        roots = {label:base/label for label in gate.REVISIONS}
        result = dict(profile=gate.PROFILE,sdk_revision=gate.SDK,
                      compiler='arm-none-eabi-gcc 14.3.1',hardware_executed=False,builds={})
        for label, root in roots.items():
            (root/'config').mkdir(parents=True)
            (root/'config/wifi_secrets.example.h').write_text('public example\n')
            folder = evidence/label
            (folder/'stack-usage').mkdir(parents=True)
            stem = 'tilt_estimation' if label=='baseline' else 'tilt'
            (folder/'stack-usage'/f'{stem}.su').write_text(f'../app/{stem}.c:1:1:tilt_heading\t8\tstatic\n')
            # Real reports contain their directory prefix; retain that for the
            # explicit filename inverse, without changing public symbols.
            sub = folder/'stack-usage/app'
            sub.mkdir()
            (folder/'stack-usage'/f'{stem}.su').rename(sub/f'{stem}.su')
            (folder/'symbols.txt').write_text('tilt_heading T 1000 18\n')
            (folder/'size.txt').write_text(f'text data bss dec hex filename\n24 4 8 36 24 {gate.IMAGE}.elf\n')
            (folder/'build.log').write_text(f'arm-none-eabi-gcc -mcpu=cortex-m0plus -o {gate.IMAGE}.elf app/{stem}.o other.o -lm\n')
            artifacts=[gate.IMAGE+suffix for suffix in ('.elf','.map','.uf2')]
            for name in ('flash.bin',*artifacts):
                (folder/name).write_bytes(b'actual report '+name.encode())
            target_metrics.collect(folder)
            result['builds'][label] = dict(revision=gate.REVISIONS[label],returncode=0,
                example_configuration_sha256=gate.digest(root/'config/wifi_secrets.example.h'),
                flash_sha256=gate.digest(folder/'flash.bin'),
                artifacts={name:gate.digest(folder/name) for name in artifacts},
                sizes=dict(text=24,data=4,bss=8,static_ram=12))
        (evidence/'results.json').write_text(json.dumps(result))
        return evidence, roots, result

    def verify(self, evidence, roots):
        with patch.object(gate.subprocess,'check_output',side_effect=lambda command,cwd,text:gate.REVISIONS[Path(cwd).name]), \
                patch.object(gate.subprocess,'run') as process:
            process.return_value.returncode=0
            return gate.verify(evidence,roots)

    def test_exact_immutable_pair_passes_with_filename_metadata_inversion(self):
        with tempfile.TemporaryDirectory() as temporary:
            evidence,roots,_=self.fixture(temporary)
            result=self.verify(evidence,roots)
            self.assertEqual(result['status'],'PASS')
            self.assertEqual(result['stack_rows_identical_after_filename_inversion'],1)
            self.assertEqual(gate.rename_path('tilt_estimation'),'tilt_estimation')

    def test_actual_55af_production_artifact_and_link_command_contract(self):
        actual=json.loads((Path(__file__).parent/'fixtures/tilt_target_55af85c.json').read_text())
        self.assertEqual(actual['revision'],gate.REVISIONS['baseline'])
        self.assertEqual(actual['profile'],gate.PROFILE)
        self.assertEqual(set(actual['artifacts']),{gate.IMAGE+suffix for suffix in ('.elf','.uf2','.map')})
        arguments=gate.link_command(actual['link_line'])
        self.assertEqual(len(arguments),193)
        self.assertEqual(arguments[0],'arm-none-eabi-gcc')
        self.assertEqual(arguments[arguments.index('-o')+1],gate.IMAGE+'.elf')
        self.assertIn('./mtkernel_3/app_program/subsystem4/tilt_estimation.o',arguments)
        renamed=actual['link_line'].replace('/tilt_estimation.o','/tilt.o')
        self.assertEqual(gate.link_command(renamed),arguments)

    def test_changed_flash_fails_even_if_receipt_rehashes_changed_bytes(self):
        with tempfile.TemporaryDirectory() as temporary:
            evidence,roots,results=self.fixture(temporary)
            (evidence/'candidate/flash.bin').write_bytes(b'changed')
            results['builds']['candidate']['flash_sha256']=gate.digest(evidence/'candidate/flash.bin')
            (evidence/'results.json').write_text(json.dumps(results))
            with self.assertRaisesRegex(ValueError,'changed flashed bytes'):
                self.verify(evidence,roots)

    def test_stale_revision_failed_build_and_wrong_profile_are_rejected(self):
        for change in ('revision','returncode','profile','sdk'):
            with self.subTest(change=change),tempfile.TemporaryDirectory() as temporary:
                evidence,roots,results=self.fixture(temporary)
                if change in {'revision','returncode'}:
                    results['builds']['candidate'][change]='changed'
                elif change=='profile':
                    results['profile']=copy.deepcopy(results['profile'])
                    results['profile']['TEST']=1
                else:
                    results['sdk_revision']='changed'
                (evidence/'results.json').write_text(json.dumps(results))
                with self.assertRaises(ValueError):
                    self.verify(evidence,roots)

    def test_artifact_mutation_link_order_and_raw_stack_change_are_rejected(self):
        for change in ('artifact','link','stack'):
            with self.subTest(change=change),tempfile.TemporaryDirectory() as temporary:
                evidence,roots,_=self.fixture(temporary)
                folder=evidence/'candidate'
                if change=='artifact':
                    (folder/(gate.IMAGE+'.elf')).write_bytes(b'changed')
                elif change=='link':
                    (folder/'build.log').write_text(f'arm-none-eabi-gcc -mcpu=cortex-m0plus -o {gate.IMAGE}.elf other.o app/tilt.o -lm\n')
                else:
                    (folder/'stack-usage/app/tilt.su').write_text('../app/tilt.c:1:1:tilt_heading\t16\tstatic\n')
                with self.assertRaises(ValueError):
                    self.verify(evidence,roots)

    def test_missing_inputs_retain_failure_receipt(self):
        with tempfile.TemporaryDirectory() as temporary:
            root=Path(temporary)
            output=root/'failure.json'
            code=gate.main(['--evidence',str(root/'missing'),'--baseline',str(root/'left'),
                            '--candidate',str(root/'right'),'--output',str(output)])
            result=json.loads(output.read_text())
            self.assertEqual(code,1)
            self.assertEqual(result['status'],'FAIL')
            self.assertTrue(result['errors'])


if __name__=='__main__':
    unittest.main()
