"""Compiler metadata must distinguish real, synthetic, and failed observations."""
import copy
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from testing.compliance import inventory_ast
from tools.compliance import barr_functions, compiler_contexts as contexts, function_union


class ContextTests(unittest.TestCase):
    def test_output_paths_do_not_create_duplicate_semantic_contexts(self):
        directory = Path.cwd()
        first = ['cc', '-std=c99', '-pthread', '-MFone.d', '-MTone.o', '-MQone.o', '-oone.o', '-c', 'one.c']
        second = ['cc', '-std=c99', '-pthread', '-MF', 'two.d', '-MT', 'two.o', '-MQ', 'two.o', '-o', 'two.o', '-c', 'two.c']
        self.assertEqual(inventory_ast.clean_arguments(first, directory), ['-std=c99','-pthread'])
        self.assertEqual(inventory_ast.clean_arguments(first, directory), inventory_ast.clean_arguments(second, directory))

    def fixture(self, directory):
        root = Path(directory)/'subject'
        (root/'app').mkdir(parents=True)
        (root/'testing/compliance').mkdir(parents=True)
        (root/'app/a.h').write_text('static inline int helper(void) { return 1; }\n')
        (root/'app/a.c').write_text('#include "a.h"\nint answer(void) { return helper(); }\n')
        (root/'app/unrecorded.c').write_text('int unrecorded(void) { return 2; }\n')
        (root/'testing/compliance/inventory_scope.json').write_text(json.dumps({'include_globs':['app/*.[ch]'], 'exclude_globs':[]}))
        database = Path(directory)/'commands.json'
        database.write_text(json.dumps([{'directory':str(root), 'file':'app/a.c',
            'arguments':['clang', '-std=c99', '-c', 'app/a.c', '-MFapp/a.d']}]))
        resource = Path(directory)/'resource'
        (resource/'include').mkdir(parents=True)
        return root.resolve(), database, resource

    def test_recorded_only_observes_header_through_consumer_and_leaves_unknown_source(self):
        with tempfile.TemporaryDirectory() as directory:
            root, database, resource = self.fixture(directory)
            with patch.object(contexts, 'revision', return_value='revision'), patch.object(contexts, 'pinned_resource', return_value=(resource, {'version':'test'})):
                result = barr_functions.collect(root, [database], ['configured'], 1,
                    recorded_only=True, profile_label='host-gcc')
            self.assertEqual(len(result['translation_units']), 1)
            self.assertEqual(result['context_counts'], {'recorded-command':1})
            self.assertEqual(result['files']['app/unrecorded.c']['status'], 'unobserved')
            self.assertEqual(result['files']['app/a.h']['status'], 'observed')
            self.assertEqual({row['name'] for row in result['functions']}, {'helper','answer'})
            receipt = result['translation_units'][0]
            self.assertEqual(receipt['original_arguments'][-1], '-MFapp/a.d')
            self.assertNotIn('-MFapp/a.d', receipt['arguments'])
            self.assertEqual(set(receipt['owned_inputs']), {'app/a.c','app/a.h'})

    def test_successful_fallback_and_derived_profiles_remain_incomplete(self):
        with tempfile.TemporaryDirectory() as directory:
            root, database, resource = self.fixture(directory)
            with patch.object(contexts, 'revision', return_value='revision'), patch.object(contexts, 'pinned_resource', return_value=(resource, {'version':'test'})):
                result = barr_functions.collect(root, [database], ['configured','diagnostics'], 1)
            unrecorded = next(row for row in result['functions'] if row['name']=='unrecorded')
            self.assertEqual(unrecorded['method'], 'libclang-fallback')
            self.assertEqual(result['files']['app/unrecorded.c']['status'], 'partial')
            answer = next(row for row in result['functions'] if row['name']=='answer')
            self.assertEqual({row['method'] for row in answer['context_observations']}, {'libclang-ast','libclang-derived'})
            self.assertFalse(result['coverage_complete'])

    def test_target_command_is_not_reused_for_unknown_host_fixture(self):
        with tempfile.TemporaryDirectory() as directory:
            root, database, resource = self.fixture(directory)
            host = root/'testing/fixture.c'
            host.write_text('int main(void) { return 0; }\n')
            planned = contexts.plan(root, [database], {root/'app/a.c', host}, ['configured'], resource)
            self.assertEqual([job[0][0] for job in planned], [root/'app/a.c'])

    def test_missing_pinned_parser_resource_retains_incomplete_receipt(self):
        with tempfile.TemporaryDirectory() as directory:
            root, database, _ = self.fixture(directory)
            output = Path(directory)/'functions.json'
            with patch.object(contexts, 'revision', return_value='revision'), patch.object(contexts, 'pinned_resource', side_effect=ValueError('Pinned resource unavailable')):
                code = barr_functions.main(['--root',str(root),'--compile-db',str(database),'--output',str(output)])
            report = json.loads(output.read_text())
            self.assertEqual(code,2)
            self.assertEqual(report['status'],'INCOMPLETE')
            self.assertFalse(report['coverage_complete'])
            self.assertEqual(report['errors'],['Pinned resource unavailable'])

    def test_same_revision_mapping_rejects_changed_header(self):
        with tempfile.TemporaryDirectory() as directory:
            root, _, _ = self.fixture(directory)
            other = Path(directory)/'tooling'
            (other/'app').mkdir(parents=True)
            (other/'app/a.h').write_text('changed\n')
            with patch.object(contexts, 'revision', return_value='same'), self.assertRaisesRegex(ValueError, 'source differs'):
                contexts.equivalent_roots(root, [other], {root/'app/a.h'})

    def test_equivalent_root_mapping_respects_path_boundaries(self):
        with tempfile.TemporaryDirectory() as directory:
            root, alternative = Path(directory)/'subject', Path(directory)/'tooling'
            self.assertEqual(contexts.remap('-I'+str(alternative/'inc'), root, [alternative]), '-I'+str(root/'inc'))
            unchanged = str(alternative)+'-other/a.c'
            self.assertEqual(contexts.remap(unchanged, root, [alternative]), unchanged)
            for unchanged in ('/prefix'+str(alternative)+'/a.c', '-DROOT='+str(alternative)):
                self.assertEqual(contexts.remap(unchanged, root, [alternative]), unchanged)

    def test_forced_include_and_output_are_not_standalone_source_commands(self):
        arguments = ['cc','-include','forced.c','-D','TEXT=macro.c','actual.c','-o','output.c']
        self.assertEqual(contexts.source_operands(arguments), ['actual.c'])
        with tempfile.TemporaryDirectory() as directory:
            root, database, resource = self.fixture(directory)
            rows = json.loads(database.read_text())
            rows[0]['arguments'] = ['cc','-include','app/unrecorded.c','app/a.c']
            rows[0]['file'] = 'app/unrecorded.c'
            database.write_text(json.dumps(rows))
            with self.assertRaisesRegex(ValueError, 'not a positional C input'):
                contexts.plan(root,[database],{root/'app/a.c',root/'app/unrecorded.c'},['configured'],resource)

    def test_non_source_context_inputs_are_rechecked_after_parse(self):
        for changed in ('scope','database','template','derived','settings-presence'):
            with self.subTest(changed=changed), tempfile.TemporaryDirectory() as directory:
                root, database, resource = self.fixture(directory)
                (root/'config').mkdir()
                template = root/'config/wifi_secrets.example.h'
                template.write_text('#define WIFI_SSID "public-example"\n')
                output = Path(directory)/'derived'
                original = barr_functions.parse_functions
                def mutate(*args):
                    result = original(*args)
                    paths = {'scope':root/'testing/compliance/inventory_scope.json',
                             'database':database, 'template':template,
                             'derived':output/'wifi_secrets.h'}
                    if changed == 'settings-presence':
                        (root/'config/wifi_secrets.h').touch()
                    else:
                        with paths[changed].open('a') as stream:
                            stream.write('\n')
                    return result
                with patch.object(contexts,'revision',return_value='revision'), \
                        patch.object(contexts,'pinned_resource',return_value=(resource,{'version':'test'})), \
                        patch.object(barr_functions,'parse_functions',side_effect=mutate), \
                        self.assertRaisesRegex(ValueError,'input changed|settings appeared'):
                    barr_functions.collect(root,[database],['configured'],1,recorded_only=True,public_config_output=output)

    def test_public_template_derivation_does_not_mutate_source_or_overwrite_private_settings(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)/'subject'
            (root/'config').mkdir(parents=True)
            template = root/'config/wifi_secrets.example.h'
            template.write_bytes(b'#define WIFI_SSID "public-example"\n')
            arguments, receipt = contexts.public_configuration(root, Path(directory)/'derived')
            self.assertEqual((Path(directory)/'derived/wifi_secrets.h').read_bytes(), template.read_bytes())
            self.assertEqual(receipt['derived_bytes_sha256'], receipt['template_bytes_sha256'])
            self.assertFalse((root/'config/wifi_secrets.h').exists())
            self.assertEqual(arguments, ['-I'+str((Path(directory)/'derived').resolve())])
            private = root/'config/wifi_secrets.h'
            private.write_text('unrelated settings\n')
            original = Path.read_bytes
            def reject_private(path):
                if path == private:
                    self.fail('Private settings must not be read')
                return original(path)
            with patch.object(Path, 'read_bytes', reject_private), self.assertRaisesRegex(ValueError, 'settings to be absent'):
                contexts.public_configuration(root, Path(directory)/'other')
            self.assertEqual(private.read_text(), 'unrelated settings\n')

    def test_clang_resource_stdint_precedes_gcc_private_stdint(self):
        from clang import cindex
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            resource = root/'clang18'
            (resource/'include').mkdir(parents=True)
            (resource/'include/stdint.h').write_text('typedef unsigned long long uint64_t;\n#define UINT64_C(x) x##ULL\n')
            gcc = root/'arm/lib/gcc/arm-none-eabi/14.3.1/include'
            gcc.mkdir(parents=True)
            (gcc/'stdint.h').write_text('#error GCC private stdint requires incompatible compiler builtins\n')
            (root/'arm/arm-none-eabi/include').mkdir(parents=True)
            arguments = inventory_ast.compiler_include_arguments(str(root/'arm/bin/arm-none-eabi-gcc'), resource)
            source = str(root/'probe.c')
            text = '#include <stdint.h>\nuint64_t f(void) { return UINT64_C(1); }\n'
            unit = cindex.Index.create().parse(source, args=arguments+['-ffreestanding','-x','c'], unsaved_files=[(source,text)])
            self.assertEqual([str(item) for item in unit.diagnostics], [])
            self.assertEqual(Path(next(unit.get_includes()).include.name), resource/'include/stdint.h')


class UnionTests(unittest.TestCase):
    def reports(self):
        binding = dict(semantic_sha256='semantic', source_location_sha256='location', source_content_sha256='content')
        reports = []
        for label in sorted(function_union.EXPECTED):
            context = dict(context_id=label, collector_profile=label, origin='recorded-command',
                profile='configured', errors=[], diagnostics=[], owned_inputs=['a.c'],
                recorded_source=True, profile_overrides={},
                original_arguments=['cc','a.c'], arguments=['-std=c99'], database='database', database_sha256='db')
            row = dict(path='a.c', line=1, end_line=1, name='answer', method='libclang-ast',
                profiles=['configured'], definition_documented=True,
                context_observations=[dict(context_id=label, origin='recorded-command', method='libclang-ast')])
            reports.append(dict(schema_version=3, revision='same', scope_sha256='scope', collector_profile=label,
                source_root=str(Path('/subject')), context_inputs_unchanged=True,
                context_input_bindings={str(Path('/subject/testing/compliance/inventory_scope.json')):'scope','database':'db'},
                recorded_only=label.startswith('host-'), files={'a.c':{**binding,'input_bindings':{'a.c':binding}}},
                translation_units=[context], functions=[row], libclang_package='18.1.1',
                resource_headers={'version':'clang version 18.1.3','stdint_header_sha256':'0'*64}, definition_count=1,
                context_counts={'recorded-command':1}))
        return reports, {'a.c':binding}

    def combine(self, reports, bindings):
        return function_union.combine(reports, 'same', bindings, 'scope')

    def test_recovery_context_survives_clean_host_observation(self):
        reports, bindings = self.reports()
        target = next(report for report in reports if report['collector_profile']=='target-production')
        target['translation_units'][0]['errors'] = ['target context failed']
        target['functions'][0]['method'] = 'libclang-recovery'
        target['functions'][0]['context_observations'][0]['method'] = 'libclang-recovery'
        result = self.combine(reports, bindings)
        self.assertEqual(result['functions'][0]['method'], 'libclang-ast')
        self.assertEqual(result['diagnostic_translation_unit_count'], 1)
        self.assertEqual(result['recovery_function_observation_count'], 1)
        self.assertEqual(len(result['functions'][0]['context_observations']), 3)
        self.assertFalse(result['coverage_complete'])

    def test_missing_duplicate_or_wrong_revision_receipt_is_rejected(self):
        reports, bindings = self.reports()
        for bad in (reports[:-1], reports+[reports[0]], [reports[0],reports[0],reports[2]]):
            with self.subTest(labels=[row['collector_profile'] for row in bad]), self.assertRaises(ValueError):
                self.combine(bad, bindings)
        reports[0]['revision'] = 'stale'
        with self.assertRaisesRegex(ValueError, 'revision'):
            self.combine(reports, bindings)

    def test_stale_source_and_included_header_bindings_are_rejected(self):
        for inside in (False,True):
            reports, bindings = self.reports()
            reports[0]['files'] = copy.deepcopy(reports[0]['files'])
            changed = reports[0]['files']['a.c']
            if inside:
                changed = changed['input_bindings']['a.c']
            changed['source_content_sha256'] = 'stale'
            with self.subTest(included=inside), self.assertRaisesRegex(ValueError, 'binding'):
                self.combine(reports, bindings)

    def test_synthetic_context_cannot_be_silently_promoted(self):
        reports, bindings = self.reports()
        target = next(report for report in reports if report['collector_profile']=='target-production')
        target['translation_units'][0]['origin'] = 'fallback-unrecorded-source'
        target['translation_units'][0]['recorded_source'] = False
        target['functions'][0]['context_observations'][0].update(origin='fallback-unrecorded-source',method='libclang-fallback')
        with self.assertRaisesRegex(ValueError, 'Aggregate function method'):
            self.combine(reports, bindings)

    def test_missing_exact_command_or_foreign_context_is_rejected(self):
        for change in ('arguments','context'):
            reports, bindings = self.reports()
            if change=='arguments':
                reports[0]['translation_units'][0]['original_arguments'] = []
            else:
                reports[0]['functions'][0]['context_observations'][0]['context_id'] = 'unknown'
            with self.subTest(change=change), self.assertRaises(ValueError):
                self.combine(reports, bindings)

    def test_recorded_context_cannot_hide_profile_override_metadata(self):
        reports, bindings = self.reports()
        reports[0]['translation_units'][0]['profile_overrides'] = {'APP_TEST':1}
        with self.assertRaisesRegex(ValueError, 'derivation metadata'):
            self.combine(reports, bindings)

    def test_unverified_context_inputs_or_changed_database_hash_fail_union(self):
        for change in ('preservation','database'):
            reports, bindings = self.reports()
            if change == 'preservation':
                reports[0]['context_inputs_unchanged'] = False
            else:
                reports[0]['context_input_bindings']['database'] = 'changed'
            with self.subTest(change=change), self.assertRaises(ValueError):
                self.combine(reports, bindings)

    def test_artifact_directory_must_match_profile_and_revision(self):
        root = Path('receipts')
        good = root/'function-inventory-host-gcc-current/function-inventory.json'
        function_union.validate_receipt_path(good, root, {'collector_profile':'host-gcc'}, 'current')
        for label, revision in [('host-clang','current'),('host-gcc','stale')]:
            with self.subTest(label=label,revision=revision), self.assertRaises(ValueError):
                function_union.validate_receipt_path(good, root, {'collector_profile':label}, revision)

    def test_failed_union_retains_explicit_incomplete_receipt(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root/'testing/compliance').mkdir(parents=True)
            (root/'testing/compliance/inventory_scope.json').write_text('{"include_globs":[],"exclude_globs":[]}')
            output = root/'result.json'
            with patch.object(contexts, 'revision', return_value='current'):
                code = function_union.main(['--root',str(root),'--receipts',str(root/'missing'),'--output',str(output)])
            result = json.loads(output.read_text())
            self.assertEqual(code,2)
            self.assertEqual(result['status'],'INCOMPLETE')
            self.assertTrue(result['errors'])
            self.assertFalse(result['coverage_complete'])


if __name__ == '__main__':
    unittest.main()
