"""Integrity regressions for source/field coverage; no C binary is executed."""
import copy
import csv
import io
import importlib.util
import hashlib
import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest import mock

HERE = Path(__file__).resolve().parent
SPEC = importlib.util.spec_from_file_location("cr1_inventory", HERE / "inventory.py")
inventory = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(inventory)
AST_SPEC = importlib.util.spec_from_file_location("cr1_inventory_ast", HERE / "inventory_ast.py")
inventory_ast = importlib.util.module_from_spec(AST_SPEC)
AST_SPEC.loader.exec_module(inventory_ast)


class DiscoveryTests(unittest.TestCase):
    def require_libclang(self):
        try:
            from clang import cindex
            cindex.Index.create()
        except (ImportError, OSError):
            if os.environ.get("CR1_REQUIRE_LIBCLANG") == "1":
                self.fail("CR1_REQUIRE_LIBCLANG=1 requires pinned libclang==18.1.1; compiler audit cannot be skipped")
            self.skipTest("Optional pinned libclang is not installed")

    def test_storage_duration_distinguishes_retained_local_from_automatic(self):
        self.assertEqual(inventory_ast.storage_duration("VAR_DECL", "FUNCTION_DECL", "NONE"), "automatic")
        self.assertEqual(inventory_ast.storage_duration("VAR_DECL", "FUNCTION_DECL", "STATIC"), "static")
        self.assertEqual(inventory_ast.storage_duration("VAR_DECL", "FUNCTION_DECL", "EXTERN"), "static")
        self.assertEqual(inventory_ast.storage_duration("VAR_DECL", "TRANSLATION_UNIT", "NONE"), "static")
        self.assertEqual(inventory_ast.storage_duration("FIELD_DECL", "STRUCT_DECL", "INVALID"), "enclosing-object")

    def test_reused_local_record_names_have_distinct_declarations(self):
        source = "void first(void) { union { unsigned long long bits; } value; }\n" \
                 "void second(void) { union { unsigned int bits; } value; }\n"
        rows, unsupported = inventory.lexical_fields(source, "app.c")
        self.assertEqual(unsupported, [])
        self.assertEqual([row["owner"] for row in rows], ["value", "value@2"])
        self.assertEqual(len({inventory.row_id(row) for row in rows}), 2)

    def test_compiler_observes_address_use_and_function_linkage(self):
        self.require_libclang()
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory).resolve()
            file = root / "fixture.c"
            header = root / "fixture.h"
            header.write_text("struct state { int count; };\n")
            file.write_text('#include "fixture.h"\nstatic struct state saved;\n'
                            "static void private_fn(int arg) { static int held; int local = arg; int *p = &saved.count; }\n"
                            "void public_fn(int external) { private_fn(external); }\n"
                            "void first(void) { union { unsigned long long bits; } value; }\n"
                            "void second(void) { union { unsigned int bits; } value; }\n")
            fields, _ = inventory.lexical_fields(file.read_text(), "fixture.c")
            lexical = {(r["path"], r["name"], r["line"], r["column"]): r["owner"] for r in fields}
            result = inventory_ast.parse_one((file, ["-std=c99"], "configured"), root, {file, header}, lexical)
            self.assertEqual(result["errors"], [])
            self.assertEqual(result["seen"], ["fixture.c", "fixture.h"])
            rows = {row["name"]: row for row in result["declarations"]}
            self.assertEqual(rows["held"]["storage_duration"], "static")
            self.assertEqual(rows["local"]["storage_duration"], "automatic")
            self.assertEqual(rows["arg"]["function_linkage"], "INTERNAL")
            self.assertEqual(rows["external"]["function_linkage"], "EXTERNAL")
            self.assertEqual(rows["count"]["address_taken"], "observed-direct")
            self.assertEqual(rows["local"]["address_taken"], "not-observed-in-recorded-TUs")
            bits = {inventory.row_id(row): (row["owner"], row["stored_bits"])
                    for row in result["declarations"] if row["name"] == "bits"}
            self.assertEqual(sorted(bits.values()), [("value", 64), ("value@2", 32)])

    def test_collector_binds_includes_and_rejects_mid_parse_source_changes(self):
        self.require_libclang()
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory).resolve()
            (root / "app").mkdir()
            (root / "testing/compliance").mkdir(parents=True)
            header = root / "app/a.h"
            header.write_text("struct retained { unsigned field; };\n")
            source = root / "app/a.c"
            source.write_text('#include "a.h"\nstatic struct retained saved;\n')
            (root / "testing/compliance/inventory_scope.json").write_text(json.dumps({
                "include_globs": ["app/*.[ch]"], "exclude_globs": []}))
            database = root / "commands.json"
            database.write_text(json.dumps([{"directory": str(root), "file": str(source),
                "arguments": ["clang", "-std=c99", "-c", str(source)]}]))
            report = inventory_ast.collect(root, [database], ["configured"], 1)
            bindings = report["files"]["app/a.c"]["input_bindings"]
            self.assertEqual(set(bindings), {"app/a.c", "app/a.h"})
            self.assertEqual(bindings["app/a.h"]["semantic_sha256"],
                             inventory.semantic_digest(header.read_text()))
            self.assertTrue(all("arguments" in receipt for receipt in report["translation_units"]))
            original = inventory_ast.parse_one

            def mutate_after_parse(*args):
                result = original(*args)
                header.write_text(header.read_text() + "/* moved source locations */\nint added;\n")
                return result

            with mock.patch.object(inventory_ast, "parse_one", side_effect=mutate_after_parse):
                with self.assertRaisesRegex(ValueError, "Compiler input changed during collection"):
                    inventory_ast.collect(root, [database], ["configured"], 1)

    def test_width_variants_cannot_claim_minimum_bit_compliance(self):
        rule = {"id": "reviewed", "status": "compare", "domain": {"min": 0, "max": 7}}
        row = {"stored_bits": 3, "method": "libclang-ast", "profile_width_variants": [3, 32]}
        self.assertEqual(inventory.classify(row, {"rules": [rule]})["status"], "unresolved")

    def test_automatic_policy_does_not_waive_static_local_or_recovery_ast(self):
        policies = inventory.read_json(HERE / "inventory_policies.json")
        row = {"path": "app_program/example.c", "owner": "helper#L3", "name": "value",
               "kind": "VAR_DECL", "type": "unsigned int", "type_category": "UINT",
               "stored_bits": 32, "storage_duration": "automatic", "method": "libclang-ast"}
        self.assertEqual(inventory.classify(row, policies)["policy"], "automatic-execution-temporary")
        self.assertEqual(inventory.classify(dict(row, storage_duration="static"), policies)["status"], "unresolved")
        self.assertEqual(inventory.classify(dict(row, method="libclang-recovery"), policies)["status"], "unresolved")

    def test_generic_rtos_integer_is_not_retained_abi_waiver(self):
        policies = inventory.read_json(HERE / "inventory_policies.json")
        row = {"path": "app_program/example.c", "owner": "state_t", "name": "active",
               "kind": "FIELD_DECL", "type": "BOOL", "type_category": "UINT",
               "stored_bits": 32, "storage_duration": "enclosing-object", "method": "libclang-ast"}
        self.assertEqual(inventory.classify(row, policies)["status"], "unresolved")

    def test_capture_valid_range_does_not_erase_invalid_metadata(self):
        policies = inventory.read_json(HERE / "inventory_policies.json")
        row = {"path": "app_program/controller/bc_capture.h", "owner": "bcap_episode_t",
               "name": "start_ms", "kind": "FIELD_DECL", "stored_bits": 32, "method": "libclang-ast"}
        result = inventory.classify(row, policies)
        self.assertEqual(result["status"], "boundary")
        self.assertEqual(result["domain"]["max"], 4294967295)
        self.assertEqual(result["domain"]["valid_max"], 4294967)
        self.assertEqual(result["required_bits"], 32)

    def test_capture_quiet_includes_collecting_pool_exhaustion(self):
        policies = inventory.read_json(HERE / "inventory_policies.json")
        row = {"path": "app_program/controller/bc_capture.h", "owner": "bcap_t", "name": "quiet",
               "kind": "FIELD_DECL", "stored_bits": 16, "method": "libclang-ast"}
        result = inventory.classify(row, policies)
        self.assertEqual(result["domain"]["max"], 4000)
        self.assertEqual(result["required_bits"], 12)
        self.assertEqual(result["status"], "gap")

    def test_edge_run_storage_includes_transition_before_validation(self):
        policies = inventory.read_json(HERE / "inventory_policies.json")
        row = {"path": "app_program/subsystem3/bc_decoder.h", "owner": "s3_barcode_decoder_t",
               "name": "runs", "kind": "FIELD_DECL", "stored_bits": 32, "method": "libclang-ast"}
        result = inventory.classify(row, policies)
        self.assertEqual(result["domain"]["max"], 2 * 214748364)
        self.assertEqual(result["required_bits"], 29)

    def test_noise_domain_has_inductive_bound_and_ten_bit_counterexample(self):
        policies = inventory.read_json(HERE / "inventory_policies.json")
        row = {"path": "app_program/subsystem3/bc_decoder.h", "owner": "s3_barcode_record_t",
               "name": "noise_q4", "kind": "FIELD_DECL", "stored_bits": 16, "method": "libclang-ast"}
        result = inventory.classify(row, policies)
        limit = result["domain"]["max"]
        self.assertTrue(all(n - n // 16 + 75 <= limit for n in range(limit + 1)))
        paper = previous = 951
        noise = 0
        sequence = [1000 if index % 2 == 0 else 951 for index in range(71)]
        sequence += list(range(925, 24, -75))
        for dark in sequence:
            paper = min(paper, dark) if dark < paper else paper + ((dark - paper) >> 5)
            self.assertLess(dark, paper + 50)
            self.assertLessEqual(previous, paper + 75)
            noise = noise - noise // 16 + abs(dark - previous)
            previous = dark
        self.assertEqual(noise, 1024)
        self.assertEqual(result["required_bits"], 11)

    def test_retained_public_view_needs_instance_review(self):
        policies = inventory.read_json(HERE / "inventory_policies.json")
        row = {"path": "app_program/subsystem2/motion.c", "owner": "", "name": "g_cal_result",
               "kind": "VAR_DECL", "type_category": "RECORD", "storage_duration": "static",
               "method": "libclang-ast", "canonical_type": "motion_calibration_t"}
        self.assertEqual(inventory.classify(row, policies)["status"], "unresolved")

    def test_inactive_branches_are_discovered(self):
        source = """
        typedef struct {
        #if APP_DEBUG
            bool debug_ready;
        #else
            unsigned normal_count : 3;
        #endif
        } sample_t;
        """
        fields, _ = inventory.lexical_fields(source, "app/a.h")
        self.assertEqual({f["name"] for f in fields}, {"debug_ready", "normal_count"})
        self.assertEqual({f["owner"] for f in fields}, {"sample_t"})

    def test_multiple_declarators_arrays_and_callback(self):
        fields, _ = inventory.lexical_fields("""
            typedef struct {
                uint32_t first, second;
                uint8_t samples[8];
                bool (*read)(void *context, unsigned channel);
                unsigned head : 3, count : 4;
            } sample_t;
        """, "app/a.h")
        self.assertEqual([f["name"] for f in fields],
                         ["first", "second", "samples", "read", "head", "count"])
        self.assertEqual(fields[-1]["stored_bits"], 4)

    def test_comment_changes_do_not_invalidate_semantic_binding(self):
        self.assertEqual(inventory.semantic_digest("int x; /* explanation */"),
                         inventory.semantic_digest(" int  x ; // revised explanation\n"))
        self.assertNotEqual(inventory.semantic_digest('char *p = "//x";'),
                            inventory.semantic_digest('char *p = "//y";'))

    def test_terminal_and_signed_domain_widths(self):
        self.assertEqual(inventory.minimum_bits(0, 8), 4)
        self.assertEqual(inventory.minimum_bits(0, 128), 8)
        self.assertEqual(inventory.minimum_bits(-64, 65), 8)
        with self.assertRaises(ValueError):
            inventory.minimum_bits(5, 1)

    def test_reviewed_discrete_domain_requires_explicit_lossless_step(self):
        rule = {"id": "distance", "status": "compare", "domain": {"min": 0, "max": 4075, "step": 25}}
        result = inventory.classify({"stored_bits": 16, "method": "libclang-ast"}, {"rules": [rule]})
        self.assertEqual(result["required_bits"], 8)
        self.assertEqual(result["status"], "gap")
        rule["domain"]["step"] = 26
        with self.assertRaises(ValueError):
            inventory.classify({}, {"rules": [rule]})

    def test_safe_envelope_does_not_claim_a_proved_minimum(self):
        rule = {"id": "envelope", "status": "compare", "domain": {
            "min": -16124062, "max": 16124062, "minimum_width_proved": False}}
        result = inventory.classify({"stored_bits": 32, "method": "libclang-ast"}, {"rules": [rule]})
        self.assertIsNone(result["required_bits"])
        self.assertEqual(result["safe_upper_bound_bits"], 25)
        self.assertEqual(result["status"], "unresolved")

    def test_record_threshold_envelopes_remain_open_until_minimum_proof(self):
        policies = inventory.read_json(HERE / "inventory_policies.json")
        for owner, name, safe_bits in [
                ("s3_barcode_record_t", "required_us", 31),
                ("s3_barcode_record_t", "quiet_required_us", 31)]:
            row = {"path": "app_program/subsystem3/bc_decoder.h", "owner": owner,
                   "name": name, "kind": "FIELD_DECL", "stored_bits": safe_bits,
                   "method": "libclang-ast"}
            result = inventory.classify(row, policies)
            self.assertEqual(result["status"], "unresolved")
            self.assertIsNone(result["required_bits"])
            self.assertEqual(result["safe_upper_bound_bits"], safe_bits)

    def test_mutable_workspace_ranges_include_existing_adversarial_traces(self):
        policies = inventory.read_json(HERE / "inventory_policies.json")
        cases = [("phase", 255, 8, 18), ("cursor", 65535, 16, 4096),
                 ("ext_us", 4294967295, 32, 128992500),
                 ("bound", 2147483647, 32, 16124062),
                 ("chosen", 2147483647, 32, 16124062)]
        for name, maximum, width, initialized_max in cases:
            row = {"path": "app_program/subsystem3/bc_decoder.h", "owner": "s3_barcode_work_t",
                   "name": name, "kind": "FIELD_DECL", "stored_bits": width,
                   "method": "libclang-ast"}
            result = inventory.classify(row, policies)
            self.assertEqual(result["status"], "boundary")
            self.assertEqual(result["domain"]["max"], maximum)
            self.assertEqual(result["required_bits"], width)
            self.assertEqual(result["domain"]["initialized_domain"]["max"], initialized_max)
            self.assertNotIn("step", result["domain"])
            self.assertTrue(any("barcode_work_test.c:" in source for source in result["evidence"]))

    def test_public_workspace_boundaries_do_not_certify_retained_instances(self):
        policies = inventory.read_json(HERE / "inventory_policies.json")
        for path, owner, name in [("bc_decoder.h", "s3_barcode_record_t", "work"),
                                  ("line_navigation.h", "s3_navigation_t", "barcode_record")]:
            row = {"path": "app_program/subsystem3/" + path, "owner": owner,
                   "name": name, "kind": "FIELD_DECL", "type_category": "RECORD"}
            result = inventory.classify(row, policies)
            self.assertEqual(result["status"], "unresolved")
            self.assertIsNone(result["required_bits"])

    def test_macro_schema_does_not_claim_wire_width_is_ram_width(self):
        fields, _ = inventory.lexical_fields("""
            #define CONTROL_DEBUG_FIELDS(U, S) \\
                U(ready, 1) \\
                S(speed, 13)
        """, "app/control_debug.h")
        self.assertEqual([(f["name"], f["schema_bits"], f["stored_bits"]) for f in fields],
                         [("ready", 1, 32), ("speed", 13, 32)])


class IntegrityTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        (self.root / "app").mkdir()
        (self.root / "app/a.h").write_text("typedef struct { unsigned head : 3; unsigned count : 4; } queue_t;\n")
        self.scope = {"schema_version": 1, "baseline_sha": "baseline", "include_globs": ["app/*.h"], "exclude_globs": []}
        self.policies = {"rules": []}
        self.ledger = inventory.collect(self.root, self.scope, self.policies)

    def errors(self, ledger=None, complete=False):
        return inventory.validate(self.root, self.scope, self.policies,
                                  self.ledger if ledger is None else ledger, complete)

    def test_explicit_unresolved_is_integrity_valid_but_not_complete(self):
        self.assertEqual(self.errors(), [])
        self.assertTrue(any("OPEN CR1 FIELD" in e for e in self.errors(complete=True)))

    def test_new_untracked_owned_file_fails(self):
        (self.root / "app/new.h").write_text("typedef struct { int state; } added_t;")
        self.assertTrue(any("UNCLASSIFIED FILE: app/new.h" in e for e in self.errors()))

    def test_new_file_outside_owned_globs_fails_repository_gate(self):
        (self.root / "new_driver.c").write_text("static unsigned new_state;")
        self.assertTrue(any("UNCLASSIFIED REPOSITORY FILE: new_driver.c" in e for e in self.errors()))

    def test_imported_content_allows_only_line_ending_normalization(self):
        file = self.root / "upstream.h"
        file.write_bytes(b"int imported_value;\r\n")
        manifest = {"files": {"upstream.h": {"baseline_content_sha256_lf":
            hashlib.sha256(b"int imported_value;\n").hexdigest()}}}
        (self.root / "imports.json").write_text(json.dumps(manifest))
        self.scope["imports_manifest"] = "imports.json"
        self.assertEqual(inventory.repository_errors(self.root, self.scope), [])
        file.write_bytes(b"long imported_value;\n")
        self.assertIn("IMPORTED BOUNDARY CHANGED: upstream.h",
                      inventory.repository_errors(self.root, self.scope))

    def test_mixed_region_preserves_surrounding_imported_body(self):
        file = self.root / "mixed.c"
        source = "void init(void) {\n#if APP_S4\ncall(1);\n#endif\nimported();\n}\n"
        file.write_text(source)
        self.scope["mixed_files"] = {"mixed.c": {
            "owned_regions": [{"kind": "conditional", "directive": "#if APP_S4"}],
            "preserved_body_sha256": hashlib.sha256(
                b"void init(void) {\n<PROJECT_REGION:0>\nimported();\n}\n").hexdigest()}}
        self.assertEqual(inventory.repository_errors(self.root, self.scope), [])
        file.write_text(source.replace("call(1)", "call(2)"))
        self.assertEqual(inventory.repository_errors(self.root, self.scope), [])
        file.write_text(source.replace("imported()", "modified()"))
        self.assertTrue(any("MIXED IMPORTED BODY CHANGED" in error for error in
                            inventory.repository_errors(self.root, self.scope)))

    def test_mixed_multiple_macros_and_nested_conditionals_preserve_lines(self):
        source = "#define IMPORTED 1\n#define OWNED 2\n#if APP\n#if CHILD\nint x;\n#endif\n#endif\nint foreign;\n"
        entry = {"owned_regions": [{"kind": "macro", "name": "OWNED"},
                                   {"kind": "conditional", "directive": "#if APP"}]}
        masked = inventory.mixed_owned_text(source, entry)
        self.assertEqual(len(masked), len(source))
        self.assertEqual(masked.count("\n"), source.count("\n"))
        self.assertNotIn("IMPORTED", masked)
        self.assertNotIn("foreign", masked)
        self.assertIn("#if CHILD\nint x;\n#endif\n#endif", masked)
        self.assertIn("#define OWNED 2", masked)
        self.assertEqual(inventory.mixed_preserved_body(source, entry),
                         "#define IMPORTED 1\n<PROJECT_REGION:0>\n<PROJECT_REGION:1>\nint foreign;\n")

    def test_mixed_missing_duplicate_or_overlapping_selectors_fail(self):
        macro = {"owned_regions": [{"kind": "macro", "name": "OWNED"}]}
        for source in ["#define ELSE 1\n", "#define OWNED 1\n#define OWNED 2\n"]:
            with self.assertRaises(ValueError):
                inventory.mixed_region_spans(source, macro)
        entry = {"owned_regions": [{"kind": "conditional", "directive": "#if APP"},
                                   {"kind": "macro", "name": "OWNED"}]}
        with self.assertRaises(ValueError):
            inventory.mixed_region_spans("#if APP\n#define OWNED 1\n#endif\n", entry)

    def test_mixed_inventory_excludes_imported_declarations(self):
        source = "struct imported { int value; };\n#if APP\nstruct owned { int value; };\n#endif\n"
        (self.root / "mixed.h").write_text(source)
        self.scope["mixed_files"] = {"mixed.h": {"owned_regions": [
            {"kind": "conditional", "directive": "#if APP"}]}}
        rows = inventory.collect(self.root, self.scope, self.policies)["fields"]
        self.assertEqual([r["owner"] for r in rows if r["path"] == "mixed.h"], ["owned"])

    def test_removed_owned_file_fails(self):
        (self.root / "app/a.h").unlink()
        self.assertTrue(any("STALE FILE" in e for e in self.errors()))

    def test_added_field_fails_even_if_file_digest_is_forged(self):
        file = self.root / "app/a.h"
        file.write_text("typedef struct { unsigned head : 3; unsigned count : 4; bool lost; } queue_t;")
        self.ledger["files"]["app/a.h"]["semantic_sha256"] = inventory.semantic_digest(file.read_text())
        self.assertTrue(any("UNCLASSIFIED FIELD" in e and "lost" in e for e in self.errors()))

    def test_removed_classification_fails(self):
        self.ledger["fields"][0].pop("status")
        self.assertTrue(any("UNCLASSIFIED FIELD" in e for e in self.errors()))

    def test_missing_field_row_fails(self):
        self.ledger["fields"].pop()
        self.assertTrue(any("UNCLASSIFIED FIELD" in e for e in self.errors()))

    def test_unknown_field_never_defaults_to_compliant(self):
        self.assertTrue(all(r["status"] == "unresolved" for r in self.ledger["fields"]))
        self.assertTrue(all(r["required_bits"] is None for r in self.ledger["fields"]))

    def test_width_edit_fails_source_binding(self):
        file = self.root / "app/a.h"
        file.write_text(file.read_text().replace("head : 3", "head : 8"))
        self.assertTrue(any("SOURCE CHANGED" in e for e in self.errors()))

    def test_token_preserving_line_shift_invalidates_source_links(self):
        file = self.root / "app/a.h"
        before = file.read_text()
        file.write_text("/* New documentation line. */\n" + before)
        self.assertEqual(inventory.semantic_digest(before), inventory.semantic_digest(file.read_text()))
        self.assertTrue(any("SOURCE LOCATIONS CHANGED" in e for e in self.errors()))

    def test_scope_drift_is_not_silent(self):
        self.scope["include_globs"].append("adapter/*.c")
        self.assertTrue(any("SCOPE CHANGED" in e for e in self.errors()))

    def test_stale_ast_cannot_be_used_to_refresh(self):
        ast = {"declarations": [], "files": {"app/a.h": {"semantic_sha256": "old"}}}
        with self.assertRaisesRegex(ValueError, "Stale compiler"):
            inventory.collect(self.root, self.scope, self.policies, ast)

    def test_ast_adds_global_declarations_that_fallback_does_not_find(self):
        ast = {"declarations": [{"path": "app/a.h", "owner": "", "name": "g_count",
                "kind": "VAR_DECL", "line": 2, "type": "unsigned int",
                "stored_bits": 32, "method": "libclang-ast", "profiles": ["configured"]}],
               "files": {"app/a.h": self.ast_receipt("app/a.h")}}
        ledger = inventory.collect(self.root, self.scope, self.policies, ast)
        self.assertTrue(any(r["name"] == "g_count" and r["status"] == "unresolved" for r in ledger["fields"]))
        ledger["fields"] = [r for r in ledger["fields"] if r["name"] != "g_count"]
        self.assertTrue(any("DECLARATION MANIFEST DIFFERS" in e for e in self.errors(ledger)))

    def ast_receipt(self, path, dependencies=()):
        bindings = {}
        for dependency in (path, *dependencies):
            text = (self.root / dependency).read_text()
            bindings[dependency] = {"semantic_sha256": inventory.semantic_digest(text),
                                    "source_location_sha256": inventory.source_location_digest(text)}
        return {**bindings[path], "input_bindings": bindings, "status": "observed",
                "profiles": ["configured"]}

    def test_changed_header_invalidates_unchanged_consumer_observations(self):
        header = self.root / "app/layout.h"
        header.write_text("struct stored { unsigned value; };\n")
        ast = {"files": {"app/a.h": self.ast_receipt("app/a.h", ("app/layout.h",)),
                          "app/layout.h": self.ast_receipt("app/layout.h")}}
        ledger = inventory.collect(self.root, self.scope, self.policies, ast)
        header.write_text("struct stored { unsigned value : 1; };\n")
        ast["files"]["app/layout.h"] = self.ast_receipt("app/layout.h")
        with self.assertRaisesRegex(ValueError, "Stale compiler dependency for app/a.h: app/layout.h"):
            inventory.collect(self.root, self.scope, self.policies, ast)
        self.assertTrue(any("STALE COMPILER DEPENDENCY: app/a.h" in e for e in self.errors(ledger)))

    def test_missing_dependency_binding_cannot_refresh_compiler_observations(self):
        receipt = self.ast_receipt("app/a.h")
        del receipt["input_bindings"]
        with self.assertRaisesRegex(ValueError, "Missing compiler dependency binding"):
            inventory.collect(self.root, self.scope, self.policies, {"files": {"app/a.h": receipt}})

    def test_unbound_compiler_declaration_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "Missing compiler input binding"):
            inventory.collect(self.root, self.scope, self.policies,
                              {"declarations": [{"path": "app/a.h"}]})

    def test_compaction_retains_classification_and_domains(self):
        compact = inventory.compact_ledger(self.ledger)
        expanded = inventory.expand_ledger(compact, self.policies)
        self.assertEqual(self.errors(expanded), [])
        self.assertEqual(expanded["fields"], self.ledger["fields"])

    def test_compact_dependency_refs_preserve_and_validate_source_bindings(self):
        bound = inventory.collect(self.root, self.scope, self.policies,
                                  {"files": {"app/a.h": self.ast_receipt("app/a.h")}})
        compact = inventory.compact_ledger(bound)
        self.assertEqual(compact["files"]["app/a.h"]["compiler_input_refs"], ["app/a.h"])
        self.assertNotIn("compiler_input_bindings", compact["files"]["app/a.h"])
        self.assertEqual(self.errors(inventory.expand_ledger(compact, self.policies)), [])
        compact["compiler_inputs"].clear()
        errors = self.errors(inventory.expand_ledger(compact, self.policies))
        self.assertTrue(any("STALE COMPILER DEPENDENCY" in error for error in errors))

    def test_complete_csv_contains_every_row_and_explicit_unknowns(self):
        rows = list(csv.DictReader(io.StringIO(inventory.render_csv(self.ledger))))
        self.assertEqual(len(rows), len(self.ledger["fields"]))
        self.assertTrue(all(row["units"] == "unresolved" for row in rows))
        self.assertTrue(all("unreviewed" in row["synchronization"] for row in rows))
        self.assertIn("Required/stored bits", inventory.render_retained(self.ledger))

    def test_generated_table_drift_fails_even_when_ledger_is_unchanged(self):
        for name, content in inventory.generated_artifacts(self.ledger).items():
            (self.root / name).write_text(content, encoding="utf-8")
        self.assertEqual(inventory.artifact_errors(self.root, self.ledger), [])
        (self.root / "CR1_FIELDS.csv").write_text("stale")
        self.assertEqual(inventory.artifact_errors(self.root, self.ledger),
                         ["GENERATED DOCUMENT STALE: regenerate CR1_FIELDS.csv"])


if __name__ == "__main__":
    unittest.main()
