"""Integrity regressions for source/field coverage; no C binary is executed."""
import copy
import importlib.util
import hashlib
import json
from pathlib import Path
import tempfile
import unittest

HERE = Path(__file__).resolve().parent
SPEC = importlib.util.spec_from_file_location("cr1_inventory", HERE / "inventory.py")
inventory = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(inventory)


class DiscoveryTests(unittest.TestCase):
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
            "project_region_start": "#if APP_S4", "project_region_end": "#endif",
            "preserved_body_sha256": hashlib.sha256(
                b"void init(void) {\n<PROJECT_I2C_REGION>\nimported();\n}\n").hexdigest()}}
        self.assertEqual(inventory.repository_errors(self.root, self.scope), [])
        file.write_text(source.replace("call(1)", "call(2)"))
        self.assertEqual(inventory.repository_errors(self.root, self.scope), [])
        file.write_text(source.replace("imported()", "modified()"))
        self.assertTrue(any("MIXED IMPORTED BODY CHANGED" in error for error in
                            inventory.repository_errors(self.root, self.scope)))

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
                "stored_bits": 32, "method": "libclang-ast", "profiles": ["configured"]}]}
        ledger = inventory.collect(self.root, self.scope, self.policies, ast)
        self.assertTrue(any(r["name"] == "g_count" and r["status"] == "unresolved" for r in ledger["fields"]))
        ledger["fields"] = [r for r in ledger["fields"] if r["name"] != "g_count"]
        self.assertTrue(any("DECLARATION MANIFEST DIFFERS" in e for e in self.errors(ledger)))

    def test_compaction_retains_classification_and_domains(self):
        compact = inventory.compact_ledger(self.ledger)
        expanded = inventory.expand_ledger(compact, self.policies)
        self.assertEqual(self.errors(expanded), [])
        self.assertEqual(expanded["fields"], self.ledger["fields"])


if __name__ == "__main__":
    unittest.main()
