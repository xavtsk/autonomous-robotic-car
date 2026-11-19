"""Regression tests for audit trust boundaries, not production behavior tests."""
import hashlib
import json
from pathlib import Path
import tempfile
import unittest

from tools.compliance import barr_check as barr


class LexicalTests(unittest.TestCase):
    def inspect(self, source, suffix=".c"):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            path = root / ("sample" + suffix)
            path.write_text(source, encoding="utf-8", newline="")
            return barr.lexical_checks(path, root)

    def test_keywords_in_literals_and_comments_are_not_code(self):
        findings = self.inspect('/** @file sample.c */\n'
                                'const char * p_text = "short long auto register";\n'
                                '/* A long explanation. */\n'
                                '/* End of file. */\n\n')
        self.assertFalse(any(row.rule in {"1.7.a", "1.7.b", "5.2.b"} for row in findings))

    def test_keyword_suffix_is_not_forbidden_keyword(self):
        findings = self.inspect('uint32_t count = 1UL;\nlong value;\n')
        self.assertEqual([row.line for row in findings if row.rule == "5.2.b"], [2])

    def test_commented_include_does_not_satisfy_own_header(self):
        findings = self.inspect('/* #include "sample.h" */\n')
        self.assertTrue(any(row.rule == "4.3.c" for row in findings))

    def test_equivalent_relative_own_header_include_is_accepted(self):
        findings = self.inspect('#include "./sample.h"\n')
        self.assertFalse(any(row.rule == "4.3.c" for row in findings))

    def test_file_tag_inside_string_is_not_documentation(self):
        findings = self.inspect('const char * p_text = "/** @file sample.c */";\n')
        self.assertTrue(any(row.rule == "2.2.h" for row in findings))

    def test_eof_marker_before_code_does_not_count(self):
        findings = self.inspect('/* End of file. */\nint count;\n/* Later. */\n\n')
        self.assertTrue(any(row.rule == "3.3.c" for row in findings))

    def test_backslash_and_url_in_comment_are_reported_literally(self):
        findings = self.inspect('/** @file sample.c\n * See https://example.invalid.\n */\n')
        self.assertTrue(any(row.rule == "2.1.b" for row in findings))

    def test_mixed_region_scan_does_not_require_imported_module_structure(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            path = root / "imported.c"
            path.write_text("long imported_value;\n#if OWNED\nshort owned_value;\n#endif\n")
            masked = "                    \n#if OWNED\nshort owned_value;\n#endif\n"
            findings = barr.lexical_checks(path, root, masked, file_level=False)
            self.assertEqual([(row.rule, row.line) for row in findings], [("5.2.b", 3)])


class AstTests(unittest.TestCase):
    def inspect(self, node):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            path = (root / "sample.c").resolve()
            tree = {"kind": "TranslationUnitDecl", "inner": [node]}
            return barr.ast_checks(tree, path, root, {path})

    def test_float_equality_uses_types_not_literal_spelling(self):
        node = {"kind": "BinaryOperator", "opcode": "==", "inner": [
            {"kind": "DeclRefExpr", "type": {"qualType": "float32_t", "desugaredQualType": "float"}},
            {"kind": "ImplicitCastExpr", "type": {"qualType": "float"}}]}
        self.assertEqual([row.rule for row in self.inspect(node)[0]], ["5.4.b.iv"])
        node["inner"] = [{"kind": "IntegerLiteral", "type": {"qualType": "int"}}]
        self.assertEqual(self.inspect(node)[0], [])

    def test_private_function_documentation_is_required(self):
        node = {"kind": "FunctionDecl", "name": "helper", "storageClass": "static",
                "loc": {"line": 8}, "inner": [{"kind": "CompoundStmt", "range": {"end": {"line": 12}}}]}
        findings, functions, documented, _ = self.inspect(node)
        self.assertEqual([row.rule for row in findings], ["2.2.h"])
        self.assertEqual(len(functions), 1)
        self.assertEqual(documented, [])
        node["inner"].append({"kind": "FullComment"})
        self.assertEqual(len(self.inspect(node)[2]), 1)

    def test_public_prefix_and_short_field_names(self):
        node = {"kind": "FunctionDecl", "name": "other_read", "inner": [
            {"kind": "ParmVarDecl", "name": "x", "type": {"qualType": "int"}}]}
        self.assertEqual({row.rule for row in self.inspect(node)[0]}, {"6.1.i", "7.1.e"})

    def test_ambiguous_include_location_is_not_claimed_covered(self):
        node = {"kind": "FunctionDecl", "name": "legacy", "loc": {"includedFrom": {"file": "sample.c"}},
                "inner": [{"kind": "CompoundStmt"}]}
        findings, functions, _, unknown = self.inspect(node)
        self.assertEqual(findings, [])
        self.assertEqual(functions, [])
        self.assertEqual(unknown, 1)


class EvidenceTests(unittest.TestCase):
    def test_ledger_has_all_explicit_subclauses(self):
        ledger = json.loads(barr.LEDGER.read_text(encoding="utf-8"))
        self.assertEqual(barr.validate_ledger(ledger), [])
        sections = {
            "1.1": "abcd", "1.2": "a", "1.3": "ab", "1.4": "ab", "1.5": "ab", "1.6": "a", "1.7": "abcd", "1.8": "abc",
            "2.1": "abc", "2.2": "abcdefghi", "3.1": "abcdefghijklm", "3.2": "abcd", "3.3": "abc", "3.4": "abc", "3.5": "a", "3.6": "ab",
            "4.1": "abcd", "4.2": "abcd", "4.3": "abcdef", "4.4": "a", "5.1": "abc", "5.2": "abc", "5.3": "abc", "5.4": "ab", "5.5": "ab", "5.6": "ab",
            "6.1": "abcdefghi", "6.2": "abcdef", "6.3": "ab", "6.4": "a", "6.5": "abcd", "7.1": "abcdefghijklmno", "7.2": "abcd",
            "8.1": "a", "8.2": "abcd", "8.3": "abc", "8.4": "abcd", "8.5": "ab", "8.6": "a"}
        expected = {f"{section}.{letter}" for section, letters in sections.items() for letter in letters}
        for section, count in {"1.8.b": 4, "1.8.c": 4, "2.1.c": 2, "2.2.i": 3, "4.2.c": 2, "5.4.b": 5, "6.3.b": 4}.items():
            expected.update(section + "." + roman for roman in ("i", "ii", "iii", "iv", "v")[:count])
        self.assertEqual({row["id"] for row in ledger["rules"]}, expected)
        example = next(row for row in ledger["rules"] if row["id"] == "1.8.c.ii")
        self.assertEqual(example["strength"], "example")

    def test_scope_includes_untracked_files_and_applies_exclusions(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "owned").mkdir()
            (root / "owned/new.c").write_text("")
            (root / "owned/generated.h").write_text("")
            scope = {"schema_version": 1, "include_globs": ["owned/**/*.c", "owned/**/*.h"], "exclude_globs": ["owned/generated.h"]}
            self.assertEqual([path.name for path in barr.owned_files(root, scope)], ["new.c"])

    def test_approved_deviation_needs_named_approval_and_evidence(self):
        ledger = {"rules": [{"id": "1.1.a", "strength": "mandatory", "status": "approved_deviation", "evidence": []}]}
        errors = barr.validate_ledger(ledger)
        self.assertTrue(any("no evidence" in error for error in errors))
        self.assertTrue(any("approver" in error for error in errors))

    def test_removing_one_rule_does_not_pass_ledger_validation(self):
        ledger = json.loads(barr.LEDGER.read_text(encoding="utf-8"))
        ledger["rules"] = [row for row in ledger["rules"] if row["id"] != "2.2.h"]
        self.assertTrue(any("missing ['2.2.h']" in error for error in barr.validate_ledger(ledger)))

    def test_stale_ast_hash_is_rejected_before_findings_can_pass(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "sample.c").write_text("int value;\n")
            manifest = root / "asts.json"
            manifest.write_text(json.dumps({"schema_version": 1, "source_sha256": {"sample.c": hashlib.sha256(b"old").hexdigest()}}))
            with self.assertRaisesRegex(ValueError, "stale"):
                barr.audit(root, {"schema_version": 1, "include_globs": ["*.c"]}, {"rules": []}, manifest)


if __name__ == "__main__":
    unittest.main()
