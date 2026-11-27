"""Regression tests for audit trust boundaries, not production behavior tests."""
import hashlib
import json
from pathlib import Path
import tempfile
import unittest

from tools.compliance import barr_check as barr


class LexicalTests(unittest.TestCase):
    def test_macro_transfers_use_logical_lines_and_skip_literals(self):
        source = ('#define TEXT(x) print("return", (x)) /* goto */\n'
                  '#define OBJECT (arg)\n'
                  '/* #define HIDDEN(x) break */\n'
                  '%:define CHECK(x) if (x) ret??/\nurn (x)\n'
                  '#define EXITING(x) longjmp((x), 1)\n')
        self.assertEqual(list(barr.macro_control_transfers(source)),
                         [(4, 'CHECK', 'return'), (6, 'EXITING', 'longjmp')])

    def test_horizontal_padding_excludes_literals_comments_and_close_indent(self):
        source = ('emit( "( ; )" ); /* ( ; ) */\n'
                  'emit(\n    "( ; )"\n    );\n'
                  'emit("text") ;\n')
        findings = [r.rule for r in self.inspect(source) if r.rule in {"3.1.i", "3.1.m"}]
        self.assertCountEqual(findings, ["3.1.i", "3.1.i", "3.1.m"])

    def test_grouped_pointer_declarators_keep_expression_checks_active(self):
        source = ('void ( * p_callback)(int count);\n'
                  'void ( * p_other)( int count );\n'
                  'emit( *p_pointer );\n')
        findings = [row.line for row in self.inspect(source) if row.rule == "3.1.i"]
        self.assertCountEqual(findings, [2, 2, 3, 3])

    def test_nonprinting_unicode_and_permitted_form_feed(self):
        text = "/* ±1 degree */\n/* \u0085 */\n/* \u200b */\n/* \f */\n"
        findings = [row.line for row in self.inspect(text) if row.rule == "3.6.b"]
        self.assertEqual(findings, [2, 3])

    def inspect(self, source, suffix=".c"):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            path = root / ("sample" + suffix)
            path.write_text(source, encoding="utf-8", newline="")
            return barr.lexical_checks(path, root)

    def test_header_guard_encloses_the_module(self):
        body = "#ifndef SAMPLE_H\n#define SAMPLE_H\n#if OPTION\nint f(void);\n#endif\n"
        valid = body + "#endif /* SAMPLE_H */\n"
        self.assertEqual(barr.enclosing_header_guard("\ufeff/* notice */\n" + valid),
                         "SAMPLE_H")
        for source in ("int f(void);\n" + valid, valid + "int g(void);\n", body,
                       body + "#else\nint g(void);\n#endif\n",
                       body + "%:un\\\ndef SAMPLE_H\n#endif\n"):
            with self.subTest(source=source):
                self.assertTrue(any(row.rule == "4.2.b"
                                    for row in self.inspect(source, ".h")))

    def test_keywords_in_literals_and_comments_are_not_code(self):
        findings = self.inspect('/** @file sample.c */\n'
                                'const char * p_text = "short long auto register";\n'
                                '/* A long explanation. */\n'
                                '/* End of file. */\n\n')
        self.assertFalse(any(row.rule in {"1.7.a", "1.7.b", "5.2.b"} for row in findings))

    def test_directive_locations_after_blank_lines(self):
        findings = self.inspect('/** @file sample.c */\n\n#define my_macro 1\n'
                                '\n#include "private.c"\n')
        self.assertEqual([(f.rule, f.line) for f in findings
                          if f.rule in {"6.1.f", "4.3.f"}],
                         [("6.1.f", 3), ("4.3.f", 5)])

    def test_keyword_suffix_is_not_forbidden_keyword(self):
        findings = self.inspect('uint32_t count = 1UL;\nlong value;\n')
        self.assertEqual([row.line for row in findings if row.rule == "5.2.b"], [2])

    def test_commented_include_does_not_satisfy_own_header(self):
        findings = self.inspect('/* #include "sample.h" */\n')
        self.assertTrue(any(row.rule == "4.3.c" for row in findings))

    def test_equivalent_relative_own_header_include_is_accepted(self):
        findings = self.inspect('#include "./sample.h"\n')
        self.assertFalse(any(row.rule == "4.3.c" for row in findings))

    def test_inactive_or_search_path_include_needs_review(self):
        for source in ('#if 0\n#include "sample.h"\n#endif\n',
                       '#include <sample.h>\n'):
            with self.subTest(source=source):
                self.assertEqual([r.severity for r in self.inspect(source)
                                  if r.rule == "4.3.c"], ["review"])

    def test_noncolocated_header_is_not_declared_missing(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "arch").mkdir()
            (root / "arch/sample.h").write_text("#ifndef SAMPLE_H\n#define SAMPLE_H\n#endif\n")
            path = root / "sample.c"
            path.write_text('#include "arch/sample.h"\n')
            findings = barr.lexical_checks(path, root)
            pairing = [row for row in findings if row.rule in {"4.2.a", "4.3.c"}]
            self.assertTrue(pairing)
            self.assertTrue(all(row.severity == "review" for row in pairing))

    def test_macro_header_requires_resolution_not_absence_claim(self):
        findings = self.inspect('#define OWN_HEADER "sample.h"\n#include OWN_HEADER\n')
        own_header = [row for row in findings if row.rule == "4.3.c"]
        self.assertEqual([row.severity for row in own_header], ["review"])

    def test_different_root_include_does_not_satisfy_pairing(self):
        findings = self.inspect('#include "other.h"\n')
        self.assertEqual([row.severity for row in findings if row.rule == "4.3.c"],
                         ["violation"])

    def test_file_tag_inside_string_is_not_documentation(self):
        findings = self.inspect('const char * p_text = "/** @file sample.c */";\n')
        self.assertTrue(any(row.rule == "2.2.h" for row in findings))

    def test_eof_marker_before_code_does_not_count(self):
        findings = self.inspect('/* End of file. */\nint count;\n/* Later. */\n\n')
        self.assertTrue(any(row.rule == "3.3.c" for row in findings))

    def test_eof_comment_and_blank_line_are_independent_of_line_endings(self):
        for newline in ("\n", "\r\n", "\r"):
            for marker, endings, valid_eof in (("/* End of file. */", 2, True),
                                               ("/* End of file. */", 1, False),
                                               ("/* Other comment. */", 2, False)):
                with self.subTest(newline=newline, marker=marker, endings=endings):
                    rules = {row.rule for row in self.inspect(marker + newline * endings)}
                    self.assertEqual("3.3.c" in rules, not valid_eof)
                    self.assertEqual("3.6.a" in rules, newline != "\n")

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

    def test_ast_respects_separate_owned_ranges(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            path = (root / "mixed.c").resolve()
            tree = {"kind": "TranslationUnitDecl", "inner": [
                {"kind": "VarDecl", "name": "x", "loc": {"line": number}}
                for number in (2, 4, 6)]}
            findings, _, _, _ = barr.ast_checks(tree, path, root, {path},
                                                {path: [(2, 2), (6, 6)]})
            self.assertEqual([(row.rule, row.line) for row in findings],
                             [("7.1.e", 2), ("7.1.e", 6)])


class AstTests(unittest.TestCase):
    def test_restricted_identifiers_apply_to_declarations_only(self):
        decl = {"kind": "FunctionDecl", "name": "class", "storageClass": "static"}
        self.assertIn("6.1.a", [f.rule for f in self.inspect(decl)[0]])
        for name, rule in (("interrupt", "7.1.a"), ("errno", "7.1.b")):
            self.assertIn(rule, [f.rule for f in self.inspect({"kind": "ParmVarDecl", "name": name})[0]])
        self.assertFalse(self.inspect({"kind": "DeclRefExpr", "name": "stderr"})[0])

    def test_do_body_and_unbraced_else_after_braced_then(self):
        expr, empty, block = ({"kind": name} for name in ("DeclRefExpr", "NullStmt", "CompoundStmt"))
        unbraced_do = {"kind": "DoStmt", "inner": [empty, expr]}
        tail = {"kind": "IfStmt", "hasElse": True, "inner": [expr, block, empty]}
        chain = {"kind": "IfStmt", "hasElse": True, "inner": [expr, block, tail]}
        for statement in (unbraced_do, chain):
            self.assertEqual([r.rule for r in self.inspect(statement)[0]], ["1.3.a"])

    def test_nested_default_does_not_satisfy_outer_switch(self):
        nested = {"kind": "SwitchStmt", "inner": [
            {"kind": "DeclRefExpr"}, {"kind": "CompoundStmt", "inner": [{"kind": "DefaultStmt"}]}]}
        outer = {"kind": "SwitchStmt", "inner": [
            {"kind": "DeclRefExpr"}, {"kind": "CompoundStmt", "inner": [nested]}]}
        self.assertEqual([r.rule for r in self.inspect(outer)[0]], ["8.3.b"])

    def test_assignments_only_in_if_condition(self):
        assigned = {"kind": "BinaryOperator", "opcode": "="}
        compound = {"kind": "CompoundAssignOperator", "opcode": "+="}
        comparison = {"kind": "BinaryOperator", "opcode": "=="}
        for operand, expected in ((assigned, ["8.2.c"]), (compound, ["8.2.c"]), (comparison, [])):
            statement = {"kind": "IfStmt", "inner": [
                {"kind": "ParenExpr", "inner": [operand]}, {"kind": "CompoundStmt", "inner": [assigned]}]}
            self.assertEqual([r.rule for r in self.inspect(statement)[0]], expected)

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

    def test_typedefs_and_local_declaration_groups(self):
        for name, expected in (("wheel_state_t", []), ("T_CMTX", ["5.1.a"]),
                               ("wheel_state", ["5.1.a"])):
            node = {"kind": "TypedefDecl", "name": name}
            self.assertEqual([r.rule for r in self.inspect(node)[0]], expected)
        self.assertEqual(self.inspect(dict(node, isImplicit=True))[0], [])
        variable = {"kind": "VarDecl", "name": "speed"}
        for children, expected in (([{"kind": "RecordDecl"}, variable], []),
                                   ([variable, dict(variable, name="count")], ["8.1.a"])):
            statement = {"kind": "DeclStmt", "inner": children}
            self.assertEqual([r.rule for r in self.inspect(statement)[0]], expected)

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

    def test_split_header_tree_is_not_a_confirmed_module_collision(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "source").mkdir()
            (root / "include").mkdir()
            (root / "source/sample.c").write_text('#include "sample.h"\n')
            (root / "include/sample.h").write_text('#ifndef SAMPLE_H\n#define SAMPLE_H\n#endif\n')
            report = barr.audit(root, {"schema_version": 1,
                                "include_globs": ["**/*.c", "**/*.h"]},
                                {"rules": []})
            collisions = [row for row in report["findings"] if row["rule"] == "4.1.b"]
            self.assertTrue(collisions)
            self.assertTrue(all(row["severity"] == "review" for row in collisions))

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
