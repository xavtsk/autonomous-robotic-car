"""Regression tests for the explicit inventory and its C-source bindings."""
from pathlib import Path
import re
import tempfile
import unittest

from testing.change_requirement_1 import check_widths as widths


class WidthCheckTests(unittest.TestCase):
    def test_integer_minimum_width_boundaries(self):
        for lo, hi, bits in ((0, 1, 1), (0, 7, 3), (0, 8, 4),
                             (-40, 40, 7), (0, 4095, 12), (0, 4096, 13)):
            self.assertEqual(widths.min_bits(lo, hi, 1)[0], bits)
        with self.assertRaises(ValueError):
            widths.min_bits(1, 0, 1)

    def test_current_selected_source_bindings(self):
        self.assertEqual(widths.source_errors(), [])

    def changed_sources(self, filename, change):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            files = {value[0] for value in widths.STATE_SOURCES.values()}
            files.add("app_program/subsystem4/terr_data.c")
            for name in files:
                source = (widths.ROOT / name).read_text(encoding="utf-8")
                if name == filename:
                    altered = change(source)
                    self.assertNotEqual(source, altered)
                    source = altered
                target = root / name
                target.parent.mkdir(parents=True, exist_ok=True)
                target.write_text(source, encoding="utf-8")
            return widths.source_errors(root)

    def test_remote_size_mismatch_is_detected(self):
        errors = self.changed_sources(
            "app_program/controller/remote_control.h",
            lambda text: re.sub(r"(REMOTE_REPLY_SIZE\s+)10U", r"\g<1>12U", text))
        self.assertTrue(any("SIZE" in error for error in errors))

    def test_queue_width_drift_is_detected(self):
        errors = self.changed_sources(
            "app_program/subsystem4/buddy4.c",
            lambda text: re.sub(r"\bhead\s*:\s*3", "head : 4", text, count=1))
        self.assertTrue(any("head" in error for error in errors))

    def test_omitting_terrain_safety_is_detected(self):
        errors = self.changed_sources(
            "app_program/subsystem4/terr_data.c",
            lambda text: re.sub(r"\bput_bits\s*\([^;]*?->safety\s*,\s*2\s*\);", "", text))
        self.assertTrue(any("terr_data_pack" in error for error in errors))


if __name__ == "__main__":
    unittest.main()
