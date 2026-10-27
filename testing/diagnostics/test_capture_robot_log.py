"""No robot, serial dependency, port enumeration or network used by these tests."""

import contextlib
import csv
import importlib.util
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest import mock

ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location("capture_robot_log", ROOT / "tools/capture_robot_log.py")
capture_log = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(capture_log)


class Stream:
    def __init__(self, chunks):
        self.chunks = iter(chunks)
        self.requested = []

    def read(self, size):
        self.requested.append(size)
        value = next(self.chunks, b"")
        if isinstance(value, BaseException):
            raise value
        return value


class Clock:
    def __init__(self):
        self.value = 0.0

    def __call__(self):
        self.value += 0.1
        return self.value


class CaptureTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.directory = Path(self.temp.name) / "recording"

    def run_capture(self, chunks, **kwargs):
        stream = Stream(chunks)
        result = capture_log.capture(stream, self.directory, source="replay",
                                     source_name="fixture", clock=Clock(),
                                     utc=lambda: "2026-10-24T00:00:00+00:00", **kwargs)
        self.assertTrue(all(size == capture_log.READ_BYTES for size in stream.requested))
        self.assertEqual(result, json.loads((self.directory / "metadata.json").read_text()))
        return result

    def test_exact_binary_and_partial_lines(self):
        data = [b"first\r", b"\nsec\xffond\n", b"partial\x00"]
        result = self.run_capture(data, line_csv=True, firmware="local-dirty", test="fixture", notes="desk")
        self.assertEqual((self.directory / "console.log").read_bytes(), b"".join(data))
        self.assertEqual(result["source"], "replay")
        self.assertEqual(result["evidence_label"], "REPLAY_NOT_HARDWARE")
        self.assertEqual(result["status"], "complete")
        self.assertEqual(result["bytes_received"], sum(map(len, data)))
        self.assertEqual(result["read_chunks"], 3)
        self.assertEqual(result["firmware_identifier_supplied_by_user"], "local-dirty")
        self.assertEqual(result["notes"], "desk")
        with (self.directory / "hostarrival.csv").open(newline="") as file:
            rows = list(csv.DictReader(file))
        self.assertEqual([row["record_kind"] for row in rows], ["line", "line", "partial"])
        self.assertEqual(rows[0]["text"], "first")
        self.assertEqual(rows[1]["text"], "sec\ufffdond")
        self.assertEqual(rows[2]["text"], "partial\x00")
        self.assertIn("not MCU", result["timestamp_note"])

    def test_long_unterminated_input_is_fragmented_and_bounded(self):
        sink = io.StringIO(newline="")
        arrival = capture_log.ArrivalCSV(sink)
        for _ in range(9):
            arrival.feed(b"x" * capture_log.READ_BYTES, "UTC", 1.0)
            self.assertLess(len(arrival.pending), capture_log.READ_BYTES)
        arrival.feed(b"tail\nlast", "UTC", 2.0)
        arrival.finish()
        rows = list(csv.DictReader(io.StringIO(sink.getvalue())))
        self.assertEqual(len(rows), 11)
        self.assertEqual(rows[9]["record_kind"], "fragment_end")
        self.assertEqual(rows[10]["record_kind"], "partial")
        self.assertTrue(all(len(row["text"]) <= capture_log.READ_BYTES for row in rows))

    def test_empty_replay(self):
        result = self.run_capture([])
        self.assertEqual(result["bytes_received"], 0)
        self.assertEqual((self.directory / "console.log").read_bytes(), b"")
        self.assertFalse((self.directory / "hostarrival.csv").exists())

    def test_interruption_keeps_raw_data_and_metadata(self):
        result = self.run_capture([b"last partial", KeyboardInterrupt()], line_csv=True)
        self.assertEqual(result["status"], "interrupted")
        self.assertIsNotNone(result["ended_utc"])
        self.assertGreater(result["elapsed_seconds"], 0)
        self.assertEqual((self.directory / "console.log").read_bytes(), b"last partial")

    def test_read_failure_keeps_raw_data_and_marks_error(self):
        result = self.run_capture([b"before failure", OSError("device disconnected")])
        self.assertEqual(result["status"], "error")
        self.assertIn("device disconnected", result["error"])
        self.assertEqual((self.directory / "console.log").read_bytes(), b"before failure")

    def test_nonbinary_input_marks_error(self):
        result = self.run_capture(["not bytes"])
        self.assertEqual(result["status"], "error")
        self.assertEqual(result["bytes_received"], 0)

    def test_existing_directory_never_overwritten(self):
        self.directory.mkdir()
        sentinel = self.directory / "console.log"
        sentinel.write_bytes(b"old evidence")
        with self.assertRaises(FileExistsError):
            self.run_capture([b"new evidence"])
        self.assertEqual(sentinel.read_bytes(), b"old evidence")

    def test_live_timeout_is_not_eof_and_duration_is_monotonic(self):
        stream = Stream([b"", b"still alive\n", b""])
        result = capture_log.capture(stream, self.directory, source="hardware",
                                     source_name="FAKE_NOT_A_REAL_PORT", duration=0.7,
                                     clock=Clock())
        self.assertEqual(result["status"], "complete")
        self.assertEqual((self.directory / "console.log").read_bytes(), b"still alive\n")
        self.assertGreater(len(stream.requested), 2)

    def test_invalid_duration_and_source_do_not_create_recording(self):
        for duration in (0, -1, float("inf"), float("nan")):
            with self.assertRaises(ValueError):
                capture_log.capture(Stream([]), self.directory, source="hardware", source_name="fake",
                                    duration=duration)
        with self.assertRaises(ValueError):
            capture_log.capture(Stream([]), self.directory, source="fake", source_name="fake")
        self.assertFalse(self.directory.exists())

    def test_replay_cli_never_imports_serial_or_opens_a_port(self):
        fixture = Path(self.temp.name) / "fixture.bin"
        fixture.write_bytes(b"REPLAY\r\n\xffpartial")
        with mock.patch.object(capture_log, "serial_module", side_effect=AssertionError("must not import")):
            with contextlib.redirect_stdout(io.StringIO()) as output:
                code = capture_log.main(["--replay", str(fixture), "--output", str(self.directory),
                                         "--line-csv", "--test", "offline"])
        self.assertEqual(code, 0)
        self.assertIn("NOT HARDWARE", output.getvalue())
        self.assertEqual((self.directory / "console.log").read_bytes(), fixture.read_bytes())

    def test_missing_replay_does_not_create_recording(self):
        with contextlib.redirect_stderr(io.StringIO()):
            code = capture_log.main(["--replay", str(Path(self.temp.name) / "missing"),
                                     "--output", str(self.directory)])
        self.assertEqual(code, 1)
        self.assertFalse(self.directory.exists())

    def test_bad_cli_arguments_refused_before_serial_access(self):
        bad = [[], ["--port", "FAKE"], ["--replay", "x", "--output", str(self.directory), "--duration", "1"],
               ["--port", "FAKE", "--output", str(self.directory), "--duration", "nan"],
               ["--port", "FAKE", "--output", str(self.directory), "--baud", "0"]]
        self.directory.mkdir()
        bad.append(["--port", "FAKE", "--output", str(self.directory)])
        for args in bad:
            with self.subTest(args=args), mock.patch.object(capture_log, "open_serial", side_effect=AssertionError):
                with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as error:
                    capture_log.main(args)
                self.assertEqual(error.exception.code, 2)

    def test_serial_open_sets_control_lines_before_open_and_never_writes(self):
        class Device:
            def __init__(self, **kwargs):
                self.arguments = kwargs
                self.is_open = False
                self.closed = False

            def open(self):
                self.assertions = (self.dtr is False, self.rts is False, self.port == "FAKE")
                self.is_open = True

            def close(self):
                self.closed = True

            def write(self, data):
                raise AssertionError("logger must not write to the port")

        fake = mock.Mock()
        fake.Serial.side_effect = Device
        with mock.patch.object(capture_log, "serial_module", return_value=fake):
            device = capture_log.open_serial("FAKE", 115200)
        self.assertTrue(all(device.assertions))
        self.assertIsNone(device.arguments["port"])
        self.assertEqual(device.arguments["timeout"], 0.25)
        device.close()

    def test_serial_open_failure_closes_device(self):
        device = mock.Mock()
        device.open.side_effect = OSError("port unavailable")
        fake = mock.Mock()
        fake.Serial.return_value = device
        with mock.patch.object(capture_log, "serial_module", return_value=fake):
            with self.assertRaises(OSError):
                capture_log.open_serial("FAKE", 115200)
        device.close.assert_called_once()
        device.write.assert_not_called()

    def test_live_main_closes_device_and_records_baud(self):
        device = io.BytesIO(b"")
        def open_after_reservation(port, baud):
            self.assertTrue((self.directory / "console.log").is_file())
            self.assertTrue((self.directory / "hostarrival.csv").is_file())
            self.assertEqual(json.loads((self.directory / "metadata.json").read_text())["status"], "running")
            return device
        with mock.patch.object(capture_log, "open_serial", side_effect=open_after_reservation) as opened:
            with contextlib.redirect_stdout(io.StringIO()):
                code = capture_log.main(["--port", "FAKE", "--output", str(self.directory),
                                         "--duration", "0.001", "--baud", "115200", "--line-csv"])
        self.assertEqual(code, 0)
        opened.assert_called_once_with("FAKE", 115200)
        metadata = json.loads((self.directory / "metadata.json").read_text())
        self.assertEqual(metadata["requested_baud_rate"], 115200)
        self.assertEqual(metadata["source"], "hardware")
        self.assertEqual(metadata["status"], "complete")
        self.assertTrue(device.closed)

    def test_broken_output_symlink_refused_before_opening_port(self):
        self.directory.symlink_to(Path(self.temp.name) / "missing-target")
        with mock.patch.object(capture_log, "open_serial", side_effect=AssertionError):
            with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                capture_log.main(["--port", "FAKE", "--output", str(self.directory)])
        self.assertTrue(self.directory.is_symlink())

    def test_interrupt_while_opening_is_clean(self):
        with mock.patch.object(capture_log, "open_serial", side_effect=KeyboardInterrupt):
            with contextlib.redirect_stderr(io.StringIO()):
                code = capture_log.main(["--port", "FAKE", "--output", str(self.directory)])
        self.assertEqual(code, 130)
        self.assertEqual(json.loads((self.directory / "metadata.json").read_text())["status"], "interrupted")
        self.assertEqual((self.directory / "console.log").read_bytes(), b"")

    def test_unusable_output_parent_never_opens_serial(self):
        parent = Path(self.temp.name) / "not-a-directory"
        parent.write_bytes(b"existing file")
        with mock.patch.object(capture_log, "open_serial", side_effect=AssertionError) as opened:
            with contextlib.redirect_stderr(io.StringIO()):
                code = capture_log.main(["--port", "FAKE", "--output", str(parent / "recording")])
        self.assertEqual(code, 1)
        opened.assert_not_called()
        self.assertEqual(parent.read_bytes(), b"existing file")

    def test_output_directory_race_never_opens_serial_or_overwrites(self):
        original_parse = capture_log.parse_args
        def parse_then_competing_directory(argv):
            args = original_parse(argv)
            self.directory.mkdir()
            (self.directory / "console.log").write_bytes(b"other recording")
            return args
        with mock.patch.object(capture_log, "parse_args", side_effect=parse_then_competing_directory):
            with mock.patch.object(capture_log, "open_serial", side_effect=AssertionError) as opened:
                with contextlib.redirect_stderr(io.StringIO()):
                    code = capture_log.main(["--port", "FAKE", "--output", str(self.directory)])
        self.assertEqual(code, 1)
        opened.assert_not_called()
        self.assertEqual((self.directory / "console.log").read_bytes(), b"other recording")

    def test_unusable_output_file_never_opens_serial(self):
        original_open = Path.open
        def fail_raw_open(path, *args, **kwargs):
            if path.name == "console.log":
                raise PermissionError("injected raw output failure")
            return original_open(path, *args, **kwargs)
        with mock.patch.object(Path, "open", new=fail_raw_open):
            with mock.patch.object(capture_log, "open_serial", side_effect=AssertionError) as opened:
                with contextlib.redirect_stderr(io.StringIO()):
                    code = capture_log.main(["--port", "FAKE", "--output", str(self.directory)])
        self.assertEqual(code, 1)
        opened.assert_not_called()
        metadata = json.loads((self.directory / "metadata.json").read_text())
        self.assertEqual(metadata["status"], "error")
        self.assertIn("Output preparation failed", metadata["error"])

    def test_serial_open_failure_leaves_error_recording(self):
        with mock.patch.object(capture_log, "open_serial", side_effect=OSError("port unavailable")):
            with contextlib.redirect_stderr(io.StringIO()):
                code = capture_log.main(["--port", "FAKE", "--output", str(self.directory)])
        self.assertEqual(code, 1)
        metadata = json.loads((self.directory / "metadata.json").read_text())
        self.assertEqual(metadata["status"], "error")
        self.assertEqual(metadata["bytes_received"], 0)
        self.assertIn("port unavailable", metadata["error"])
        self.assertEqual((self.directory / "console.log").read_bytes(), b"")

    def test_initial_metadata_failure_never_opens_serial(self):
        with mock.patch.object(capture_log, "atomic_metadata", side_effect=OSError("metadata unavailable")):
            with mock.patch.object(capture_log, "open_serial", side_effect=AssertionError) as opened:
                with contextlib.redirect_stderr(io.StringIO()):
                    code = capture_log.main(["--port", "FAKE", "--output", str(self.directory)])
        self.assertEqual(code, 1)
        opened.assert_not_called()
        self.assertFalse((self.directory / "console.log").exists())

    def test_final_metadata_dump_failure_preserves_valid_running_json(self):
        original_dump = json.dump
        writes = 0
        def fail_second_dump(value, file, **kwargs):
            nonlocal writes
            writes += 1
            if writes == 2:
                file.write('{"partial":')
                raise OSError("injected final metadata write failure")
            return original_dump(value, file, **kwargs)
        with mock.patch.object(capture_log.json, "dump", side_effect=fail_second_dump):
            result = capture_log.capture(Stream([b"raw survives\xff"]), self.directory,
                                         source="replay", source_name="fixture", clock=Clock())
        self.assertEqual(writes, 2)
        self.assertEqual(result["status"], "error")
        self.assertIn("Metadata finalization failed", result["error"])
        stored = json.loads((self.directory / "metadata.json").read_text())
        self.assertEqual(stored["status"], "running")
        self.assertIsNone(stored["ended_utc"])
        self.assertEqual((self.directory / "console.log").read_bytes(), b"raw survives\xff")
        self.assertEqual(list(self.directory.glob(".metadata-*.tmp")), [])

    def test_failed_final_metadata_makes_cli_fail_without_losing_raw(self):
        fixture = Path(self.temp.name) / "fixture.bin"
        fixture.write_bytes(b"raw log retained")
        original_dump = json.dump
        writes = 0
        def fail_second_dump(value, file, **kwargs):
            nonlocal writes
            writes += 1
            if writes == 2:
                raise OSError("injected final JSON failure")
            return original_dump(value, file, **kwargs)
        with mock.patch.object(capture_log.json, "dump", side_effect=fail_second_dump):
            with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
                code = capture_log.main(["--replay", str(fixture), "--output", str(self.directory)])
        self.assertEqual(code, 1)
        self.assertEqual(json.loads((self.directory / "metadata.json").read_text())["status"], "running")
        self.assertEqual((self.directory / "console.log").read_bytes(), fixture.read_bytes())

    def test_final_metadata_replace_failure_preserves_valid_json(self):
        original_replace = Path.replace
        replacements = 0
        def fail_second_replace(path, target):
            nonlocal replacements
            replacements += 1
            if replacements == 2:
                raise OSError("injected replace failure")
            return original_replace(path, target)
        with mock.patch.object(Path, "replace", new=fail_second_replace):
            result = capture_log.capture(Stream([b"raw data"]), self.directory,
                                         source="replay", source_name="fixture", clock=Clock())
        self.assertEqual(result["status"], "error")
        self.assertEqual(json.loads((self.directory / "metadata.json").read_text())["status"], "running")
        self.assertEqual((self.directory / "console.log").read_bytes(), b"raw data")
        self.assertEqual(list(self.directory.glob(".metadata-*.tmp")), [])


if __name__ == "__main__":
    unittest.main()
