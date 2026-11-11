"""Run controls with fake time/MQTT; never connects to the real car."""
from pathlib import Path
import struct
import sys
import tempfile
import threading
import unittest
from unittest.mock import Mock

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools"))
from dashboard.state import DashboardState
from dashboard.traces import TraceStore
from dashboard.control_protocol import TOPIC as DEBUG_TOPIC, encode_control
from dashboard.run_control_protocol import (COMMAND_TOPIC, REPLY_TOPIC, STATUS_TOPIC,
    encode_request, encode_reply, encode_status, decode_reply, decode_status)
from dashboard.transport import NativeMqttTransport


class RunControlTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.now = 100.0
        self.state = DashboardState(TraceStore(Path(self.tmp.name)), clock=lambda: self.now)
        self.state.set_connection("connected")
        self.sent = []
        self.state.transport = Mock()
        self.state.transport.publish.side_effect = lambda t, p: self.sent.append((t, p))
        self.serial = 0

    def preview(self, **changes):
        self.serial += 1
        values = dict(timestamp_us=self.serial*200000, motion_timestamp_us=self.serial*200000,
            sequence=self.serial & 255, motion_available=1, motion_valid=1, braked=1,
            accepted_calibration=7, drive_state=0)
        values.update(changes)
        self.state.ingest(DEBUG_TOPIC, encode_control(**values))

    def ready(self, flags=3, last_id=50, epoch=7, **preview):
        self.preview(**preview)
        self.state.ingest(STATUS_TOPIC, encode_status(epoch, 1230000, last_id, flags))

    def view(self):
        return self.state.snapshot()["control"]

    def reply(self, result="queued", action=None, ident=None, epoch=7, reason="none"):
        req = self.view()["last_request"]
        self.state.ingest(REPLY_TOPIC, encode_reply(ident if ident is not None else req["id"],
            action or req["action"], result, reason, epoch))

    def test_start_requires_fresh_ready_firmware_and_controller(self):
        self.assertFalse(self.view()["can_start"])
        self.assertTrue(self.view()["can_stop"])
        self.preview()
        self.assertFalse(self.view()["can_start"])  # old firmware lacks capability
        self.ready()
        self.assertTrue(self.view()["can_start"])
        for fields in (dict(readiness=1), dict(accepted_calibration=3), dict(collecting=1),
                       dict(motion_valid=0), dict(braked=0), dict(motion_state=1),
                       dict(motion_fault=1), dict(busy=1), dict(drive_state=1)):
            self.preview(**fields)
            self.assertFalse(self.view()["can_start"], fields)
        self.ready(flags=1)
        self.assertFalse(self.view()["can_start"])
        self.ready(); self.now += 1.01
        with self.assertRaises(ValueError): self.state.request_control("start")
        self.preview(); self.now += 1.01; self.preview()
        self.assertFalse(self.view()["can_start"])  # fresh preview, expired capability

    def test_sent_queued_executed_are_separate_from_observed(self):
        self.ready(); self.state.request_control("start")
        self.assertEqual(self.sent, [(COMMAND_TOPIC, encode_request(51, "start", 7, 1230000))])
        self.assertEqual(self.view()["last_request"]["status"], "sent")
        self.assertFalse(self.view()["observed"]["running"])
        self.reply(); self.assertEqual(self.view()["last_request"]["status"], "queued")
        self.reply("executed"); self.assertEqual(self.view()["last_request"]["status"], "executed")
        self.reply(); self.assertEqual(self.view()["last_request"]["status"], "executed")
        self.preview(drive_state=1, motion_state=1, braked=0)
        self.assertEqual(self.view()["last_request"]["status"], "observed")
        self.assertTrue(self.view()["observed"]["running"])

    def test_stale_stop_supersedes_pending_start_and_ignores_old_reply(self):
        self.ready(); self.state.request_control("start")
        old = self.view()["last_request"]["id"]
        self.now += 2
        self.assertTrue(self.view()["pending"])
        self.assertTrue(self.view()["can_stop"])
        self.state.request_control("stop")
        self.assertEqual(len(self.sent), 2)
        self.reply(ident=old, action="start")
        self.assertEqual(self.view()["last_request"]["status"], "sent")
        self.reply(); self.preview()
        self.assertEqual(self.view()["last_request"]["status"], "queued")
        self.reply("executed")
        self.assertEqual(self.view()["last_request"]["status"], "observed")
        self.assertTrue(self.view()["observed"]["stopped"])

    def test_reply_identity_epoch_and_retention(self):
        self.ready(); self.state.request_control("start")
        for fields in (dict(ident=99), dict(action="reset"), dict(epoch=8)):
            self.reply(**fields)
            self.assertEqual(self.view()["last_request"]["status"], "sent")
        self.state.ingest(REPLY_TOPIC, encode_reply(51, "start", "queued", epoch=7), retained=True)
        self.assertEqual(self.view()["last_request"]["status"], "sent")
        self.reply("rejected", reason="not_ready")
        self.assertEqual(self.view()["last_request"]["status"], "rejected")
        self.assertIn("not ready", self.view()["last_request"]["detail"])

    def test_timeout_disconnect_reconnect_never_resend(self):
        self.ready(); self.state.request_control("start")
        self.reply(); self.now += 8
        self.assertEqual(self.view()["last_request"]["status"], "timeout")
        self.reply("executed")
        self.assertEqual(self.view()["last_request"]["status"], "timeout")
        self.state.set_connection("disconnected")
        with self.assertRaises(ValueError): self.state.request_control("stop")
        self.state.set_connection("connected")
        self.assertEqual(len(self.sent), 1)
        self.assertFalse(self.view()["can_start"])
        self.assertTrue(self.view()["can_stop"])

    def test_new_epoch_invalidates_pending_start_and_uses_current_id(self):
        self.ready(); self.state.request_control("start")
        self.state.ingest(STATUS_TOPIC, encode_status(8, 500, 0, 3))
        self.assertEqual(self.view()["last_request"]["status"], "error")
        self.assertFalse(self.view()["observed"]["fresh"])
        self.assertFalse(self.view()["can_start"])
        self.preview()
        self.state.request_control("start")
        self.assertEqual(self.sent[-1][1], encode_request(1, "start", 8, 500))

    def test_stop_without_any_telemetry_and_current_epoch_reply(self):
        self.state.request_control("stop")
        self.assertEqual(self.sent[0][1], encode_request(1, "stop", 0, 0))
        self.reply(epoch=99)
        self.assertEqual(self.view()["last_request"]["status"], "queued")
        self.preview(); self.reply("executed", epoch=99)
        self.assertEqual(self.view()["last_request"]["status"], "observed")

    def test_observed_running_does_not_override_later_firmware_rejection(self):
        self.ready(); self.state.request_control("start"); self.reply()
        self.preview(drive_state=1, motion_state=1, braked=0)
        self.assertTrue(self.view()["observed"]["running"])
        self.assertEqual(self.view()["last_request"]["status"], "queued")
        self.reply("cancelled", reason="superseded")
        self.assertEqual(self.view()["last_request"]["status"], "cancelled")

    def test_fault_reset_never_starts_automatically(self):
        self.ready(flags=5, drive_state=6, reason=2, motion_state=5, motion_fault=1)
        self.assertTrue(self.view()["can_reset"])
        self.assertFalse(self.view()["can_start"])
        self.state.request_control("reset"); self.reply("executed")
        self.preview()
        self.assertEqual(self.view()["last_request"]["status"], "observed")
        self.assertFalse(self.view()["observed"]["running"])
        self.assertEqual(len(self.sent), 1)

    def test_publish_failure_does_not_claim_queueing(self):
        self.ready()
        self.state.transport.publish.side_effect = RuntimeError("connection lost")
        with self.assertRaises(ValueError): self.state.request_control("start")
        self.assertEqual(self.view()["last_request"]["status"], "error")
        self.assertTrue(self.view()["can_stop"])

    def test_stop_bypasses_blocked_start_publish_and_late_return(self):
        self.ready()
        entered, release, stop_done = threading.Event(), threading.Event(), threading.Event()
        errors = []
        def publish(topic, payload):
            self.sent.append((topic, payload))
            if payload[1] == 1:
                entered.set()
                if not release.wait(3): raise RuntimeError("test release missing")
        self.state.transport.publish.side_effect = publish
        def command(action):
            try: self.state.request_control(action)
            except Exception as exc: errors.append(exc)
            finally:
                if action == "stop": stop_done.set()
        start = threading.Thread(target=command, args=("start",))
        start.start(); self.assertTrue(entered.wait(2))
        stop = threading.Thread(target=command, args=("stop",))
        stop.start()
        try:
            # STOP must complete while START's publish is still blocked.
            self.assertTrue(stop_done.wait(1))
            self.assertTrue(start.is_alive())
            self.assertEqual(self.view()["last_request"]["action"], "stop")
            self.assertFalse(self.view()["can_start"])
            self.reply("executed"); self.preview()
            self.assertEqual(self.view()["last_request"]["status"], "observed")
            self.assertTrue(self.view()["can_start"])
        finally:
            release.set(); start.join(3); stop.join(3)
        self.assertFalse(start.is_alive() or stop.is_alive())
        self.assertEqual(errors, [])
        self.assertEqual([p[1] for _, p in self.sent], [1, 2])
        self.assertEqual(self.view()["last_request"]["action"], "stop")
        self.reply(ident=51, action="start")
        self.assertEqual(self.view()["last_request"]["action"], "stop")
        self.assertEqual(self.view()["last_request"]["status"], "observed")

    def test_stop_preserves_prior_epoch_across_dashboard_only_reconnect(self):
        self.ready(); self.state.request_control("start")
        self.state.set_connection("disconnected")
        self.state.set_connection("connected")
        self.state.request_control("stop")
        self.assertEqual(self.sent[-1][1], encode_request(52, "stop", 7, 0))
        # A lagging status after reconnect cannot roll back the local ID.
        self.state.ingest(STATUS_TOPIC, encode_status(7, 1300000, 50, 3))
        self.preview(); self.assertFalse(self.view()["can_start"])
        self.reply("executed"); self.assertTrue(self.view()["can_start"])
        self.state.request_control("start")
        self.assertEqual(self.sent[-1][1], encode_request(53, "start", 7, 1300000))

    def test_unconfirmed_stop_timeout_still_blocks_start(self):
        self.ready(); self.state.request_control("stop")
        self.now += 8; self.ready()
        self.assertEqual(self.view()["last_request"]["status"], "timeout")
        self.assertTrue(self.view()["can_stop"])
        self.assertFalse(self.view()["can_start"])
        with self.assertRaises(ValueError): self.state.request_control("start")

    def test_demo_controls_follow_ack_and_observation_without_mqtt(self):
        self.state.mode = "demo"; self.state.demo_select("ready")
        self.state.request_control("start")
        self.assertEqual(self.view()["last_request"]["status"], "sent")
        self.now += .25; self.state.demo_ir_tick()
        self.assertEqual(self.view()["last_request"]["status"], "queued")
        self.now += .25; self.state.demo_ir_tick()
        self.assertEqual(self.view()["last_request"]["status"], "observed")
        self.assertTrue(self.view()["observed"]["running"])
        self.state.request_control("stop")
        self.now += .25; self.state.demo_ir_tick()
        self.now += .25; self.state.demo_ir_tick()
        self.assertTrue(self.view()["observed"]["stopped"])
        self.state.demo_select("fault"); self.now += .25; self.state.demo_ir_tick()
        self.state.request_control("reset")
        self.now += .25; self.state.demo_ir_tick()
        self.now += .25; self.state.demo_ir_tick()
        self.assertTrue(self.view()["observed"]["stopped"])
        self.assertEqual(self.view()["observed"]["state"], "off")
        self.assertEqual(self.sent, [])


class RunWireTests(unittest.TestCase):
    def test_fixed_little_endian_vectors_and_invalid_packets(self):
        self.assertEqual(encode_request(0x12345678, "start", 0x11223344, 0xaabbccdd).hex(),
                         "03017856341244332211ddccbbaa")
        self.assertEqual(decode_reply(bytes.fromhex("030178563412010044332211")),
                         dict(id=0x12345678, action="start", result="executed", reason="none", epoch=0x11223344))
        self.assertEqual(decode_status(bytes.fromhex("0344332211ddccbbaa7856341203"))["last_id"], 0x12345678)
        for packet in (b"", bytes(12), bytes.fromhex("030178563412040044332211")):
            with self.assertRaises(ValueError): decode_reply(packet)
        with self.assertRaises(ValueError): decode_status(encode_status()[:-1])
        with self.assertRaises(ValueError): decode_status(bytes.fromhex("0344332211ddccbbaa7856341210"))
        with self.assertRaises(ValueError): encode_request(-1, "start", 1, 1)

    def test_native_publisher_is_one_shot_qos_zero_nonretained(self):
        state = Mock()
        transport = NativeMqttTransport(state)
        transport._client = Mock(); transport._subscribed = True
        mqtt = Mock(); mqtt.CallbackAPIVersion.VERSION2 = 2
        sender = mqtt.Client.return_value
        sender.publish.return_value.is_published.return_value = True
        transport._mqtt = lambda: mqtt
        packet = encode_request(1, "stop", 0, 0)
        transport.publish(COMMAND_TOPIC, packet)
        mqtt.Client.assert_called_once_with(2, reconnect_on_failure=False)
        sender.publish.assert_called_once_with(COMMAND_TOPIC, packet, qos=0, retain=False)
        sender.disconnect.assert_called_once(); sender.loop_stop.assert_called_once()


if __name__ == "__main__":
    unittest.main()
