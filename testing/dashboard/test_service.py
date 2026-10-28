"""Local HTTP and isolated Mosquitto integration; never contacts the real car."""
import http.client
import json
from pathlib import Path
import shutil
import socket
import subprocess
import sys
import tempfile
import threading
import time
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools"))
from dashboard.server import DashboardServer
from dashboard.state import DashboardState
from dashboard.traces import TraceStore
from dashboard.transport import MqttTransport, validate_address
from dashboard.ir_protocol import encode_ir
from dashboard.control_protocol import TOPIC as CONTROL_TOPIC, encode_control


class ServiceTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.state = DashboardState(TraceStore(Path(self.tmp.name)), mode="demo")
        self.server = DashboardServer(("127.0.0.1", 0), self.state)
        self.port = self.server.server_address[1]
        threading.Thread(target=self.server.serve_forever, daemon=True).start()
        self.state.demo_select("ready")

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()
        self.tmp.cleanup()

    def request(self, path, body=None, token=True, origin=None, host=None):
        conn = http.client.HTTPConnection("127.0.0.1", self.port, timeout=3)
        headers = {"Host": host or f"127.0.0.1:{self.port}"}
        if token:
            headers["X-Dashboard-Token"] = self.server.token
        if origin:
            headers["Origin"] = origin
        if body is not None:
            headers["Content-Type"] = "application/json"
        conn.request("GET" if body is None else "POST", path,
                     None if body is None else json.dumps(body), headers)
        response = conn.getresponse()
        result = (response.status, response.read(), dict(response.getheaders()))
        conn.close()
        return result

    def test_page_assets_and_no_secret_read_access(self):
        for path in ("/", "/app.css", "/app.js", "/sensor_graphs.js", "/buddy_graphs.js", "/api/state", "/api/sensors", "/api/debug"):
            status, data, headers = self.request(path)
            self.assertEqual(status, 200)
            self.assertEqual(headers["Cache-Control"], "no-store")
            self.assertTrue(data)
        html = self.request("/")[1]
        self.assertIn(self.server.token.encode(), html)
        self.assertNotIn(b"__DASHBOARD_TOKEN__", html)
        self.assertIn(b'href="/app.css"', html)
        for path in ("/../config/wifi_secrets.h", "/api/traces/../../config", "/state.py"):
            self.assertEqual(self.request(path)[0], 404)

    def test_origin_host_and_session_required(self):
        self.assertEqual(self.request("/api/snapshot", {}, token=False)[0], 403)
        self.assertEqual(self.request("/api/snapshot", {}, origin="http://elsewhere.invalid")[0], 403)
        self.assertEqual(self.request("/api/state", host="elsewhere.invalid")[0], 403)

    def test_map_validation_async_reply_and_snapshot(self):
        self.assertEqual(self.request("/api/map", dict(forward="A", left="A", right="C", uturn="D"))[0], 400)
        status, body, _ = self.request("/api/map", dict(forward="A", left="B", right="C", uturn="D"))
        self.assertEqual(status, 200)
        self.assertEqual(json.loads(body)["state"]["commands"]["map"]["status"], "pending")
        time.sleep(0.65)
        self.state.demo_tick()
        self.assertEqual(self.state.snapshot()["commands"]["map"]["status"], "accepted")
        self.assertEqual(self.request("/api/snapshot", {})[0], 200)

    def test_demo_cannot_connect_real_broker_and_motor_commands_unsupported(self):
        self.assertEqual(self.request("/api/connect", {"host": "127.0.0.1", "port": 1883})[0], 400)
        self.assertEqual(self.request("/api/start", {})[0], 404)

    def test_trace_import_retrieve_download(self):
        text = "[BCAP] run summary: episodes=0 not_stored=0 decoded=0 letters=- rejected=0 timing_error=0 overflow=0 | raw:dark4:flags:mm:dt\n[BCAP] end\n"
        status, body, _ = self.request("/api/traces/import", {"name": "my_capture.txt", "text": text})
        self.assertEqual(status, 200)
        ident = json.loads(body)["trace"]["id"]
        self.assertTrue(json.loads(self.request(f"/api/traces/{ident}")[1])["complete"])
        status, content, headers = self.request(f"/api/traces/{ident}/download")
        self.assertEqual(content.decode(), text)
        self.assertIn("attachment", headers["Content-Disposition"])
        self.assertEqual(self.request("/api/traces/import", {"name": "../escape", "text": text})[0], 400)


class TransportTests(unittest.TestCase):
    def test_address_validation(self):
        for host in ("localhost", "127.0.0.1", "172.20.10.2", "test-broker.local"):
            self.assertEqual(validate_address(host, 1883), (host, 1883))
        for host, port in (("-h", 1883), ("mqtt://host", 1883), ("host x", 1883), ("localhost", True), ("localhost", 0)):
            with self.assertRaises(ValueError):
                validate_address(host, port)

    @unittest.skipUnless(shutil.which("mosquitto") and shutil.which("mosquitto_pub"), "Mosquitto unavailable")
    def test_real_mqtt_binary_packets_commands_and_disconnect(self):
        with tempfile.TemporaryDirectory() as folder:
            with socket.socket() as sock:
                sock.bind(("127.0.0.1", 0))
                port = sock.getsockname()[1]
            broker = subprocess.Popen(["mosquitto", "-p", str(port)], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            transport = None
            try:
                deadline = time.monotonic() + 3
                while time.monotonic() < deadline:
                    try:
                        with socket.create_connection(("127.0.0.1", port), timeout=0.2):
                            break
                    except OSError:
                        time.sleep(0.05)
                state = DashboardState(TraceStore(Path(folder)))
                transport = MqttTransport(state)
                state.transport = transport
                transport.connect("127.0.0.1", port)
                deadline = time.monotonic() + 4
                while state.snapshot()["connection"]["status"] != "connected" and time.monotonic() < deadline:
                    time.sleep(0.05)
                self.assertEqual(state.snapshot()["connection"]["status"], "connected")
                subprocess.run(["mosquitto_pub", "-h", "127.0.0.1", "-p", str(port), "-t", "car/telemetry/motion", "-s"], input=bytes.fromhex("023c7602"), timeout=3, check=True)
                deadline = time.monotonic() + 3
                while not state.snapshot()["sections"]["motion"]["valid"] and time.monotonic() < deadline:
                    time.sleep(0.05)
                self.assertEqual(state.snapshot()["sections"]["motion"]["data"]["right_mms"], -118)
                preview = encode_ir(50000, 1, [100, 200, 3000], [0, 50, 900], [1, 1, 2])
                subprocess.run(["mosquitto_pub", "-h", "127.0.0.1", "-p", str(port), "-t", "car/debug/ir", "-s"], input=preview, timeout=3, check=True)
                deadline = time.monotonic() + 3
                while not state.sensor_snapshot()["received"] and time.monotonic() < deadline:
                    time.sleep(0.05)
                self.assertEqual(state.sensor_snapshot()["latest"]["sensors"]["barcode"]["raw"], 3000)
                subprocess.run(["mosquitto_pub","-h","127.0.0.1","-p",str(port),"-t",CONTROL_TOPIC,"-s"],
                               input=encode_control(timestamp_us=100000,motion_available=1,motion_valid=1,
                                                    setpoint_left=40,measured_left=35,drive_state=5,reason=4),timeout=3,check=True)
                deadline=time.monotonic()+3
                while not state.debug_snapshot()["controller"]["received"] and time.monotonic()<deadline:
                    time.sleep(0.05)
                self.assertEqual(state.debug_snapshot()["controller"]["latest"]["data"]["reason"],4)
                capture = subprocess.Popen(["mosquitto_sub", "-h", "127.0.0.1", "-p", str(port), "-t", "car/cmd/barcode_map", "-C", "1", "-W", "3", "-F", "%x"], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                time.sleep(0.15)
                state.request("map", dict(forward="A", left="B", right="C", uturn="D"))
                output, errors = capture.communicate(timeout=4)
                self.assertEqual(output.strip(), b"02208801", errors)
                subprocess.run(["mosquitto_pub", "-h", "127.0.0.1", "-p", str(port), "-t", "car/reply/barcode_map", "-s"], input=b"\x02\x00", timeout=3, check=True)
                deadline = time.monotonic() + 3
                while state.snapshot()["commands"]["map"]["status"] == "pending" and time.monotonic() < deadline:
                    time.sleep(0.05)
                self.assertEqual(state.snapshot()["commands"]["map"]["status"], "accepted")
                with self.assertRaises(ValueError):
                    transport.publish("car/cmd/start", b"anything")
                transport.disconnect()
                self.assertTrue(state.snapshot()["sections"]["motion"]["stale"])
            finally:
                if transport:
                    transport.disconnect()
                broker.terminate()
                broker.wait(timeout=2)


if __name__ == "__main__":
    unittest.main()
