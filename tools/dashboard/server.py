"""Loopback-only HTTP service for the robot telemetry dashboard."""
import hmac
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import secrets
import threading
from urllib.parse import urlsplit

STATIC = Path(__file__).with_name("static")
MAX_BODY = 4 * 1024 * 1024


class DashboardServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def __init__(self, address, state):
        if address[0] != "127.0.0.1":
            raise ValueError("Dashboard must bind to 127.0.0.1")
        self.state = state
        self.token = secrets.token_urlsafe(32)
        super().__init__(address, Handler)


class Handler(BaseHTTPRequestHandler):
    server_version = "RobotDashboard/1"

    def log_message(self, fmt, *args):
        return  # raw trace content and session tokens never go to console logs

    def _host_ok(self):
        port = self.server.server_address[1]
        return self.headers.get("Host") in (f"127.0.0.1:{port}", f"localhost:{port}")

    def _send(self, code, data, content_type="application/json; charset=utf-8", filename=None):
        if not isinstance(data, bytes):
            data = json.dumps(data, allow_nan=False).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("Referrer-Policy", "no-referrer")
        self.send_header("Content-Security-Policy", "default-src 'self'; script-src 'self'; style-src 'self'; connect-src 'self'; img-src 'self' data:; frame-ancestors 'self'; base-uri 'none'; form-action 'self'")
        if filename:
            safe = "".join(c for c in filename if c.isascii() and (c.isalnum() or c in "._- "))[:128]
            self.send_header("Content-Disposition", f'attachment; filename="{safe or "trace.txt"}"')
        self.end_headers()
        try:
            self.wfile.write(data)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def do_GET(self):
        if not self._host_ok():
            self._send(403, {"error": "Local host required"})
            return
        path = urlsplit(self.path).path
        state = self.server.state
        try:
            if path == "/api/state":
                self._send(200, state.snapshot())
            elif path == "/api/sensors":
                self._send(200, state.sensor_snapshot())
            elif path == "/api/debug":
                self._send(200, state.debug_snapshot())
            elif path.startswith("/api/traces/"):
                parts = path.split("/")
                if len(parts) == 4:
                    self._send(200, state.traces.get(parts[3]))
                elif len(parts) == 5 and parts[4] == "download":
                    name, text = state.traces.download(parts[3])
                    self._send(200, text.encode("utf-8"), "text/plain; charset=utf-8", name)
                else:
                    self._send(404, {"error": "Not found"})
            elif path in ("/", "/index.html", "/app.css", "/app.js", "/sensor_graphs.js", "/buddy_graphs.js", "/route_map.js"):
                name = "index.html" if path == "/" else path[1:]
                data = (STATIC / name).read_bytes()
                if name == "index.html":
                    data = data.replace(b"__DASHBOARD_TOKEN__", self.server.token.encode("ascii"))
                self._send(200, data, {"index.html": "text/html", "app.css": "text/css", "app.js": "text/javascript", "sensor_graphs.js": "text/javascript", "buddy_graphs.js":"text/javascript", "route_map.js": "text/javascript"}[name] + "; charset=utf-8")
            else:
                self._send(404, {"error": "Not found"})
        except (KeyError, FileNotFoundError, ValueError):
            self._send(404, {"error": "Trace or file not found"})

    def do_POST(self):
        port = self.server.server_address[1]
        origin = self.headers.get("Origin")
        if not self._host_ok() or (origin and origin not in (f"http://127.0.0.1:{port}", f"http://localhost:{port}")):
            self._send(403, {"error": "Local origin required"})
            return
        if not hmac.compare_digest(self.headers.get("X-Dashboard-Token", "").encode("utf-8"), self.server.token.encode("ascii")):
            self._send(403, {"error": "Reload the dashboard to establish a local session"})
            return
        try:
            length = int(self.headers.get("Content-Length", "0"))
            if length <= 0 or length > MAX_BODY:
                self._send(413, {"error": "Request too large or empty"})
                return
            if self.headers.get_content_type() != "application/json":
                raise ValueError("Send JSON content")
            body = json.loads(self.rfile.read(length))
            if not isinstance(body, dict):
                raise ValueError("Expected a JSON object")
            state = self.server.state
            path = urlsplit(self.path).path
            if path == "/api/connect":
                if state.mode == "demo":
                    raise ValueError("Demo mode does not connect to a real broker")
                state.transport.connect(body.get("host"), body.get("port"))
            elif path == "/api/disconnect":
                if state.mode == "demo":
                    raise ValueError("Use the simulation scenarios in demo mode")
                state.transport.disconnect()
            elif path == "/api/map":
                state.request("map", body)
            elif path == "/api/snapshot":
                state.request("snapshot")
            elif path == "/api/demo":
                state.demo_select(body.get("scenario"))
            elif path == "/api/traces/import":
                text, name = body.get("text"), body.get("name")
                if not isinstance(text, str) or not isinstance(name, str):
                    raise ValueError("Supply a trace name and its text")
                if len(text.encode("utf-8")) > 2 * 1024 * 1024:
                    raise ValueError("Trace file must be at most 2 MiB")
                trace = state.traces.import_text(name, text)
                self._send(200, dict(ok=True, trace=trace, state=state.snapshot()))
                return
            else:
                self._send(404, {"error": "Unsupported action"})
                return
            self._send(200, dict(ok=True, state=state.snapshot()))
        except (ValueError, TypeError, UnicodeError, json.JSONDecodeError) as exc:
            self._send(400, {"error": str(exc)[:500]})
        except OSError:
            self._send(500, {"error": "Local file or MQTT client operation failed"})


def demo_worker(state, stop_event):
    periods = 0
    while not stop_event.wait(0.05):
        state.demo_ir_tick()
        periods += 1
        if periods >= 15:
            state.demo_tick()
            periods = 0
