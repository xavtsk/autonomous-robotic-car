"""MQTT bridge using the Mosquitto command-line clients already used in labs."""
import os
import pty
import re
import shutil
import subprocess
import threading

COMMAND_TOPICS = {"car/cmd/barcode_map", "car/cmd/snapshot"}


def validate_address(host, port):
    if not isinstance(host, str) or not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9.:-]{0,252}", host):
        raise ValueError("Enter a hostname or IP address without a URL or spaces")
    if isinstance(port, bool) or not isinstance(port, int) or not 1 <= port <= 65535:
        raise ValueError("MQTT port must be between 1 and 65535")
    return host, port


class MqttTransport:
    def __init__(self, state, heartbeat_topic="mtk3smp-pico/heartbeat"):
        self.state, self.heartbeat_topic = state, heartbeat_topic
        self._lock = threading.RLock()
        self._process = None
        self._generation = 0
        self.host, self.port = "127.0.0.1", 1883

    def connect(self, host, port):
        validate_address(host, port)
        if not shutil.which("mosquitto_sub") or not shutil.which("mosquitto_pub"):
            raise ValueError("Install Mosquitto first: brew install mosquitto")
        self.disconnect()
        with self._lock:
            self._generation += 1
            generation = self._generation
            self.host, self.port = host, port
            self.state.set_connection("connecting", host, port)
            args = ["mosquitto_sub", "-h", host, "-p", str(port), "-R", "-d",
                    "-F", "%t %x", "-q", "1"]
            for topic in ("car/telemetry/+", "car/reply/+", "car/status",
                          "car/debug/barcode", "car/debug/ir", "car/debug/controller", self.heartbeat_topic):
                args.extend(("-t", topic))
            master, slave = pty.openpty()
            try:
                # Mosquitto buffers debug acknowledgements when stdout is a
                # pipe. A private PTY makes connection/subscription lines
                # visible immediately, even before the car sends a packet.
                self._process = subprocess.Popen(args, stdin=subprocess.DEVNULL,
                                                  stdout=slave, stderr=subprocess.STDOUT)
                self._process.stdout = os.fdopen(master, "rb", buffering=0)
            except OSError as exc:
                os.close(master)
                self.state.set_connection("error", error=str(exc))
                raise ValueError(str(exc)) from exc
            finally:
                os.close(slave)
            process = self._process
            threading.Thread(target=self._read, args=(process, generation),
                             name="dashboard-mqtt", daemon=True).start()

    def _read(self, process, generation):
        joined = False
        try:
            while True:
                line = process.stdout.readline(131073)
                if not line:
                    break
                with self._lock:
                    if generation != self._generation:
                        return
                    if len(line) > 131072:
                        self.state.add_event(1, "warning", "Oversized MQTT output ignored")
                        # A partial output line cannot be decoded as a packet.
                        while line and not line.endswith(b"\n"):
                            line = process.stdout.readline(131073)
                        continue
                    text = line.decode("ascii", errors="replace").strip()
                    if "received CONNACK (0)" in text:
                        joined = True
                    elif text.startswith("Subscribed (mid:") and joined:
                        granted = [int(value) for value in re.findall(r"\d+", text.split("):", 1)[-1])]
                        if granted and all(value < 128 for value in granted):
                            self.state.set_connection("connected")
                        else:
                            self.state.set_connection("error", error="Broker rejected a telemetry subscription")
                    elif "sending CONNECT" in text:
                        joined = False
                        self.state.set_connection("connecting")
                    elif text.startswith("Error:") or "Connection Refused" in text:
                        self.state.set_connection("error", error=text[:300])
                    else:
                        topic, _, hex_payload = text.partition(" ")
                        if topic.startswith("car/") or topic == self.heartbeat_topic:
                            try:
                                self.state.ingest(topic, bytes.fromhex(hex_payload))
                            except ValueError:
                                self.state.add_event(1, "warning", "Malformed MQTT client output ignored")
        except (OSError, ValueError) as exc:
            with self._lock:
                if generation == self._generation:
                    self.state.set_connection("error", error=str(exc))
        finally:
            process.stdout.close()
            with self._lock:
                if generation == self._generation:
                    self.state.set_connection("error", error="MQTT subscriber stopped; reconnect to retry")

    def disconnect(self):
        with self._lock:
            self._generation += 1
            process, self._process = self._process, None
        if process is not None and process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=1)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=1)
        self.state.set_connection("disconnected")

    def publish(self, topic, payload):
        if topic not in COMMAND_TOPICS:
            raise ValueError("Unsupported MQTT command")
        with self._lock:
            if self._process is None or self._process.poll() is not None:
                raise RuntimeError("MQTT subscriber is disconnected")
            host, port = self.host, self.port
        try:
            result = subprocess.run(["mosquitto_pub", "-h", host, "-p", str(port),
                                     "-t", topic, "-q", "1", "-s"], input=bytes(payload),
                                    stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                    timeout=5, check=False)
        except (OSError, subprocess.TimeoutExpired) as exc:
            raise RuntimeError("MQTT publish failed or timed out") from exc
        if result.returncode:
            raise RuntimeError(result.stderr.decode("utf-8", errors="replace").strip()[:300] or "MQTT publish failed")
