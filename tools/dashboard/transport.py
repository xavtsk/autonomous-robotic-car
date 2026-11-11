"""MQTT bridge using the Mosquitto command-line clients already used in labs."""
import os
if os.name != "nt":
    import pty
import re
import shutil
import subprocess
import threading

COMMAND_TOPICS = {"car/cmd/barcode_map", "car/cmd/snapshot", "car/cmd/control"}


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
                                     "-t", topic, "-q", "0" if topic == "car/cmd/control" else "1", "-s"], input=bytes(payload),
                                    stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                    timeout=5, check=False)
        except (OSError, subprocess.TimeoutExpired) as exc:
            raise RuntimeError("MQTT publish failed or timed out") from exc
        if result.returncode:
            raise RuntimeError(result.stderr.decode("utf-8", errors="replace").strip()[:300] or "MQTT publish failed")


class NativeMqttTransport:
    """Windows MQTT client: no Unix PTY or buffered CLI acknowledgements."""

    def __init__(self, state, heartbeat_topic="mtk3smp-pico/heartbeat"):
        self.state, self.heartbeat_topic = state, heartbeat_topic
        self._lock = threading.RLock()
        self._client = None
        self._subscribed = False
        self.host, self.port = "127.0.0.1", 1883

    @staticmethod
    def _mqtt():
        try:
            import paho.mqtt.client as mqtt
        except ImportError as exc:
            raise ValueError("Install Windows MQTT support: python -m pip install paho-mqtt==2.1.0") from exc
        return mqtt

    def connect(self, host, port):
        validate_address(host, port)
        mqtt = self._mqtt()
        self.disconnect()
        client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        client.reconnect_delay_set(min_delay=1, max_delay=10)
        client.connect_timeout = 3

        def connected(current, userdata, flags, reason, properties):
            with self._lock:
                if current is not self._client:
                    return
                self._subscribed = False
                if reason.is_failure:
                    self.state.set_connection("error", error=f"MQTT connection refused: {reason}")
                    return
                topics = ("car/telemetry/+", "car/reply/+", "car/status",
                          "car/debug/barcode", "car/debug/ir", "car/debug/controller", self.heartbeat_topic)
                result, _ = current.subscribe([(topic, 1) for topic in topics])
                if result != mqtt.MQTT_ERR_SUCCESS:
                    self.state.set_connection("error", error="MQTT subscribe failed")

        def subscribed(current, userdata, mid, reasons, properties):
            with self._lock:
                if current is not self._client:
                    return
                self._subscribed = bool(reasons) and all(not r.is_failure for r in reasons)
                self.state.set_connection("connected" if self._subscribed else "error",
                                          error=None if self._subscribed else "Broker rejected a telemetry subscription")

        def disconnected(current, userdata, flags, reason, properties):
            with self._lock:
                if current is self._client:
                    self._subscribed = False
                    self.state.set_connection("connecting", error="Broker disconnected; retrying")

        def failed(current, userdata):
            with self._lock:
                if current is self._client:
                    self._subscribed = False
                    self.state.set_connection("error", error="Cannot reach MQTT broker; retrying")

        def message(current, userdata, msg):
            with self._lock:
                if current is not self._client or msg.retain:
                    return
                if len(msg.payload) > 65536:
                    self.state.add_event(1, "warning", "Oversized MQTT payload ignored")
                    return
                self.state.ingest(msg.topic, bytes(msg.payload))

        client.on_connect, client.on_subscribe = connected, subscribed
        client.on_disconnect, client.on_connect_fail = disconnected, failed
        client.on_message = message
        with self._lock:
            self.host, self.port = host, port
            self._client = client
            self.state.set_connection("connecting", host, port)
        client.connect_async(host, port, keepalive=20)
        if client.loop_start() != mqtt.MQTT_ERR_SUCCESS:
            self.disconnect()
            raise ValueError("Cannot start MQTT connection worker")

    def disconnect(self):
        with self._lock:
            client, self._client = self._client, None
            self._subscribed = False
        if client is not None:
            client.disconnect()
            client.loop_stop()
        self.state.set_connection("disconnected")

    def publish(self, topic, payload):
        if topic not in COMMAND_TOPICS:
            raise ValueError("Unsupported MQTT command")
        with self._lock:
            if self._client is None or not self._subscribed:
                raise RuntimeError("MQTT subscriber is disconnected")
            host, port = self.host, self.port
        # Separate short-lived command connection, as with mosquitto_pub.
        # A timed-out command must not remain queued for a later reconnect.
        mqtt = self._mqtt()
        sender = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, reconnect_on_failure=False)
        sender.connect_timeout = 3
        try:
            sender.connect(host, port, keepalive=20)
            sender.loop_start()
            # Motion commands are one-shot QoS 0: never queued/replayed on a
            # reconnect. A separate correlated firmware reply confirms queueing.
            info = sender.publish(topic, bytes(payload), qos=0 if topic == "car/cmd/control" else 1, retain=False)
            info.wait_for_publish(timeout=5)
            if not info.is_published():
                raise RuntimeError("MQTT publish timed out; command outcome unknown")
        except (OSError, ValueError) as exc:
            raise RuntimeError("MQTT publish failed") from exc
        finally:
            sender.disconnect()
            sender.loop_stop()


if os.name == "nt":
    MqttTransport = NativeMqttTransport
