#!/usr/bin/env python3
"""Run the local robot dashboard: python3 tools/robot_dashboard.py --demo --open."""
import argparse
from pathlib import Path
import threading
import webbrowser

from dashboard.server import DashboardServer, demo_worker
from dashboard.state import DashboardState, HEARTBEAT_TOPIC
from dashboard.traces import TraceStore
from dashboard.transport import MqttTransport, validate_address


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--demo", action="store_true", help="Simulated data; never connects to the car")
    parser.add_argument("--open", action="store_true", help="Open the dashboard in your browser")
    parser.add_argument("--port", type=int, default=8765, help="Local web port (default 8765)")
    parser.add_argument("--broker", default="127.0.0.1", help="MQTT hostname/IP (default local Mac)")
    parser.add_argument("--mqtt-port", type=int, default=1883)
    parser.add_argument("--connect", action="store_true", help="Connect to the broker immediately")
    parser.add_argument("--heartbeat-topic", default=HEARTBEAT_TOPIC)
    parser.add_argument("--data-dir", type=Path, default=Path(__file__).resolve().parents[1] / ".dashboard-data")
    args = parser.parse_args()
    if not 0 <= args.port <= 65535:
        parser.error("--port must be between 0 and 65535")
    try:
        validate_address(args.broker, args.mqtt_port)
        if not args.heartbeat_topic or len(args.heartbeat_topic) > 256 or any(c in args.heartbeat_topic for c in "\x00\r\n#+"):
            raise ValueError("Use an exact heartbeat topic, without wildcards")
        # Demo captures never mix with real car captures.
        traces = TraceStore(args.data_dir / ("demo" if args.demo else "live"))
        state = DashboardState(traces, "demo" if args.demo else "live", heartbeat_topic=args.heartbeat_topic)
        state.connection.update(host=args.broker, port=args.mqtt_port)
        transport = MqttTransport(state, args.heartbeat_topic)
        state.transport = transport
        server = DashboardServer(("127.0.0.1", args.port), state)
    except (ValueError, OSError) as exc:
        parser.error(str(exc))
    stop_event = threading.Event()
    if args.demo:
        state.demo_select("ready")
        threading.Thread(target=demo_worker, args=(state, stop_event), daemon=True).start()
    elif args.connect:
        try:
            transport.connect(args.broker, args.mqtt_port)
        except ValueError as exc:
            state.set_connection("error", error=str(exc))
    url = f"http://127.0.0.1:{server.server_address[1]}"
    print(f"Robot dashboard: {url}", flush=True)
    print("SIMULATION — no robot connection" if args.demo else "Live mode — use Connect, then wait for car traffic", flush=True)
    print(f"Trace folder: {traces.folder}", flush=True)
    print("Press Control+C to close.", flush=True)
    if args.open:
        webbrowser.open(url)
    try:
        server.serve_forever(poll_interval=0.25)
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        transport.disconnect()
        server.server_close()


if __name__ == "__main__":
    main()
