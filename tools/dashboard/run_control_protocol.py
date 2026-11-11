"""Version 3 run-control packets, separate from controller debug previews."""
import struct

COMMAND_TOPIC = "car/cmd/control"
REPLY_TOPIC = "car/reply/control"
STATUS_TOPIC = "car/telemetry/control"
VERSION = 3
ACTIONS = {"start": 1, "stop": 2, "reset": 3}
RESULTS = ("queued", "executed", "rejected", "cancelled")
REASONS = ("none", "invalid", "stale", "replay", "busy", "not_ready", "superseded", "unavailable")


def _uint32(value):
    if isinstance(value, bool) or not isinstance(value, int) or not 0 <= value <= 0xffffffff:
        raise ValueError("Control field must be an unsigned 32-bit integer")
    return value


def encode_request(ident, action, epoch, robot_time_us):
    if action not in ACTIONS:
        raise ValueError("Unknown control action")
    return struct.pack("<BBIII", VERSION, ACTIONS[action], _uint32(ident), _uint32(epoch), _uint32(robot_time_us))


def decode_reply(payload):
    if len(payload) != 12:
        raise ValueError("Control reply requires 12 bytes")
    version, action, ident, result, reason, epoch = struct.unpack("<BBIBBI", payload)
    if version != VERSION or action not in ACTIONS.values() or result >= len(RESULTS) or reason >= len(REASONS):
        raise ValueError("Invalid control reply version or enum")
    return dict(id=ident, action=next(k for k, v in ACTIONS.items() if v == action),
                result=RESULTS[result], reason=REASONS[reason], epoch=epoch)


def decode_status(payload):
    if len(payload) != 14:
        raise ValueError("Control status requires 14 bytes")
    version, epoch, robot_time_us, last_id, flags = struct.unpack("<BIIIB", payload)
    if version != VERSION or flags & ~15:
        raise ValueError("Invalid control status version or flags")
    return dict(epoch=epoch, robot_time_us=robot_time_us, last_id=last_id,
                available=bool(flags & 1), can_start=bool(flags & 2),
                can_reset=bool(flags & 4), running=bool(flags & 8))


def encode_reply(ident, action, result, reason="none", epoch=1):
    """Simulation/test encoder; never published as a command."""
    return struct.pack("<BBIBBI", VERSION, ACTIONS[action], _uint32(ident),
                       RESULTS.index(result), REASONS.index(reason), _uint32(epoch))


def encode_status(epoch=1, robot_time_us=0, last_id=0, flags=3):
    """Simulation/test encoder."""
    payload = struct.pack("<BIIIB", VERSION, _uint32(epoch), _uint32(robot_time_us), _uint32(last_id), flags)
    decode_status(payload)
    return payload
