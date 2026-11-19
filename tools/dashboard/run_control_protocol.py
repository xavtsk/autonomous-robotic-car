"""Version 3 requests/status and compact version 4 run-control replies."""
import struct

COMMAND_TOPIC = "car/cmd/control"
REPLY_TOPIC = "car/reply/control"
STATUS_TOPIC = "car/telemetry/control"
VERSION = 3
REPLY_VERSION = 4
REPLY_ACTION_BITS = 2
REPLY_RESULT_BITS = 2
REPLY_REASON_BITS = 3
REPLY_BITS = REPLY_ACTION_BITS + REPLY_RESULT_BITS + REPLY_REASON_BITS
REPLY_SIZE = 10
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
    if len(payload) == 12 and payload[0] == VERSION:
        # Existing flashed firmware still sends the original version 3 layout.
        _, action, ident, result, reason, epoch = struct.unpack("<BBIBBI", payload)
    elif len(payload) == REPLY_SIZE and payload[0] == REPLY_VERSION:
        _, packed, ident, epoch = struct.unpack("<BBII", payload)
        if packed >> REPLY_BITS:
            raise ValueError("Reserved control reply bits must be zero")
        action = packed & ((1 << REPLY_ACTION_BITS) - 1)
        result = (packed >> REPLY_ACTION_BITS) & ((1 << REPLY_RESULT_BITS) - 1)
        reason = (packed >> (REPLY_ACTION_BITS + REPLY_RESULT_BITS)) & ((1 << REPLY_REASON_BITS) - 1)
    else:
        raise ValueError("Control reply requires version 3/12 bytes or version 4/10 bytes")
    if action not in ACTIONS.values() or result >= len(RESULTS) or reason >= len(REASONS):
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


def encode_reply(ident, action, result, reason="none", epoch=1, *, version=REPLY_VERSION):
    """Simulation/test encoder; never published as a command."""
    if version == VERSION:
        return struct.pack("<BBIBBI", VERSION, ACTIONS[action], _uint32(ident),
                           RESULTS.index(result), REASONS.index(reason), _uint32(epoch))
    if version != REPLY_VERSION:
        raise ValueError("Unsupported control reply version")
    packed = (ACTIONS[action] | RESULTS.index(result) << REPLY_ACTION_BITS |
              REASONS.index(reason) << (REPLY_ACTION_BITS + REPLY_RESULT_BITS))
    return struct.pack("<BBII", REPLY_VERSION, packed, _uint32(ident), _uint32(epoch))


def encode_status(epoch=1, robot_time_us=0, last_id=0, flags=3):
    """Simulation/test encoder."""
    payload = struct.pack("<BIIIB", VERSION, _uint32(epoch), _uint32(robot_time_us), _uint32(last_id), flags)
    decode_status(payload)
    return payload
