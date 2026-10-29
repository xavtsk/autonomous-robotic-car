"""Binary BCAP wireless v1; readable text is reconstructed only on the laptop.

Each packet carries the capture's wrapping run-start clock. Episode identities,
sample offsets, advertised counts and an end record make loss detectable. This
is observational data, never an incoming robot command. Legacy .txt imports
remain supported by traces.py; legacy ASCII MQTT packets are not accepted here.
"""
VERSION = 1
MAX_PACKET = 47
KINDS = ("summary", "episode", "samples", "end")


def encode_barcode(capture_id, kind, **data):
    """Independent demo/test encoder; firmware uses bcap_binary_next()."""
    bits, offset = 0, 0

    def put(value, width, maximum=None):
        nonlocal bits, offset
        maximum = (1 << width)-1 if maximum is None else maximum
        if type(value) is not int or not 0 <= value <= maximum:
            raise ValueError("Barcode capture field outside declared range")
        bits |= value << offset
        offset += width

    if kind not in KINDS:
        raise ValueError("Unknown barcode capture kind")
    put(VERSION, 8); put(capture_id, 32); put(KINDS.index(kind), 2)
    if kind == "summary":
        letters = data.get("letters", "")
        if len(letters)>8 or any(not "A"<=c<="Z" for c in letters):
            raise ValueError("Invalid barcode capture letters")
        put(data["episodes"], 4, 12); put(data.get("not_stored", 0), 8)
        for field in ("decoded", "rejected", "timing_error", "overflow"):
            put(data.get(field, 0), 16)
        put(len(letters), 4, 8)
        for letter in letters:
            put(ord(letter)-65, 5, 25)
    elif kind in ("episode", "samples"):
        episode = data["episode"]
        if type(episode) is not int or not 1 <= episode <= 12:
            raise ValueError("Invalid barcode capture episode")
        put(episode-1, 4, 11)
        if kind == "episode":
            put(data["start_ms"], 23, 4294967); put(data["count"], 12, 2400)
        else:
            samples = data["samples"]
            index = data["index"]
            if not 1<=len(samples)<=6 or type(index) is not int or index+len(samples)>2400:
                raise ValueError("Invalid barcode sample slice")
            put(index, 12, 2399); put(len(samples)-1, 3, 5)
            for raw, dark4, flags, mm, dt in samples:
                if flags & 3 > 2 or (flags >> 2) & 7 > 4:
                    raise ValueError("Reserved barcode sample flags")
                for value, width, maximum in ((raw,12,4095),(dark4,8,250),
                                              (flags,8,255),(mm,16,65535),(dt,8,255)):
                    put(value, width, maximum)
    return bits.to_bytes((offset+7)//8, "little")


def decode_barcode(payload):
    payload = bytes(payload)
    if not 6 <= len(payload) <= MAX_PACKET:
        raise ValueError("Invalid binary barcode capture length")
    bits, offset = int.from_bytes(payload, "little"), 0

    def get(width):
        nonlocal offset
        if offset+width > len(payload)*8:
            raise ValueError("Truncated binary barcode capture")
        value = bits >> offset & ((1 << width)-1)
        offset += width
        return value

    if get(8) != VERSION:
        raise ValueError("Unsupported binary barcode capture version")
    capture_id, kind = get(32), KINDS[get(2)]
    frame = dict(capture_id=capture_id, kind=kind)
    if kind == "summary":
        frame.update(episodes=get(4), not_stored=get(8))
        for name in ("decoded", "rejected", "timing_error", "overflow"):
            frame[name] = get(16)
        length = get(4)
        if length>8:
            raise ValueError("Too many barcode capture letters")
        frame["letters"] = "".join(chr(65+get(5)) for _ in range(length))
    elif kind in ("episode", "samples"):
        frame["episode"] = get(4)+1
        if kind == "episode":
            frame.update(start_ms=get(23), count=get(12))
        else:
            frame["index"], count = get(12), get(3)+1
            if count>6:
                raise ValueError("Too many barcode samples per packet")
            frame["samples"] = [tuple(get(w) for w in (12,8,8,16,8)) for _ in range(count)]
    body = {key:value for key,value in frame.items() if key not in ("capture_id", "kind")}
    # Independently rebuild to reject reserved ranges, padding and extra bytes.
    if encode_barcode(capture_id, kind, **body) != payload:
        raise ValueError("Noncanonical binary barcode capture")
    if kind == "summary":
        frame["text"] = (f"[BCAP] run summary: episodes={frame['episodes']} "
                         f"not_stored={frame['not_stored']} decoded={frame['decoded']} "
                         f"letters={frame['letters'] or '-'} rejected={frame['rejected']} "
                         f"timing_error={frame['timing_error']} overflow={frame['overflow']} "
                         "| raw:dark4:flags:mm:dt")
    elif kind == "episode":
        frame["text"] = f"[BCAP] episode {frame['episode']} t={frame['start_ms']}ms samples={frame['count']}"
    elif kind == "samples":
        frame["text"] = f"BC{frame['episode']} {frame['index']}: " + " ".join(
            ":".join(map(str, row)) for row in frame["samples"])
    else:
        frame["text"] = "[BCAP] end"
    return frame
