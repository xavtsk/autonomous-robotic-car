"""Independent decoder for optional 70-byte controller diagnostics v1."""
TOPIC = "car/debug/controller"
# Field order is an explicit wire contract, not host struct layout.
FIELDS = [
    ("timestamp_us",32,False),("motion_timestamp_us",32,False),("sequence",8,False),
    ("motion_available",1,False),("motion_valid",1,False),("braked",1,False),("busy",1,False),
    ("motion_state",3,False),("motion_fault",3,False),
    *[(k,13,True) for k in ("setpoint_left","setpoint_right","measured_left","measured_right")],
    ("duty_left",8,True),("duty_right",8,True),("encoder_left",20,True),("encoder_right",20,True),
    ("drive_state",3,False),("reason",4,False),("pending",3,False),("line_error",8,True),("line_flags",5,False),
    ("barcode_status",3,False),("accepted_calibration",3,False),("readiness",5,False),
    ("barcode_events",16,False),("junction_events",16,False),("lost_events",16,False),
    ("sample_gap_us",32,False),("run_max_gap_us",32,False),
    *[(f"{channel}_{field}",width,False) for channel in ("left","right","barcode")
      for field,width in (("white",12),("black",12),("white_limit",10),("black_limit",10))],
    ("cal_sensor",2,False),("collecting",1,False),
    *[(k,12,False) for k in ("white_min","white_max","black_min","black_max")],
    ("white_count",16,False),("black_count",16,False)]
DRIVE_STATES = ("off","following","braking","manoeuvre","reacquiring","halted","fault")
REASONS = ("OK","IR sensor fault","motor / encoder fault",
           "motion request rejected","line lost","junction without a direction / duplicate barcode",
           "barcode letter not mapped","controller timeout","motion report fault","bad input")
MOTION_STATES = ("idle","velocity","drive","turn","complete","fault")
MOTION_FAULTS = ("none","stall","stale command","encoder","direction","timeout")
ACTIONS = ("none","left","right","straight","u-turn")
BARCODE_STATUS = ("none","decoded","rejected","timing error","overflow")


def validate(d):
    for name, maximum in (("motion_state",5),("motion_fault",5),("drive_state",6),
                          ("reason",9),("pending",4),("barcode_status",4),("cal_sensor",2)):
        if d[name] > maximum:
            raise ValueError("Reserved controller diagnostic value")
    if abs(d["line_error"]) > 100 or abs(d["duty_left"]) > 100 or abs(d["duty_right"]) > 100:
        raise ValueError("Controller diagnostic value out of range")
    if d["motion_valid"] and not d["motion_available"]:
        raise ValueError("Motion validity requires an available snapshot")
    if any(d[f"{c}_{s}_limit"] > 1000 for c in ("left","right","barcode") for s in ("white","black")):
        raise ValueError("Calibration threshold out of range")


def decode_control(payload):
    payload = bytes(payload)
    if len(payload) != 70 or payload[0] != 1:
        raise ValueError("Controller diagnostics require 70-byte version 1")
    bits = int.from_bytes(payload[1:],"little")
    d, offset = {}, 0
    for name,width,signed in FIELDS:
        v = bits >> offset & ((1 << width)-1)
        if signed and v & (1 << (width-1)):
            v -= 1 << width
        d[name] = v
        offset += width
    if bits >> offset:
        raise ValueError("Nonzero controller diagnostic padding")
    validate(d)
    return dict(timestamp_us=d["timestamp_us"], sequence=d["sequence"], data=d)


def encode_control(**values):
    """Demo/test builder; not a robot command."""
    d = {name:values.get(name,0) for name,_,_ in FIELDS}
    validate(d)
    bits,offset = 0,0
    for name,width,signed in FIELDS:
        value = d[name]
        lower,upper = (-(1 << (width-1)),1 << (width-1)) if signed else (0,1 << width)
        if not isinstance(value,int) or not lower <= value < upper:
            raise ValueError("Controller field outside encoded range: " + name)
        bits |= (value & ((1 << width)-1)) << offset
        offset += width
    return b"\x01" + bits.to_bytes(69,"little")
