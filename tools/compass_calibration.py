"""Capture and check two supervised three-axis LSM303DLHC calibration sweeps."""
import argparse
import csv
import json
import math
from pathlib import Path
import re
from statistics import mean, median
import sys
import time


COLUMNS = ["timestamp_us", "x_mG", "y_mG", "z_mG"]
MAG_LINE = re.compile(rb"\[MAGCAL\] (\d+) (-?\d+) (-?\d+) (-?\d+)\r?\n")


def capture(args):
    import serial

    # DTR tells the Pico that this USB console has an active reader.
    with serial.Serial(args.port, baudrate=115200, timeout=0.25) as port:
        port.dtr = True
        port.reset_input_buffer()
        with args.output.open("x", newline="", encoding="ascii") as output:
            writer = csv.writer(output)
            writer.writerow(COLUMNS)
            start = time.monotonic()
            count = 0
            print("Recording the braked car. Rotate the assembled chassis "
                  "slowly through all orientations.", flush=True)
            while time.monotonic() - start < args.seconds:
                match = MAG_LINE.fullmatch(port.read_until(b"\n", 256))
                if match:
                    writer.writerow([int(item) for item in match.groups()])
                    count += 1
                    if count % 50 == 0:
                        print(f"{count} magnetic vectors recorded", flush=True)
                        output.flush()
    print(f"{count} samples saved to {args.output}")
    if count < 100:
        raise ValueError("Need at least 100 readings per sweep. Check the "
                         "capture firmware, USB port and stopped/braked state.")


def load(path):
    samples = []
    previous = None
    with path.open(newline="", encoding="ascii") as source:
        rows = csv.reader(source)
        if next(rows, None) != COLUMNS:
            raise ValueError(f"{path}: expected {','.join(COLUMNS)}")
        for timestamp, *axes in rows:
            stamp = int(timestamp)
            data = tuple(int(item) for item in axes)
            if (not 0 <= stamp < 2**32 or len(data) != 3 or
                    max(map(abs, data)) > 2090):
                raise ValueError(f"{path}: invalid magnetic record")
            if previous is not None:
                elapsed = (stamp - previous) % 2**32
                if not 100000 <= elapsed < 10000000:
                    raise ValueError(f"{path}: duplicate, reset or gap of 10 s or more")
            previous = stamp
            samples.append(tuple(item / 1000 for item in data))
    if len(samples) < 100:
        raise ValueError(f"{path}: at least 100 readings are needed")
    return samples


def percentile(values, fraction):
    ordered = sorted(values)
    return ordered[round((len(ordered) - 1) * fraction)]


def estimate(samples):
    """Use robust axis extremes for a diagonal soft-iron fit, in gauss."""
    limits = [(percentile(axis, .01), percentile(axis, .99))
              for axis in zip(*samples)]
    radius = [(hi - lo) / 2 for lo, hi in limits]
    if min(radius) < .05:
        raise ValueError("Insufficient swing on at least one axis")
    field = mean(radius)
    offsets = [(hi + lo) / 2 for lo, hi in limits]
    scales = [field / half_span for half_span in radius]
    if (max(map(abs, offsets)) > 5 or min(scales) < .1 or max(scales) > 10 or
            not .1 <= field <= .8):
        raise ValueError("Estimated field or coefficients outside firmware limits")
    return offsets, scales


def check_sweep(samples, offsets, scales):
    corrected = [tuple((value - bias) * gain for value, bias, gain
                       in zip(vector, offsets, scales)) for vector in samples]
    lengths = [math.sqrt(sum(v*v for v in vector)) for vector in corrected]
    strength = median(lengths)
    if min(lengths) <= 0 or not .1 <= strength <= .8:
        raise ValueError("Unusable corrected field strength")
    # Three samples in each cone prevent one isolated extreme passing coverage.
    for axis in range(3):
        for sign in (-1, 1):
            hits = sum(sign * v[axis] / length >= .85
                       for v, length in zip(corrected, lengths))
            if hits < 3:
                raise ValueError("Insufficient coverage near each of +/-X, +/-Y and +/-Z")
    error = percentile([abs(length / strength - 1) for length in lengths], .95)
    if error > .12:
        raise ValueError(f"Field-strength variation {error:.1%} exceeds 12%")
    return {"samples": len(samples), "median_gauss": strength,
            "p95_relative_radius_error": error}


def fit(first, second):
    if first == second:
        raise ValueError("Use two independently recorded sweeps")
    offset, scale = estimate(first)
    ref_offset, ref_scale = estimate(second)
    training = check_sweep(first, offset, scale)
    held_out = check_sweep(second, offset, scale)
    if (max(abs(a-b) for a,b in zip(offset, ref_offset)) >
            .12 * training["median_gauss"] or
            max(abs(a/b-1) for a,b in zip(scale, ref_scale)) > .15 or
            abs(held_out["median_gauss"]/training["median_gauss"]-1) > .12):
        raise ValueError("Sweeps disagree. Repeat away from magnetic interference.")
    return {"vehicle_axes": ["forward", "left", "up"], "offset_gauss": offset,
            "scale": scale, "training": training, "held_out": held_out}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    modes = parser.add_subparsers(dest="mode", required=True)
    record = modes.add_parser("capture")
    record.add_argument("--port", required=True)
    record.add_argument("--seconds", type=int, default=60)
    record.add_argument("--output", type=Path, required=True)
    compute = modes.add_parser("fit")
    compute.add_argument("first", type=Path)
    compute.add_argument("second", type=Path)
    args = parser.parse_args()
    try:
        if args.mode == "capture":
            if not 15 <= args.seconds <= 600:
                raise ValueError("Choose a capture duration from 15 to 600 seconds")
            capture(args)
        else:
            print(json.dumps(fit(load(args.first), load(args.second)), indent=2))
    except (ValueError, OSError, ImportError) as error:
        sys.exit(str(error))


if __name__ == "__main__":
    main()
