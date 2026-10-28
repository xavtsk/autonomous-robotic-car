# Buddy 4 internal packet v2

11 bytes, 83 meaningful bits and five zero padding bits; bit 0 is the least
significant bit of byte 0. Explicit packing avoids ABI padding. Arithmetic and
calibration use floats; shared measurement snapshots/events use the packet.
This is an internal format revision. Buddy 1 MQTT protocol v2 is unchanged.

| Start bit | Width | Meaning |
|---|---|---|
| 0 | 32 | Timestamp, microseconds modulo 2^32 |
| 32 | 3 | Terrain state enum, 0 unknown through 7 impact |
| 35 | 5 | Tilt valid, turn valid, peak valid, calibrated, fault |
| 40 | 11 | Signed pitch, -900..900, 0.1 degree |
| 51 | 11 | Signed roll, -900..900, 0.1 degree |
| 62 | 10 | Signed turn rate, -360..360, 1 degree/second |
| 72 | 9 | Highest completed valid hump, 0..500 mm |
| 81 | 1 | Hump completion event |
| 82 | 1 | Impact candidate |
| 83 | 5 | Reserved, zero |

Signed fields use two's complement. Eleven bits cover 1801 pitch/roll values;
ten cover 721 turn-rate values; nine cover 501 height values. These are the
minimum integer widths for the declared ranges/resolutions. Invalid fields have
their validity bit cleared and numeric payload zero. Zero alone never means invalid.
Quantization bounds: 0.05 degree, 0.5 degree/second, 0.5 mm. Sensor/estimation errors
are additional and unmeasured. The operating ranges remain engineering choices,
not claimed instructor-approved values.

The prior packet used 12-bit pitch/roll/rate fields (87 meaningful bits). This
revision removes four unused bits but still occupies 11 whole bytes; it does not
claim a further RAM-byte saving. All producers/consumers must use the same source.

The shared snapshot also has a 16-bit run generation and 16-bit cumulative completed
hump count. They are copied with the packet under the same priority-inheritance
mutex, not independently read across a reset. Generation wraps modulo 65536;
count saturates at 65535. Both widths are minimal for their declared 0..65535 ranges.
The controller owns run-reset requests. A pending reset invalidates the old snapshot;
only the sensing task acknowledges it with a new publication. Queue capacity remains
eight 11-byte events, and overflow is surfaced to the controller.

The controller updates only `BUDDY1_FAULT_IMU` through Buddy 1's protected masked
update API. No unsynchronised read-modify-write of the shared fault mask is used.
Highest height/count use Buddy 1's existing hump payload; no rate-to-heading cast.

Week 13 evidence still needs pack/unpack/boundary tests, actual memory/build records,
full-load timing, and physical height/impact demonstrations. Host tests alone do not
establish any physical demonstration.
