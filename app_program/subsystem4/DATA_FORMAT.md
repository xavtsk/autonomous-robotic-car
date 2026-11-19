# Buddy 4 internal packet v4

12 bytes, 95 meaningful bits and one zero padding bit. Bit 0 is the least
significant bit of byte 0. Explicit packing avoids ABI padding. Arithmetic and
calibration use floats. Shared measurement snapshots and events use this packet.

| Start bit | Width | Meaning |
|---|---|---|
| 0 | 32 | Timestamp, microseconds modulo 2^32 |
| 32 | 3 | Terrain state enum, 0 unknown through 7 impact |
| 35 | 5 | Tilt valid, turn valid, peak valid, calibrated, fault |
| 40 | 11 | Signed pitch, -900..900, 0.1 degree, + = nose up |
| 51 | 11 | Signed roll, -900..900, 0.1 degree, + = right side up |
| 62 | 10 | Signed turn rate, -360..360, 1 degree/second |
| 72 | 9 | Highest completed valid hump, 0..500 mm |
| 81 | 1 | Hump completion event |
| 82 | 1 | Impact candidate |
| 83 | 1 | Hump in progress |
| 84 | 9 | Height climbed so far on that hump, 0..500 mm; zero when none |
| 93 | 2 | Safety detection: 0 clear, 1 lifted, 2 overturned, 3 reserved |
| 95 | 1 | Reserved, zero |

Signed fields use two's complement. Eleven bits cover 1801 pitch/roll values;
ten cover 721 turn-rate values; nine cover 501 height values. These are the
minimum integer widths for the declared ranges/resolutions. Invalid fields have
their validity bit cleared and numeric payload zero. Zero alone never means invalid.
Quantization bounds: 0.05 degree, 0.5 degree/second, 0.5 mm. Sensor/estimation errors
are additional and unmeasured. The operating ranges remain engineering choices,
not claimed instructor-approved values.

Version 4 uses two bits that were zero in version 3. The controller reads the
safety field in both snapshots and queued events. Buddy 1 MQTT protocol v3
reports the existing IMU fault bit and the controller terrain fault when a
safety detection stops the run.

The shared snapshot also has a 16-bit run generation and 16-bit cumulative completed
hump count. They are copied with the packet under the same priority-inheritance
mutex, not independently read across a reset. Generation wraps modulo 65536;
count saturates at 65535. Both widths are minimal for their declared 0..65535 ranges.
The controller owns run-reset requests. A pending reset invalidates the old snapshot;
only the sensing task acknowledges it with a new publication. Queue capacity remains
eight 12-byte events, and overflow is surfaced to the controller.

The controller updates only `BUDDY1_FAULT_IMU` through Buddy 1's protected masked
update API. No unsynchronised read-modify-write of the shared fault mask is used.
Highest height/count use Buddy 1's existing hump payload; no rate-to-heading cast.

Week 13 evidence still needs pack/unpack/boundary tests, actual memory/build records,
full-load timing, and physical height/impact demonstrations. Host tests alone do not
establish any physical demonstration.

## Raw accelerometer diagnostic stream

`IMU_DEBUG=1` publishes `car/debug/imu` while the Buddy 4 task is running. Packets
contain a 12-byte header followed by the sample records. Version 1 has up to
eight 18-byte samples. Version 2 has up to five 28-byte samples and appends the
hump diagnostics to each sample. Multi-byte values are little-endian.

| Header offset | Type | Field |
|---:|---|---|
| 0 | u8 | Protocol version, `1` or `2` |
| 1 | u8 | Sample count, `1..8` for v1 or `1..5` for v2 |
| 2 | u16 | Buddy 4 run token, `0` before the first run |
| 4 | u32 | Sequence of the first sample |
| 8 | u32 | Firmware boot tag |

| Sample offset | Type | Field |
|---:|---|---|
| 0 | u32 | Sample time from the Pico timer, in microseconds |
| 4 | 3 × i16 | Raw acceleration on vehicle X, Y, Z in milli-g |
| 10 | 3 × i16 | Bias-corrected acceleration on vehicle X, Y, Z in milli-g |
| 16 | u8 | Sample and detector flags |
| 17 | u8 | Safety state, `0` clear, `1` lifted, `2` overturned |
| 18 | i32 | v2 signed wheel-centre distance delta in micrometres |
| 22 | i32 | v2 qualifying forward travel accumulated for this crossing, in micrometres |
| 26 | u8 | v2 hump events and state flags |
| 27 | u8 | v2 hump rejection reason |

Flag bits 0 through 6 indicate valid raw data, a calibrated detector sample,
sensor overrun, impact, lift confirmation pending, the upward-acceleration
condition, and the tilted-lift condition. Bit 7 is reserved. The condition flags
record the firmware's comparisons before rounding acceleration for transmission.
The upward-acceleration flag is set when either pickup rule qualifies.
Sequence increments once per captured sample. A sequence gap or sensor overrun
marks missing data, and the hardware timestamp wraps modulo 2³².

In v2, a distance value of `INT32_MIN` indicates unavailable data. The distance
delta is the change in wheel-centre odometry used to assess the current sample.
Qualifying travel counts forward movement while the estimated height is at
least 5 mm. A discard record retains the accumulated travel at the time of
rejection. Hump flags from bit 0 through bit 6 mark armed, started, discarded,
completed, acceleration skipped, active after the sample, and armed after the
sample. Bit 7 is reserved.

| Code | Rejection reason |
|---:|---|
| 0 | None |
| 1 | Invalid IMU sample |
| 2 | Gap in IMU samples |
| 3 | Invalid tilt estimate |
| 4 | Odometry unavailable |
| 5 | Establishing a new odometry baseline |
| 6 | Odometry step or speed out of range |
| 7 | Safety stop or impact |
| 8 | Turn rate above limit |
| 9 | Acceleration spike |
| 10 | Reverse travel |
| 11 | Invalid geometry or tilt angle outside the hump range |
| 12 | Crossing timed out |
| 13 | Left wheel raised |
| 14 | Waiting for level samples to arm |
| 15 | Height below start threshold |
| 16 | No qualifying forward motion to start |
| 17 | Insufficient forward travel to count the crossing |
| 18 | Calibration pending |
