# Provisional Buddy 4 packet v1

11 bytes; bit 0 is the least significant bit of byte 0. Explicit packing avoids
ABI padding. Raw/calculation precision remains float; only shared data is packed.

| Start bit | Width | Meaning |
|---|---|---|
| 0 | 32 | Timestamp, microseconds modulo 2^32 |
| 32 | 3 | Terrain state enum, 0 unknown through 7 impact |
| 35 | 5 | Tilt valid, turn valid, peak valid, calibrated, fault |
| 40 | 12 | Signed two's complement pitch, 0.1 degree |
| 52 | 12 | Signed two's complement roll, 0.1 degree |
| 64 | 12 | Signed two's complement turn rate, 1 degree/second |
| 76 | 9 | Highest completed valid hump, 1 mm |
| 85 | 1 | Hump completion event |
| 86 | 1 | Impact candidate |
| 87 | 1 | Reserved, zero |

Accepted ranges: pitch/roll +/-90 degrees, turn +/-360 degrees/second,
height 0..500 mm. Invalid fields have their validity bit cleared and numeric
payload zero. Zero alone never means invalid. Quantization errors are at most
0.05 degree, 0.5 degree/second and 0.5 mm respectively; sensor/estimation errors
are additional and unmeasured. These ranges are proposed, not Tribunal-approved.
