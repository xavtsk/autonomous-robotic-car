# Buddy 1 data representation after Tribunal 1

Source requirement: **Change Requirement 1: Minimum-Bit Data Representation**.
Implementation: `telemetry_codec.c/.h`, coordinated binary protocol version 2.
The added aggregate IR bit leaves packet byte lengths unchanged: snapshot body
117 bits in 16 bytes including version, heartbeat body 8 bits in 2 bytes.
Version 1 packets are explicitly rejected. Units below are application units;
packets also contain a one-byte version header. C enums are temporary decoded
values only: their compiler-dependent representation is never the wire format.

| Application field | Valid range / codes | Resolution | Data bits | Implementation / validation status |
|---|---|---|---:|---|
| Each wheel magnitude | 0–510 mm/s | 2 mm/s | 8 | explicit bits; proposed team range; verify against calibration |
| Each wheel reverse flag | forward/zero or reverse | Boolean | 1 | separate sign bit; no negative zero |
| Barcode letter | A–Z mapped to 0–25 | one letter | 5 | checked before packing |
| Barcode action | unknown, forward, left, right, uturn (0–4) | one action | 3 | 5–7 rejected |
| Hump count | 0–65535 | one hump | 16 provisional | full existing API range preserved; Buddy 4 must supply physical/run maximum |
| Hump height | 0–65535 mm | 1 mm | 16 provisional | Buddy 4 must validate actual range and required error before narrowing |
| Heading | 0–359 degrees | 1 degree | 9 | 360–511 rejected |
| Pose distance since START | 0–4095 cm, modulo (wrapping counter) | 1 cm | 12 | width set by the largest step between samples plus a loss margin, not the run length; laptop unwraps |
| Pose heading since START | 0–359 degrees | 1 degree | 9 | 360–511 rejected; live-only packet |
| Obstacle distance | 0–65535 mm | 1 mm | 16 provisional | Buddy 5 must supply validated sensor/operating range |
| Obstacle width | 0–65535 mm | 1 mm | 16 provisional | Buddy 5 must supply maximum profile width and clearance precision |
| Obstacle route | unknown/left/right (0–2) | one route | 2 | code 3 rejected |
| Robot state | eight valid states (0–7) | one state | 3 | unknown represented by snapshot validity, not a ninth code |
| Snapshot section validity | invalid/valid for seven sections | Boolean | 7 total | validity separate from numeric zero |
| Shared telemetry faults | stall, encoder, IMU, no echo, Wi-Fi loss, IR sensor fault | Boolean | 6 total | protected mask; bit ownership documented in protocol |
| Each configured barcode letter | A–Z mapped to 0–25 | one letter | 5 | four letters = 20 bits; all different |
| Configuration open/closed | before run / frozen | Boolean | 1 | stored in spare bit of LOCAL map version byte; never sent |
| Mapping reply result | OK/invalid/closed/internal | one result | 2 | exact two-byte packet including version |
| Heartbeat alive | true | Boolean | 1 | versioned status packet; not proof every task is healthy |
| Heartbeat fault validity | unknown / update received | Boolean | 1 | faults occupy six adjacent bits |
| Outbox topic selector | 11 topics (0–10) | one topic | 4 | explicit one-byte envelope; high nibble zero; replaces 64-byte text topic; compile-time check that topics fit 4 bits |
| Incoming command selector | unknown/snapshot/map (0–2) | one kind | 2 | one-byte envelope; upper six bits zero |

The physical storage unit is a byte. Fields are packed across byte boundaries;
the final packet may have up to seven zero padding bits. Fixed RTOS slots reserve
space for the largest packet to retain the agreed bounded message counts. The
queue slot tail is storage capacity, not extra application fields. MQTT publishes
only the packet's exact length.

The 16-bit provisional fields are deliberately not reduced using invented limits.
They are binary and bounded by the existing `UH` API, but final minimum-bit
compliance for Buddy 4/5 is still OPEN until their owners justify the ranges.
Changing any field width or meaning requires updating the codec, laptop decoder,
this table and tests together, and increasing the wire version for an incompatible
released format. Do not silently truncate or clamp an out-of-range measurement.

Buddy 1 publishes processed reports only. It does not repack private raw IMU,
ADC, encoder or control data belonging to other buddies. Those owners must apply
the requirement to their own storage and interfaces. Line/junction telemetry,
encoder counts and distance travelled are not newly added by this change; add
their agreed fields when those producers are integrated.

For measured sizes, RAM accounting and remaining timing checks, see
`testing/subsystem1/EVIDENCE.md`.
