# Buddy 1 data representation after Tribunal 1

Source requirement: **Change Requirement 1: Minimum-Bit Data Representation**.
Implementation: `telemetry_codec.c/.h`, coordinated binary protocol version 3.
Snapshot body: 102 meaningful bits, 14 bytes including version; heartbeat body
8 bits in 2 bytes. Versions 1/2 are explicitly rejected. Units are application units;
packets also contain a one-byte version header. C enums are temporary decoded
values only: their compiler-dependent representation is never the wire format.

| Application field | Valid range / codes | Resolution | Data bits | Implementation / validation status |
|---|---|---|---:|---|
| Each wheel magnitude | 0–510 mm/s | 2 mm/s | 8 | explicit bits; proposed team range; verify against calibration |
| Each wheel reverse flag | forward/zero or reverse | Boolean | 1 | separate sign bit; no negative zero |
| Barcode letter | A–Z mapped to 0–25 | one letter | 5 | checked before packing |
| Barcode action | unknown, forward, left, right, uturn (0–4) | one action | 3 | 5–7 rejected |
| Hump count | 0–65535 | one hump | 16 | matches Buddy 4's saturating cumulative count |
| Hump height | 0–500 mm | 1 mm | 9 | matches Buddy 4 packet; values 501–511 rejected |
| Heading | 0–359 degrees | 1 degree | 9 | 360–511 rejected |
| Pose distance since START | 0–4095 cm, modulo (wrapping counter) | 1 cm | 12 | width set by the largest step between samples plus a loss margin, not the run length; laptop unwraps |
| Pose heading since START | 0–359 degrees | 1 degree | 9 | 360–511 rejected; live-only packet |
| Obstacle distance | 30–4000 mm | 1 mm | 12 | matches Buddy 5; invalid uses snapshot validity, not zero distance |
| Obstacle width | 0–4095 mm | 1 mm | 12 | conservative profile width, not exact physical width |
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

The adopted ranges come from the integrated Buddy 4/5 contracts, not invented
telemetry-only limits. Their physical estimation accuracy and operating ranges
still need validation on the real car. Quantisation does not establish accuracy.
Changing any field width or meaning requires updating the codec, laptop decoder,
this table and tests together, and increasing the wire version for an incompatible
released format. Do not silently truncate or clamp an out-of-range measurement.

Normal Buddy 1 telemetry publishes processed reports. Opt-in diagnostic capture
and previews are packed by their source/controller owner; Buddy 1 transports
those bytes without reading peripherals or changing control values. Distance
and relative heading use the live-only pose packet. See `docs/IR_DEBUG_PROTOCOL.md`
and `docs/BARCODE_WIRE_PROTOCOL.md` for the source-owned diagnostic field tables.

For measured sizes, RAM accounting and remaining timing checks, see
`testing/subsystem1/EVIDENCE.md`.
