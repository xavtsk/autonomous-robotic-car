# Recorded barcode wireless format v1 — 28 October 2026

Topic `car/debug/barcode`, QoS 0, retain false. A binary packet carries one
summary, episode header, sample slice or end record. The laptop reconstructs
the original BCAP text for graphs, imports, downloads and terminal log files.
USB `bcap_format_next()` is unchanged. Old ASCII MQTT is rejected explicitly.

All fields are LSB-first, packed across byte boundaries; unused high bits are
zero. No native struct memory is transmitted. Common fields: version (8 bits,
value 1), capture ID (32-bit wrapping run-start microsecond clock), kind (2 bits:
0 summary, 1 header, 2 samples, 3 end). Capture IDs separate interleaved uploads;
they are session identifiers, not globally unique across reboots/clock wraps.
Keep a new dashboard session/archive when changing cars or starting a new demo.

| Kind | Fields after the common 42 bits | Bytes |
|---|---|---:|
| Summary | episodes 4 (0–12); not-stored 8 (0–255); decoded/rejected/timing-error/overflow counters 16 each; stored letter count 4 (0–8); that many A–Z codes, 5 each | 16–21 |
| Header | zero-based episode 4 (0–11); episode start ms 23 (0–4,294,967); advertised sample count 12 (0–2400) | 11 |
| Samples | zero-based episode 4; first sample index 12 (0–2399); count-minus-one 3 (0–5); that many samples | 15–47 |
| End | no extra fields | 6 |

Each sample has **52 meaningful bits**: raw ADC 12 (0–4095), normalized dark4
8 (0–250), flags 8, relative distance mm 16 (0–65535, saturation retained),
and delta-time ms 8 (0–255, saturation retained). Flags contain surface 2 bits
(unknown/white/black), decoder status 3 bits (0–4), collecting/rearming/long-gap
one bit each. Invalid codes/padding/lengths are rejected before display.
Start milliseconds require 23 bits because the source is a wrapping uint32
microsecond difference divided by 1000; no new run-duration limit was introduced.

The four 16-bit counters retain their existing wrapping semantics. Only eight
decoded letters are stored, independently of the total decode count. No sensor,
barcode decoder, motion, navigation, calibration or capture-trigger logic changes.

## Ownership, limits and transfer behaviour

- The existing source owner/capture mutex protects reads; network access uses
  `TMO_POL`, skips a busy lock, and resumes later without advancing the cursor.
- Packing a packet is bounded to six samples. Insufficient capacity or invalid
  data does not advance the cursor. Network staging is 47 bytes normally, or
  70 bytes when also serving controller previews (`IR_DEBUG=1`), reduced from
  the prior 192-byte text buffer.
- Ordinary application telemetry has priority. A diagnostic packet is paced and
  retained for transient publish rejection. Delivery can be missing/duplicated;
  there is no application acknowledgement or retransmission request command.
- Laptop storage uses the existing bounded archive and random file IDs. Binary
  capture IDs prevent two in-progress runs being joined. Identical retransmissions
  are ignored; conflicting rows, missing summary/header/samples/end, limits and
  saturated intervals are handled by the existing incomplete/timing warnings.
- A new run reuses the fixed capture buffer and can cancel an unfinished upload,
  as before. The laptop keeps its partial evidence and never fills gaps with zeros.
- ASCII `.txt` imports remain accepted locally. Network payloads are binary only.

Tests: `testing/integration/bcap_wire_test.c` emits real-C vectors; independent
Python roundtrips, malformed/boundary/loss/duplicate tests and isolated MQTT
integration live in `testing/dashboard/test_barcode_protocol.py`/`test_service.py`.
Actual wireless load, deadlines and STOP timing remain hardware tests.
