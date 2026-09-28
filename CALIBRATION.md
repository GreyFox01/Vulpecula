# Calibration

The thresholds shipped in `config.h` are path-loss maths plus an assumed
implementation margin. They are a starting point, not a measurement of your
board. Antenna choice, whether the 0-ohm RF selector actually routes to your
U.FL socket, and board-to-board variation all move them by several dB.

Two experiments, both about thirty minutes, and both worth doing before you
trust a single reading.

---

## Experiment 1: does DFS work here?

Answered automatically at boot. Flash, watch the serial console:

```
probing 5 GHz channels...
5 GHz usable: 25 channels, DFS yes
```

Also shown on the SETUP screen. Three outcomes:

- **25 channels, DFS yes** — full coverage. Good.
- **9 channels, DFS no** — you have UNII-1 (36–48) and UNII-3 (149–165) but
  lost 52–144. A real gap; plenty of consumer cameras sit there. Consider
  setting a different country in the regulatory config if lawful where you are.
- **0 channels** — no 5 GHz at all. Something is wrong with the band-mode call
  or the core version. Fix this before going further, because 5 GHz coverage is
  the main reason to build on a C5.

---

## Experiment 2: threshold calibration

### You need

- A reference transmitter whose position you control. Use a second ESP32
  running `tools/reference_beacon/` (below), *not* your phone — you need a
  known, constant transmit power and a name you cannot mistake for a real
  find.
- A tape measure.
- A room you can leave alone for twenty minutes. Empty-ish, no people moving.

### Procedure

Repeat per band: 2.4 GHz, 5 GHz, BLE.

1. Put the CYD on a non-metallic surface at roughly chest height. Do not hold
   it — your hand costs several dB and it will not be there consistently.
2. Place the reference at a tape-measured **2.00 m**, same height, clear line
   of sight.
3. On the CYD: SETUP → tap the right half to start calibration. The prompt
   reads `Reference at 2.0 m. Hold still.`
4. Wait for the sample count to pass 40. Leave the room if you can — a body
   between the two is worth 3–6 dB.
5. Tap right half again. Prompt: `Move reference to 0.3 m.`
6. Move the reference to **0.30 m**. Wait for 40+ samples.
7. Tap right half. You should see `Done. Commit to save thresholds.`
8. Tap once more to commit. Values persist to NVS.

The routine takes the **80th percentile**, not the mean, because the gate fires
on peak. High enough to represent peak behaviour, low enough that one multipath
spike does not set the threshold.

### Acceptance criteria

Calibration fails if the two stages are less than 8 dB apart. That almost
always means one of:

- the reference never actually moved
- you measured 2 m but the reference was behind something
- the antenna is not connected, or the 0-ohm selector was never moved, so
  you are reading the PCB antenna at a fraction of the expected level

A healthy 2.4 GHz result looks roughly like NEAR ≈ −30 to −40 and
CONTACT ≈ −10 to −20, with about 16 dB between them (the free-space delta
between 2.0 m and 0.3 m is 16.5 dB). If your measured delta is much *smaller*
than 16 dB, the room is reflective enough that near-field readings are
compressed — note it, because your distance estimates will be optimistic.

### Sanity check afterwards

Take the reference to the far side of an interior wall at 2 m and confirm it
reads ambient. If it still shows NEAR, the wall is thinner than the design
assumed and you should lower `near_thresh` for that band by the difference.
This is the single most important check, because rejecting the next room is the
whole point.

---

## Reference beacon sketch

Shipped as `Vulpecula/reference_beacon/reference_beacon.ino`. Flash to
any spare ESP32 - it does not need to be a C5. It advertises with a name
nobody could mistake for a genuine find, at a fixed transmit power, and
includes an explicit TX Power Level AD field so you can exercise the
reference-based BLE path rather than only the raw-RSSI fallback.


Note the name is deliberately alarming. A calibration transmitter that could
be confused with a real detection during a later sweep is a hazard, and
`DO-NOT-TRUST` in the name means it can never quietly end up in a report.

---

## Re-calibrate when

- you change the antenna, or move the RF selector resistor
- you change enclosure, especially to anything with metal in it
- you swap boards — do not assume two NM-CYD-C5s match
- readings drift noticeably against the same reference at the same distance

Calibration state per band is shown on SETUP. An uncalibrated build says so in
yellow, on purpose: you should never be unsure whether the numbers you are
looking at were measured or assumed.
