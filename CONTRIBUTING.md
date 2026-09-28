# Contributing

## Run the checks first

```sh
bash test/run_all.sh
```

Five suites, needing only `g++` and `python3` — no hardware, no Arduino
toolchain. On Windows use Git Bash or WSL. CI runs the same five plus a real
`arduino-cli` compile for the ESP32-C5.

## Four things that will bite you

These are not style preferences. Each one has already broken this project.

**1. New types go in the TYPE DEFINITIONS block at the top.**

The Arduino IDE generates a forward declaration for every top-level function
and injects the whole set at the first function definition it finds. Any
injected prototype naming a type declared later in the file fails to parse,
and the errors cascade into dozens of misleading messages pointing at code
that is perfectly correct:

```
error: variable or field 'ring_reset' declared void
error: 'tier_name' redeclared as different kind of entity
```

A plain `g++` build does no prototype injection, so **it compiles a sketch the
IDE rejects**. `test/arduino_prototype_check.py` performs the injection itself
and then compiles. Run it.

**2. No default arguments in the `.ino`.**

A default specified in both the injected prototype and the definition is a
compile error. Add an overload or a differently named function instead.

**3. Check the layout arithmetically, not by eye.**

Nothing at compile time knows the panel is 320x240. A button at y=220 with
h=32 compiles perfectly and is unreachable behind the tab bar. A 54-character
string at x=8 clips mid-word. `test/layout_check.py` parses every `rect_t` and
every literal string; it is conservative and skips runtime-computed
coordinates, so a pass is a floor rather than a ceiling.

**4. Anything touching SPI must take the mutex.**

Display, touch and SD share the bus and are driven from two different tasks.
`SPI.beginTransaction()` does not serialise across tasks. Use `spi_take()` and
`spi_give()` around the whole logical operation.

## Changing the proximity gate

`test/extract_and_test.py` slices the gate out of the shipped `.ino` and runs
34 assertions against it, so the test cannot drift from the firmware. If you
change a threshold, the link-budget assertions will tell you what it does to
through-wall rejection. Do not weaken them to make a change pass — the whole
premise of the tool is rejecting the next room.

The same applies to `triage_extract_and_test.py`. Three rules there are
load-bearing and each has a test:

- anything outside the gate caps at MEDIUM, whatever its signature
- microphones are not down-ranked relative to cameras
- declared speakers and phones are never treated as recorders

## Adding signatures

Read the provenance warning at the top of the signature tables before adding
anything. A wrong OUI does not fail quietly — it puts a confident vendor name,
device type and triage label next to innocent hardware, and a label that is
wrong once costs the operator's trust permanently.

- `STRONG` is for vendors that make cameras, NVRs or audio capture gear and
  essentially nothing else. A vendor that also ships phones or routers must be
  `WEAK`, because an OUI hit then says nothing about what the device is.
- Do not type prefixes from memory. Use `tools/make_vendor_table.py` against a
  freshly downloaded IEEE registry.
- Field-confirmed reports are worth more than anything inferable from a
  registry. Say how you confirmed the device is what you claim.

## Verifying on hardware

Some things cannot be tested on a host, and the values involved came from spec
knowledge rather than a capture. If you can confirm any of these against real
devices, that is among the most useful contributions available:

- WPS Primary Device Type subcategory numbers under WSC 2.0
- BLE Appearance category numbers, especially Audio Source vs Audio Sink
- Whether the Microphone Control (0x184D) and Audio Input Control (0x1843)
  service UUIDs appear on real wireless microphones
- Measured RSSI at 2.00 m and 0.30 m for your board and antenna, and whether
  the reference reads ambient through an interior wall

## Scope

This is a counter-surveillance tool for spaces you occupy or are authorised to
assess. It is receive-only by design — no probe requests, no active BLE
scanning, no association, nothing transmitted — and contributions that change
that will not be merged. Neither will anything that turns it into a jammer, a
deauth tool or a means of interfering with other people's equipment.
