# Vulpecula

**Proximity-gated Wi-Fi + BLE bug sweeper for the RockBase IoT NM-CYD-C5.**

A handheld counter-surveillance tool that looks for wireless cameras and
microphones in a room you occupy — a hotel room, a rental, an office. Single
`.ino`, one library dependency, receive only.

[![CI](https://github.com/OWNER/REPO/actions/workflows/ci.yml/badge.svg)](https://github.com/OWNER/REPO/actions/workflows/ci.yml)

> Replace `OWNER/REPO` in the badge URL once you have pushed.

---

## The idea

The ~2 metre detection gate is the design centre, and it is **not** a
sensitivity handicap. Room coverage comes from you walking the board around,
so the gate's job is to reject everything that is *not in this room*.

That matters because at full sensitivity a hotel gives you 100–300 BLE devices
and 50+ access points. The gate cuts that to single digits — the difference
between an instrument and a log file.

| Tier | Distance | Meaning |
|---|---|---|
| **CONTACT** | ~0.3 m | Confirmation. Open the fixture. |
| **NEAR** | ~2 m | Alert. Worth investigating. |
| **ambient** | beyond | Logged silently, never alerts. Kept for the baseline diff. |

Devices are then identified (vendor, model, device type) and triaged
**CRITICAL / HIGH / MEDIUM / LOW**, highest risk at the top of the list, with
the rule that produced each label visible on screen.

## Read this before you rely on it

- **A camera recording to SD with its radio off is invisible here.** Largest
  blind spot, unfixable in firmware. Always also sweep for lenses.
- LTE/4G cameras, wired cameras and analog transmitters are invisible.
- **No detector can tell whether a device is recording.** A detection means a
  signal matched a profile at a distance.
- **Absence of an alert is never proof of absence.**
- The shipped gate thresholds are calculated, not measured on *your* board.
  Until you run the RSSI calibration, every distance and tier is provisional.

Receive only: no probe requests, no active BLE scanning, no association,
nothing transmitted. Logs hold frame metadata only, never payloads, with MACs
written as a session-salted hash whose salt is never persisted.

**Sweep spaces you occupy or are authorised to assess.** Radio, privacy and
surveillance law varies by jurisdiction, and none of this is legal advice.

## Hardware

- **RockBase IoT NM-CYD-C5** — ESP32-C5-WROOM-1, 16 MB flash, 8 MB PSRAM,
  2.8" 320x240 ST7789 with XPT2046 resistive touch, SD slot.
- The **-Colorful-Ant** variant is worth the extra: it ships with an IPEX
  pigtail and a dual-band antenna, which gives a characterisable radiation
  pattern to calibrate against. On boards where you fit the connector
  yourself, the 0-ohm RF selector resistor must be physically moved — plugging
  an antenna into the socket does not select it.
- Optional: any spare ESP32 for the calibration reference beacon, and a
  microSD card for logging.

5 GHz coverage is the main reason to build this on a C5. Nearly every other
ESP32 detector is 2.4 GHz only, which means a 5 GHz camera is simply invisible
to it.

## Quick start

```sh
git clone https://github.com/OWNER/REPO.git
cd REPO
bash test/run_all.sh          # optional, and fast: five host checks
```

Then flash, as below. Keep the repo on a **local drive** — OneDrive and
similar can leave a file as a cloud placeholder that looks present in Explorer
but reads as empty to the compiler, and can take a file lock mid-build.

## Repository layout

```
Vulpecula/Vulpecula.ino        the whole firmware
reference_beacon/              calibration transmitter for a spare ESP32
tools/make_vendor_table.py     accurate OUI entries from the IEEE registry
test/run_all.sh                five host checks; CI runs these plus a compile
CALIBRATION.md                 the two experiments worth doing before trusting it
CONTRIBUTING.md                four gotchas that have each broken this project
CHANGELOG.md                   what changed, and the bugs found along the way
```

Arduino requires a sketch's `.ino` to match its parent folder name, which is
why the firmware sits in `Vulpecula/`. Keep `reference_beacon.ino` a sibling,
never inside that folder — Arduino compiles every `.ino` in a sketch folder as
one program, so you would get duplicate `setup()` and `loop()`.

---

## Flash it

1. **Boards Manager** → `esp32` by Espressif, **3.3.5 or newer**. Older cores
   have no ESP32-C5 support and will not compile.
2. **Library Manager** → `NimBLE-Arduino` by h2zero, 2.2.x or newer. That is
   the only library needed.
3. **Tools**:

   | Setting | Value |
   |---|---|
   | Board | ESP32C5 Dev Module |
   | USB CDC On Boot | Enabled |
   | Flash Size | 16MB (128Mb) |
   | Partition Scheme | Huge APP (3MB No OTA/1MB SPIFFS) |
   | PSRAM | **Enabled** (required — the track table lives there) |
   | Upload Speed | 460800 |

4. Plug into the **ESP32-C5 USB-C port**, not the CH340 one. Upload.
5. First boot shows colour bars labelled **R G B**. If red and blue are
   swapped, set `PANEL_BGR 0` and reflash.

## Why a single file, and why Arduino IDE

Normally PlatformIO wins on this board for one reason: upstream TFT_eSPI has
no ESP32-C5 backend, so it needs the vendor's `TFT_eSPI_ESP32_C5.c/h` dropped
into the library plus target-guard edits. In Arduino IDE that means
hand-patching a managed library that gets wiped on every update.

This sketch talks to the ST7789 directly over the stock SPI library in about
200 lines, so there is no library to patch and Arduino IDE becomes the simpler
choice. Reach for PlatformIO only if you want pinned dependency versions or to
split this back into modules.

## Display pin provenance

The vendor pinout table publishes SCK/MISO/MOSI and the three CS pins, but
**not the display DC pin**. `TFT_DC = 24` comes from RockBase-iot/NM-CYD-C5
**issue #3**, filed by someone who brought the board up on TFT_eSPI and
published a working config. Same source for `TFT_RST = -1` and SPI at 20 MHz
rather than 40.

That issue also documents two colour traps this sketch handles up front:

- the panel wants **inversion OFF**. TFT_eSPI's ST7789 init table sends INVON
  unconditionally, which is why people see red render as yellow or cyan.
- colour order may need to be **BGR**. Hence the boot colour-bar test.

The wiki calls the touch panel capacitive while also listing an XPT2046 on
GPIO 1 — XPT2046 is a *resistive* controller, so the wiki contradicts itself.
This sketch drives it as resistive, matching the vendor README and every
community project.

## Reading the list

Six rows, three lines each, highest risk first:

```
| CRIT  CAMERA   MY-IPCAM-A41F                     |  ^
|   HANGZHOU HIKVISION DS-2CD2043 *                |
|   2.4G CH6   -28DBM ~1.20M  S R C                |  =
|--------------------------------------------------|
| HIGH  MIC      DJI MIC MINI                      |  v
```

- **Line 1** risk level, device type, name or SSID
- **Line 2** vendor and model. A trailing `*` means the vendor string came
  from a WPS attribute off the air, not from the OUI table — treat those as
  exact.
- **Line 3** band, channel, peak RSSI, estimated distance, evidence letters

Scroll with the **arrows on the right**, which page by five rows and keep one
for continuity. The short bar between them is a position thumb. Tapping the
header jumps back to the top. Every row is selectable — tap one for the INFO
screen, which shows the full name on its own line plus the triage rule that
fired and each piece of evidence behind it.

## Rendering

Nothing is cleared per frame. Chrome — banners, labels, button frames, the tab
bar — is painted once when the screen changes. Values live in fields that pad
to a fixed character width, so a value that gets shorter erases its own tail,
and that diff per character cell so only changed cells are written. A steady
reading costs zero SPI traffic.

The first build cleared a full region and repainted it 4.5 times a second on
every screen, which was the flicker. The clears were never needed:
`draw_char` already writes background pixels for the off-bits of every glyph
cell, so text overwrites cleanly in place. Removing them also cut SPI traffic
by roughly an order of magnitude, which reduces contention with SD logging on
the shared bus.

## Typography

Body text is 6x8 at scale 1; headings and the RSSI readout use a real 12x16
face. The first build generated its font from DejaVu Sans Mono **Bold** and
then scaled it 2x–4x, which put 2px stems in a 6px cell and is why everything
looked heavy and oversized. Both faces are now regular weight and verified
glyph by glyph. Descenders on `g j p q y` sit on the baseline — normal for a
6x8 cell, which is why critical labels are uppercase.

## If the display does not cooperate

Set `SERIAL_ONLY 1`. The entire detector runs headless over the serial monitor
with single-key commands, so you can verify the radio side before fighting the
panel:

```
w = WiFi sweep    b = BLE sweep    t = watch mode
m = mark position p = advance baseline phase
l = list tracks   c = advance calibration
```

Then try, in order: `PANEL_DRIVER 1` (ILI9341 — the vendor says some units
ship that way), `PANEL_BGR 0`, `PANEL_INVERT 1`, `SPI_HZ_LCD 10000000`, then
`PIN_LCD_DC` 21 / 22 / 20. Issue #3 is one report, not a datasheet.

## Verify before you flash

```sh
./test/run_all.sh
```

Five checks, no hardware needed:

1. **Host compile** — ordinary type and syntax errors, under `-Wall -Wextra`.
2. **Arduino prototype injection** — the one a host compile *cannot* catch.
   The IDE preprocessor generates a forward declaration for every top-level
   function and injects the set at the first function definition it finds. Any
   injected prototype naming a type declared later in the file fails to parse,
   and the errors cascade into dozens of misleading messages pointing at
   correct code. `g++` does no such injection, so it happily compiles a sketch
   the IDE rejects. This script performs the injection itself, then compiles.
   **This is why all structs, enums and typedefs live in one block at the top
   of the sketch — do not move them back down next to the code that uses
   them.**
3. **Screen layout** — parses every `rect_t` and every literal string and
   checks it fits inside 320x240 without overlapping the tab bar or the
   procedure-page status line. Nothing at compile time knows how big the
   panel is, so a button at y=220 compiles perfectly and is unreachable.
4. **Gate behaviour** — 34 assertions.
5. **Triage rules** — 15 assertions, including the three load-bearing rules
   above. Sliced from the shipped `.ino` like the gate test, so it cannot
   drift from the rules the firmware actually uses.

The gate test slices the gate out of the shipped `.ino` rather than testing a
copy, so it cannot drift out of sync with the firmware. It covers the 2.4 GHz link budget including through-wall rejection, the
CONTACT tier against a through-wall adversary, the minimum-packet rule,
hysteresis both directions, BLE reference gating across a 24 dB transmit-power
span, and peak-vs-median under multipath nulls.

Then read **CALIBRATION.md** and actually calibrate. The shipped thresholds
are path-loss maths plus an assumed margin, not a measurement of your board
and antenna.

## First boot

1. **Colour-bar self-test.** Bars must read R G B left to right. If red and
   blue are swapped, set `PANEL_BGR 0`.
2. **Touch calibration runs automatically** the first time, because the panel
   has no usable factory mapping and guessed raw limits are not worth having.
   Tap the centre of four crosshairs, then drag on the check screen to confirm
   the dot sits under your finger, and tap ACCEPT. Stored in NVS; you will not
   see it again unless you ask for it from SETUP or WELCOME.
3. **Welcome screen.** Radios stay off until you choose a sweep.

The calibration measures the axis transposition and both inversions rather
than asking you to guess which of the eight orientations applies, and
extrapolates from the inset crosshairs out to the true screen edges so the
outer 28px stay reachable.

## Procedure pages

**BASELINE** and **RSSI CAL** in SETUP do not fire on tap. Each opens a page
explaining what the procedure does, what it needs and how to run it, with
**BACK** and an action button at the bottom. Nothing begins until you tap the
action.

Both are multi-step, and run wrong the result is worse than not running them:
a baseline taken without leaving the room flags nothing, and a calibration
with the reference in the wrong place moves the gate thresholds to values you
then trust. So the pages open at *every* stage, not just from a standing
start — mid-procedure is exactly when it matters what the next tap does. The
action button names the next step (`START`, `ROOM >`, `APPLY`, `0.30M >`,
`COMMIT`, `RETRY`) and a live status line shows the phase countdown or the
running sample count.

Procedure pages have no tab bar. BACK and the action button are the only exits,
so a stray tap cannot abandon a half-finished capture.

## Identification and triage

### Where device type comes from

Four tiers of evidence, and the tier decides how much triage trusts it:

| Tier | Source | Strength |
|---|---|---|
| **A** | Self-declared: WPS Primary Device Type in a Wi-Fi beacon, BLE Appearance, or a SIG microphone/audio-input service UUID | The device tells you what it is, in cleartext |
| **B** | Behaviour: sustained uplink-dominant traffic | No database, survives MAC randomisation |
| **C** | Advertised name or SSID pattern | Vendor naming conventions |
| **D** | OUI belongs to a camera vendor | Weakest — see the provenance warning |

The standout is **WPS**. A Wi-Fi device in SoftAP mode usually broadcasts its
Manufacturer (0x1021), Model Name (0x1023) and Primary Device Type (0x1054)
as plain attributes in every beacon. That gives an exact vendor string with no
database lookup, and a self-declared category — camera is WPS category 4, with
subcategories for still, video, web and security camera. A device announcing
itself as a security camera in a plaintext beacon beats every heuristic here.

BLE equivalents: Appearance (AD type 0x19) distinguishes **Audio Source**
(a microphone) from **Audio Sink** (a speaker), which matters a great deal —
conflating them would put every Bluetooth speaker at the top of the list.
Microphone Control (0x184D) and Audio Input Control (0x1843) service UUIDs
mean the device has a microphone in it.

The exact BLE Appearance category numbers and the WPS subcategory numbers
should be confirmed against a live capture. The WPS *category* is what drives
typing, so an unrecognised subcategory still yields the right answer.

### Vendor names

Two sources. WPS Manufacturer gives an exact string off the air and is marked
`*` in the list. Otherwise a short curated OUI table compiled in.

That table is **deliberately short** — only prefixes that could be checked.
A wrong prefix does not fail quietly: it puts a confident vendor name, device
type and triage label next to somebody's phone. To extend it accurately, run
`tools/make_vendor_table.py` against a freshly downloaded IEEE registry; it
filters the real registry by vendor keyword and emits paste-ready entries, so
prefixes never get typed from memory.

### Triage levels

Risk combines two independent questions — how confident we are this is a
recording device, and how confident we are it is in *this* room. The list is
ordered by level, but the rule that fired is recorded and shown on the INFO
screen, so a label can be judged rather than taken on faith.

Policy is **balanced**: strong type evidence at NEAR is enough for CRITICAL.
Contact range and a corridor baseline are not required.

| Level | Fires when |
|---|---|
| **CRIT** | Recorder at contact range; or tier A/B recorder inside the gate; or a recorder that is NEAR and in-room only |
| **HIGH** | Tier C/D recorder inside the gate; streaming at contact; in-room only; or anything unidentified at contact |
| **MED** | Inside the gate with weaker evidence; or a declared recorder *outside* the gate |
| **LOW** | No proximity and no type evidence |

Three rules are load-bearing and each has a test:

- **Anything outside the gate is capped at MEDIUM**, however perfect its
  signature. A declared camera in the next room must not crowd out the thing
  in the smoke detector.
- **Microphones are not down-ranked** relative to cameras. Audio needs no
  line of sight and works through fabric and plastic.
- **Declared speakers and phones are never recorders**, so a soundbar cannot
  reach CRITICAL. A triage label that is wrong once costs trust permanently.

Levels have promotion and demotion hysteresis (1.5 s up, 6 s down), and the
list re-sorts on a 1 Hz cadence rather than every frame, so rows do not churn.

## Procedures: baseline and RSSI calibration

Both are multi-step and both are worse than useless if run wrong — a baseline
taken without leaving the room flags nothing, and a calibration taken with the
reference misplaced moves the gate thresholds to values you then trust. So
both are fronted by an explainer page from SETUP, with **START** and **BACK**
at the bottom. Nothing begins until START.

The explainer appears on the **first** tap only. Once a procedure is running
the same button becomes its advance control, since re-reading instructions
mid-procedure is just in the way.

**Baseline** — START sets the corridor phase and jumps to SWEEP, which is
where the 60-second countdown lives and where you want to be looking as you
walk out of the room. Return to SETUP to advance to the room phase, then once
more to apply the diff.

**RSSI calibration** — START begins the 2.00 m stage and returns to SETUP,
where the button shows the live sample count against the 40 needed, turning
green once there are enough. Tap to move to the 0.30 m stage, tap again to
save. Full procedure and the acceptance criteria are in CALIBRATION.md.

Neither page carries a tab bar, so a stray tap cannot abandon a half-read
explanation — BACK and START are the only exits.

## Sweep modes

| Button | What it does |
|---|---|
| **SWEEP ALL** | Wi-Fi pass across the room, then a prompt, then a BLE pass. Two laps. |
| **WI-FI** | 2.4 and 5 GHz, alternating on a 1.5 s band slice. One lap. |
| **BLE** | Passive listen only, full radio. One lap. |
| **WATCH** | Round-robin all three. For leaving the board on a nightstand. |

Touch calibration is no longer on the home screen — it lives in SETUP with the
other calibration. It still runs automatically on first boot, so a board with
no stored mapping is never unusable.

**SWEEP ALL is two sequential passes, not a simultaneous one**, because the
hardware cannot do simultaneous. The C5 has one radio and one antenna, no
simultaneous dual-band, and Wi-Fi/BLE coexist by time-division. A three-way
round-robin at 33% duty would need 30+ seconds of wall clock per position to
cover a 10.24 s BLE advertiser — unusable while walking. So the Wi-Fi pass
runs, then the screen tells you to **walk the room again** for BLE. It
advances on the `BLE PASS >` button, or automatically after 150 s of Wi-Fi
radio time so an operator who never taps it still gets both.

Round-robin survives only in WATCH, where the board sits still and dwell is
unbounded.

## Using it

The 2 m gate is a **spatial filter**, not a sensitivity limit — room coverage
comes from you walking the board around. In a hotel you would otherwise hear
100–300 BLE devices and 50+ access points; the gate cuts that to single digits.

| Tier | Distance | Meaning |
|---|---|---|
| CONTACT | ~0.3 m | Confirmation. Open the fixture. |
| NEAR | ~2 m | Alert. Worth investigating. |
| ambient | beyond | Logged silently, never alerts. Kept for the baseline diff. |

Workflow:

1. **Corridor baseline** — SETUP, tap left half, stand outside the door 60 s.
2. **Room baseline** — tap left again, 60 s inside.
3. **Tap left again** to apply the diff. Anything strong in both is
   infrastructure or a neighbour; anything only inside is interesting.
4. **WiFi pass** — walk the room. Pause at each fixture until the dwell
   readout says you are covered. Tap the SWEEP body to mark each position.
5. **BLE pass** — switch mode, walk it again. Separate passes on purpose: the
   C5 cannot do 2.4, 5 GHz and BLE at once, and a round-robin would need 30+
   seconds per position to cover a slow BLE advertiser.
6. **Triage** on LIST, read the evidence on DETAIL.

Worth pausing at: smoke detectors, TV surrounds and soundbars, alarm clocks,
USB chargers and power bricks, mirrors, air vents, picture frames, desk lamps,
tissue boxes, curtain rails, and the whole headboard.

## Limits, in full

The short version is at the top of this file. The rest:

- **A camera recording to SD with its radio off is invisible here.** Largest
  blind spot, unfixable in firmware. Always also sweep for lenses.
- LTE/4G cameras, wired cameras and analog transmitters are invisible.
- No detector can tell whether a device is recording.
- Devices that rotate addresses or drop their name weaken signature matching.
  The gate, the baseline diff and the streaming heuristic still work.
- **Absence of an alert is never proof of absence.**
- The CONTACT tier has one documented failure case: a +23 dBm transmitter
  pressed against thin 8 dB drywall can reach it from the next room. Not
  fixable by moving the threshold — the ranges genuinely overlap. The test
  asserts the gap so it cannot regress silently.
- OUI matching has poor coverage on no-name cameras by construction, which is
  why the table weights it low and the non-database signals carry the weight.

## Licence

MIT — see [LICENSE](LICENSE).

The signature tables are leads requiring field confirmation, not proof of
anything. Contributions are welcome; see [CONTRIBUTING.md](CONTRIBUTING.md),
which lists four gotchas that have each already broken this project.
