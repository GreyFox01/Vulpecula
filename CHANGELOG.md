# Changelog

Notable changes, and in particular the bugs found along the way — several were
silent dead code paths rather than crashes, and they are recorded here because
the same mistakes are easy to reintroduce.

## Unreleased

### Added

- **Procedure pages** for baseline capture and RSSI calibration. Both open an
  explanation page with BACK and an action button instead of firing on tap.
  They open at every stage, not just from a standing start, and the action
  button names the next step (`START` → `ROOM >` → `APPLY`). No tab bar, so a
  stray tap cannot abandon a half-finished capture.
- **List scrolling** — a dedicated 26px column with two large arrow targets, a
  position thumb, and header-tap to jump to the top. Every row is selectable;
  the last row is no longer stolen for paging.
- **Device type identification** from four tiers of evidence: self-declared
  (WPS Primary Device Type, BLE Appearance, SIG microphone service UUIDs),
  behavioural (uplink-dominant traffic), name patterns, and OUI.
- **WPS information element parser.** Manufacturer (0x1021), Model Name
  (0x1023), Device Name (0x1011) and Primary Device Type (0x1054) are read
  from beacons. An exact vendor string off the air with no database lookup.
- **Triage** — CRITICAL / HIGH / MEDIUM / LOW from an auditable rule table.
  The rule that fired is recorded and shown, with promotion and demotion
  hysteresis. Three load-bearing rules: anything outside the gate caps at
  MEDIUM, microphones are not down-ranked relative to cameras, and declared
  speakers and phones are never treated as recorders.
- **Guided 4-point touch calibration**, persisted to NVS, deriving the axis
  transposition and both inversions empirically. Runs automatically on first
  boot.
- **Welcome screen** with mode selection. Radios stay off until a sweep is
  chosen.
- **SWEEP ALL** — two sequential passes, Wi-Fi then BLE, with a prompt between.
  Not simultaneous, because the hardware cannot be.
- **Dwell accounting** on the sweep screen: radio time held at this position
  and the longest transmit interval that honestly covers.
- **Boot-time 5 GHz channel probe.** The C5 supports DFS channels but only
  passive radar detection, so the regulatory table may refuse 52–144. Measured
  rather than assumed, and reported on SETUP.
- **Five host check suites** (`test/run_all.sh`), needing only `g++` and
  `python3`.

### Fixed

- **SSID parsing had never once returned a result.** The capture buffer was 36
  bytes and the SSID element starts at exactly offset 36, so the guard
  `avail <= off + 1` was true for every frame at every length. SSIDs were
  never captured, `E_SOFTAP_CAM_SSID` and `E_HIDDEN_SSID` never fired, and
  every Wi-Fi track displayed as "(unnamed)". The entire SoftAP camera
  signature path was inert. Management frames now get a separate 320-byte
  payload queue.
- **Display flicker.** Every screen cleared a full region and repainted it 4.5
  times a second. The clears were never necessary — `draw_char` already writes
  background pixels for the off-bits of every glyph cell. Replaced with
  chrome-once painting and per-character-cell dirty fields; a steady reading
  now costs zero SPI traffic, and total SPI traffic dropped by roughly an
  order of magnitude, reducing contention with SD logging.
- **Touch mapped to the wrong place.** The axes were swapped *after* mapping
  and then rescaled, compounding the error, and the raw limits were invented
  constants. Swap now happens in the raw domain before mapping, and the limits
  are measured.
- **Detail screen could show the wrong device's name.** The chrome cache was
  keyed on `mac[5]` alone, so two tracks sharing a last MAC byte would display
  each other's header. With a few hundred tracks that collision is likely, not
  theoretical. Now keyed on the full MAC plus band.
- **Arduino prototype injection broke the build.** The IDE generates a forward
  declaration for every top-level function and injects the set at the first
  function definition. Type definitions sat two-thirds down the file, so the
  injected prototypes referenced types that did not exist yet and the errors
  cascaded into dozens of misleading messages. All structs, enums and typedefs
  now live in one block at the top. A plain `g++` build cannot catch this,
  which is why `arduino_prototype_check.py` exists.
- **SPI bus was not serialised across tasks.** SD logging runs on the pump
  task while drawing and touch run on the UI task.
  `SPI.beginTransaction()` alone does not protect against preemption, and a
  preempted transfer leaves the display mid-command with CS asserted. All SPI
  users now take a recursive mutex.
- **CONTACT tier was reachable through a wall.** At −20 dBm a +20 dBm camera
  0.3 m behind a 10 dB wall reads −19.7 dBm and tripped the confirmation tier
  from the next room. Tightened to −15 dBm (2.4 GHz) and −22 (5 GHz). One
  residual case remains and is documented and asserted: the ranges genuinely
  overlap, so a +23 dBm transmitter on thin 8 dB drywall can still reach it.
- **Font was too heavy and descenders clipped.** Generated from DejaVu Sans
  Mono Bold and scaled 2×–4×, which put 2px stems in a 6px cell. Regenerated
  at regular weight with a real 12x16 face for headings, and the display is
  now upper case throughout, since a 6x8 cell has no room below the baseline.
  The SD log still records names in their true case.
- **Procedure page body collided with the status line** at 14 lines. Caught by
  the new layout check, which is why that check exists.

### Changed

- Home screen: TOUCH CAL removed (it lives in SETUP with the other
  calibration), subtitle is now **BUG SWEEPER**, WATCH widened to full width.
- Framework switched from PlatformIO to a single-file Arduino sketch. Upstream
  TFT_eSPI has no ESP32-C5 backend, so it needs the vendor's processor files
  plus target-guard edits — hand-patching a managed library in Arduino IDE
  that gets wiped on every update. The ST7789 is now driven directly over the
  stock SPI library in about 200 lines, leaving NimBLE-Arduino as the only
  dependency.
- `draw_text` lost its scale parameter rather than gaining a default argument.
  A default specified in both the injected prototype and the definition is
  itself a compile error, so default arguments are avoided in the `.ino`.

### Hardware notes

- `TFT_DC = 24` and `TFT_RST = -1` are from RockBase-iot/NM-CYD-C5 issue #3,
  not the vendor pinout table, which omits them. Same source for SPI at
  20 MHz rather than 40.
- The panel needs inversion **off**. TFT_eSPI's ST7789 init table sends INVON
  unconditionally, which is why red renders as yellow or cyan on this board.
- The vendor wiki calls the touch panel capacitive while also listing an
  XPT2046, which is a resistive controller. Driven as resistive, matching the
  vendor README.
