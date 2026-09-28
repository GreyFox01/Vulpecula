# Pushing this to GitHub

The repo is already initialised with one commit on `main`. Delete this file
once you have pushed — it is scaffolding, not documentation.

## 1. Create the remote

Empty repo, **no** README, licence or .gitignore — those exist here and GitHub
adding its own would cause a conflict on first push.

Name suggestion: `vulpecula`. Description:

> Proximity-gated Wi-Fi + BLE bug sweeper for the ESP32-C5 Cheap Yellow Display

Topics worth adding: `esp32`, `esp32c5`, `arduino`, `counter-surveillance`,
`wifi`, `bluetooth-le`, `hidden-camera-detector`, `rf`, `cyd`.

## 2. Push

```sh
git remote add origin https://github.com/YOUR-USER/vulpecula.git
git push -u origin main
```

If you would rather start the history fresh under your own name:

```sh
rm -rf .git
git init -b main
git add -A
git commit -m "Initial commit"
```

## 3. Three things to edit

- **`LICENSE`** — currently `Copyright (c) 2026 Vulpecula contributors`. Put
  your own name there if you want it attributed.
- **`README.md`** — replace `OWNER/REPO` in the CI badge URL and the clone
  command with your actual path.
- **`.github/workflows/ci.yml`** — nothing required, but see below.

## 4. What CI does

Two jobs on every push:

**Host checks** run in seconds with no toolchain: the `-Wall -Wextra` compile
against stubs, the Arduino prototype-injection emulation, the screen layout
check, 34 proximity-gate assertions and 15 triage assertions.

**Compile for ESP32-C5** installs the real esp32 core and NimBLE and runs
`arduino-cli compile`. This is the authoritative check — the host job uses
stub headers and cannot see a genuine core problem, and the prototype-injection
bug that broke the first build was invisible to plain `g++`.

The first run takes several minutes because the ESP32 toolchain is a large
download. It is cached on the key `esp32-core-3.3.11-nimble-2.2.3`; bump that
string when you want to pick up newer versions.

The compile job uses the bare `esp32:esp32:esp32c5` FQBN. PSRAM, partition
scheme and flash size are **runtime** settings that do not affect whether the
sketch compiles, so they are not pinned there — but they do matter when you
flash, and PSRAM must be enabled or the firmware halts at boot. The workflow
prints `arduino-cli board details` so the valid option keys appear in the log
if you ever want to pin them.

## 5. What is deliberately not committed

- **The IEEE OUI registry** and anything generated from it. Several MB,
  changes constantly, and `tools/make_vendor_table.py` regenerates it.
- **Sweep logs** (`*.csv`, `sweeps/`, `vulpecula/`). They are
  session-pseudonymised, but advertised names and SSIDs can still identify
  people and places. Treat any you export as sensitive.
- **The vendor TFT_eSPI C5 backend** (`vendor/`). Not needed any more, since
  the display is driven directly, and those are the board vendor's files under
  their own licence.
