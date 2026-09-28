#!/usr/bin/env python3
"""
Runs the proximity-gate test suite against the gate as it exists in the
shipped sketch - not against a copy that can drift out of sync.

It slices the config block and the gate section straight out of the .ino
using the section marker comments, wraps them in host stubs, and compiles
gate_test.cpp against the result.

    python3 test/extract_and_test.py

Needs only g++. No hardware, no Arduino toolchain.
"""
import os, subprocess, sys, tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
INO  = os.path.join(HERE, "..", "Vulpecula", "Vulpecula.ino")

STUBS = r'''
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <initializer_list>
#define C_RED 1
#define C_AMBER 2
#define C_GREEN 3
typedef int esp_err_t;
#define ESP_OK 0
typedef int nvs_handle_t;
typedef enum { NVS_READONLY, NVS_READWRITE } nvs_open_mode_t;
static inline esp_err_t nvs_open(const char*,nvs_open_mode_t,nvs_handle_t*){return -1;}
static inline esp_err_t nvs_get_blob(nvs_handle_t,const char*,void*,size_t*){return -1;}
static inline esp_err_t nvs_set_blob(nvs_handle_t,const char*,const void*,size_t){return 0;}
static inline esp_err_t nvs_commit(nvs_handle_t){return 0;}
static inline void nvs_close(nvs_handle_t){}
// NVS names moved to the top of the sketch when the touch calibration
// started persisting to the same namespace, so they fall outside the slices.
static const char *NVS_NS  = "vulpecula";
static const char *NVS_KEY = "prox_cal";
'''

def cut(text, start, end, what):
    """Slice text between two markers, asserting both exist."""
    if start not in text:
        sys.exit(f"extraction failed: start marker for {what} not found - "
                 f"did the section markers in the .ino change?")
    i = text.index(start)
    if end not in text[i:]:
        sys.exit(f"extraction failed: end marker for {what} not found")
    return text[i:i + text[i:].index(end)]


def main():
    src = open(INO).read()

    # 1. every tunable #define (thresholds, ring sizes, BLE ranging, track
    #    table sizes - track_t needs NAME_LEN and SWEEP_PROFILE_SLOTS)
    cfg = cut(src, "#define THRESH_NEAR_24", '#define LOG_DIR', "config") \
        + '#define LOG_DIR "/sweeps"\n'

    # 2. the hoisted TYPE DEFINITIONS block. These live at the top of the
    #    sketch because the Arduino preprocessor injects function prototypes
    #    ahead of them otherwise - see arduino_prototype_check.py.
    # ends at the SPI arbitration block, which sits between the type block and
    # the display driver and pulls in FreeRTOS symbols the host stubs omit
    types = cut(src, "typedef enum { BAND_24 = 0",
                "// ---------------------------------------------------------------------------\n// SPI BUS ARBITRATION",
                "types")

    # 3. the gate implementation itself
    gate = cut(src, "static prox_cal_t g_cal;",
               "// ===========================================================================\n// TRACK TABLE",
               "gate")
    gate = gate.rsplit("// ====", 1)[0]

    for need, blob, what in (("THRESH_NEAR_24", cfg, "config"),
                             ("DIST_CONTACT_M", cfg, "config"),
                             ("SWEEP_PROFILE_SLOTS", cfg, "config"),
                             ("rssi_ring_t", types, "types"),
                             ("ranging_ref_t", types, "types"),
                             ("prox_resolve_tier", gate, "gate"),
                             ("prox_ref_from_ibeacon", gate, "gate")):
        if need not in blob:
            sys.exit(f"extraction failed: {need} missing from the {what} slice")

    with tempfile.TemporaryDirectory() as td:
        hdr = os.path.join(td, "gate_under_test.h")
        open(hdr, "w").write(STUBS + "\n" + cfg + "\n" + types + "\n" + gate + "\n")
        binp = os.path.join(td, "gate_test")
        test = open(os.path.join(HERE, "gate_test.cpp")).read()
        test = test.replace('#include "/tmp/g_cfg.h"', f'#include "{hdr}"')
        tsrc = os.path.join(td, "t.cpp")
        open(tsrc, "w").write(test)

        r = subprocess.run(["g++", "-std=gnu++17", "-w", tsrc, "-o", binp],
                           capture_output=True, text=True)
        if r.returncode:
            print(r.stderr[:4000])
            sys.exit("compile failed")
        sys.exit(subprocess.run([binp]).returncode)


if __name__ == "__main__":
    main()
