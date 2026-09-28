#!/usr/bin/env python3
"""
Runs the triage rule test against triage_eval() as it exists in the shipped
sketch, not against a copy that can drift out of sync.

    python3 test/triage_extract_and_test.py
"""
import os, subprocess, sys, tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
INO  = os.path.join(HERE, "..", "Vulpecula", "Vulpecula.ino")

STUBS = r"""
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#define C_RED 1
#define C_AMBER 2
#define C_YELLOW 3
#define C_PANEL 4
typedef enum { TIER_AMBIENT=0, TIER_NEAR, TIER_CONTACT } tier_t;
typedef enum { DTYPE_UNKNOWN=0, DTYPE_CAMERA, DTYPE_MIC, DTYPE_NVR, DTYPE_AV,
               DTYPE_SPEAKER, DTYPE_PHONE, DTYPE_COMPUTER } dtype_t;
#define DT_TIER_A 0
#define DT_TIER_B 1
#define DT_TIER_C 2
#define DT_TIER_D 3
#define DT_TIER_NONE 9
typedef enum { RISK_LOW=0, RISK_MEDIUM, RISK_HIGH, RISK_CRITICAL } risk_t;
enum { E_STREAM_PROFILE=1u<<3, E_ROOM_ONLY=1u<<5 };
typedef struct {
  tier_t tier_best; dtype_t dtype; uint8_t dtype_tier; uint32_t evidence;
  risk_t risk, risk_raw; uint32_t risk_since; const char *risk_why;
} track_t;
static bool dtype_is_recorder(dtype_t d){
  return d==DTYPE_CAMERA||d==DTYPE_MIC||d==DTYPE_NVR; }
"""

def main():
    src = open(INO).read()
    start = "// ===========================================================================\n// TRIAGE"
    end   = "// ===========================================================================\n// CLASSIFIER"
    if start not in src or end not in src:
        sys.exit("extraction failed: TRIAGE section markers not found in the .ino")
    triage = src[src.index(start):src.index(end)]
    for need in ("triage_eval", "RISK_CRITICAL", "risk_colour"):
        if need not in triage:
            sys.exit(f"extraction failed: {need} missing from the triage slice")

    with tempfile.TemporaryDirectory() as td:
        hdr = os.path.join(td, "triage_under_test.h")
        open(hdr, "w").write(STUBS + triage)
        test = open(os.path.join(HERE, "triage_test.cpp")).read()
        test = test.replace('#include "TRIAGE_UNDER_TEST_H"', f'#include "{hdr}"')
        tsrc = os.path.join(td, "t.cpp"); open(tsrc, "w").write(test)
        binp = os.path.join(td, "t")
        r = subprocess.run(["g++", "-std=gnu++17", "-w", tsrc, "-o", binp],
                           capture_output=True, text=True)
        if r.returncode:
            print(r.stderr[:4000]); sys.exit("compile failed")
        sys.exit(subprocess.run([binp]).returncode)

if __name__ == "__main__":
    main()
