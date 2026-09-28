#!/usr/bin/env python3
"""
Generate accurate VENDOR_OUI entries from the real IEEE registry.

WHY THIS EXISTS

The curated table compiled into Vulpecula.ino is deliberately short - only
prefixes that could be checked. Padding it out to a few hundred entries typed
from memory would not fail quietly: a wrong prefix puts a confident vendor
name, a device type, and a triage label next to somebody's phone. A label that
is wrong once costs the operator's trust permanently.

So instead of inventing prefixes, this script filters the actual registry by
vendor keyword and emits entries in exactly the format the sketch expects.
Paste the output over the VENDOR_OUI block.

USAGE

  1. Download the registry (a few MB):
         https://standards-oui.ieee.org/oui/oui.csv
  2. python3 tools/make_vendor_table.py oui.csv > vendors.txt
  3. Paste the generated lines into VENDOR_OUI[] in Vulpecula.ino.

Add your own field findings to KEYWORDS below. Anything you capture with the
firmware and confirm by hand is worth more than anything in this list.
"""

import csv
import sys

# (keyword matched case-insensitively against the vendor name, device type,
#  weight). Order matters: the first match wins, so put specific before
#  generic.
#
# STRONG is reserved for vendors that make cameras, NVRs or audio capture gear
# and essentially nothing else. A vendor that also ships phones, routers or
# laptops must be WEAK, because an OUI hit then says nothing about what the
# device in front of you actually is.
KEYWORDS = [
    # --- surveillance and IP video ---
    ("hikvision",        "DTYPE_CAMERA",  "SIG_STRONG"),
    ("dahua",            "DTYPE_CAMERA",  "SIG_STRONG"),
    ("uniview",          "DTYPE_CAMERA",  "SIG_STRONG"),
    ("zhejiang uniview", "DTYPE_CAMERA",  "SIG_STRONG"),
    ("axis communication","DTYPE_CAMERA", "SIG_STRONG"),
    ("vivotek",          "DTYPE_CAMERA",  "SIG_STRONG"),
    ("arecont",          "DTYPE_CAMERA",  "SIG_STRONG"),
    ("mobotix",          "DTYPE_CAMERA",  "SIG_STRONG"),
    ("pelco",            "DTYPE_CAMERA",  "SIG_STRONG"),
    ("bosch security",   "DTYPE_CAMERA",  "SIG_STRONG"),
    ("hanwha",           "DTYPE_CAMERA",  "SIG_STRONG"),
    ("avigilon",         "DTYPE_CAMERA",  "SIG_STRONG"),
    ("geovision",        "DTYPE_CAMERA",  "SIG_STRONG"),
    ("acti corporation", "DTYPE_CAMERA",  "SIG_STRONG"),
    ("foscam",           "DTYPE_CAMERA",  "SIG_STRONG"),
    ("amcrest",          "DTYPE_CAMERA",  "SIG_STRONG"),
    ("reolink",          "DTYPE_CAMERA",  "SIG_STRONG"),
    ("wyze",             "DTYPE_CAMERA",  "SIG_STRONG"),
    ("arlo",             "DTYPE_CAMERA",  "SIG_STRONG"),
    ("ezviz",            "DTYPE_CAMERA",  "SIG_STRONG"),
    ("annke",            "DTYPE_CAMERA",  "SIG_STRONG"),
    ("swann",            "DTYPE_CAMERA",  "SIG_STRONG"),
    ("lorex",            "DTYPE_CAMERA",  "SIG_STRONG"),
    ("vstarcam",         "DTYPE_CAMERA",  "SIG_STRONG"),
    ("nvr",              "DTYPE_NVR",     "SIG_STRONG"),

    # --- action and body cameras ---
    ("gopro",            "DTYPE_CAMERA",  "SIG_STRONG"),
    ("arashi vision",    "DTYPE_CAMERA",  "SIG_STRONG"),   # Insta360
    ("sjcam",            "DTYPE_CAMERA",  "SIG_STRONG"),
    ("axon",             "DTYPE_CAMERA",  "SIG_STRONG"),

    # --- audio capture ---
    ("rode",             "DTYPE_MIC",     "SIG_STRONG"),
    ("hollyland",        "DTYPE_MIC",     "SIG_STRONG"),
    ("saramonic",        "DTYPE_MIC",     "SIG_STRONG"),
    ("shure",            "DTYPE_MIC",     "SIG_STRONG"),
    ("sennheiser",       "DTYPE_MIC",     "SIG_STRONG"),
    ("audio-technica",   "DTYPE_MIC",     "SIG_STRONG"),
    ("shenzhen boya",    "DTYPE_MIC",     "SIG_STRONG"),

    # --- mixed-portfolio vendors: WEAK, because an OUI hit tells you the
    #     brand and nothing about the device ---
    ("dji",              "DTYPE_UNKNOWN", "SIG_WEAK"),
    ("ring",             "DTYPE_UNKNOWN", "SIG_WEAK"),
    ("google",           "DTYPE_UNKNOWN", "SIG_WEAK"),
    ("amazon",           "DTYPE_UNKNOWN", "SIG_WEAK"),
    ("tp-link",          "DTYPE_UNKNOWN", "SIG_WEAK"),
    ("d-link",           "DTYPE_UNKNOWN", "SIG_WEAK"),
    ("netgear",          "DTYPE_UNKNOWN", "SIG_WEAK"),
    ("xiaomi",           "DTYPE_UNKNOWN", "SIG_WEAK"),
    ("tuya",             "DTYPE_UNKNOWN", "SIG_WEAK"),
    ("shenzhen",         "DTYPE_UNKNOWN", "SIG_WEAK"),

    # --- SoC and module vendors found inside no-name cameras. Shared with
    #     thousands of unrelated devices, so never STRONG and never typed. ---
    ("realtek",          "DTYPE_UNKNOWN", "SIG_WEAK"),
    ("ingenic",          "DTYPE_UNKNOWN", "SIG_WEAK"),
    ("anyka",            "DTYPE_UNKNOWN", "SIG_WEAK"),
    ("espressif",        "DTYPE_UNKNOWN", "SIG_WEAK"),
    ("mediatek",         "DTYPE_UNKNOWN", "SIG_WEAK"),
]

MAX_NAME = 20          # the track's vendor[] field is 24 bytes


NOISE = ("co.,ltd.", "co.,ltd", "co.", "ltd.", "ltd", "inc.", "inc", "llc",
         "gmbh", "corporation", "corp.", "corp", "company", "limited",
         "technology", "technologies", "technolgy", "electronics",
         "electronic", "digital", "s.a.", "s.a.s", "ab", "a/s", "bv", "b.v.",
         "plc", "pte", "sdn", "bhd", "&", "-")


def short_name(name):
    """Trim a registry name to something that fits the 20-char field.

    Truncates on WORD boundaries. A name cut mid-word ("Hangzhou Hikvision C")
    reads like a different company, which is worse on a triage screen than a
    shorter but honest one.
    """
    words = [w for w in name.replace(",", " ").split()
             if w.lower().strip(".") not in NOISE and w.lower() not in NOISE]
    if not words:
        words = name.split()

    out = ""
    for w in words:
        cand = w if not out else out + " " + w
        if len(cand) > MAX_NAME:
            break
        out = cand
    # A single first word longer than the field still has to be cut, but that
    # is one unavoidable case rather than the common one.
    return out or words[0][:MAX_NAME]


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)

    out, seen = [], set()
    with open(sys.argv[1], newline="", encoding="utf-8", errors="replace") as fh:
        for row in csv.DictReader(fh):
            assign = (row.get("Assignment") or "").strip().upper()
            org    = (row.get("Organization Name") or "").strip()
            if len(assign) != 6 or not org:
                continue
            low = org.lower()
            for kw, dtype, weight in KEYWORDS:
                if kw in low:
                    if assign in seen:
                        break
                    seen.add(assign)
                    out.append((assign, short_name(org), dtype, weight, kw))
                    break

    out.sort(key=lambda r: (r[3] != "SIG_STRONG", r[1], r[0]))

    print("    // Generated by tools/make_vendor_table.py from the IEEE registry.")
    print(f"    // {len(out)} entries. Every prefix below came from the registry,")
    print("    // not from memory - but the WEIGHT and TYPE are judgement calls,")
    print("    // so review them before trusting a triage label built on one.")
    # Pad to the widest entry so the emitted table lines up like the
    # hand-written one it replaces.
    nw = max((len(r[1]) for r in out), default=0) + 3
    dw = max((len(r[2]) for r in out), default=0) + 2
    for assign, name, dtype, weight, kw in out:
        oui = "{0x%s,0x%s,0x%s}" % (assign[0:2], assign[2:4], assign[4:6])
        q = f'"{name}",'
        print(f"    {{ {oui}, {q:<{nw}}{dtype + ',':<{dw}}{weight} }},")
    print(f"\n    // matched keywords: "
          f"{len(set(r[4] for r in out))} of {len(KEYWORDS)}", file=sys.stderr)
    print(f"    // STRONG: {sum(1 for r in out if r[3]=='SIG_STRONG')}  "
          f"WEAK: {sum(1 for r in out if r[3]=='SIG_WEAK')}", file=sys.stderr)


if __name__ == "__main__":
    main()
