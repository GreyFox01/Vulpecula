#!/usr/bin/env python3
"""
Reproduces the Arduino IDE's automatic prototype injection, then compiles.

WHY THIS EXISTS
The Arduino preprocessor generates a forward declaration for every top-level
function in a .ino and injects the entire set at the first function definition
it finds. If any injected prototype names a type that is declared later in the
file, it fails to parse and the errors cascade into dozens of misleading
messages pointing at correct code.

A plain g++ build of the same .ino succeeds, because g++ does no prototype
injection. So a host compile alone CANNOT catch this. This script closes that
gap: it performs the injection itself and then compiles, which is what the IDE
effectively does.

    python3 test/arduino_prototype_check.py

Needs g++ and the host stubs in test/stubs/.
"""
import os, re, subprocess, sys, tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
INO  = os.path.join(HERE, "..", "Vulpecula", "Vulpecula.ino")
STUBS = os.path.join(HERE, "stubs")

# top-level function definition: "static <ret> name(args)" then { on this or next line
FN = re.compile(
    r'^(static\s+(?:inline\s+)?[A-Za-z_][\w:<>,\s\*&]*?[\*&\s])([A-Za-z_]\w*)\s*\(([^;{]*)\)\s*$'
)
FN_INLINE = re.compile(
    r'^(static\s+(?:inline\s+)?[A-Za-z_][\w:<>,\s\*&]*?[\*&\s])([A-Za-z_]\w*)\s*\(([^;{]*)\)\s*\{'
)

def main():
    lines = open(INO).read().split("\n")

    protos, first_fn = [], None
    depth = 0
    for i, ln in enumerate(lines):
        # only consider lines at brace depth 0
        if depth == 0:
            m = FN_INLINE.match(ln) or FN.match(ln)
            if m and "typedef" not in ln:
                ret, name, args = m.group(1).strip(), m.group(2), m.group(3)
                # skip the sketch entry points; the IDE does not prototype them
                if name not in ("setup", "loop"):
                    protos.append(f"{ret} {name}({args});")
                if first_fn is None:
                    first_fn = i
        depth += ln.count("{") - ln.count("}")
        if depth < 0:
            depth = 0

    if first_fn is None:
        sys.exit("found no function definitions - parser needs updating")
    print(f"first function definition: line {first_fn+1}")
    print(f"generated {len(protos)} prototypes, injecting at that point")

    injected = (lines[:first_fn]
                + ["// ---- injected by arduino_prototype_check.py ----"]
                + protos
                + ["// ---- end injected ----"]
                + lines[first_fn:])

    with tempfile.TemporaryDirectory() as td:
        src = os.path.join(td, "sk.cpp")
        open(src, "w").write("\n".join(injected))
        r = subprocess.run(
            ["g++", "-std=gnu++17", "-fsyntax-only", f"-I{STUBS}", src],
            capture_output=True, text=True)
        if r.returncode:
            print("\nFAILED - the IDE would reject this sketch:\n")
            print(r.stderr[:6000])
            sys.exit(1)
    print("\nPASS - prototype injection is safe; every type resolves.")

if __name__ == "__main__":
    main()
