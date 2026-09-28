#!/bin/sh
# All host-side checks. Needs only g++ and python3 - no hardware, no Arduino.
set -e
D=$(dirname "$0")
echo "### 1/5  host compile (ordinary type and syntax errors)"
cp "$D/../Vulpecula/Vulpecula.ino" /tmp/_vulp.cpp
g++ -std=gnu++17 -fsyntax-only -I "$D/stubs" /tmp/_vulp.cpp -Wall -Wextra
echo "    clean"
echo
echo "### 2/5  Arduino prototype injection (catches what a host compile CANNOT)"
python3 "$D/arduino_prototype_check.py"
echo
echo "### 3/5  screen layout - nothing off-panel or overlapping"
python3 "$D/layout_check.py"
echo "### 4/5  proximity gate, 34 assertions"
python3 "$D/extract_and_test.py"
echo
echo "### 5/5  triage rules, 15 assertions"
python3 "$D/triage_extract_and_test.py"
