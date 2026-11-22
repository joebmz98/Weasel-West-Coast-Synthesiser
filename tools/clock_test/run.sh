#!/bin/sh
# Builds and runs the MIDI clock sync test against the current sketch. Needs python3 and g++.
set -e
cd "$(dirname "$0")"
python3 extract.py ../../src/WEASEL_FW/WEASEL_FW.ino gen.h
g++ -std=gnu++17 -O2 -Wall -Wno-unused-variable clock_test.cpp -o clock_test
./clock_test
rm -f gen.h clock_test
