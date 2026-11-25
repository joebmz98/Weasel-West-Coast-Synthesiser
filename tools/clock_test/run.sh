#!/bin/sh
# Builds and runs the MIDI clock sync test against the firmware's midi_clock.h. Needs g++.
set -e
cd "$(dirname "$0")"
g++ -std=gnu++17 -O2 -Wall -Wno-unused-variable -I../host -I../../src/WEASEL_FW clock_test.cpp -o clock_test
./clock_test
rm -f clock_test
