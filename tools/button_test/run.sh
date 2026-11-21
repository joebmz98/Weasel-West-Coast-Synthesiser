#!/bin/sh
# Builds and runs the button test against the current sketch. Needs python3 and g++.
set -e
cd "$(dirname "$0")"
python3 extract.py ../../src/WEASEL_FW/WEASEL_FW.ino gen.h
g++ -std=gnu++17 -Wall -Wno-unused-variable button_test.cpp -o button_test
./button_test
rm -f gen.h button_test
