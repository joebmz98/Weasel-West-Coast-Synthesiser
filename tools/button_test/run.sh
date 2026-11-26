#!/bin/sh
# Builds and runs the button test against the firmware's buttons.h. Needs g++.
set -e
cd "$(dirname "$0")"
g++ -std=gnu++17 -Wall -Wno-unused-variable -I../host -I../../src/WEASEL_FW button_test.cpp -o button_test
./button_test
rm -f button_test
