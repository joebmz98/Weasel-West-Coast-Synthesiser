#!/bin/sh
# Measures the complex oscillator's aliasing with oversampling off (the 1.1.x code),
# 2x and 4x. Needs g++, python3 with numpy + scipy, and the DaisyDuino library.
set -e
cd "$(dirname "$0")"
. ../firmware_sim/find_libs.sh
for os in 1 2 4; do
  mkdir -p out$os
  g++ -O2 -std=gnu++17 -w -DWEASEL_COMPLEX_OVERSAMPLING=$os -I../firmware_sim/stub -I"$DAISYDUINO" -I../../src/WEASEL_FW \
    dsptest.cpp "$DAISYDUINO"/utility/DaisySP/modules/oscillator.cpp -o dsptest$os
  ./dsptest$os out$os > /dev/null
done
python3 alias.py out1 out2 out4
rm -rf out1 out2 out4 dsptest1 dsptest2 dsptest4
