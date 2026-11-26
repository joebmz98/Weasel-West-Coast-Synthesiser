#!/bin/sh
# Runs the real firmware (setup, loop, audio callback) on this computer against a
# model of the panel, for scripted sessions, and compares it with another version.
#
#   sh tools/firmware_sim/run.sh <reference sketch folder> [compiler flags]
#
# e.g. after `git worktree add /tmp/w115 <1.1.5 commit>`:
#   sh tools/firmware_sim/run.sh /tmp/w115/src/WEASEL_FW -DWEASEL_COMPLEX_OVERSAMPLING=1
# Prints, for each session, whether the audio and every LED write are identical.
# Needs g++, python3 + numpy, and the DaisyDuino and MIDI libraries.
set -e
cd "$(dirname "$0")"
. ./find_libs.sh
REF="$1"; shift || true
mkdir -p out
build() {  # <sketch dir> <binary> [flags]
  sk="$1"; bin="$2"; shift 2
  printf '#include "Arduino.h"\n#include "%s/WEASEL_FW.ino"\n' "$sk" > out/$bin.sketch.cpp
  g++ -O2 -std=gnu++17 -w -ffp-contract=off "$@" -Istub -I"$DAISYDUINO" -I"$MIDILIB" -I"$sk" \
    sim.cpp out/$bin.sketch.cpp $(ls "$sk"/*.cpp 2>/dev/null) "$DAISYDUINO"/utility/DaisySP/modules/*.cpp -o out/$bin
}
build "$(cd ../../src/WEASEL_FW && pwd)" sim_new "$@"
build "$(cd "$REF" && pwd)" sim_ref
python3 make_scenarios.py out
for s in out/*.sc; do n=$(basename $s .sc); out/sim_ref $s out/$n.ref.f32 out/$n.ref.led; out/sim_new $s out/$n.new.f32 out/$n.new.led; done
python3 compare.py out
