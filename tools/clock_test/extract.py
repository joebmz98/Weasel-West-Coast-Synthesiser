# Pulls the MIDI clock sync block out of WEASEL_FW.ino unchanged into gen.h, with the few
# globals it uses. Run via run.sh.
import sys
src = open(sys.argv[1]).read()
a = src.index('// ---- MIDI CLOCK SYNC (firmware 1.1.4) ----')
b = src.index('// ---- END MIDI CLOCK SYNC ----')
hdr = '''#include <cstdint>
#include <cstdio>
bool midiClockActive = false;
float midiTempo = 120.0f;
uint32_t lastMidiClockTime = 0;
const uint32_t MIDI_CLOCK_TIMEOUT = 500;
float currentDivision = 6.0f;
int seqCurrentStep = 0;
int seqMaxSteps = 5;
'''
open(sys.argv[2], 'w').write(hdr + src[a:b])
