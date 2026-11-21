# Pulls the button code out of WEASEL_FW.ino unchanged (constants, state and readButtons())
# into gen.h, with millis(), the shift registers, the editor and handleButtonPress() faked.
import sys
src = open(sys.argv[1]).read()
hdr = '''#include <cstdint>
#include <cstdio>
#include <vector>
unsigned long fakeMs = 0; unsigned long millis() { return fakeMs; }
uint32_t fakeRaw = 0; uint32_t readAllShiftRegisters() { return fakeRaw; }
namespace weasel { const uint8_t PITCH_EXCLUDE_COMPLEX = 1, PITCH_EXCLUDE_MODULATION = 2; }
struct Ed { uint8_t px = 0; int saves = 0; uint8_t pitchExclude() const { return px; } void setPitchExclude(uint8_t m) { px = m; saves++; } } editor;
std::vector<int> actions;
void handleButtonPress(uint8_t bit) { actions.push_back(bit); }
bool seqStepEnabled[5];
'''
a = src.index('#define SR_BIT')
c = src.index('uint32_t buttonRawPrev = 0;')
d = src.index('// ============================================', c)
f = src.index('void readButtons() {')
g = src.index('// ============================================\n// PATCH BAY MODULATION')
open(sys.argv[2], 'w').write(hdr + src[a:d] + '\ninline bool buttonIsDown(uint8_t bit) { return (buttonDebounced >> bit) & 1UL; }\n' + src[f:g])
