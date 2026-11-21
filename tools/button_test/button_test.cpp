// Desktop check of the panel buttons in WEASEL_FW.ino: act on release, the MIDI shift
// (hold 1 s, LPG 1 / LPG 2 pick which oscillators follow MIDI pitch), boot hold. Run: sh run.sh
#include "gen.h"
#include <cassert>
// step time 10ms, two reads to debounce
void scan(uint32_t raw, int ms=10){ fakeRaw=raw; for(int t=0;t<ms;t+=10){ fakeMs+=10; readButtons(); } }
const uint32_t M=1UL<<BTN_QUANTIZE_MIDI, L1=1UL<<BTN_LPG1_MODE, L2=1UL<<BTN_LPG2_MODE, PU=1UL<<BTN_PULSAR_MANUAL, ENV=1UL<<BTN_ENV_MODE, S1=1UL<<BTN_SEQ_STEP1;
// latching step switches read LOW = enabled; keep them all 'enabled' (low)
int main(){
  // boot with pulsar held: nothing fires, on press or release
  fakeRaw=PU; readButtons(); readButtons(); buttonsArmed=true;
  scan(0,30); assert(actions.empty()); puts("ok  held at boot: no action on release");
  // plain env press: acts on release, not press
  scan(ENV,50); assert(actions.empty()); scan(0,30); assert(actions.size()==1 && actions[0]==BTN_ENV_MODE); actions.clear(); puts("ok  mode button acts on release");
  // pulsar manual: on press
  scan(PU,30); assert(actions.size()==1); scan(0,30); assert(actions.size()==1); actions.clear(); puts("ok  pulsar manual fires on press only");
  // short MIDI press/release toggles
  scan(M,300); scan(0,30); assert(actions.size()==1 && actions[0]==BTN_QUANTIZE_MIDI); actions.clear(); puts("ok  short MIDI press+release toggles MIDI");
  // hold MIDI 1.2s -> shift active; release without pressing: no toggle, no change
  scan(M,1200); assert(shiftActive); scan(0,30); assert(actions.empty() && editor.saves==0 && !shiftActive); puts("ok  long hold then release: no MIDI toggle, nothing changes");
  // hold, press LPG1 (excluded), LPG2 twice (back), release
  scan(M,1100); assert(shiftActive);
  scan(M|L1,50); scan(M,50); assert(shiftPitchExclude==1 && editor.px==0);
  scan(M|L2,50); scan(M,50); scan(M|L2,50); scan(M,50); assert(shiftPitchExclude==1);
  assert(actions.empty());
  scan(0,30); assert(editor.px==1 && editor.saves==1 && actions.empty()); puts("ok  shift: LPG1 excludes complex osc, applied only on MIDI release, no LPG mode change, no MIDI toggle");
  // LPG pressed and released after MIDI released? hold MIDI, press LPG1 and keep it while releasing MIDI, then release LPG1
  scan(M,1100); scan(M|L1,50); scan(L1,50); assert(editor.px==0); scan(0,50); assert(actions.empty()); puts("ok  LPG released after MIDI: still no LPG mode change");
  // press LPG before the shift is active: ignored, and MIDI does not toggle
  scan(M,300); scan(M|L1,50); scan(M,50); scan(0,30); assert(actions.empty() && editor.px==0); puts("ok  LPG pressed before 1 s: ignored, MIDI not toggled");
  // other buttons during shift ignored
  scan(M,1100); scan(M|ENV,50); scan(M,50); scan(0,30); assert(actions.empty()); puts("ok  other buttons ignored during the shift");
  // latching step switch flicked while holding MIDI does not cancel the toggle
  scan(M,200); scan(M|S1,50); scan(M,50); scan(0,30); assert(actions.size()==1 && actions[0]==BTN_QUANTIZE_MIDI); actions.clear(); puts("ok  step switches don't count as a combo");
  // a button already held before MIDI goes down still acts on its own release
  scan(ENV,50); scan(ENV|M,50); scan(M,50); assert(actions.size()==1 && actions[0]==BTN_ENV_MODE); scan(0,30); actions.clear(); puts("ok  a button held before MIDI is unaffected");
  puts("ALL BUTTON CHECKS PASSED");
}
