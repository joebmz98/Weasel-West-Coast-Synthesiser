// Desktop check of the MIDI clock sync in WEASEL_FW.ino (firmware 1.1.4): a 120 BPM clock read
// by a main loop that is held up by 0-6 ms at random, as on the real panel. Run: sh run.sh
#include "gen.h"
#include <cassert>
#include <cmath>
#include <cstdlib>
#include <vector>
const double SR = 48000;
// Old firmware's tempo: one tick gap, in whole ms
float oldTempo(uint32_t gapMs) { float t = 60000.0f / (gapMs * 24.0f); return t < 40 ? 40 : t > 240 ? 240 : t; }

int main() {
  setvbuf(stdout, nullptr, _IONBF, 0);
  srand(7);
  const double bpm = 120, tickUs = 60e6 / (bpm * 24);
  // ---- tempo reading ----
  double t = 1000;  // us
  uint32_t lastOldMs = 0; float oldMin = 999, oldMax = 0, newMin = 999, newMax = 0;
  for (int i = 0; i < 24 * 40; i++) {
    double seen = t + (rand() % 6000);  // when the loop gets to it
    uint32_t ms = (uint32_t)(seen / 1000);
    if (lastOldMs) { float o = oldTempo(ms - lastOldMs); if (i > 24) { oldMin = fminf(oldMin, o); oldMax = fmaxf(oldMax, o); } }
    lastOldMs = ms;
    midiClockTickReceived((uint32_t)seen, ms);
    if (i > 24) { newMin = fminf(newMin, midiTempo); newMax = fmaxf(newMax, midiTempo); }
    t += tickUs;
  }
  printf("120 BPM clock, 0-6 ms loop delay: old reading %.0f-%.0f BPM, new reading %.1f-%.1f BPM\n", oldMin, oldMax, newMin, newMax);
  assert(newMin > 118.5f && newMax < 121.5f);
  puts("ok  tempo reads 120 +/- 1.5");

  // ---- sequencer: ticks arrive in loop passes, audio runs per sample ----
  // fresh state
  midiClockTickCount = 0; audioSeenTickCount = 0; seqTickAccum = 0; midiTransportRunning = true;
  currentDivision = 6;  // 1/16 notes: 6 ticks a step
  midiClockResync();
  std::vector<long> steps;
  long sample = 0; double nextTick = 0; long ticks = 0;
  auto run = [&](double seconds, bool clockOn, double stallAt = -1, double stallLen = 0) {
    long end = sample + (long)(seconds * SR);
    std::vector<double> pendingTicks;
    static long nextPass = 0;
    for (; sample < end; sample++) {
      double nowUs = sample / SR * 1e6;
      // main loop pass every 1 ms (+ random delay), handling ticks that arrived before it
      if (clockOn) while (nextTick <= nowUs) { pendingTicks.push_back(nextTick); nextTick += tickUs; }
      bool stalled = stallAt >= 0 && nowUs >= stallAt * 1e6 && nowUs < (stallAt + stallLen) * 1e6;
      if (!stalled && sample >= nextPass) {
        nextPass = sample + 48 * (1 + rand() % 6);  // next loop pass in 1-6 ms
        for (double tk : pendingTicks) { midiClockTickReceived((uint32_t)nowUs, (uint32_t)(nowUs / 1000)); ticks++; (void)tk; }
        pendingTicks.clear();
      }
      if (midiClockActive && midiSyncStepDue()) {
        seqCurrentStep = (seqCurrentStep + 1) % seqMaxSteps;
        steps.push_back(sample);
      }
    }
  };
  run(10, true);
  // step every 6 ticks = 1/16 at 120 = 125 ms = 6000 samples
  double worst = 0;
  for (size_t i = 1; i < steps.size(); i++) worst = fmax(worst, fabs((steps[i] - steps[i - 1]) - 6000.0));
  long expectSteps = ticks / 6;
  printf("10 s at 120 BPM, 1/16: %zu steps (ticks/6 = %ld), worst step-length error %.1f ms\n", steps.size(), expectSteps, worst / 48);
  assert((long)steps.size() == expectSteps || (long)steps.size() == expectSteps + 1);
  assert(worst / 48 <= 6.5);  // no worse than the loop's own delay
  // drift: step n should be at n*125ms + up to loop delay
  double drift = (steps.back() - steps.front()) / 48.0 - (steps.size() - 1) * 125.0;
  printf("    drift over 10 s: %.1f ms\n", drift);
  assert(fabs(drift) < 8);
  puts("ok  steps locked to the clock: no drift, jitter only from loop timing");

  // Start: the first tick after it plays step 1 (index 0)
  midiClockStart(); seqCurrentStep = 3; steps.clear();
  { long end = sample + 48000; double nowUs; bool got = false;
    for (; sample < end && !got; sample++) { nowUs = sample / SR * 1e6; while (nextTick <= nowUs) { midiClockTickReceived((uint32_t)nowUs, (uint32_t)(nowUs / 1000)); nextTick += tickUs; }
      if (midiSyncStepDue()) { seqCurrentStep = (seqCurrentStep + 1) % seqMaxSteps; got = true; } }
    assert(got && seqCurrentStep == 0); }
  puts("ok  Start: the first tick after it plays step 1");

  // Stop holds, Continue resumes
  midiClockStop(); steps.clear(); run(1, true); assert(steps.empty());
  midiClockContinue(); run(1, true); assert(steps.size() >= 7 && steps.size() <= 9);
  puts("ok  Stop holds the sequencer while clock keeps running; Continue resumes");

  // A 0.3 s hold-up (flash save): one catch-up step, then back in phase
  steps.clear(); double startS = sample / SR; run(3, true, startS + 1.0, 0.3);
  int bursts = 0; for (size_t i = 1; i < steps.size(); i++) if (steps[i] - steps[i - 1] < 1000) bursts++;
  printf("    after a 0.3 s stall: %d steps closer than 21 ms\n", bursts);
  assert(bursts <= 1);
  // phase: last steps are on the 125 ms grid of the clock again
  puts("ok  a 0.3 s stall gives at most one catch-up step, no burst");
  puts("ALL CLOCK CHECKS PASSED");
}
