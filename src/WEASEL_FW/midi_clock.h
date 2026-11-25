// =============================================================================
// midi_clock.h — MIDI clock sync for the sequencer (firmware 1.1.4+)
//
// The sequencer steps by COUNTING the clock ticks received: one step every
// `division` ticks (24 a quarter note), so it stays locked to the master and
// never drifts. Before 1.1.4 it free-ran on a step length worked out from the
// gap between the last two ticks, which jumped about with main-loop timing (a
// 120 BPM clock read anywhere from ~90 to ~160).
//
// The tempo shown on the web page is averaged over the last beat (24 ticks),
// timed in us. Ticks are counted by the main loop (MIDI is read there) and
// consumed by the audio callback.
// =============================================================================
#pragma once
#include "Arduino.h"
#include "config.h"
#include "sequencer.h"

namespace weasel
{
class MidiClock
{
 public:
  static const uint32_t kTimeoutMs = 500;  // no tick for this long = clock stopped
  static const uint8_t kTempoWindow = 24;  // ticks averaged for the tempo (one beat)
  static const int kDivisionCount = 8;

  bool sync = false;        // CLOCK SYNC mode: the sequencer follows MIDI clock
  bool active = false;      // ticks are arriving (only tracked while synced)
  float tempo = 120.0f;     // received tempo in BPM, averaged over a beat (web monitor)
  uint32_t lastTickMs = 0;  // when the last tick arrived

  // Ticks per step, chosen by the CLOCK pot in sync mode
  int divisionIndex = 0;
  float division = 1.0f;

  // Main loop: a tick arrived
  WEASEL_NOINLINE void tickReceived(uint32_t nowUs, uint32_t nowMs)
  {
    // Clock (re)started after a gap: forget the old tick times
    if (lastTickMs == 0 || nowMs - lastTickMs > kTimeoutMs) tickTimeCount_ = 0;
    tickTimesUs_[tickTimeHead_] = nowUs;
    tickTimeHead_ = (tickTimeHead_ + 1) % (kTempoWindow + 1);
    if (tickTimeCount_ < kTempoWindow + 1) tickTimeCount_++;
    if (tickTimeCount_ >= 2)
    {
      uint8_t oldest = (tickTimeHead_ + (kTempoWindow + 1) - tickTimeCount_) % (kTempoWindow + 1);
      uint32_t spanUs = nowUs - tickTimesUs_[oldest];
      if (spanUs > 0)
      {
        float bpm = 60.0e6f * (float)(tickTimeCount_ - 1) / (24.0f * (float)spanUs);
        if (bpm < 20.0f) bpm = 20.0f;
        if (bpm > 400.0f) bpm = 400.0f;
        tempo = bpm;
      }
    }
    lastTickMs = nowMs;
    active = true;
    tickCount_ = tickCount_ + 1;
  }

  // Main loop: transport messages
  void start()
  {
    startTickCount_ = tickCount_;  // written before the flag the audio callback reads
    startPending_ = true;
    transportRunning_ = true;
  }
  void cont() { transportRunning_ = true; }
  void stop() { transportRunning_ = false; }
  void resync()  // sync switched on: start from step 1 on the next tick
  {
    resyncPending_ = true;
    transportRunning_ = true;
  }

  // Main loop: is a clock arriving at all (whether or not sync is on)?
  bool receiving(uint32_t nowMs) const { return lastTickMs > 0 && (nowMs - lastTickMs) <= kTimeoutMs; }

  // Main loop: drop `active` once the ticks stop
  void checkActivity(uint32_t nowMs)
  {
    if (sync && (nowMs - lastTickMs > kTimeoutMs))
    {
      active = false;
    }
#if WEASEL_DEBUG
    // Report clock starting/stopping, whether or not clock sync is switched on
    static bool debugRunning = false;
    bool running = receiving(nowMs);
    if (running != debugRunning)
    {
      debugRunning = running;
      Serial.println(running ? "MIDI clock started" : "MIDI clock stopped");
    }
#endif
  }

  // CLOCK pot (0..1) in sync mode. REVERSED: 0% = fastest, 100% = slowest.
  WEASEL_NOINLINE void updateDivision(float clockPot)
  {
    static const float kDivisions[kDivisionCount] = { 1.0f, 2.0f, 4.0f, 8.0f, 12.0f, 16.0f, 24.0f, 32.0f };
    divisionIndex = (int)((1.0f - clockPot) * kDivisionCount);
    if (divisionIndex >= kDivisionCount) divisionIndex = kDivisionCount - 1;
    if (divisionIndex < 0) divisionIndex = 0;
    division = kDivisions[divisionIndex];
  }

  // Audio callback, once a sample while synced: true when this sample starts a step
  inline bool stepDue(Sequencer& seq)
  {
    uint32_t div = (uint32_t)division;
    if (div < 1) div = 1;
    if (resyncPending_)
    {
      resyncPending_ = false;
      audioSeenTickCount_ = tickCount_;
      seqTickAccum_ = div - 1;
      seq.currentStep = seq.maxSteps - 1;  // the next step is step 1
    }
    if (startPending_)
    {
      startPending_ = false;
      audioSeenTickCount_ = startTickCount_;  // ticks after the Start count towards step 1
      seqTickAccum_ = div - 1;                // ...and the first of them plays it
      seq.currentStep = seq.maxSteps - 1;
    }
    uint32_t t = tickCount_;
    uint32_t newTicks = t - audioSeenTickCount_;
    audioSeenTickCount_ = t;
    if (!transportRunning_) return false;  // stopped: hold position
    seqTickAccum_ += newTicks;
    if (seqTickAccum_ < div) return false;
    seqTickAccum_ -= div;
    // After a long hold-up (e.g. a flash save) take one step and keep the phase, rather than a burst
    if (seqTickAccum_ >= div) seqTickAccum_ %= div;
    return true;
  }

 private:
  volatile uint32_t tickCount_ = 0;          // every tick received (main loop writes)
  volatile uint32_t startTickCount_ = 0;     // tick count when Start arrived
  volatile bool startPending_ = false;       // Start: next tick plays step 1
  volatile bool resyncPending_ = false;      // sync switched on: count from the next tick
  volatile bool transportRunning_ = true;    // false after Stop, until Start / Continue
  uint32_t tickTimesUs_[kTempoWindow + 1];
  uint8_t tickTimeHead_ = 0, tickTimeCount_ = 0;
  uint32_t audioSeenTickCount_ = 0;  // audio callback only
  uint32_t seqTickAccum_ = 0;        // audio callback only: ticks towards the next step
};
}  // namespace weasel
