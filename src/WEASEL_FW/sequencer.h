// =============================================================================
// sequencer.h — 5-step sequential voltage source (3, 4 or 5 stages)
//
// The step clock comes from the internal clock (CLOCK pot), the pulsar, MIDI
// notes or MIDI clock (midi_clock.h); audio.h decides which and calls advance().
// =============================================================================
#pragma once
#include <stdint.h>

namespace weasel
{
enum SeqTriggerMode : uint8_t
{
  SEQ_TRIGGER_CLOCK,
  SEQ_TRIGGER_PULSAR,
  SEQ_TRIGGER_MIDI
};

class Sequencer
{
 public:
  static const int kSteps = 5;

  SeqTriggerMode triggerMode = SEQ_TRIGGER_CLOCK;
  int maxSteps = 5;                 // 3, 4 or 5 stages
  int currentStep = 0;
  volatile int activeStep = 0;      // last ENABLED step reached - the step whose CV is output (drives step LEDs)
  bool stepEnabled[kSteps] = { true, true, true, true, true };
  float stepCV[kSteps] = { 0.0f };  // from the step pots, 0..1
  float activeCV = 0.0f;            // the CV of the currently active step

  // Internal clock
  float clockPeriodMs = 100.0f;     // step time set by the CLOCK pot
  uint32_t samplesPerTick = 48000;  // the same, in samples
  uint32_t sampleCounter = 0;       // counts samples towards the next step

  // Free-running clock, once a sample: true when this sample starts a step
  inline bool clockTick()
  {
    sampleCounter++;
    if (sampleCounter >= samplesPerTick)
    {
      sampleCounter = 0;
      return true;
    }
    return false;
  }

  // The clock ticked: move to the next step. Returns true if that step is
  // enabled (it fires a trigger); a disabled step is a rest.
  inline bool advance()
  {
    currentStep = (currentStep + 1) % maxSteps;
    if (stepEnabled[currentStep])
    {
      activeCV = stepCV[currentStep];
      activeStep = currentStep;  // disabled steps leave the previous step lit
      return true;
    }
    return false;
  }

  // STAGES button: 5 -> 4 -> 3 -> 5
  void cycleStages()
  {
    if (maxSteps == 5) maxSteps = 4;
    else if (maxSteps == 4) maxSteps = 3;
    else maxSteps = 5;
    currentStep %= maxSteps;
  }

  void setStages(int steps)
  {
    if (steps != maxSteps)
    {
      maxSteps = steps;
      currentStep %= maxSteps;
    }
  }
};
}  // namespace weasel
