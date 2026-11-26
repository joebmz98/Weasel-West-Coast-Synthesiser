// =============================================================================
// envelopes.h — envelope generator (Buchla ASD/ASR), pulsar, random voltage
//
// Each module owns its mode and state; audio.h works out each sample's
// triggers (sequencer, pulsar, MIDI) and calls process().
// =============================================================================
#pragma once
#include <stdlib.h>
#include "DaisyDuino.h"

namespace weasel
{
// -----------------------------------------------------------------------------
// ENVELOPE GENERATOR
// -----------------------------------------------------------------------------
enum EnvelopeMode : uint8_t
{
  ENV_MODE_TRANSIENT,  // one-shot: the gate is a fixed pulse (SUSTAIN pot), cannot be held
  ENV_MODE_SUSTAIN,    // gate held for the SUSTAIN pot time, or while the MIDI note is held
  ENV_MODE_OSCILLATE   // loops on its own
};

enum EnvTriggerMode : uint8_t
{
  ENV_TRIGGER_SEQ,     // sequencer steps (default)
  ENV_TRIGGER_PULSAR,  // pulsar trigger
  ENV_TRIGGER_MIDI     // MIDI note on
};

class EnvelopeGenerator
{
 public:
  EnvelopeMode mode = ENV_MODE_TRANSIENT;
  EnvTriggerMode triggerMode = ENV_TRIGGER_SEQ;

  // Set from the pots (controls.h)
  float attackTime = 0.0f;
  float releaseTime = 0.0f;
  float sustainDuration = 0.0f;
  uint32_t transientGateSamples = 0;
  uint32_t sustainGateSamples = 0;

  // Remaining gate of the current note. A rest on the sequencer clears it.
  uint32_t gateRemainingSamples = 0;

  daisysp::Adsr adsr;

  void init(float sampleRate)
  {
    sampleRate_ = sampleRate;
    adsr.Init(sampleRate);
    gateRemainingSamples = 0;
  }

  void cycleMode() { mode = static_cast<EnvelopeMode>((mode + 1) % 3); }
  void cycleTrigger() { triggerMode = static_cast<EnvTriggerMode>((triggerMode + 1) % 3); }

  // One sample. `midiNoteHeld` holds the gate open in SUSTAIN mode with MIDI triggering.
  inline float process(bool trigger, bool midiNoteHeld)
  {
    bool gate = false;

    if (mode == ENV_MODE_TRANSIENT)
    {
      if (trigger)
      {
        gateRemainingSamples = transientGateSamples;
        adsr.Retrigger(true);
      }
      if (gateRemainingSamples > 0)
      {
        gate = true;
        gateRemainingSamples--;
      }
    }
    else if (mode == ENV_MODE_SUSTAIN)
    {
      if (trigger)
      {
        adsr.Retrigger(true);
        gateRemainingSamples = sustainGateSamples;
      }

      if (triggerMode == ENV_TRIGGER_MIDI)
      {
        // Gate stays open while the MIDI note is held
        gate = midiNoteHeld;
        gateRemainingSamples = 0;
      }
      else if (gateRemainingSamples > 0)
      {
        gate = true;
        gateRemainingSamples--;
      }
    }
    else if (mode == ENV_MODE_OSCILLATE)
    {
      const float kSpeedMultiplier = 0.5f;

      if (!adsr.IsRunning() && oscGateSamples_ == 0)
      {
        adsr.Retrigger(true);
        uint32_t totalCycleSamples = (uint32_t)((attackTime + sustainDuration + releaseTime) * sampleRate_ * kSpeedMultiplier);
        oscGateSamples_ = totalCycleSamples;
      }

      if (oscGateSamples_ > 0)
      {
        gate = true;
        oscGateSamples_--;
      }
      else
      {
        gate = false;
      }
    }

    return adsr.Process(gate);
  }

 private:
  float sampleRate_ = 48000.0f;
  uint32_t oscGateSamples_ = 0;
};

// -----------------------------------------------------------------------------
// PULSAR — a short AD burst; free-running it becomes a second clock / LFO
// -----------------------------------------------------------------------------
enum PulsarMode : uint8_t
{
  PULSAR_MODE_SEQ,   // fires on each sequencer step
  PULSAR_MODE_MIDI,  // fires on each MIDI note
  PULSAR_MODE_OSC    // retriggers itself as soon as it finishes
};

class Pulsar
{
 public:
  static constexpr float kMinTime = 0.02f;
  static constexpr float kMaxTime = 5.0f;

  PulsarMode mode = PULSAR_MODE_SEQ;
  float releaseTime = 0.02f;             // PERIOD pot, shortened by the patch bay
  float periodModCoeff = 0.0f;           // patch-bay modulation of the period (audio callback)
  volatile bool manualTrigger = false;   // MANUAL button: fires on the next sample
  volatile bool running = false;         // mirrored from the audio callback for the activity LED
  bool lastActive = false;               // for the sequencer's PULSAR trigger mode

  daisysp::Adsr adsr;

  void init(float sampleRate)
  {
    adsr.Init(sampleRate);
    adsr.SetAttackTime(0.02f);
    adsr.SetDecayTime(0.02f);
    adsr.SetSustainLevel(1.0f);
  }

  void cycleMode() { mode = static_cast<PulsarMode>((mode + 1) % 3); }

  inline bool isRunning() { return adsr.IsRunning(); }

  inline void fire()
  {
    adsr.SetAttackTime(0.02f);
    adsr.SetDecayTime(0.02f);
    adsr.SetSustainLevel(0.0f);
    adsr.SetReleaseTime(releaseTime);
    adsr.Retrigger(true);
  }

  inline float process(bool trigger) { return adsr.Process(trigger); }

  // PERIOD pot (0..1) with the patch-bay modulation applied
  void setPeriod(float pot)
  {
    float base = kMinTime * powf(kMaxTime / kMinTime, pot);
    float adjusted = base - (2.0f * periodModCoeff);
    if (adjusted < kMinTime) adjusted = kMinTime;
    if (adjusted > kMaxTime) adjusted = kMaxTime;
    releaseTime = adjusted;
    adsr.SetReleaseTime(releaseTime);
  }
};

// -----------------------------------------------------------------------------
// RANDOM VOLTAGE — sample & hold of a random 0..1 value
// -----------------------------------------------------------------------------
enum RandomMode : uint8_t
{
  RANDOM_MODE_SEQ,
  RANDOM_MODE_PULSAR,
  RANDOM_MODE_MIDI
};

class RandomVoltage
{
 public:
  RandomMode mode = RANDOM_MODE_SEQ;
  float value = 0.0f;

  void cycleMode() { mode = static_cast<RandomMode>((mode + 1) % 3); }
  inline void sample() { value = (float)rand() * (1.0f / RAND_MAX); }
};
}  // namespace weasel
