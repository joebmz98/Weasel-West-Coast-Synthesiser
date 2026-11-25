// =============================================================================
// synth.h — every module of the instrument, gathered in one struct
//
// There is one Synth, `synth`, defined in WEASEL_FW.ino. The panel code
// (controls.h, panel.h, midi_input.h) and the audio callback (audio.h) work on
// it. Writers and readers:
//   main loop  : pots, buttons, patch bay, LEDs, MIDI -> controls, modes, triggers
//   audio IRQ  : reads controls / modes, runs the sequencer, envelopes and voice
// 32-bit loads and stores are atomic on the Cortex-M7, so single values need no
// locking; flags passed from the main loop to the audio callback are volatile.
// =============================================================================
#pragma once
#include "DaisyDuino.h"
#include "config.h"
#include "hardware.h"
#include "smoothing.h"
#include "pots.h"
#include "shift_registers.h"
#include "patch_bay.h"
#include "leds.h"
#include "sequencer.h"
#include "midi_clock.h"
#include "midi_notes.h"
#include "envelopes.h"
#include "oscillators.h"
#include "lpg_channel.h"

namespace weasel
{
// Values worked out from the pots by controls.h (main loop). The ones the audio
// callback uses directly go through `smooth` first (see Smoothed).
struct Controls
{
  // Complex oscillator
  float compPitchOct = 8.0f;      // FREQ pot, log2 Hz (smoothed in octaves: even glides)
  float compFine = 0.0f;          // FINE pot, octaves (+/- 3 semitones)
  float compTimbre = 0.0f;        // TIMBRE pot: sine .. selected waveform
  float compFold = 0.0f;          // WAVEFOLD pot: folder gain 1..20
  float compFreqModDepth = 0.0f;  // patch-bay attenuators
  float compFoldModDepth = 0.0f;
  // Modulation oscillator
  float modPitchOct = 5.0f;
  float modFine = 0.0f;
  float modDepth = 0.0f;          // MOD pot: FM / AM depth applied to the complex oscillator
  float modFreqModDepth = 0.0f;
  float modDepthModDepth = 0.0f;
  // Low pass gates
  float lpg1Level = 0.0f;
  float lpg1ModDepth = 0.0f;
  float lpg2Level = 0.0f;
  float lpg2ModDepth = 0.0f;
  // Output
  float reverbMix = 0.0f;
  float masterVolume = 0.0f;      // 0..1, already squared (audio taper)
};

// Smoothed copies of Controls, in this order
enum Smoothed : uint8_t
{
  SM_COMP_PITCH, SM_COMP_FINE, SM_MOD_PITCH, SM_MOD_FINE,
  SM_COMP_TIMBRE, SM_COMP_FOLD, SM_MOD_DEPTH, SM_REVERB,
  SM_LPG1_LEVEL, SM_LPG2_LEVEL, SM_LPG1_MOD, SM_LPG2_MOD,
  SM_COMP_FREQ_DEPTH, SM_COMP_FOLD_DEPTH, SM_MOD_FREQ_DEPTH, SM_MOD_DEPTH_DEPTH,
  SM_COUNT
};

struct Synth
{
  float sampleRate = 48000.0f;

  // Panel hardware
  Pots pots;
  ShiftRegisters shiftRegisters;
  PatchBay patchBay;
  Leds leds;

  // Pot values and their audio-rate smoothing
  Controls controls;
  SmoothedParams<SM_COUNT> smooth;

  // Modules
  Sequencer seq;
  MidiClock midiClock;
  MidiNotes midi;
  EnvelopeGenerator env;
  Pulsar pulsar;
  RandomVoltage random;
  ModulationOscillator modOsc;
  ComplexOscillator compOsc;
  LpgChannel lpg1;  // complex oscillator
  LpgChannel lpg2;  // modulation oscillator
  daisysp::MoogLadder outputFilter;  // "analogue" output filter
  daisysp::ReverbSc reverb;

  // Output
  float masterVolumeSmoothed = 0.0f;  // per-sample smoothed (boots silent, fades in)

  // All DSP objects (call once, before audio starts)
  WEASEL_NOINLINE void initDsp(float sr)
  {
    const float* sources[SM_COUNT] = {
      &controls.compPitchOct, &controls.compFine, &controls.modPitchOct, &controls.modFine,
      &controls.compTimbre, &controls.compFold, &controls.modDepth, &controls.reverbMix,
      &controls.lpg1Level, &controls.lpg2Level, &controls.lpg1ModDepth, &controls.lpg2ModDepth,
      &controls.compFreqModDepth, &controls.compFoldModDepth, &controls.modFreqModDepth, &controls.modDepthModDepth,
    };
    for (int i = 0; i < SM_COUNT; i++) smooth.p[i] = { sources[i], 0, 0, 0, 0 };

    sampleRate = sr;
    smooth.init(sr);
    compOsc.init(sr);
    modOsc.init(sr);
    lpg1.init(sr);
    lpg2.init(sr);
    env.init(sr);
    pulsar.init(sr);
    outputFilter.Init(sr);
    outputFilter.SetFreq(8000.0f);
    outputFilter.SetRes(0.2f);
    reverb.Init(sr);
    reverb.SetFeedback(0.85f);
    reverb.SetLpFreq(17000.0f);
  }
};

extern Synth synth;
}  // namespace weasel
