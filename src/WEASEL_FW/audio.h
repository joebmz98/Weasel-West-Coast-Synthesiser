// =============================================================================
// audio.h — the audio callback (48 kHz, 64-sample blocks)
//
// Per sample:
//   1. sequencer clock (internal / pulsar / MIDI note / MIDI clock) and step
//   2. pulsar trigger      3. random voltage trigger      4. envelope
//   5. patch bay: sources -> destinations      6. attenuators (CTRL pots)
//   7. modulation oscillator -> FM / AM      8. complex oscillator, wavefolder, AM
//   9. two Buchla 292 low pass gates -> mix -> soft knee
//  10. output filter      11. reverb, master volume -> out
//
// Signal flow:
//
//   MOD OSC ──┬──────────────────────────────────► LPG 2 ──┐
//             └─ FM / AM ─┐                                ├─ mix ─► filter ─► reverb ─► OUT
//   COMPLEX OSC (sine ⇄ wave) ─► FOLD ─► AM ─────► LPG 1 ──┘
// =============================================================================
#pragma once
#include "DaisyDuino.h"
#include "dsp_util.h"
#include "synth.h"
#include "WeaselEditor.h"

namespace weasel
{
extern Editor editor;

const float kMixHeadroom = 0.5f;     // the two LPG channels are summed at half level
const float kOutputHeadroom = 0.8f;  // doubled from 0.4 to make up for kMixHeadroom

// Patch bay: each patched source adds its signal to the destination.
// The pulsar is scaled up (it's a short burst, so it needs more to be heard).
inline void patchModulation(Synth& s, float envSig, float pulsarSig, float* mod)
{
  for (int i = 0; i < kPatchDestCount; i++) mod[i] = 0.0f;

  for (int src = 0; src < kPatchSourceCount; src++)
  {
    float v = 0.0f;
    switch (src)
    {
      case 0: v = s.seq.activeCV; break;
      case 1: v = envSig; break;
      case 2: v = pulsarSig; break;
      case 3: v = s.random.value; break;
    }

    float scaleFactor = (src == 2) ? 100.0f : 1.0f;
    float freqScaleFactor = (src == 2) ? 200.0f : 1.0f;

    if (s.patchBay.connected[src][DEST_PULSAR_PERIOD]) mod[DEST_PULSAR_PERIOD] += v * scaleFactor;
    if (s.patchBay.connected[src][DEST_MODOSC_FREQ]) mod[DEST_MODOSC_FREQ] += v * freqScaleFactor;
    if (s.patchBay.connected[src][DEST_MODOSC_MOD]) mod[DEST_MODOSC_MOD] += v * scaleFactor;
    if (s.patchBay.connected[src][DEST_COMP_PITCH]) mod[DEST_COMP_PITCH] += v * freqScaleFactor;
    if (s.patchBay.connected[src][DEST_COMP_WAVE]) mod[DEST_COMP_WAVE] += v * scaleFactor;
    if (s.patchBay.connected[src][DEST_LPG1_LEVEL]) mod[DEST_LPG1_LEVEL] += v * freqScaleFactor;
    if (s.patchBay.connected[src][DEST_LPG2_LEVEL]) mod[DEST_LPG2_LEVEL] += v * freqScaleFactor;
  }
}

inline void audioCallback(float** in, float** out, size_t size)
{
  Synth& s = synth;
  const float sr = s.sampleRate;
  s.smooth.beginBlock();  // pick up any new pot values for this block

  // Which oscillators follow the MIDI note's pitch (web editor / MIDI shift + LPG buttons)
  const uint8_t pitchExclude = editor.pitchExclude();
  const bool compFollowsMidi = !(pitchExclude & PITCH_EXCLUDE_COMPLEX);
  const bool modFollowsMidi = !(pitchExclude & PITCH_EXCLUDE_MODULATION);

  // A note-on sets every module's MIDI trigger flag, but only modules in MIDI mode use
  // theirs. Clear the rest so switching a module to MIDI later doesn't fire one stale trigger.
  if (s.seq.triggerMode != SEQ_TRIGGER_MIDI) s.midi.seqTrigger = false;
  if (s.env.triggerMode != ENV_TRIGGER_MIDI) s.midi.envTrigger = false;
  if (s.pulsar.mode != PULSAR_MODE_MIDI) s.midi.pulsarTrigger = false;
  if (s.random.mode != RANDOM_MODE_MIDI) s.midi.randomTrigger = false;

  for (size_t i = 0; i < size; i++)
  {
    // --- 1. SEQUENCER ---
    // stepTriggered = the sequencer clock ticked and the sequencer advanced.
    // stepFired     = the step it landed on is enabled, so it sends a trigger.
    // Everything the sequencer triggers (EG, pulsar, random in SEQ mode) uses
    // stepFired, so a disabled step is a true rest: no CV change and no trigger.
    bool stepTriggered = false;

    if (s.seq.triggerMode == SEQ_TRIGGER_CLOCK)
    {
      if (s.midiClock.sync && s.midiClock.active)
      {
        stepTriggered = s.midiClock.stepDue(s.seq);  // a step every `division` ticks received
      }
      else if (s.midiClock.sync && !s.midiClock.active)
      {
        // Sync on but no clock arriving: the sequencer stops
      }
      else
      {
        stepTriggered = s.seq.clockTick();  // free-running clock
      }
    }
    else if (s.seq.triggerMode == SEQ_TRIGGER_PULSAR)
    {
      bool pulsarActive = s.pulsar.isRunning();
      if (pulsarActive && !s.pulsar.lastActive) stepTriggered = true;
      s.pulsar.lastActive = pulsarActive;
    }
    else if (s.seq.triggerMode == SEQ_TRIGGER_MIDI)
    {
      if (s.midi.seqTrigger)
      {
        stepTriggered = true;
        s.midi.seqTrigger = false;
      }
    }

    bool stepFired = false;
    if (stepTriggered)
    {
      if (s.seq.advance())
      {
        stepFired = true;
      }
      else if (s.env.triggerMode == ENV_TRIGGER_SEQ)
      {
        // A disabled step is a rest, so it ends the envelope's gate - but only when the EG
        // follows the sequencer. Otherwise it cut MIDI/pulsar-triggered notes short at random.
        s.env.gateRemainingSamples = 0;
      }
    }

    // --- 2. PULSAR TRIGGER ---
    bool pulsarTrigger = false;
    if (s.pulsar.manualTrigger)
    {
      pulsarTrigger = true;
      s.pulsar.manualTrigger = false;
    }
    else if (s.pulsar.mode == PULSAR_MODE_SEQ)
    {
      pulsarTrigger = stepFired;
    }
    else if (s.pulsar.mode == PULSAR_MODE_OSC)
    {
      if (!s.pulsar.isRunning()) pulsarTrigger = true;
    }
    else if (s.pulsar.mode == PULSAR_MODE_MIDI)
    {
      if (s.midi.pulsarTrigger)
      {
        pulsarTrigger = true;
        s.midi.pulsarTrigger = false;
      }
    }
    if (pulsarTrigger) s.pulsar.fire();

    // --- 3. RANDOM VOLTAGE ---
    bool randomTrigger = false;
    if (s.random.mode == RANDOM_MODE_SEQ)
    {
      randomTrigger = stepFired;
    }
    else if (s.random.mode == RANDOM_MODE_PULSAR)
    {
      randomTrigger = pulsarTrigger;
    }
    else if (s.random.mode == RANDOM_MODE_MIDI)
    {
      if (s.midi.randomTrigger)
      {
        randomTrigger = true;
        s.midi.randomTrigger = false;
      }
    }
    if (randomTrigger) s.random.sample();

    // --- 4. ENVELOPE ---
    bool envTrigger = false;
    if (s.env.triggerMode == ENV_TRIGGER_SEQ)
    {
      if (stepFired) envTrigger = true;
    }
    else if (s.env.triggerMode == ENV_TRIGGER_MIDI)
    {
      if (s.midi.envTrigger)
      {
        envTrigger = true;
        s.midi.envTrigger = false;
      }
    }
    else if (s.env.triggerMode == ENV_TRIGGER_PULSAR)
    {
      if (pulsarTrigger) envTrigger = true;
    }

    float envSig = s.env.process(envTrigger, s.midi.noteHeld);
    float pulsarSig = s.pulsar.process(pulsarTrigger);

    // --- 5. PATCH BAY (every sample - no staircase) ---
    float mod[kPatchDestCount];
    patchModulation(s, envSig, pulsarSig, mod);

    // Smoothed pot values for this sample
    s.smooth.tick();
    const SmoothedParams<SM_COUNT>& sm = s.smooth;

    // --- 6. ATTENUATORS (the CTRL pots) ---
    s.pulsar.periodModCoeff = mod[DEST_PULSAR_PERIOD] * s.pots.norm(POT_PULSAR_CTRL);
    float modFreqCoeff = mod[DEST_MODOSC_FREQ] * sm[SM_MOD_FREQ_DEPTH];
    float modDepthCoeff = mod[DEST_MODOSC_MOD] * sm[SM_MOD_DEPTH_DEPTH];
    float compFreqCoeff = mod[DEST_COMP_PITCH] * sm[SM_COMP_FREQ_DEPTH];
    float compFoldCoeff = mod[DEST_COMP_WAVE] * sm[SM_COMP_FOLD_DEPTH];
    float lpg1Mod = mod[DEST_LPG1_LEVEL] * sm[SM_LPG1_MOD];
    float lpg2Mod = mod[DEST_LPG2_LEVEL] * sm[SM_LPG2_MOD];

    // --- 7. MODULATION OSCILLATOR ---
    float modExp = (modFreqCoeff * 5.0f) + sm[SM_MOD_FINE];
    float modFreq = pitchExp2(sm[SM_MOD_PITCH] + modExp);
    if (s.midi.follow && s.midi.noteReceived && modFollowsMidi)
    {
      modFreq *= s.midi.noteRatio;  // pot frequency is the C4 reference, transposed by the note
    }
    float modSig = s.modOsc.process(clampOscFreq(modFreq, sr));

    float modDepth = sm[SM_MOD_DEPTH] + (modDepthCoeff * 2.0f);
    if (modDepth < 0.0f) modDepth = 0.0f;
    if (modDepth > 2.0f) modDepth = 2.0f;

    const bool am = s.modOsc.amplitudeMod;
    float fmSignal = am ? 0.0f : modSig * modDepth;
    // AM normalised so full depth doesn't drive the LPG up to 3x over level
    float amGain = am ? (1.0f + (modSig * modDepth)) / (1.0f + modDepth) : 1.0f;

    // --- 8. COMPLEX OSCILLATOR + WAVEFOLDER + AM ---
    float compExp = (compFreqCoeff + fmSignal) * 5.0f + sm[SM_COMP_FINE];
    float compFreq = pitchExp2(sm[SM_COMP_PITCH] + compExp);
    if (s.midi.follow && s.midi.noteReceived && compFollowsMidi) compFreq *= s.midi.noteRatio;
    compFreq = clampOscFreq(compFreq, sr);

    float foldGain = sm[SM_COMP_FOLD] + (compFoldCoeff * 20.0f);
    if (foldGain < 1.0f) foldGain = 1.0f;
    if (foldGain > 20.0f) foldGain = 20.0f;

    float compSig = s.compOsc.process(compFreq, sm[SM_COMP_TIMBRE], foldGain, amGain);

    // --- 9. BUCHLA LPG MIXER ---
    float lpgOut1 = s.lpg1.process(compSig, sm[SM_LPG1_LEVEL] + lpg1Mod);
    float lpgOut2 = s.lpg2.process(clamp(modSig, -1.0f, 1.0f), sm[SM_LPG2_LEVEL] + lpg2Mod);

    // Sum at half level so two full channels don't clip, then a soft knee as a safety net
    float mix = (lpgOut1 + lpgOut2) * kMixHeadroom;
    if (isBadFloat(mix)) mix = 0.0f;
    mix = softKnee(mix);

    // --- 10. OUTPUT FILTER ---
    mix = s.outputFilter.Process(mix);

    // --- 11. REVERB, MASTER VOLUME & OUTPUT ---
    float revL, revR;
    s.reverb.Process(mix, mix, &revL, &revR);

    // One-pole smoothing (~20 ms) to avoid zipper noise from the 5 ms pot scan
    s.masterVolumeSmoothed += (s.controls.masterVolume - s.masterVolumeSmoothed) * 0.001f;
    float outGain = kOutputHeadroom * s.masterVolumeSmoothed;

    float dryWet = 1.0f - sm[SM_REVERB];
    out[0][i] = (mix * dryWet + revL * sm[SM_REVERB]) * outGain;
    out[1][i] = (mix * dryWet + revR * sm[SM_REVERB]) * outGain;

    s.pulsar.running = s.pulsar.isRunning();
  }
}
}  // namespace weasel
