// =============================================================================
// controls.h — what each pot does (main loop, after every pot scan)
// =============================================================================
#pragma once
#include "synth.h"

namespace weasel
{
WEASEL_NOINLINE inline void updateControls(Synth& s)
{
  Controls& c = s.controls;
  const Pots& pots = s.pots;
  using daisysp::fmap;
  using daisysp::Mapping;

  // FINE TUNE range: +/- 3 semitones
  const float fineRange = 3.0f / 12.0f;

  // -- COMPLEX OSCILLATOR --
  float compFreq = fmap(pots.norm(POT_COMPOSC_FREQ), 25.0f, 1760.0f, Mapping::EXP);
  c.compPitchOct = log2f(compFreq);  // smoothed in octaves in the audio callback
  c.compFreqModDepth = pots.norm(POT_COMPOSC_FREQ_CTRL);
  c.compFine = (pots.norm(POT_COMPOSC_FINE) * 2.0f - 1.0f) * fineRange;
  c.compTimbre = pots.norm(POT_COMPOSC_TIMBRE);
  c.compFold = 1.0f + (19.0f * pots.norm(POT_COMPOSC_WF));
  c.compFoldModDepth = pots.norm(POT_COMPOSC_WF_CTRL);

  // -- LPG 1 --
  c.lpg1ModDepth = fmap(pots.norm(POT_LPG1_CTRL), 0.0f, 0.98f, Mapping::LINEAR);
  c.lpg1Level = fmap(pots.norm(POT_LPG1_LEVEL), 0.0f, 0.98f, Mapping::EXP);

  // -- MODULATION OSCILLATOR --
  float modFreq = fmap(pots.norm(POT_MODOSC_FREQ), 0.1f, 1000.0f, Mapping::EXP);
  c.modPitchOct = log2f(modFreq);
  c.modFreqModDepth = pots.norm(POT_MODOSC_FREQ_CTRL);
  c.modFine = (pots.norm(POT_MODOSC_FINE) * 2.0f - 1.0f) * fineRange;
  c.modDepth = pots.norm(POT_MODOSC_MOD);
  c.modDepthModDepth = pots.norm(POT_MODOSC_MOD_CTRL);

  // -- LPG 2 --
  c.lpg2ModDepth = fmap(pots.norm(POT_LPG2_CTRL), 0.0f, 0.98f, Mapping::LINEAR);
  c.lpg2Level = fmap(pots.norm(POT_LPG2_LEVEL), 0.0f, 0.98f, Mapping::EXP);

  // -- ENVELOPE [Buchla ASD] --
  EnvelopeGenerator& env = s.env;
  const float minTime = 0.002f;
  const float maxTime = 5.0f;

  env.attackTime = fmap(pots.norm(POT_ENV_ATTACK), minTime, maxTime, Mapping::EXP);
  env.adsr.SetAttackTime(env.attackTime);

  env.sustainDuration = fmap(pots.norm(POT_ENV_SUSTAIN), 0.02f, maxTime, Mapping::EXP);

  // TRANSIENT mode: gate pulse length = sustain pot value (minimum 20 ms)
  env.transientGateSamples = (uint32_t)(env.sustainDuration * s.sampleRate);
  uint32_t minTransientSamples = (uint32_t)(0.02f * s.sampleRate);
  if (env.transientGateSamples < minTransientSamples) env.transientGateSamples = minTransientSamples;

  // SUSTAIN mode: gate length = sustain pot value (minimum 100 ms)
  uint32_t minSustainGate = (uint32_t)(0.1f * s.sampleRate);
  env.sustainGateSamples = (uint32_t)(env.sustainDuration * s.sampleRate);
  if (env.sustainGateSamples < minSustainGate) env.sustainGateSamples = minSustainGate;

  // Release (DECAY pot) - shorter range in oscillate mode
  float releasePot = pots.norm(POT_ENV_DECAY);
  if (env.mode == ENV_MODE_OSCILLATE)
  {
    env.releaseTime = fmap(releasePot, minTime, maxTime * 0.2f, Mapping::EXP);
  }
  else
  {
    env.releaseTime = fmap(releasePot, minTime, maxTime, Mapping::EXP);
  }
  env.adsr.SetReleaseTime(env.releaseTime);
  env.adsr.SetDecayTime(0.0f);
  env.adsr.SetSustainLevel(1.0f);

  // -- PULSAR --
  s.pulsar.setPeriod(pots.norm(POT_PULSAR_PERIOD));

  // -- SEQUENCER STEPS --
  for (int i = 0; i < Sequencer::kSteps; i++)
  {
    s.seq.stepCV[i] = fmap(pots.norm(POT_SEQ_STEP1 + i), 0.0f, 1.0f, Mapping::EXP);
  }

  // -- CLOCK --
  if (s.midiClock.sync)
  {
    // Pot selects the clock division in MIDI sync mode
    s.midiClock.updateDivision(pots.norm(POT_CLOCK));
  }
  else
  {
    float clockPot = pots.norm(POT_CLOCK);
    float periodMs = 0.1f * powf(40000.0f, (1.0f - clockPot));
    if (periodMs < 0.1f) periodMs = 0.1f;
    if (periodMs > 4000.0f) periodMs = 4000.0f;
    s.seq.clockPeriodMs = periodMs;

    s.seq.samplesPerTick = (uint32_t)((periodMs / 1000.0f) * s.sampleRate);
    if (s.seq.samplesPerTick < 1) s.seq.samplesPerTick = 1;
  }

  // -- REVERB --
  c.reverbMix = pots.norm(POT_REVERB_MIX);

  // -- MASTER VOLUME --
  // Squared for an audio-taper feel on a linear pot; dead zone at the bottom for true silence
  float vol = pots.norm(POT_MASTER_VOLUME);
  if (vol < 0.01f) vol = 0.0f;
  c.masterVolume = vol * vol;
}
}  // namespace weasel
