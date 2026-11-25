// =============================================================================
// oscillators.h — modulation oscillator and complex oscillator
//
// MODULATION OSCILLATOR: sine / triangle / saw / square, used as an audio
// voice (LPG 2) and as the FM or AM source for the complex oscillator.
//
// COMPLEX OSCILLATOR: a sine crossfaded (TIMBRE) into a second waveform
// (triangle / saw / square), then the wavefolder, then AM.
// =============================================================================
#pragma once
#include <math.h>
#include "DaisyDuino.h"
#include "config.h"
#include "antialias.h"

namespace weasel
{
class ModulationOscillator
{
 public:
  daisysp::Oscillator osc;
  int waveIndex = 0;          // 0 SIN, 1 TRI, 2 SAW, 3 SQUARE (panel order)
  bool amplitudeMod = false;  // false = FM, true = AM

  void init(float sampleRate)
  {
    osc.Init(sampleRate);
    osc.SetWaveform(osc.WAVE_SIN);
    osc.SetFreq(440.0f);
    osc.SetAmp(1.0f);
  }

  void setWave(int index)
  {
    waveIndex = index;
    if (index == 0) osc.SetWaveform(osc.WAVE_SIN);
    else if (index == 1) osc.SetWaveform(osc.WAVE_TRI);
    else if (index == 2) osc.SetWaveform(osc.WAVE_POLYBLEP_SAW);
    else osc.SetWaveform(osc.WAVE_POLYBLEP_SQUARE);
  }

  inline float process(float freq)
  {
    osc.SetFreq(freq);
    float s = osc.Process();
    // Safety clamp (band-limited saw/square overshoot slightly)
    if (s > 1.2f) s = 1.2f;
    if (s < -1.2f) s = -1.2f;
    return s;
  }
};

// COMPLEX OSCILLATOR
// Oversampled (WEASEL_COMPLEX_OVERSAMPLING, config.h): the oscillator, the
// wavefolder and the AM all run at 2x or 4x the sample rate, then two half-band
// filters bring it back to 48 kHz (antialias.h). On top of that each part is
// band-limited where it can be: polyBLEP saw / square, polyBLAMP triangle, and
// the wavefolder uses antiderivative anti-aliasing. The waveforms are the same
// shapes as DaisySP's (same phase, polarity and levels), so TIMBRE, POLARITY and
// the folder behave exactly as before - just without the aliasing.
// WEASEL_COMPLEX_OVERSAMPLING 1 runs the 1.1.x code instead, for comparison.
class ComplexOscillator
{
 public:
  static const int kOversample = WEASEL_COMPLEX_OVERSAMPLING;
  static_assert(kOversample == 1 || kOversample == 2 || kOversample == 4,
                "WEASEL_COMPLEX_OVERSAMPLING must be 1, 2 or 4");

  int waveIndex = 1;      // 0 TRI, 1 SAW, 2 SQUARE (firmware order; boots on SAW)
  bool inverted = false;  // POLARITY button

  ComplexOscillator()
      : dec192_(halfband::kTaps192, halfband::kCentre192),
        dec96_(halfband::kTaps96, halfband::kCentre96)
  {
  }

  void init(float sampleRate)
  {
#if WEASEL_COMPLEX_OVERSAMPLING == 1
    sine_.Init(sampleRate);
    sine_.SetWaveform(sine_.WAVE_SIN);
    sine_.SetFreq(440.0f);
    sine_.SetAmp(1.0f);
    morph_.Init(sampleRate);
    morph_.SetWaveform(morph_.WAVE_POLYBLEP_SAW);  // matches waveIndex = 1
    morph_.SetFreq(440.0f);
    morph_.SetAmp(1.0f);
#else
    invOsRate_ = 1.0f / (sampleRate * (float)kOversample);
    phase_ = 0.0f;
    prevFreq_ = 440.0f;
    prevTimbre_ = 0.0f;
    prevFoldGain_ = 1.0f;
    prevAmGain_ = 1.0f;
    folder_.reset();
    dec192_.reset();
    dec96_.reset();
#endif
  }

  // index: 0 TRI, 1 SAW, 2 SQUARE
  void setWave(int index)
  {
    waveIndex = index;
#if WEASEL_COMPLEX_OVERSAMPLING == 1
    if (index == 0) morph_.SetWaveform(morph_.WAVE_TRI);
    else if (index == 1) morph_.SetWaveform(morph_.WAVE_POLYBLEP_SAW);
    else morph_.SetWaveform(morph_.WAVE_POLYBLEP_SQUARE);
#endif
  }

  // One output sample.
  //   freq     Hz (already clamped below Nyquist)
  //   timbre   0 = sine .. 1 = the selected waveform
  //   foldGain wavefolder input gain, 1..20 (1 = no folding)
  //   amGain   multiplier after the folder (1 in FM mode)
  // Oversampled, the four controls are ramped across the sub-samples from the
  // previous sample's values, so fast FM / AM / fold modulation doesn't step.
  inline float process(float freq, float timbre, float foldGain, float amGain)
  {
#if WEASEL_COMPLEX_OVERSAMPLING == 1
    sine_.SetFreq(freq);
    morph_.SetFreq(freq);

    float complexSig = sine_.Process();
    float morphSig = morph_.Process();

    float mixedSig = complexSig + (morphSig - complexSig) * timbre;
    if (inverted) mixedSig = -mixedSig;

    float foldedSig = AdaaFolder::fold(mixedSig * foldGain);
    foldedSig *= amGain;
    return foldedSig;
#else
    const float step = 1.0f / (float)kOversample;
    const float dFreq = (freq - prevFreq_) * step;
    const float dTimbre = (timbre - prevTimbre_) * step;
    const float dFold = (foldGain - prevFoldGain_) * step;
    const float dAm = (amGain - prevAmGain_) * step;
    const float polarity = inverted ? -1.0f : 1.0f;

    float y[kOversample];
    for (int j = 0; j < kOversample; j++)
    {
      const float jj = (float)(j + 1);
      const float dt = (prevFreq_ + dFreq * jj) * invOsRate_;  // phase increment

      const float sine = sin2pi(phase_);
      const float wave = waveform(phase_, dt);
      phase_ += dt;
      if (phase_ >= 1.0f) phase_ -= 1.0f;

      const float t = prevTimbre_ + dTimbre * jj;
      const float mixed = (sine + (wave - sine) * t) * polarity;
      const float folded = folder_.process(mixed * (prevFoldGain_ + dFold * jj));
      y[j] = folded * (prevAmGain_ + dAm * jj);
    }

    prevFreq_ = freq;
    prevTimbre_ = timbre;
    prevFoldGain_ = foldGain;
    prevAmGain_ = amGain;

#if WEASEL_COMPLEX_OVERSAMPLING == 4
    // (two statements: the order of evaluation of function arguments is unspecified)
    const float a = dec192_.process(y[0], y[1]);
    const float b = dec192_.process(y[2], y[3]);
    return dec96_.process(a, b);
#else
    return dec96_.process(y[0], y[1]);
#endif
#endif
  }

 private:
#if WEASEL_COMPLEX_OVERSAMPLING == 1
  daisysp::Oscillator sine_;
  daisysp::Oscillator morph_;
#else
  // The TIMBRE waveform at phase t (0..1), band-limited for increment dt.
  // Same shapes as DaisySP: triangle +1 at phase 0, saw falling from +1,
  // square +1 for the first half at 0.707.
  inline float waveform(float t, float dt) const
  {
    if (waveIndex == 1)  // SAW
    {
      return 1.0f - 2.0f * t + polyBlep(t, dt);
    }
    if (waveIndex == 2)  // SQUARE
    {
      float t2 = t + 0.5f;
      if (t2 >= 1.0f) t2 -= 1.0f;
      float sq = (t < 0.5f) ? 1.0f : -1.0f;
      return (sq + polyBlep(t, dt) - polyBlep(t2, dt)) * 0.707f;
    }
    // TRIANGLE: corners at phase 0 (peak, slope -8 per cycle change) and 0.5
    // (trough, +8), each rounded with a polyBLAMP
    float tri = 2.0f * fabsf(2.0f * t - 1.0f) - 1.0f;
    if (dt < 1.0e-7f) return tri;  // (near) stopped: nothing to band-limit
    const float invDt = 1.0f / dt;
    const float corner = 8.0f * dt;  // slope change per sample
    tri += corner * polyBlamp((t - 0.5f) * invDt);
    float d0 = (t < 0.5f) ? t : t - 1.0f;  // distance to the peak at phase 0
    tri -= corner * polyBlamp(d0 * invDt);
    return tri;
  }

  float invOsRate_ = 1.0f / 192000.0f;
  float phase_ = 0.0f;
  float prevFreq_ = 440.0f;
  float prevTimbre_ = 0.0f;
  float prevFoldGain_ = 1.0f;
  float prevAmGain_ = 1.0f;
  AdaaFolder folder_;
#endif
  Halfband192 dec192_;
  Halfband96 dec96_;
};
}  // namespace weasel
