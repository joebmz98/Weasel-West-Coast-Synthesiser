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

class ComplexOscillator
{
 public:
  int waveIndex = 1;      // 0 TRI, 1 SAW, 2 SQUARE (firmware order; boots on SAW)
  bool inverted = false;  // POLARITY button

  void init(float sampleRate)
  {
    sine_.Init(sampleRate);
    sine_.SetWaveform(sine_.WAVE_SIN);
    sine_.SetFreq(440.0f);
    sine_.SetAmp(1.0f);
    morph_.Init(sampleRate);
    morph_.SetWaveform(morph_.WAVE_POLYBLEP_SAW);  // matches waveIndex = 1
    morph_.SetFreq(440.0f);
    morph_.SetAmp(1.0f);
  }

  // index: 0 TRI, 1 SAW, 2 SQUARE
  void setWave(int index)
  {
    waveIndex = index;
    if (index == 0) morph_.SetWaveform(morph_.WAVE_TRI);
    else if (index == 1) morph_.SetWaveform(morph_.WAVE_POLYBLEP_SAW);
    else morph_.SetWaveform(morph_.WAVE_POLYBLEP_SQUARE);
  }

  // One output sample.
  //   freq     Hz (already clamped below Nyquist)
  //   timbre   0 = sine .. 1 = the selected waveform
  //   foldGain wavefolder input gain, 1..20 (1 = no folding)
  //   amGain   multiplier after the folder (1 in FM mode)
  inline float process(float freq, float timbre, float foldGain, float amGain)
  {
    sine_.SetFreq(freq);
    morph_.SetFreq(freq);

    float complexSig = sine_.Process();
    float morphSig = morph_.Process();

    float mixedSig = complexSig + (morphSig - complexSig) * timbre;
    if (inverted) mixedSig = -mixedSig;

    float foldedSig = fold(mixedSig * foldGain);
    foldedSig *= amGain;
    return foldedSig;
  }

 private:
  // Triangle wavefolder (DaisySP Wavefolder): |x| <= 1 passes, beyond that it
  // reflects back off +/-1.
  static inline float fold(float in)
  {
    float ft = floorf((in + 1.0f) * 0.5f);
    float sgn = static_cast<int>(ft) % 2 == 0 ? 1.0f : -1.0f;
    return sgn * (in - 2.0f * ft);
  }

  daisysp::Oscillator sine_;
  daisysp::Oscillator morph_;
};
}  // namespace weasel
