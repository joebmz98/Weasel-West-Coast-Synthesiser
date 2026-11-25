// =============================================================================
// lpg_channel.h — one panel low pass gate channel (Buchla 292 model)
//
// The gates are a model of the Buchla 292 circuit (buchla_lpg.cpp): a vactrol
// driving the 292's audio circuit, with VCA / Lopass / Both set by switching the
// circuit's components the way the real mode switch does. The vactrol provides
// the gate's slew.
// =============================================================================
#pragma once
#include "buchla_lpg.h"

namespace weasel
{
enum LpgMode : uint8_t
{
  LPG_MODE_VCA,
  LPG_MODE_LP,
  LPG_MODE_COMBI
};

class LpgChannel
{
 public:
  // Panel control (level pot + patch-bay modulation, 0..1) maps linearly onto the
  // 292's 0..10.7 V control input.
  static constexpr float kCvVolts = 10.7f;
  // Vactrol settle times in seconds. 12 ms / 250 ms are the modelled VTL5C3-style
  // values. Shorten kVactrolOff (e.g. 0.05) for snappier sequenced gates.
  static constexpr float kVactrolOn = 0.012f;
  static constexpr float kVactrolOff = 0.25f;
  // How much a harder hit opens faster and rings longer (0 = none, 0.9 = model default)
  static constexpr float kVactrolRho = 0.9f;
  // Lopass mode resonance as a fraction of the self-oscillation limit.
  // 0.8 gives the ~+4.5 dB "small resonant bump" of the real 292; 1.0 would self-oscillate.
  static constexpr float kLopassResonance = 0.8f;

  LpgMode mode = LPG_MODE_COMBI;

  void init(float sampleRate)
  {
    lpg_.Init(sampleRate);
    lpg_.SetVactrolTimes(kVactrolOn, kVactrolOff);
    lpg_.SetVactrolRho(kVactrolRho, kVactrolRho);
    lpg_.SetResonance(kLopassResonance);
    lpg_.SetMode(circuitMode(mode));
  }

  void cycleMode() { mode = static_cast<LpgMode>((mode + 1) % 3); }

  // ctrl: 0..1 (clamped)
  inline float process(float in, float ctrl)
  {
    BuchlaLpg::Mode m = circuitMode(mode);
    if (lpg_.GetMode() != m) lpg_.SetMode(m);

    if (ctrl < 0.0f) ctrl = 0.0f;
    if (ctrl > 1.0f) ctrl = 1.0f;
    return lpg_.Process(ctrl * kCvVolts, in);
  }

 private:
  // Panel LPG mode -> 292 circuit mode
  static inline BuchlaLpg::Mode circuitMode(LpgMode mode)
  {
    switch (mode)
    {
      case LPG_MODE_VCA: return BuchlaLpg::MODE_VCA;
      case LPG_MODE_LP: return BuchlaLpg::MODE_LOPASS;
      case LPG_MODE_COMBI:
      default: return BuchlaLpg::MODE_BOTH;
    }
  }

  BuchlaLpg lpg_;
};
}  // namespace weasel
