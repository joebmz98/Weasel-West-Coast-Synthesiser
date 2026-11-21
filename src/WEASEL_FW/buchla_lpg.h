/*
Buchla 292-style low pass gate: vactrol + audio circuit model.

Based on:
  - J. Parker & S. D'Angelo, "A Digital Model of the Buchla Lowpass-Gate", DAFx-13
  - J. Rohs, "A Matlab Implementation of the Buchla Lopass Gate", McGill MUMT618 (2018)
    (docs/references/MUMT618_Buchla_LPG_Report.pdf)
  - sfdsp::BuchlaLPG, Copyright (c) 2023 Segfault1602, MIT License
    https://github.com/Segfault1602/libdsp

Changes from sfdsp::BuchlaLPG for the WEASEL:
  - per-sample Process() instead of block processing
  - all three 292 modes (VCA, Lopass, Both) by switching the circuit's fixed
    component values, as in Parker & D'Angelo Table 1
  - Lopass-mode resonance (feedback gain a, normalised to the stability limit)
  - the audio circuit is solved as a 2x2 trapezoidal (TPT) state-space system,
    so the C3 feedback path is handled exactly
  - vactrol and filter coefficients are updated every CTRL_INTERVAL samples
  - adjustable vactrol on/off times and a deeper dark-resistance floor so a
    closed gate is properly silent

Use of this source code is governed by an MIT-style license.
*/
#pragma once
#ifndef WEASEL_BUCHLA_LPG_H
#define WEASEL_BUCHLA_LPG_H

#include <stdint.h>

namespace weasel
{

class BuchlaLpg
{
  public:
    enum Mode : uint8_t
    {
        MODE_VCA = 0,
        MODE_LOPASS,
        MODE_BOTH,
    };

    // Vactrol and filter coefficients are recomputed every CTRL_INTERVAL samples.
    static const uint8_t CTRL_INTERVAL = 4;

    BuchlaLpg() {}

    void Init(float sample_rate);

    /** Clears the vactrol and filter state (gate fully closed). */
    void Reset();

    /** VCA, Lopass or Both (combination). Takes effect from the next sample. */
    void SetMode(Mode mode);
    Mode GetMode() const { return mode_; }

    /** Lopass-mode resonance, 0..1 of the stability limit (1 = self-oscillation). */
    void SetResonance(float res);

    /** Vactrol time constants in seconds. Defaults are 12 ms on, 250 ms off. */
    void SetVactrolTimes(float tau_on, float tau_off);

    /** How much the vactrol's speed depends on the drive level, 0..1 (default 0.9). */
    void SetVactrolRho(float rho_on, float rho_off);

    /** One sample.
        @param cv control voltage into the vactrol driver, 0..10.7 V
        @param in audio input
        @return gated / filtered audio */
    float Process(float cv, float in);

    /** Current vactrol resistance in ohms (for debugging / LEDs). */
    float GetResistance() const { return rf_; }

  private:
    void UpdateControl(float cv);
    void UpdateCoefficients();

    float sample_rate_ = 48000.0f;
    float f_           = 0.0f; // trapezoidal step: 1 / (2 * fs)
    float ctrl_dt_     = 0.0f; // vactrol update period

    Mode  mode_     = MODE_BOTH;
    float res_      = 0.0f;
    float r_alpha_  = 5.0e6f;
    float c3_       = 0.0f;
    float makeup_   = 1.0f;

    // vactrol
    float wc_on_    = 0.0f;
    float wc_off_   = 0.0f;
    float rho_on_   = 0.9f;
    float rho_off_  = 0.9f;
    float wc_       = 0.0f;
    float sc_       = 0.0f; // vactrol one-pole state
    float yc_       = 0.0f; // smoothed vactrol current (A)
    float prev_i_   = 0.0f;
    int8_t prev_dir_ = 2;
    float rf_       = 1.0e9f;
    uint8_t ctrl_counter_ = 0;

    // audio circuit: x' = A x + B u, x = [Vx, Vout]
    // TPT solve: x = M (s + f B u), s <- 2x - s
    float m00_ = 0.0f, m01_ = 0.0f, m10_ = 0.0f, m11_ = 0.0f;
    float fb_  = 0.0f; // f * B[0]
    float s0_  = 0.0f, s1_ = 0.0f;
};

} // namespace weasel

#endif
