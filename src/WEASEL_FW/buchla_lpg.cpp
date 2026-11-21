// Buchla 292-style low pass gate. See buchla_lpg.h for sources and credits.
#include "buchla_lpg.h"
#include <math.h>

using namespace weasel;

namespace
{
// -- Vactrol (Parker & D'Angelo / Rohs) --
const float kIMax     = 0.040f;      // max LED current (A)
const float kIMin     = 1.0e-7f;     // dark-current floor: ~20 GOhm, gate fully shut
const float kVMax     = 10.7f;       // driver input clamp (V)
const float kIs       = 10.1e-5f;    // driver I-V curve scale
const float kIvExp    = 0.569832f;   // driver I-V exponent (87 / 3.9696 * 0.026, as in sfdsp)
const float kAVac     = 3.4645912f;  // vactrol: R = A / I^1.4 + B
const float kBVac     = 1136.2129956f;
const float kVacPow   = 1.4f;

// -- Audio circuit fixed parts (Parker & D'Angelo, Table 1) --
const float kC1          = 1.0e-9f;
const float kC2          = 220.0e-12f;
const float kC3Lopass    = 4.7e-9f;
const float kRAlphaBoth  = 5.0e6f;
const float kRAlphaVca   = 5.0e3f;

inline float clampf(float x, float lo, float hi)
{
    return x < lo ? lo : (x > hi ? hi : x);
}

inline bool badFloat(float x)
{
    return (x != x) || (x > 1e10f) || (x < -1e10f);
}

// Vactrol resistance for a given LED current
inline float vactrolR(float i)
{
    if(i < kIMin)
        i = kIMin;
    // i^-1.4 via exp2/log2: cheaper than powf on the M7
    return kAVac * exp2f(-kVacPow * log2f(i)) + kBVac;
}
} // namespace

void BuchlaLpg::Init(float sample_rate)
{
    sample_rate_ = sample_rate;
    f_           = 1.0f / (2.0f * sample_rate_);
    ctrl_dt_     = (float)CTRL_INTERVAL / sample_rate_;
    SetVactrolTimes(0.012f, 0.25f);
    SetVactrolRho(0.9f, 0.9f);
    SetMode(MODE_BOTH);
    Reset();
}

void BuchlaLpg::Reset()
{
    sc_ = yc_ = prev_i_ = 0.0f;
    prev_dir_     = 2;
    wc_           = wc_off_;
    s0_ = s1_     = 0.0f;
    rf_           = vactrolR(0.0f);
    ctrl_counter_ = 0;
    UpdateCoefficients();
}

void BuchlaLpg::SetMode(Mode mode)
{
    mode_ = mode;
    switch(mode_)
    {
        case MODE_VCA:
            r_alpha_ = kRAlphaVca;
            c3_      = 0.0f;
            break;
        case MODE_LOPASS:
            r_alpha_ = kRAlphaBoth;
            c3_      = kC3Lopass;
            break;
        case MODE_BOTH:
        default:
            r_alpha_ = kRAlphaBoth;
            c3_      = 0.0f;
            break;
    }
    // DC gain is Ra / (Ra + 2Rf). The real 292 makes this up elsewhere (the VCA
    // switch also raises the input stage gain), so normalise fully open to unity.
    const float rf_open = vactrolR(kIMax);
    makeup_             = (r_alpha_ + 2.0f * rf_open) / r_alpha_;
    UpdateCoefficients();
}

void BuchlaLpg::SetResonance(float res)
{
    res_ = clampf(res, 0.0f, 0.99f);
    UpdateCoefficients();
}

void BuchlaLpg::SetVactrolTimes(float tau_on, float tau_off)
{
    if(tau_on < 1.0e-4f)
        tau_on = 1.0e-4f;
    if(tau_off < 1.0e-4f)
        tau_off = 1.0e-4f;
    // As in Rohs/sfdsp: tau is the time to settle (~5 time constants)
    wc_on_  = 5.0f / tau_on;
    wc_off_ = 5.0f / tau_off;
}

void BuchlaLpg::SetVactrolRho(float rho_on, float rho_off)
{
    rho_on_  = clampf(rho_on, -1.0f, 1.0f);
    rho_off_ = clampf(rho_off, -1.0f, 1.0f);
}

void BuchlaLpg::UpdateControl(float cv)
{
    // Driver: control voltage -> LED current
    cv      = clampf(cv, 0.0f, kVMax);
    float i = kIs * (expf(kIvExp * cv) - 1.0f);
    if(i > kIMax)
        i = kIMax;

    // Vactrol: one-pole (TPT) whose speed depends on direction and level
    int8_t dir = (i > prev_i_) ? 1 : ((i < prev_i_) ? -1 : 0);
    if(dir == 1)
    {
        // Rising: follow the drive level, so a harder hit opens faster.
        // (Recomputed while rising so slow pot moves aren't stuck on a slow rate.)
        wc_ = wc_on_ * (1.0f - rho_on_ + rho_on_ * i / kIMax);
    }
    else if(dir == -1 && dir != prev_dir_)
    {
        // Falling: latch on the turn, scaled by how far open the vactrol is
        wc_ = wc_off_ * (1.0f + rho_off_ - rho_off_ * yc_ / kIMax);
    }
    prev_i_ = i;
    if(dir != 0)
        prev_dir_ = dir;

    const float g = wc_ * ctrl_dt_ * 0.5f;
    const float v = (i - sc_) * g / (1.0f + g);
    yc_           = v + sc_;
    sc_           = yc_ + v;

    rf_ = vactrolR(yc_);
    UpdateCoefficients();
}

void BuchlaLpg::UpdateCoefficients()
{
    // Nodal equations (Parker & D'Angelo eq. 1-2), x = [Vx, Vout]:
    //   Vx'   = b1 Vin + b2 Vx + b3 Vout + b4 (a Vout' - Vx')
    //   Vout' = a1 Vx + a2 Vout
    // Solving the first for Vx' gives a plain linear system x' = A x + B Vin.
    const float g_rf = 1.0f / rf_;
    const float a1   = g_rf / kC1;
    const float a2   = -(g_rf + 1.0f / r_alpha_) / kC1;
    const float b1   = g_rf / kC2;
    const float b2   = -2.0f * g_rf / kC2;
    const float b3   = b1;
    const float b4   = c3_ / kC2;

    float a = 0.0f;
    if(c3_ > 0.0f)
    {
        // Normalised resonance: fraction of the stability limit
        const float a_max
            = (2.0f * kC1 * r_alpha_ + (kC2 + c3_) * (r_alpha_ + rf_))
              / (c3_ * r_alpha_);
        a = res_ * a_max;
    }

    const float k   = 1.0f / (1.0f + b4);
    const float p   = (b2 + b4 * a * a1) * k; // A00
    const float q   = (b3 + b4 * a * a2) * k; // A01
    const float r   = b1 * k;                 // B0

    // M = (I - f A)^-1
    const float n00 = 1.0f - f_ * p;
    const float n01 = -f_ * q;
    const float n10 = -f_ * a1;
    const float n11 = 1.0f - f_ * a2;
    const float inv = 1.0f / (n00 * n11 - n01 * n10);
    m00_            = n11 * inv;
    m01_            = -n01 * inv;
    m10_            = -n10 * inv;
    m11_            = n00 * inv;
    fb_             = f_ * r;
}

float BuchlaLpg::Process(float cv, float in)
{
    if(++ctrl_counter_ >= CTRL_INTERVAL)
    {
        ctrl_counter_ = 0;
        UpdateControl(cv);
    }

    // Trapezoidal (TPT) step: x = M (s + f B u), s <- 2x - s
    const float v0 = s0_ + fb_ * in;
    const float v1 = s1_;
    const float x0 = m00_ * v0 + m01_ * v1;
    const float x1 = m10_ * v0 + m11_ * v1;
    s0_            = 2.0f * x0 - s0_;
    s1_            = 2.0f * x1 - s1_;

    const float out = x1 * makeup_;
    if(badFloat(out))
    {
        Reset();
        return 0.0f;
    }
    return out;
}
