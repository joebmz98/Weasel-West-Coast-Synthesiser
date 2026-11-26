// =============================================================================
// antialias.h — building blocks for the oversampled complex oscillator
//
//   sin2pi()            fast sine of a 0..1 phase (error < 2e-7, -134 dB)
//   polyBlep/polyBlamp  2-point corrections for jumps (saw, square) and corners
//                       (triangle), so the waveforms are band-limited at source
//   AdaaFolder          the triangle wavefolder with first-order antiderivative
//                       anti-aliasing (ADAA), float only
//   Halfband*           decimate-by-2 half-band FIR filters, 192k -> 96k -> 48k
//
// Why: the wavefolder (up to 20x gain), deep exponential FM and AM all create
// harmonics far above 24 kHz. At 48 kHz those fold back into the audible band as
// inharmonic "digital" noise that doesn't follow the pitch. Running the
// oscillator, folder and AM at 4x (192 kHz), with each stage band-limited where
// it can be, and filtering back down to 48 kHz removes almost all of it.
// =============================================================================
#pragma once
#include <math.h>
#include <stdint.h>

namespace weasel
{
// sin(2 pi t) for t in [0, 1). Odd 9th-order polynomial on a quarter wave.
inline float sin2pi(float t)
{
  float x = (t >= 0.5f) ? t - 1.0f : t;  // [-0.5, 0.5)
  if (x > 0.25f) x = 0.5f - x;
  else if (x < -0.25f) x = -0.5f - x;    // [-0.25, 0.25]
  const float z = x * x;
  return ((((3.953597260e+01f * z - 7.654969025e+01f) * z + 8.160099792e+01f) * z - 4.134165573e+01f) * z
          + 6.283185005e+00f)
         * x;
}

// PolyBLEP residual for an upward jump of 2 at phase 0 (DaisySP's definition).
// t: phase 0..1, dt: phase increment per sample.
inline float polyBlep(float t, float dt)
{
  if (t < dt)
  {
    t /= dt;
    return t + t - t * t - 1.0f;
  }
  if (t > 1.0f - dt)
  {
    t = (t - 1.0f) / dt;
    return t * t + t + t + 1.0f;
  }
  return 0.0f;
}

// PolyBLAMP residual for a corner (slope change of 1 per sample) at distance
// d samples: (1 - |d|)^3 / 6 for |d| < 1 - the integral of the polyBLEP residual.
inline float polyBlamp(float d)
{
  float a = 1.0f - fabsf(d);
  if (a <= 0.0f) return 0.0f;
  return a * a * a * (1.0f / 6.0f);
}

// -----------------------------------------------------------------------------
// Triangle wavefolder with ADAA. The fold is piecewise linear, slope +1 / -1 in
// segments of width 2 (corners at the odd numbers): |x| <= 1 passes, beyond that
// it reflects off +/-1. ADAA outputs the average of the fold over the step from
// the previous input to this one, (F(x) - F(x_prev)) / (x - x_prev) with F the
// antiderivative, which removes most of the aliasing the corners would cause.
// All in float (double is software-emulated on this build):
//  - both points in one segment: the average of a straight line is its midpoint
//  - one corner crossed (the usual case): the exact average of the two straight
//    pieces either side of it, weighted by their lengths - well conditioned even
//    for tiny steps
//  - two or more corners (a step wider than 2): the antiderivative, which is
//    periodic (period 4) and small, so float is accurate at that step size
// -----------------------------------------------------------------------------
class AdaaFolder
{
 public:
  void reset()
  {
    x1_ = 0.0f;
    seg1_ = 0.0f;
    f1_ = 0.0f;
  }

  // The plain fold (no anti-aliasing): DaisySP Wavefolder with offset 0
  static inline float fold(float in)
  {
    float ft = floorf((in + 1.0f) * 0.5f);
    float sgn = static_cast<int>(ft) % 2 == 0 ? 1.0f : -1.0f;
    return sgn * (in - 2.0f * ft);
  }

  inline float process(float x)
  {
    const float seg = floorf((x + 1.0f) * 0.5f);  // segment [2 seg - 1, 2 seg + 1)
    const int segInt = static_cast<int>(seg);
    const float sgn = (segInt & 1) ? -1.0f : 1.0f;
    const float fx = sgn * (x - 2.0f * seg);      // the plain fold of x
    const float dseg = seg - seg1_;
    float y;
    if (dseg == 0.0f)
    {
      y = sgn * (0.5f * (x + x1_) - 2.0f * seg);
    }
    else if (dseg == 1.0f || dseg == -1.0f)
    {
      const int m = (dseg > 0.0f) ? segInt : static_cast<int>(seg1_);
      const float c = 2.0f * (float)m - 1.0f;     // the corner between the two segments
      const float fc = (m & 1) ? 1.0f : -1.0f;    // fold value at that corner
      const float w = (c - x1_) / (x - x1_);      // share of the step before the corner
      y = 0.5f * fc + 0.5f * (w * f1_ + (1.0f - w) * fx);
    }
    else
    {
      y = (antiderivative(x) - antiderivative(x1_)) / (x - x1_);
    }
    x1_ = x;
    seg1_ = seg;
    f1_ = fx;
    return y;
  }

 private:
  // Antiderivative of the fold: period 4, between -0.5 and 0. Reducing x by a
  // whole number of periods first keeps it exact for large x.
  static inline float antiderivative(float x)
  {
    const float k = floorf((x + 1.0f) * 0.25f);
    const float r = x - 4.0f * k;  // [-1, 3)
    if (r < 1.0f) return 0.5f * (r * r - 1.0f);
    const float b = 2.0f - r;
    return 0.5f * (1.0f - b * b);
  }

  float x1_ = 0.0f;   // previous input
  float seg1_ = 0.0f;  // its segment
  float f1_ = 0.0f;   // its plain fold
};

// -----------------------------------------------------------------------------
// Half-band decimators. Equiripple (Parks-McClellan) designs, flat to 20 kHz:
//   192k -> 96k: 15 taps, passband +/-0.001 dB, stopband (76-96 kHz) -86 dB
//    96k -> 48k: 63 taps, passband +/-0.001 dB, stopband (28-48 kHz) -89 dB
// Content between 24 and 28 kHz folds to 20-24 kHz, above hearing.
// Every other tap of a half-band filter is zero and the rest are symmetric, so
// each output costs (N + 1) / 4 multiplies plus the centre tap.
// -----------------------------------------------------------------------------
template <int N, int K>  // N taps, K non-zero outer tap pairs
class HalfbandDecimator
{
 public:
  HalfbandDecimator(const float* taps, float centre) : taps_(taps), centre_(centre) {}

  void reset()
  {
    for (int i = 0; i < 2 * N; i++) buf_[i] = 0.0f;
    pos_ = 0;
  }

  // Two input samples (oldest first) -> one output sample
  inline float process(float x0, float x1)
  {
    push(x0);
    push(x1);
    const float* w = &buf_[pos_];  // w[0] = newest
    float acc = centre_ * w[(N - 1) / 2];
    for (int k = 0; k < K; k++) acc += taps_[k] * (w[2 * k] + w[N - 1 - 2 * k]);
    return acc;
  }

 private:
  inline void push(float x)
  {
    pos_ = (pos_ == 0) ? N - 1 : pos_ - 1;
    buf_[pos_] = x;
    buf_[pos_ + N] = x;  // mirrored copy: the window is always contiguous
  }

  const float* taps_;
  float centre_;
  float buf_[2 * N] = { 0 };
  int pos_ = 0;
};

namespace halfband
{
// Outer taps h[0], h[2], h[4], ... (the other half mirrors them)
const float kTaps192[4] = {
  -2.626996608e-03f, 1.745992836e-02f, -6.835996264e-02f, 3.035154375e-01f
};
const float kCentre192 = 5.000231868e-01f;

const float kTaps96[16] = {
  -6.902000165e-05f, 1.849234851e-04f, -4.156192788e-04f, 8.178033171e-04f,
  -1.466494908e-03f, 2.459473226e-03f, -3.914164465e-03f, 5.981670963e-03f,
  -8.856182096e-03f, 1.281659241e-02f, -1.829946829e-02f, 2.609786974e-02f,
  -3.788502013e-02f, 5.802957697e-02f, -1.026366261e-01f, 3.171475302e-01f
};
const float kCentre96 = 5.000143100e-01f;
}  // namespace halfband

typedef HalfbandDecimator<15, 4> Halfband192;
typedef HalfbandDecimator<63, 16> Halfband96;
}  // namespace weasel
