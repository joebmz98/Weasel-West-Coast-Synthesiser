// =============================================================================
// dsp_util.h — small maths helpers shared by the audio code
// =============================================================================
#pragma once
#include <math.h>

namespace weasel
{
// Exponential pitch scaling, from absolute pitch in octaves (log2 Hz).
// The old Taylor approximation went NEGATIVE for exponents between about -1.4
// and -2.9 (and grew again below that), so deep FM produced negative
// frequencies. DaisySP oscillators never wrap a negative phase, so the phase
// ran away: saw/tri outputs went out of range and the sine lost precision over
// time (noise/flutter). exp2f is exact and cheap on the M7 FPU.
inline float pitchExp2(float octaves)
{
  if (octaves > 15.0f) octaves = 15.0f;  // 32kHz - clampOscFreq() trims to below Nyquist
  if (octaves < -10.0f) octaves = -10.0f;
  return exp2f(octaves);
}

// Keeps an oscillator frequency positive and below Nyquist
inline float clampOscFreq(float f, float sampleRate)
{
  if (f < 0.0f) return 0.0f;
  float maxF = sampleRate * 0.45f;
  return (f > maxF) ? maxF : f;
}

// Transparent below 0.8, then a smooth knee that approaches +/-1.
// Replaces the hard +/-1 clamp on the mix, which squared off peaks whenever
// both LPG channels were loud at once.
inline float softKnee(float x)
{
  const float t = 0.8f;
  float a = fabsf(x);
  if (a <= t) return x;
  float over = (a - t) / (1.0f - t);
  float y = t + (1.0f - t) * (over / (1.0f + over));
  return (x < 0.0f) ? -y : y;
}

// NaN and runaway detection
inline bool isBadFloat(float x)
{
  return (x != x) || (x > 1e10f) || (x < -1e10f);
}

// Frequency ratio of a MIDI note relative to C4 (note 60)
inline float midiTransposeRatio(int midiNote)
{
  float semitones = (float)(midiNote - 60);
  return powf(2.0f, semitones / 12.0f);
}

inline float clamp(float x, float lo, float hi)
{
  return x < lo ? lo : (x > hi ? hi : x);
}
}  // namespace weasel
