// =============================================================================
// smoothing.h — audio-rate smoothing of pot-driven parameters
//
// Two stages per parameter, run at audio rate:
//  1. Linear ramp from the previous pot reading to the new one, spread over the
//     time the last pot scan actually took. This removes the scan-rate (~200 Hz)
//     ripple a plain one-pole leaves behind - the main cause of audible stepping.
//  2. A short one-pole (kParamSmoothMs) that rounds off the corners between ramps.
//
// The pot scan (main loop) writes the source values and calls publish(); the
// audio callback calls beginBlock() once a block and tick() once a sample.
// =============================================================================
#pragma once
#include <stdint.h>
#include <math.h>

namespace weasel
{
// Shorter = snappier response. 1-3 ms is a good range.
const float kParamSmoothMs = 2.0f;

struct SmoothedParam
{
  const float* source;  // pot-derived value written by the pot scan
  float ramp;           // linearly ramped value
  float inc;            // ramp increment per sample
  float target;         // value this ramp is heading to (latched at ramp start)
  float out;            // final smoothed value used by the DSP
};

template <int N>
class SmoothedParams
{
 public:
  SmoothedParam p[N];

  void init(float sampleRate)
  {
    sampleRate_ = sampleRate;
    // One-pole coefficient for the requested glide time (time constant = kParamSmoothMs)
    coeff_ = 1.0f - expf(-1.0f / (kParamSmoothMs * 0.001f * sampleRate));
  }

  // Start every parameter exactly at its pot position (no glide at power-up)
  void snap()
  {
    for (int i = 0; i < N; i++)
    {
      p[i].ramp = *p[i].source;
      p[i].out = p[i].ramp;
      p[i].inc = 0.0f;
      p[i].target = p[i].ramp;
    }
    versionSeen_ = version_;
  }

  // Main loop, after a pot scan: new values are ready. Ramps run over the time
  // this scan actually took to come round, so they join up end to end even when
  // the loop is held up by other work.
  void publish(unsigned long nowUs)
  {
    float intervalSamples = (float)(nowUs - lastPublishUs_) * 1e-6f * sampleRate_;
    lastPublishUs_ = nowUs;
    if (intervalSamples < 48.0f) intervalSamples = 48.0f;    // 1ms
    if (intervalSamples > 960.0f) intervalSamples = 960.0f;  // 20ms
    rampSamples_ = (uint32_t)intervalSamples;
    version_ = version_ + 1;
  }

  // Audio callback, start of each block: if new pot values arrived, set up ramps to them
  inline void beginBlock()
  {
    uint32_t v = version_;
    if (v == versionSeen_) return;
    versionSeen_ = v;
    float invLen = 1.0f / (float)rampSamples_;
    for (int i = 0; i < N; i++)
    {
      p[i].target = *p[i].source;
      p[i].inc = (p[i].target - p[i].ramp) * invLen;
    }
  }

  // Audio callback, every sample
  inline void tick()
  {
    const float k = coeff_;
    for (int i = 0; i < N; i++)
    {
      SmoothedParam& s = p[i];
      if (s.inc != 0.0f)
      {
        s.ramp += s.inc;
        // stop exactly at the target instead of overshooting
        if ((s.inc > 0.0f && s.ramp >= s.target) || (s.inc < 0.0f && s.ramp <= s.target))
        {
          s.ramp = s.target;
          s.inc = 0.0f;
        }
      }
      s.out += k * (s.ramp - s.out);
    }
  }

  float operator[](int i) const { return p[i].out; }

 private:
  float sampleRate_ = 48000.0f;
  float coeff_ = 0.01f;
  volatile uint32_t version_ = 0;      // bumped after each pot scan
  uint32_t versionSeen_ = 0;           // last version the callback ramped towards
  volatile uint32_t rampSamples_ = 240;  // measured pot-scan interval, in samples
  unsigned long lastPublishUs_ = 0;
};
}  // namespace weasel
