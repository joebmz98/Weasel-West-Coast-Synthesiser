// =============================================================================
// cpu_meter.h — audio CPU load from the Cortex-M7 cycle counter
//
// Load = cycles spent in the audio callback / cycles available per block.
// At 480 MHz and 64 samples per block at 48 kHz there are 640,000 cycles per
// block; using 160,000 of them is 25 %. With WEASEL_DEBUG 1 the load is printed
// over USB serial every few seconds.
//
// (DaisyDuino's CpuLoadMeter uses a timer the Arduino startup never starts, so
// it would always read 0 - hence this small version, as in GLOOP.)
// =============================================================================
#pragma once
#include "Arduino.h"

namespace weasel
{
class CpuMeter
{
 public:
  void init(float sampleRate, size_t blockSize)
  {
#if defined(DWT)
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;  // enable the trace unit
    DWT->LAR = 0xC5ACCE55;                           // unlock the DWT (M7)
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;             // start the cycle counter
    budget_ = (float)SystemCoreClock * (float)blockSize / sampleRate;
#endif
  }

  inline void blockStart()
  {
#if defined(DWT)
    t0_ = DWT->CYCCNT;
#endif
  }

  inline void blockEnd()
  {
#if defined(DWT)
    last_ = (float)(DWT->CYCCNT - t0_) / budget_;
    avg_ += 0.01f * (last_ - avg_);  // ~0.13 s smoothing at 750 blocks a second
    if (last_ > peak_) peak_ = last_;
#endif
  }

  float last() const { return last_; }  // this block, 0..1+
  float avg() const { return avg_; }    // smoothed
  float peak() const { return peak_; }  // worst since resetPeak()
  void resetPeak() { peak_ = 0.0f; }

 private:
  float budget_ = 1.0f;
  volatile uint32_t t0_ = 0;
  volatile float last_ = 0.0f, avg_ = 0.0f, peak_ = 0.0f;
};
}  // namespace weasel
