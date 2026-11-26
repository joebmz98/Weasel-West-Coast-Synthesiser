// Renders the complex oscillator (oscillators.h) on its own for a set of test cases,
// 2 s each at 48 kHz. Built by run.sh once per WEASEL_COMPLEX_OVERSAMPLING setting.
//   dsptest <out_dir>
#include "Arduino.h"
#include "DaisyDuino.h"
#include "oscillators.h"
#include <stdio.h>
#include <string>
HardwareSerial Serial1; Stream Serial; Stream SerialUSB; AudioClass DAISY;
namespace sim { uint64_t nowUs; DaisyDuinoCallback audioCb; void pinWrite(int,int){} int pinRead(int){return 0;} int adcRead(int){return 0;} }
using namespace weasel;

struct Case { const char* name; int wave; float f0, timbre, fold, fm, fmDepth, am, amDepth; };
// wave: 0 TRI 1 SAW 2 SQUARE. FM: exponential, compExp = sin(2 pi fm t) * depth * 5 (as in audio.h).
// AM: amGain = (1 + sin(2 pi fm t) d) / (1 + d).
// Each case repeats every 1/F s for its line spacing F (see analysis), so the
// true spectrum has lines only at multiples of F.
const Case cases[] = {
  { "saw_1009",        1, 1009, 1.0f, 1.0f, 0, 0, 0, 0 },
  { "saw_3001",        1, 3001, 1.0f, 1.0f, 0, 0, 0, 0 },
  { "tri_3001",        0, 3001, 1.0f, 1.0f, 0, 0, 0, 0 },
  { "square_1499",     2, 1499, 1.0f, 1.0f, 0, 0, 0, 0 },
  { "sine_fold8_1009", 0, 1009, 0.0f, 8.0f, 0, 0, 0, 0 },
  { "sine_fold20_2003",0, 2003, 0.0f, 20.0f, 0, 0, 0, 0 },
  { "saw_fold6_1009",  1, 1009, 1.0f, 6.0f, 0, 0, 0, 0 },
  { "mix_fold12_521",  2, 521,  0.5f, 12.0f, 0, 0, 0, 0 },
  { "am_saw_fold4",    1, 1010, 1.0f, 4.0f, 0, 0, 101, 1.0f },
  { "fm_sine",         0, 0,    0.0f, 1.0f, 101, 0.3f, 0, 0 },
  { "fm_sine_fold6",   0, 0,    0.0f, 6.0f, 101, 0.3f, 0, 0 },
  { "fm_saw_deep",     1, 0,    1.0f, 1.0f, 101, 0.6f, 0, 0 },
};

// Carrier base frequency so the FM signal repeats every 1/fm: mean frequency =
// f0 * I0(depth*5*ln2) must be a whole multiple K of fm.
static double besselI0(double x) { double s = 1, t = 1; for (int k = 1; k < 60; k++) { t *= (x / 2) * (x / 2) / (k * (double)k); s += t; } return s; }

int main(int argc, char** argv) {
  const float sr = 48000.0f;
  for (const Case& c : cases) {
    ComplexOscillator osc;
    osc.init(sr);
    osc.setWave(c.wave);
    double f0 = c.f0;
    if (c.fm > 0) f0 = c.fm * 9.0 / besselI0(c.fmDepth * 5.0 * log(2.0));  // K = 9: mean 909 Hz
    std::string fn = std::string(argv[1]) + "/" + c.name + ".f32";
    FILE* f = fopen(fn.c_str(), "wb");
    const int N = 48000 * 2;  // 2 s: the analysis skips the first second
    for (int n = 0; n < N; n++) {
      double tt = n / (double)sr;
      float freq = (float)f0, am = 1.0f;
      if (c.fm > 0) freq = (float)(f0 * exp2(sin(2 * M_PI * c.fm * tt) * c.fmDepth * 5.0));
      if (c.am > 0) am = (float)((1.0 + sin(2 * M_PI * c.am * tt) * c.amDepth) / (1.0 + c.amDepth));
      if (freq > sr * 0.45f) freq = sr * 0.45f;
      float y = osc.process(freq, c.timbre, c.fold, am);
      fwrite(&y, 4, 1, f);
    }
    fclose(f);
  }
  printf("%s\n", argv[1]);
}
