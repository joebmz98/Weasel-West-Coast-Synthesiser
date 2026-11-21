// Host-side check of the WEASEL LPG model (src/WEASEL_FW/buchla_lpg.cpp).
// Compares each mode against the Parker & D'Angelo analytic transfer function,
// prints the resonance peak, ping/gate envelopes, loudness vs panel control and a
// stress test. Build on a desktop:
//   g++ -O2 -I../../src/WEASEL_FW lpg_test.cpp ../../src/WEASEL_FW/buchla_lpg.cpp -o lpg_test
#include <initializer_list>
#include "buchla_lpg.h"
#include <cmath>
#include <complex>
#include <cstdio>
#include <vector>
using namespace weasel;
const float SR = 48000;
typedef std::complex<double> cd;

double analytic(double rf, double ra, double c3, double a, double f) {
  double C1 = 1e-9, C2 = 220e-12;
  double w = 2 * SR * tan(M_PI * f / SR);  // bilinear warp
  cd s(0, w);
  double a1 = 1 + 2 * rf / ra;
  double a2 = rf * (2 * C1 + C2 - c3 * (a - 1) + (C2 + c3) * rf / ra);
  double a3 = rf * rf * C1 * (C2 + c3);
  return 20 * log10(std::abs(1.0 / (a1 + a2 * s + a3 * s * s)));
}

double measure(BuchlaLpg::Mode m, float res, float cv, float f, float* rfOut) {
  BuchlaLpg l; l.Init(SR); l.SetMode(m); l.SetResonance(res);
  for (int i = 0; i < 48000; i++) l.Process(cv, 0);  // settle vactrol
  int N = 48000; double a = 0, b = 0;
  for (int i = 0; i < N; i++) {
    float x = sinf(2 * M_PI * f * i / SR); float y = l.Process(cv, x);
    if (i > N / 2) { a += x * x; b += y * y; }
  }
  *rfOut = l.GetResistance();
  return 20 * log10(sqrt(b / a) + 1e-15);
}

int main() {
  const char* names[] = {"VCA", "LOPASS", "BOTH"};
  for (int m = 0; m < 3; m++) {
    float res = (m == 1) ? 0.5f : 0.f;
    double ra = (m == 0) ? 5e3 : 5e6, c3 = (m == 1) ? 4.7e-9 : 0;
    printf("== %s (res %.2f) ==  model / analytic dB\n", names[m], res);
    for (float cv : {0.f, 2.f, 4.f, 6.f, 8.f, 10.7f}) {
      printf("cv %4.1f ", cv);
      float rf = 0;
      for (float f : {100.f, 1000.f, 5000.f, 15000.f}) {
        double g = measure((BuchlaLpg::Mode)m, res, cv, f, &rf);
        double C2 = 220e-12;
        double amax = c3 > 0 ? (2 * 1e-9 * ra + (C2 + c3) * (ra + rf)) / (c3 * ra) : 0;
        double makeup = 20 * log10((ra + 2 * 1450.1) / ra);
        double an = analytic(rf, ra, c3, res * amax, f) + makeup;
        printf(" %7.1f/%7.1f", g, an);
      }
      printf("   R=%.3g\n", rf);
    }
  }
  // Lopass resonance peak vs res at cv 5
  printf("== LOPASS peak (cv 5V) ==\n");
  for (float res : {0.f, 0.25f, 0.5f, 0.75f, 0.9f}) {
    double best = -999, bf = 0; float rf;
    for (float f = 50; f < 20000; f *= 1.05f) { double g = measure(BuchlaLpg::MODE_LOPASS, res, 5.f, f, &rf); if (g > best) { best = g; bf = f; } }
    printf("res %.2f peak %+.1f dB @ %.0f Hz\n", res, best, bf);
  }
  // Ping and gate envelopes in BOTH mode, 440 Hz sine
  for (int pulse : {96, 4800}) {
    BuchlaLpg l; l.Init(SR);
    printf("== BOTH, %d-sample 10V pulse, peak per 25ms ==\n", pulse);
    for (int b = 0; b < 32; b++) {
      float p = 0;
      for (int i = b * 1200; i < (b + 1) * 1200; i++) { float y = l.Process(i < pulse ? 10.f : 0.f, sinf(2 * M_PI * 440 * i / SR)); p = fmaxf(p, fabsf(y)); }
      printf("%.3f ", p);
    }
    printf("\n");
  }
  // Loudness of a 220Hz naive saw vs pot (ctrl 0..1 -> V = ctrl*10.7)
  printf("== loudness vs ctrl (V = 10.7*ctrl), 220Hz saw, dB ==\n");
  for (int m = 0; m < 3; m++) {
    printf("%-7s", names[m]);
    for (float c = 0; c <= 1.001f; c += 0.1f) {
      BuchlaLpg l; l.Init(SR); l.SetMode((BuchlaLpg::Mode)m); l.SetResonance(0.5f);
      double a = 0, b = 0; float ph = 0;
      for (int i = 0; i < 96000; i++) { float x = 2 * ph - 1; ph += 220 / SR; if (ph >= 1) ph -= 1; float y = l.Process(c * 10.7f, x); if (i > 48000) { a += x * x; b += y * y; } }
      printf("%6.1f", 10 * log10(b / a + 1e-15));
    }
    printf("\n");
  }
  // Mode switch / bad-float robustness: hammer with random cv and mode changes
  BuchlaLpg l; l.Init(SR); unsigned s = 1; float mx = 0;
  for (int i = 0; i < 480000; i++) {
    s = s * 1664525 + 1013904223; float cv = (s >> 8) / 16777216.f * 10.7f;
    if (i % 4800 == 0) { l.SetMode((BuchlaLpg::Mode)((i / 4800) % 3)); l.SetResonance(0.9f); }
    float y = l.Process(cv, sinf(i * 0.05f)); mx = fmaxf(mx, fabsf(y));
  }
  printf("stress max |out| = %.3f\n", mx);
}
