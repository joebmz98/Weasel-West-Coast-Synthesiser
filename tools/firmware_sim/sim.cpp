// WEASEL firmware simulator: runs the real sketch's setup(), loop() and
// AudioCallback() on a computer, against a model of the panel hardware (pot muxes,
// 74HC165 chain, patch bay, AW9523s, MIDI UART), from a scripted session.
//   sim <session.sc> <out.f32> <leds.log>
// out.f32: stereo float samples at 48 kHz. leds.log: every I2C write, with its time.
// Time only moves one audio block (64 samples) per step, so runs are deterministic.
// Session format: make_scenarios.py. Built and run by run.sh.
#include "Arduino.h"
#include "DaisyDuino.h"
#include "Wire.h"
#include <stdio.h>
#include <vector>
#include <string>

HardwareSerial Serial1; Stream Serial; Stream SerialUSB; TwoWire Wire; AudioClass DAISY;
void setup(); void loop();

namespace sim {
uint64_t nowUs = 0;
DaisyDuinoCallback audioCb = nullptr;
int pins[64] = {0};
int pots[32] = {0};
uint32_t buttons = 0;             // SR1..SR3 bits (see ButtonBit)
bool patch[4][7] = {};
uint32_t latched = 0; int shiftIdx = 0;
FILE* ledLog = nullptr;
const int SRC_PINS[4] = {17, 18, 19, 20};

void pinWrite(int pin, int v) {
  int old = pins[pin]; pins[pin] = v;
  if (pin == 7 && old && !v) {  // SH/LD low: parallel load
    uint32_t w = buttons;
    for (int d = 0; d < 7; d++)
      for (int s = 0; s < 4; s++)
        if (pins[SRC_PINS[s]] && patch[s][d]) w |= 1u << d;
    latched = w; shiftIdx = 0;
  }
  if (pin == 8 && !old && v) shiftIdx++;  // CLK rising: next bit
}
int pinRead(int pin) {
  if (pin == 9) return shiftIdx < 32 ? (latched >> (31 - shiftIdx)) & 1 : 0;
  return pins[pin];
}
int adcRead(int pin) {
  int ch = pins[0] | pins[1] << 1 | pins[2] << 2 | pins[3] << 3;
  return pin == A0 ? pots[ch] : pots[16 + ch];
}
void i2cByte(uint8_t addr, uint8_t b, bool start) {
  if (!ledLog) return;
  if (start) fprintf(ledLog, "\n%llu %02X:", (unsigned long long)(nowUs / 1000), addr);
  else fprintf(ledLog, " %02X", b);
}
}

// Scenario lines:  <ms> pot <index> <0..1>  |  <ms> btn <bit> <0|1>  |  <ms> patch <src> <dst> <0|1>
//                  <ms> midi <hex bytes...>  |  <ms> end
struct Ev { double ms; std::string kind; std::vector<double> a; };

int main(int argc, char** argv) {
  srand(1);
  std::vector<Ev> evs;
  FILE* f = fopen(argv[1], "r");
  char line[512];
  while (fgets(line, sizeof line, f)) {
    if (line[0] == '#' || line[0] == '\n') continue;
    Ev e; char kind[32]; int n = 0;
    if (sscanf(line, "%lf %31s%n", &e.ms, kind, &n) < 2) continue;
    e.kind = kind;
    char* p = line + n; char* end;
    for (;;) {
      double v = (e.kind == "midi") ? (double)strtol(p, &end, 16) : strtod(p, &end);
      if (end == p) break;
      e.a.push_back(v); p = end;
    }
    evs.push_back(e);
  }
  fclose(f);
  FILE* out = fopen(argv[2], "wb");
  sim::ledLog = fopen(argv[3], "w");

  // events at time 0 are the power-up panel
  size_t ei = 0;
  auto apply = [&](double ms) {
    while (ei < evs.size() && evs[ei].ms <= ms) {
      Ev& e = evs[ei++];
      if (e.kind == "pot") sim::pots[(int)e.a[0]] = (int)(e.a[1] * 65535.0 + 0.5);
      else if (e.kind == "btn") { uint32_t m = 1u << (int)e.a[0]; sim::buttons = e.a[1] ? (sim::buttons | m) : (sim::buttons & ~m); }
      else if (e.kind == "patch") sim::patch[(int)e.a[0]][(int)e.a[1]] = e.a[2] != 0;
      else if (e.kind == "midi") for (double b : e.a) Serial1.push((uint8_t)b);
    }
  };
  double endMs = evs.empty() ? 1000 : evs.back().ms;
  apply(0);
  setup();
  const int BS = 64;
  float l[BS], r[BS], il[BS] = {0}, ir[BS] = {0};
  float* ins[2] = {il, ir}; float* outs[2] = {l, r};
  double blockUs = BS * 1e6 / 48000.0, tUs = 0;
  while (sim::nowUs / 1000.0 < endMs) {
    apply(sim::nowUs / 1000.0);
    loop();
    sim::audioCb(ins, outs, BS);
    for (int i = 0; i < BS; i++) { fwrite(&l[i], 4, 1, out); fwrite(&r[i], 4, 1, out); }
    tUs += blockUs; sim::nowUs = (uint64_t)tUs;
  }
  fclose(out); fclose(sim::ledLog);
  return 0;
}
