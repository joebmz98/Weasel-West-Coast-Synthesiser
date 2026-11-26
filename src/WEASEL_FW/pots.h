// =============================================================================
// pots.h — 32 pots on 2x CD74HC4067 muxes, with noise filtering
//
// Both muxes are switched to the same channel together and then both ADCs are
// read. Current schematic: U4 and U5 share MUX1_S0..S3 (D4/D5/D6/D10 are
// unconnected on the board). Prototype boards: U5 still has its own select
// lines, so those are driven to the same channel as well - the same code works
// on both boards.
//
// Noise filtering: a one-pole low-pass on the raw ADC value, then hysteresis so
// a pot at rest stops twitching, and snapping at both ends so minimum and
// maximum are exact (e.g. wavefolder gain sits at exactly 1.0 and morph mix at
// exactly 0 with the pot down).
// =============================================================================
#pragma once
#include "Arduino.h"
#include "config.h"
#include "hardware.h"

namespace weasel
{
class Pots
{
 public:
  static const int kPerMux = 16;
  static const int kCount = 32;  // 2 muxes * 16 channels

  // Noise filter on the raw ADC, per 5 ms scan. Higher = less lag, more noise.
  // 0.5 settles in ~15 ms (total glide = this + kParamSmoothMs).
  static constexpr float kFilterCoeff = 0.5f;
  // Hysteresis only applies to a pot at rest: once a pot moves past kHysteresis
  // it tracks every scan (no stepping) until it has been still for kActiveMs.
  static const int kHysteresis = 48;          // 16-bit LSBs (~0.07% of travel)
  static const unsigned long kActiveMs = 300;  // keep tracking this long after the last movement
  static const int kEndSnap = 200;             // dead zone at each end of travel

  void begin()
  {
    for (int i = 0; i < 4; i++)
    {
      pinMode(pins::kMux1Sel[i], OUTPUT);
      digitalWrite(pins::kMux1Sel[i], LOW);
    }
    for (int i = 0; i < 4; i++)
    {
      pinMode(pins::kMux2Sel[i], OUTPUT);
      digitalWrite(pins::kMux2Sel[i], LOW);
    }
    pinMode(pins::kMux1Sig, INPUT);
    pinMode(pins::kMux2Sig, INPUT);
  }

  // Reads every pot. `idle` is called every few channels (keeps MIDI drained).
  WEASEL_NOINLINE void scan(void (*idle)())
  {
    for (int ch = 0; ch < kPerMux; ch++)
    {
      selectChannel(ch, pins::kMux1Sel);
      selectChannel(ch, pins::kMux2Sel);  // prototype only, harmless otherwise
      delayMicroseconds(20);              // settle after switching the mux
      int raw1 = analogRead(pins::kMux1Sig);
      int raw2 = analogRead(pins::kMux2Sig);
      filter(ch, raw1);
      filter(ch + kPerMux, raw2);
      if ((ch & 3) == 3) idle();  // keep MIDI clock timing tight during the scan
#if WEASEL_DEBUG
      debugJump(ch, raw1);
      debugJump(ch + kPerMux, raw2);
#endif
    }
    primed_ = true;
  }

  // 0..1
  inline float norm(int index) const { return values_[index] * (1.0f / 65535.0f); }

 private:
  static void selectChannel(int channel, const uint32_t* sel)
  {
    digitalWrite(sel[0], bitRead(channel, 0));
    digitalWrite(sel[1], bitRead(channel, 1));
    digitalWrite(sel[2], bitRead(channel, 2));
    digitalWrite(sel[3], bitRead(channel, 3));
  }

  inline void filter(int i, int raw)
  {
    if (!primed_) filtered_[i] = (float)raw;
    else filtered_[i] += kFilterCoeff * ((float)raw - filtered_[i]);

    int v = (int)(filtered_[i] + 0.5f);
    if (v < kEndSnap) v = 0;
    else if (v > 65535 - kEndSnap) v = 65535;

    unsigned long now = millis();
    if (abs(v - values_[i]) >= kHysteresis)
    {
      activeUntil_[i] = now + kActiveMs;  // pot is being turned
    }
    if (v == 0 || v == 65535 || (long)(activeUntil_[i] - now) > 0)
    {
      values_[i] = v;  // track continuously while moving
    }
  }

#if WEASEL_DEBUG
  // Logs any single-scan jump in a raw pot reading larger than ~6% of travel.
  // Turning a pot quickly will show up too; jumps while nobody touches the panel
  // are glitches (mux select line, ADC noise, a bad pot or wiring).
  void debugJump(int index, int raw)
  {
    static const char* const kNames[kCount] = {
      "SEQ STEP 1", "SEQ STEP 2", "SEQ STEP 3", "SEQ STEP 4", "SEQ STEP 5",
      "ENV ATTACK", "ENV SUSTAIN", "ENV DECAY", "PULSAR CTRL", "PULSAR PERIOD",
      "MODOSC FREQ CTRL", "MODOSC FREQ", "MODOSC FINE", "MODOSC MOD CTRL", "MODOSC MOD",
      "COMPOSC FREQ CTRL",
      "COMPOSC FREQ", "COMPOSC FINE", "COMPOSC WF CTRL", "COMPOSC WF", "COMPOSC TIMBRE",
      "LPG1 CTRL", "LPG1 LEVEL", "LPG2 CTRL", "LPG2 LEVEL", "CLOCK", "REVERB MIX",
      "MASTER VOLUME", "MUX2 C12", "MUX2 C13", "MUX2 C14", "MUX2 C15"
    };
    if (debugPrimed_ && abs(raw - debugLastRaw_[index]) > 4000)
    {
      Serial.print("Pot jump: ");
      Serial.print(kNames[index]);
      Serial.print(index < kPerMux ? " (MUX1 C" : " (MUX2 C");
      Serial.print(index % kPerMux);
      Serial.print(") ");
      Serial.print(debugLastRaw_[index]);
      Serial.print(" -> ");
      Serial.println(raw);
    }
    debugLastRaw_[index] = raw;
    if (index == kCount - 1) debugPrimed_ = true;
  }
  int debugLastRaw_[kCount] = { 0 };
  bool debugPrimed_ = false;
#endif

  int values_[kCount] = { 0 };
  float filtered_[kCount] = { 0 };
  unsigned long activeUntil_[kCount] = { 0 };
  bool primed_ = false;
};
}  // namespace weasel
