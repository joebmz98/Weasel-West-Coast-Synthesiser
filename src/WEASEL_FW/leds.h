// =============================================================================
// leds.h — 4x AW9523 LED drivers over I2C
//
// The panel code builds a whole frame (clear(), then set() for every lit LED),
// and flush() writes only the chips whose outputs changed.
// =============================================================================
#pragma once
#include "Arduino.h"
#include <Wire.h>
#include "config.h"
#include "hardware.h"

namespace weasel
{
class Leds
{
 public:
  // 400 kHz cuts the time LED updates block the main loop by 4x. The AW9523
  // supports 400 kHz; drop back to 100000 if the LEDs misbehave (long wires /
  // weak pull-ups).
  static const uint32_t kI2cClockHz = 400000;

  // LED current (LED mode only, WEASEL_LED_SINK_WIRING 1). The AW9523 sinks a
  // regulated current in LED mode, so brightness is set here, not by the series
  // resistors. Max per pin = 37 mA scaled by ISEL:
  //   0 = 37 mA, 1 = 27.75 mA, 2 = 18.5 mA, 3 = 9.25 mA
  // Pin current = Imax * brightness / 256. ISEL 3 + 128 gives ~4.6 mA, roughly half brightness.
  static const uint8_t kLedCurrentRange = 3;
  uint8_t brightness = 128;  // 0-255; call forceWrite() after changing it

  bool chipPresent[kLedChipCount] = { false };

  WEASEL_NOINLINE void begin()
  {
    Wire.begin();
    Wire.setClock(kI2cClockHz);  // must come after begin() - begin() resets it to 100 kHz
    for (int chip = 0; chip < kLedChipCount; chip++)
    {
      chipPresent[chip] = initChip(kLedChipAddress[chip]);
#if WEASEL_DEBUG
      if (!chipPresent[chip])
      {
        Serial.print("AW9523 not found at 0x");
        Serial.println(kLedChipAddress[chip], HEX);
      }
#endif
    }
    force_ = true;
  }

  void clear() { memset(frame_, 0, sizeof(frame_)); }

  // Lights entry `index` of a group (out-of-range indexes are ignored)
  inline void set(const LedLocation* group, int size, int index)
  {
    if (index < 0 || index >= size) return;
    const LedLocation& l = group[index];
    frame_[l.chip][l.port] |= (1 << l.bit);
  }
  template <int N>
  inline void set(const LedLocation (&group)[N], int index) { set(group, N, index); }

  void forceWrite() { force_ = true; }

  // Writes the chips whose outputs changed. `idle` runs after each chip write
  // (each is ~0.5 ms of I2C: don't let MIDI clock wait behind all four).
  WEASEL_NOINLINE void flush(void (*idle)())
  {
    for (int chip = 0; chip < kLedChipCount; chip++)
    {
      if (!chipPresent[chip]) continue;
      if (!force_ && frame_[chip][0] == written_[chip][0] && frame_[chip][1] == written_[chip][1]) continue;

      const uint8_t p0 = frame_[chip][0];
      const uint8_t p1 = frame_[chip][1];
      Wire.beginTransmission(kLedChipAddress[chip]);
#if !WEASEL_LED_SINK_WIRING
      Wire.write(kRegOutput);
      Wire.write(p0);
      Wire.write(p1);
#else
      // DIM register order: 0x20-0x23 = P1_0..P1_3, 0x24-0x2B = P0_0..P0_7, 0x2C-0x2F = P1_4..P1_7
      Wire.write(kRegDimBase);
      for (int b = 0; b < 4; b++) Wire.write((p1 >> b) & 1 ? brightness : 0);
      for (int b = 0; b < 8; b++) Wire.write((p0 >> b) & 1 ? brightness : 0);
      for (int b = 4; b < 8; b++) Wire.write((p1 >> b) & 1 ? brightness : 0);
#endif
      if (Wire.endTransmission() == 0)
      {
        written_[chip][0] = frame_[chip][0];
        written_[chip][1] = frame_[chip][1];
      }
      idle();
    }
    force_ = false;
  }

 private:
  static const uint8_t kRegOutput = 0x02;
  static const uint8_t kRegConfig = 0x04;
  static const uint8_t kRegGlobalCtrl = 0x11;
  static const uint8_t kRegLedModeP0 = 0x12;  // 0 = LED (constant-current sink) mode, 1 = GPIO
  static const uint8_t kRegDimBase = 0x20;    // 0x20..0x2F, one 0-255 current level per pin
  static const uint8_t kRegSoftwareReset = 0x7F;

  static bool initChip(uint8_t address)
  {
    Wire.beginTransmission(address);
    Wire.write(kRegSoftwareReset);
    Wire.write(0x00);
    if (Wire.endTransmission() != 0) return false;

    delay(10);

#if !WEASEL_LED_SINK_WIRING
    // Prototype: all pins GPIO outputs (default after reset), Port 0 push-pull, all off
    Wire.beginTransmission(address);
    Wire.write(kRegGlobalCtrl);
    Wire.write(1 << 4);  // GPOMD: Port 0 push-pull (Port 1 is always push-pull)
    if (Wire.endTransmission() != 0) return false;

    Wire.beginTransmission(address);
    Wire.write(kRegConfig);
    Wire.write(0x00);
    Wire.write(0x00);
    if (Wire.endTransmission() != 0) return false;

    Wire.beginTransmission(address);
    Wire.write(kRegOutput);
    Wire.write(0x00);
    Wire.write(0x00);
    if (Wire.endTransmission() != 0) return false;

    return true;
#else
    // Global control: current range (ISEL bits 1:0). GPOMD is irrelevant in LED mode.
    Wire.beginTransmission(address);
    Wire.write(kRegGlobalCtrl);
    Wire.write(kLedCurrentRange & 0x03);
    if (Wire.endTransmission() != 0) return false;

    // All 16 pins to LED mode (0x12 and 0x13, auto-increment)
    Wire.beginTransmission(address);
    Wire.write(kRegLedModeP0);
    Wire.write(0x00);
    Wire.write(0x00);
    if (Wire.endTransmission() != 0) return false;

    // All LEDs off
    Wire.beginTransmission(address);
    Wire.write(kRegDimBase);
    for (int i = 0; i < 16; i++) Wire.write(0x00);
    if (Wire.endTransmission() != 0) return false;

    return true;
#endif
  }

  uint8_t frame_[kLedChipCount][2] = { { 0 } };    // desired state [chip][port]
  uint8_t written_[kLedChipCount][2] = { { 0 } };  // last state sent to the chip
  bool force_ = true;
};
}  // namespace weasel
