#pragma once
#include "Arduino.h"
// I2C stub: every AW9523 answers; writes are recorded as an LED log by the harness
namespace sim { void i2cByte(uint8_t addr, uint8_t b, bool start); }
class TwoWire {
  uint8_t addr_ = 0;
public:
  void begin() {}
  void setClock(uint32_t) {}
  void beginTransmission(uint8_t a) { addr_ = a; sim::i2cByte(a, 0, true); }
  size_t write(uint8_t b) { sim::i2cByte(addr_, b, false); return 1; }
  uint8_t endTransmission() { return 0; }
};
extern TwoWire Wire;
