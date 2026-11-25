// =============================================================================
// shift_registers.h — the 4x 74HC165 chain (panel buttons + patch bay inputs)
//
// One read returns all 32 bits: SR1 = bits 31..24, SR2 = 23..16, SR3 = 15..8,
// SR4 = 7..0 (see srBit() in hardware.h).
// =============================================================================
#pragma once
#include "Arduino.h"
#include "hardware.h"

namespace weasel
{
class ShiftRegisters
{
 public:
  void begin()
  {
    pinMode(pins::kShiftLoad, OUTPUT);
    pinMode(pins::kShiftClock, OUTPUT);
    pinMode(pins::kShiftData, INPUT);
    digitalWrite(pins::kShiftClock, LOW);
    digitalWrite(pins::kShiftLoad, HIGH);
  }

  // Latches the inputs and shifts all 32 bits out. QH is sampled before each clock pulse.
  uint32_t read()
  {
    digitalWrite(pins::kShiftLoad, LOW);
    delayMicroseconds(10);
    digitalWrite(pins::kShiftLoad, HIGH);
    delayMicroseconds(10);

    uint32_t allData = 0;
    for (int i = 31; i >= 0; i--)
    {
      if (digitalRead(pins::kShiftData) == HIGH)
      {
        allData |= (1UL << i);
      }
      digitalWrite(pins::kShiftClock, HIGH);
      delayMicroseconds(5);
      digitalWrite(pins::kShiftClock, LOW);
      delayMicroseconds(5);
    }
    return allData;
  }
};
}  // namespace weasel
