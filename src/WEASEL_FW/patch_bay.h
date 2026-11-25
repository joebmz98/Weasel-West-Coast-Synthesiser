// =============================================================================
// patch_bay.h — 4 sources x 7 destinations, scanned through the shift registers
//
// Each source pin is driven HIGH in turn and the destinations are read on SR4
// D0-D6. connected[source][destination] is read by the audio callback.
//
// FLOATING-INPUT GUARD
// Each destination relies on its 10k pull-down to read LOW when no source drives
// it. If that resistor isn't connected (bad joint, broken track), the input keeps
// the charge from the last source that drove it, and the next sources in the scan
// read as "patched" too: ghost connections that trigger in time with the
// sequencer. So before each source is driven, every input is read with all
// sources LOW; a reading only counts as a connection if that input was LOW just
// before. An input already HIGH with nothing driving it is flagged faulty.
// =============================================================================
#pragma once
#include "Arduino.h"
#include "config.h"
#include "hardware.h"
#include "shift_registers.h"

namespace weasel
{
class PatchBay
{
 public:
  static const uint8_t kDebounceReads = 3;
  static const uint8_t kFaultClearScans = 50;  // clean scans (~1 s) before a fault flag clears

  // connected[source][destination]
  // Sources:      0 SEQ, 1 EG, 2 PLSR, 3 RNDM
  // Destinations: PatchDest (hardware.h)
  volatile bool connected[kPatchSourceCount][kPatchDestCount] = {};
  bool inputFaulty[kPatchDestCount] = { false };

  void begin()
  {
    for (int s = 0; s < kPatchSourceCount; s++)
    {
      pinMode(pins::kPatchSources[s], OUTPUT);
      digitalWrite(pins::kPatchSources[s], LOW);
    }
  }

  // `idle` is called between sources (keeps MIDI drained)
  WEASEL_NOINLINE void scan(ShiftRegisters& sr, void (*idle)())
  {
#if WEASEL_DEBUG
    static const char* const kSourceNames[kPatchSourceCount] = { "SEQ", "EG", "PLSR", "RNDM" };
    static const char* const kDestNames[kPatchDestCount] = { "PLSR_PERIOD", "MODOSC_FREQ", "MODOSC_MOD",
                                                             "COMP_PITCH", "COMP_WAVE", "LPG_LVL1", "LPG_LVL2" };
#endif
    for (int s = 0; s < kPatchSourceCount; s++)
    {
      digitalWrite(pins::kPatchSources[s], LOW);
    }
    delayMicroseconds(50);

    // Baseline with nothing driven: every healthy input reads LOW here
    uint8_t idleBits = sr.read() & 0xFF;
    uint8_t floatingSeen = idleBits;

    for (int s = 0; s < kPatchSourceCount; s++)
    {
      digitalWrite(pins::kPatchSources[s], HIGH);
      delayMicroseconds(100);

      uint8_t sr4 = sr.read() & 0xFF;

      for (int d = 0; d < kPatchDestCount; d++)
      {
        // Only a LOW -> HIGH change caused by this source counts as a connection
        bool reading = (sr4 & (1 << d)) && !(idleBits & (1 << d));
        if (reading)
        {
          if (counter_[s][d] < kDebounceReads) counter_[s][d]++;
        }
        else
        {
          if (counter_[s][d] > 0) counter_[s][d]--;
        }
        connected[s][d] = (counter_[s][d] >= kDebounceReads);
      }

      digitalWrite(pins::kPatchSources[s], LOW);
      delayMicroseconds(50);

      // Re-read with everything LOW again: the baseline for the next source
      if (s < kPatchSourceCount - 1)
      {
        idleBits = sr.read() & 0xFF;
        floatingSeen |= idleBits;
      }

      idle();
    }

    // Fault flags (with a hold-off so a floating input doesn't flicker on and off)
    for (int d = 0; d < kPatchDestCount; d++)
    {
      if (floatingSeen & (1 << d))
      {
        faultHold_[d] = kFaultClearScans;
      }
      else if (faultHold_[d] > 0)
      {
        faultHold_[d]--;
      }
      bool faulty = faultHold_[d] > 0;
#if WEASEL_DEBUG
      if (faulty != inputFaulty[d])
      {
        Serial.print("Patch input ");
        Serial.print(kDestNames[d]);
        Serial.println(faulty ? " reads HIGH with nothing driving it - check its 10k pull-down and solder joints"
                              : " OK again");
      }
#endif
      inputFaulty[d] = faulty;
    }

#if WEASEL_DEBUG
    for (int s = 0; s < kPatchSourceCount; s++)
    {
      for (int d = 0; d < kPatchDestCount; d++)
      {
        if (connected[s][d] != debugLast_[s][d])
        {
          Serial.print(kSourceNames[s]);
          Serial.print(connected[s][d] ? " -> " : " -x ");
          Serial.println(kDestNames[d]);
          debugLast_[s][d] = connected[s][d];
        }
      }
    }
#endif
  }

 private:
  uint8_t counter_[kPatchSourceCount][kPatchDestCount] = {};
  uint8_t faultHold_[kPatchDestCount] = { 0 };  // >0 = input recently seen floating
#if WEASEL_DEBUG
  bool debugLast_[kPatchSourceCount][kPatchDestCount] = {};
#endif
};
}  // namespace weasel
