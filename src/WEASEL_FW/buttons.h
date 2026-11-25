// =============================================================================
// buttons.h — panel button scanning: debounce, act-on-release, MIDI shift
//
// BUTTONS act on RELEASE (press then let go), so a button can also be held as a
// shift. The manual pulsar trigger still fires on press, as a performance trigger.
//
// MIDI SHIFT: hold MIDI for kShiftHoldMs and the LPG 1 and LPG 2 mode LEDs flash.
// While they flash, LPG 1 / LPG 2 toggle whether the complex / modulation
// oscillator follows the MIDI note's pitch (flashing = follows, steady = ignores
// MIDI pitch). The change applies when MIDI is released, and that release does
// NOT toggle MIDI on/off. Any other button pressed while MIDI is held is ignored,
// and also cancels the MIDI toggle.
//
// Buttons held at power-up do nothing until they have been let go.
//
// This class only decides WHEN a button acts; what it does is the onAction
// callback (panel.h), and the pitch-follow setting lives in the web editor
// (getPitchExclude / setPitchExclude). Kept free of the rest of the synth so
// tools/button_test can run it on a computer.
// =============================================================================
#pragma once
#include "Arduino.h"
#include "config.h"
#include "hardware.h"

namespace weasel
{
class Buttons
{
 public:
  static const unsigned long kReadIntervalMs = 10;  // -> 10-20 ms effective debounce
  static const unsigned long kShiftHoldMs = 500;    // hold MIDI this long to enter the shift
  static const unsigned long kShiftFlashMs = 150;   // LPG LED flash half-period
  static const uint8_t kPitchExcludeComplex = 1;     // = PITCH_EXCLUDE_COMPLEX (WeaselEditor.h)
  static const uint8_t kPitchExcludeModulation = 2;  // = PITCH_EXCLUDE_MODULATION

  void (*onAction)(uint8_t bit) = nullptr;
  uint8_t (*getPitchExclude)() = nullptr;
  void (*setPitchExclude)(uint8_t mask) = nullptr;

  bool armed = false;  // false during setup(): track state, take no actions

  // Shift state, shown by the LEDs
  bool shiftActive = false;       // held long enough: LPG LEDs flashing, LPG buttons edit
  uint8_t shiftPitchExclude = 0;  // pending pitch-follow mask

  inline bool isDown(uint8_t bit) const { return (debounced_ >> bit) & 1UL; }

  // One scan of the shift registers (all 32 bits)
  WEASEL_NOINLINE void update(uint32_t raw)
  {
    raw &= kButtonMask;

    // Accept a new state only once two consecutive reads agree
    if (raw == rawPrev_) debounced_ = raw;
    rawPrev_ = raw;

    uint32_t pressed = debounced_ & ~lastHandled_;   // rising edges
    uint32_t released = ~debounced_ & lastHandled_;  // falling edges
    lastHandled_ = debounced_;

    if (!armed)
    {
      // setup(): anything down now is ignored until it has been let go
      consumed_ = debounced_;
      pressed = released = 0;
    }

    // -- presses --
    while (pressed)
    {
      uint8_t bit = __builtin_ctz(pressed);
      pressed &= pressed - 1;

      if (bit == kShiftButton)
      {
        shiftDownAt_ = millis();
        shiftActive = false;
        shiftUsed_ = false;
        shiftPitchExclude = getPitchExclude();
        continue;
      }
      if (shiftHeld() && ((1UL << bit) & kShiftableMask))
      {
        // Pressed while MIDI is held: a shift combo, never a normal press or release
        shiftUsed_ = true;
        consumed_ |= (1UL << bit);
        if (shiftActive)
        {
          if (bit == BTN_LPG1_MODE) shiftPitchExclude ^= kPitchExcludeComplex;
          else if (bit == BTN_LPG2_MODE) shiftPitchExclude ^= kPitchExcludeModulation;
        }
        continue;
      }
      if (bit == BTN_PULSAR_MANUAL) onAction(bit);  // performance trigger: on press
    }

    // -- MIDI shift: held long enough? --
    if (shiftHeld() && !shiftActive && millis() - shiftDownAt_ >= kShiftHoldMs)
    {
      shiftActive = true;
    }

    // -- releases --
    while (released)
    {
      uint8_t bit = __builtin_ctz(released);
      released &= released - 1;
      uint32_t mask = 1UL << bit;

      if (consumed_ & mask)
      {  // part of a combo, or held since power-up
        consumed_ &= ~mask;
        if (bit == kShiftButton) shiftActive = false;
        continue;
      }
      if (bit == kShiftButton)
      {
        if (shiftActive || shiftUsed_)
        {
          // End of a shift: apply the pitch-follow choice, leave MIDI on/off alone
          if (shiftPitchExclude != getPitchExclude()) setPitchExclude(shiftPitchExclude);
        }
        else
        {
          onAction(bit);  // a plain press and release: toggle MIDI
        }
        shiftActive = false;
        shiftUsed_ = false;
        continue;
      }
      if (bit == BTN_PULSAR_MANUAL) continue;  // already fired on press
      if (!(mask & kShiftableMask)) continue;  // latching step switches: read by level (stepEnabled)
      onAction(bit);
    }
  }

  // Sequencer step switch i (0..4): is the step on? (latching switches, by level)
  inline bool stepEnabled(int i) const
  {
#if SEQ_STEP_ENABLED_WHEN_HIGH
    return isDown(kSeqStepBits[i]);
#else
    return !isDown(kSeqStepBits[i]);
#endif
  }

 private:
  // Buttons that can take part in a shift combo (not the latching step switches)
#if SEQ_STEP_SWITCHES_LATCHING
  static const uint32_t kShiftableMask = kButtonMask & ~((1UL << BTN_SEQ_STEP1) | (1UL << BTN_SEQ_STEP2) | (1UL << BTN_SEQ_STEP3) | (1UL << BTN_SEQ_STEP4) | (1UL << BTN_SEQ_STEP5));
#else
  static const uint32_t kShiftableMask = kButtonMask;
#endif

  // MIDI is down as a shift (not a button held since power-up)
  inline bool shiftHeld() const { return isDown(kShiftButton) && !(consumed_ & (1UL << kShiftButton)); }

  uint32_t rawPrev_ = 0;      // previous raw read (for 2-read debounce)
  uint32_t debounced_ = 0;    // stable button state
  uint32_t lastHandled_ = 0;  // state at last edge-processing pass
  uint32_t consumed_ = 0;     // buttons whose release must do nothing (used in a shift combo, or held at boot)
  unsigned long shiftDownAt_ = 0;
  bool shiftUsed_ = false;    // another button was pressed during this hold
};
}  // namespace weasel
