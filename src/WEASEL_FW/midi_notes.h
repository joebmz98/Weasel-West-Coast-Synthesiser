// =============================================================================
// midi_notes.h — held MIDI notes and the triggers they send
//
// Last-note priority: the gate only closes when the LAST held note is released,
// and releasing the newest note falls back to the previous one (pitch only, no
// retrigger). The MIDI handlers (midi_input.h) run in the main loop; the
// trigger flags are consumed by the audio callback.
// =============================================================================
#pragma once
#include <stdint.h>
#include "dsp_util.h"

namespace weasel
{
class MidiNotes
{
 public:
  static const uint8_t kMaxHeld = 16;

  bool follow = false;              // MIDI button: oscillators follow the note's pitch
  bool noteReceived = false;        // a note has arrived since MIDI follow was switched on
  volatile bool noteHeld = false;   // a note is physically held
  int currentNote = 60;             // middle C until a note arrives
  float noteRatio = 261.63f;        // transpose ratio of the current note relative to C4

  // One flag per module that can be MIDI-triggered. A note-on sets them all;
  // the audio callback clears those whose module isn't in MIDI mode.
  volatile bool seqTrigger = false;
  volatile bool envTrigger = false;
  volatile bool pulsarTrigger = false;
  volatile bool randomTrigger = false;

  void setFollow(bool on)
  {
    if (on && !follow) noteReceived = false;
    follow = on;
  }

  void noteOn(uint8_t note)
  {
    remove(note);                                  // a repeated note moves to the top
    if (heldCount_ == kMaxHeld) remove(held_[0]);  // forget the oldest
    held_[heldCount_++] = note;

    currentNote = note;
    seqTrigger = true;
    noteRatio = midiTransposeRatio(note);
    noteReceived = true;
    noteHeld = true;
    pulsarTrigger = true;
    randomTrigger = true;
    envTrigger = true;
  }

  void noteOff(uint8_t note)
  {
    bool wasNewest = (heldCount_ > 0 && held_[heldCount_ - 1] == note);
    remove(note);
    if (heldCount_ == 0)
    {
      noteHeld = false;
    }
    else if (wasNewest)
    {
      // Fall back to the previous held note: pitch only, the gate stays open, no retrigger
      currentNote = held_[heldCount_ - 1];
      noteRatio = midiTransposeRatio(currentNote);
    }
  }

  void allOff()
  {
    heldCount_ = 0;
    noteHeld = false;
  }

 private:
  void remove(uint8_t note)
  {
    for (uint8_t i = 0; i < heldCount_; i++)
    {
      if (held_[i] == note)
      {
        for (uint8_t j = i; j + 1 < heldCount_; j++) held_[j] = held_[j + 1];
        heldCount_--;
        return;
      }
    }
  }

  uint8_t held_[kMaxHeld];
  uint8_t heldCount_ = 0;
};
}  // namespace weasel
