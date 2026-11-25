// =============================================================================
// midi_input.h — TRS MIDI in (6N138 on USART1): notes, clock, transport
//
// MIDI is read in the main loop (and between the slow panel scans, see
// drainMidi()); the handlers update synth.midi / synth.midiClock.
// Channel filtering follows the web editor's receive channel setting.
// =============================================================================
#pragma once
#include "Arduino.h"
#include <MIDI.h>
#include "config.h"
#include "hardware.h"
#include "synth.h"
#include "WeaselEditor.h"

// Created in WEASEL_FW.ino with MIDI_CREATE_INSTANCE(HardwareSerial, Serial1, MIDI)
extern midi::MidiInterface<midi::SerialMIDI<HardwareSerial>> MIDI;

namespace weasel
{
extern Editor editor;

// MIDI.read() parses one byte per call, and returns false until a message is
// complete, so `while (MIDI.read())` stopped at the first byte of every note and
// took three passes of the main loop to read one. Read everything that has
// arrived instead. Also passed to the slow panel scans as their idle callback.
inline void drainMidi()
{
  while (Serial1.available())
  {
    MIDI.read();
  }
}

// The audio callback runs inside the SAI DMA interrupt for a whole block (~1.3 ms
// at 64 samples), which DaisyDuino sets to priority 0. The MIDI UART interrupt
// was priority 1, so it couldn't run during the callback, and the UART only holds
// one received byte. At 31250 baud a new byte arrives every 320 us, so bytes
// landing mid-callback overwrote each other (UART overrun) and were lost: missed
// note-ons, missed note-offs (stuck notes). Give the UART the top priority and
// drop the audio DMA one step. The UART interrupt only copies one byte into a
// buffer, so audio timing is unaffected. Call after DAISY.begin().
inline void configureMidiInterruptPriority()
{
  const IRQn_Type audioDma[] = { DMA1_Stream0_IRQn, DMA1_Stream1_IRQn,    // SAI1
                                 DMA1_Stream3_IRQn, DMA1_Stream4_IRQn };  // SAI2
  for (unsigned i = 0; i < sizeof(audioDma) / sizeof(audioDma[0]); i++)
  {
    HAL_NVIC_SetPriority(audioDma[i], 1, 0);
  }
  HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
}

#if WEASEL_DEBUG
inline void debugPrintNote(const char* label, byte channel, byte note, byte velocity)
{
  static const char* const kNoteNames[12] = { "C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B" };
  Serial.print(label);
  Serial.print(" ch");
  Serial.print(channel);
  Serial.print(" ");
  Serial.print(kNoteNames[note % 12]);
  Serial.print(note / 12 - 1);
  Serial.print(" (");
  Serial.print(note);
  Serial.print(") vel ");
  Serial.println(velocity);
}

// WEASEL ignores SysEx. Seeing it here with a controller that doesn't send SysEx
// means bytes are being corrupted on the MIDI input (e.g. clock F8 arriving as F0).
inline void handleMidiSysExDebug(byte* data, unsigned size)
{
  Serial.print("MIDI SysEx (");
  Serial.print(size);
  Serial.println(" bytes) - if your controller doesn't send SysEx, the MIDI input is corrupting bytes");
}
#endif

// -- handlers --
inline void handleNoteOn(byte channel, byte note, byte velocity)
{
  if (!editor.acceptsChannel(channel)) return;  // MIDI receive channel (web editor)
  if (velocity > 0) editor.noteIn(channel, note, velocity);
  else editor.activity();
#if WEASEL_DEBUG
  debugPrintNote(velocity > 0 ? "MIDI note on " : "MIDI note off", channel, note, velocity);
#endif
  if (velocity > 0) synth.midi.noteOn(note);
  else synth.midi.noteOff(note);  // note-on with velocity 0 = note-off
}

inline void handleNoteOff(byte channel, byte note, byte velocity)
{
  if (!editor.acceptsChannel(channel)) return;
  editor.activity();
#if WEASEL_DEBUG
  debugPrintNote("MIDI note off", channel, note, velocity);
#endif
  synth.midi.noteOff(note);
}

// CC 120 (All Sound Off) and CC 123 (All Notes Off): clear any stuck notes
inline void handleControlChange(byte channel, byte number, byte value)
{
  if (!editor.acceptsChannel(channel)) return;
  if (number == 120 || number == 123)
  {
    synth.midi.allOff();
#if WEASEL_DEBUG
    Serial.println("MIDI all notes off");
#endif
  }
}

inline void handleMidiClock()  // 24 ticks per quarter note
{
  synth.midiClock.tickReceived(micros(), millis());
}

inline void handleMidiStart()
{
  if (synth.midiClock.sync)
  {
    synth.midiClock.start();  // the audio callback restarts on step 1 at the next tick
  }
  else
  {
    synth.seq.currentStep = 0;
    synth.seq.sampleCounter = 0;
  }
}

inline void handleMidiStop()
{
  synth.seq.sampleCounter = 0;
  synth.midiClock.stop();  // synced: hold until Start / Continue (clock-only sources never send Stop)
}

inline void handleMidiContinue()
{
  synth.midiClock.cont();
}

WEASEL_NOINLINE inline void beginMidi()
{
  Serial1.setRx(pins::kMidiRx);
  Serial1.begin(31250);
  Serial1.flush();

  MIDI.setHandleNoteOn(handleNoteOn);
  MIDI.setHandleNoteOff(handleNoteOff);
  MIDI.setHandleClock(handleMidiClock);
  MIDI.setHandleStart(handleMidiStart);
  MIDI.setHandleStop(handleMidiStop);
  MIDI.setHandleContinue(handleMidiContinue);
  MIDI.setHandleControlChange(handleControlChange);
#if WEASEL_DEBUG
  MIDI.setHandleSystemExclusive(handleMidiSysExDebug);
#endif
  MIDI.begin(MIDI_CHANNEL_OMNI);  // channel filtering is done in the handlers (web editor setting)
  MIDI.turnThruOff();
}
}  // namespace weasel
