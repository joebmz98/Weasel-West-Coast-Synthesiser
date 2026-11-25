// =============================================================================
// WEASEL — Buchla Music Easel inspired digital synth, Daisy Seed (Arduino IDE)
// designed by .axs instruments
//
// A complex oscillator, a modulation oscillator, an envelope generator, the
// pulsar, a 5-step sequential voltage source and a random voltage source, into
// two Buchla 292 low pass gates; a 4 x 7 patch bay routes the four sources to
// seven destinations, simulating the modular experience.
//
// Board setup (Arduino IDE): see README.md next to this file.
//   Board             : STM32 MCU based boards -> Generic STM32H7 Series
//   Board part number : Daisy Seed
//   USB support       : CDC (generic 'Serial' supersede U(S)ART)  (web editor)
//   Libraries         : DaisyDuino, MIDI Library
//
// Two threads of execution:
//
//   AUDIO CALLBACK (audio.h, interrupt, 64 samples = 1.33 ms)
//     sequencer, pulsar, envelopes, patch bay, oscillators, LPGs, reverb
//
//   loop()  (whatever time is left)
//     MIDI in, web editor, panel memory, buttons (10 ms), patch bay (20 ms),
//     pots (5 ms), LEDs (10 ms)
//
// Files:
//   config.h           version and build options
//   hardware.h         pins, pot / button / patch bay / LED maps
//   pots.h             pot muxes and filtering
//   shift_registers.h  74HC165 chain
//   patch_bay.h        patch bay scanning (with the floating-input guard)
//   leds.h             AW9523 LED drivers
//   buttons.h          button scanning, act-on-release, MIDI shift
//   panel.h            what the buttons do, mode LEDs, panel memory
//   controls.h         what each pot does
//   midi_input.h       MIDI handlers;  midi_notes.h held notes;  midi_clock.h clock sync
//   sequencer.h        sequential voltage source
//   envelopes.h        envelope generator, pulsar, random voltage
//   oscillators.h      modulation and complex oscillators (oversampled, anti-aliased)
//   lpg_channel.h      panel LPG channel;  buchla_lpg.* the 292 circuit model
//   smoothing.h        audio-rate pot smoothing;  dsp_util.h maths helpers
//   synth.h            every module in one struct
//   audio.h            the audio callback
//   antialias.h        oversampling filters, band-limited waveforms, ADAA folder
//   cpu_meter.h        audio CPU load (printed with WEASEL_DEBUG 1)
//   WeaselEditor.h     USB link to the web pages (MIDI channel, presets, DFU)
// =============================================================================

// =============================================================================
// LICENSE & COPYRIGHT
//
// This project is released under the MIT License.
// Copyright (c) 2024 .axs instruments
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
// =============================================================================

#include "DaisyDuino.h"
#include <Wire.h>
#include <MIDI.h>

#include "config.h"         // first: sets WEASEL_FIRMWARE_VERSION for the editor
#include "WeaselEditor.h"   // web pages: MIDI channel + panel modes over USB serial

// The sketch's own code runs at -O2 whatever the IDE's Optimize setting
// (the editor above stays at the IDE setting, to save flash).
#pragma GCC optimize("O2")

#include "synth.h"
#include "controls.h"
#include "panel.h"
#include "midi_input.h"
#include "audio.h"
#include "cpu_meter.h"

// -- THE INSTRUMENT --
DaisyHardware hw;
MIDI_CREATE_INSTANCE(HardwareSerial, Serial1, MIDI);
namespace weasel
{
Synth synth;
Editor editor;  // github.com/joebmz98/Weasel-Flasher
}  // namespace weasel

using namespace weasel;

Buttons buttons;
PanelMemory panelMemory;
CpuMeter cpuMeter;

// Main-loop schedule
unsigned long lastButtonRead = 0;
unsigned long lastPatchScan = 0;
unsigned long lastPotRead = 0;
unsigned long lastLedUpdate = 0;
const unsigned long kPatchScanIntervalMs = 20;
const unsigned long kPotReadIntervalMs = 5;
const unsigned long kLedUpdateIntervalMs = 10;
const size_t kAudioBlockSize = 64;

// Called between slow panel I/O so MIDI bytes (clock above all) are read promptly
void serviceMidi() { drainMidi(); }

// Button hooks (buttons.h): the action, and the pitch-follow setting kept by the editor
void onButton(uint8_t bit) { buttonAction(synth, bit); }
uint8_t getPitchExclude() { return editor.pitchExclude(); }
void setPitchExclude(uint8_t mask) { editor.setPitchExclude(mask); }

// Web presets page hooks
void readPanelForEditor(uint8_t* out) { readPanelModes(synth, out); }
void writePanelFromEditor(const uint8_t* in) { writePanelModes(synth, in); }

void AudioCallback(float** in, float** out, size_t size)
{
  cpuMeter.blockStart();
  audioCallback(in, out, size);
  cpuMeter.blockEnd();
}

// One pot scan, then hand the new values to the audio callback
void scanPots()
{
  synth.pots.scan(serviceMidi);
  updateControls(synth);
  synth.smooth.publish(micros());
}

// =============================================================================
// SETUP
// =============================================================================
void setup()
{
#if WEASEL_DEBUG
  Serial.begin(115200);
#endif
  delay(100);

  // -- PANEL HARDWARE --
  synth.shiftRegisters.begin();
  synth.patchBay.begin();
  synth.leds.begin();
  analogReadResolution(16);  // 16-bit ADC
  synth.pots.begin();

  // -- DAISY SEED AT 48 kHz --
  hw = DAISY.init(DAISY_SEED, AUDIO_SR_48K);
  DAISY.SetAudioBlockSize(kAudioBlockSize);
  synth.initDsp(DAISY.get_samplerate());
  cpuMeter.init(synth.sampleRate, kAudioBlockSize);

  // -- MIDI --
  beginMidi();

  // -- WEB EDITOR: MIDI channel (kept in QSPI), panel modes, restart into DFU --
  editor.onBootloader = []() { hw.ResetToBootloader(); };
  editor.readPanel = readPanelForEditor;
  editor.writePanel = writePanelFromEditor;
  editor.begin();

  // -- INITIAL PANEL STATE (before audio starts) --
  synth.pots.scan(serviceMidi);
  updateControls(synth);
  synth.smooth.snap();  // start at the pot positions: nothing glides at power-up

  for (int i = 0; i < PatchBay::kDebounceReads + 2; i++)
  {
    synth.patchBay.scan(synth.shiftRegisters, serviceMidi);
  }
  buttons.onAction = onButton;
  buttons.getPitchExclude = getPitchExclude;
  buttons.setPitchExclude = setPitchExclude;
  scanButtons(synth, buttons);
  scanButtons(synth, buttons);
  buttons.armed = true;  // buttons held at boot do nothing, on press or on release

  // -- PANEL MEMORY: power up with the modes from last time --
  panelMemory.begin(synth, buttons.isDown(kPanelResetButton));
  updateLeds(synth, buttons, serviceMidi);

  // -- START AUDIO (last, so every DSP object is initialised first) --
  DAISY.begin(AudioCallback);
  configureMidiInterruptPriority();  // after DAISY.begin(), which sets the audio DMA priorities

#if WEASEL_DEBUG
  Serial.print("WEASEL firmware ");
  Serial.println(WEASEL_FIRMWARE_VERSION);
#endif
}

// =============================================================================
// LOOP
// =============================================================================
void loop()
{
  unsigned long now = millis();

  // MIDI
  drainMidi();
  synth.midiClock.checkActivity(millis());

  // WEB EDITOR: report MIDI state, answer requests from the USB port
  bool clockReceiving = synth.midiClock.receiving(millis());
  editor.live.noteHeld = synth.midi.noteHeld;
  editor.live.clockReceiving = clockReceiving;
  editor.live.clockSync = synth.midiClock.sync;
  editor.live.bpm = (clockReceiving && synth.midiClock.sync) ? synth.midiClock.tempo : 0.0f;
  // Sequencer clock for the monitor: the internal clock's actual step time, and the
  // division the clock pot selects in MIDI sync mode
  editor.live.stepPeriod10us = (uint32_t)((uint64_t)synth.seq.samplesPerTick * 100000ULL / (uint32_t)synth.sampleRate);
  editor.live.ticksPerStep = (uint8_t)synth.midiClock.division;
  editor.poll();

  // PANEL MEMORY: save the modes a few seconds after the last change
  panelMemory.poll(synth, now);

  // BUTTONS
  if (now - lastButtonRead >= Buttons::kReadIntervalMs)
  {
    scanButtons(synth, buttons);
    lastButtonRead = now;
  }

  // PATCH BAY
  if (now - lastPatchScan >= kPatchScanIntervalMs)
  {
    synth.patchBay.scan(synth.shiftRegisters, serviceMidi);
    lastPatchScan = now;
  }

  // POTS
  if (now - lastPotRead >= kPotReadIntervalMs)
  {
    scanPots();
    lastPotRead = now;
  }

  // LEDS
  if (now - lastLedUpdate >= kLedUpdateIntervalMs)
  {
    updateLeds(synth, buttons, serviceMidi);
    lastLedUpdate = now;
  }

#if WEASEL_DEBUG
  // AUDIO CPU LOAD
  static unsigned long lastCpuReport = 0;
  if (now - lastCpuReport >= 3000)
  {
    lastCpuReport = now;
    Serial.print("Audio CPU: ");
    Serial.print(cpuMeter.avg() * 100.0f, 1);
    Serial.print("% average, ");
    Serial.print(cpuMeter.peak() * 100.0f, 1);
    Serial.println("% peak");
    cpuMeter.resetPeak();
  }
#endif
}
