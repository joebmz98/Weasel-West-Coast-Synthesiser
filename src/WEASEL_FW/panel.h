// =============================================================================
// panel.h — what the buttons do, mode LEDs and panel memory (main loop)
//
// BUTTONS: buttons.h decides when a button acts; buttonAction() is what it does.
//
// PANEL MEMORY: the panel button modes (the same 14 the presets page handles)
// are saved to the Seed's QSPI flash and restored at power-up. Not saved: the
// patch bay, and the sequencer step switches (latching, so they always show their
// real position). Saving waits until the modes have been left alone for
// kSaveDelayMs, so a run of button presses is one write, and only writes if
// something actually changed. Hold PULSAR MANUAL at power-up to start from the
// defaults.
// =============================================================================
#pragma once
#include "Arduino.h"
#include "config.h"
#include "hardware.h"
#include "synth.h"
#include "buttons.h"
#include "WeaselEditor.h"

namespace weasel
{
extern Editor editor;

// -----------------------------------------------------------------------------
// WHAT EACH BUTTON DOES
// -----------------------------------------------------------------------------
WEASEL_NOINLINE inline void setClockSync(Synth& s, bool on)
{
  if (on && !s.midiClock.sync)
  {
    s.seq.sampleCounter = 0;
    s.midiClock.sync = true;
    s.midiClock.updateDivision(s.pots.norm(POT_CLOCK));
    s.midiClock.resync();
  }
  else
  {
    s.midiClock.sync = on;
  }
}

WEASEL_NOINLINE inline void buttonAction(Synth& s, uint8_t bit)
{
  switch (bit)
  {
    // --- SR1 ---
    case BTN_SEQ_TRIG_MODE:  // CLOCK -> PULSAR -> MIDI
      s.seq.triggerMode = static_cast<SeqTriggerMode>((s.seq.triggerMode + 1) % 3);
      break;
    case BTN_SEQ_STAGES:  // 5 -> 4 -> 3 steps
      s.seq.cycleStages();
      break;
    case BTN_ENV_TRIG_MODE:  // SEQ -> PULSAR -> MIDI
      s.env.cycleTrigger();
      break;

    // --- SR2 ---
    case BTN_ENV_MODE:  // TRANSIENT -> SUSTAIN -> OSCILLATE
      s.env.cycleMode();
      break;
    case BTN_PULSAR_MANUAL:
      s.pulsar.manualTrigger = true;  // fires on the next audio sample
      break;
    case BTN_PULSAR_MODE:  // SEQ -> MIDI -> OSC (free running)
      s.pulsar.cycleMode();
      break;
    case BTN_MODOSC_AM_FM:
      s.modOsc.amplitudeMod = !s.modOsc.amplitudeMod;
      break;
    case BTN_MODOSC_WAVE:  // SIN -> TRI -> SAW -> SQUARE
      s.modOsc.setWave((s.modOsc.waveIndex + 1) % 4);
      break;
    case BTN_COMPOSC_POLARITY:
      s.compOsc.inverted = !s.compOsc.inverted;
      break;
    case BTN_COMPOSC_WAVE:  // boots on SAW, then SAW -> SQUARE -> TRI (index 1 -> 2 -> 0)
      s.compOsc.setWave((s.compOsc.waveIndex + 1) % 3);
      break;
    case BTN_QUANTIZE_MIDI:
      s.midi.setFollow(!s.midi.follow);
      break;

    // --- SR3 ---
    case BTN_LPG1_MODE:  // VCA -> LP -> COMBI
      s.lpg1.cycleMode();
      break;
    case BTN_LPG2_MODE:
      s.lpg2.cycleMode();
      break;
    case BTN_RANDOM_MODE:  // SEQ -> PULSAR -> MIDI
      s.random.cycleMode();
      break;
    case BTN_CLOCK_SYNC:
      setClockSync(s, !s.midiClock.sync);
      break;

    default:
#if !SEQ_STEP_SWITCHES_LATCHING
      for (int i = 0; i < Sequencer::kSteps; i++)
      {
        if (bit == kSeqStepBits[i]) s.seq.stepEnabled[i] = !s.seq.stepEnabled[i];
      }
#endif
      break;
  }

#if WEASEL_DEBUG
  Serial.print("Button bit ");
  Serial.println(bit);
#endif
}

// -----------------------------------------------------------------------------
// BUTTON SCANNING (buttons.h decides when a button acts)
// -----------------------------------------------------------------------------
static_assert(Buttons::kPitchExcludeComplex == PITCH_EXCLUDE_COMPLEX, "pitch-exclude bits");
static_assert(Buttons::kPitchExcludeModulation == PITCH_EXCLUDE_MODULATION, "pitch-exclude bits");

inline void scanButtons(Synth& s, Buttons& buttons)
{
  buttons.update(s.shiftRegisters.read());
#if SEQ_STEP_SWITCHES_LATCHING
  // Level-based step enable
  for (int i = 0; i < Sequencer::kSteps; i++) s.seq.stepEnabled[i] = buttons.stepEnabled(i);
#endif
}

// -----------------------------------------------------------------------------
// MODE LEDS: built from the real synth state, so they always show the firmware state
// -----------------------------------------------------------------------------
WEASEL_NOINLINE inline void updateLeds(Synth& s, const Buttons& buttons, void (*idle)())
{
  Leds& l = s.leds;
  l.clear();

  l.set(leds::SEQ_TRIG, s.seq.triggerMode);
  l.set(leds::SEQ_STAGES, s.seq.maxSteps - 3);
  l.set(leds::ENV_TRIG, s.env.triggerMode);
  l.set(leds::ENV_MODE, s.env.mode);
  l.set(leds::PULSAR_MODE, s.pulsar.mode);
  if (s.pulsar.running) l.set(leds::PULSAR_ACT, 0);
  l.set(leds::MODOSC_AMFM, s.modOsc.amplitudeMod ? 1 : 0);
  l.set(leds::MODOSC_WAVE, s.modOsc.waveIndex);
  l.set(leds::COMP_POL, s.compOsc.inverted ? 1 : 0);
  l.set(leds::COMP_WAVE, s.compOsc.waveIndex);
  l.set(leds::QUANTIZE, s.midi.follow ? 1 : 0);
  l.set(leds::RANDOM_MODE, s.random.mode);
  l.set(leds::CLOCK_SYNC, s.midiClock.sync ? 1 : 0);
  l.set(leds::SEQ_STEP, s.seq.activeStep);

  // LPG modes. In the MIDI shift, a channel that follows MIDI pitch flashes all its
  // mode LEDs; one that ignores MIDI pitch shows its mode steady.
  bool flashOn = ((millis() / Buttons::kShiftFlashMs) & 1) == 0;
  bool flash1 = buttons.shiftActive && !(buttons.shiftPitchExclude & PITCH_EXCLUDE_COMPLEX);
  bool flash2 = buttons.shiftActive && !(buttons.shiftPitchExclude & PITCH_EXCLUDE_MODULATION);
  for (int i = 0; i < 3; i++)
  {
    if (flash1 ? flashOn : (i == (int)s.lpg1.mode)) l.set(leds::LPG1_MODE, i);
    if (flash2 ? flashOn : (i == (int)s.lpg2.mode)) l.set(leds::LPG2_MODE, i);
  }

  l.flush(idle);
}

// -----------------------------------------------------------------------------
// PANEL MODES for the web presets page and panel memory
// -----------------------------------------------------------------------------
// Byte order and meaning: PanelField in WeaselEditor.h (PANEL_FIELDS in the
// Weasel-Flasher's protocol.js). Every value is the position in the order the
// panel button steps through.

// Complex waveform: the panel steps SAW -> SQUARE -> TRI, the firmware stores TRI=0, SAW=1, SQUARE=2
inline uint8_t complexWaveToPanel(int index) { return (uint8_t)((index + 2) % 3); }
inline int complexWaveFromPanel(uint8_t value) { return (value + 1) % 3; }

WEASEL_NOINLINE inline void readPanelModes(const Synth& s, uint8_t* out)
{
  out[P_SEQ_TRIGGER] = (uint8_t)s.seq.triggerMode;
  out[P_SEQ_STAGES] = (uint8_t)(5 - s.seq.maxSteps);
  out[P_ENV_TRIGGER] = (uint8_t)s.env.triggerMode;
  out[P_ENV_MODE] = (uint8_t)s.env.mode;
  out[P_PULSAR_TRIGGER] = (uint8_t)s.pulsar.mode;
  out[P_RANDOM_TRIGGER] = (uint8_t)s.random.mode;
  out[P_MOD_WAVE] = (uint8_t)s.modOsc.waveIndex;
  out[P_MOD_TYPE] = s.modOsc.amplitudeMod ? 1 : 0;
  out[P_LPG2_MODE] = (uint8_t)s.lpg2.mode;
  out[P_COMPLEX_WAVE] = complexWaveToPanel(s.compOsc.waveIndex);
  out[P_POLARITY] = s.compOsc.inverted ? 1 : 0;
  out[P_LPG1_MODE] = (uint8_t)s.lpg1.mode;
  out[P_MIDI_FOLLOW] = s.midi.follow ? 1 : 0;
  out[P_CLOCK_MODE] = s.midiClock.sync ? 1 : 0;
}

// Same effect as pressing each button until its mode is reached. Values arrive clamped.
WEASEL_NOINLINE inline void writePanelModes(Synth& s, const uint8_t* in)
{
  s.seq.triggerMode = static_cast<SeqTriggerMode>(in[P_SEQ_TRIGGER]);
  s.seq.setStages(5 - in[P_SEQ_STAGES]);
  s.env.triggerMode = static_cast<EnvTriggerMode>(in[P_ENV_TRIGGER]);
  s.env.mode = static_cast<EnvelopeMode>(in[P_ENV_MODE]);
  s.pulsar.mode = static_cast<PulsarMode>(in[P_PULSAR_TRIGGER]);
  s.random.mode = static_cast<RandomMode>(in[P_RANDOM_TRIGGER]);

  if (in[P_MOD_WAVE] != s.modOsc.waveIndex) s.modOsc.setWave(in[P_MOD_WAVE]);
  s.modOsc.amplitudeMod = in[P_MOD_TYPE] != 0;
  s.lpg2.mode = static_cast<LpgMode>(in[P_LPG2_MODE]);

  int complexWave = complexWaveFromPanel(in[P_COMPLEX_WAVE]);
  if (complexWave != s.compOsc.waveIndex) s.compOsc.setWave(complexWave);
  s.compOsc.inverted = in[P_POLARITY] != 0;
  s.lpg1.mode = static_cast<LpgMode>(in[P_LPG1_MODE]);

  s.midi.setFollow(in[P_MIDI_FOLLOW] != 0);
  setClockSync(s, in[P_CLOCK_MODE] != 0);
}

class PanelMemory
{
 public:
  static const unsigned long kSaveDelayMs = 3000;
  static const unsigned long kCheckIntervalMs = 100;

  // setup(), with the buttons read: restore the saved modes (or forget them if
  // the reset button is held)
  WEASEL_NOINLINE void begin(Synth& s, bool resetHeld)
  {
    uint8_t saved[PANEL_COUNT];
    if (resetHeld)
    {
      editor.clearPanel();
#if WEASEL_DEBUG
      Serial.println("Panel memory cleared (button held at power-up) - starting from defaults");
#endif
    }
    else if (editor.loadPanel(saved))
    {
      writePanelModes(s, saved);
#if WEASEL_DEBUG
      Serial.println("Panel memory: restored the saved modes");
#endif
    }
    readPanelModes(s, lastSeen_);
  }

  // Main loop: save the modes a few seconds after the last change
  WEASEL_NOINLINE void poll(const Synth& s, unsigned long now)
  {
    if (now - lastCheck_ < kCheckIntervalMs) return;
    lastCheck_ = now;
    uint8_t panelNow[PANEL_COUNT];
    readPanelModes(s, panelNow);
    if (memcmp(panelNow, lastSeen_, sizeof(panelNow)) != 0)
    {
      memcpy(lastSeen_, panelNow, sizeof(panelNow));
      changedAt_ = now;
      savePending_ = true;
    }
    else if (savePending_ && now - changedAt_ >= kSaveDelayMs)
    {
      savePending_ = false;
      editor.storePanel(panelNow);  // erases/writes QSPI only if different from what's stored
#if WEASEL_DEBUG
      Serial.println("Panel memory: saved");
#endif
    }
  }

 private:
  uint8_t lastSeen_[PANEL_COUNT] = { 0 };
  unsigned long changedAt_ = 0;
  unsigned long lastCheck_ = 0;
  bool savePending_ = false;
};
}  // namespace weasel
