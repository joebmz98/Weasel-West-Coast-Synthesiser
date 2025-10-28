#include "DaisyDuino.h"
#include <MIDI.h>
//#include "buchla_lpg.h"

// MUX PINS - FIRST MUX
#define MUX1_S0 0
#define MUX1_S1 1
#define MUX1_S2 2
#define MUX1_S3 3
#define MUX1_SIG A0  // MUX signal pin

// MUX PINS - SECOND MUX
#define MUX2_S0 4
#define MUX2_S1 5
#define MUX2_S2 6
#define MUX2_S3 7
#define MUX2_SIG A1  // Second MUX signal pin

// NEW 4x7 BUTTON MATRIX PINS
#define MATRIX_COL0 19  // Column 0 (B0) // SEQUENCER CV OUT
#define MATRIX_COL1 20  // Column 1 (B1) // EG CV OUT
#define MATRIX_COL2 21  // Column 2 (B2) // PULSAR CV OUT
#define MATRIX_COL3 22  // Column 3 (B3) // RANDOM VOLTAGE CV OUT
#define MATRIX_ROW0 23  // Row 0 (A0) // PULSAR PERIOD MODULATION
#define MATRIX_ROW1 24  // Row 1 (A1) // MOD OSC FREQ
#define MATRIX_ROW2 25  // Row 2 (A2) // MOD OSC MOD AMOUNT
#define MATRIX_ROW3 26  // Row 3 (A3) // COMPLEX OSC PITCH
#define MATRIX_ROW4 27  // Row 4 (A4) // COMPLEX OSC WAVEFOLDER AMOUNT
#define MATRIX_ROW5 28  // Row 5 (A5) // LPG CH1 LEVEL MODULATION
#define MATRIX_ROW6 29  // Row 6 (A6) // LPG CH2 LEVEL MODULATION

// SECOND BUTTON MATRIX PINS
#define MATRIX2_COL0 12  // X0
#define MATRIX2_COL1 13  // X1
#define MATRIX2_COL2 17  // X2
#define MATRIX2_COL3 18  // X3
#define MATRIX2_ROW0 8   // Y0
#define MATRIX2_ROW1 9   // Y1
#define MATRIX2_ROW2 10  // Y2
#define MATRIX2_ROW3 11  // Y3
#define MATRIX2_ROW4 14  // Y4

// MOVED BUTTON PINS (keeping for compatibility, but functionality moved to second matrix)
#define SEQUENCER_TOGGLE_BUTTON 8   // SEQ TOGGLE (moved from 19) - NOW PART OF SECOND MATRIX
#define MODULATION_TOGGLE_BUTTON 9  // MODULATION TOGGLE (moved from 20) - NOW PART OF SECOND MATRIX
#define LPG_CH1_TOGGLE_BUTTON 10    // LPG CHANNEL 1 TOGGLE (moved from 21) - NOW PART OF SECOND MATRIX
#define LPG_CH2_TOGGLE_BUTTON 11    // LPG CHANNEL 2 TOGGLE (moved from 22) - NOW PART OF SECOND MATRIX

// MUX CHANNEL ASSIGNMENTS (FIRST MUX - updated assignments)
#define SEQ_STEP_1_CHANNEL 0          // C0 - Sequencer step 1
#define SEQ_STEP_2_CHANNEL 1          // C1 - Sequencer step 2
#define SEQ_STEP_3_CHANNEL 2          // C2 - Sequencer step 3
#define SEQ_STEP_4_CHANNEL 3          // C3 - Sequencer step 4
#define SEQ_STEP_5_CHANNEL 4          // C4 - Sequencer step 5
#define ASD_ATTACK_CHANNEL 5          // C5 - BUCHLA ATTACK // ATTACK
#define ASD_SUSTAIN_CHANNEL 6         // C6 - BUCHLA SUSTAIN // DECAY
#define ASD_DECAY_CHANNEL 7           // C7 - BUCHLA DECAY // RELEASE
#define PULSAR_ENV_DECAYCONTROL_CHANNEL 8  // C8 - Pulsar envelope decay control (new)
#define PULSAR_ENV_DECAY_CHANNEL 9    // C9 - Pulsar envelope decay time
#define MOD_OSC_PITCHCONTROL_CHANNEL 10    // C10 - Mod oscillator pitch control (new)
#define MOD_OSC_PITCH_CHANNEL 11      // C11 - Mod oscillator pitch
#define MOD_OSC_FINETUNE_CHANNEL 12   // C12 - Mod oscillator finetune (new)
#define MOD_AMOUNTCONTROL_CHANNEL 13  // C13 - Mod amount control (new)
#define MOD_AMOUNT_CHANNEL 14         // C14 - Mod amount
#define COMPLEX_OSC_PITCHCONTROL_CHANNEL 15 // C15 - Complex oscillator pitch control (new)

// MUX CHANNEL ASSIGNMENTS (SECOND MUX - updated assignments)
#define COMPLEX_OSC_PITCHCONTROL_CHANNEL 0  // C0 - Complex oscillator pitch control
#define COMPLEX_OSC_FINETUNE_CHANNEL 1      // C1 - Complex oscillator finetune (new)
#define COMPLEX_OSC_FOLDCONTROL_CHANNEL 2   // C2 - Complex oscillator fold control (new)
#define COMPLEX_OSC_FOLD_CHANNEL 3          // C3 - Complex oscillator fold amount
#define COMPLEX_OSC_TIMBRE_CHANNEL 4        // C4 - Complex oscillator timbre
#define LPG_CH1_LEVELCONTROL 5              // C5 - LPG channel 1 level control (new)
#define LPG_CH1_LEVEL 6                     // C6 - LPG channel 1 level (renamed from COMPLEX_OSC_LEVEL_CHANNEL)
#define LPG_CH2_LEVELCONTROL 7              // C7 - LPG channel 2 level control (new)
#define LPG_CH2_LEVEL 8                     // C8 - LPG channel 2 level (renamed from MOD_OSC_LEVEL_CHANNEL)
#define CLOCK_CHANNEL 9                     // C9 - Clock
#define REVERB_MIX 10                       // C10 - Reverb mix

// MIDI Settings
#define MIDI_RX_PIN 30  // USART1 Rx (Digital pin 30)

// LPG Mode Definitions
enum LPGMode {
  LPG_MODE_COMBI = 0,
  LPG_MODE_VCA = 1,
  LPG_MODE_LP = 2
};

// DEFINE DAISYSEED
DaisyHardware hw;

//INIT OSCILLATORS, FILTER
static Oscillator complexOsc;               // PRIMARY COMPLEX OSC
static Oscillator complexOscTri;            // SECONDARY COMPLEX OSC (triangle)
static Oscillator modOsc;                   // MODULATION OSC
static MoogLadder complexOsc_analogFilter;  // FILTER - HIGH CUT FOR "ANALOGUE" FEEL OF WAVEFORMS
static MoogLadder modOsc_analogFilter;      // FILTER FOR MOD OSCILLATOR
static Adsr pulsarEnv;                      // PULSAR ADSR ENVELOPE (replaces pulsarOsc)

// INIT LPG
static MoogLadder lpgChannel1_filter;  // FILTER FOR BUCHLA LPG CH1
static MoogLadder lpgChannel2_filter;  // FILTER FOR BUCHLA LPG CH2

// INIT REVERB
ReverbSc verb;

// OSCILLATOR PARAMETER VARIABLES
float complexOsc_basePitch;     // COMPLEX OSC BASE PITCH
float modOsc_pitch;             // MOD OSC BASE PITCH
float modOsc_modAmount;         // MOD AMOUNT AFFECTING COMPLEX OSC
float complexOsc_timbreAmount;  // COMPLEX OSC TIMBRE WAVEMORPHING AMOUNT (0.0 = sine, 1.0 = triangle)
float complexOsc_foldAmount;    // COMPLEX OSC WAVEFOLDING AMOUNT (0.0 = no fold, 1.0 = max fold)
float complexOsc_level;         // COMPLEX OSC LEVEL CONTROL (0.0 to 1.0)
float modOsc_level;             // MODULATION OSC LEVEL CONTROL (0.0 to 1.0)
float pulsarEnv_decayTime;      // PULSAR AD ENVELOPE DECAY TIME

// PULSAR ADSR ENVELOPE VARIABLES
bool pulsarEnv_gate = false;                // Whether the pulsar envelope gate is open
unsigned long lastPulsarEnvTime = 0;        // Last time the pulsar envelope was triggered
float pulsarEnv_attackTime = 0.02f;         // Fixed attack time of 0.02 seconds
float pulsarEnv_releaseTime = 0.02f;        // Fixed release time of 0.02 seconds
float pulsarEnv_sawtoothValue = 0.0f;       // Current sawtooth value for modulation
float pulsarEnv_baseDecayTime = 0.1f;       // Base decay time (from pot)
float pulsarEnv_modulatedDecayTime = 0.1f;  // Final decay time after modulation

// LP MODE CUTOFF CONTROL VARIABLES
float lpgCh1_baseCutoff = 0.5f;  // Base cutoff for Channel 1 in LP mode (0.0 to 1.0)
float lpgCh2_baseCutoff = 0.5f;  // Base cutoff for Channel 2 in LP mode (0.0 to 1.0)

// LPG BASE LEVEL CONTROLS (from MUX2 C6 and C8)
float lpgCh1Level = 1.0f;  // Base LPG level for channel 1 (complex osc)
float lpgCh2Level = 1.0f;  // Base LPG level for channel 2 (mod osc)

// WAVEFOLDER MODULATION
float wavefolderEnvModDepth = 0.0f;    // Depth of envelope modulation on wavefolder (0.0 to 1.0)
bool wavefolderEnvModEnabled = false;  // Whether envelope modulation of wavefolder is active

// SEQUENCER CV MODULATION
float seqCVWavefolderModDepth = 0.0f;          // Depth of sequencer CV modulation on wavefolder (0.0 to 1.0)
float seqCVModDepth_ch1 = 0.0f;                // Sequencer modulation depth for channel 1
float seqCVModDepth_ch2 = 0.0f;                // Sequencer modulation depth for channel 2
bool seqCVModOscPitchEnabled = false;          // Whether sequencer CV controls modOsc pitch
bool seqCVWavefolderModEnabled = false;        // Whether sequencer CV modulation of wavefolder is active
bool seqCVModAmountEnabled = false;            // Whether sequencer CV controls modOsc_modAmount
bool seqCVComplexOscPitchEnabled = false;      // Whether sequencer CV controls complexOsc pitch
bool seqCVLPGCh1LevelEnabled = false;          // Whether sequencer CV modulates LPG Channel 1 level (B0+A5)
bool seqCVLPGCh2LevelEnabled = false;          // Whether sequencer CV modulates LPG Channel 2 level (B0+A6)
bool seqCVPulsarEnvDecayEnabled = false;       // NEW: Whether sequencer CV modulates pulsar envelope decay time (B0+A0)
bool pulsarSelfModEnabled = false;             // Whether pulsar envelope modulates its own decay time (B2+A0)
bool pulsarModOscPitchEnabled = false;         // Whether pulsar envelope modulates modOsc pitch (B2+A1)
bool pulsarModAmountEnabled = false;           // Whether pulsar envelope modulates modOsc modAmount (B2+A2)
bool pulsarModComplexOscPitchEnabled = false;  // Whether pulsar envelope modulates complexOsc pitch (B2+A3)
bool pulsarModWavefolderEnabled = false;       // Pulsar envelope modulation of wavefolder amount (B2+A4)
bool pulsarModLPGCh1Enabled = false;           // Whether pulsar envelope modulates LPG Channel 1 (B2+A5)
bool pulsarModLPGCh2Enabled = false;           // Whether pulsar envelope modulates LPG Channel 2 (B2+A6)

// ENVELOPE MODULATION CONTROL
float envModDepth_ch1 = 1.0f;   // Envelope modulation depth for channel 1
float envModDepth_ch2 = 1.0f;   // Envelope modulation depth for channel 2
bool envModCh1Enabled = false;  // Whether envelope modulates LPG Channel 1 (B1+A5)
bool envModCh2Enabled = false;  // Whether envelope modulates LPG Channel 2 (B1+A6)

// REVERB CONTROL
float reverbMix = 0.0f;  // Reverb wet/dry mix (0.0 = dry, 1.0 = wet)

// ADSR ENVELOPE VARIABLES & OBJECT
Adsr env;               // ADSR envelope
float eg_attackTime;    // ATTACK time in seconds
float eg_decayTime;     // DECAY time in seconds (will control gate duration)
float eg_sustainLevel;  // SUSTAIN level (0.0 to 1.0)
float eg_releaseTime;   // RELEASE time in seconds

// ENVELOPE STATE VARIABLES
bool gateOpen = false;           // Whether the envelope gate is open
unsigned long lastGateTime = 0;  // Last time the gate was triggered

// SEQUENCER VARIABLES
const int SEQ_STEPS = 5;           // NUM OF STEPS
float sequencerValues[SEQ_STEPS];  // STORE POT VALUES
int currentStep = 0;               // CURRENT STEP
unsigned long stepStartTime = 0;   // TIME WHEN CURRENT STEP STARTED
float BPM = 100.0f;                // CLOCK SPEED (now variable)
float STEP_DURATION_MS;            // STEP DURATION (will be calculated based on BPM)

// MIDI SEQUENCER CONTROL
bool useMidiClock = false;  // false = internal clock, true = MIDI note triggers

// MODULATION TYPE CONTROL
bool useAmplitudeModulation = false;  // false = FM, true = AM

// LPG MODE CONTROL
LPGMode lpgChannel1_mode = LPG_MODE_COMBI;  // Default to COMBI mode
LPGMode lpgChannel2_mode = LPG_MODE_COMBI;  // Default to COMBI mode

// MIDI PITCH CONTROL
bool midiPitchEnabled = true;  // Whether oscillators' pitches are impacted by MIDI input

// BUTTONS - REMOVED since we're using matrix now
// Switch sequencerToggle;     // SEQUENCER CLOCK TOGGLE
// Switch modulationToggle;    // MODULATION TYPE TOGGLE
// Switch lpgToggle_channel1;  // LPG CH1 MODE TOGGLE
// Switch lpgToggle_channel2;  // LPG CH2 MODE TOGGLE

// NEW 4x7 BUTTON MATRIX VARIABLES
bool matrixStates[4][7] = { { false } };  // Store states for [col][row] - 4 columns x 7 rows
bool lastMatrixStates[4][7] = { { false } };
unsigned long lastMatrixRead = 0;
const unsigned long MATRIX_READ_INTERVAL = 50;  // Read matrix every 50ms

// SECOND BUTTON MATRIX VARIABLES
bool matrix2States[4][5] = { { false } };  // Store states for [col][row] - 4 columns x 5 rows
bool lastMatrix2States[4][5] = { { false } };
unsigned long lastMatrix2Read = 0;

// Button debouncing variables for second matrix
unsigned long lastMatrix2DebounceTime[4][5] = { { 0 } };
const unsigned long MATRIX2_DEBOUNCE_DELAY = 50;  // 50ms debounce delay

// MIDI Object
MIDI_CREATE_INSTANCE(HardwareSerial, Serial1, MIDI);

// MIDI Debug variables
unsigned long lastMidiDebugTime = 0;
bool midiNoteReceived = false;
byte lastMidiNote = 0;
byte lastMidiVelocity = 0;
byte lastMidiChannel = 0;
unsigned long midiNoteCount = 0;
unsigned long midiErrorCount = 0;

// MIDI to CV variables
float midiPitchCV = 0.0f;                      // MIDI pitch control voltage (in semitones) - PERSISTS after note off
bool midiNoteActive = false;                   // Whether a MIDI note is currently active (for triggering)
unsigned long lastMidiNoteTime = 0;            // Time when last MIDI note was received
const unsigned long MIDI_NOTE_TIMEOUT = 1000;  // 1 second timeout for MIDI notes

// Function to convert linear values to logarithmic scale for pitch
float linearToLog(float value, float minVal, float maxVal) {
  return minVal * pow(maxVal / minVal, value);
}

// Function to convert semitones to frequency ratio
float semitonesToRatio(float semitones) {
  return pow(2.0f, semitones / 12.0f);
}

// Function to convert MIDI note to frequency (1V/octave standard)
float midiNoteToFrequency(byte midiNote, float baseFreq = 261.63f) {  // C4 = 261.63Hz
  // Convert MIDI note to frequency (MIDI note 69 = A4 = 440Hz)
  return 440.0f * pow(2.0f, (midiNote - 69) / 12.0f);
}

// Function to convert MIDI note to CV (1V/octave, where 1V = 12 semitones)
float midiNoteToCV(byte midiNote, float baseNote = 60.0f) {  // C4 = MIDI note 60
  // Convert MIDI note to CV voltage (1V/octave standard)
  // Returns semitones offset from base note
  return (midiNote - baseNote);
}

// WAVEFOLDER FNC.
float wavefolder(float input, float amount) {
  if (amount < 0.001f) return input;

  float scaledInput = input * (1.0f + amount * 80.0f);

  for (int fold = 0; fold < 3; fold++) {
    while (fabs(scaledInput) > 1.0f) {
      if (scaledInput > 1.0f) {
        scaledInput = 2.0f - scaledInput;
      } else if (scaledInput < -1.0f) {
        scaledInput = -2.0f - scaledInput;
      }
    }

    if (amount > 0.5f) {
      scaledInput *= (0.95f + 0.1f * amount);
    }
  }

  float wetDryMix;
  if (amount < 0.3f) {
    wetDryMix = amount / 0.3f;
  } else {
    wetDryMix = 1.0f;
  }

  return (input * (1.0f - wetDryMix)) + ((scaledInput * wetDryMix) * 0.25);
}

void processLPG(MoogLadder& filter, LPGMode mode, float& signal, float channelLevel, float baseCutoffControl, float envValue, float baseLevel, bool envModEnabled, float envModDepth = 1.0f, float seqCVValue = 0.0f, float seqCVModDepth = 0.0f, float pulsarEnvValue = 0.0f, bool pulsarModEnabled = false) {
  float outputGain = 1.0f;

  // PULSER MODULATION DEPTH CONTROLS
  float pulsarLevelModDepth = 0.8f;        // Level modulation depth (0.0-1.0)
  float pulsarCutoffModDepthCombi = 0.8f;  // Combi mode cutoff depth as percentage
  float pulsarCutoffModDepthVCA = 0.5f;    // VCA mode cutoff depth as percentage  
  float pulsarCutoffModDepthLP = 0.9f;     // LP mode cutoff depth as percentage

  // Use baseLevel (from MUX2 C6/C8) as the starting point
  float finalLevel = baseLevel;

  // Apply sequencer CV modulation to level if enabled
  if (seqCVModDepth > 0.0f) {
    finalLevel += (seqCVValue * seqCVModDepth);
    finalLevel = fminf(fmaxf(finalLevel, 0.0f), 1.0f);
  }

  // Apply pulser modulation to level if enabled (affects base level for all modes)
  if (pulsarModEnabled) {
    // Pulsar envelope modulates FROM baseLevel UPWARD
    // When baseLevel=0, pulser sweeps from 0 to pulsarLevelModDepth
    // When baseLevel=0.5, pulser sweeps from 0.5 to 0.5 + (pulsarLevelModDepth * 0.5)
    finalLevel = baseLevel + (pulsarEnvValue * pulsarLevelModDepth * (1.0f - baseLevel));
    finalLevel = fminf(fmaxf(finalLevel, 0.0f), 1.0f);
  }

  if (envModEnabled) {
    float modulatedEnv = envValue * envModDepth;
    finalLevel = baseLevel + ((1.0f - baseLevel) * modulatedEnv);
    finalLevel = fminf(fmaxf(finalLevel, 0.0f), 1.0f);
  }

  switch (mode) {
    case LPG_MODE_COMBI:
      {
        // COMBI mode: MUX2 C6/C8 control base level AND base cutoff
        
        // Apply the final level to the signal
        signal *= finalLevel;

        // Base cutoff determined by baseLevel (from MUX2 C6/C8)
        float baseCutoff = 20.0f + (baseLevel * 17980.0f);
        float maxCutoff = 18000.0f;

        // Apply sequencer CV modulation to cutoff if enabled
        float combiCutoff = baseCutoff;
        if (seqCVModDepth > 0.0f) {
          combiCutoff = baseCutoff + (seqCVValue * seqCVModDepth * 5000.0f);
          combiCutoff = fminf(fmaxf(combiCutoff, 20.0f), maxCutoff);
        }

        // Apply pulser modulation to cutoff if enabled - MODULATES FROM BASE CUTOFF
        if (pulsarModEnabled) {
          // Pulsar envelope sweeps FROM baseCutoff UPWARD toward maxCutoff
          float cutoffRange = maxCutoff - baseCutoff;
          combiCutoff = baseCutoff + (pulsarEnvValue * pulsarCutoffModDepthCombi * cutoffRange);
          combiCutoff = fminf(fmaxf(combiCutoff, 20.0f), maxCutoff);
        }

        // Envelope opens the filter from base cutoff up to 20kHz (only if modulation enabled)
        if (envModEnabled) {
          float modulatedEnv = envValue * envModDepth;
          combiCutoff = baseCutoff + ((maxCutoff - baseCutoff) * modulatedEnv);
        }
        
        filter.SetFreq(fminf(combiCutoff, maxCutoff));
        filter.SetRes(0.0f);

        outputGain = 1.0f;
        signal = filter.Process(signal) * outputGain;
        break;
      }

    case LPG_MODE_VCA:
      {
        // VCA mode: MUX2 C6/C8 control base level
        
        // Apply the final level to the signal (already includes pulser modulation)
        signal *= finalLevel;

        // Filter cutoff increases with oscillator level
        float baseCutoff = 1200.0f + (channelLevel * 17800.0f);
        float maxCutoff = 19000.0f;

        // Apply sequencer CV modulation to cutoff if enabled
        float vcaCutoff = baseCutoff;
        if (seqCVModDepth > 0.0f) {
          vcaCutoff += (seqCVValue * seqCVModDepth * 2000.0f);
          vcaCutoff = fminf(fmaxf(vcaCutoff, 1200.0f), maxCutoff);
        }

        // Apply pulser modulation to cutoff if enabled - SUBTLE EFFECT IN VCA MODE
        if (pulsarModEnabled) {
          // In VCA mode, pulser mainly affects level, but can subtly affect cutoff too
          float cutoffRange = maxCutoff - baseCutoff;
          vcaCutoff = baseCutoff + (pulsarEnvValue * pulsarCutoffModDepthVCA * cutoffRange * 0.3f);
          vcaCutoff = fminf(fmaxf(vcaCutoff, 1200.0f), maxCutoff);
        }

        // Envelope also opens the filter slightly for timbral variation
        if (envModEnabled) {
          float modulatedEnv = envValue * envModDepth;
          vcaCutoff = baseCutoff + ((maxCutoff - baseCutoff) * modulatedEnv * 0.5f);
        }
        
        filter.SetFreq(fminf(vcaCutoff, maxCutoff));
        filter.SetRes(0.0f);

        outputGain = 1.0f;
        signal = filter.Process(signal) * outputGain;
        break;
      }

    case LPG_MODE_LP:
      {
        // LP mode: MUX2 C6/C8 control base cutoff frequency
        
        // Base cutoff determined by baseLevel parameter (from MUX2 C6/C8 pots)
        float baseCutoff = 20.0f + (baseLevel * 17980.0f);
        float maxCutoff = 18000.0f;

        // Apply sequencer CV modulation to cutoff if enabled
        float lpCutoff = baseCutoff;
        if (seqCVModDepth > 0.0f) {
          lpCutoff = baseCutoff + (seqCVValue * seqCVModDepth * 10000.0f);
          lpCutoff = fminf(fmaxf(lpCutoff, 20.0f), maxCutoff);
        }

        // Apply pulser modulation to cutoff if enabled - STRONG EFFECT IN LP MODE
        if (pulsarModEnabled) {
          // Pulsar envelope sweeps FROM baseCutoff UPWARD toward maxCutoff
          float cutoffRange = maxCutoff - baseCutoff;
          lpCutoff = baseCutoff + (pulsarEnvValue * pulsarCutoffModDepthLP * cutoffRange);
          lpCutoff = fminf(fmaxf(lpCutoff, 20.0f), maxCutoff);
        }

        // Envelope opens the filter from base cutoff up to 18kHz
        if (envModEnabled) {
          float modulatedEnv = envValue * envModDepth;
          lpCutoff = baseCutoff + ((maxCutoff - baseCutoff) * modulatedEnv);
        }
        
        filter.SetFreq(fminf(lpCutoff, maxCutoff));

        // Safe resonance curve based on baseLevel only
        float resonance = pow(baseLevel, 1.8f) * 0.9f;
        resonance = fminf(resonance, 0.92f);

        if (resonance > 0.85f) {
          resonance = 0.85f + (resonance - 0.85f) * 0.3f;
        }

        filter.SetRes(resonance);

        // Input gain compensation
        float safeGain = 1.0f / (1.0f + resonance * 0.5f);
        signal *= safeGain;

        // Boost for LP mode
        outputGain = 2.5f + (resonance * 3.0f);

        signal = filter.Process(signal) * outputGain;
        break;
      }
  }
}

// MUX FUNCTIONS
void setMux1Channel(int channel) {
  digitalWrite(MUX1_S0, channel & 0x01);
  digitalWrite(MUX1_S1, channel & 0x02);
  digitalWrite(MUX1_S2, channel & 0x04);
  digitalWrite(MUX1_S3, channel & 0x08);
}

void setMux2Channel(int channel) {
  digitalWrite(MUX2_S0, channel & 0x01);
  digitalWrite(MUX2_S1, channel & 0x02);
  digitalWrite(MUX2_S2, channel & 0x04);
  digitalWrite(MUX2_S3, channel & 0x08);
}

float readMux1Channel(int channel, float minVal, float maxVal, bool logarithmic = false) {
  setMux1Channel(channel);
  delayMicroseconds(10);

  int rawValue = analogRead(MUX1_SIG);
  float normalizedValue = rawValue / 65535.0f;

  if (logarithmic) {
    return linearToLog(normalizedValue, minVal, maxVal);
  } else {
    return minVal + (maxVal - minVal) * normalizedValue;
  }
}

float readMux2Channel(int channel, float minVal, float maxVal, bool logarithmic = false) {
  setMux2Channel(channel);
  delayMicroseconds(10);

  int rawValue = analogRead(MUX2_SIG);
  float normalizedValue = rawValue / 65535.0f;

  if (logarithmic) {
    return linearToLog(normalizedValue, minVal, maxVal);
  } else {
    return minVal + (maxVal - minVal) * normalizedValue;
  }
}

// NEW 4x7 BUTTON MATRIX FUNCTIONS
void initButtonMatrix() {
  // Initialize column pins as outputs
  pinMode(MATRIX_COL0, OUTPUT);
  pinMode(MATRIX_COL1, OUTPUT);
  pinMode(MATRIX_COL2, OUTPUT);
  pinMode(MATRIX_COL3, OUTPUT);

  // Initialize row pins as inputs with pulldown
  pinMode(MATRIX_ROW0, INPUT_PULLDOWN);
  pinMode(MATRIX_ROW1, INPUT_PULLDOWN);
  pinMode(MATRIX_ROW2, INPUT_PULLDOWN);
  pinMode(MATRIX_ROW3, INPUT_PULLDOWN);
  pinMode(MATRIX_ROW4, INPUT_PULLDOWN);
  pinMode(MATRIX_ROW5, INPUT_PULLDOWN);
  pinMode(MATRIX_ROW6, INPUT_PULLDOWN);

  // Start with all columns LOW
  digitalWrite(MATRIX_COL0, LOW);
  digitalWrite(MATRIX_COL1, LOW);
  digitalWrite(MATRIX_COL2, LOW);
  digitalWrite(MATRIX_COL3, LOW);
}

// SECOND BUTTON MATRIX FUNCTIONS
void initButtonMatrix2() {
  // Initialize column pins as outputs
  pinMode(MATRIX2_COL0, OUTPUT);
  pinMode(MATRIX2_COL1, OUTPUT);
  pinMode(MATRIX2_COL2, OUTPUT);
  pinMode(MATRIX2_COL3, OUTPUT);

  // Initialize row pins as inputs with pulldown
  pinMode(MATRIX2_ROW0, INPUT_PULLDOWN);
  pinMode(MATRIX2_ROW1, INPUT_PULLDOWN);
  pinMode(MATRIX2_ROW2, INPUT_PULLDOWN);
  pinMode(MATRIX2_ROW3, INPUT_PULLDOWN);
  pinMode(MATRIX2_ROW4, INPUT_PULLDOWN);

  // Start with all columns LOW
  digitalWrite(MATRIX2_COL0, LOW);
  digitalWrite(MATRIX2_COL1, LOW);
  digitalWrite(MATRIX2_COL2, LOW);
  digitalWrite(MATRIX2_COL3, LOW);
}

void readButtonMatrix() {
  // Scan each column one at a time
  for (int col = 0; col < 4; col++) {
    // Activate current column
    switch (col) {
      case 0:
        digitalWrite(MATRIX_COL0, HIGH);
        digitalWrite(MATRIX_COL1, LOW);
        digitalWrite(MATRIX_COL2, LOW);
        digitalWrite(MATRIX_COL3, LOW);
        break;
      case 1:
        digitalWrite(MATRIX_COL0, LOW);
        digitalWrite(MATRIX_COL1, HIGH);
        digitalWrite(MATRIX_COL2, LOW);
        digitalWrite(MATRIX_COL3, LOW);
        break;
      case 2:
        digitalWrite(MATRIX_COL0, LOW);
        digitalWrite(MATRIX_COL1, LOW);
        digitalWrite(MATRIX_COL2, HIGH);
        digitalWrite(MATRIX_COL3, LOW);
        break;
      case 3:
        digitalWrite(MATRIX_COL0, LOW);
        digitalWrite(MATRIX_COL1, LOW);
        digitalWrite(MATRIX_COL2, LOW);
        digitalWrite(MATRIX_COL3, HIGH);
        break;
    }

    delayMicroseconds(10);  // Small delay for stabilization

    // Read all 7 rows for this column
    for (int row = 0; row < 7; row++) {
      lastMatrixStates[col][row] = matrixStates[col][row];

      // Read the appropriate row pin
      switch (row) {
        case 0: matrixStates[col][row] = (digitalRead(MATRIX_ROW0) == HIGH); break;
        case 1: matrixStates[col][row] = (digitalRead(MATRIX_ROW1) == HIGH); break;
        case 2: matrixStates[col][row] = (digitalRead(MATRIX_ROW2) == HIGH); break;
        case 3: matrixStates[col][row] = (digitalRead(MATRIX_ROW3) == HIGH); break;
        case 4: matrixStates[col][row] = (digitalRead(MATRIX_ROW4) == HIGH); break;
        case 5: matrixStates[col][row] = (digitalRead(MATRIX_ROW5) == HIGH); break;
        case 6: matrixStates[col][row] = (digitalRead(MATRIX_ROW6) == HIGH); break;
      }
    }
  }

  // Deactivate all columns
  digitalWrite(MATRIX_COL0, LOW);
  digitalWrite(MATRIX_COL1, LOW);
  digitalWrite(MATRIX_COL2, LOW);
  digitalWrite(MATRIX_COL3, LOW);

  // Update modulation enable flags based on new matrix positions

  // NEW: B0 + A0 enables sequencer CV modulation of pulsar envelope decay time
  seqCVPulsarEnvDecayEnabled = matrixStates[0][0];

  // B0 + A1 enables sequencer CV control of modOsc pitch
  seqCVModOscPitchEnabled = matrixStates[0][1];

  // B2 + A1 enables pulsar envelope modulation of modOsc pitch
  pulsarModOscPitchEnabled = matrixStates[2][1];

  // B0 + A2 enables sequencer CV control of modOsc_modAmount
  seqCVModAmountEnabled = matrixStates[0][2];

  // B2 + A2 enables pulsar envelope modulation of modOsc_modAmount
  pulsarModAmountEnabled = matrixStates[2][2];

  // B0 + A3 enables sequencer CV control of complexOsc pitch
  seqCVComplexOscPitchEnabled = matrixStates[0][3];

  // B0 + A4 enables sequencer CV modulation of wavefolder
  seqCVWavefolderModEnabled = matrixStates[0][4];

  // B1 + A4 enables envelope modulation of wavefolder
  wavefolderEnvModEnabled = matrixStates[1][4];

  // B2 + A0 enables pulsar envelope self-modulation (modulates its own decay time)
  pulsarSelfModEnabled = matrixStates[2][0];

  // B2 + A3 enables pulsar envelope modulation of complexOsc pitch
  pulsarModComplexOscPitchEnabled = matrixStates[2][3];

  // B2 + A4 enables pulsar envelope modulation of wavefolder amount
  pulsarModWavefolderEnabled = matrixStates[2][4];

  // B0 + A5 enables sequencer CV modulation of LPG Channel 1 level
  seqCVLPGCh1LevelEnabled = matrixStates[0][5];

  // B0 + A6 enables sequencer CV modulation of LPG Channel 2 level
  seqCVLPGCh2LevelEnabled = matrixStates[0][6];

  // NEW: B1 + A5 enables envelope modulation of LPG Channel 1
  envModCh1Enabled = matrixStates[1][5];

  // NEW: B1 + A6 enables envelope modulation of LPG Channel 2
  envModCh2Enabled = matrixStates[1][6];

  // NEW: B2 + A5 enables pulsar envelope modulation of LPG Channel 1
  pulsarModLPGCh1Enabled = matrixStates[2][5];

  // NEW: B2 + A6 enables pulsar envelope modulation of LPG Channel 2
  pulsarModLPGCh2Enabled = matrixStates[2][6];

  // Set envelope modulation depth based on B0+A5/6 combinations
  if (matrixStates[0][5]) {
    envModDepth_ch1 = 2.0f;  // Double the envelope modulation depth for channel 1
  } else {
    envModDepth_ch1 = 1.0f;  // Normal envelope modulation depth for channel 1
  }

  if (matrixStates[0][6]) {
    envModDepth_ch2 = 2.0f;  // Double the envelope modulation depth for channel 2
  } else {
    envModDepth_ch2 = 1.0f;  // Normal envelope modulation depth for channel 2
  }

  // FIX: Control sequencer CV modulation depth based on B0+A5/6 buttons
  // Sequencer CV should ONLY modulate LPG when B0+A5 or B0+A6 are pressed
  if (matrixStates[0][5]) {
    seqCVModDepth_ch1 = 1.0f;  // Enable sequencer CV modulation for channel 1
  } else {
    seqCVModDepth_ch1 = 0.0f;  // Disable sequencer CV modulation for channel 1
  }

  if (matrixStates[0][6]) {
    seqCVModDepth_ch2 = 1.0f;  // Enable sequencer CV modulation for channel 2
  } else {
    seqCVModDepth_ch2 = 0.0f;  // Disable sequencer CV modulation for channel 2
  }
}

void readButtonMatrix2() {
  unsigned long currentTime = millis();
  
  // Scan each column one at a time
  for (int col = 0; col < 4; col++) {
    // Activate current column
    switch (col) {
      case 0:
        digitalWrite(MATRIX2_COL0, HIGH);
        digitalWrite(MATRIX2_COL1, LOW);
        digitalWrite(MATRIX2_COL2, LOW);
        digitalWrite(MATRIX2_COL3, LOW);
        break;
      case 1:
        digitalWrite(MATRIX2_COL0, LOW);
        digitalWrite(MATRIX2_COL1, HIGH);
        digitalWrite(MATRIX2_COL2, LOW);
        digitalWrite(MATRIX2_COL3, LOW);
        break;
      case 2:
        digitalWrite(MATRIX2_COL0, LOW);
        digitalWrite(MATRIX2_COL1, LOW);
        digitalWrite(MATRIX2_COL2, HIGH);
        digitalWrite(MATRIX2_COL3, LOW);
        break;
      case 3:
        digitalWrite(MATRIX2_COL0, LOW);
        digitalWrite(MATRIX2_COL1, LOW);
        digitalWrite(MATRIX2_COL2, LOW);
        digitalWrite(MATRIX2_COL3, HIGH);
        break;
    }

    delayMicroseconds(50);  // Increased delay for better stabilization

    // Read all 5 rows for this column
    for (int row = 0; row < 5; row++) {
      bool currentState = false;
      
      // Read the appropriate row pin
      switch (row) {
        case 0: currentState = (digitalRead(MATRIX2_ROW0) == HIGH); break;
        case 1: currentState = (digitalRead(MATRIX2_ROW1) == HIGH); break;
        case 2: currentState = (digitalRead(MATRIX2_ROW2) == HIGH); break;
        case 3: currentState = (digitalRead(MATRIX2_ROW3) == HIGH); break;
        case 4: currentState = (digitalRead(MATRIX2_ROW4) == HIGH); break;
      }
      
      // Check for state change
      if (currentState != lastMatrix2States[col][row]) {
        // Update debounce timer
        lastMatrix2DebounceTime[col][row] = currentTime;
      }
      
      // Check if debounce time has passed
      if ((currentTime - lastMatrix2DebounceTime[col][row]) > MATRIX2_DEBOUNCE_DELAY) {
        // State is stable, check for rising edge (button press)
        if (currentState && !matrix2States[col][row]) {
          // Button pressed - handle the function
          handleMatrix2ButtonPress(col, row);
        }
        
        // Update the stable state
        matrix2States[col][row] = currentState;
      }
      
      // Update last state for edge detection
      lastMatrix2States[col][row] = currentState;
    }
  }

  // Deactivate all columns
  digitalWrite(MATRIX2_COL0, LOW);
  digitalWrite(MATRIX2_COL1, LOW);
  digitalWrite(MATRIX2_COL2, LOW);
  digitalWrite(MATRIX2_COL3, LOW);
}

void handleMatrix2ButtonPress(int col, int row) {
  Serial.print("Matrix2 Button Press: X");
  Serial.print(col);
  Serial.print(" Y");
  Serial.println(row);

  // X3 Y1: Toggle modulation type (FM/AM)
  if (col == 3 && row == 1) {
    useAmplitudeModulation = !useAmplitudeModulation;
    Serial.print("Modulation type: ");
    Serial.println(useAmplitudeModulation ? "AM (Amplitude Modulation)" : "FM (Frequency Modulation)");
  }

  // X3 Y2: Toggle LPG mode on channel 1
  else if (col == 3 && row == 2) {
    lpgChannel1_mode = static_cast<LPGMode>((lpgChannel1_mode + 1) % 3);
    Serial.print("LPG Channel 1 mode: ");
    switch (lpgChannel1_mode) {
      case LPG_MODE_COMBI: Serial.println("COMBI (env controls cutoff+amp)"); break;
      case LPG_MODE_VCA: Serial.println("VCA (env controls amplitude)"); break;
      case LPG_MODE_LP: Serial.println("LP (env controls cutoff)"); break;
    }
  }

  // X0 Y3: Toggle LPG mode on channel 2
  else if (col == 0 && row == 3) {
    lpgChannel2_mode = static_cast<LPGMode>((lpgChannel2_mode + 1) % 3);
    Serial.print("LPG Channel 2 mode: ");
    switch (lpgChannel2_mode) {
      case LPG_MODE_COMBI: Serial.println("COMBI (env controls cutoff+amp)"); break;
      case LPG_MODE_VCA: Serial.println("VCA (env controls amplitude)"); break;
      case LPG_MODE_LP: Serial.println("LP (env controls cutoff)"); break;
    }
  }

  // X2 Y1: Toggle whether oscillators' pitches are impacted by MIDI input
  else if (col == 2 && row == 1) {
    midiPitchEnabled = !midiPitchEnabled;
    Serial.print("MIDI pitch control: ");
    Serial.println(midiPitchEnabled ? "ENABLED" : "DISABLED");
  }
}

void printButtonStates() {
  // Print button states only when they change for the specific buttons we care about
  bool anyChange = false;

  // Check only the buttons we're using for modulation
  if (matrixStates[0][0] != lastMatrixStates[0][0] || matrixStates[0][4] != lastMatrixStates[0][4] || matrixStates[1][4] != lastMatrixStates[1][4] || matrixStates[0][1] != lastMatrixStates[0][1] || matrixStates[0][2] != lastMatrixStates[0][2] || matrixStates[0][3] != lastMatrixStates[0][3] || matrixStates[2][1] != lastMatrixStates[2][1] || matrixStates[2][2] != lastMatrixStates[2][2] || matrixStates[2][0] != lastMatrixStates[2][0] || matrixStates[2][3] != lastMatrixStates[2][3] || matrixStates[2][4] != lastMatrixStates[2][4] || matrixStates[0][5] != lastMatrixStates[0][5] || matrixStates[0][6] != lastMatrixStates[0][6] || matrixStates[1][5] != lastMatrixStates[1][5] || matrixStates[1][6] != lastMatrixStates[1][6] || matrixStates[2][5] != lastMatrixStates[2][5] || matrixStates[2][6] != lastMatrixStates[2][6]) {
    anyChange = true;
  }

  if (anyChange) {
    Serial.print("Button Matrix - ");
    Serial.print("B0+A0: ");
    Serial.print(matrixStates[0][0] ? "HIGH" : "LOW");
    Serial.print(" | B0+A4: ");
    Serial.print(matrixStates[0][4] ? "HIGH" : "LOW");
    Serial.print(" | B1+A4: ");
    Serial.print(matrixStates[1][4] ? "HIGH" : "LOW");
    Serial.print(" | B0+A1: ");
    Serial.print(matrixStates[0][1] ? "HIGH" : "LOW");
    Serial.print(" | B2+A1: ");
    Serial.print(matrixStates[2][1] ? "HIGH" : "LOW");
    Serial.print(" | B0+A2: ");
    Serial.print(matrixStates[0][2] ? "HIGH" : "LOW");
    Serial.print(" | B2+A2: ");
    Serial.print(matrixStates[2][2] ? "HIGH" : "LOW");
    Serial.print(" | B0+A3: ");
    Serial.print(matrixStates[0][3] ? "HIGH" : "LOW");
    Serial.print(" | B2+A0: ");
    Serial.print(matrixStates[2][0] ? "HIGH" : "LOW");
    Serial.print(" | B2+A3: ");
    Serial.print(matrixStates[2][3] ? "HIGH" : "LOW");
    Serial.print(" | B2+A4: ");
    Serial.print(matrixStates[2][4] ? "HIGH" : "LOW");
    Serial.print(" | B0+A5: ");
    Serial.print(matrixStates[0][5] ? "HIGH" : "LOW");
    Serial.print(" | B0+A6: ");
    Serial.print(matrixStates[0][6] ? "HIGH" : "LOW");
    Serial.print(" | B1+A5: ");
    Serial.print(matrixStates[1][5] ? "HIGH" : "LOW");
    Serial.print(" | B1+A6: ");
    Serial.print(matrixStates[1][6] ? "HIGH" : "LOW");
    Serial.print(" | B2+A5: ");
    Serial.print(matrixStates[2][5] ? "HIGH" : "LOW");
    Serial.print(" | B2+A6: ");
    Serial.print(matrixStates[2][6] ? "HIGH" : "LOW");

    // Show modulation status changes
    if (matrixStates[0][0] != lastMatrixStates[0][0]) {
      Serial.print(" | Seq CV Pulsar Env Decay: ");
      Serial.print(seqCVPulsarEnvDecayEnabled ? "ENABLED" : "DISABLED");
    }
    if (matrixStates[0][4] != lastMatrixStates[0][4]) {
      Serial.print(" | Seq CV Wavefolder Mod: ");
      Serial.print(seqCVWavefolderModEnabled ? "ENABLED" : "DISABLED");
    }
    if (matrixStates[1][4] != lastMatrixStates[1][4]) {
      Serial.print(" | Wavefolder Env Mod: ");
      Serial.print(wavefolderEnvModEnabled ? "ENABLED" : "DISABLED");
    }
    if (matrixStates[0][1] != lastMatrixStates[0][1]) {
      Serial.print(" | Seq CV ModOsc Pitch: ");
      Serial.print(seqCVModOscPitchEnabled ? "ENABLED" : "DISABLED");
    }
    if (matrixStates[2][1] != lastMatrixStates[2][1]) {
      Serial.print(" | Pulsar Env ModOsc Pitch: ");
      Serial.print(pulsarModOscPitchEnabled ? "ENABLED" : "DISABLED");
    }
    if (matrixStates[0][2] != lastMatrixStates[0][2]) {
      Serial.print(" | Seq CV Mod Amount: ");
      Serial.print(seqCVModAmountEnabled ? "ENABLED" : "DISABLED");
    }
    if (matrixStates[2][2] != lastMatrixStates[2][2]) {
      Serial.print(" | Pulsar Env Mod Amount: ");
      Serial.print(pulsarModAmountEnabled ? "ENABLED" : "DISABLED");
    }
    if (matrixStates[0][3] != lastMatrixStates[0][3]) {
      Serial.print(" | Seq CV ComplexOsc Pitch: ");
      Serial.print(seqCVComplexOscPitchEnabled ? "ENABLED" : "DISABLED");
    }
    if (matrixStates[2][0] != lastMatrixStates[2][0]) {
      Serial.print(" | Pulsar Self-Mod: ");
      Serial.print(pulsarSelfModEnabled ? "ENABLED" : "DISABLED");
    }
    if (matrixStates[2][3] != lastMatrixStates[2][3]) {
      Serial.print(" | Pulsar Env ComplexOsc Pitch: ");
      Serial.print(pulsarModComplexOscPitchEnabled ? "ENABLED" : "DISABLED");
    }
    if (matrixStates[2][4] != lastMatrixStates[2][4]) {
      Serial.print(" | Pulsar Env Wavefolder Mod: ");
      Serial.print(pulsarModWavefolderEnabled ? "ENABLED" : "DISABLED");
    }
    if (matrixStates[0][5] != lastMatrixStates[0][5]) {
      Serial.print(" | Seq CV LPG Ch1 Level: ");
      Serial.print(seqCVLPGCh1LevelEnabled ? "ENABLED" : "DISABLED");
    }
    if (matrixStates[0][6] != lastMatrixStates[0][6]) {
      Serial.print(" | Seq CV LPG Ch2 Level: ");
      Serial.print(seqCVLPGCh2LevelEnabled ? "ENABLED" : "DISABLED");
    }
    if (matrixStates[1][5] != lastMatrixStates[1][5]) {
      Serial.print(" | Env Mod Ch1: ");
      Serial.print(envModCh1Enabled ? "ENABLED" : "DISABLED");
    }
    if (matrixStates[1][6] != lastMatrixStates[1][6]) {
      Serial.print(" | Env Mod Ch2: ");
      Serial.print(envModCh2Enabled ? "ENABLED" : "DISABLED");
    }
    if (matrixStates[2][5] != lastMatrixStates[2][5]) {
      Serial.print(" | Pulsar Env LPG Ch1 Mod: ");
      Serial.print(pulsarModLPGCh1Enabled ? "ENABLED" : "DISABLED");
    }
    if (matrixStates[2][6] != lastMatrixStates[2][6]) {
      Serial.print(" | Pulsar Env LPG Ch2 Mod: ");
      Serial.print(pulsarModLPGCh2Enabled ? "ENABLED" : "DISABLED");
    }

    Serial.println();
  }
}

// Update sequencer step
void updateSequencer() {
  unsigned long currentTime = millis();
  STEP_DURATION_MS = (60000.0f / BPM) / 4.0f;

  if (!useMidiClock && currentTime - stepStartTime >= STEP_DURATION_MS) {
    advanceStep();
  }
}

// Advance to the next sequencer step
void advanceStep() {
  currentStep = (currentStep + 1) % SEQ_STEPS;
  stepStartTime = millis();

  gateOpen = true;
  lastGateTime = millis();
  env.Retrigger(false);
  env.Retrigger(true);
}

// Read all sequencer potentiometers
void readSequencerValues() {
  sequencerValues[0] = readMux1Channel(SEQ_STEP_1_CHANNEL, 0.0f, 48.0f);
  sequencerValues[1] = readMux1Channel(SEQ_STEP_2_CHANNEL, 0.0f, 48.0f);
  sequencerValues[2] = readMux1Channel(SEQ_STEP_3_CHANNEL, 0.0f, 48.0f);
  sequencerValues[3] = readMux1Channel(SEQ_STEP_4_CHANNEL, 0.0f, 48.0f);
  sequencerValues[4] = readMux1Channel(SEQ_STEP_5_CHANNEL, 0.0f, 48.0f);
}

// MIDI Note On handler
void handleNoteOn(byte channel, byte note, byte velocity) {
  midiNoteReceived = true;
  lastMidiNote = note;
  lastMidiVelocity = velocity;
  lastMidiChannel = channel;
  midiNoteCount++;

  // Convert MIDI note to CV and store it (this value will persist)
  midiPitchCV = midiNoteToCV(note);
  midiNoteActive = true;  // For triggering sequencer if needed
  lastMidiNoteTime = millis();

  if (useMidiClock) {
    advanceStep();
  }

  Serial.print("MIDI Note On - Channel: ");
  Serial.print(channel);
  Serial.print(" Note: ");
  Serial.print(note);
  Serial.print(" Velocity: ");
  Serial.print(velocity);
  Serial.print(" CV: ");
  Serial.print(midiPitchCV);
  Serial.println(" semitones");
}

// MIDI Note Off handler
void handleNoteOff(byte channel, byte note, byte velocity) {
  // Only turn off the active flag for triggering purposes
  // BUT DO NOT reset midiPitchCV - it persists!
  if (note == lastMidiNote) {
    midiNoteActive = false;  // This only affects triggering, not pitch
  }

  Serial.print("MIDI Note Off - Channel: ");
  Serial.print(channel);
  Serial.print(" Note: ");
  Serial.print(note);
  Serial.print(" Velocity: ");
  Serial.println(velocity);
}

void AudioCallback(float** in, float** out, size_t size) {
  for (size_t i = 0; i < size; i++) {
    // Get current sequencer pitch offset
    float sequencerPitchOffset = sequencerValues[currentStep];

    // Combine all pitch sources: pots + sequencer + MIDI (if enabled)
    float totalPitchOffset = sequencerPitchOffset;
    if (midiPitchEnabled) {
      totalPitchOffset += midiPitchCV;
    }

    float pitchRatio = semitonesToRatio(totalPitchOffset);

    // PROCESS PULSAR ADSR ENVELOPE (for modulation use only)
    // Trigger the envelope on each sequencer step with proper gate management
    unsigned long currentTime = millis();
    static bool pulsarEnvTriggered = false;
    
    // Calculate when to close the gate (at 90% of step duration to allow for release)
    float gateCloseTime = STEP_DURATION_MS * 0.9f;
    
    if (currentTime - lastPulsarEnvTime > STEP_DURATION_MS) {
      // Close gate first to ensure clean retrigger
      pulsarEnv_gate = false;
      // Small delay to allow release to complete (in samples)
      for(int j = 0; j < 10; j++) {
        pulsarEnv.Process(false);
      }
      // Then reopen gate
      pulsarEnv_gate = true;
      lastPulsarEnvTime = currentTime;
      pulsarEnvTriggered = true;
    }
    
    // Close gate before end of step to allow release
    if (pulsarEnvTriggered && (currentTime - lastPulsarEnvTime > gateCloseTime)) {
      pulsarEnv_gate = false;
      pulsarEnvTriggered = false;
    }

    // Process the envelope and convert to sawtooth shape
    float pulsarEnvValue = pulsarEnv.Process(pulsarEnv_gate);

    // Convert envelope to REGULAR sawtooth shape (starts low, rises to high)
    pulsarEnv_sawtoothValue = pulsarEnvValue;

    // Calculate base decay time from C9 MUX1 pot
    float baseDecayFromPot = pulsarEnv_baseDecayTime;

    // NEW: Apply sequencer CV modulation to pulsar envelope decay time if B0+A0 is pressed
    if (seqCVPulsarEnvDecayEnabled) {
      // Convert sequencer CV (0-48 semitones) to modulation amount (0.0-1.0 range)
      float seqCVMod = sequencerPitchOffset / 48.0f;
      
      // Sequencer CV modulates decay time from the base value
      // At seqCVMod = 0.0: decay time = baseDecayFromPot * 0.1 (10% of base)
      // At seqCVMod = 1.0: decay time = baseDecayFromPot * 2.0 (200% of base)
      float decayModRange = 10.0f;  // From 0.1x to 2.0x of base decay time
      float decayModFactor = 0.1f + (seqCVMod * (decayModRange - 0.1f));
      baseDecayFromPot = pulsarEnv_baseDecayTime * decayModFactor;
      
      // Clamp to reasonable values
      baseDecayFromPot = fmaxf(baseDecayFromPot, 0.001f);
      baseDecayFromPot = fminf(baseDecayFromPot, 20.0f);
    }

    // APPLY SELF-MODULATION IF B2+A0 IS PRESSED (uses the potentially sequencer-modulated base decay)
    if (pulsarSelfModEnabled) {
      // Use the sawtooth value to modulate the decay time
      // Scale the modulation appropriately (0.1x to 2x of base decay time)
      float modAmount = 0.5f;  // Modulation depth (can be made into a parameter)
      float modFactor = 1.0f + (pulsarEnv_sawtoothValue * modAmount);
      pulsarEnv_modulatedDecayTime = baseDecayFromPot * modFactor;

      // Clamp to reasonable values
      pulsarEnv_modulatedDecayTime = fmaxf(pulsarEnv_modulatedDecayTime, 0.001f);
      pulsarEnv_modulatedDecayTime = fminf(pulsarEnv_modulatedDecayTime, 20.0f);
    } else {
      // No self-modulation, use base decay time (which may be sequencer-modulated)
      pulsarEnv_modulatedDecayTime = baseDecayFromPot;
    }

    // Update envelope parameters with current decay time
    pulsarEnv.SetTime(ADSR_SEG_ATTACK, pulsarEnv_attackTime);
    pulsarEnv.SetTime(ADSR_SEG_DECAY, pulsarEnv_modulatedDecayTime);
    pulsarEnv.SetTime(ADSR_SEG_RELEASE, pulsarEnv_releaseTime);

    // PROCESS MODULATOR OSCILLATOR with pitch modulation
    float modulatedModPitch;

    // Calculate base pitch from C11 pot
    float baseModPitch = modOsc_pitch;

    // Apply sequencer CV modulation if B0+A1 is pressed
    if (seqCVModOscPitchEnabled) {
      baseModPitch *= pitchRatio;
    }

    // Apply pulsar envelope modulation if B2+A1 is pressed
    if (pulsarModOscPitchEnabled) {
      // Scale the pulsar envelope to create meaningful pitch modulation
      // The envelope ranges from 0 to 1 (sawtooth shape)
      float pulsarModAmount = 0.5f;  // You can make this a pot-controlled parameter if desired

      // Use multiplicative scaling for frequency modulation
      float pulsarPitchMod = 1.0f + (pulsarEnv_sawtoothValue * pulsarModAmount);
      modulatedModPitch = baseModPitch * pulsarPitchMod;

      // Clamp to prevent excessive frequencies
      modulatedModPitch = fmaxf(modulatedModPitch, 1.0f);
      modulatedModPitch = fminf(modulatedModPitch, 10000.0f);
    } else {
      modulatedModPitch = baseModPitch;
    }

    modOsc.SetFreq(modulatedModPitch);
    float modOsc_signal = modOsc.Process();

    // APPLY ANALOG FILTER TO MOD OSCILLATOR
    float modOsc_filteredSignal = modOsc_analogFilter.Process(modOsc_signal);

    // PROCESS COMPLEX OSCILLATOR with pitch modulation
    float modulatedComplexBasePitch;

    // Calculate base pitch from C15 pot
    float baseComplexPitch = complexOsc_basePitch;

    // Apply sequencer CV modulation if B0+A3 is pressed
    if (seqCVComplexOscPitchEnabled) {
      baseComplexPitch *= pitchRatio;
    }

    // Apply pulsar envelope modulation if B2+A3 is pressed
    if (pulsarModComplexOscPitchEnabled) {
      // Scale the pulsar envelope to create meaningful pitch modulation
      // The envelope ranges from 0 to 1 (sawtooth shape)
      float pulsarModAmount = 0.5f;  // You can make this a pot-controlled parameter if desired

      // Use multiplicative scaling for frequency modulation (same as modOsc)
      float pulsarPitchMod = 1.0f + (pulsarEnv_sawtoothValue * pulsarModAmount);
      modulatedComplexBasePitch = baseComplexPitch * pulsarPitchMod;

      // Clamp to prevent excessive frequencies
      modulatedComplexBasePitch = fmaxf(modulatedComplexBasePitch, 1.0f);
      modulatedComplexBasePitch = fminf(modulatedComplexBasePitch, 10000.0f);
    } else {
      modulatedComplexBasePitch = baseComplexPitch;
    }

    float complexOsc_freq = modulatedComplexBasePitch;

    // Get envelope value once per sample
    float envValue = env.Process(gateOpen);

    // Calculate base modulation parameters from C14 pot
    float baseModAmount = modOsc_modAmount;

    // MODULATION ROUTING:
    // When B0+A2 is pressed: Sequencer CV modulates AM depth (AM mode) or FM depth (FM mode)
    // When B2+A2 is pressed: Pulsar envelope modulates AM depth (AM mode) or FM depth (FM mode)
    // Both can be active simultaneously

    // FIXED: Better scaling for AM depth - map 0-800 range to 0.0-1.0 more intuitively
    float finalAMDepth = baseModAmount / 800.0f;  // Base AM depth from C14 pot (0.0-1.0)
    float finalFMDepth = baseModAmount * 3.0f;    // Base FM depth from C14 pot

    // Apply sequencer CV modulation if B0+A2 is pressed
    if (seqCVModAmountEnabled) {
      // Convert sequencer CV (in semitones) to a modulation amount
      // Normalize sequencer CV to 0-1 range (assuming 0-48 semitones range)
      float seqCVMod = sequencerPitchOffset / 48.0f;

      if (useAmplitudeModulation) {
        // In AM mode: sequencer CV modulates the AM depth
        // Add up to 0.5 to the depth (so sequencer can double the depth)
        finalAMDepth = fminf(fmaxf(finalAMDepth + (seqCVMod * 0.5f), 0.0f), 1.0f);
      } else {
        // In FM mode: sequencer CV modulates the FM depth
        finalFMDepth += (seqCVMod * 400.0f);  // Scale appropriately for FM
      }
    }

    // Apply pulsar envelope modulation if B2+A2 is pressed
    if (pulsarModAmountEnabled) {
      float pulsarDepthMod = pulsarEnv_sawtoothValue * 1000.0f;  //
      if (useAmplitudeModulation) {
        // FIXED: Use appropriate pulsar modulation for AM (was 500.0f - way too high!)
        finalAMDepth = fminf(fmaxf(finalAMDepth + pulsarDepthMod, 0.0f), 1.0f);
      } else {
        // FM mode: pulsar envelope modulates the FM depth
        finalFMDepth += pulsarDepthMod;
      }
    }

    // Clamp to prevent excessive values
    finalAMDepth = fminf(fmaxf(finalAMDepth, 0.0f), 1.0f);
    finalFMDepth = fminf(fmaxf(finalFMDepth, 0.0f), 1600.0f);

    // ================================
    // MODIFIED WAVEFOLDER MODULATION
    // ================================

    // Start with the base wavefolder amount from C3 pot (MUX2)
    float currentFoldAmount = complexOsc_foldAmount;

    // Apply envelope modulation if enabled (B1+A4)
    if (wavefolderEnvModEnabled) {
      // Envelope modulates the wavefolder amount - additive to base
      currentFoldAmount += (wavefolderEnvModDepth * envValue * 0.5f);
    }

    // Apply sequencer CV modulation if enabled (B0+A4)
    if (seqCVWavefolderModEnabled) {
      // Convert sequencer CV (in semitones) to a modulation amount
      // Normalize sequencer CV to 0-1 range (assuming 0-48 semitones range)
      float seqCVMod = sequencerPitchOffset / 48.0f;
      // Apply modulation with depth control - additive to base
      currentFoldAmount += (seqCVWavefolderModDepth * seqCVMod * 0.5f);
    }

    // Apply pulsar envelope modulation if B2+A4 is pressed - MODULATES FROM BASE
    if (pulsarModWavefolderEnabled) {
      // Use the C3 pot value as the CENTER point, pulsar modulates around it
      // wavefolderEnvModDepth controls how much modulation is applied
      float baseAmount = complexOsc_foldAmount;
      float modulationDepth = wavefolderEnvModDepth * 0.8f;  // Scale the modulation depth

      // Pulsar envelope sweeps from 0 to 1, creating modulation around the base value
      // This creates: baseAmount + (pulsarEnv * modulationDepth)
      // So when pulsarEnv=0: baseAmount, when pulsarEnv=1: baseAmount + modulationDepth
      float pulsarModulation = baseAmount + (pulsarEnv_sawtoothValue * modulationDepth);

      // Blend between unmodulated and modulated based on pulsar envelope
      currentFoldAmount = pulsarModulation;
    }

    // Clamp to prevent excessive folding
    currentFoldAmount = fminf(fmaxf(currentFoldAmount, 0.0f), 1.0f);

    // ===========================================================================
    // END OF MODIFIED WAVEFOLDER SECTION
    // ===========================================================================

    // Determine the appropriate parameters for each channel based on LPG mode
    float ch1_level, ch1_cutoffControl, ch2_level, ch2_cutoffControl;
    float ch1_baseLevel, ch2_baseLevel;  // Base levels from MUX2 C6/C8

    // For Channel 1 (complex oscillator)
    if (lpgChannel1_mode == LPG_MODE_LP) {
      ch1_level = 0.3f;                       // Reduced static level in LP mode to match other modes
      ch1_cutoffControl = lpgCh1_baseCutoff;  // Use MUX2 C6 pot for cutoff control (backup)
      ch1_baseLevel = lpgCh1Level;            // Use MUX2 C6 pot as base cutoff frequency in LP mode
    } else if (lpgChannel1_mode == LPG_MODE_VCA || lpgChannel1_mode == LPG_MODE_COMBI) {
      // Apply sequencer CV modulation to LPG Channel 1 level if B0+A5 is pressed
      if (seqCVLPGCh1LevelEnabled) {
        // Use C6 MUX2 pot (lpgCh1Level) as base, sequencer CV modulates from that point
        // Convert sequencer CV (0-48 semitones) to modulation amount (0.0-1.0 range)
        float seqCVMod = sequencerPitchOffset / 48.0f;

        // Start from C6 MUX2 pot value and add sequencer modulation
        ch1_level = lpgCh1Level + (seqCVMod * 0.5f);  // 0.5f limits max modulation to prevent excessive levels

        // Clamp to safe range
        ch1_level = fminf(fmaxf(ch1_level, 0.0f), 1.0f);
      } else {
        ch1_level = 1.0f;  // Use full level in VCA/COMBI mode - level controlled by LPG
      }
      ch1_cutoffControl = complexOsc_level;  // Use level pot for cutoff control
      ch1_baseLevel = lpgCh1Level;           // Base level from MUX2 C6
    } else {
      ch1_level = complexOsc_level;          // Use level pot (for any other modes)
      ch1_cutoffControl = complexOsc_level;  // Use level for cutoff
      ch1_baseLevel = 1.0f;                  // Full level
    }

    // For Channel 2 (modulator oscillator)
    if (lpgChannel2_mode == LPG_MODE_LP) {
      ch2_level = 0.3f;                       // Reduced static level in LP mode to match other modes
      ch2_cutoffControl = lpgCh2_baseCutoff;  // Use MUX2 C8 pot for cutoff control (backup)
      ch2_baseLevel = lpgCh2Level;            // Use MUX2 C8 pot as base cutoff frequency in LP mode
    } else if (lpgChannel2_mode == LPG_MODE_VCA || lpgChannel2_mode == LPG_MODE_COMBI) {
      ch2_level = 1.0f;                  // Use full level in VCA/COMBI mode - level controlled by LPG
      ch2_cutoffControl = modOsc_level;  // Use level pot for cutoff control
      ch2_baseLevel = lpgCh2Level;       // Base level from MUX2 C8
    } else {
      ch2_level = modOsc_level;          // Use level pot (for any other modes)
      ch2_cutoffControl = modOsc_level;  // Use level for cutoff
      ch2_baseLevel = 1.0f;              // Full level
    }

    // Calculate sequencer CV modulation values (normalized 0-1 range)
    float seqCVMod_ch1 = sequencerPitchOffset / 48.0f;
    float seqCVMod_ch2 = sequencerPitchOffset / 48.0f;

    // APPLY MODULATION BASED ON SELECTED TYPE
    if (useAmplitudeModulation) {
      // TRADITIONAL AMPLITUDE MODULATION (AM)
      // C14 MUX1 pot controls modulation depth (0.0-1.0)
      // At 0 depth: no modulation, constant amplitude
      // At full depth: full modulation depth

      // Generate complex oscillator signal
      complexOsc.SetFreq(complexOsc_freq);
      complexOscTri.SetFreq(complexOsc_freq);

      float complexOsc_sineSignal = complexOsc.Process();
      float complexOsc_triSignal = complexOscTri.Process();
      float complexOsc_rawSignal = (complexOsc_sineSignal * (1.0f - complexOsc_timbreAmount)) + (complexOsc_triSignal * complexOsc_timbreAmount);
      float complexOsc_filteredSignal = complexOsc_analogFilter.Process(complexOsc_rawSignal);
      float complexOsc_foldedSignal = wavefolder(complexOsc_filteredSignal, currentFoldAmount);

      // Generate modulator signal (scaled appropriately for AM)
      // Use bipolar modulation for proper AM (oscillates between -1 and 1)
      float modSignal = modOsc_filteredSignal * 50.0f;  // Reduced level for AM

      // Traditional AM formula: carrier * (1 + depth * modulator)
      // This gives us the classic AM sound where depth controls how much the modulator affects the amplitude
      float amSignal = complexOsc_foldedSignal * ch1_level * (1.0f + (finalAMDepth * modSignal));

      // Clamp to prevent excessive amplitudes
      amSignal = fmaxf(amSignal, -1.0f);
      amSignal = fminf(amSignal, 1.0f);

      // Apply low-pass filtering to reduce aliasing in AM signal
      static float prevAmSignal = 0.0f;
      float filteredAmSignal = 0.9f * amSignal + 0.1f * prevAmSignal;
      prevAmSignal = filteredAmSignal;

      float modulated_complexOsc = filteredAmSignal;

      // Process modulator through LPG (for monitoring/feedback purposes)
      float modulated_modOsc = modOsc_filteredSignal * ch2_level;

      // APPLY LPG FILTER TO MODULATOR OSC with envelope modulation depth, sequencer CV, AND PULSER MODULATION
      processLPG(lpgChannel2_filter, lpgChannel2_mode, modulated_modOsc, modOsc_level, ch2_cutoffControl, envValue, ch2_baseLevel, envModCh2Enabled, envModDepth_ch2, seqCVMod_ch2, seqCVModDepth_ch2, pulsarEnv_sawtoothValue, pulsarModLPGCh2Enabled);

      // APPLY LPG FILTER TO AM SIGNAL with envelope modulation depth, sequencer CV, AND PULSER MODULATION
      processLPG(lpgChannel1_filter, lpgChannel1_mode, modulated_complexOsc, complexOsc_level, ch1_cutoffControl, envValue, ch1_baseLevel, envModCh1Enabled, envModDepth_ch1, seqCVMod_ch1, seqCVModDepth_ch1, pulsarEnv_sawtoothValue, pulsarModLPGCh1Enabled);

      // Mix: mostly AM signal with a tiny bit of modulator for character
      float oscillatorSum_signal = modulated_complexOsc + (modulated_modOsc * 0.05f);

      // APPLY REVERB
      float wetL, wetR;
      verb.Process(oscillatorSum_signal, oscillatorSum_signal, &wetL, &wetR);

      // Mix dry and wet signals
      float finalL = (oscillatorSum_signal * (1.0f - reverbMix)) + (wetL * reverbMix);
      float finalR = (oscillatorSum_signal * (1.0f - reverbMix)) + (wetR * reverbMix);

      out[0][i] = finalL;
      out[1][i] = finalR;

    } else {
      // FREQUENCY MODULATION (FM) - UNCHANGED
      // Full modulator level for FM mode
      float modulated_modOsc = modOsc_filteredSignal * ch2_level;

      // Use the potentially modulated FM depth
      float modulatorSignal = modOsc_filteredSignal * finalFMDepth;
      float complexOsc_modulatedFreq = complexOsc_freq + modulatorSignal - 16.0f;
      complexOsc_modulatedFreq = max(complexOsc_modulatedFreq, 17.0f);

      complexOsc.SetFreq(complexOsc_modulatedFreq);
      complexOscTri.SetFreq(complexOsc_modulatedFreq);

      float complexOsc_sineSignal = complexOsc.Process();
      float complexOsc_triSignal = complexOscTri.Process();
      float complexOsc_rawSignal = (complexOsc_sineSignal * (1.0f - complexOsc_timbreAmount)) + (complexOsc_triSignal * complexOsc_timbreAmount);
      float complexOsc_filteredSignal = complexOsc_analogFilter.Process(complexOsc_rawSignal);
      float complexOsc_foldedSignal = wavefolder(complexOsc_filteredSignal, currentFoldAmount);

      float modulated_complexOsc = complexOsc_foldedSignal * ch1_level;

      // APPLY LPG FILTER TO MODULATOR OSC with envelope modulation depth, sequencer CV, AND PULSER MODULATION
      processLPG(lpgChannel2_filter, lpgChannel2_mode, modulated_modOsc, modOsc_level, ch2_cutoffControl, envValue, ch2_baseLevel, envModCh2Enabled, envModDepth_ch2, seqCVMod_ch2, seqCVModDepth_ch2, pulsarEnv_sawtoothValue, pulsarModLPGCh2Enabled);

      // APPLY LPG FILTER TO AM SIGNAL with envelope modulation depth, sequencer CV, AND PULSER MODULATION
      processLPG(lpgChannel1_filter, lpgChannel1_mode, modulated_complexOsc, complexOsc_level, ch1_cutoffControl, envValue, ch1_baseLevel, envModCh1Enabled, envModDepth_ch1, seqCVMod_ch1, seqCVModDepth_ch1, pulsarEnv_sawtoothValue, pulsarModLPGCh1Enabled);

      float oscillatorSum_signal = modulated_complexOsc + modulated_modOsc;

      // APPLY REVERB
      float wetL, wetR;
      verb.Process(oscillatorSum_signal, oscillatorSum_signal, &wetL, &wetR);

      // Mix dry and wet signals
      float finalL = (oscillatorSum_signal * (1.0f - reverbMix)) + (wetL * reverbMix);
      float finalR = (oscillatorSum_signal * (1.0f - reverbMix)) + (wetR * reverbMix);

      out[0][i] = finalL;
      out[1][i] = finalR;
    }
  }
}


void setup() {
  Serial.begin(115200);  // Increased baud rate for faster debugging

  // INIT BLOCK SIZE

  // INIT NEW 4x7 BUTTON MATRIX
  initButtonMatrix();
  
  // INIT SECOND BUTTON MATRIX
  initButtonMatrix2();

  // REMOVED individual button initialization since we're using matrix now

  // INIT MUX_1 PINS
  pinMode(MUX1_S0, OUTPUT);
  pinMode(MUX1_S1, OUTPUT);
  pinMode(MUX1_S2, OUTPUT);
  pinMode(MUX1_S3, OUTPUT);

  // INIT MUX_2 PINS
  pinMode(MUX2_S0, OUTPUT);
  pinMode(MUX2_S1, OUTPUT);
  pinMode(MUX2_S2, OUTPUT);
  pinMode(MUX2_S3, OUTPUT);

  // Set both MUXes to initial state
  digitalWrite(MUX1_S0, LOW);
  digitalWrite(MUX1_S1, LOW);
  digitalWrite(MUX1_S2, LOW);
  digitalWrite(MUX1_S3, LOW);

  digitalWrite(MUX2_S0, LOW);
  digitalWrite(MUX2_S1, LOW);
  digitalWrite(MUX2_S2, LOW);
  digitalWrite(MUX2_S3, LOW);

  // INIT 16-BIT ADC
  analogReadResolution(16);

  // INIT MIDI - Configure Serial1 to use pin 30 for RX
  Serial1.setRx(MIDI_RX_PIN);

  // Try different approaches for MIDI initialization
  Serial1.begin(31250);  // Standard MIDI baud rate

  // Configure MIDI library for better performance
  MIDI.begin(MIDI_CHANNEL_OMNI);
  MIDI.setHandleNoteOn(handleNoteOn);
  MIDI.setHandleNoteOff(handleNoteOff);

  // Increase MIDI throughput (if supported by the library)
  MIDI.turnThruOff();  // Disable MIDI thru to reduce load

  Serial.println("MIDI initialized with pin 30 (Digital pin 30)");

  // INIT SEED AT 48kHz
  float sample_rate;
  hw = DAISY.init(DAISY_SEED, AUDIO_SR_48K);
  sample_rate = DAISY.get_samplerate();
  DAISY.SetAudioBlockSize(128);

  // INIT OSCILLATORS
  complexOsc.Init(sample_rate);
  complexOscTri.Init(sample_rate);
  modOsc.Init(sample_rate);

  // INIT PULSAR ADSR ENVELOPE (replaces pulsarOsc)
  pulsarEnv.Init(sample_rate);

  // INIT COMPLEX OSC FILTER
  complexOsc_analogFilter.Init(sample_rate);
  complexOsc_analogFilter.SetFreq(15000.0f);
  complexOsc_analogFilter.SetRes(0.3f);

  // INIT MOD OSC FILTER (same settings as complexOsc_analogFilter)
  modOsc_analogFilter.Init(sample_rate);
  modOsc_analogFilter.SetFreq(15000.0f);
  modOsc_analogFilter.SetRes(0.3f);

  // INIT LPG FILTERS
  lpgChannel1_filter.Init(sample_rate);
  lpgChannel1_filter.SetFreq(15000.0f);
  lpgChannel1_filter.SetRes(0.0f);  // No resonance

  lpgChannel2_filter.Init(sample_rate);
  lpgChannel2_filter.SetFreq(15000.0f);
  lpgChannel2_filter.SetRes(0.0f);  // No resonance

  // INIT ADSR ENVELOPE
  env.Init(sample_rate);

  // REVERB INIT
  verb.Init(sample_rate);
  verb.SetFeedback(0.85f);
  verb.SetLpFreq(18000.0f);

  // OSCILLATORS INIT
  complexOsc.SetWaveform(complexOsc.WAVE_SIN);
  complexOsc.SetAmp(1.0);

  complexOscTri.SetWaveform(complexOscTri.WAVE_SQUARE);
  complexOscTri.SetAmp(1.0);

  modOsc.SetWaveform(modOsc.WAVE_TRI);
  modOsc.SetAmp(0.5);

  // PULSAR ADSR ENVELOPE INIT
  // Set fixed attack time of 0.02 seconds
  pulsarEnv_attackTime = 0.02f;
  // Set fixed release time of 0.02 seconds
  pulsarEnv_releaseTime = 0.02f;
  // Initial decay time will be set from pot reading
  pulsarEnv_baseDecayTime = 0.1f;
  pulsarEnv_modulatedDecayTime = pulsarEnv_baseDecayTime;

  // Configure ADSR envelope with fixed sustain at 0 and release
  pulsarEnv.SetTime(ADSR_SEG_ATTACK, pulsarEnv_attackTime);
  pulsarEnv.SetTime(ADSR_SEG_DECAY, pulsarEnv_modulatedDecayTime);
  pulsarEnv.SetTime(ADSR_SEG_RELEASE, pulsarEnv_releaseTime);
  pulsarEnv.SetSustainLevel(0.0f);  // Go to zero after decay

  // OSC INIT
  complexOsc_basePitch = 440.0f;
  modOsc_pitch = 1.0f;
  modOsc_modAmount = 0.0f;
  complexOsc_timbreAmount = 0.0f;
  complexOsc_foldAmount = 0.0f;
  complexOsc_level = 1.0f;
  modOsc_level = 1.0f;

  // LP MODE CUTOFF INIT
  lpgCh1_baseCutoff = 0.5f;
  lpgCh2_baseCutoff = 0.5f;

  // LPG BASE LEVEL INIT
  lpgCh1Level = 1.0f;
  lpgCh2Level = 1.0f;

  // WAVEFOLDER MODULATION INIT
  wavefolderEnvModDepth = 1.0f;  // Default modulation depth
  wavefolderEnvModEnabled = false;
  seqCVWavefolderModDepth = 1.0f;  // Default modulation depth
  seqCVWavefolderModEnabled = false;
  seqCVModDepth_ch1 = 0.0f;                 // Disabled by default - sequencer CV only when B0+A5 pressed
  seqCVModDepth_ch2 = 0.0f;                 // Disabled by default - sequencer CV only when B0+A6 pressed
  seqCVPulsarEnvDecayEnabled = false;       // NEW: Sequencer CV modulation of pulsar envelope decay disabled by default
  seqCVModAmountEnabled = false;            // Sequencer CV control of mod amount disabled by default
  seqCVComplexOscPitchEnabled = false;      // Sequencer CV control of complexOsc pitch disabled by default
  pulsarModOscPitchEnabled = false;         // Pulsar envelope modulation of modOsc pitch disabled by default
  pulsarModAmountEnabled = false;           // Pulsar envelope modulation of modAmount disabled by default
  pulsarSelfModEnabled = false;             // Pulsar self-modulation disabled by default
  pulsarModComplexOscPitchEnabled = false;  // Pulsar envelope modulation of complexOsc pitch disabled by default
  pulsarModWavefolderEnabled = false;       // Pulsar envelope modulation of wavefolder amount disabled by default

  // INIT NEW PULSER LPG MODULATION FLAGS
  pulsarModLPGCh1Enabled = false;
  pulsarModLPGCh2Enabled = false;

  // ENVELOPE MODULATION INIT
  envModCh1Enabled = false;
  envModCh2Enabled = false;

  // REVERB INIT
  reverbMix = 0.0f;  // Start with dry signal

  // ADSR INIT
  eg_attackTime = 0.1f;
  eg_decayTime = 0.1f;
  eg_sustainLevel = 1.0f;
  eg_releaseTime = 0.2f;
  env.SetTime(ADSR_SEG_ATTACK, eg_attackTime);
  env.SetTime(ADSR_SEG_DECAY, eg_decayTime);
  env.SetTime(ADSR_SEG_RELEASE, eg_releaseTime);
  env.SetSustainLevel(eg_sustainLevel);

  // Initialize sequencer values
  for (int i = 0; i < SEQ_STEPS; i++) {
    sequencerValues[i] = 0.0f;
  }

  BPM = 100.0f;
  STEP_DURATION_MS = (60000.0f / BPM) / 4.0f;
  stepStartTime = millis();

  // Start audio callback
  DAISY.begin(AudioCallback);

  Serial.println("Weasel Initialised with 4x7 Button Matrix and updated pin assignments");
  Serial.println("Second button matrix added with pins: X0-3: 12,13,17,18 | Y0-4: 8,9,10,11,14");
  Serial.println("Button functions moved to second matrix with debouncing:");
  Serial.println("  X3 Y1: Toggle FM/AM modulation");
  Serial.println("  X3 Y2: Toggle LPG mode on channel 1");
  Serial.println("  X0 Y3: Toggle LPG mode on channel 2");
  Serial.println("  X2 Y1: Toggle MIDI pitch control (enable/disable MIDI impact on oscillator pitches)");
  Serial.println("MUX pot assignments updated as requested");
}

void loop() {

  // 16 BIT ADC
  analogReadResolution(16);

  // MIDI PROCESS
  MIDI.read();

  // CHECK MIDI TIMEOUT
  if (midiNoteActive && (millis() - lastMidiNoteTime > MIDI_NOTE_TIMEOUT)) {
    midiNoteActive = false;
  }

  // READ NEW 4x7 BUTTON MATRIX
  if (millis() - lastMatrixRead > MATRIX_READ_INTERVAL) {
    readButtonMatrix();
    printButtonStates();
    lastMatrixRead = millis();
  }

  // READ SECOND BUTTON MATRIX
  if (millis() - lastMatrix2Read > MATRIX_READ_INTERVAL) {
    readButtonMatrix2();
    lastMatrix2Read = millis();
  }

  // POTENTIOMETER HANDLING - UPDATED MUX1 CHANNELS
  sequencerValues[0] = readMux1Channel(SEQ_STEP_1_CHANNEL, 0.0f, 48.0f);
  sequencerValues[1] = readMux1Channel(SEQ_STEP_2_CHANNEL, 0.0f, 48.0f);
  sequencerValues[2] = readMux1Channel(SEQ_STEP_3_CHANNEL, 0.0f, 48.0f);
  sequencerValues[3] = readMux1Channel(SEQ_STEP_4_CHANNEL, 0.0f, 48.0f);
  sequencerValues[4] = readMux1Channel(SEQ_STEP_5_CHANNEL, 0.0f, 48.0f);

  eg_attackTime = readMux1Channel(ASD_ATTACK_CHANNEL, 0.02f, 10.0f, true);
  eg_decayTime = readMux1Channel(ASD_SUSTAIN_CHANNEL, 0.02f, 10.0f, true);
  eg_sustainLevel = 1.0f;
  eg_releaseTime = readMux1Channel(ASD_DECAY_CHANNEL, 0.02f, 10.0f, true);

  // Read new MUX1 channels (placeholders for now)
  float pulsarEnvDecayControl = readMux1Channel(PULSAR_ENV_DECAYCONTROL_CHANNEL, 0.0f, 1.0f);
  pulsarEnv_baseDecayTime = readMux1Channel(PULSAR_ENV_DECAY_CHANNEL, 0.02f, 10.0f, true);
  float modOscPitchControl = readMux1Channel(MOD_OSC_PITCHCONTROL_CHANNEL, 0.0f, 1.0f);
  modOsc_pitch = readMux1Channel(MOD_OSC_PITCH_CHANNEL, 16.0f, 1760.0f, true);
  float modOscFinetune = readMux1Channel(MOD_OSC_FINETUNE_CHANNEL, 0.0f, 1.0f);
  float modAmountControl = readMux1Channel(MOD_AMOUNTCONTROL_CHANNEL, 0.0f, 1.0f);
  modOsc_modAmount = readMux1Channel(MOD_AMOUNT_CHANNEL, 0.0f, 800.0f);
  float complexOscPitchControl = readMux1Channel(COMPLEX_OSC_PITCHCONTROL_CHANNEL, 0.0f, 1.0f);

  // POTENTIOMETER HANDLING - UPDATED MUX2 CHANNELS
  float complexOscPitchControl2 = readMux2Channel(COMPLEX_OSC_PITCHCONTROL_CHANNEL, 0.0f, 1.0f);
  complexOsc_basePitch = readMux2Channel(COMPLEX_OSC_PITCHCONTROL_CHANNEL, 55.0f, 1760.0f, true);
  float complexOscFinetune = readMux2Channel(COMPLEX_OSC_FINETUNE_CHANNEL, 0.0f, 1.0f);
  float complexOscFoldControl = readMux2Channel(COMPLEX_OSC_FOLDCONTROL_CHANNEL, 0.0f, 1.0f);
  complexOsc_foldAmount = readMux2Channel(COMPLEX_OSC_FOLD_CHANNEL, 0.0f, 0.5f);
  complexOsc_timbreAmount = readMux2Channel(COMPLEX_OSC_TIMBRE_CHANNEL, 0.0f, 1.0f);
  float lpgCh1LevelControl = readMux2Channel(LPG_CH1_LEVELCONTROL, 0.0f, 1.0f);
  lpgCh1Level = readMux2Channel(LPG_CH1_LEVEL, 0.0f, 1.0f);
  float lpgCh2LevelControl = readMux2Channel(LPG_CH2_LEVELCONTROL, 0.0f, 1.0f);
  lpgCh2Level = readMux2Channel(LPG_CH2_LEVEL, 0.0f, 1.0f);
  BPM = readMux2Channel(CLOCK_CHANNEL, 1.0f, 1000.0f, true);
  reverbMix = readMux2Channel(REVERB_MIX, 0.0f, 1.0f);

  // Always read the level pots, but they'll be used differently based on LPG mode
  // Note: complexOsc_level and modOsc_level are now controlled by MUX2 C6 and C8
  complexOsc_level = lpgCh1Level;  // Use LPG level for complex oscillator
  modOsc_level = lpgCh2Level;      // Use LPG level for modulator oscillator

  // In LP mode, use the level pots as cutoff controls
  if (lpgChannel1_mode == LPG_MODE_LP) {
    lpgCh1_baseCutoff = complexOsc_level;
  }
  if (lpgChannel2_mode == LPG_MODE_LP) {
    lpgCh2_baseCutoff = modOsc_level;
  }

  // WAVEFOLDER MODULATION DEPTH CONTROLS - placeholder reads
  wavefolderEnvModDepth = 1.0f;  // Default modulation depth
  seqCVWavefolderModDepth = 1.0f;  // Default modulation depth

  // REMOVED individual button handling since we're using matrix now

  // ADR PARAMETERS
  env.SetTime(ADSR_SEG_ATTACK, eg_attackTime);
  env.SetTime(ADSR_SEG_DECAY, eg_decayTime);
  env.SetTime(ADSR_SEG_RELEASE, eg_releaseTime);
  env.SetSustainLevel(eg_sustainLevel);

  // PULSAR ADSR ENVELOPE PARAMETERS
  // Attack time is fixed at 0.02s, base decay time is controlled by MUX1 C9
  // The modulated decay time is calculated in the audio callback
  // Release time is fixed at 0.02s, sustain is fixed at 0
  pulsarEnv.SetTime(ADSR_SEG_ATTACK, pulsarEnv_attackTime);
  pulsarEnv.SetTime(ADSR_SEG_RELEASE, pulsarEnv_releaseTime);
  pulsarEnv.SetSustainLevel(0.0f);

  readSequencerValues();
  updateSequencer();

  // HANDLE ENV RELEASE
  float gateDurationMs = eg_decayTime * 1000.0f;
  if (gateDurationMs > STEP_DURATION_MS) {
    gateDurationMs = STEP_DURATION_MS;
  }

  if (gateOpen && (millis() - stepStartTime) > gateDurationMs) {
    gateOpen = false;
  }
}