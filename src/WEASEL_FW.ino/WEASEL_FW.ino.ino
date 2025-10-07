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

// BUTTON MATRIX PINS
#define BUTTON_COL 8   // Column pin
#define BUTTON_ROW0 9  // Row 0 pin  
#define BUTTON_ROW1 10 // Row 1 pin

// MUX CHANNEL ASSIGNMENTS (FIRST MUX - keeping all existing pots on MUX1)
#define MOD_OSC_PITCH_CHANNEL 0       // C0 - modOsc_pitch
#define MOD_AMOUNT_CHANNEL 1          // C1 - modOsc_modAmount
#define COMPLEX_OSC_PITCH_CHANNEL 2   // C2 - complexOsc_pitch
#define COMPLEX_OSC_TIMBRE_CHANNEL 3  // C3 - timbre control (sine/triangle blend)
#define COMPLEX_OSC_FOLD_CHANNEL 4    // C4 - wavefolding amount
#define COMPLEX_OSC_LEVEL_CHANNEL 5   // C5 - complex oscillator output level
#define MOD_OSC_LEVEL_CHANNEL 6       // C6 - modulator oscillator output level
#define SEQ_STEP_1_CHANNEL 7          // C7 - Sequencer step 1
#define SEQ_STEP_2_CHANNEL 8          // C8 - Sequencer step 2
#define SEQ_STEP_3_CHANNEL 9          // C9 - Sequencer step 3
#define SEQ_STEP_4_CHANNEL 10         // C10 - Sequencer step 4
#define SEQ_STEP_5_CHANNEL 11         // C11 - Sequencer step 5
#define ASD_ATTACK_CHANNEL 12         // C12 - BUCHLA ATTACK // ATTACK
#define ASD_SUSTAIN_CHANNEL 13        // C13 - BUCHLA SUSTAIN // DECAY
#define ASD_DECAY_CHANNEL 14          // C14 - BUCHLA DECAY // RELEASE
#define CLOCK_CHANNEL 15              // C15 - CLOCK

// MUX CHANNEL ASSIGNMENTS (SECOND MUX - modulation depth controls)
#define ENV_MOD_DEPTH_CH1 0  // C0 - Envelope modulation depth for Channel 1 (complex osc)
#define ENV_MOD_DEPTH_CH2 1  // C1 - Envelope modulation depth for Channel 2 (mod osc)
#define MUX2_CHANNEL_2 2
#define MUX2_CHANNEL_3 3
#define MUX2_CHANNEL_4 4
#define MUX2_CHANNEL_5 5
#define MUX2_CHANNEL_6 6
#define MUX2_CHANNEL_7 7
#define MUX2_CHANNEL_8 8
#define MUX2_CHANNEL_9 9
#define MUX2_CHANNEL_10 10
#define MUX2_CHANNEL_11 11
#define MUX2_CHANNEL_12 12
#define MUX2_CHANNEL_13 13
#define MUX2_CHANNEL_14 14
#define MUX2_CHANNEL_15 15

#define SEQUENCER_TOGGLE_BUTTON 19   // SEQ TOGGLE
#define MODULATION_TOGGLE_BUTTON 20  // MODULATION TOGGLE
#define LPG_CH1_TOGGLE_BUTTON 21     // LPG CHANNEL 1 TOGGLE
#define LPG_CH2_TOGGLE_BUTTON 22     // LPG CHANNEL 2 TOGGLE

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
static MoogLadder complexOsc_analogFilter;  // FILTER - HIGH CUT AT 15kHz FOR "ANALOGUE" FEEL OF WAVEFORMS
static MoogLadder modOsc_analogFilter;      // FILTER FOR MOD OSCILLATOR

// INIT LPG
static MoogLadder lpgChannel1_filter;  // FILTER FOR BUCHLA LPG CH1
static MoogLadder lpgChannel2_filter;  // FILTER FOR BUCHLA LPG CH2

// OSCILLATOR PARAMETER VARIABLES
float complexOsc_basePitch;     // COMPLEX OSC BASE PITCH
float modOsc_pitch;             // MOD OSC BASE PITCH
float modOsc_modAmount;         // MOD AMOUNT AFFECTING COMPLEX OSC
float complexOsc_timbreAmount;  // COMPLEX OSC TIMBRE WAVEMORPHING AMOUNT (0.0 = sine, 1.0 = triangle)
float complexOsc_foldAmount;    // COMPLEX OSC WAVEFOLDING AMOUNT (0.0 = no fold, 1.0 = max fold)
float complexOsc_level;         // COMPLEX OSC LEVEL CONTROL (0.0 to 1.0)
float modOsc_level;             // MODULATION OSC LEVEL CONTROL (0.0 to 1.0)

// LP MODE CUTOFF CONTROL VARIABLES
float lpgCh1_baseCutoff = 0.5f;  // Base cutoff for Channel 1 in LP mode (0.0 to 1.0)
float lpgCh2_baseCutoff = 0.5f;  // Base cutoff for Channel 2 in LP mode (0.0 to 1.0)

// ENVELOPE MODULATION DEPTH CONTROLS
float envModDepth_ch1 = 1.0f;  // Envelope modulation depth for channel 1 (complex osc)
float envModDepth_ch2 = 1.0f;  // Envelope modulation depth for channel 2 (mod osc)

// WAVEFOLDER ENVELOPE MODULATION
float wavefolderEnvModDepth = 0.0f;  // Depth of envelope modulation on wavefolder (0.0 to 1.0)
bool wavefolderEnvModEnabled = false; // Whether envelope modulation of wavefolder is active

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

// BUTTONS
Switch sequencerToggle;     // SEQUENCER CLOCK TOGGLE
Switch modulationToggle;    // MODULATION TYPE TOGGLE
Switch lpgToggle_channel1;  // LPG CH1 MODE TOGGLE
Switch lpgToggle_channel2;  // LPG CH2 MODE TOGGLE

// BUTTON MATRIX VARIABLES
bool buttonStates[2] = {false, false};  // Store states for row0 and row1
bool lastButtonStates[2] = {false, false};
unsigned long lastButtonRead = 0;
const unsigned long BUTTON_READ_INTERVAL = 50;  // Read buttons every 50ms

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

// Function to apply LPG processing based on mode with envelope modulation depth
void processLPG(MoogLadder& filter, LPGMode mode, float& signal, float channelLevel, float baseCutoffControl, float envValue, float modDepth) {
  float outputGain = 1.0f;

  // Apply modulation depth to envelope value
  // modDepth = 0.0: no envelope effect, modDepth = 1.0: full envelope effect
  float modulatedEnv = envValue * modDepth;

  switch (mode) {
    case LPG_MODE_COMBI:
      {
        // COMBI mode: MUX1 C5/C6 control base level AND base cutoff
        // MUX2 C0/C1 control envelope modulation for BOTH level AND cutoff

        // Calculate final level: baseLevel + envelope modulation (same as VCA mode)
        float baseLevel = channelLevel;  // From MUX1 C5/C6
        float modulatedLevel = baseLevel + ((1.0f - baseLevel) * modulatedEnv);

        // Apply the level to the signal
        signal *= modulatedLevel;

        // Base cutoff determined by channel level (20Hz to 20kHz)
        float baseCutoff = 20.0f + (channelLevel * 17980.0f);

        // Envelope opens the filter from base cutoff up to 20kHz
        float combiCutoff = baseCutoff + ((18000.0f - baseCutoff) * modulatedEnv);
        filter.SetFreq(fminf(combiCutoff, 18000.0f));
        filter.SetRes(0.0f);

        outputGain = 1.0f;
        signal = filter.Process(signal) * outputGain;
        break;
      }

    case LPG_MODE_VCA:
      {
        // VCA mode: MUX1 C5/C6 control base level, MUX2 C0/C1 control envelope modulation amount

        // Calculate final level: baseLevel + envelope modulation
        float baseLevel = channelLevel;  // From MUX1 C5/C6
        float modulatedLevel = baseLevel + ((1.0f - baseLevel) * modulatedEnv);

        // Apply the level to the signal
        signal *= modulatedLevel;

        // Filter cutoff increases with oscillator level (Buchla characteristic)
        float baseCutoff = 1200.0f + (channelLevel * 17800.0f);

        // Envelope also opens the filter slightly for timbral variation
        float vcaCutoff = baseCutoff + ((19000.0f - baseCutoff) * modulatedEnv * 0.5f);
        filter.SetFreq(fminf(vcaCutoff, 19000.0f));
        filter.SetRes(0.0f);

        outputGain = 1.0f;
        signal = filter.Process(signal) * outputGain;
        break;
      }

    case LPG_MODE_LP:
      {
        // LP mode: envelope controls ONLY the filter cutoff with modulation depth
        // Base cutoff determined by baseCutoffControl parameter (20Hz to 18kHz)
        float baseCutoff = 20.0f + (baseCutoffControl * 17980.0f);

        // Envelope opens the filter from base cutoff up to 18kHz
        float lpCutoff = baseCutoff + ((18000.0f - baseCutoff) * modulatedEnv);
        filter.SetFreq(fminf(lpCutoff, 18000.0f));

        // Safe resonance curve based on baseCutoffControl only (not envelope)
        float resonance = pow(baseCutoffControl, 1.8f) * 0.9f;
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

// BUTTON MATRIX FUNCTIONS
void initButtonMatrix() {
  pinMode(BUTTON_COL, OUTPUT);
  pinMode(BUTTON_ROW0, INPUT_PULLDOWN);
  pinMode(BUTTON_ROW1, INPUT_PULLDOWN);
  
  digitalWrite(BUTTON_COL, HIGH); // Set column high (pull-up)
}

void readButtonMatrix() {
  // Set column to HIGH (active)
  digitalWrite(BUTTON_COL, HIGH);
  delayMicroseconds(10); // Small delay for stabilization
  
  // Read each row
  lastButtonStates[0] = buttonStates[0];
  lastButtonStates[1] = buttonStates[1];
  
  buttonStates[0] = (digitalRead(BUTTON_ROW0) == HIGH);
  buttonStates[1] = (digitalRead(BUTTON_ROW1) == HIGH);
  
  // Set column back to LOW to save power (optional)
  digitalWrite(BUTTON_COL, LOW);
}

void printButtonStates() {
  // Print button states only when they change
  if (buttonStates[0] != lastButtonStates[0] || buttonStates[1] != lastButtonStates[1]) {
    Serial.print("Button Matrix - Row0: ");
    Serial.print(buttonStates[0] ? "HIGH" : "LOW");
    Serial.print(" | Row1: ");
    Serial.print(buttonStates[1] ? "HIGH" : "LOW");
    
    // Check if wavefolder envelope modulation was enabled/disabled
    if (buttonStates[0] != lastButtonStates[0]) {
      wavefolderEnvModEnabled = buttonStates[0];
      Serial.print(" | Wavefolder Env Mod: ");
      Serial.print(wavefolderEnvModEnabled ? "ENABLED" : "DISABLED");
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

    // Combine all pitch sources: pots + sequencer + MIDI
    float totalPitchOffset = sequencerPitchOffset + midiPitchCV;

    float pitchRatio = semitonesToRatio(totalPitchOffset);

    // PROCESS MODULATOR OSCILLATOR with combined pitch modulation
    float modulatedModPitch = modOsc_pitch * pitchRatio;
    modOsc.SetFreq(modulatedModPitch);
    float modOsc_signal = modOsc.Process();

    // APPLY ANALOG FILTER TO MOD OSCILLATOR
    float modOsc_filteredSignal = modOsc_analogFilter.Process(modOsc_signal);

    // PROCESS COMPLEX OSCILLATOR with combined pitch modulation
    float modulatedComplexBasePitch = complexOsc_basePitch * pitchRatio;
    float complexOsc_freq = modulatedComplexBasePitch;

    // Get envelope value once per sample
    float envValue = env.Process(gateOpen);

    // Calculate modulated wavefolder amount if enabled
    float currentFoldAmount = complexOsc_foldAmount;
    if (wavefolderEnvModEnabled) {
      // Envelope modulates the wavefolder amount
      // Base amount from pot + envelope modulation (0 to full amount based on envelope)
      currentFoldAmount = complexOsc_foldAmount + (wavefolderEnvModDepth * envValue);
      // Clamp to prevent excessive folding
      currentFoldAmount = fminf(currentFoldAmount, 1.0f);
    }

    // Determine the appropriate parameters for each channel based on LPG mode
    float ch1_level, ch1_cutoffControl, ch2_level, ch2_cutoffControl;

    // For Channel 1 (complex oscillator)
    if (lpgChannel1_mode == LPG_MODE_LP) {
      ch1_level = 0.3f;                       // Reduced static level in LP mode to match other modes
      ch1_cutoffControl = lpgCh1_baseCutoff;  // Use cutoff control pot
    } else if (lpgChannel1_mode == LPG_MODE_VCA || lpgChannel1_mode == LPG_MODE_COMBI) {
      ch1_level = 1.0f;                      // Use full level in VCA/COMBI mode - level controlled by LPG
      ch1_cutoffControl = complexOsc_level;  // Use level pot for cutoff control
    } else {
      ch1_level = complexOsc_level;          // Use level pot (for any other modes)
      ch1_cutoffControl = complexOsc_level;  // Use level for cutoff
    }

    // For Channel 2 (modulator oscillator)
    if (lpgChannel2_mode == LPG_MODE_LP) {
      ch2_level = 0.3f;                       // Reduced static level in LP mode to match other modes
      ch2_cutoffControl = lpgCh2_baseCutoff;  // Use cutoff control pot
    } else if (lpgChannel2_mode == LPG_MODE_VCA || lpgChannel2_mode == LPG_MODE_COMBI) {
      ch2_level = 1.0f;                  // Use full level in VCA/COMBI mode - level controlled by LPG
      ch2_cutoffControl = modOsc_level;  // Use level pot for cutoff control
    } else {
      ch2_level = modOsc_level;          // Use level pot (for any other modes)
      ch2_cutoffControl = modOsc_level;  // Use level for cutoff
    }

    // APPLY MODULATION BASED ON SELECTED TYPE
    if (useAmplitudeModulation) {
      // AMPLITUDE MODULATION (AM) - FIXED IMPLEMENTATION
      float amDepth = modOsc_modAmount * 0.5f;

      // Reduced modulator level for AM mode (consistent in both cases)
      float modulated_modOsc = modOsc_filteredSignal * ch2_level * 0.85f;

      // Only apply AM if depth is significant
      if (amDepth < 0.001f) {
        // No AM - just pass through complex oscillator
        complexOsc.SetFreq(complexOsc_freq);
        complexOscTri.SetFreq(complexOsc_freq);

        float complexOsc_sineSignal = complexOsc.Process();
        float complexOsc_triSignal = complexOscTri.Process();
        float complexOsc_rawSignal = (complexOsc_sineSignal * (1.0f - complexOsc_timbreAmount)) + (complexOsc_triSignal * complexOsc_timbreAmount);
        float complexOsc_filteredSignal = complexOsc_analogFilter.Process(complexOsc_rawSignal);
        float complexOsc_foldedSignal = wavefolder(complexOsc_filteredSignal, currentFoldAmount);

        float modulated_complexOsc = complexOsc_foldedSignal * ch1_level;

        // APPLY LPG FILTERS BASED ON CURRENT MODE with envelope modulation depth
        processLPG(lpgChannel1_filter, lpgChannel1_mode, modulated_complexOsc, complexOsc_level, ch1_cutoffControl, envValue, envModDepth_ch1);
        processLPG(lpgChannel2_filter, lpgChannel2_mode, modulated_modOsc, modOsc_level, ch2_cutoffControl, envValue, envModDepth_ch2);

        float oscillatorSum_signal = modulated_complexOsc + modulated_modOsc;
        out[0][i] = oscillatorSum_signal;
        out[1][i] = oscillatorSum_signal;
      } else {
        // Apply AM with proper level handling
        float amSignal = 1.0f + (amDepth * modOsc_filteredSignal);

        // Clamp to prevent excessive amplification
        amSignal = fmaxf(fminf(amSignal, 2.0f), 0.0f);

        // Set carrier frequency
        complexOsc.SetFreq(complexOsc_freq);
        complexOscTri.SetFreq(complexOsc_freq);

        float complexOsc_sineSignal = complexOsc.Process();
        float complexOsc_triSignal = complexOscTri.Process();
        float complexOsc_rawSignal = (complexOsc_sineSignal * (1.0f - complexOsc_timbreAmount)) + (complexOsc_triSignal * complexOsc_timbreAmount);
        float complexOsc_filteredSignal = complexOsc_analogFilter.Process(complexOsc_rawSignal);
        float complexOsc_foldedSignal = wavefolder(complexOsc_filteredSignal, currentFoldAmount);

        // Apply AM
        float modulated_complexOsc = complexOsc_foldedSignal * ch1_level * amSignal;

        // APPLY LPG FILTERS BASED ON CURRENT MODE with envelope modulation depth
        processLPG(lpgChannel1_filter, lpgChannel1_mode, modulated_complexOsc, complexOsc_level, ch1_cutoffControl, envValue, envModDepth_ch1);
        processLPG(lpgChannel2_filter, lpgChannel2_mode, modulated_modOsc, modOsc_level, ch2_cutoffControl, envValue, envModDepth_ch2);

        // OUTPUT SUMMATION
        float oscillatorSum_signal = modulated_complexOsc + modulated_modOsc;
        out[0][i] = oscillatorSum_signal;
        out[1][i] = oscillatorSum_signal;
      }

    } else {
      // FREQUENCY MODULATION (FM) - default (unchanged)
      // Full modulator level for FM mode
      float modulated_modOsc = modOsc_filteredSignal * ch2_level;

      float modulationDepth = modOsc_modAmount * 1.0f;
      float modulatorSignal = modOsc_filteredSignal * modulationDepth;
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

      // APPLY LPG FILTERS BASED ON CURRENT MODE with envelope modulation depth
      processLPG(lpgChannel1_filter, lpgChannel1_mode, modulated_complexOsc, complexOsc_level, ch1_cutoffControl, envValue, envModDepth_ch1);
      processLPG(lpgChannel2_filter, lpgChannel2_mode, modulated_modOsc, modOsc_level, ch2_cutoffControl, envValue, envModDepth_ch2);

      float oscillatorSum_signal = modulated_complexOsc + modulated_modOsc;
      out[0][i] = oscillatorSum_signal;
      out[1][i] = oscillatorSum_signal;
    }
  }
}

void setup() {
  Serial.begin(115200);  // Increased baud rate for faster debugging

  // INIT BUTTON MATRIX
  initButtonMatrix();

  // BUTTON INIT - Fixed the modulationToggle pin assignment
  sequencerToggle.Init(1000, true, SEQUENCER_TOGGLE_BUTTON, INPUT_PULLUP);
  modulationToggle.Init(1000, true, MODULATION_TOGGLE_BUTTON, INPUT_PULLUP);
  lpgToggle_channel1.Init(1000, true, LPG_CH1_TOGGLE_BUTTON, INPUT_PULLUP);
  lpgToggle_channel2.Init(1000, true, LPG_CH2_TOGGLE_BUTTON, INPUT_PULLUP);

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

  // INIT OSCILLATORS
  complexOsc.Init(sample_rate);
  complexOscTri.Init(sample_rate);
  modOsc.Init(sample_rate);

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

  complexOsc.SetWaveform(complexOsc.WAVE_SIN);
  complexOsc.SetAmp(1.0);

  complexOscTri.SetWaveform(complexOscTri.WAVE_SQUARE);
  complexOscTri.SetAmp(1.0);

  modOsc.SetWaveform(modOsc.WAVE_TRI);
  modOsc.SetAmp(0.5);

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

  // ENVELOPE MODULATION DEPTH INIT
  envModDepth_ch1 = 1.0f;
  envModDepth_ch2 = 1.0f;

  // WAVEFOLDER ENVELOPE MODULATION INIT
  wavefolderEnvModDepth = 1.0f;  // Default modulation depth
  wavefolderEnvModEnabled = false;

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

  Serial.println("Weasel Initialised with MIDI functionality and second MUX");
  Serial.println("Use button 19 to toggle between internal clock and MIDI note triggers");
  Serial.println("Use button 20 to toggle between FM and AM modulation");
  Serial.println("Current modulation: FM");
  Serial.println("LPG Channel 1: COMBI mode | LPG Channel 2: COMBI mode");
  Serial.println("MUX2 C0: Env modulation depth for Ch1 | MUX2 C1: Env modulation depth for Ch2");
  Serial.println("In LP mode: MUX1 C5/C6 control filter cutoff, oscillator level is static");
  Serial.println("Button Matrix initialized - monitoring Row0 and Row1");
  Serial.println("Connect Y0 to X0 to enable envelope modulation of wavefolder amount");
}

void loop() {

  // MIDI PROCESS
  MIDI.read();

  // CHECK MIDI TIMEOUT
  if (midiNoteActive && (millis() - lastMidiNoteTime > MIDI_NOTE_TIMEOUT)) {
    midiNoteActive = false;
  }

  // READ BUTTON MATRIX
  if (millis() - lastButtonRead > BUTTON_READ_INTERVAL) {
    readButtonMatrix();
    printButtonStates();
    lastButtonRead = millis();
  }

  // POTENTIOMETER HANDLING - ALL FROM FIRST MUX
  modOsc_pitch = readMux1Channel(MOD_OSC_PITCH_CHANNEL, 16.35f, 2500.0f, true);
  modOsc_modAmount = readMux1Channel(MOD_AMOUNT_CHANNEL, 0.0f, 800.0f);
  complexOsc_basePitch = readMux1Channel(COMPLEX_OSC_PITCH_CHANNEL, 55.0f, 1760.0f, true);
  complexOsc_timbreAmount = readMux1Channel(COMPLEX_OSC_TIMBRE_CHANNEL, 0.0f, 1.0f);
  complexOsc_foldAmount = readMux1Channel(COMPLEX_OSC_FOLD_CHANNEL, 0.0f, 1.0f);

  // Always read the level pots, but they'll be used differently based on LPG mode
  complexOsc_level = readMux1Channel(COMPLEX_OSC_LEVEL_CHANNEL, 0.0f, 1.0f);
  modOsc_level = readMux1Channel(MOD_OSC_LEVEL_CHANNEL, 0.0f, 1.0f);

  // In LP mode, use the level pots as cutoff controls
  if (lpgChannel1_mode == LPG_MODE_LP) {
    lpgCh1_baseCutoff = complexOsc_level;
  }
  if (lpgChannel2_mode == LPG_MODE_LP) {
    lpgCh2_baseCutoff = modOsc_level;
  }

  eg_attackTime = readMux1Channel(ASD_ATTACK_CHANNEL, 0.02f, 10.0f, true);
  eg_decayTime = readMux1Channel(ASD_SUSTAIN_CHANNEL, 0.02f, 10.0f, true);
  eg_sustainLevel = 1.0f;
  eg_releaseTime = readMux1Channel(ASD_DECAY_CHANNEL, 0.02f, 10.0f, true);

  BPM = readMux1Channel(CLOCK_CHANNEL, 1.0f, 1000.0f);

  // READ ENVELOPE MODULATION DEPTH CONTROLS FROM SECOND MUX
  envModDepth_ch1 = readMux2Channel(ENV_MOD_DEPTH_CH1, 0.0f, 1.0f);  // Channel 1 modulation depth
  envModDepth_ch2 = readMux2Channel(ENV_MOD_DEPTH_CH2, 0.0f, 1.0f);  // Channel 2 modulation depth

  // Use one of the unused MUX2 channels to control wavefolder envelope modulation depth
  wavefolderEnvModDepth = readMux2Channel(MUX2_CHANNEL_2, 0.0f, 1.0f);

  // READ REMAINING SECOND MUX CHANNELS (for future use)
  float mux2_ch3 = readMux2Channel(MUX2_CHANNEL_3, 0.0f, 1.0f);
  float mux2_ch4 = readMux2Channel(MUX2_CHANNEL_4, 0.0f, 1.0f);
  float mux2_ch5 = readMux2Channel(MUX2_CHANNEL_5, 0.0f, 1.0f);
  float mux2_ch6 = readMux2Channel(MUX2_CHANNEL_6, 0.0f, 1.0f);
  float mux2_ch7 = readMux2Channel(MUX2_CHANNEL_7, 0.0f, 1.0f);
  float mux2_ch8 = readMux2Channel(MUX2_CHANNEL_8, 0.0f, 1.0f);
  float mux2_ch9 = readMux2Channel(MUX2_CHANNEL_9, 0.0f, 1.0f);
  float mux2_ch10 = readMux2Channel(MUX2_CHANNEL_10, 0.0f, 1.0f);
  float mux2_ch11 = readMux2Channel(MUX2_CHANNEL_11, 0.0f, 1.0f);
  float mux2_ch12 = readMux2Channel(MUX2_CHANNEL_12, 0.0f, 1.0f);
  float mux2_ch13 = readMux2Channel(MUX2_CHANNEL_13, 0.0f, 1.0f);
  float mux2_ch14 = readMux2Channel(MUX2_CHANNEL_14, 0.0f, 1.0f);
  float mux2_ch15 = readMux2Channel(MUX2_CHANNEL_15, 0.0f, 1.0f);

  // BUTTON HANDLING
  sequencerToggle.Debounce();
  modulationToggle.Debounce();
  lpgToggle_channel1.Debounce();
  lpgToggle_channel2.Debounce();

  // TOGGLE MIDI SEQUENCER TRIGGER
  if (sequencerToggle.RisingEdge()) {
    useMidiClock = !useMidiClock;
    Serial.print("Sequencer mode: ");
    Serial.println(useMidiClock ? "MIDI Note Triggers" : "Internal Clock");
  }

  // TOGGLE MODULATION TYPE
  if (modulationToggle.RisingEdge()) {
    useAmplitudeModulation = !useAmplitudeModulation;
    Serial.print("Modulation type: ");
    Serial.println(useAmplitudeModulation ? "AM (Amplitude Modulation)" : "FM (Frequency Modulation)");
  }

  // TOGGLE LPG CHANNEL 1 MODE
  if (lpgToggle_channel1.RisingEdge()) {
    lpgChannel1_mode = static_cast<LPGMode>((lpgChannel1_mode + 1) % 3);
    Serial.print("LPG Channel 1 mode: ");
    switch (lpgChannel1_mode) {
      case LPG_MODE_COMBI: Serial.println("COMBI (env controls cutoff+amp)"); break;
      case LPG_MODE_VCA: Serial.println("VCA (env controls amplitude)"); break;
      case LPG_MODE_LP: Serial.println("LP (env controls cutoff)"); break;
    }
  }

  // TOGGLE LPG CHANNEL 2 MODE
  if (lpgToggle_channel2.RisingEdge()) {
    lpgChannel2_mode = static_cast<LPGMode>((lpgChannel2_mode + 1) % 3);
    Serial.print("LPG Channel 2 mode: ");
    switch (lpgChannel2_mode) {
      case LPG_MODE_COMBI: Serial.println("COMBI (env controls cutoff+amp)"); break;
      case LPG_MODE_VCA: Serial.println("VCA (env controls amplitude)"); break;
      case LPG_MODE_LP: Serial.println("LP (env controls cutoff)"); break;
    }
  }

  // ADR PARAMETERS
  env.SetTime(ADSR_SEG_ATTACK, eg_attackTime);
  env.SetTime(ADSR_SEG_DECAY, eg_decayTime);
  env.SetTime(ADSR_SEG_RELEASE, eg_releaseTime);
  env.SetSustainLevel(eg_sustainLevel);

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

  // DEBUG OUTPUT
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 500) {
    Serial.print("Step: ");
    Serial.print(currentStep + 1);
    Serial.print("/5 | Seq Mode: ");
    Serial.print(useMidiClock ? "MIDI" : "INT");
    Serial.print(" | Mod Type: ");
    Serial.print(useAmplitudeModulation ? "AM" : "FM");
    Serial.print(" | BPM: ");
    Serial.print(BPM);
    Serial.print(" | MIDI CV: ");
    Serial.print(midiPitchCV);
    Serial.print(" | EnvMod Ch1: ");
    Serial.print(envModDepth_ch1);
    Serial.print(" | EnvMod Ch2: ");
    Serial.print(envModDepth_ch2);
    
    // Show wavefolder envelope modulation status
    Serial.print(" | Wavefolder Env Mod: ");
    Serial.print(wavefolderEnvModEnabled ? "ON" : "OFF");
    if (wavefolderEnvModEnabled) {
      Serial.print(" (Depth: ");
      Serial.print(wavefolderEnvModDepth);
      Serial.print(")");
    }

    // Show LP mode specific info
    if (lpgChannel1_mode == LPG_MODE_LP) {
      Serial.print(" | Ch1 Cutoff: ");
      Serial.print(lpgCh1_baseCutoff);
    }
    if (lpgChannel2_mode == LPG_MODE_LP) {
      Serial.print(" | Ch2 Cutoff: ");
      Serial.print(lpgCh2_baseCutoff);
    }

    // Show MIDI status
    if (midiNoteReceived) {
      Serial.print(" | Note: ");
      Serial.print(lastMidiNote);
      midiNoteReceived = false;
    }

    Serial.println();

    lastPrint = millis();
  }
}