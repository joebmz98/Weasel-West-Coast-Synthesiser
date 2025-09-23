#include "DaisyDuino.h"
#include <MIDI.h>
//#include "buchla_lpg.h"

// MUX PINS
#define MUX_S0 0
#define MUX_S1 1
#define MUX_S2 2
#define MUX_S3 3
#define MUX_SIG A0  // MUX signal pin

// MUX CHANNEL ASSIGNMENTS
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

#define SEQUENCER_TOGGLE_BUTTON 19    // SEQ TOGGLE
#define MODULATION_TOGGLE_BUTTON 20   // MODULATION TOGGLE

// MIDI Settings
#define MIDI_RX_PIN 30  // USART1 Rx (Digital pin 30)

// DEFINE DAISYSEED
DaisyHardware hw;

//INIT OSCILLATORS, FILTER, AND WAVEFOLDER
static Oscillator complexOsc;         // PRIMARY COMPLEX OSC
static Oscillator complexOscTri;      // SECONDARY COMPLEX OSC (triangle)
static Oscillator modOsc;             // MODULATION OSC
static MoogLadder complexOsc_filter;  // FILTER - HIGH CUT AT 15kHz FOR "ANALOGUE" FEEL OF WAVEFORMS

// OSCILLATOR PARAMETER VARIABLES
float complexOsc_basePitch;     // COMPLEX OSC BASE PITCH
float modOsc_pitch;             // MOD OSC BASE PITCH
float modOsc_modAmount;         // MOD AMOUNT AFFECTING COMPLEX OSC
float complexOsc_timbreAmount;  // COMPLEX OSC TIMBRE WAVEMORPHING AMOUNT (0.0 = sine, 1.0 = triangle)
float complexOsc_foldAmount;    // COMPLEX OSC WAVEFOLDING AMOUNT (0.0 = no fold, 1.0 = max fold)
float complexOsc_level;         // COMPLEX OSC LEVEL CONTROL (0.0 to 1.0)
float modOsc_level;             // MODULATION OSC LEVEL CONTROLl (0.0 to 1.0)

// ADSR ENVELOPE VARIABLES
float eg_attackTime;    // ATTACK time in seconds
float eg_decayTime;     // DECAY time in seconds (will control gate duration)
float eg_sustainLevel;  // SUSTAIN level (0.0 to 1.0)
float eg_releaseTime;   // RELEASE time in seconds
Adsr env;               // ADSR envelope

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

// BUTTONS
Switch sequencerToggle;   // SEQUENCER CLOCK TOGGLE
Switch modulationToggle;  // MODULATION TYPE TOGGLE

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
float midiPitchCV = 0.0f;        // MIDI pitch control voltage (in semitones) - PERSISTS after note off
bool midiNoteActive = false;     // Whether a MIDI note is currently active (for triggering)
unsigned long lastMidiNoteTime = 0; // Time when last MIDI note was received
const unsigned long MIDI_NOTE_TIMEOUT = 1000; // 1 second timeout for MIDI notes

// Function to convert linear values to logarithmic scale for pitch
float linearToLog(float value, float minVal, float maxVal) {
  return minVal * pow(maxVal / minVal, value);
}

// Function to convert semitones to frequency ratio
float semitonesToRatio(float semitones) {
  return pow(2.0f, semitones / 12.0f);
}

// Function to convert MIDI note to frequency (1V/octave standard)
float midiNoteToFrequency(byte midiNote, float baseFreq = 261.63f) { // C4 = 261.63Hz
  // Convert MIDI note to frequency (MIDI note 69 = A4 = 440Hz)
  return 440.0f * pow(2.0f, (midiNote - 69) / 12.0f);
}

// Function to convert MIDI note to CV (1V/octave, where 1V = 12 semitones)
float midiNoteToCV(byte midiNote, float baseNote = 60.0f) { // C4 = MIDI note 60
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

// MUX
void setMuxChannel(int channel) {
  digitalWrite(MUX_S0, channel & 0x01);
  digitalWrite(MUX_S1, channel & 0x02);
  digitalWrite(MUX_S2, channel & 0x04);
  digitalWrite(MUX_S3, channel & 0x08);
}

float readMuxChannel(int channel, float minVal, float maxVal, bool logarithmic = false) {
  setMuxChannel(channel);
  delayMicroseconds(10);

  int rawValue = analogRead(MUX_SIG);
  float normalizedValue = rawValue / 65535.0f;

  if (logarithmic) {
    return linearToLog(normalizedValue, minVal, maxVal);
  } else {
    return minVal + (maxVal - minVal) * normalizedValue;
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
  sequencerValues[0] = readMuxChannel(SEQ_STEP_1_CHANNEL, 0.0f, 48.0f);
  sequencerValues[1] = readMuxChannel(SEQ_STEP_2_CHANNEL, 0.0f, 48.0f);
  sequencerValues[2] = readMuxChannel(SEQ_STEP_3_CHANNEL, 0.0f, 48.0f);
  sequencerValues[3] = readMuxChannel(SEQ_STEP_4_CHANNEL, 0.0f, 48.0f);
  sequencerValues[4] = readMuxChannel(SEQ_STEP_5_CHANNEL, 0.0f, 48.0f);
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

void AudioCallback(float **in, float **out, size_t size) {
  for (size_t i = 0; i < size; i++) {
    // Get current sequencer pitch offset
    float sequencerPitchOffset = sequencerValues[currentStep];
    
    // Combine all pitch sources: pots + sequencer + MIDI
    // MIDI pitch CV PERSISTS even after note off for envelope release
    float totalPitchOffset = sequencerPitchOffset + midiPitchCV;
    
    float pitchRatio = semitonesToRatio(totalPitchOffset);

    // PROCESS MODULATOR OSCILLATOR with combined pitch modulation
    float modulatedModPitch = modOsc_pitch * pitchRatio;
    modOsc.SetFreq(modulatedModPitch);
    float modOsc_signal = modOsc.Process();

    // PROCESS COMPLEX OSCILLATOR with combined pitch modulation
    float modulatedComplexBasePitch = complexOsc_basePitch * pitchRatio;
    float complexOsc_freq = modulatedComplexBasePitch;

    // APPLY MODULATION BASED ON SELECTED TYPE
    if (useAmplitudeModulation) {
      // AMPLITUDE MODULATION (AM) - IMPROVED IMPLEMENTATION
      // Scale modulation amount to be more musical (0.0 to 1.0 range)
      float amDepth = modOsc_modAmount * 0.5f; // Increased scaling for better range
      
      // Convert bipolar modulator signal to proper AM signal
      // AM formula: carrier * (1 + depth * modulator)
      // This ensures no level drop and proper modulation
      float amSignal = 1.0f + (amDepth * modOsc_signal);
      
      // Clamp to prevent negative amplitudes (which cause distortion)
      amSignal = max(amSignal, 0.0f);
      
      // Set carrier frequency (no FM in AM mode)
      complexOsc.SetFreq(complexOsc_freq);
      complexOscTri.SetFreq(complexOsc_freq);
      
      // Process both waveforms
      float complexOsc_sineSignal = complexOsc.Process();
      float complexOsc_triSignal = complexOscTri.Process();

      // BLEND BETWEEN SINE AND TRIANGLE USING TIMBRE CONTROL
      float complexOsc_rawSignal = (complexOsc_sineSignal * (1.0f - complexOsc_timbreAmount)) + (complexOsc_triSignal * complexOsc_timbreAmount);

      // APPLY MOOG LOWPASS FILTER
      float complexOsc_filteredSignal = complexOsc_filter.Process(complexOsc_rawSignal);

      // APPLY WAVEFOLDING
      float complexOsc_foldedSignal = wavefolder(complexOsc_filteredSignal, complexOsc_foldAmount);

      // APPLY AMPLITUDE MODULATION to carrier
      float modulated_complexOsc = complexOsc_foldedSignal * complexOsc_level * amSignal;
      
      // Modulator oscillator output - normal level in AM mode
      float modulated_modOsc = modOsc_signal * modOsc_level;

      // APPLY ADSR ENVELOPE
      float envValue = env.Process(gateOpen);
      modulated_complexOsc *= envValue;
      modulated_modOsc *= envValue;

      // OSC STAGE OUTPUT
      float oscillatorSum_signal = modulated_complexOsc + modulated_modOsc;

      // OUTPUT with gentle limiting to prevent clipping
      out[0][i] = tanh(oscillatorSum_signal * 0.7f); // Soft clipping
      out[1][i] = tanh(oscillatorSum_signal * 0.7f);
      
    } else {
      // FREQUENCY MODULATION (FM) - default
      float modulationDepth = modOsc_modAmount; // Scale FM depth
      float modulatorSignal = modOsc_signal * modulationDepth;
      float complexOsc_modulatedFreq = complexOsc_freq + modulatorSignal - 16.0f;
      complexOsc_modulatedFreq = max(complexOsc_modulatedFreq, 17.0f);

      // Set carrier frequency with FM
      complexOsc.SetFreq(complexOsc_modulatedFreq);
      complexOscTri.SetFreq(complexOsc_modulatedFreq);

      // Process both waveforms
      float complexOsc_sineSignal = complexOsc.Process();
      float complexOsc_triSignal = complexOscTri.Process();

      // BLEND BETWEEN SINE AND TRIANGLE USING TIMBRE CONTROL
      float complexOsc_rawSignal = (complexOsc_sineSignal * (1.0f - complexOsc_timbreAmount)) + (complexOsc_triSignal * complexOsc_timbreAmount);

      // APPLY MOOG LOWPASS FILTER
      float complexOsc_filteredSignal = complexOsc_filter.Process(complexOsc_rawSignal);

      // APPLY WAVEFOLDING
      float complexOsc_foldedSignal = wavefolder(complexOsc_filteredSignal, complexOsc_foldAmount);

      // APPLY OUTPUT LEVEL CONTROL TO BOTH OSCILLATORS
      float modulated_complexOsc = complexOsc_foldedSignal * complexOsc_level;
      float modulated_modOsc = modOsc_signal * modOsc_level;

      // APPLY ADSR ENVELOPE
      float envValue = env.Process(gateOpen);
      modulated_complexOsc *= envValue;
      modulated_modOsc *= envValue;

      // OSC STAGE OUTPUT
      float oscillatorSum_signal = modulated_complexOsc + modulated_modOsc;

      // OUTPUT
      out[0][i] = oscillatorSum_signal;
      out[1][i] = oscillatorSum_signal;
    }
  }
}

void setup() {
  Serial.begin(115200);  // Increased baud rate for faster debugging

  // BUTTON INIT - Fixed the modulationToggle pin assignment
  sequencerToggle.Init(1000, true, SEQUENCER_TOGGLE_BUTTON, INPUT_PULLUP);
  modulationToggle.Init(1000, true, MODULATION_TOGGLE_BUTTON, INPUT_PULLUP);

  // INIT MUX_1 PINS
  pinMode(MUX_S0, OUTPUT);
  pinMode(MUX_S1, OUTPUT);
  pinMode(MUX_S2, OUTPUT);
  pinMode(MUX_S3, OUTPUT);

  digitalWrite(MUX_S0, LOW);
  digitalWrite(MUX_S1, LOW);
  digitalWrite(MUX_S2, LOW);
  digitalWrite(MUX_S3, LOW);

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

  // INIT COMPLEX FILTER
  complexOsc_filter.Init(sample_rate);
  complexOsc_filter.SetFreq(15000.0f);
  complexOsc_filter.SetRes(0.3f);

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

  Serial.println("Weasel Initialised with MIDI functionality");
  Serial.println("Use button 19 to toggle between internal clock and MIDI note triggers");
  Serial.println("Use button 20 to toggle between FM and AM modulation");
  Serial.println("Current modulation: FM");
}

void loop() {

  // MIDI PROCESS
  MIDI.read();

  // CHECK MIDI TIMEOUT
  if (midiNoteActive && (millis() - lastMidiNoteTime > MIDI_NOTE_TIMEOUT)) {
    midiNoteActive = false;
  }

  // POTENTIOMETER HANDLING
  modOsc_pitch = readMuxChannel(MOD_OSC_PITCH_CHANNEL, 16.35f, 2500.0f, true);
  modOsc_modAmount = readMuxChannel(MOD_AMOUNT_CHANNEL, 0.0f, 800.0f);
  complexOsc_basePitch = readMuxChannel(COMPLEX_OSC_PITCH_CHANNEL, 55.0f, 1760.0f, true);
  complexOsc_timbreAmount = readMuxChannel(COMPLEX_OSC_TIMBRE_CHANNEL, 0.0f, 1.0f);
  complexOsc_foldAmount = readMuxChannel(COMPLEX_OSC_FOLD_CHANNEL, 0.0f, 1.0f);
  complexOsc_level = readMuxChannel(COMPLEX_OSC_LEVEL_CHANNEL, 0.0f, 1.0f);
  modOsc_level = readMuxChannel(MOD_OSC_LEVEL_CHANNEL, 0.0f, 1.0f);

  eg_attackTime = readMuxChannel(ASD_ATTACK_CHANNEL, 0.002f, 10.0f, true);
  eg_decayTime = readMuxChannel(ASD_SUSTAIN_CHANNEL, 0.002f, 10.0f, true);
  eg_sustainLevel = 1.0f;
  eg_releaseTime = readMuxChannel(ASD_DECAY_CHANNEL, 0.02f, 10.0f, true);

  BPM = readMuxChannel(CLOCK_CHANNEL, 60.0f, 120.0f);

  // BUTTON HANDLING
  sequencerToggle.Debounce();
  modulationToggle.Debounce();

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
    Serial.print(" st");
    
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