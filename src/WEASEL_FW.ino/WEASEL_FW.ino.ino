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

#define SEQUENCER_TOGGLE_BUTTON 19 // SEQ TOGGLE

// MIDI Settings
#define MIDI_RX_PIN 30  // USART1 Rx

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

// BUTTONS
Switch sequencerToggle;  // SEQUENCER CLOCK TOGGLE

// MIDI Object
MIDI_CREATE_INSTANCE(HardwareSerial, Serial1, MIDI);

// MIDI Debug variables
unsigned long lastMidiDebugTime = 0;
bool midiNoteReceived = false;
byte lastMidiNote = 0;
byte lastMidiVelocity = 0;
byte lastMidiChannel = 0;

// Function to convert linear values to logarithmic scale for pitch
float linearToLog(float value, float minVal, float maxVal) {
  return minVal * pow(maxVal / minVal, value);
}

// Function to convert semitones to frequency ratio
float semitonesToRatio(float semitones) {
  return pow(2.0f, semitones / 12.0f);
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
  
  if (useMidiClock) {
    advanceStep();
  }
  
  Serial.print("MIDI Note On - Channel: ");
  Serial.print(channel);
  Serial.print(" Note: ");
  Serial.print(note);
  Serial.print(" Velocity: ");
  Serial.println(velocity);
}

// MIDI Note Off handler
void handleNoteOff(byte channel, byte note, byte velocity) {
  Serial.print("MIDI Note Off - Channel: ");
  Serial.print(channel);
  Serial.print(" Note: ");
  Serial.print(note);
  Serial.print(" Velocity: ");
  Serial.println(velocity);
}

void AudioCallback(float **in, float **out, size_t size) {
  for (size_t i = 0; i < size; i++) {
    float sequencerPitchOffset = sequencerValues[currentStep];
    float pitchRatio = semitonesToRatio(sequencerPitchOffset);

    float modulatedModPitch = modOsc_pitch * pitchRatio;
    modOsc.SetFreq(modulatedModPitch);
    float modOsc_signal = modOsc.Process();

    float modulatedComplexBasePitch = complexOsc_basePitch * pitchRatio;
    float complexOsc_modulatedFreq = modulatedComplexBasePitch + (modOsc_signal * modOsc_modAmount) - 16.0;
    complexOsc_modulatedFreq = max(complexOsc_modulatedFreq, 17.0f);

    complexOsc.SetFreq(complexOsc_modulatedFreq);
    complexOscTri.SetFreq(complexOsc_modulatedFreq);

    float complexOsc_sineSignal = complexOsc.Process();
    float complexOsc_triSignal = complexOscTri.Process();

    float complexOsc_rawSignal = (complexOsc_sineSignal * (1.0f - complexOsc_timbreAmount)) + (complexOsc_triSignal * complexOsc_timbreAmount);

    float complexOsc_filteredSignal = complexOsc_filter.Process(complexOsc_rawSignal);

    float complexOsc_foldedSignal = wavefolder(complexOsc_filteredSignal, complexOsc_foldAmount);

    float modulated_complexOsc = complexOsc_foldedSignal * complexOsc_level;
    float modulated_modOsc = modOsc_signal * modOsc_level;

    float envValue = env.Process(gateOpen);
    modulated_complexOsc *= envValue;
    modulated_modOsc *= envValue;

    float oscillatorSum_signal = modulated_complexOsc + modulated_modOsc;

    out[0][i] = oscillatorSum_signal;
    out[1][i] = oscillatorSum_signal;
  }
}

void setup() {
  Serial.begin(9600);

  // BUTTON INIT
  sequencerToggle.Init(1000, true, SEQUENCER_TOGGLE_BUTTON, INPUT_PULLUP);

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

  // INIT MIDI - Configure Serial1 to use pin 37 for RX
  Serial1.setRx(MIDI_RX_PIN);
  Serial1.begin(31250);
  MIDI.begin(MIDI_CHANNEL_OMNI);
  MIDI.setHandleNoteOn(handleNoteOn);
  MIDI.setHandleNoteOff(handleNoteOff);

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

  // Set initial values
  complexOsc_basePitch = 440.0f;
  modOsc_pitch = 1.0f;
  modOsc_modAmount = 0.0f;
  complexOsc_timbreAmount = 0.0f;
  complexOsc_foldAmount = 0.0f;
  complexOsc_level = 1.0f;
  modOsc_level = 1.0f;

  // Initialize ADSR values
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
  Serial.println("Use button to toggle between internal clock and MIDI note triggers");
  Serial.println("MIDI Debug enabled - will show incoming MIDI messages");
}

void loop() {
  // Read potentiometers
  modOsc_pitch = readMuxChannel(MOD_OSC_PITCH_CHANNEL, 16.35f, 2500.0f, true);
  modOsc_modAmount = readMuxChannel(MOD_AMOUNT_CHANNEL, 0.0f, 1000.0f);
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

  // Button handling
  sequencerToggle.Debounce();
  
  // Toggle MIDI clock mode when button is pressed
  if (sequencerToggle.RisingEdge()) {
    useMidiClock = !useMidiClock;
    Serial.print("Sequencer mode: ");
    Serial.println(useMidiClock ? "MIDI Note Triggers" : "Internal Clock");
  }

  // Update ADSR parameters
  env.SetTime(ADSR_SEG_ATTACK, eg_attackTime);
  env.SetTime(ADSR_SEG_DECAY, eg_decayTime);
  env.SetTime(ADSR_SEG_RELEASE, eg_releaseTime);
  env.SetSustainLevel(eg_sustainLevel);

  readSequencerValues();
  updateSequencer();
  MIDI.read();

  // Handle envelope release
  float gateDurationMs = eg_decayTime * 1000.0f;
  if (gateDurationMs > STEP_DURATION_MS) {
    gateDurationMs = STEP_DURATION_MS;
  }

  if (gateOpen && (millis() - stepStartTime) > gateDurationMs) {
    gateOpen = false;
  }

  // Debug output
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 200) {
    Serial.print("Step: ");
    Serial.print(currentStep + 1);
    Serial.print("/5 | Mode: ");
    Serial.print(useMidiClock ? "MIDI" : "INT");
    Serial.print(" | BPM: ");
    Serial.print(BPM);
    
    // Show MIDI status if no notes received recently
    if (millis() - lastMidiDebugTime > 1000) {
      if (!midiNoteReceived) {
        Serial.print(" | No MIDI received");
      } else {
        Serial.print(" | Last MIDI: Ch");
        Serial.print(lastMidiChannel);
        Serial.print(" Note:");
        Serial.print(lastMidiNote);
        Serial.print(" Vel:");
        Serial.print(lastMidiVelocity);
        midiNoteReceived = false;
      }
      lastMidiDebugTime = millis();
    }
    
    Serial.println();
    
    lastPrint = millis();
  }

  delay(10);
}