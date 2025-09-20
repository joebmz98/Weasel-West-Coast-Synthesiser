#include "DaisyDuino.h"
//#include "buchla_lpg.h"

// MUX PINS
#define MUX_S0 0
#define MUX_S1 1
#define MUX_S2 2
#define MUX_S3 3
#define MUX_SIG A0  // MUX signal pin

// MUX CHANNEL ASSIGNMENTS
#define MOD_OSC_PITCH_CHANNEL 0    // C0 - modOsc_pitch
#define MOD_AMOUNT_CHANNEL 1       // C1 - modOsc_modAmount  
#define COMPLEX_OSC_PITCH_CHANNEL 2 // C2 - complexOsc_pitch
#define COMPLEX_OSC_TIMBRE_CHANNEL 3 // C3 - timbre control (sine/triangle blend)
#define COMPLEX_OSC_FOLD_CHANNEL 4  // C4 - wavefolding amount
#define COMPLEX_OSC_LEVEL_CHANNEL 5 // C5 - complex oscillator output level
#define MOD_OSC_LEVEL_CHANNEL 6    // C6 - modulator oscillator output level
#define SEQ_STEP_1_CHANNEL 7       // C7 - Sequencer step 1
#define SEQ_STEP_2_CHANNEL 8       // C8 - Sequencer step 2
#define SEQ_STEP_3_CHANNEL 9       // C9 - Sequencer step 3
#define SEQ_STEP_4_CHANNEL 10      // C10 - Sequencer step 4
#define SEQ_STEP_5_CHANNEL 11      // C11 - Sequencer step 5
#define ASD_ATTACK_CHANNEL 12      // C12 - ATTACK
#define ASD_SUSTAIN_CHANNEL 13     // C13 - SUSTAIN
#define ASD_DECAY_CHANNEL 14       // C13 - DECAY
#define CLOCK_CHANNEL 15           // C15 - CLOCK

// DEFINE DAISYSEED
DaisyHardware hw;

//INIT OSCILLATORS, FILTER, AND WAVEFOLDER
static Oscillator complexOsc;      // PRIMARY COMPLEX OSC
static Oscillator complexOscTri;   // SECONDARY COMPLEX OSC (triangle)
static Oscillator modOsc;          // MODULATION OSC
static MoogLadder complexOsc_filter; // FILTER - HIGH CUT AT 15kHz FOR "ANALOGUE" FEEL OF WAVEFORMS

// OSCILLATOR PARAMETER VARIABLES
float complexOsc_basePitch;        // COMPLEX OSC BASE PITCH
float modOsc_pitch;                // MOD OSC BASE PITCH
float modOsc_modAmount;            // MOD AMOUNT AFFECTING COMPLEX OSC
float complexOsc_timbreAmount;     // COMPLEX OSC TIMBRE WAVEMORPHING AMOUNT (0.0 = sine, 1.0 = triangle)
float complexOsc_foldAmount;       // COMPLEX OSC WAVEFOLDING AMOUNT (0.0 = no fold, 1.0 = max fold)
float complexOsc_level;            // COMPLEX OSC LEVEL CONTROL (0.0 to 1.0)
float modOsc_level;                // MODULATION OSC LEVEL CONTROLl (0.0 to 1.0)

// SEQUENCER VARIABLES
const int SEQ_STEPS = 5;           // NUM OF STEPS
float sequencerValues[SEQ_STEPS];  // STORE POT VALUES
int currentStep = 0;               // CURRENT STEP
unsigned long stepStartTime = 0;   // TIME WHEN CURRENT STEP STARTED
const float BPM = 100.0f;          // CLOCK SPEED
const float STEP_DURATION_MS = (60000.0f / BPM) / 4.0f; // QUARTER NOTE DURATION AT 100BPM

// Function to convert linear values to logarithmic scale for pitch
float linearToLog(float value, float minVal, float maxVal) {
  // Convert linear 0-1 value to exponential frequency
  // Using the formula: freq = minVal * pow(maxVal/minVal, value)
  return minVal * pow(maxVal / minVal, value);
}

// Function to convert semitones to frequency ratio
float semitonesToRatio(float semitones) {
  return pow(2.0f, semitones / 12.0f);
}

// WAVEFOLDER FNC.
float wavefolder(float input, float amount) {
  // More intense wavefolding algorithm
  // amount = 0.0: no folding, amount = 1.0: maximum aggressive folding
  if (amount < 0.001f) return input; // Bypass when amount is very small
  
  // Increased gain before folding for more intense effect
  // Scale input based on fold amount - much more aggressive now
  float scaledInput = input * (1.0f + amount * 80.0f);  // 
  
  // Multiple folding stages for more complex harmonics
  for (int fold = 0; fold < 3; fold++) {  // Multiple folding passes
    while (fabs(scaledInput) > 1.0f) {
      if (scaledInput > 1.0f) {
        scaledInput = 2.0f - scaledInput;
      } else if (scaledInput < -1.0f) {
        scaledInput = -2.0f - scaledInput;
      }
    }
    
    // Add some asymmetry for more interesting harmonics
    if (amount > 0.5f) {
      scaledInput *= (0.95f + 0.1f * amount);  // Slight asymmetry
    }
  }
  
  // More aggressive blending - full wet at higher amounts
  float wetDryMix;
  if (amount < 0.3f) {
    wetDryMix = amount / 0.3f;  // Smooth transition at low amounts
  } else {
    wetDryMix = 1.0f;  // Full wet for higher amounts
  }
  
  return (input * (1.0f - wetDryMix)) + ((scaledInput * wetDryMix) *0.25 ); // MANUAL LEVEL SCALING
}

// MUX
void setMuxChannel(int channel) {
  // Set the MUX channel by controlling S0-S3 pins
  digitalWrite(MUX_S0, channel & 0x01);
  digitalWrite(MUX_S1, channel & 0x02);
  digitalWrite(MUX_S2, channel & 0x04);
  digitalWrite(MUX_S3, channel & 0x08);
}

float readMuxChannel(int channel, float minVal, float maxVal, bool logarithmic = false) {
  // Set MUX to desired channel
  setMuxChannel(channel);
  
  // Small delay for MUX to settle
  delayMicroseconds(10);
  
  // Read analog value and map to 0-1 range
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
  
  // Check if it's time to advance to the next step
  if (currentTime - stepStartTime >= STEP_DURATION_MS) {
    currentStep = (currentStep + 1) % SEQ_STEPS;
    stepStartTime = currentTime;
  }
}

// Read all sequencer potentiometers
void readSequencerValues() {
  sequencerValues[0] = readMuxChannel(SEQ_STEP_1_CHANNEL, 0.0f, 48.0f); // 0 to +48 semitones
  sequencerValues[1] = readMuxChannel(SEQ_STEP_2_CHANNEL, 0.0f, 48.0f);
  sequencerValues[2] = readMuxChannel(SEQ_STEP_3_CHANNEL, 0.0f, 48.0f);
  sequencerValues[3] = readMuxChannel(SEQ_STEP_4_CHANNEL, 0.0f, 48.0f);
  sequencerValues[4] = readMuxChannel(SEQ_STEP_5_CHANNEL, 0.0f, 48.0f);
}

void AudioCallback(float **in, float **out, size_t size) {
  for (size_t i = 0; i < size; i++) {
    // Get current sequencer pitch offset
    float sequencerPitchOffset = sequencerValues[currentStep];
    float pitchRatio = semitonesToRatio(sequencerPitchOffset);
    
    // PROCESS MODULATOR OSCILLATOR with sequencer pitch modulation
    float modulatedModPitch = modOsc_pitch * pitchRatio;
    modOsc.SetFreq(modulatedModPitch);
    float modOsc_signal = modOsc.Process();
    
    // PROCESS COMPLEX OSCILLATOR WITH FM and sequencer pitch modulation
    float modulatedComplexBasePitch = complexOsc_basePitch * pitchRatio;
    float complexOsc_modulatedFreq = modulatedComplexBasePitch + (modOsc_signal * modOsc_modAmount) - 16.0;
    complexOsc_modulatedFreq = max(complexOsc_modulatedFreq, 17.0f);  // MIN 17Hz
    
    // Set frequency for both complex oscillators
    complexOsc.SetFreq(complexOsc_modulatedFreq);
    complexOscTri.SetFreq(complexOsc_modulatedFreq);
    
    // Process both waveforms
    float complexOsc_sineSignal = complexOsc.Process();
    float complexOsc_triSignal = complexOscTri.Process();
    
    // BLEND BETWEEN SINE AND TRIANGLE USING TIMBRE CONTROL
    float complexOsc_rawSignal = (complexOsc_sineSignal * (1.0f - complexOsc_timbreAmount)) + (complexOsc_triSignal * complexOsc_timbreAmount);

    // APPLY MOOG LOWPASS FILTER
    float complexOsc_filteredSignal = complexOsc_filter.Process(complexOsc_rawSignal);

    // APPLY WAVEFOLDING (more aggressive now)
    float complexOsc_foldedSignal = wavefolder(complexOsc_filteredSignal, complexOsc_foldAmount);

    // APPLY OUTPUT LEVEL CONTROL TO BOTH OSCILLATORS
    float modulated_complexOsc = complexOsc_foldedSignal * complexOsc_level;
    float modulated_modOsc = modOsc_signal * modOsc_level;

    // VCA + ENVELOPE

    // OSC STAGE OUTPUT
    float oscillatorSum_signal = modulated_complexOsc + modulated_modOsc;

    // OUTPUT
    out[0][i] = oscillatorSum_signal;
    out[1][i] = oscillatorSum_signal; 
  }
}

void setup() {
  Serial.begin(9600);
  
  // INIT MUX_1 PINS
  pinMode(MUX_S0, OUTPUT);
  pinMode(MUX_S1, OUTPUT);
  pinMode(MUX_S2, OUTPUT);
  pinMode(MUX_S3, OUTPUT);
  
  // SET ALL MUX_1 PINS LOW
  digitalWrite(MUX_S0, LOW);
  digitalWrite(MUX_S1, LOW);
  digitalWrite(MUX_S2, LOW);
  digitalWrite(MUX_S3, LOW);
  
  // INIT 16-BIT ADC
  analogReadResolution(16);
  
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
  complexOsc_filter.SetFreq(15000.0f);  // 15kHz cutoff
  complexOsc_filter.SetRes(0.3f);       // Small amount of resonance
  
  // INIT PRIMARY CARRIER OSC (sine wave)
  complexOsc.SetWaveform(complexOsc.WAVE_SIN);
  complexOsc.SetAmp(1.0);
  
  // INIT SECONDARY CARRIER OSC (triangle wave)
  complexOscTri.SetWaveform(complexOscTri.WAVE_TRI);
  complexOscTri.SetAmp(1.0);
  
  // INIT MODULATOR OSC
  modOsc.SetWaveform(modOsc.WAVE_TRI);
  modOsc.SetAmp(0.5);
  
  // Set initial values
  complexOsc_basePitch = 440.0f;
  modOsc_pitch = 1.0f;
  modOsc_modAmount = 0.0f;
  complexOsc_timbreAmount = 0.0f;  // Start with pure sine wave
  complexOsc_foldAmount = 0.0f;    // Start with no wavefolding
  complexOsc_level = 1.0f;         // Start with full volume for complex oscillator
  modOsc_level = 1.0f;             // Start with full volume for modulator oscillator
  
  // Initialize sequencer values
  for (int i = 0; i < SEQ_STEPS; i++) {
    sequencerValues[i] = 0.0f;
  }
  
  stepStartTime = millis();
  
  // Start audio callback
  DAISY.begin(AudioCallback);
  
  Serial.println("Weasel Initialised with AGGRESSIVE Wavefolder, Moog Filter, and Buchla-style Timbre Control");
  Serial.println("Now with logarithmic pitch scaling for more natural response");
  Serial.println("And individual output level controls for both oscillators");
  Serial.println("And 5-step sequencer with 0 to +48 semitone range");
}

void loop() {
  // POTENTIOMETER READ with logarithmic scaling for pitch channels
  modOsc_pitch = readMuxChannel(MOD_OSC_PITCH_CHANNEL, 16.35f, 2500.0f, true); // Logarithmic
  modOsc_modAmount = readMuxChannel(MOD_AMOUNT_CHANNEL, 0.0f, 1000.0f); // Linear
  complexOsc_basePitch = readMuxChannel(COMPLEX_OSC_PITCH_CHANNEL, 55.0f, 1760.0f, true); // Logarithmic
  complexOsc_timbreAmount = readMuxChannel(COMPLEX_OSC_TIMBRE_CHANNEL, 0.0f, 1.0f);  // Linear
  complexOsc_foldAmount = readMuxChannel(COMPLEX_OSC_FOLD_CHANNEL, 0.0f, 1.0f);      // Linear
  complexOsc_level = readMuxChannel(COMPLEX_OSC_LEVEL_CHANNEL, 0.0f, 1.0f);          // Linear - Complex OSC level
  modOsc_level = readMuxChannel(MOD_OSC_LEVEL_CHANNEL, 0.0f, 1.0f);                  // Linear - Mod OSC level
  
  // Read sequencer values
  readSequencerValues();
  
  // Update sequencer
  updateSequencer();
  
  // SERIAL DEBUG
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 200) { // Print every 200ms to avoid flooding
    Serial.print("Step: ");
    Serial.print(currentStep + 1);
    Serial.print("/5 | Step Pitch: +");
    Serial.print(sequencerValues[currentStep]);
    Serial.print("st | Mod Freq: ");
    Serial.print(modOsc_pitch);
    Serial.print("Hz | Mod Amount: ");
    Serial.print(modOsc_modAmount/10); // SCALE TO PERCENTAGE
    Serial.print("% | Complex Pitch: ");
    Serial.print(complexOsc_basePitch);
    Serial.print("Hz | Timbre: ");
    Serial.print(complexOsc_timbreAmount * 100); // SCALE TO PERCENTAGE
    Serial.print("% | Fold: ");
    Serial.print(complexOsc_foldAmount * 100); // SCALE TO PERCENTAGE
    Serial.print("% | Complex Level: ");
    Serial.print(complexOsc_level * 100); // SCALE TO PERCENTAGE
    Serial.print("% | Mod Level: ");
    Serial.print(modOsc_level * 100); // SCALE TO PERCENTAGE
    Serial.println("%"); 
    lastPrint = millis();
  }
  
  delay(10); // Small delay to prevent reading too fast
}