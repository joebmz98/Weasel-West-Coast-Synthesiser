#include "DaisyDuino.h"

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

// DEFINE DAISYSEED
DaisyHardware hw;

//INIT OSCILLATORS, FILTER, AND WAVEFOLDER
static Oscillator complexOsc;      // Primary carrier oscillator (sine)
static Oscillator complexOscTri;   // Secondary oscillator (triangle)
static Oscillator modOsc;          // Modulator oscillator
static MoogLadder complexOsc_filter; // Moog-style lowpass filter

// OSCILLATOR PARAMETER VARIABLES
float complexOsc_basePitch;        // COMPLEX OSC BASE PITCH
float modOsc_pitch;                // MOD OSC BASE PITCH
float modOsc_modAmount;            // MOD AMOUNT AFFECTING COMPLEX OSC
float complexOsc_timbreAmount;     // Timbre blend amount (0.0 = sine, 1.0 = triangle)
float complexOsc_foldAmount;       // Wavefolding amount (0.0 = no fold, 1.0 = max fold)

// More aggressive wavefolder function
float wavefolder(float input, float amount) {
  // More intense wavefolding algorithm
  // amount = 0.0: no folding, amount = 1.0: maximum aggressive folding
  if (amount < 0.001f) return input; // Bypass when amount is very small
  
  // Increased gain before folding for more intense effect
  // Scale input based on fold amount - much more aggressive now
  float scaledInput = input * (1.0f + amount * 100.0f);  // Increased from 3.0 to 8.0
  
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
  
  return (input * (1.0f - wetDryMix)) + (scaledInput * wetDryMix);
}

void setMuxChannel(int channel) {
  // Set the MUX channel by controlling S0-S3 pins
  digitalWrite(MUX_S0, channel & 0x01);
  digitalWrite(MUX_S1, channel & 0x02);
  digitalWrite(MUX_S2, channel & 0x04);
  digitalWrite(MUX_S3, channel & 0x08);
}

float readMuxChannel(int channel, float minVal, float maxVal) {
  // Set MUX to desired channel
  setMuxChannel(channel);
  
  // Small delay for MUX to settle
  delayMicroseconds(10);
  
  // Read analog value and map to desired range
  int rawValue = analogRead(MUX_SIG);
  return minVal + (maxVal - minVal) * (rawValue / 65535.0f);
}

void AudioCallback(float **in, float **out, size_t size) {
  for (size_t i = 0; i < size; i++) {
    // PROCESS MODULATOR OSCILLATOR
    modOsc.SetFreq(modOsc_pitch);
    float modOsc_signal = modOsc.Process();
    
    // PROCESS COMPLEX OSCILLATOR WITH FM
    float complexOsc_modulatedFreq = complexOsc_basePitch + (modOsc_signal * modOsc_modAmount);
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

    // OUTPUT
    out[0][i] = complexOsc_foldedSignal;
    out[1][i] = complexOsc_foldedSignal; 
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
  modOsc.SetWaveform(modOsc.WAVE_SIN);
  modOsc.SetAmp(1.0);
  
  // Set initial values
  complexOsc_basePitch = 440.0f;
  modOsc_pitch = 1.0f;
  modOsc_modAmount = 0.0f;
  complexOsc_timbreAmount = 0.0f;  // Start with pure sine wave
  complexOsc_foldAmount = 0.0f;    // Start with no wavefolding
  
  // Start audio callback
  DAISY.begin(AudioCallback);
  
  Serial.println("Weasel Initialised with AGGRESSIVE Wavefolder, Moog Filter, and Buchla-style Timbre Control");
}

void loop() {
  // POTENTIOMETER READ
  modOsc_pitch = readMuxChannel(MOD_OSC_PITCH_CHANNEL, 16.35f, 5274.0f);
  modOsc_modAmount = readMuxChannel(MOD_AMOUNT_CHANNEL, 0.0f, 1000.0f);
  complexOsc_basePitch = readMuxChannel(COMPLEX_OSC_PITCH_CHANNEL, 55.0f, 1760.0f);
  complexOsc_timbreAmount = readMuxChannel(COMPLEX_OSC_TIMBRE_CHANNEL, 0.0f, 1.0f);  // WAVE MORPHING AMOUNT
  complexOsc_foldAmount = readMuxChannel(COMPLEX_OSC_FOLD_CHANNEL, 0.0f, 1.0f);      // WAVEFOLDING AMOUNT
  
  // SERIAL DEBUG
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 200) { // Print every 200ms to avoid flooding
    Serial.print("Mod Freq: ");
    Serial.print(modOsc_pitch);
    Serial.print("Hz | Mod Amount: ");
    Serial.print(modOsc_modAmount/10); // SCALE TO PERCENTAGE
    Serial.print("% | Complex Pitch: ");
    Serial.print(complexOsc_basePitch);
    Serial.print("Hz | Timbre: ");
    Serial.print(complexOsc_timbreAmount * 100); // SCALE TO PERCENTAGE
    Serial.print("% | Fold: ");
    Serial.print(complexOsc_foldAmount * 100); // SCALE TO PERCENTAGE
    Serial.println("%"); 
    lastPrint = millis();
  }
  
  delay(10); // Small delay to prevent reading too fast
}