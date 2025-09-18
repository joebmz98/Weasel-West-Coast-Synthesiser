#include "DaisyDuino.h"

// MUX PINS
#define MUX_S0 0
#define MUX_S1 1
#define MUX_S2 2
#define MUX_S3 3
#define MUX_SIG A0  // MUX signal pin

// MUX CHANNEL ASSIGNMENTS (as requested)
#define MOD_OSC_PITCH_CHANNEL 0    // C0 - modOsc_pitch
#define MOD_AMOUNT_CHANNEL 1       // C1 - modOsc_modAmount  
#define COMPLEX_OSC_PITCH_CHANNEL 2 // C2 - complexOsc_pitch

// DEFINE DAISYSEED
DaisyHardware hw;

//INIT OSCILLATORS
static Oscillator complexOsc;  // This is the CARRIER oscillator
static Oscillator modOsc;      // This is the MODULATOR oscillator

// OSCILLATOR PARAMETER VARIABLES
float complexOsc_basePitch;   // COMPLEX OSC BASE PITCH
float modOsc_pitch;           // MOD OSC BASE PITCH
float modOsc_modAmount;       // MOD AMOUNT AFFECTING COMPLEX OSC

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
    float modulatedFreq = complexOsc_basePitch + (modOsc_signal * modOsc_modAmount);
    modulatedFreq = max(modulatedFreq, 17.0f);  // MIN 17Hz
    
    complexOsc.SetFreq(modulatedFreq);
    float complexOsc_signal = complexOsc.Process();

    // OUTPUT
    out[0][i] = complexOsc_signal;
    out[1][i] = complexOsc_signal; 
  }
}

void setup() {
  Serial.begin(9600);
  
  // Initialize MUX control pins
  pinMode(MUX_S0, OUTPUT);
  pinMode(MUX_S1, OUTPUT);
  pinMode(MUX_S2, OUTPUT);
  pinMode(MUX_S3, OUTPUT);
  
  // Set all MUX pins low initially
  digitalWrite(MUX_S0, LOW);
  digitalWrite(MUX_S1, LOW);
  digitalWrite(MUX_S2, LOW);
  digitalWrite(MUX_S3, LOW);
  
  // INIT 16-BIT ADC
  analogReadResolution(16);
  
  // Initialize seed at 48kHz
  float sample_rate;
  hw = DAISY.init(DAISY_SEED, AUDIO_SR_48K);
  sample_rate = DAISY.get_samplerate();
  
  complexOsc.Init(sample_rate);
  modOsc.Init(sample_rate);
  
  // INIT CARRIER OSC 
  complexOsc.SetWaveform(complexOsc.WAVE_SIN);
  complexOsc.SetAmp(0.5);
  
  // INIT MODULATOR OSC (affects carrier's frequency)
  modOsc.SetWaveform(modOsc.WAVE_SIN);
  modOsc.SetAmp(0.5);
  
  // Set initial values
  complexOsc_basePitch = 440.0f;
  modOsc_pitch = 1.0f;
  modOsc_modAmount = 0.0f;
  
  // Start audio callback
  DAISY.begin(AudioCallback);
  
  Serial.println("Weasel Initialised");
}

void loop() {
  // Read potentiometers from MUX channels
  modOsc_pitch = readMuxChannel(MOD_OSC_PITCH_CHANNEL, 16.35f, 5274.0f);
  modOsc_modAmount = readMuxChannel(MOD_AMOUNT_CHANNEL, 0.0f, 1000.0f);
  complexOsc_basePitch = readMuxChannel(COMPLEX_OSC_PITCH_CHANNEL, 16.4f, 1047.0f);
  
  // Serial debug
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 200) { // Print every 200ms to avoid flooding
    Serial.print("Mod Freq: ");
    Serial.print(modOsc_pitch);
    Serial.print("Hz | Mod Amount: ");
    Serial.print(modOsc_modAmount/10); // SCALE TO PERCENTAGE
    Serial.print("% | Complex Pitch: ");
    Serial.print(complexOsc_basePitch);
    Serial.println("Hz"); 
    lastPrint = millis();
  }
  
  delay(10); // Small delay to prevent reading too fast
}