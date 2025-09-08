#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>

// GUItool: begin automatically generated code
AudioSynthWaveform       sampleHoldOsc; //xy=365.50000762939453,612.7500095367432
AudioMixer4              modOscCtrlMxr; //xy=631.7500076293945,517.7500076293945
AudioMixer4              complexOscCtrlMxr; //xy=653.7500076293945,425.7500057220459
AudioSynthWaveformModulated complexOscFM;   //xy=859.7500076293945,428.75003814697266
AudioSynthWaveform       modOsc; //xy=860.7500343322754,481.75000858306885
AudioSynthWaveform       complexOsc;      //xy=874.7500076293945,396.7500066757202
AudioEffectMultiply      complexOscAM;      //xy=1365.7501373291016,389.75006675720215
AudioMixer4              complexOscMxr; //xy=1597.7500228881836,405.75000762939453
AudioMixer4              modOscMxr; //xy=1607.7500190734863,484.75003814697266
AudioMixer4              outputMxr;      //xy=1831.7501335144043,432.0000686645508
AudioOutputI2S           dacOut;         //xy=2021.750072479248,438.00000858306885
AudioMixer4              complexOscModCtrl; //xy=2237.7500228881836,387.00003814697266
AudioConnection          patchCord1(sampleHoldOsc, 0, complexOscCtrlMxr, 1);
AudioConnection          patchCord2(sampleHoldOsc, 0, modOscCtrlMxr, 1);
AudioConnection          patchCord3(complexOscCtrlMxr, 0, complexOscFM, 0);
AudioConnection          patchCord4(complexOscFM, 0, complexOscMxr, 1);
AudioConnection          patchCord5(modOsc, 0, modOscMxr, 0);
AudioConnection          patchCord6(modOsc, 0, complexOscAM, 1);
AudioConnection          patchCord7(modOsc, 0, complexOscCtrlMxr, 0);
AudioConnection          patchCord8(complexOsc, 0, complexOscAM, 0);
AudioConnection          patchCord9(complexOscAM, 0, complexOscMxr, 0);
AudioConnection          patchCord10(complexOscMxr, 0, outputMxr, 0);
AudioConnection          patchCord11(modOscMxr, 0, outputMxr, 1);
AudioConnection          patchCord12(outputMxr, 0, dacOut, 0);
AudioConnection          patchCord13(outputMxr, 0, dacOut, 1);
// GUItool: end automatically generated code

AudioControlSGTL5000     audioControl;     //xy=357.25000762939453,95.00000190734863

// Oscillator parameters
float complexFreq = 440.0;  // Current complex oscillator frequency
float modFreq = 220.0;      // Current modulation oscillator frequency
const float AMPLITUDE = 0.7;       // Amplitude level

// Potentiometer pins
const int MOD_FREQ_POT = A0;    // Modulation oscillator frequency control
const int COMPLEX_FREQ_POT = A1; // Complex oscillator frequency control
const int COMPLEX_CTRL_MIX_POT = A2; // Complex Osc Ctrl Mixer channel 0/1 mix
const int COMPLEX_AM_GAIN_POT = A3;  // Complex Osc AM control gain
const int COMPLEX_MXR_MIX_POT = A4;  // Complex Osc Mixer channel 0/1 mix

void setup() {
  Serial.begin(9600);
  
  // Initialize potentiometer pins
  pinMode(MOD_FREQ_POT, INPUT);
  pinMode(COMPLEX_FREQ_POT, INPUT);
  pinMode(COMPLEX_CTRL_MIX_POT, INPUT);
  pinMode(COMPLEX_AM_GAIN_POT, INPUT);
  pinMode(COMPLEX_MXR_MIX_POT, INPUT);
  
  // Audio memory allocation
  AudioMemory(50);
  
  // Initialize audio shield
  audioControl.enable();
  audioControl.volume(0.8);
  
  // Configure complex oscillator: initial 440Hz triangle wave
  complexOsc.begin(AMPLITUDE, complexFreq, WAVEFORM_TRIANGLE);
  
  // Configure modulation oscillator: initial 220Hz triangle wave
  modOsc.begin(0.1, modFreq, WAVEFORM_TRIANGLE);
  
  // Configure sample and hold oscillator: 1Hz sample and hold waveform
  sampleHoldOsc.begin(1.0, 1.0, WAVEFORM_SAMPLE_HOLD);
  
  // Configure mixer gains
  outputMxr.gain(0, 0.7);  // Complex oscillator level
  outputMxr.gain(1, 0.0);  // Modulation oscillator level
  outputMxr.gain(2, 0.0);  // Unused channels
  outputMxr.gain(3, 0.0);
  
  // Set modOscCtrlMxr channel 0 gain to 0.5
  modOscCtrlMxr.gain(0, 0.5);
  
  Serial.println("Buchla Easel Synth Initialized");
  Serial.print("Complex Osc: ");
  Serial.print(complexFreq);
  Serial.println("Hz Triangle");
  Serial.print("Mod Osc: ");
  Serial.print(modFreq);
  Serial.println("Hz Triangle");
  Serial.print("Sample & Hold Osc: 1Hz");
}

void loop() {
  // Read potentiometers and update frequencies
  readPotentiometers();
  
  // Update oscillator frequencies
  updateOscillators();
  
  // Update mixer controls
  updateMixers();
  
  // Optional: Print status occasionally
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 1000) {
    lastPrint = millis();
    Serial.print("Complex: ");
    Serial.print(complexFreq);
    Serial.print("Hz | Mod: ");
    Serial.print(modFreq);
    Serial.println("Hz");
  }
  
  // Small delay for stability
  delay(20);
}

void readPotentiometers() {
  // Read modulation oscillator frequency pot (A0) - range 1Hz to 500Hz
  int modPotValue = analogRead(MOD_FREQ_POT);
  modFreq = mapFloat(modPotValue, 0, 1023, 1.0, 100.0);
  
  // Read complex oscillator frequency pot (A1) - typical audio range
  int complexPotValue = analogRead(COMPLEX_FREQ_POT);
  complexFreq = mapFloat(complexPotValue, 0, 1023, 110.0, 2000.0);
}

void updateOscillators() {
  // Update modulation oscillator frequency
  static float lastModFreq = 0;
  if (abs(modFreq - lastModFreq) > 0.1) { // Only update if changed significantly
    modOsc.frequency(modFreq);
    lastModFreq = modFreq;
  }
  
  // Update complex oscillator frequency
  static float lastComplexFreq = 0;
  if (abs(complexFreq - lastComplexFreq) > 0.1) { // Only update if changed significantly
    complexOsc.frequency(complexFreq);
    lastComplexFreq = complexFreq;
  }
}

void updateMixers() {
  // Read complex oscillator control mixer mix pot (A2)
  int ctrlMixPotValue = analogRead(COMPLEX_CTRL_MIX_POT);
  float ctrlMix = ctrlMixPotValue / 1023.0;
  complexOscCtrlMxr.gain(0, 1.0 - ctrlMix); // Channel 0 (modOsc)
  complexOscCtrlMxr.gain(1, ctrlMix);       // Channel 1 (sampleHoldOsc)
  
  // Read complex oscillator AM gain pot (A3)
  int amGainPotValue = analogRead(COMPLEX_AM_GAIN_POT);
  float amGain = mapFloat(amGainPotValue, 0, 1023, 0.0, 1.0);
  // Since complexOscAM doesn't have direct gain control, we adjust input levels
  // You might need to adjust modOsc amplitude or use another mixer if needed
  
  // Read complex oscillator mixer mix pot (A4)
  int mxrMixPotValue = analogRead(COMPLEX_MXR_MIX_POT);
  float mxrMix = mxrMixPotValue / 1023.0;
  complexOscMxr.gain(0, 1.0 - mxrMix); // Channel 0 (complexOscAM)
  complexOscMxr.gain(1, mxrMix);       // Channel 1 (complexOscFM)
}

// Utility function for floating-point mapping
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Optional: Functions to modify parameters in real-time
void setComplexFrequency(float freq) {
  complexFreq = freq;
  complexOsc.frequency(complexFreq);
}

void setModFrequency(float freq) {
  modFreq = freq;
  modOsc.frequency(modFreq);
}

void setComplexWaveform(int waveform) {
  complexOsc.begin(AMPLITUDE, complexFreq, waveform);
}

void setModWaveform(int waveform) {
  modOsc.begin(AMPLITUDE, modFreq, waveform);
}

void setOutputLevels(float complexLevel, float modLevel) {
  outputMxr.gain(0, complexLevel);
  outputMxr.gain(1, modLevel);
}