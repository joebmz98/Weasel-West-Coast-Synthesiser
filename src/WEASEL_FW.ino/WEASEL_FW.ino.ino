#include "DaisyDuino.h"

// MUX_1 PINS
#define MUX_S0 0
#define MUX_S1 1
#define MUX_S2 2
#define MUX_S3 3
#define MUX_SIG A0  // MUX signal pin

// DEFINE DAISYSEED
DaisyHardware hw;

//INIT OSCILLATORS
static Oscillator complexOsc;  // This is the CARRIER oscillator
static Oscillator modOsc;      // This is the MODULATOR oscillator

// OSCILLATOR PARAMETER VARIABLES
float complexOsc_basePitch;   // COMPLEX OSC BASE PITCH
float modOsc_pitch;           // MOD OSC BASE PITCH
float modOsc_modAmount;              // MOD AMOUNT AFFECTING COMPLEX OSC

// OUTPUT MIXER
float mixOut_signal;        // OUTPUT MIX SUM

void AudioCallback(float **in, float **out, size_t size) {
  for (size_t i = 0; i < size; i++) {

    // PROCESS MODULATOR OSCILLATOR
    modOsc.SetFreq(modOsc_pitch);
    float modOsc_signal = modOsc.Process();  // INIT OSC FLOAT & PROCESS
    
    // PROCESS COMPLEX OSCILLATOR
      // PROCESS FM
      float modulatedFreq = complexOsc_basePitch + (modOsc_signal * modOsc_modAmount); // INIT MOD FLOAT & PROCESS FM
        // ENSURE MINIMUM FREQUENCY ( NO NEGATIVES )
        modulatedFreq = max(modulatedFreq, 17.0f);  // MIN 17Hz
    
    complexOsc.SetFreq(modulatedFreq);
    float complexOsc_signal = complexOsc.Process(); // INIT OSC FLOAT & PROCESS


    // SUMMATION
    mixOut_signal = complexOsc_signal;
    
    // OUTPUT (carrier only - the modulator is not heard in pure FM)
    out[0][i] = mixOut_signal;
    out[1][i] = mixOut_signal; 
  }
}

void setup() {
  Serial.begin(9600);
  
  // INIT 16-BIT ADC
  analogReadResolution(16);
  
  // Initialize seed at 48kHz
  float sample_rate;
  hw = DAISY.init(DAISY_SEED, AUDIO_SR_48K);
  sample_rate = DAISY.get_samplerate();
  
  complexOsc.Init(sample_rate);
  modOsc.Init(sample_rate);
  
  // INIT CARRIER OSC (the one you hear)
  complexOsc.SetWaveform(complexOsc.WAVE_SIN);
  complexOsc.SetFreq(440);
  complexOsc.SetAmp(0.7);  // Increased amplitude for better hearing
  
  // INIT MODULATOR OSC (affects carrier's frequency)
  modOsc.SetWaveform(modOsc.WAVE_SIN);
  modOsc.SetFreq(1.0);    // Start with very slow modulation
  modOsc.SetAmp(1.0);     // Full amplitude for modulation
  
  // Set initial values
  complexOsc_basePitch = 440.0f;
  modOsc_pitch = 1.0f;
  modOsc_modAmount = 0.0f;
  
  // Start audio callback
  DAISY.begin(AudioCallback);
  
  Serial.println("Weasel Initialised");
}

void loop() {
  // Read potentiometers
  modOsc_pitch = 2500.0f * (analogRead(PITCHPOT_PIN2) / 65535.0f) + 17.0f;  // 0.1Hz to 100Hz
  modOsc_modAmount = 1000.0f * (analogRead(MODPOT_PIN) / 65535.0f);              // 17 to 2500Hz modulation depth

  complexOsc_basePitch = 1760.0f * (analogRead(PITCHPOT_PIN) / 65535.0f) + 55.0f;  // 55Hz to 1760Hz
  
  // Serial debug
  Serial.print("Mod Freq: ");
  Serial.print(modOsc_pitch);
  Serial.print("Hz | Mod Amount: ");
  Serial.print(modOsc_modAmount/10);
  Serial.print("% | Base Pitch: ");
  Serial.print(complexOsc_basePitch);
  Serial.println("Hz");
  
  //delay(50);
}