#include "DaisyDuino.h"

// PINS
#define PITCHPOT_PIN A0
#define PITCHPOT_PIN2 A1
#define MODPOT_PIN A2

// DEFINE DAISYSEED
DaisyHardware hw;

//INIT OSCILLATORS
static Oscillator complexOsc;
static Oscillator modOsc;

size_t num_channels;

// OSCILLATOR PARAMETER VARIABLES
float complexOsc_pitch;
float modOsc_pitch;
float modOsc_modAmount;

void MyCallback(float **in, float **out, size_t size) {
  
  // INIT OSCILLATOR FLOATS
  float complexOsc_signal;
  float modOsc_signal;

  // INIT MIX OUT
  float mixSum_signal;


  for (size_t i = 0; i < size; i++) {

    // PROCESS MOD OSCILLATOR
    //    - FREQ
    modOsc.SetFreq(modOsc_pitch);
    modOsc_signal = modOsc.Process();
    Serial.println(modOsc_signal);

    // PROCESS COMPLEX OSCILLATOR
    //    - FREQ
    complexOsc.SetFreq(complexOsc_pitch * (modOsc_signal /* * modOsc_modAmount*/) + 55); // FM MODULATION // "+ 55" = Minimum freq of 55Hz
    complexOsc_signal = complexOsc.Process();

    mixSum_signal = complexOsc_signal; // + modOsc_signal;

    // OUTPUT
    out[0][i] = mixSum_signal;
    out[1][i] = mixSum_signal; 
  }
}

void setup() {

  // SERIAL MONITOR INIT
  Serial.begin(9600);

  // Initialize seed at 48kHz
  float sample_rate;
  hw = DAISY.init(DAISY_SEED, AUDIO_SR_48K);
  num_channels = hw.num_channels;
  sample_rate = DAISY.get_samplerate();
  complexOsc.Init(sample_rate);
  modOsc.Init(sample_rate);

  // INIT COMPLEX OSC
  complexOsc.SetWaveform(complexOsc.WAVE_SIN);
  complexOsc.SetFreq(440);
  complexOsc.SetAmp(0.5);
  // INIT MODULATION OSC
  modOsc.SetWaveform(modOsc.WAVE_SIN);
  modOsc.SetFreq(440);
  modOsc.SetAmp(0.5);

  // start callback
  DAISY.begin(MyCallback);
}

void loop() {

  // INIT 16-BIT ADC
  analogReadResolution(16);

  modOsc_pitch = 2500.0 * (analogRead(PITCHPOT_PIN2) / 65535.0) + 17.0;
  modOsc_modAmount = (analogRead(MODPOT_PIN) / 65535.0) + 1.0; // MODLATION COEFFICIENT

  complexOsc_pitch = 1760.0 * (analogRead(PITCHPOT_PIN) / 65535.0) + 55.0; // 1760 = MAX PITCH // +55 = MIN PITCH


  // SERIAL DEBUG
  //Serial.println(modOsc_modAmount);
  //delay(1);

}
