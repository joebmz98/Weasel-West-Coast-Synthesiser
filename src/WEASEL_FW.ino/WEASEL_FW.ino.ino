#include "DaisyDuino.h"

// PINS
#define PITCHPOT_PIN A0

DaisyHardware hw;
static Oscillator osc;

size_t num_channels;

// FLOAT VARIABLES
float osc1Pitch;

void MyCallback(float **in, float **out, size_t size) {
  float sine_signal;

  // SET OSC1 PITCH
  osc.SetFreq(osc1Pitch);

  // OUTPUT
  for (size_t i = 0; i < size; i++) {
    sine_signal = osc.Process();
    out[0][i] = sine_signal;
    out[1][i] = sine_signal; 
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
  osc.Init(sample_rate);

  // Set the parameters for oscillator 
  osc.SetWaveform(osc.WAVE_SIN);
  osc.SetFreq(440);
  osc.SetAmp(0.5);

  // start callback
  DAISY.begin(MyCallback);
}

void loop() {

  // INIT 16-BIT ADC
  analogReadResolution(16);

  osc1Pitch = 1760.0 * (analogRead(PITCHPOT_PIN) / 65535.0) + 55.0;

}
