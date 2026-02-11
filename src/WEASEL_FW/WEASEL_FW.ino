// ********************** //
// ==== WEASEL ========== //
// Buchla Music Easel     //
// Inspired Digital Synth //
// firmware version 0.1   //
// Hardware Version: 1.0  //
// ********************** //
// designed by            //
// .axs instruments       //
// ********************** //
// Description: Buchla Music Easel nspired digital synth
//              Virtual patch bay simulating modular
//              experience.
// ********************** //

// ********************** //
// LICENSE & COPYRIGHT
//
// This project is released under the MIT License.
// Copyright (c) 2024 .axs instruments
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
// ********************* //


#include "DaisyDuino.h"

// EXTRA INCLUSIONS
#include <MIDI.h>          // MIDI
#include <CpuLoadMeter.h>  // CPU LOAD METER
#include "wavefolder.h"    // EXTRACTED FROM DAISYSP
#include <math.h>

// DEBUG
//unsigned long lastCpuPrint = 0;
//CpuLoadMeter cpuMeter;

// -- MUX PINS - FIRST MUX
#define MUX1_S0 D0
#define MUX1_S1 D1
#define MUX1_S2 D2
#define MUX1_S3 D3
#define MUX1_SIG A0  // MUX signal pin

// -- MUX PINS - SECOND MUX
#define MUX2_S0 D4
#define MUX2_S1 D5
#define MUX2_S2 D6
#define MUX2_S3 D7
#define MUX2_SIG A1  // Second MUX signal pin

// -- 4x7 BUTTON MATRIX PINS // VIRTUAL PATCH BAY
#define MATRIX_COL0 19  // Column 0 (B0) // SEQUENCER CV OUT
#define MATRIX_COL1 20  // Column 1 (B1) // EG CV OUT
#define MATRIX_COL2 21  // Column 2 (B2) // PULSAR CV OUT
#define MATRIX_COL3 22  // Column 3 (B3) // RANDOM VOLTAGE CV OUT
#define MATRIX_ROW0 23  // Row 0 (A0) // PULSAR PERIOD MODULATION
#define MATRIX_ROW1 24  // Row 1 (A1) // MOD OSC FREQ
#define MATRIX_ROW2 25  // Row 2 (A2) // MOD OSC MOD AMOUNT
#define MATRIX_ROW3 26  // Row 3 (A3) // COMPLEX OSC PITCH
#define MATRIX_ROW4 27  // Row 4 (A4) // COMPLEX OSC WAVEFOLDER AMOUNT
#define MATRIX_ROW5 28  // Row 5 (A5) // LPG CH1 LEVEL MODULATION
#define MATRIX_ROW6 29  // Row 6 (A6) // LPG CH2 LEVEL MODULATION

// -- SECOND BUTTON MATRIX PINS
#define MATRIX2_COL0 12  // X0
#define MATRIX2_COL1 13  // X1
#define MATRIX2_COL2 17  // X2
#define MATRIX2_COL3 18  // X3
#define MATRIX2_ROW0 8   // Y0
#define MATRIX2_ROW1 9   // Y1
#define MATRIX2_ROW2 10  // Y2
#define MATRIX2_ROW3 11  // Y3
#define MATRIX2_ROW4 30  // Y4

// -- BUTTON MATRIX 1 VARIABLES --
bool matrixStates[4][7] = { { false } };
bool lastMatrixStates[4][7] = { { false } };
unsigned long lastMatrixRead = 0;
const unsigned long MATRIX_READ_INTERVAL = 10;

const char* matrix1Functions[4][7] = {
  { "Seq CV -> Pulsar Decay", "Seq CV -> ModOsc Pitch", "Seq CV -> Mod Amount", "Seq CV -> CompOsc Pitch", "Seq CV -> WF Mod", "Seq CV -> LPG1", "Seq CV -> LPG2" },
  { "Env -> Pulsar Decay", "Env -> ModOsc Pitch", "Env -> Mod Amount", "Env -> CompOsc Pitch", "Env -> WF Mod", "Env -> LPG1", "Env -> LPG2" },
  { "Pulsar -> Pulsar Decay", "Pulsar -> ModOsc Pitch", "Pulsar -> Mod Amount", "Pulsar -> CompOsc Pitch", "Pulsar -> WF Mod", "Pulsar -> LPG1", "Pulsar -> LPG2" },
  { "S&H -> Pulsar Decay", "S&H -> ModOsc Pitch", "S&H -> Mod Amount", "S&H -> CompOsc Pitch", "S&H -> WF Mod", "S&H -> LPG1", "S&H -> LPG2" }
};

// -- BUTTON MATRIX 2 --
bool matrix2CurrentStates[4][5] = { { false } };
bool matrix2PreviousStates[4][5] = { { false } };
bool matrix2DebouncedStates[4][5] = { { false } };
unsigned long lastMatrix2Read = 0;

// -- POT VARIABLES --
const int NUM_POTS_PER_MUX = 16;
const int TOTAL_POTS = 32;  // 2 muxes * 16 channels each
int potValues[TOTAL_POTS] = { 0 };
int lastPotValues[TOTAL_POTS] = { 0 };
unsigned long lastPotRead = 0;
const unsigned long POT_READ_INTERVAL = 1;  // Read pots every Xms
String potNames[TOTAL_POTS];                // Array to store pot names
const int POT_THRESHOLD = 50;               //


// -- DAISY DUINO OBJECTS --
DaisyHardware hw;  // DAISY SEED
// -- OSCILLATORS
Oscillator complexOsc;       // COMPLEX OSCILLATOR
Oscillator complexOscMorph;  // COMPLEX MORPH OSCILLATOR
Oscillator modulationOsc;    // COMPLEX OSCILLATOR
// -- ENVELOPE GENERATOR
Adsr env;  // ENV GENERATOR
// -- PULSAR GENERATOR
Adsr pulsar;  // ENV GENERATOR FOR PULSAR
// -- LOW-PASS - LPG
MoogLadder lpGateFilter1;  // FILTER FOR LPG CHAN1
MoogLadder lpGateFilter2;  // FILTER FOR LPG CHAN2
// -- OUTPUT ANALOGUE FILTER
MoogLadder outputFilter;
// -- REVERB
ReverbSc reverb;  // REVERB
// -- WAVEFOLDER
daisysp::Wavefolder waveFolder;


// -- OBJECT VARIABLES --
// -- OSCILLATORS
// -- COMPLEX
float complexOscFreq;                 // CONTROLS FREQUENCY
float complexOscFine;                 // CONTROLS FINE TUNE
float complexOscFreqCoeff;            // COEFF FOR FREQUENCY MODULATION USING MATRIX
float complexOscWF;                   // CONTROLS WF AMOUNT
float complexOscWFCoeff;              // COEFF FOR WF MODULATION USING MATRIX
float complexOscWFModDepth = 0.0f;    // WAVEFOLDER MOD DEPTH
float complexOscFreqModDepth = 0.0f;  // FREQ MOD DEPTH
bool complexOscInverted = false;      // TOGGLE POLARITY
bool complexOscMidiEnabled = false;   // TOGGLE MIDI
float complexOscMidiFreq = 0.0f;      // FREQUENCY OFFSET FROM MIDI
float complexOscMorphMix;             // COMPLEX OSC TIMBRE AMOUNT
int morphWaveformIndex = 0;           // 0 = SAW, 1 = SQUARE, 2 = TRIANGLE
float complexOscSigLevel = 0.0f;      // LPG CONTROL

// -- MODULATION
float modulationOscFreq = 0.0f;           // CONTROLS FREQUENCY
float modulationFine = 0.0f;              // CONTROLS FINE TUNE
float modulationFreqCoeff = 0.0f;         // COEFF FOR FREQUENCY MODULATION USING MATRIX
float modulationOscMod = 0.0f;            // CONTROLS MOD AMOUNT APPLIED TO COMPLEXOSC
float modulationOscModCoeff;              // COEFF FOR MOD AMOUNT MODULATION USING MATRIX
float modulationFreqModDepth = 0.0f;      // FREQ MOD DEPTH
float modulationOscModCoeffDepth = 0.0f;  // MODULATION MOD DEPTH
bool useAmplitudeMod = false;             // False = FM, True = AM
bool modulationMidiEnabled = false;       // TOGGLE MIDI
float modulationOscMidiFreq = 0.0f;       // FREQUENCY OFFSET FROM MIDI
int modWaveformIndex = 0;                 // 0: SIN, 1: TRI, 2: SQUARE, 3: SAW
float modOscSigLevel = 0.0f;              // LPG CONTROL

// -- LPG --
float vcaComplexOsc;              // AMPLITUDE COEFF FOR COMPLEXOSC
float vcaModulationOsc;           // AMPLITUDE COEFF FOR MODULATIONOSC
float lpComplexOsc;               // FREQ COEFF FOR COMPLEXOSC
float lpModulationOsc;            // FREQ COEFF FOR MODULATIONOSC
float foldedLpgModAmount = 0.0f;  // Multiplier for Folded LPG mod
float modOscLpgModAmount = 0.0f;  // Multiplier for Mod Osc LPG mod
enum LpgMode { LPG_MODE_VCA,
               LPG_MODE_LP,
               LPG_MODE_COMBI };
LpgMode foldedLpgMode = LPG_MODE_COMBI;  // Default to COMBI mode
LpgMode modOscLpgMode = LPG_MODE_COMBI;  // Default to COMBI mode

// -- ENVELOPE [BUCHLA ASD/ASR] --
float attackTime = 0.0f;
float releaseTime = 0.0f;
float sustainDuration = 0.0f;
uint32_t gateRemainingSamples = 0;

// -- PULSAR ENV --
enum PulsarMode { PULSAR_MODE_SEQ,
                  PULSAR_MODE_MIDI,
                  PULSAR_MODE_OSC };
PulsarMode currentPulsarMode = PULSAR_MODE_SEQ;
float pulsarReleaseTime = 0.02f;
float pulsarEnvSig = 0.0f;
bool lastPulsarEnvActive = false;
float pulsarPeriodModCoeff = 0.0f;
bool pulsarGate = false;

// -- SAMPLE&HOLD / RANDOM VOLTAGE --
enum RandomMode { RANDOM_MODE_SEQ,
                  RANDOM_MODE_PULSAR,
                  RANDOM_MODE_MIDI };
RandomMode currentRandomMode = RANDOM_MODE_SEQ;
float currentRandomValue = 0.0f;

// -- REVERB --
float reverbMix;  // REVERB MIX COEFF

// -- MIDI --
#define MIDI_RX_PIN 14  // USART1 Rx (Digital pin 30)
  // MIDI OBJECT
MIDI_CREATE_INSTANCE(HardwareSerial, Serial1, MIDI);

// -- SEQUENCER --
int seqCurrentStep = 0;
bool seqStepEnabled[5] = { true, true, true, true, true };  // All steps ON by default
float seqStepCV[5] = { 0.0f };                              // Stores current CV from MUX1 0-4
float seqClockSpeed = 100.0f;
int seqMaxSteps = 5;  // Default sequence length
// AUDIO INTERRUPT TIMING
uint32_t seqSampleCounter = 0;  // Counts samples to trigger the next step
float activeSeqCV = 0.0f;       // The CV of the currently active step
// SEQUENCER MODES
enum SeqTriggerMode { SEQ_TRIGGER_CLOCK,
                      SEQ_TRIGGER_PULSAR,
                      SEQ_TRIGGER_MIDI };
SeqTriggerMode currentSeqTriggerMode = SEQ_TRIGGER_CLOCK;  // Default Mode
bool midiTriggerPending = false;                           // Flag to catch MIDI notes for the sequencer
int currentMidiNote = 60;                                  // Default to Middle C when no midi is detected
uint8_t midiTriggerCounter = 0;
const uint8_t MIDI_TRIGGER_HOLDOFF = 10;  // Adjust as needed (5-20 is good)

// -- OPTIMIZATION VARIABLES --
float rawModulationValues[7] = { 0 };  // Pre-calculated modulation values
float ln2 = 0.69314718056f;            // ln(2) for fast exp approximations
uint32_t modUpdateCounter = 0;         // REMOVE THE DUPLICATE IN AudioCallback
const uint32_t MOD_UPDATE_RATE = 4;    // Update modulation every 4 samples

// -- FAST MATH APPROXIMATIONS --
inline float fast_exp2_approx(float x) {
  // Fast 2^x approximation using Taylor series expansion
  x = 1.0f + x * ln2;  // First-order approximation
  // Second-order improvement for better accuracy
  return x * (1.0f + (x - 1.0f) * 0.5f);
}

inline float fast_pow_approx(float base, float exponent) {
  // Simple pow approximation for audio use
  // Works well for exponent in range 0-1
  return 1.0f + (base - 1.0f) * exponent;
}

// -- MUX INIT --
void selectMuxChannel(int channel, int s0, int s1, int s2, int s3) {
  digitalWrite(s0, bitRead(channel, 0));
  digitalWrite(s1, bitRead(channel, 1));
  digitalWrite(s2, bitRead(channel, 2));
  digitalWrite(s3, bitRead(channel, 3));
}

// -- INIT POTS --
void initPotentiometers() {
  // Initialize MUX control pins as outputs
  pinMode(MUX1_S0, OUTPUT);
  pinMode(MUX1_S1, OUTPUT);
  pinMode(MUX1_S2, OUTPUT);
  pinMode(MUX1_S3, OUTPUT);

  pinMode(MUX2_S0, OUTPUT);
  pinMode(MUX2_S1, OUTPUT);
  pinMode(MUX2_S2, OUTPUT);
  pinMode(MUX2_S3, OUTPUT);

  // Set initial states
  digitalWrite(MUX1_S0, LOW);
  digitalWrite(MUX1_S1, LOW);
  digitalWrite(MUX1_S2, LOW);
  digitalWrite(MUX1_S3, LOW);

  digitalWrite(MUX2_S0, LOW);
  digitalWrite(MUX2_S1, LOW);
  digitalWrite(MUX2_S2, LOW);
  digitalWrite(MUX2_S3, LOW);

  // Initialize analog pins
  pinMode(MUX1_SIG, INPUT);
  pinMode(MUX2_SIG, INPUT);

  // Initialize pot names
  for (int i = 0; i < TOTAL_POTS; i++) {
    if (i < NUM_POTS_PER_MUX) {
      // Pots on MUX1
      if (i == 0) potNames[i] = "MUX1-CH0: Sequencer Step 1 CV";
      else if (i == 1) potNames[i] = "MUX1-CH1: Sequencer Step 2 CV";
      else if (i == 2) potNames[i] = "MUX1-CH2: Sequencer Step 3 CV";
      else if (i == 3) potNames[i] = "MUX1-CH3: Sequencer Step 4 CV";
      else if (i == 4) potNames[i] = "MUX1-CH4: Sequencer Step 5 CV";
      else if (i == 5) potNames[i] = "MUX1-CH5: Attack";
      else if (i == 6) potNames[i] = "MUX1-CH6: Sustain";
      else if (i == 7) potNames[i] = "MUX1-CH7: Decay/Release";
      else if (i == 8) potNames[i] = "MUX1-CH8: Pulsar Period Modulation Control";
      else if (i == 9) potNames[i] = "MUX1-CH9: Pulsar Period";
      else if (i == 10) potNames[i] = "MUX1-CH10: Modulation Oscillator Frequency Control";
      else if (i == 11) potNames[i] = "MUX1-CH11: Modulation Oscillator Frequency";
      else if (i == 12) potNames[i] = "MUX1-CH12: Modulation Oscillator Fine Tune +/-3";
      else if (i == 13) potNames[i] = "MUX1-CH13: Modulation Oscillator Modulation Control";
      else if (i == 14) potNames[i] = "MUX1-CH14: Modulation Oscillator Modulation Control";
      else if (i == 15) potNames[i] = "MUX1-CH15: Complex Oscillator Frequency Control";
    } else {
      // Pots on MUX2 (indices 16-31)
      int mux2Index = i - NUM_POTS_PER_MUX;
      if (mux2Index == 0) potNames[i] = "MUX2-CH0: Complex Oscillator Frequency";
      else if (mux2Index == 1) potNames[i] = "MUX2-CH1: Complex Oscillator Fine Tune";
      else if (mux2Index == 2) potNames[i] = "MUX2-CH2: Complex Oscillator Wavefolder Control";
      else if (mux2Index == 3) potNames[i] = "MUX2-CH3: Complex Oscillator Wavefolder";
      else if (mux2Index == 4) potNames[i] = "MUX2-CH4: Complex Oscillator Timbre";
      else if (mux2Index == 5) potNames[i] = "MUX2-CH5: LPG1 Control";
      else if (mux2Index == 6) potNames[i] = "MUX2-CH6: LPG1 Level";
      else if (mux2Index == 7) potNames[i] = "MUX2-CH7: LPG2 Control";
      else if (mux2Index == 8) potNames[i] = "MUX2-CH8: LPG2 Level";
      else if (mux2Index == 9) potNames[i] = "MUX2-CH9: Clock Speed";
      else if (mux2Index == 10) potNames[i] = "MUX2-CH10: Reverb Mix";
      else if (mux2Index == 11) potNames[i] = "MUX2-CH11: N/C";
      else if (mux2Index == 12) potNames[i] = "MUX2-CH12: N/C";
      else if (mux2Index == 13) potNames[i] = "MUX2-CH13: N/C";
      else if (mux2Index == 14) potNames[i] = "MUX2-CH14: N/C";
      else if (mux2Index == 15) potNames[i] = "MUX2-CH15: N/C";
    }
  }

  // Read initial pot values
  //readPotentiometers();
  // Copy to last values to avoid initial spurious changes
  for (int i = 0; i < TOTAL_POTS; i++) {
    lastPotValues[i] = potValues[i];
  }
}

// -- READ POTS --
void readPotentiometers() {
  for (int i = 0; i < TOTAL_POTS; i++) {
    if (i < NUM_POTS_PER_MUX) {
      selectMuxChannel(i, MUX1_S0, MUX1_S1, MUX1_S2, MUX1_S3);
      delayMicroseconds(100);  // Use the stable delay from your test
      potValues[i] = analogRead(MUX1_SIG);
    } else {
      int mux2Channel = i - NUM_POTS_PER_MUX;
      selectMuxChannel(mux2Channel, MUX2_S0, MUX2_S1, MUX2_S2, MUX2_S3);
      delayMicroseconds(100);
      potValues[i] = analogRead(MUX2_SIG);
    }
  }
}

// -- INIT BUTTON MATRICES --
// -- MATRIX 1
void initButtonMatrix() {
  // Initialize column pins as outputs
  pinMode(MATRIX_COL0, OUTPUT);
  pinMode(MATRIX_COL1, OUTPUT);
  pinMode(MATRIX_COL2, OUTPUT);
  pinMode(MATRIX_COL3, OUTPUT);

  // Initialize row pins as inputs with pulldown
  pinMode(MATRIX_ROW0, INPUT_PULLDOWN);
  pinMode(MATRIX_ROW1, INPUT_PULLDOWN);
  pinMode(MATRIX_ROW2, INPUT_PULLDOWN);
  pinMode(MATRIX_ROW3, INPUT_PULLDOWN);
  pinMode(MATRIX_ROW4, INPUT_PULLDOWN);
  pinMode(MATRIX_ROW5, INPUT_PULLDOWN);
  pinMode(MATRIX_ROW6, INPUT_PULLDOWN);

  // Start with all columns LOW
  digitalWrite(MATRIX_COL0, LOW);
  digitalWrite(MATRIX_COL1, LOW);
  digitalWrite(MATRIX_COL2, LOW);
  digitalWrite(MATRIX_COL3, LOW);
}
// -- MATRIX 2
void initButtonMatrix2() {
  // Initialize column pins as outputs
  pinMode(MATRIX2_COL0, OUTPUT);
  pinMode(MATRIX2_COL1, OUTPUT);
  pinMode(MATRIX2_COL2, OUTPUT);
  pinMode(MATRIX2_COL3, OUTPUT);

  // Initialize row pins as inputs with pulldown
  pinMode(MATRIX2_ROW0, INPUT_PULLDOWN);
  pinMode(MATRIX2_ROW1, INPUT_PULLDOWN);
  pinMode(MATRIX2_ROW2, INPUT_PULLDOWN);
  pinMode(MATRIX2_ROW3, INPUT_PULLDOWN);
  pinMode(MATRIX2_ROW4, INPUT_PULLDOWN);

  // Start with all columns LOW
  digitalWrite(MATRIX2_COL0, LOW);
  digitalWrite(MATRIX2_COL1, LOW);
  digitalWrite(MATRIX2_COL2, LOW);
  digitalWrite(MATRIX2_COL3, LOW);
}
// -- READ BUTTON MATRICES --
// -- MATRIX 1
void readButtonMatrix() {
  for (int col = 0; col < 4; col++) {
    // Activate current column
    switch (col) {
      case 0:
        digitalWrite(MATRIX_COL0, HIGH);
        digitalWrite(MATRIX_COL1, LOW);
        digitalWrite(MATRIX_COL2, LOW);
        digitalWrite(MATRIX_COL3, LOW);
        break;
      case 1:
        digitalWrite(MATRIX_COL0, LOW);
        digitalWrite(MATRIX_COL1, HIGH);
        digitalWrite(MATRIX_COL2, LOW);
        digitalWrite(MATRIX_COL3, LOW);
        break;
      case 2:
        digitalWrite(MATRIX_COL0, LOW);
        digitalWrite(MATRIX_COL1, LOW);
        digitalWrite(MATRIX_COL2, HIGH);
        digitalWrite(MATRIX_COL3, LOW);
        break;
      case 3:
        digitalWrite(MATRIX_COL0, LOW);
        digitalWrite(MATRIX_COL1, LOW);
        digitalWrite(MATRIX_COL2, LOW);
        digitalWrite(MATRIX_COL3, HIGH);
        break;
    }

    delayMicroseconds(10);

    // Read all rows for this column
    for (int row = 0; row < 7; row++) {
      lastMatrixStates[col][row] = matrixStates[col][row];

      switch (row) {
        case 0: matrixStates[col][row] = (digitalRead(MATRIX_ROW0) == HIGH); break;
        case 1: matrixStates[col][row] = (digitalRead(MATRIX_ROW1) == HIGH); break;
        case 2: matrixStates[col][row] = (digitalRead(MATRIX_ROW2) == HIGH); break;
        case 3: matrixStates[col][row] = (digitalRead(MATRIX_ROW3) == HIGH); break;
        case 4: matrixStates[col][row] = (digitalRead(MATRIX_ROW4) == HIGH); break;
        case 5: matrixStates[col][row] = (digitalRead(MATRIX_ROW5) == HIGH); break;
        case 6: matrixStates[col][row] = (digitalRead(MATRIX_ROW6) == HIGH); break;
      }
    }
  }

  // Deactivate all columns
  digitalWrite(MATRIX_COL0, LOW);
  digitalWrite(MATRIX_COL1, LOW);
  digitalWrite(MATRIX_COL2, LOW);
  digitalWrite(MATRIX_COL3, LOW);
}
// -- MATRIX 2
void readButtonMatrix2() {
  int colPins[] = { MATRIX2_COL0, MATRIX2_COL1, MATRIX2_COL2, MATRIX2_COL3 };  // FIXED: Use COL3
  int rowPins[] = { MATRIX2_ROW0, MATRIX2_ROW1, MATRIX2_ROW2, MATRIX2_ROW3, MATRIX2_ROW4 };

  for (int col = 0; col < 4; col++) {
    // 1. ACTIVATE: Set column to OUTPUT and HIGH to source current
    pinMode(colPins[col], OUTPUT);
    digitalWrite(colPins[col], HIGH);

    delayMicroseconds(80);

    for (int row = 0; row < 5; row++) {
      matrix2PreviousStates[col][row] = matrix2CurrentStates[col][row];
      // Reading HIGH means button is pressed
      matrix2CurrentStates[col][row] = (digitalRead(rowPins[row]) == HIGH);
    }

    // 2. DEACTIVATE: Instead of digitalWrite(LOW), set to INPUT
    pinMode(colPins[col], INPUT);
  }
}

// -- UPDATE MATRIX MODULATION VALUES --
void updateMatrixModulation(float envSig, float pulsarEnvSig) {
  // Reset modulation values
  for (int i = 0; i < 7; i++) {
    rawModulationValues[i] = 0.0f;
  }

  // Calculate modulation from all sources
  for (int col = 0; col < 4; col++) {
    float src = 0.0f;

    // Get source value
    switch (col) {
      case 0: src = activeSeqCV; break;
      case 1: src = envSig; break;
      case 2: src = pulsarEnvSig; break;
      case 3: src = currentRandomValue; break;
    }

    // Apply different scaling for pulsar source
    float scaleFactor = (col == 2) ? 100.0f : 1.0f;
    float freqScaleFactor = (col == 2) ? 200.0f : 1.0f;

    // Accumulate modulation for each destination
    // Only process if matrix connection is active
    if (matrixStates[col][0]) rawModulationValues[0] += src * scaleFactor;
    if (matrixStates[col][1]) rawModulationValues[1] += src * freqScaleFactor;
    if (matrixStates[col][2]) rawModulationValues[2] += src * scaleFactor;
    if (matrixStates[col][3]) rawModulationValues[3] += src * freqScaleFactor;
    if (matrixStates[col][4]) rawModulationValues[4] += src * scaleFactor;
    if (matrixStates[col][5]) rawModulationValues[5] += src * freqScaleFactor;
    if (matrixStates[col][6]) rawModulationValues[6] += src * freqScaleFactor;
  }
}

// -- PRINT BUTTON PRESSES --
// -- SERIAL PRINT
// -- BUTTON FUNCTIONS
void printButtonChanges() {
  bool anyChange = false;

  // Check Matrix 1 (4x7) - Modulation Matrix
  for (int col = 0; col < 4; col++) {
    for (int row = 0; row < 7; row++) {
      if (matrixStates[col][row] != lastMatrixStates[col][row]) {
        anyChange = true;

        Serial.print(F("Matrix 1 - B"));
        Serial.print(col);
        Serial.print(F("+A"));
        Serial.print(row);
        Serial.print(F(" ["));
        Serial.print(matrix1Functions[col][row]);
        Serial.print(F("]: "));
        Serial.println(matrixStates[col][row] ? F("PRESSED") : F("RELEASED"));
      }
    }
  }

  // Check Matrix 2 (4x5) - XY Matrix
  for (int col = 0; col < 4; col++) {
    for (int row = 0; row < 5; row++) {
      if (matrix2CurrentStates[col][row] != matrix2PreviousStates[col][row]) {
        anyChange = true;

        Serial.print(F("Matrix 2 - X"));
        Serial.print(col);
        Serial.print(F("+Y"));
        Serial.print(row);
        Serial.print(F(" ["));

        // Specific Logic for Buttons
        if (col == 1 && row == 0) {
          Serial.print(F("SEQUENCER_MODE_BUTTON"));
          if (matrix2CurrentStates[col][row] && !matrix2PreviousStates[col][row]) {
            currentSeqTriggerMode = static_cast<SeqTriggerMode>((currentSeqTriggerMode + 1) % 3);
            Serial.print(F(" -> "));
            if (currentSeqTriggerMode == SEQ_TRIGGER_CLOCK) Serial.print(F("INTERNAL CLOCK"));
            else if (currentSeqTriggerMode == SEQ_TRIGGER_PULSAR) Serial.print(F("PULSAR ENVELOPE"));
            else if (currentSeqTriggerMode == SEQ_TRIGGER_MIDI) Serial.print(F("MIDI NOTE ON"));
          }
        } else if (col == 1 && row == 1) Serial.print(F("PULSER CLOCK MODE"));
        else if (col == 1 && row == 2) {
          Serial.print(F("OSCILLATOR MIDI_TOGGLE"));
          if (matrix2CurrentStates[col][row] && !matrix2PreviousStates[col][row]) {
            complexOscMidiEnabled = !complexOscMidiEnabled;
            modulationMidiEnabled = !modulationMidiEnabled;
          }
        } else if (col == 1 && row == 3) {
          Serial.print(F("RANDOM_TRIGGER_MODE_TOGGLE"));
          if (matrix2CurrentStates[col][row] && !matrix2PreviousStates[col][row]) {
            currentRandomMode = static_cast<RandomMode>((currentRandomMode + 1) % 3);
            Serial.print(F(" -> "));
            if (currentRandomMode == RANDOM_MODE_SEQ) Serial.print(F("SEQUENCER"));
            else if (currentRandomMode == RANDOM_MODE_PULSAR) Serial.print(F("PULSAR"));
            else Serial.print(F("MIDI NOTE ON"));
          }
        } else if (col == 3 && row == 1) {
          Serial.print(F("FM/AM_TOGGLE"));
          if (matrix2CurrentStates[col][row] && !matrix2PreviousStates[col][row]) {
            useAmplitudeMod = !useAmplitudeMod;
          }
        } else if (col == 3 && row == 2) {
          Serial.print(F("LPG_MODE_TOGGLE"));
          if (matrix2CurrentStates[col][row] && !matrix2PreviousStates[col][row]) {
            foldedLpgMode = static_cast<LpgMode>((foldedLpgMode + 1) % 3);
            Serial.print(F(" -> "));
            if (foldedLpgMode == LPG_MODE_VCA) Serial.print(F("VCA"));
            else if (foldedLpgMode == LPG_MODE_LP) Serial.print(F("LP"));
            else Serial.print(F("COMBI"));
          }
        } else if (col == 0 && row == 3) {
          Serial.print(F("MODOSC_LPG_MODE"));
          if (matrix2CurrentStates[col][row] && !matrix2PreviousStates[col][row]) {
            modOscLpgMode = static_cast<LpgMode>((modOscLpgMode + 1) % 3);
            Serial.print(F(" -> "));
            Serial.print(modOscLpgMode == LPG_MODE_VCA ? F("VCA") : modOscLpgMode == LPG_MODE_LP ? F("LP")
                                                                                                 : F("COMBI"));
          }
        } else if (col == 2 && row == 1) {
          Serial.print(F("MOD_WAVE_CYCLE"));
          if (matrix2CurrentStates[col][row] && !matrix2PreviousStates[col][row]) {
            modWaveformIndex = (modWaveformIndex + 1) % 4;
            Serial.print(F(" -> "));

            if (modWaveformIndex == 0) {
              Serial.print(F("SINE"));
              modulationOsc.SetWaveform(modulationOsc.WAVE_SIN);
            } else if (modWaveformIndex == 1) {
              Serial.print(F("TRI"));
              modulationOsc.SetWaveform(modulationOsc.WAVE_TRI);
            } else if (modWaveformIndex == 2) {
              Serial.print(F("SAW"));
              modulationOsc.SetWaveform(modulationOsc.WAVE_SAW);
            } else if (modWaveformIndex == 3) {
              Serial.print(F("SQUARE"));
              modulationOsc.SetWaveform(modulationOsc.WAVE_SQUARE);
            }
          }
        } else if (col == 0 && row == 2) {
          Serial.print(F("OSC_POLARITY_INVERT"));
          if (matrix2CurrentStates[col][row] && !matrix2PreviousStates[col][row]) {
            complexOscInverted = !complexOscInverted;
          }
        } else if (col == 2 && row == 2) {
          Serial.print(F("COMPLEX_WAVE_CYCLE"));
          if (matrix2CurrentStates[col][row] && !matrix2PreviousStates[col][row]) {
            morphWaveformIndex = (morphWaveformIndex + 1) % 3;
            Serial.print(F(" -> "));

            if (morphWaveformIndex == 0) {
              Serial.print(F("TRI"));
              complexOscMorph.SetWaveform(complexOscMorph.WAVE_TRI);
            } else if (morphWaveformIndex == 1) {
              Serial.print(F("SAW"));
              complexOscMorph.SetWaveform(complexOscMorph.WAVE_SAW);
            } else if (morphWaveformIndex == 2) {
              Serial.print(F("SQUARE"));
              complexOscMorph.SetWaveform(complexOscMorph.WAVE_SQUARE);
            }
          }
        }
        // -- SEQUENCER STEP TOGGLE SWITCHES (SPDT) --
        // Set state directly based on switch position, don't toggle
        else if (col == 3 && row == 3) {
          Serial.print(F("SEQ_STEP_1_TOGGLE"));
          // Directly set step state to switch position (HIGH = DISABLED, LOW = ENABLED)
          seqStepEnabled[0] = !matrix2CurrentStates[col][row];  // When switch is ON (HIGH), step is DISABLED (false)
          Serial.print(F(" -> "));
          Serial.print(seqStepEnabled[0] ? F("ENABLED") : F("DISABLED"));
        } else if (col == 0 && row == 4) {
          Serial.print(F("SEQ_STEP_2_TOGGLE"));
          seqStepEnabled[1] = !matrix2CurrentStates[col][row];
          Serial.print(F(" -> "));
          Serial.print(seqStepEnabled[1] ? F("ENABLED") : F("DISABLED"));
        } else if (col == 1 && row == 4) {
          Serial.print(F("SEQ_STEP_3_TOGGLE"));
          seqStepEnabled[2] = !matrix2CurrentStates[col][row];
          Serial.print(F(" -> "));
          Serial.print(seqStepEnabled[2] ? F("ENABLED") : F("DISABLED"));
        } else if (col == 2 && row == 4) {
          Serial.print(F("SEQ_STEP_4_TOGGLE"));
          seqStepEnabled[3] = !matrix2CurrentStates[col][row];
          Serial.print(F(" -> "));
          Serial.print(seqStepEnabled[3] ? F("ENABLED") : F("DISABLED"));
        } else if (col == 3 && row == 4) {
          Serial.print(F("SEQ_STEP_5_TOGGLE"));
          seqStepEnabled[4] = !matrix2CurrentStates[col][row];
          Serial.print(F(" -> "));
          Serial.print(seqStepEnabled[4] ? F("ENABLED") : F("DISABLED"));
        } else if (col == 0 && row == 0) {
          Serial.print(F("SEQ_LENGTH_CYCLE"));
          if (matrix2CurrentStates[col][row] && !matrix2PreviousStates[col][row]) {
            if (seqMaxSteps == 5) seqMaxSteps = 4;
            else if (seqMaxSteps == 4) seqMaxSteps = 3;
            else seqMaxSteps = 5;
            seqCurrentStep %= seqMaxSteps;
          }
        } else if (col == 0 && row == 1) {
          Serial.print(F("PULSAR_MODE_TOGGLE"));
          if (matrix2CurrentStates[col][row] && !matrix2PreviousStates[col][row]) {
            currentPulsarMode = static_cast<PulsarMode>((currentPulsarMode + 1) % 3);
          }
        } else {
          Serial.print(F("BUTTON_UNASSIGNED"));
        }

        Serial.print(F("] Status: "));
        Serial.println(matrix2CurrentStates[col][row] ? F("PRESSED") : F("RELEASED"));
      }
    }
  }

  if (anyChange) {
    Serial.println();
  }
}

// -- AUDIO PROCESSING --
void AudioCallback(float** in, float** out, size_t size) {

  // -- CPU --
  //cpuMeter.OnBlockStart();

  float sampleRate = DAISY.get_samplerate();

  // Update samples per tick less frequently to save CPU
  static uint32_t tickUpdateCounter = 0;
  uint32_t samplesPerTick = (uint32_t)((seqClockSpeed / 1000.0f) * sampleRate);

  if (++tickUpdateCounter >= 1000) {
    samplesPerTick = (uint32_t)((seqClockSpeed / 1000.0f) * sampleRate);
    tickUpdateCounter = 0;
  }

  for (size_t i = 0; i < size; i++) {
    // --- 1. TRIGGERS & SEQUENCER (optimized) ---
    bool seqStepTriggered = false;

    if (currentSeqTriggerMode == SEQ_TRIGGER_CLOCK) {
      seqSampleCounter++;
      if (seqSampleCounter >= samplesPerTick) {
        seqStepTriggered = true;
        seqSampleCounter = 0;
      }
    } else if (currentSeqTriggerMode == SEQ_TRIGGER_PULSAR) {
      bool isPulsarActive = pulsar.IsRunning();
      if (isPulsarActive && !lastPulsarEnvActive) {
        seqStepTriggered = true;
      }
      lastPulsarEnvActive = isPulsarActive;
    } else if (currentSeqTriggerMode == SEQ_TRIGGER_MIDI) {
      if (midiTriggerPending) {
        seqStepTriggered = true;
        // Decrement counter and clear flag when done
        if (--midiTriggerCounter == 0) {
          midiTriggerPending = false;
        }
      }
    }

    if (seqStepTriggered) {
      seqCurrentStep = (seqCurrentStep + 1) % seqMaxSteps;
      if (seqStepEnabled[seqCurrentStep]) {
        activeSeqCV = seqStepCV[seqCurrentStep];
        gateRemainingSamples = (uint32_t)(sustainDuration * sampleRate);
      } else {
        gateRemainingSamples = 0;
      }
    }

    // --- 2. PULSAR TRIGGER LOGIC ---
    bool pulsarTrigger = false;
    if (currentPulsarMode == PULSAR_MODE_SEQ) {
      pulsarTrigger = seqStepTriggered;
    } else if (currentPulsarMode == PULSAR_MODE_OSC) {
      if (!pulsar.IsRunning()) {
        pulsarTrigger = true;
      }
    } else if (currentPulsarMode == PULSAR_MODE_MIDI) {
      if (midiTriggerPending) {
        pulsarTrigger = true;
        // Decrement counter and clear flag when done
        if (--midiTriggerCounter == 0) {
          midiTriggerPending = false;
        }
      }
    }

    if (pulsarTrigger) {
      pulsar.SetAttackTime(0.02f);
      pulsar.SetDecayTime(0.02f);
      pulsar.SetSustainLevel(0.0f);
      pulsar.SetReleaseTime(pulsarReleaseTime);
      pulsar.Retrigger(true);
    }

    // --- 3. RANDOM TRIGGER LOGIC ---
    bool randomTrigger = false;
    if (currentRandomMode == RANDOM_MODE_SEQ) {
      randomTrigger = seqStepTriggered;
    } else if (currentRandomMode == RANDOM_MODE_PULSAR) {
      randomTrigger = pulsarTrigger;
    } else if (currentRandomMode == RANDOM_MODE_MIDI) {
      if (midiTriggerPending) {
        randomTrigger = true;
      }
    }

    if (randomTrigger) {
      // Fast random value generation
      currentRandomValue = (float)rand() * (1.0f / RAND_MAX);
    }

    // --- 4. ENVELOPE GENERATION ---
    bool gate = false;
    if (gateRemainingSamples > 0) {
      gate = true;
      gateRemainingSamples--;
    }

    float envSig = env.Process(gate);
    float pulsarEnvSig = pulsar.Process(pulsarTrigger);

    // --- 5. UPDATE MODULATION MATRIX (less frequently) ---
    // Use the GLOBAL modUpdateCounter, not a local static one
    if (++modUpdateCounter >= MOD_UPDATE_RATE) {
      updateMatrixModulation(envSig, pulsarEnvSig);
      modUpdateCounter = 0;
    }

    // --- 6. APPLY ATTENUATORS (using pre-calculated modulation values) ---
    float pot8Norm = potValues[8] * (1.0f / 65535.0f);
    pulsarPeriodModCoeff = rawModulationValues[0] * pot8Norm;
    modulationFreqCoeff = rawModulationValues[1] * modulationFreqModDepth;
    modulationOscModCoeff = rawModulationValues[2] * modulationOscModCoeffDepth;
    complexOscFreqCoeff = rawModulationValues[3] * complexOscFreqModDepth;
    complexOscWFCoeff = rawModulationValues[4] * complexOscWFModDepth;
    vcaComplexOsc = rawModulationValues[5] * foldedLpgModAmount;
    vcaModulationOsc = rawModulationValues[6] * modOscLpgModAmount;

    // --- 7. OSCILLATORS (optimized with fast approximations) ---

    // MODULATION OSCILLATOR
    float modFinalFreq;
    if (modulationMidiEnabled) {
      float rawNote = 24.0f + (potValues[27] * (72.0f / 65535.0f));
      int quantizedNote = (int)(rawNote + 0.5f);
      modFinalFreq = mtof(quantizedNote) * fast_exp2_approx((modulationFreqCoeff * 5.0f) + modulationFine);
    } else {
      modFinalFreq = modulationOscFreq * fast_exp2_approx((modulationFreqCoeff * 5.0f) + modulationFine);
    }
    modulationOsc.SetFreq(modFinalFreq);
    float modOscSig = modulationOsc.Process();

    float totalModDepth = modulationOscMod + (modulationOscModCoeff * 2.0f);
    if (totalModDepth < 0.0f) totalModDepth = 0.0f;
    if (totalModDepth > 2.0f) totalModDepth = 2.0f;

    float fmSignal = 0.0f;
    if (!useAmplitudeMod) {
      fmSignal = modOscSig * totalModDepth;
    }

    // COMPLEX OSCILLATOR
    float compFinalFreq;
    if (complexOscMidiEnabled) {
      float rawNote = 24.0f + (potValues[16] * (72.0f / 65535.0f));
      int quantizedNote = (int)(rawNote + 0.5f);
      float totalFreqMod = complexOscFreqCoeff + fmSignal;
      compFinalFreq = mtof(quantizedNote) * fast_exp2_approx(totalFreqMod * 5.0f + complexOscFine);
    } else {
      float totalFreqMod = complexOscFreqCoeff + fmSignal;
      compFinalFreq = complexOscFreq * fast_exp2_approx(totalFreqMod * 5.0f + complexOscFine);
    }

    complexOsc.SetFreq(compFinalFreq);
    complexOscMorph.SetFreq(compFinalFreq);

    float complexSig = complexOsc.Process();
    float morphSig = complexOscMorph.Process();

    float mixedSig = complexSig + (morphSig - complexSig) * complexOscMorphMix;

    if (complexOscInverted) {
      mixedSig = -mixedSig;
    }

    // --- 8. WAVEFOLDER & AM ---
    float wfGain = complexOscWF + (complexOscWFCoeff * 20.0f);
    if (wfGain < 1.0f) wfGain = 1.0f;
    if (wfGain > 20.0f) wfGain = 20.0f;

    waveFolder.SetGain(wfGain);
    float foldedSig = waveFolder.Process(mixedSig);

    if (useAmplitudeMod) {
      foldedSig *= (1.0f + (modOscSig * totalModDepth));
    }

    // --- 9. BUCHLA LPG MIXER (optimized) ---
    float finalMix = 0.0f;

    // Process folded signal (channel 0)
    float signal0 = foldedSig;
    float ctrl0 = complexOscSigLevel + vcaComplexOsc;
    if (ctrl0 < 0.0f) ctrl0 = 0.0f;
    if (ctrl0 > 1.0f) ctrl0 = 1.0f;

    float processedSig0 = signal0;
    if (foldedLpgMode == LPG_MODE_VCA) {
      processedSig0 *= ctrl0;
    } else {
      float filterFreq0 = 20.0f * powf(1000.0f, ctrl0);
      lpGateFilter1.SetFreq(filterFreq0);

      if (foldedLpgMode == LPG_MODE_LP) {
        lpGateFilter1.SetRes(ctrl0 * ctrl0 * 0.2f);
      } else {
        lpGateFilter1.SetRes(0.0f);
      }

      processedSig0 = lpGateFilter1.Process(processedSig0);

      if (foldedLpgMode == LPG_MODE_COMBI) {
        processedSig0 *= ctrl0;
      }
    }
    finalMix += processedSig0;

    // Process modulation oscillator signal (channel 1)
    float signal1 = modOscSig;
    float ctrl1 = modOscSigLevel + vcaModulationOsc;
    if (ctrl1 < 0.0f) ctrl1 = 0.0f;
    if (ctrl1 > 1.0f) ctrl1 = 1.0f;

    float processedSig1 = signal1;
    if (modOscLpgMode == LPG_MODE_VCA) {
      processedSig1 *= ctrl1;
    } else {
      float filterFreq1 = 20.0f * powf(1000.0f, ctrl1);
      lpGateFilter2.SetFreq(filterFreq1);

      if (modOscLpgMode == LPG_MODE_LP) {
        lpGateFilter2.SetRes(ctrl1 * ctrl1 * 0.2f);
      } else {
        lpGateFilter2.SetRes(0.0f);
      }

      processedSig1 = lpGateFilter2.Process(processedSig1);

      if (modOscLpgMode == LPG_MODE_COMBI) {
        processedSig1 *= ctrl1;
      }
    }
    finalMix += processedSig1;

    // --- 10. OUTPUT ANALOGUE FILTER ---
    finalMix = outputFilter.Process(finalMix);

    // --- 11. REVERB & OUTPUT ---
    float revL, revR;
    reverb.Process(finalMix, finalMix, &revL, &revR);

    float dryWet = 1.0f - reverbMix;
    out[0][i] = (finalMix * dryWet + revL * reverbMix) * 0.4f;
    out[1][i] = (finalMix * dryWet + revR * reverbMix) * 0.4f;
  }

  // -- CPU -- 
  //cpuMeter.OnBlockEnd();

}

// -- SETUP --
void setup() {
  Serial.begin(115200);
  delay(1000);  // Wait for serial to initialize

  Serial.println("=== DaisySeed Button & Potentiometer Monitor ===");
  Serial.println("Displaying button state changes and potentiometer values...");
  Serial.println();

  // -- INIT BUTTON MATRICES --
  initButtonMatrix();
  initButtonMatrix2();

  // -- INIT POTS --
  analogReadResolution(16);  // 16-BIT ADC
  initPotentiometers();

  Serial.println("Potentiometer Configuration:");
  Serial.println("MUX1: Channels 0-15 connected to A0");
  Serial.println("MUX2: Channels 0-15 connected to A1");
  Serial.println("Threshold for reporting changes: ±50 units");
  Serial.println("==============================================");

    // -- DAISY SEED INIT AT 48kHz --
  float sample_rate;
  hw = DAISY.init(DAISY_SEED, AUDIO_SR_48K);
  sample_rate = DAISY.get_samplerate();
  int block_size = 128;  // Get the actual block size
  DAISY.SetAudioBlockSize(block_size);

  // -- CPU DEBUG --
  //cpuMeter.Init(sample_rate, block_size);
  // -- START AUDIO
  DAISY.begin(AudioCallback);

  // -- DAISY DUINO INIT --
  // -- COMPLEX OSC // SINE + MORPH --
  complexOsc.Init(sample_rate);
  complexOsc.SetWaveform(complexOsc.WAVE_SIN);
  complexOsc.SetFreq(440.0f);
  complexOsc.SetAmp(1.0f);
  complexOscMorph.Init(sample_rate);
  complexOscMorph.SetWaveform(complexOsc.WAVE_SAW);
  complexOscMorph.SetFreq(440.0f);
  complexOscMorph.SetAmp(1.0f);
  // -- WAVEFOLDER --
  waveFolder.Init();
  // -- MODULATION OSC --
  modulationOsc.Init(sample_rate);
  modulationOsc.SetWaveform(modulationOsc.WAVE_SIN);
  modulationOsc.SetFreq(440.0f);
  modulationOsc.SetAmp(1.0f);
  // -- LPG FILTER --
  lpGateFilter1.Init(sample_rate);
  lpGateFilter2.Init(sample_rate);
  // -- ENVELOPE --
  env.Init(sample_rate);
  // -- PULSAR ENV --
  pulsar.Init(sample_rate);
  pulsar.SetAttackTime(0.02f);
  pulsar.SetDecayTime(0.02f);
  pulsar.SetSustainLevel(1.0f);
  // -- ANALOGUE OUTPUT FILTER --
  outputFilter.Init(sample_rate);
  outputFilter.SetFreq(15000.0f);
  outputFilter.SetRes(0.3f);
  // -- REVERB --
  reverb.Init(sample_rate);
  reverb.SetFeedback(0.85f);
  reverb.SetLpFreq(17000.0f);

  // -- MIDI INIT --
  // Configure Serial1 to use pin D14 for RX
  Serial1.setRx(MIDI_RX_PIN);
  Serial1.begin(31250);  // Standard MIDI baud rate
  MIDI.setHandleNoteOn(handleNoteOn);
  MIDI.setHandleNoteOff(handleNoteOff);
  MIDI.begin(MIDI_CHANNEL_OMNI);
  MIDI.turnThruOff();
  currentMidiNote = 60;
  midiTriggerPending = false;

  Serial.println("=============================================");
  Serial.println("MIDI initialized with pin 30 (Digital pin 30)");
  Serial.println("=============================================");
}

// -- LOOP --
void loop() {

  // MIDI PROCESSING
  MIDI.read();  // Call once per loop - the callbacks will handle the events

  static bool lastMidiTriggerState = false;
  if (midiTriggerPending) {
    lastMidiTriggerState = true;
  } else {
    lastMidiTriggerState = false;
  }

  // READ BUTTON MATRICES
  if (millis() - lastMatrixRead > MATRIX_READ_INTERVAL) {
    readButtonMatrix();
    readButtonMatrix2();
    printButtonChanges();  //BUTTON FNCs
    lastMatrixRead = millis();
  }

  // READ POTENTIOMETERS AND UPDATE PARAMETERS
  if (millis() - lastPotRead > POT_READ_INTERVAL) {
    readPotentiometers();
    updateParameters();  // UPDATE PARAMS
    lastPotRead = millis();
  }

  // CPU & DEBUG PRINTING (Runs every 0.5 seconds)
  /*
  if (millis() - lastCpuPrint >= 500) {
    lastCpuPrint = millis();

    // CPU LOAD REPORTING
    float avgLoad = cpuMeter.GetAvgCpuLoad();
    float maxLoad = cpuMeter.GetMaxCpuLoad();

    Serial.print("CPU Load: ");
    Serial.print(avgLoad * 100.0f);
    Serial.print("% | Max: ");
    Serial.print(maxLoad * 100.0f);
    Serial.println("%");
  }*/
}

// -- UPDATE PARAMETERS --
void updateParameters() {
  // -- OSCILLATORS --

  // FINE TUNE
  // 3 cents is 3/1200 of an octave.
  float fineRange = 3.0f / 12.0f;

  // -- COMPLEX OSCILLATOR (25Hz to 4000Hz) --
  // PITCH
  float complexOscFreqPot = potValues[16] / 65535.0f;
  // LOGARITHMIC MAPPING: 20Hz to 8000Hz (6 octaves)
  // Use exact powf for smooth, accurate response
  float freqMin = 20.0f;    // Low C (~32.7Hz) but slightly lower for sub-bass
  float freqMax = 3560.0f;  // ~8kHz, 8 octaves above 31.25Hz
  complexOscFreq = freqMin * powf(freqMax / freqMin, complexOscFreqPot);

  // Pot 15: Complex Osc Frequency Modulation Amount
  complexOscFreqModDepth = potValues[15] / 65535.0f;

  // Pot 17: Fine Tune Complex Oscillator (+/- 3 cents)
  complexOscFine = ((potValues[17] / 65535.0f) * 2.0f - 1.0f) * fineRange;

  // TIMBRE/MORPH CONTROL
  complexOscMorphMix = potValues[20] / 65535.0f;

  // WAVEFOLDER (0.0 to 1.0f)
  float complexOscWFPot = potValues[19] / 65535.0f;
  complexOscWF = 1.0f + (19.0f * complexOscWFPot);

  // Pot 18: Wavefolder Modulation Amount
  complexOscWFModDepth = potValues[18] / 65535.0f;

  // LPG
  // Pot 21: Amount of modulation for Folded Signal LPG
  foldedLpgModAmount = potValues[21] / 65535.0f;

  // Pot 22: Channel 1 Sig Level
  complexOscSigLevel = potValues[22] / 65535.0f;

  // -- MODULATION OSCILLATOR (1Hz to 2000Hz) --
  // PITCH
  float modulationOscFreqPot = potValues[11] / 65535.0f;
  float modMin = 0.1f;
  float modMax = 1760.0f;
  modulationOscFreq = modMin * powf(modMax / modMin, modulationOscFreqPot);

  // Pot 10: Amount of modulation applied to Mod Osc Frequency
  modulationFreqModDepth = potValues[10] / 65535.0f;

  // Pot 12: Fine Tune Modulation Oscillator (+/- 3 cents)
  modulationFine = ((potValues[12] / 65535.0f) * 2.0f - 1.0f) * fineRange;

  // MODULATION (FM/AM)
  // Pot 14: Modulation Depth (FM or AM depth)
  modulationOscMod = potValues[14] / 65535.0f;

  // Pot 13: Amount of modulation applied to Mod Osc Modulation Depth (FM/AM amount)
  modulationOscModCoeffDepth = potValues[13] / 65535.0f;

  // LPG
  // Pot 23: Amount of modulation for Mod Osc Signal LPG
  modOscLpgModAmount = potValues[23] / 65535.0f;

  // Pot 24: Channel 2 Sig Level
  modOscSigLevel = potValues[24] / 65535.0f;

  // -- ENVELOPE [Buchla ASD config] --
  float minTime = 0.02f;
  float maxTime = 10.0f;
  float timeRatio = maxTime / minTime;  // 500.0f

  // Attack Time (Logarithmic)
  float attackPot = potValues[5] / 65535.0f;
  attackTime = minTime * fast_pow_approx(timeRatio, attackPot);
  env.SetAttackTime(attackTime);

  // Sustain Duration (Logarithmic)
  float sustainDurationPot = potValues[6] / 65535.0f;
  sustainDuration = minTime * fast_pow_approx(timeRatio, sustainDurationPot);

  // Release Time (Logarithmic)
  float releasePot = potValues[7] / 65535.0f;
  releaseTime = minTime * fast_pow_approx(timeRatio, releasePot);
  env.SetReleaseTime(releaseTime);
  env.SetDecayTime(0.0f);
  env.SetSustainLevel(1.0f);

  // -- PULSAR --
  // Pot 9: Base Pulsar Release Time
  float pulsarReleasePot = potValues[9] / 65535.0f;

  // Logarithmic mapping: 20ms to 5000ms (0.02s to 5.0s)
  float pulsarMinTime = 0.02f;  // 20ms - very short decay
  float pulsarMaxTime = 5.0f;   // 5000ms - long decay
  float pulsarTimeRatio = pulsarMaxTime / pulsarMinTime;

  // Base release time from pot (logarithmic)
  float basePulsarRelease = pulsarMinTime * powf(pulsarTimeRatio, pulsarReleasePot);

  // Subtract modulation (clamped) to ensure time decreases
  float adjustedPulsarTime = basePulsarRelease - (2.0f * pulsarPeriodModCoeff);  // Scale modulation appropriately
  if (adjustedPulsarTime < pulsarMinTime) adjustedPulsarTime = pulsarMinTime;
  if (adjustedPulsarTime > pulsarMaxTime) adjustedPulsarTime = pulsarMaxTime;

  pulsarReleaseTime = adjustedPulsarTime;
  pulsar.SetReleaseTime(pulsarReleaseTime);

  // -- SEQUENCER --
  // Update Sequencer CV steps from MUX1 Channels 0-4
  for (int i = 0; i < 5; i++) {
    // Store normalized 0.0 to 1.0 values
    seqStepCV[i] = potValues[i] / 65535.0f;
  }

  // -- CLOCK --
  float clockPot = potValues[25] / 65535.0f;
  seqClockSpeed = 1.0f * powf(2000.0f, (1.0f - clockPot));  // Use exact powf

  // CLOCK CLAMPING
  if (seqClockSpeed < 0.5f) seqClockSpeed = 0.5f;
  if (seqClockSpeed > 2000.0f) seqClockSpeed = 2000.0f;

  // -- REVERB --
  reverbMix = potValues[26] / 65535.0f;  // REVERB MIX CONTROL
}

// -- MIDI NOTE HANDLING
void handleNoteOn(byte channel, byte note, byte velocity) {
  if (velocity > 0) {
    currentMidiNote = note;
    midiTriggerPending = true;
    midiTriggerCounter = MIDI_TRIGGER_HOLDOFF;  // Set holdoff counter
  }
}

void handleNoteOff(byte channel, byte note, byte velocity) {
  // You can handle note off if needed
  // Serial.print("MIDI Note OFF: ");
  // Serial.println(note);
}