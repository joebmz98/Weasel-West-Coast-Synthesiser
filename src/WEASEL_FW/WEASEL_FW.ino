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
#include <MIDI.h> // MIDI 
#include "wavefolder.h" // EXTRACTED FROM DAISYSP

// MUX PINS - FIRST MUX
#define MUX1_S0 0
#define MUX1_S1 1
#define MUX1_S2 2
#define MUX1_S3 3
#define MUX1_SIG A0  // MUX signal pin

// MUX PINS - SECOND MUX
#define MUX2_S0 4
#define MUX2_S1 5
#define MUX2_S2 6
#define MUX2_S3 7
#define MUX2_SIG A1  // Second MUX signal pin

// NEW 4x7 BUTTON MATRIX PINS
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

// SECOND BUTTON MATRIX PINS
#define MATRIX2_COL0 12  // X0
#define MATRIX2_COL1 13  // X1
#define MATRIX2_COL2 17  // X2
#define MATRIX2_COL3 18  // X3
#define MATRIX2_ROW0 8   // Y0
#define MATRIX2_ROW1 9   // Y1
#define MATRIX2_ROW2 10  // Y2
#define MATRIX2_ROW3 11  // Y3
#define MATRIX2_ROW4 14  // Y4

// BUTTON MATRIX 1 VARIABLES
bool matrixStates[4][7] = { { false } };
bool lastMatrixStates[4][7] = { { false } };
unsigned long lastMatrixRead = 0;
const unsigned long MATRIX_READ_INTERVAL = 10;

// BUTTON MATRIX 2 - SIMPLIFIED without debounce for testing
bool matrix2CurrentStates[4][5] = { { false } };
bool matrix2PreviousStates[4][5] = { { false } };
bool matrix2DebouncedStates[4][5] = { { false } };
unsigned long lastMatrix2Read = 0;

// POT VARIABLES
const int NUM_POTS_PER_MUX = 16;
const int TOTAL_POTS = 27;  // 2 muxes * 16 channels each
int potValues[TOTAL_POTS] = { 0 };
int lastPotValues[TOTAL_POTS] = { 0 };
unsigned long lastPotRead = 0;
const unsigned long POT_READ_INTERVAL = 1;  // Read pots every Xms
String potNames[TOTAL_POTS];                // Array to store pot names
const int POT_THRESHOLD = 100;              // 

// DAISY DUINO OBJECTS
DaisyHardware hw;          // DAISY SEED
  // OSCILLATORS
  Oscillator complexOsc;      // COMPLEX OSCILLATOR
  Oscillator complexOscMorph; // COMPLEX MORPH OSCILLATOR 
  Oscillator modulationOsc;   // COMPLEX OSCILLATOR
  // ENVELOPE GENERATOR
  Adsr env;                  // ENV GENERATOR
  // PULSAR GENERATOR
  Adsr pulsar;               // ENV GENERATOR FOR PULSAR
  // LOW-PASS - LPG
  MoogLadder lpGateFilter1;   // FILTER FOR LPG CHAN1
  MoogLadder lpGateFilter2;   // FILTER FOR LPG CHAN2
  // OUTPUT ANALOGUE FILTER
  MoogLadder outputFilter;
  // REVERB
  ReverbSc reverb;           // REVERB
  // WAVEFOLDER
  daisysp::Wavefolder waveFolder;


// OBJECT VARIABLES
// OSCILLATORS
// COMPLEX
float complexOscFreq;                // CONTROLS FREQUENCY
float complexOscFine;                // CONTROLS FINE TUNE
float complexOscFreqCoeff;           // COEFF FOR FREQUENCY MODULATION USING MATRIX
float complexOscWF;                  // CONTROLS WF AMOUNT
float complexOscWFCoeff;             // COEFF FOR WF MODULATION USING MATRIX
float complexOscWFModDepth = 0.0f;   // WAVEFOLDER MOD DEPTH
float complexOscFreqModDepth = 0.0f; // FREQ MOD DEPTH
bool complexOscInverted = false;     // TOGGLE POLARITY
bool complexOscMidiEnabled = false;  // TOGGLE MIDI
float complexOscMidiFreq = 0.0f;     // FREQUENCY OFFSET FROM MIDI
float complexOscMorphMix;            // COMPLEX OSC TIMBRE AMOUNT
int morphWaveformIndex = 0;          // 0 = SAW, 1 = SQUARE, 2 = TRIANGLE
float complexOscSigLevel = 0.0f;     // LPG CONTROL

// MODULATION
float modulationOscFreq = 0.0f;             // CONTROLS FREQUENCY
float modulationFine = 0.0f;                // CONTROLS FINE TUNE
float modulationFreqCoeff = 0.0f;           // COEFF FOR FREQUENCY MODULATION USING MATRIX
float modulationOscMod = 0.0f;              // CONTROLS MOD AMOUNT APPLIED TO COMPLEXOSC
float modulationOscModCoeff;         // COEFF FOR MOD AMOUNT MODULATION USING MATRIX
float modulationFreqModDepth = 0.0f;    // FREQ MOD DEPTH
float modulationOscModCoeffDepth = 0.0f; // MODULATION MOD DEPTH
bool useAmplitudeMod = false;        // False = FM, True = AM
bool modulationMidiEnabled = false;  // TOGGLE MIDI
float modulationOscMidiFreq = 0.0f;  // FREQUENCY OFFSET FROM MIDI
int modWaveformIndex = 0;            // 0: SIN, 1: TRI, 2: SQUARE, 3: SAW
float modOscSigLevel = 0.0f;         // LPG CONTROL

// LPG
float vcaComplexOsc;     // AMPLITUDE COEFF FOR COMPLEXOSC
float vcaModulationOsc;  // AMPLITUDE COEFF FOR MODULATIONOSC
float lpComplexOsc;      // FREQ COEFF FOR COMPLEXOSC
float lpModulationOsc;   // FREQ COEFF FOR MODULATIONOSC
float foldedLpgModAmount = 0.0f; // Multiplier for Folded LPG mod
float modOscLpgModAmount = 0.0f; // Multiplier for Mod Osc LPG mod
enum LpgMode { LPG_MODE_VCA, LPG_MODE_LP, LPG_MODE_COMBI };
LpgMode foldedLpgMode = LPG_MODE_COMBI; // Default to COMBI mode
LpgMode modOscLpgMode = LPG_MODE_COMBI; // Default to COMBI mode

// ENVELOPE [BUCHLA ASD/ASR]
float attackTime = 0.0f; 
float releaseTime = 0.0f;
float sustainDuration = 0.0f;

// PULSAR ENV
enum PulsarMode { PULSAR_MODE_SEQ, PULSAR_MODE_MIDI, PULSAR_MODE_OSC };
PulsarMode currentPulsarMode = PULSAR_MODE_SEQ;
float pulsarReleaseTime = 0.02f;
float pulsarEnvSig = 0.0f;
bool lastPulsarEnvActive = false;
float pulsarPeriodModCoeff = 0.0f;

// SAMPLE&HOLD / RANDOM VOLTAGE
enum RandomMode { RANDOM_MODE_SEQ, RANDOM_MODE_PULSAR, RANDOM_MODE_MIDI };
RandomMode currentRandomMode = RANDOM_MODE_SEQ;
float currentRandomValue = 0.0f;

// REVERB
float reverbMix;  // REVERB MIX COEFF

// MIDI 
#define MIDI_RX_PIN 30  // USART1 Rx (Digital pin 30)
  // MIDI OBJECT
  MIDI_CREATE_INSTANCE(HardwareSerial, Serial1, MIDI);

// SEQUENCER 
int   seqCurrentStep = 0;
bool  seqStepEnabled[5] = {true, true, true, true, true}; // All steps ON by default
float seqStepCV[5] = {0.0f};                              // Stores current CV from MUX1 0-4
float seqClockSpeed = 100.0f;
int seqMaxSteps = 5; // Default sequence length
  // AUDIO INTERRUPT TIMING
  uint32_t seqSampleCounter = 0;    // Counts samples to trigger the next step
  float    activeSeqCV = 0.0f;      // The CV of the currently active step
  // SEQUENCER MODES
  enum SeqTriggerMode { SEQ_TRIGGER_CLOCK, SEQ_TRIGGER_PULSAR, SEQ_TRIGGER_MIDI };
  SeqTriggerMode currentSeqTriggerMode = SEQ_TRIGGER_CLOCK; // Default Mode
  bool midiTriggerPending = false; // Flag to catch MIDI notes for the sequencer


// MUX control function
void selectMuxChannel(int channel, int s0, int s1, int s2, int s3) {
  digitalWrite(s0, bitRead(channel, 0));
  digitalWrite(s1, bitRead(channel, 1));
  digitalWrite(s2, bitRead(channel, 2));
  digitalWrite(s3, bitRead(channel, 3));
}

// Initialize potentiometer monitoring
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

  // Initialize pot names - you can customize these based on your setup
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
  readPotentiometers();
  // Copy to last values to avoid initial spurious changes
  for (int i = 0; i < TOTAL_POTS; i++) {
    lastPotValues[i] = potValues[i];
  }
}

// Read all potentiometers
void readPotentiometers() {
  for (int i = 0; i < TOTAL_POTS; i++) {
    if (i < NUM_POTS_PER_MUX) {
      
      // Read from MUX1
      selectMuxChannel(i, MUX1_S0, MUX1_S1, MUX1_S2, MUX1_S3);
      delayMicroseconds(20);  // Small delay for MUX to settle
      potValues[i] = analogRead(MUX1_SIG);
    } else {
      
      // Read from MUX2 (channels 16-31)
      int mux2Channel = i - NUM_POTS_PER_MUX;
      selectMuxChannel(mux2Channel, MUX2_S0, MUX2_S1, MUX2_S2, MUX2_S3);
      delayMicroseconds(20);  // Small delay for MUX to settle
      potValues[i] = analogRead(MUX2_SIG);
    }
  }
}

// Check for potentiometer changes and print them
void printPotentiometerChanges() {
  bool anyPotChange = false;
  float fullScale = 65535.0f; // Since you set resolution to 16-bit

  for (int i = 0; i < TOTAL_POTS; i++) {
    // Calculate the difference as a percentage of the full range
    float difference = abs(potValues[i] - lastPotValues[i]);
    float percentChange = difference / fullScale;

    // Only print if the change is 2% (0.02) or greater
    if (percentChange >= 0.02f) {
      anyPotChange = true;

      String muxIdentifier = (i < NUM_POTS_PER_MUX) ? "MUX1" : "MUX2";
      int channel = (i < NUM_POTS_PER_MUX) ? i : i - NUM_POTS_PER_MUX;

      // Print the change
      Serial.print("POT - ");
      Serial.print(muxIdentifier);
      Serial.print(" CH");
      if (channel < 10) Serial.print("0");
      Serial.print(channel);
      Serial.print(" [");
      Serial.print(potNames[i]);
      Serial.print("]: ");
      
      // Print raw value and calculated percentage
      Serial.print(potValues[i]);
      Serial.print(" (");
      Serial.print((potValues[i] / fullScale) * 100.0f, 1); // 1 decimal place
      Serial.print("%)");

      // Show direction
      if (potValues[i] > lastPotValues[i]) Serial.println(" ▲");
      else Serial.println(" ▼");

      // Update last value to the new "anchor" point
      lastPotValues[i] = potValues[i];
    }
  }

  if (anyPotChange) {
    Serial.println();
  }
}

// Button matrix initialization
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

// Read button matrices
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

void readButtonMatrix2() {
  int colPins[] = {MATRIX2_COL0, MATRIX2_COL1, MATRIX2_COL2, MATRIX2_COL3};
  int rowPins[] = {MATRIX2_ROW0, MATRIX2_ROW1, MATRIX2_ROW2, MATRIX2_ROW3, MATRIX2_ROW4};

  for (int col = 0; col < 4; col++) {
    // 1. ACTIVATE: Set column to OUTPUT and HIGH to source current
    pinMode(colPins[col], OUTPUT);
    digitalWrite(colPins[col], HIGH);
    
    // Give Pin 13 (LED) extra time to overcome the resistor/LED load
    delayMicroseconds(80); 

    for (int row = 0; row < 5; row++) {
      matrix2PreviousStates[col][row] = matrix2CurrentStates[col][row];
      // Reading HIGH means button is pressed
      matrix2CurrentStates[col][row] = (digitalRead(rowPins[row]) == HIGH);
    }

    // 2. DEACTIVATE: Instead of digitalWrite(LOW), set to INPUT
    // This puts the pin in High-Impedance mode, "unplugging" the LED circuit
    pinMode(colPins[col], INPUT); 
  }
}

// Print button changes with detailed coordinate information
void printButtonChanges() {
  bool anyChange = false;

  // Check Matrix 1 (4x7) - B0-B3 + A0-A6
  for (int col = 0; col < 4; col++) {
    for (int row = 0; row < 7; row++) {
      if (matrixStates[col][row] != lastMatrixStates[col][row]) {
        anyChange = true;
        // Create button coordinate string
        String buttonCoord = "B" + String(col) + "+A" + String(row);
        String function = "";

        // Add function description based on position
        if (col == 0) {
          if (row == 0) function = " (Seq CV Pulsar Env Decay)";
          else if (row == 1) function = " (Seq CV ModOsc Pitch)";
          else if (row == 2) function = " (Seq CV Mod Amount)";
          else if (row == 3) function = " (Seq CV ComplexOsc Pitch)";
          else if (row == 4) function = " (Seq CV Wavefolder Mod)";
          else if (row == 5) function = " (Seq CV LPG Ch1 Level)";
          else if (row == 6) function = " (Seq CV LPG Ch2 Level)";
        } else if (col == 1) {
          if (row == 0) function = " (Env Mod Pulsar Decay)";
          else if (row == 1) function = " (Env Mod ModOsc Pitch)";
          else if (row == 2) function = " (Env Mod Mod Amount)";
          else if (row == 3) function = " (Env Mod ComplexOsc Pitch)";
          else if (row == 4) function = " (Wavefolder Env Mod)";
          else if (row == 5) function = " (Env Mod Ch1)";
          else if (row == 6) function = " (Env Mod Ch2)";
        } else if (col == 2) {
          if (row == 0) function = " (Pulsar Self-Mod)";
          else if (row == 1) function = " (Pulsar ModOsc Pitch)";
          else if (row == 2) function = " (Pulsar Mod Amount)";
          else if (row == 3) function = " (Pulsar ComplexOsc Pitch)";
          else if (row == 4) function = " (Pulsar Wavefolder Mod)";
          else if (row == 5) function = " (Pulsar LPG Ch1 Mod)";
          else if (row == 6) function = " (Pulsar LPG Ch2 Mod)";
        }

        Serial.print("Matrix 1 - ");
        Serial.print(buttonCoord);
        Serial.print(" [");
        Serial.print(col);
        Serial.print("][");
        Serial.print(row);
        Serial.print("]");
        Serial.print(function);
        Serial.print(": ");
        Serial.println(matrixStates[col][row] ? "PRESSED" : "RELEASED");
      }
    }
  }

  // Check Matrix 2 (4x5) - XY Matrix
  for (int col = 0; col < 4; col++) {
    for (int row = 0; row < 5; row++) {
      if (matrix2CurrentStates[col][row] != matrix2PreviousStates[col][row]) {
        anyChange = true;
        String buttonCoord = "X" + String(col) + "+Y" + String(row);
        String buttonName = "";
        String function = "";

        // Corrected Mapping
        if (col == 1 && row == 0) {
          buttonName = "SEQUENCER_MODE_BUTTON";
          if (matrix2CurrentStates[col][row] && !matrix2PreviousStates[col][row]) {
            // Cycle: 0 -> 1 -> 2 -> 0
            currentSeqTriggerMode = static_cast<SeqTriggerMode>((currentSeqTriggerMode + 1) % 3);

            Serial.print("Sequencer Trigger Mode changed to: ");
            if (currentSeqTriggerMode == SEQ_TRIGGER_CLOCK) Serial.println("INTERNAL CLOCK");
            else if (currentSeqTriggerMode == SEQ_TRIGGER_PULSAR) Serial.println("PULSAR ENVELOPE");
            else if (currentSeqTriggerMode == SEQ_TRIGGER_MIDI) Serial.println("MIDI NOTE ON");
          }
        } else if (col == 1 && row == 1) {
          buttonName = "MODULATION_TOGGLE_BUTTON";
          function = " (MODULATION TOGGLE)";
        } else if (col == 1 && row == 2) {
          buttonName = "MIDI_TOGGLE";
          function = " (Toggle MIDI pitch for Complex Osc)";
          if (matrix2CurrentStates[col][row] && !matrix2PreviousStates[col][row]) {
            complexOscMidiEnabled = !complexOscMidiEnabled;
            modulationMidiEnabled = !modulationMidiEnabled;
          }
        } else if (col == 1 && row == 3) {
          buttonName = "RANDOM_MODE_TOGGLE";
          if (matrix2CurrentStates[col][row] && !matrix2PreviousStates[col][row]) {
            currentRandomMode = static_cast<RandomMode>((currentRandomMode + 1) % 3);

            Serial.print("Random Mode: ");
            if (currentRandomMode == RANDOM_MODE_SEQ) Serial.println("SEQUENCER");
            else if (currentRandomMode == RANDOM_MODE_PULSAR) Serial.println("PULSAR");
            else Serial.println("MIDI");
          }
        } else if (col == 3 && row == 1) {
          buttonName = "FM/AM_TOGGLE";
          function = " (Toggle FM/AM modulation)";
          if (matrix2CurrentStates[col][row] && !matrix2PreviousStates[col][row]) {
            useAmplitudeMod = !useAmplitudeMod;
          }
        } else if (col == 3 && row == 2) {
          buttonName = "LPG_MODE_TOGGLE";
          function = " (Cycle LPG Mode: VCA -> LP -> COMBI)";

          if (matrix2CurrentStates[col][row] && !matrix2PreviousStates[col][row]) {
            foldedLpgMode = static_cast<LpgMode>((foldedLpgMode + 1) % 3);

            Serial.print("LPG Mode set to: ");
            if (foldedLpgMode == LPG_MODE_VCA) Serial.println("VCA");
            else if (foldedLpgMode == LPG_MODE_LP) Serial.println("LP");
            else Serial.println("COMBI");
          }
        } else if (col == 0 && row == 3) {
          buttonName = "MODOSC_LPG_MODE";
          if (matrix2CurrentStates[col][row] && !matrix2PreviousStates[col][row]) {
            modOscLpgMode = static_cast<LpgMode>((modOscLpgMode + 1) % 3);
            Serial.print("ModOsc LPG Mode: ");
            Serial.println(modOscLpgMode == LPG_MODE_VCA ? "VCA" : modOscLpgMode == LPG_MODE_LP ? "LP"
                                                                                                : "COMBI");
          }
        } else if (col == 2 && row == 1) {
          buttonName = "MOD_WAVE_CYCLE";
          function = " (Cycle Modulation Waveform)";

          // Toggle on Press (Rising Edge)
          if (matrix2CurrentStates[col][row] && !matrix2PreviousStates[col][row]) {
            modWaveformIndex = (modWaveformIndex + 1) % 4;  // Cycle 0, 1, 2, 3

            switch (modWaveformIndex) {
              case 0:
                modulationOsc.SetWaveform(Oscillator::WAVE_SIN);
                Serial.println("MOD WAVE: SINE");
                break;
              case 1:
                modulationOsc.SetWaveform(Oscillator::WAVE_TRI);
                Serial.println("MOD WAVE: TRIANGLE");
                break;
              case 2:
                modulationOsc.SetWaveform(Oscillator::WAVE_SQUARE);
                Serial.println("MOD WAVE: SQUARE");
                break;
              case 3:
                modulationOsc.SetWaveform(Oscillator::WAVE_SAW);
                Serial.println("MOD WAVE: SAWTOOTH");
                break;
            }
          }
        } else if (col == 0 && row == 2) {
          buttonName = "OSC_POLARITY_INVERT";
          function = " (Toggle Phase Inversion)";

          if (matrix2CurrentStates[col][row] && !matrix2PreviousStates[col][row]) {
            complexOscInverted = !complexOscInverted;
            Serial.print("POLARITY: ");
            Serial.println(complexOscInverted ? "INVERTED (-)" : "NORMAL (+)");
          }
        } else if (col == 2 && row == 2) {
          buttonName = "MORPH_WAVE_TOGGLE";
          function = " (Cycle Morph Waveform)";

          // Toggle on Press (Rising Edge)
          if (matrix2CurrentStates[col][row] && !matrix2PreviousStates[col][row]) {
            morphWaveformIndex = (morphWaveformIndex + 1) % 3;  // Cycle 0, 1, 2

            switch (morphWaveformIndex) {
              case 0:
                complexOscMorph.SetWaveform(Oscillator::WAVE_SAW);
                Serial.println("MORPH WAVE: SAW");
                break;
              case 1:
                complexOscMorph.SetWaveform(Oscillator::WAVE_SQUARE);
                Serial.println("MORPH WAVE: SQUARE");
                break;
              case 2:
                complexOscMorph.SetWaveform(Oscillator::WAVE_TRI);
                Serial.println("MORPH WAVE: TRIANGLE");
                break;
            }
          }
          // Step 1: X3 Y3
        } else if (col == 3 && row == 3) {
          buttonName = "SEQ_STEP_1_TOGGLE";
          // If button is PRESSED, step is DISABLED (false)
          seqStepEnabled[0] = !matrix2CurrentStates[col][row];

          if (matrix2CurrentStates[col][row] != matrix2PreviousStates[col][row]) {
            Serial.print("Step 1: ");
            Serial.println(seqStepEnabled[0] ? "ON" : "OFF (Button High)");
          }
        }
        // Step 2: X0 Y4
        else if (col == 0 && row == 4) {
          buttonName = "SEQ_STEP_2_TOGGLE";
          seqStepEnabled[1] = !matrix2CurrentStates[col][row];
          if (matrix2CurrentStates[col][row] != matrix2PreviousStates[col][row]) {
            Serial.print("Step 2: ");
            Serial.println(seqStepEnabled[1] ? "ON" : "OFF (Button High)");
          }
        }
        // Step 3: X1 Y4
        else if (col == 1 && row == 4) {
          buttonName = "SEQ_STEP_3_TOGGLE";
          seqStepEnabled[2] = !matrix2CurrentStates[col][row];
          if (matrix2CurrentStates[col][row] != matrix2PreviousStates[col][row]) {
            Serial.print("Step 3: ");
            Serial.println(seqStepEnabled[2] ? "ON" : "OFF (Button High)");
          }
        }
        // Step 4: X2 Y4
        else if (col == 2 && row == 4) {
          buttonName = "SEQ_STEP_4_TOGGLE";
          seqStepEnabled[3] = !matrix2CurrentStates[col][row];
          if (matrix2CurrentStates[col][row] != matrix2PreviousStates[col][row]) {
            Serial.print("Step 4: ");
            Serial.println(seqStepEnabled[3] ? "ON" : "OFF (Button High)");
          }

          // Step 5: X3 Y4
        } else if (col == 3 && row == 4) {
          buttonName = "SEQ_STEP_5_TOGGLE";
          seqStepEnabled[4] = !matrix2CurrentStates[col][row];
          if (matrix2CurrentStates[col][row] != matrix2PreviousStates[col][row]) {
            Serial.print("Step 5: ");
            Serial.println(seqStepEnabled[4] ? "ON" : "OFF (Button High)");
          }
        } else if (col == 0 && row == 0) {
          buttonName = "SEQ_LENGTH_CYCLE";
          function = " (Cycle sequence length 5->4->3)";

          // Cycle on Press (Rising Edge)
          if (matrix2CurrentStates[col][row] && !matrix2PreviousStates[col][row]) {
            if (seqMaxSteps == 5) seqMaxSteps = 4;
            else if (seqMaxSteps == 4) seqMaxSteps = 3;
            else seqMaxSteps = 5;

            // Ensure current step isn't out of bounds after shortening
            seqCurrentStep %= seqMaxSteps;

            Serial.print("Sequence Length changed to: ");
            Serial.println(seqMaxSteps);
          }
        } else if (col == 0 && row == 1) {
          buttonName = "PULSAR_MODE_TOGGLE";
          function = " (Cycle Pulsar Trigger Mode)";

          if (matrix2CurrentStates[col][row] && !matrix2PreviousStates[col][row]) {
            currentPulsarMode = static_cast<PulsarMode>((currentPulsarMode + 1) % 3);

            Serial.print("Pulsar Mode: ");
            if (currentPulsarMode == PULSAR_MODE_SEQ) Serial.println("SEQUENCER");
            else if (currentPulsarMode == PULSAR_MODE_MIDI) Serial.println("MIDI");
            else Serial.println("OSCILLATING");
          }
        } else {
          buttonName = "BUTTON_" + String(col) + "_" + String(row);
          function = " (Unassigned)";
        }

        Serial.print("Matrix 2 - ");
        Serial.print(buttonCoord);
        Serial.print(" [");
        Serial.print(buttonName);
        Serial.print("]");
        Serial.print(" Status: ");
        Serial.println(matrix2CurrentStates[col][row] ? "PRESSED" : "RELEASED");
      }
    }
  }

  //
  if (anyChange) {
    Serial.println();
  }
}

// AUDIO PROCESSING
void AudioCallback(float** in, float** out, size_t size) {
    float sampleRate = DAISY.get_samplerate();
    uint32_t samplesPerTick = (uint32_t)((seqClockSpeed / 1000.0f) * sampleRate);

    for (size_t i = 0; i < size; i++) {
        // --- 1. TRIGGERS & SEQUENCER ---
        bool seqStepTriggered = false;

        if (currentSeqTriggerMode == SEQ_TRIGGER_CLOCK) {
            seqSampleCounter++;
            if (seqSampleCounter >= samplesPerTick) { 
                seqStepTriggered = true;
                seqSampleCounter = 0;
            }
        } 
        else if (currentSeqTriggerMode == SEQ_TRIGGER_PULSAR) {
            bool isPulsarActive = pulsar.IsRunning();
            if (isPulsarActive && !lastPulsarEnvActive) {
                seqStepTriggered = true;
            }
            lastPulsarEnvActive = isPulsarActive;
        } 
        else if (currentSeqTriggerMode == SEQ_TRIGGER_MIDI) {
            if (midiTriggerPending) {
                seqStepTriggered = true;
                midiTriggerPending = false; 
            }
        }

        if (seqStepTriggered) {
            seqCurrentStep = (seqCurrentStep + 1) % seqMaxSteps;
            if (seqStepEnabled[seqCurrentStep]) { 
                activeSeqCV = seqStepCV[seqCurrentStep];
                env.Retrigger(false);
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
        }

        if (pulsarTrigger) {
            pulsar.Init(sampleRate);
            pulsar.SetAttackTime(0.02f);
            pulsar.SetDecayTime(0.02f);
            pulsar.SetSustainLevel(1.0f);
            pulsar.SetReleaseTime(pulsarReleaseTime);
            pulsar.Retrigger(false);
        }

        // --- 3. RANDOM TRIGGER LOGIC ---
        bool randomTrigger = false;
        if (currentRandomMode == RANDOM_MODE_SEQ) {
            randomTrigger = seqStepTriggered;
        } else if (currentRandomMode == RANDOM_MODE_PULSAR) {
            randomTrigger = pulsarTrigger;
        }
        if (randomTrigger) {
            currentRandomValue = (float)rand() / (float)RAND_MAX;
        }

        // --- 4. ENVELOPE GENERATION ---
        uint32_t gateSamples = (uint32_t)(sustainDuration * sampleRate);
        bool gate = (seqSampleCounter < gateSamples) && seqStepEnabled[seqCurrentStep];
        float envSig = env.Process(gate);
        
        // FIX: Pass pulsarTrigger as the gate to the Process function
        float pulsarEnvSig = pulsar.Process(pulsarTrigger);

        // --- 5. VIRTUAL PATCH BAY SUMMING (A0-A6) ---
        float rawPulsarPeriodMod = 0.0f;
        float rawModOscFreqMod = 0.0f;  
        float rawModOscModMod = 0.0f;   
        float rawFreqMod = 0.0f;        
        float rawWFMod = 0.0f;          
        float rawModLPG1 = 0.0f;        
        float rawModLPG2 = 0.0f;        

        for (int col = 0; col < 4; col++) {
            float src = 0.0f;
            if (col == 0) src = activeSeqCV;
            else if (col == 1) src = envSig;
            else if (col == 2) src = pulsarEnvSig;
            else if (col == 3) src = currentRandomValue;

            if (col == 0) { // SEQUENCER CV 
                if (matrixStates[col][0]) rawPulsarPeriodMod += src;
                if (matrixStates[col][1]) rawModOscFreqMod   += src; 
                if (matrixStates[col][2]) rawModOscModMod    += src; 
                if (matrixStates[col][3]) rawFreqMod          += src; 
                if (matrixStates[col][4]) rawWFMod            += src; 
                if (matrixStates[col][5]) rawModLPG1         += src;
                if (matrixStates[col][6]) rawModLPG2         += src;
            } else if (col == 1) { // ENV GEN CV
                if (matrixStates[col][0]) rawPulsarPeriodMod += src;
                if (matrixStates[col][1]) rawModOscFreqMod   += src; 
                if (matrixStates[col][2]) rawModOscModMod    += src; 
                if (matrixStates[col][3]) rawFreqMod          += src; 
                if (matrixStates[col][4]) rawWFMod            += src; 
                if (matrixStates[col][5]) rawModLPG1         += src;
                if (matrixStates[col][6]) rawModLPG2         += src;
            } else if (col == 2) { // PULSER CV // SCALED 
                if (matrixStates[col][0]) rawPulsarPeriodMod += src * 100.0f;
                if (matrixStates[col][1]) rawModOscFreqMod   += src * 200.0f; 
                if (matrixStates[col][2]) rawModOscModMod    += src * 100.0f; 
                if (matrixStates[col][3]) rawFreqMod          += src * 200.0f; 
                if (matrixStates[col][4]) rawWFMod            += src * 100.0f; 
                if (matrixStates[col][5]) rawModLPG1         += src * 400.0f;
                if (matrixStates[col][6]) rawModLPG2         += src * 400.0f;
            } else { // RANDOM VOLTAGE
                if (matrixStates[col][0]) rawPulsarPeriodMod += src;
                if (matrixStates[col][1]) rawModOscFreqMod   += src;
                if (matrixStates[col][2]) rawModOscModMod    += src;
                if (matrixStates[col][3]) rawFreqMod          += src;
                if (matrixStates[col][4]) rawWFMod            += src;
                if (matrixStates[col][5]) rawModLPG1         += src;
                if (matrixStates[col][6]) rawModLPG2         += src;
            }
        }

        // --- 6. APPLY ATTENUATORS ---
        pulsarPeriodModCoeff = rawPulsarPeriodMod * (potValues[8] / 65535.0f);
        modulationFreqCoeff = rawModOscFreqMod * modulationFreqModDepth;
        modulationOscModCoeff = rawModOscModMod * modulationOscModCoeffDepth;
        complexOscFreqCoeff = rawFreqMod * complexOscFreqModDepth;
        complexOscWFCoeff = rawWFMod * complexOscWFModDepth;
        vcaComplexOsc = rawModLPG1 * foldedLpgModAmount;
        vcaModulationOsc = rawModLPG2 * modOscLpgModAmount;

        // --- 7. OSCILLATORS ---
        float modMidiFactor = modulationMidiEnabled ? modulationOscMidiFreq : 1.0f;
        float compMidiFactor = complexOscMidiEnabled ? complexOscMidiFreq : 1.0f;

        float modPanelFreq = modulationOscFreq * powf(2.0f, (modulationFreqCoeff * 5.0f) + modulationFine);
        modulationOsc.SetFreq(modPanelFreq * modMidiFactor);
        float modOscSig = modulationOsc.Process();

        float totalModDepth = fminf(fmaxf(modulationOscMod + (modulationOscModCoeff * 2.0f), 0.0f), 2.0f); 

        float fmSignal = 0.0f;
        if (!useAmplitudeMod) fmSignal = modOscSig * totalModDepth;

        float compPanelFreq = complexOscFreq * powf(2.0f, (complexOscFreqCoeff + fmSignal) * 5.0f + complexOscFine);
        complexOsc.SetFreq(compPanelFreq * compMidiFactor);
        complexOscMorph.SetFreq(compPanelFreq * compMidiFactor);

        float mixedSig = (complexOsc.Process() * (1.0f - complexOscMorphMix)) + (complexOscMorph.Process() * complexOscMorphMix);
        if (complexOscInverted) mixedSig *= -1.0f;

        // --- 8. WAVEFOLDER & AM ---
        waveFolder.SetGain(fminf(fmaxf(complexOscWF + (complexOscWFCoeff * 20.0f), 1.0f), 20.0f)); 
        float foldedSig = waveFolder.Process(mixedSig);
        if (useAmplitudeMod) foldedSig *= (1.0f + (modOscSig * totalModDepth));

        // --- 9. BUCHLA LPG MIXER ---
        float finalMix = 0.0f;
        for (int ch = 0; ch < 2; ch++) {
            float signal = (ch == 0) ? foldedSig : modOscSig;
            float ctrl = fminf(fmaxf(((ch == 0) ? complexOscSigLevel : modOscSigLevel) + ((ch == 0) ? vcaComplexOsc : vcaModulationOsc), 0.0f), 1.0f);
            MoogLadder &filter = (ch == 0) ? lpGateFilter1 : lpGateFilter2;
            LpgMode mode = (ch == 0) ? foldedLpgMode : modOscLpgMode;

            float processedSig = signal;
            if (mode == LPG_MODE_VCA) processedSig *= ctrl;
            else {
                filter.SetFreq(20.0f * powf(1000.0f, ctrl));
                filter.SetRes((mode == LPG_MODE_LP) ? powf(ctrl, 2.0f) * 0.2f : 0.0f);
                processedSig = filter.Process(processedSig);
                if (mode == LPG_MODE_COMBI) processedSig *= ctrl;
            }
            finalMix += processedSig;
        }

        // --- 10. REVERB & OUTPUT ---
        float revL, revR;
        reverb.Process(finalMix, finalMix, &revL, &revR);
        out[0][i] = ((finalMix * (1.0f - reverbMix)) + (revL * reverbMix)) * 0.5f;
        out[1][i] = ((finalMix * (1.0f - reverbMix)) + (revR * reverbMix)) * 0.5f;
    }
}

void setup() {
  Serial.begin(115200);
  delay(1000);  // Wait for serial to initialize

  Serial.println("=== DaisySeed Button & Potentiometer Monitor ===");
  Serial.println("Displaying button state changes and potentiometer values...");
  Serial.println();

  // Initialize button matrices
  initButtonMatrix();
  initButtonMatrix2();

  // Initialize potentiometers
  initPotentiometers();

  Serial.println("Potentiometer Configuration:");
  Serial.println("MUX1: Channels 0-15 connected to A0");
  Serial.println("MUX2: Channels 0-15 connected to A1");
  Serial.println("Threshold for reporting changes: ±10 units");
  Serial.println();
  Serial.println("Turn pots or press buttons to see changes...");
  Serial.println("==============================================");

  // DAISY SEED INIT AT 48kHz
  float sample_rate;
  hw = DAISY.init(DAISY_SEED, AUDIO_SR_48K);
  sample_rate = DAISY.get_samplerate();
  DAISY.SetAudioBlockSize(128);
  // START AUDIO
  DAISY.begin(AudioCallback);

  // DAISY DUINO INIT
  // COMPLEX OSC // SINE + MORPH
  complexOsc.Init(sample_rate);
  complexOsc.SetWaveform(complexOsc.WAVE_SIN);
  complexOsc.SetFreq(440.0f);
  complexOsc.SetAmp(1.0f);
  complexOscMorph.Init(sample_rate);
  complexOscMorph.SetWaveform(complexOsc.WAVE_SAW);
  complexOscMorph.SetFreq(440.0f);
  complexOscMorph.SetAmp(1.0f);
    // WAVEFOLDER
    waveFolder.Init();
  // MODULATION OSC
  modulationOsc.Init(sample_rate);
  modulationOsc.SetWaveform(modulationOsc.WAVE_TRI);
  modulationOsc.SetFreq(440.0f);
  modulationOsc.SetAmp(1.0f);
  // LPG FILTER
  lpGateFilter1.Init(sample_rate);
  lpGateFilter2.Init(sample_rate);
  // ENVELOPE
  env.Init(sample_rate);
  // PULSAR ENV
  pulsar.Init(sample_rate);
  pulsar.SetAttackTime(0.02f);
  pulsar.SetDecayTime(0.02f);
  pulsar.SetSustainLevel(1.0f);
  // ANALOGUE OUTPUT FILTER
  outputFilter.Init(sample_rate);
  outputFilter.SetFreq(15000.0f);
  outputFilter.SetRes(0.3f);
  // PULSAR
  //pulsar.Init(sample_rate);
  // REVERB
  reverb.Init(sample_rate);
  reverb.SetFeedback(0.85f);
  reverb.SetLpFreq(18000.0f);

  // MIDI INIT
  // Configure Serial1 to use pin 30 for RX
  Serial1.setRx(MIDI_RX_PIN);
  Serial1.begin(31250);  // Standard MIDI baud rate
  MIDI.begin(MIDI_CHANNEL_OMNI);
  MIDI.turnThruOff();  // Disable MIDI thru to reduce load
  Serial.println("=============================================");
  Serial.println("MIDI initialized with pin 30 (Digital pin 30)");
  Serial.println("=============================================");

}

void loop() {

  // REMOVE COMMENT WHEN NEEDED
  analogReadResolution(16); // 16-BIT ADC

  // MIDI PROCESSING
    if (MIDI.read()) {
        switch (MIDI.getType()) {
            case midi::NoteOn:
                // Only update if we're not receiving a 'Note On' with 0 velocity (which some controllers use as Note Off)
                if (MIDI.getData2() > 0) {
                    complexOscMidiFreq = mtof(MIDI.getData1()); // Use the built-in Daisy mtof (midi to freq) function
                    modulationOscMidiFreq = mtof(MIDI.getData1()); // Use the built-in Daisy mtof (midi to freq) function
                }
                break;
            default:
                break;
        }
    }
  
  // Wait 200 microseconds for the MUX selector pins to settle 
  // and the ADC internal capacitor to drain
  delayMicroseconds(200);

  // READ BUTTON MATRICES
  if (millis() - lastMatrixRead > MATRIX_READ_INTERVAL) {
    readButtonMatrix();
    readButtonMatrix2();  // Read both matrices
    printButtonChanges();
    lastMatrixRead = millis();
  }

  // READ POTENTIOMETERS
  if (millis() - lastPotRead > POT_READ_INTERVAL) {

    readPotentiometers();
    updateParameters(); // UPDATE PARAMS
    printPotentiometerChanges();
    lastPotRead = millis();
  }

}

// UPDATE PARAMETERS
void updateParameters() {

  // OSCILLATORS

  // FINE TUNE
    // 3 cents is 3/1200 of an octave.
    float fineRange = 3.0f / 12.0f; 

  // COMPLEX OSCILLATOR (25Hz to 4000Hz)
    // PITCH
    float complexOscFreqPot = potValues[16] / 65535.0f;
    // 4000 / 25 = 160 (The total frequency ratio)
    complexOscFreq = 25.0f * pow(160.0f, complexOscFreqPot);
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

  // MODULATION OSCILLATOR (1Hz to 2000Hz)
    // PITCH
    float modulationOscFreqPot = potValues[11] / 65535.0f;
    float modMin = 1.0f;
    float modMax = 2000.0f;
    modulationOscFreq = modMin * pow((modMax / modMin), modulationOscFreqPot);
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

  // ENVELOPE [Buchla ASD config]
    float minTime = 0.02f;
    float maxTime = 10.0f;
    float timeRatio = maxTime / minTime;  // 500.0f
    
    // Attack Time (Logarithmic)
    float attackPot = potValues[5] / 65535.0f;
    attackTime = minTime * pow(timeRatio, attackPot);
    env.SetAttackTime(attackTime);
    
    // Release Time (Logarithmic)
    float releasePot = potValues[7] / 65535.0f;
    releaseTime = minTime * pow(timeRatio, releasePot);
    env.SetReleaseTime(releaseTime);

    // Sustain Duration (Logarithmic)
    float sustainDurationPot = potValues[6] / 65535.0f;
     sustainDuration = minTime * pow(timeRatio, sustainDurationPot);

    // Fixed ADSR settings to behave as a gated ASR
    env.SetDecayTime(0.0f);
    env.SetSustainLevel(1.0f);

  // PULSAR
  // Pot 9: Base Pulsar Release Time
    float pulsarReleasePot = potValues[9] / 65535.0f;
    
    // Subtract modulation (clamped) to ensure time decreases
    // We use a log-style calculation similar to your other time constants
    float adjustedPulsarPot = pulsarReleasePot - pulsarPeriodModCoeff;
    adjustedPulsarPot = fminf(fmaxf(adjustedPulsarPot, 0.0f), 1.0f);
    
    pulsarReleaseTime = 0.02f * powf(500.0f, adjustedPulsarPot);
    pulsar.SetReleaseTime(pulsarReleaseTime);
    

  // SEQUENCER
  // Update Sequencer CV steps from MUX1 Channels 0-4
  // Mapping the 16-bit pot value to a usable frequency offset (e.g., 0 to 500Hz)
  // SEQUENCER
  for (int i = 0; i < 5; i++) {
    // Store normalized 0.0 to 1.0 values
    seqStepCV[i] = potValues[i] / 65535.0f;
  }

  // Sequencer Clock Speed from MUX2 Channel 9 (potValues[25])
  float clockPot = potValues[25] / 65535.0f;
  seqClockSpeed = 1.0f * pow(2000.0f, (1.0f - clockPot));

  // REVERB
  reverbMix = potValues[26] / 65535.0f; // REVERB MIX CONTROL

}

// MIDI HANDLING
// 60 is C4 (Middle C)
// We calculate how many octaves the incoming note is from C4
// Factor = 2 ^ ((Note - 60) / 12)
float midiNoteToFactor(int note) {
    return powf(2.0f, (note - 60.0f) / 12.0f);
}

void handleNoteOn(byte channel, byte note, byte velocity) {
    if (velocity > 0) {
        // Set the flag for the sequencer to advance one step
        midiTriggerPending = true;

        // Trigger the Pulser if it is in MIDI mode
        if (currentPulsarMode == PULSAR_MODE_MIDI) {
            pulsar.Retrigger(false);
        }

        // Calculate relative octave distance from C4
        float factor = powf(2.0f, (note - 60.0f) / 12.0f);
        
        complexOscMidiFreq = factor;
        modulationOscMidiFreq = factor;
    }
}

void handleNoteOff(byte channel, byte note, byte velocity) {
    // Optional: Reset factor to 1.0 or leave at last note played
}