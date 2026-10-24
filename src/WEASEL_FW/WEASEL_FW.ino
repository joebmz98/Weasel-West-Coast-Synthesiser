// ********************** //
// ==== WEASEL ========== //
// Buchla Music Easel     //
// Inspired Digital Synth //
// firmware version 1.0   //
// ********************** //
// designed by            //
// .axs instruments       //
// ********************** //
// Description: Buchla Music Easel inspired digital synth
//              Patch bay simulating modular experience.
//
// v1.0 hardware changes vs v0.9:
//  - Button matrices replaced by 4x 74HC165 shift registers (32-bit chain)
//      SR1..SR3 = panel buttons, SR4 = patch bay destinations
//  - Patch bay sources driven from D17..D20, destinations read on SR4
//  - Mode LEDs driven by 4x AW9523 (0x58..0x5B) over I2C
//  - MUX2_S3 moved from D7 to D10 (D7 is now the 74HC165 SH/LD line)
//  - Master volume pot on MUX2 C11
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
#include <Wire.h>          // I2C (AW9523 LED drivers)
#include <MIDI.h>          // MIDI
#include "wavefolder.h"    // EXTRACTED FROM DAISYSP
#include <math.h>

using namespace daisysp;

// -- FIRMWARE VERSION --
const char* FIRMWARE_VERSION = "1.0";

#pragma GCC optimize("O2")

// -- DEBUG --
// 1 = print button presses / patch changes over USB serial
#define WEASEL_DEBUG 0

// ============================================
// PIN MAP  (Daisy D-number = physical pin - 1)
// ============================================
// -- MUX PINS - FIRST MUX
#define MUX1_S0 D0
#define MUX1_S1 D1
#define MUX1_S2 D2
#define MUX1_S3 D3
#define MUX1_SIG A0  // MUX1 signal pin

// -- MUX PINS - SECOND MUX
#define MUX2_S0 D4
#define MUX2_S1 D5
#define MUX2_S2 D6
#define MUX2_S3 D10  // MOVED from D7 (physical pin 11 / SPI1_MOSI)
#define MUX2_SIG A1  // MUX2 signal pin

// -- 74HC165 SHIFT REGISTER CHAIN
#define SH_LD_PIN D7  // physical pin 8
#define CLK_PIN D8    // physical pin 9
#define QH_PIN D9     // physical pin 10

// -- PATCH BAY SOURCE DRIVE PINS
#define SEQ_OUT_PIN D17   // physical pin 24
#define EG_OUT_PIN D18    // physical pin 25
#define PLSR_OUT_PIN D19  // physical pin 26
#define RNDM_OUT_PIN D20  // physical pin 27

// -- MIDI
#define MIDI_RX_PIN 14  // D14 / USART1 Rx (physical pin 15)

// I2C uses the default Wire pins: D11 (SCL) / D12 (SDA)

// ============================================
// POT INDEX MAP  (potValues[] index: MUX1 = 0..15, MUX2 = 16..31)
// ============================================
enum PotIndex {
  // MUX1
  POT_SEQ_STEP1 = 0,
  POT_SEQ_STEP2 = 1,
  POT_SEQ_STEP3 = 2,
  POT_SEQ_STEP4 = 3,
  POT_SEQ_STEP5 = 4,
  POT_ENV_ATTACK = 5,
  POT_ENV_SUSTAIN = 6,
  POT_ENV_DECAY = 7,
  POT_PULSAR_CTRL = 8,
  POT_PULSAR_PERIOD = 9,
  POT_MODOSC_FREQ_CTRL = 10,
  POT_MODOSC_FREQ = 11,
  POT_MODOSC_FINE = 12,
  POT_MODOSC_MOD_CTRL = 13,
  POT_MODOSC_MOD = 14,
  POT_COMPOSC_FREQ_CTRL = 15,
  // MUX2
  POT_COMPOSC_FREQ = 16,
  POT_COMPOSC_FINE = 17,
  POT_COMPOSC_WF_CTRL = 18,
  POT_COMPOSC_WF = 19,
  POT_COMPOSC_TIMBRE = 20,
  POT_LPG1_CTRL = 21,
  POT_LPG1_LEVEL = 22,
  POT_LPG2_CTRL = 23,
  POT_LPG2_LEVEL = 24,
  POT_CLOCK = 25,
  POT_REVERB_MIX = 26,
  POT_MASTER_VOLUME = 27
};

// ============================================
// SHIFT REGISTER BUTTON MAP
// 32-bit read: SR1 = bits 31..24, SR2 = 23..16, SR3 = 15..8, SR4 = 7..0
// SRn Dk -> bit (32 - 8n + k)
// ============================================
#define SR_BIT(sr, d) (32 - 8 * (sr) + (d))

enum ButtonBit {
  // SR1
  BTN_SEQ_TRIG_MODE = SR_BIT(1, 0),
  BTN_SEQ_STAGES = SR_BIT(1, 1),
  BTN_SEQ_STEP1 = SR_BIT(1, 2),
  BTN_SEQ_STEP2 = SR_BIT(1, 3),
  BTN_SEQ_STEP3 = SR_BIT(1, 4),
  BTN_SEQ_STEP4 = SR_BIT(1, 5),
  BTN_SEQ_STEP5 = SR_BIT(1, 6),
  BTN_ENV_TRIG_MODE = SR_BIT(1, 7),
  // SR2
  BTN_ENV_MODE = SR_BIT(2, 0),
  BTN_PULSAR_MANUAL = SR_BIT(2, 1),
  BTN_PULSAR_MODE = SR_BIT(2, 2),
  BTN_MODOSC_AM_FM = SR_BIT(2, 3),
  BTN_MODOSC_WAVE = SR_BIT(2, 4),
  BTN_COMPOSC_POLARITY = SR_BIT(2, 5),
  BTN_COMPOSC_WAVE = SR_BIT(2, 6),
  BTN_QUANTIZE_MIDI = SR_BIT(2, 7),
  // SR3
  BTN_LPG1_MODE = SR_BIT(3, 0),
  BTN_LPG2_MODE = SR_BIT(3, 1),
  BTN_RANDOM_MODE = SR_BIT(3, 2),
  BTN_CLOCK_SYNC = SR_BIT(3, 3)
};

// Only the button bits (SR1..SR3, SR3 D0-D3) are considered for edge detection
const uint32_t BUTTON_MASK = 0xFFFF0F00UL;

// Sequencer step buttons:
// 1 = latching switches, step is ENABLED while the switch reads LOW (v0.9 behaviour)
// 0 = momentary buttons, each press toggles the step on/off
#define SEQ_STEP_SWITCHES_LATCHING 1

const uint8_t SEQ_STEP_BITS[5] = { BTN_SEQ_STEP1, BTN_SEQ_STEP2, BTN_SEQ_STEP3, BTN_SEQ_STEP4, BTN_SEQ_STEP5 };

uint32_t buttonRawPrev = 0;      // previous raw read (for 2-read debounce)
uint32_t buttonDebounced = 0;    // stable button state
uint32_t buttonLastHandled = 0;  // state at last edge-processing pass
unsigned long lastButtonRead = 0;
const unsigned long BUTTON_READ_INTERVAL = 10;  // ms -> 10-20ms effective debounce

// ============================================
// PATCH BAY (4 sources x 7 destinations, read on SR4 D0-D6)
// ============================================
#define NUM_SOURCES 4
#define NUM_DESTINATIONS 7
#define PATCH_DEBOUNCE_READS 3

const int sourcePins[NUM_SOURCES] = { SEQ_OUT_PIN, EG_OUT_PIN, PLSR_OUT_PIN, RNDM_OUT_PIN };

// patchMatrix[source][destination]
// Sources:      0 SEQ, 1 EG, 2 PLSR, 3 RNDM
// Destinations: 0 PLSR_PERIOD, 1 MODOSC_FREQ, 2 MODOSC_MOD, 3 COMP_PITCH,
//               4 COMP_WAVE, 5 LPG_LVL1, 6 LPG_LVL2
volatile bool patchMatrix[NUM_SOURCES][NUM_DESTINATIONS] = {};
uint8_t patchCounter[NUM_SOURCES][NUM_DESTINATIONS] = {};
unsigned long lastPatchScan = 0;
const unsigned long PATCH_SCAN_INTERVAL = 20;  // ms

#if WEASEL_DEBUG
bool lastPatchMatrix[NUM_SOURCES][NUM_DESTINATIONS] = {};
const char* sourceNames[NUM_SOURCES] = { "SEQ", "EG", "PLSR", "RNDM" };
const char* destNames[NUM_DESTINATIONS] = { "PLSR_PERIOD", "MODOSC_FREQ", "MODOSC_MOD",
                                            "COMP_PITCH", "COMP_WAVE", "LPG_LVL1", "LPG_LVL2" };
#endif

// ============================================
// AW9523 LED DRIVERS
// ============================================
#define AW9523_REG_OUTPUT 0x02
#define AW9523_REG_CONFIG 0x04
#define AW9523_REG_GLOBAL_CTRL 0x11
#define AW9523_REG_SOFTWARE_RESET 0x7F

const uint8_t aw9523_addresses[] = { 0x58, 0x59, 0x5A, 0x5B };
const int NUM_CHIPS = 4;
bool chipPresent[NUM_CHIPS] = { false };

struct LedLocation {
  uint8_t chip;
  uint8_t port;
  uint8_t bit;
};

// LED groups - entry N lights when the function is in state N.
// Reorder entries here if the panel order differs from the firmware state order.
const LedLocation LED_SEQ_TRIG[] = { { 0, 0, 0 }, { 0, 0, 1 }, { 0, 0, 2 } };    // CLOCK, PULSAR, MIDI
const LedLocation LED_SEQ_STAGES[] = { { 0, 0, 3 }, { 0, 0, 4 }, { 0, 0, 5 } };  // 3, 4, 5 steps
const LedLocation LED_ENV_TRIG[] = { { 0, 1, 3 }, { 0, 1, 4 }, { 0, 1, 5 } };    // SEQ, PULSAR, MIDI
const LedLocation LED_ENV_MODE[] = { { 0, 1, 6 }, { 0, 1, 7 }, { 1, 0, 0 } };    // TRANSIENT, SUSTAIN, OSCILLATE
const LedLocation LED_PULSAR_MODE[] = { { 1, 0, 2 }, { 1, 0, 3 }, { 1, 0, 4 } };  // SEQ, MIDI, OSC
const LedLocation LED_PULSAR_ACT[] = { { 1, 0, 5 }, { 1, 0, 6 } };               // idle, firing
const LedLocation LED_MODOSC_AMFM[] = { { 2, 0, 4 }, { 2, 0, 5 } };              // FM, AM
const LedLocation LED_MODOSC_WAVE[] = { { 2, 0, 0 }, { 2, 0, 1 }, { 2, 0, 2 }, { 2, 0, 3 } };  // SIN, TRI, SAW, SQUARE
const LedLocation LED_COMP_POL[] = { { 2, 0, 6 }, { 2, 0, 7 } };                 // NORMAL, INVERTED
const LedLocation LED_COMP_WAVE[] = { { 2, 1, 0 }, { 2, 1, 1 }, { 2, 1, 2 } };   // TRI, SAW, SQUARE
const LedLocation LED_QUANTIZE[] = { { 2, 1, 3 }, { 2, 1, 4 } };                 // OFF, ON
const LedLocation LED_RANDOM_MODE[] = { { 2, 1, 5 }, { 2, 1, 6 }, { 2, 1, 7 } };  // SEQ, PULSAR, MIDI
const LedLocation LED_CLOCK_SYNC[] = { { 3, 0, 1 }, { 3, 0, 2 } };               // FREE, MIDI SYNC
// LPG mode LEDs not yet assigned - add groups here and in buildLedFrame() when known

uint8_t ledFrame[NUM_CHIPS][2] = { { 0 } };     // desired state [chip][port]
uint8_t ledWritten[NUM_CHIPS][2] = { { 0 } };   // last state sent to the chip
bool ledForceWrite = true;
unsigned long lastLedUpdate = 0;
const unsigned long LED_UPDATE_INTERVAL = 10;  // ms

// -- POT VARIABLES --
const int NUM_POTS_PER_MUX = 16;
const int TOTAL_POTS = 32;  // 2 muxes * 16 channels each
int potValues[TOTAL_POTS] = { 0 };
unsigned long lastPotRead = 0;
const unsigned long POT_READ_INTERVAL = 5;  // Read pots every Xms


// -- DAISY DUINO OBJECTS --
DaisyHardware hw;  // DAISY SEED
// -- OSCILLATORS
Oscillator complexOsc;       // COMPLEX OSCILLATOR
Oscillator complexOscMorph;  // COMPLEX MORPH OSCILLATOR
Oscillator modulationOsc;    // MODULATION OSCILLATOR
// -- ENVELOPE GENERATOR
Adsr env;  // ENV GENERATOR
// -- PULSAR GENERATOR
Adsr pulsar;  // ENV GENERATOR FOR PULSAR
// -- LOW-PASS - LPG
Svf lpGateFilter1;  // FILTER FOR LPG CHAN1
Svf lpGateFilter2;  // FILTER FOR LPG CHAN2
// -- OUTPUT ANALOGUE FILTER
MoogLadder outputFilter;
// -- REVERB
ReverbSc reverb;  // REVERB
// -- WAVEFOLDER
daisysp::Wavefolder waveFolder;


// -- OBJECT VARIABLES --
float sampleRate;
// -- OSCILLATORS
// -- COMPLEX
float complexOscFreq;                 // CONTROLS FREQUENCY
float complexOscFine;                 // CONTROLS FINE TUNE
float complexOscFreqCoeff;            // COEFF FOR FREQUENCY MODULATION USING PATCH BAY
float complexOscWF;                   // CONTROLS WF AMOUNT
float complexOscWFCoeff;              // COEFF FOR WF MODULATION USING PATCH BAY
float complexOscWFModDepth = 0.0f;    // WAVEFOLDER MOD DEPTH
float complexOscFreqModDepth = 0.0f;  // FREQ MOD DEPTH
bool complexOscInverted = false;      // TOGGLE POLARITY
bool complexOscMidiEnabled = false;   // TOGGLE MIDI
float complexOscMorphMix;             // COMPLEX OSC TIMBRE AMOUNT
int morphWaveformIndex = 1;           // 0 = TRI, 1 = SAW, 2 = SQUARE (boots on SAW)
float complexOscSigLevel = 0.0f;      // LPG CONTROL

// -- MODULATION
float modulationOscFreq = 0.0f;           // CONTROLS FREQUENCY
float modulationFine = 0.0f;              // CONTROLS FINE TUNE
float modulationFreqCoeff = 0.0f;         // COEFF FOR FREQUENCY MODULATION USING PATCH BAY
float modulationOscMod = 0.0f;            // CONTROLS MOD AMOUNT APPLIED TO COMPLEXOSC
float modulationOscModCoeff;              // COEFF FOR MOD AMOUNT MODULATION USING PATCH BAY
float modulationFreqModDepth = 0.0f;      // FREQ MOD DEPTH
float modulationOscModCoeffDepth = 0.0f;  // MODULATION MOD DEPTH
bool useAmplitudeMod = false;             // False = FM, True = AM
bool modulationMidiEnabled = false;       // TOGGLE MIDI
int modWaveformIndex = 0;                 // 0: SIN, 1: TRI, 2: SAW, 3: SQUARE
float modOscSigLevel = 0.0f;              // LPG CONTROL

// -- LPG --
float vcaComplexOsc;              // AMPLITUDE COEFF FOR COMPLEXOSC
float vcaModulationOsc;           // AMPLITUDE COEFF FOR MODULATIONOSC
float foldedLpgModAmount = 0.0f;  // Multiplier for Folded LPG mod
float modOscLpgModAmount = 0.0f;  // Multiplier for Mod Osc LPG mod
enum LpgMode { LPG_MODE_VCA,
               LPG_MODE_LP,
               LPG_MODE_COMBI };
LpgMode foldedLpgMode = LPG_MODE_COMBI;  // Default to COMBI mode
LpgMode modOscLpgMode = LPG_MODE_COMBI;  // Default to COMBI mode
float lastCtrl0 = -1.0f;
float lastCtrl1 = -1.0f;
int lastMode0 = -1;
int lastMode1 = -1;
#define LPG_TABLE_SIZE 64
float lpgFreqTable[LPG_TABLE_SIZE];


// -- ENVELOPE [BUCHLA ASD/ASR] --
enum EnvelopeMode { ENV_MODE_TRANSIENT,            // One-shot, cannot sustain
                    ENV_MODE_SUSTAIN,              // Can sustain with gate
                    ENV_MODE_OSCILLATE };          // Looping envelope
EnvelopeMode currentEnvMode = ENV_MODE_TRANSIENT;  // Default to Transient
float attackTime = 0.0f;
float releaseTime = 0.0f;
float sustainDuration = 0.0f;
uint32_t gateRemainingSamples = 0;
uint32_t transientGateSamples = 0;
uint32_t sustainGateSamples = 0;
bool envRetriggerFlag = false;  // For oscillating mode

// -- ENVELOPE TRIGGER SELECT --
enum EnvTriggerMode { ENV_TRIGGER_SEQ,                   // Sequencer steps (default)
                      ENV_TRIGGER_PULSAR,                // Pulsar trigger
                      ENV_TRIGGER_MIDI };                // MIDI note on
EnvTriggerMode currentEnvTriggerMode = ENV_TRIGGER_SEQ;  // Default to Sequencer
volatile bool envMidiTrigger = false;                    // Separate flag for MIDI triggering
static uint32_t oscGateSamples = 0;


// -- ENVELOPE STATE --
volatile bool midiNoteHeld = false;  // True while a MIDI note is physically held

// -- PULSAR ENV --
enum PulsarMode { PULSAR_MODE_SEQ,
                  PULSAR_MODE_MIDI,
                  PULSAR_MODE_OSC };
PulsarMode currentPulsarMode = PULSAR_MODE_SEQ;
float pulsarReleaseTime = 0.02f;
bool lastPulsarEnvActive = false;
float pulsarPeriodModCoeff = 0.0f;
volatile bool manualPulsarTrigger = false;
volatile bool pulsarMidiTrigger = false;
volatile bool pulsarRunning = false;  // mirrored from the audio callback for the activity LED


// -- SAMPLE&HOLD / RANDOM VOLTAGE --
enum RandomMode { RANDOM_MODE_SEQ,
                  RANDOM_MODE_PULSAR,
                  RANDOM_MODE_MIDI };
RandomMode currentRandomMode = RANDOM_MODE_SEQ;
float currentRandomValue = 0.0f;
volatile bool randomMidiTrigger = false;

// -- REVERB --
float reverbMix;  // REVERB MIX COEFF

// -- MASTER VOLUME --
float masterVolume = 0.0f;          // target, set from pot (0..1)
float masterVolumeSmoothed = 0.0f;  // per-sample smoothed value (boots silent, fades in)
const float OUTPUT_HEADROOM = 0.4f;  // v0.9 fixed output gain, now the ceiling at full volume

// -- MIDI --
MIDI_CREATE_INSTANCE(HardwareSerial, Serial1, MIDI);
// --
bool quantizeToMidi = false;       // Enable/disable MIDI quantization mode
float lastMidiNoteFreq = 261.63f;  // Last received MIDI note transpose ratio
bool midiNoteReceived = false;     // Track if we've received a MIDI note in quantize mode

// -- MIDI CLOCK SYNC --
bool midiClockSync = false;               // Enable/disable MIDI clock sync
bool midiClockActive = false;             // Whether MIDI clock is currently being received
uint32_t midiClockTicks = 0;              // Counter for MIDI clock ticks
uint32_t midiClockSamplesPerTick = 0;     // Samples between steps (calculated from tempo)
float midiTempo = 120.0f;                 // Current MIDI tempo in BPM
uint32_t lastMidiClockTime = 0;           // Last time we received a MIDI clock message
const uint32_t MIDI_CLOCK_TIMEOUT = 500;  // Timeout in milliseconds (0.5 seconds)


// -- CLOCK DIVISIONS (for MIDI sync mode) --
const float clockDivisions[] = { 1.0f, 2.0f, 4.0f, 8.0f, 12.0f, 16.0f, 24.0f, 32.0f };
const int NUM_DIVISIONS = 8;
int currentDivisionIndex = 0;  // 0 = 1/1, 1 = 1/2, etc.
float currentDivision = 1.0f;  // Current clock division multiplier


// -- SEQUENCER --
int seqCurrentStep = 0;
bool seqStepEnabled[5] = { true, true, true, true, true };  // All steps ON by default
float seqStepCV[5] = { 0.0f };                              // Stores current CV from MUX1 0-4
float seqClockSpeed = 100.0f;
int seqMaxSteps = 5;  // Default sequence length
// AUDIO INTERRUPT TIMING
uint32_t seqSampleCounter = 0;  // Counts samples to trigger the next step
float activeSeqCV = 0.0f;       // The CV of the currently active step
// SAMPLE RATE
uint32_t samplesPerTick = 48000;  // default, will be updated in updateParameters()

// SEQUENCER MODES
enum SeqTriggerMode { SEQ_TRIGGER_CLOCK,
                      SEQ_TRIGGER_PULSAR,
                      SEQ_TRIGGER_MIDI };
SeqTriggerMode currentSeqTriggerMode = SEQ_TRIGGER_CLOCK;  // Default Mode
volatile bool midiTriggerPending = false;                  // Flag to catch MIDI notes for the sequencer
int currentMidiNote = 60;                                  // Default to Middle C when no midi is detected

// -- OPTIMIZATION VARIABLES --
float rawModulationValues[7] = { 0 };  // Pre-calculated modulation values
float ln2 = 0.69314718056f;            // ln(2) for fast exp approximations
uint32_t modUpdateCounter = 0;
const uint32_t MOD_UPDATE_RATE = 16;

// -- FORWARD DECLARATIONS --
void updateClockDivision();
void updateParameters();
void checkMidiClockActivity();
void handleNoteOn(byte channel, byte note, byte velocity);
void handleNoteOff(byte channel, byte note, byte velocity);
void handleMidiClock();
void handleMidiStart();
void handleMidiStop();

// -- FAST MATH APPROXIMATIONS --
inline float fast_exp2_approx(float x) {
  // Fast 2^x approximation using Taylor series expansion
  x = 1.0f + x * ln2;  // First-order approximation
  // Second-order improvement for better accuracy
  return x * (1.0f + (x - 1.0f) * 0.5f);
}

inline float fast_lpg_freq(float ctrl) {
  if (ctrl < 0.0f) ctrl = 0.0f;
  if (ctrl > 1.0f) ctrl = 1.0f;

  float idx = ctrl * (LPG_TABLE_SIZE - 1);
  int i = (int)idx;
  if (i >= LPG_TABLE_SIZE - 1) return lpgFreqTable[LPG_TABLE_SIZE - 1];
  float frac = idx - i;
  return lpgFreqTable[i] + frac * (lpgFreqTable[i + 1] - lpgFreqTable[i]);
}

// NaN and bad float detection
inline bool isBadFloat(float x) {
  return (x != x) || (x > 1e10f) || (x < -1e10f);
}

// Get frequency ratio from MIDI note relative to C4 (note 60)
inline float getMidiTransposeRatio(int midiNote) {
  float semitones = (float)(midiNote - 60);
  return powf(2.0f, semitones / 12.0f);
}

inline float potNorm(int index) {
  return potValues[index] * (1.0f / 65535.0f);
}

// ============================================
// POTENTIOMETERS (2x 16-ch MUX)
// ============================================
void selectMuxChannel(int channel, int s0, int s1, int s2, int s3) {
  digitalWrite(s0, bitRead(channel, 0));
  digitalWrite(s1, bitRead(channel, 1));
  digitalWrite(s2, bitRead(channel, 2));
  digitalWrite(s3, bitRead(channel, 3));
}

void initPotentiometers() {
  const int muxPins[] = { MUX1_S0, MUX1_S1, MUX1_S2, MUX1_S3, MUX2_S0, MUX2_S1, MUX2_S2, MUX2_S3 };
  for (int i = 0; i < 8; i++) {
    pinMode(muxPins[i], OUTPUT);
    digitalWrite(muxPins[i], LOW);
  }
  pinMode(MUX1_SIG, INPUT);
  pinMode(MUX2_SIG, INPUT);
}

void readPotentiometers() {
  for (int i = 0; i < TOTAL_POTS; i++) {
    if (i < NUM_POTS_PER_MUX) {
      selectMuxChannel(i, MUX1_S0, MUX1_S1, MUX1_S2, MUX1_S3);
      delayMicroseconds(10);
      potValues[i] = analogRead(MUX1_SIG);
    } else {
      selectMuxChannel(i - NUM_POTS_PER_MUX, MUX2_S0, MUX2_S1, MUX2_S2, MUX2_S3);
      delayMicroseconds(10);
      potValues[i] = analogRead(MUX2_SIG);
    }
  }
}

// ============================================
// 74HC165 SHIFT REGISTERS
// ============================================
void initShiftRegisters() {
  pinMode(SH_LD_PIN, OUTPUT);
  pinMode(CLK_PIN, OUTPUT);
  pinMode(QH_PIN, INPUT);
  digitalWrite(CLK_PIN, LOW);
  digitalWrite(SH_LD_PIN, HIGH);
}

// Reads all 32 bits. QH is sampled before each clock pulse.
uint32_t readAllShiftRegisters() {
  digitalWrite(SH_LD_PIN, LOW);
  delayMicroseconds(10);
  digitalWrite(SH_LD_PIN, HIGH);
  delayMicroseconds(10);

  uint32_t allData = 0;
  for (int i = 31; i >= 0; i--) {
    if (digitalRead(QH_PIN) == HIGH) {
      allData |= (1UL << i);
    }
    digitalWrite(CLK_PIN, HIGH);
    delayMicroseconds(5);
    digitalWrite(CLK_PIN, LOW);
    delayMicroseconds(5);
  }
  return allData;
}

inline bool buttonIsDown(uint8_t bit) {
  return (buttonDebounced >> bit) & 1UL;
}

// ============================================
// PATCH BAY
// ============================================
void initPatchBay() {
  for (int s = 0; s < NUM_SOURCES; s++) {
    pinMode(sourcePins[s], OUTPUT);
    digitalWrite(sourcePins[s], LOW);
  }
}

void scanPatchMatrix() {
  for (int s = 0; s < NUM_SOURCES; s++) {
    digitalWrite(sourcePins[s], LOW);
  }

  for (int s = 0; s < NUM_SOURCES; s++) {
    digitalWrite(sourcePins[s], HIGH);
    delayMicroseconds(100);

    uint8_t sr4 = readAllShiftRegisters() & 0xFF;

    for (int d = 0; d < NUM_DESTINATIONS; d++) {
      bool reading = (sr4 & (1 << d)) != 0;
      if (reading) {
        if (patchCounter[s][d] < PATCH_DEBOUNCE_READS) patchCounter[s][d]++;
      } else {
        if (patchCounter[s][d] > 0) patchCounter[s][d]--;
      }
      patchMatrix[s][d] = (patchCounter[s][d] >= PATCH_DEBOUNCE_READS);
    }

    digitalWrite(sourcePins[s], LOW);
    delayMicroseconds(50);

    // Keep the MIDI input drained between source scans
    while (MIDI.read()) {}
  }

#if WEASEL_DEBUG
  for (int s = 0; s < NUM_SOURCES; s++) {
    for (int d = 0; d < NUM_DESTINATIONS; d++) {
      if (patchMatrix[s][d] != lastPatchMatrix[s][d]) {
        Serial.print(sourceNames[s]);
        Serial.print(patchMatrix[s][d] ? " -> " : " -x ");
        Serial.println(destNames[d]);
        lastPatchMatrix[s][d] = patchMatrix[s][d];
      }
    }
  }
#endif
}

// ============================================
// AW9523 LED DRIVERS
// ============================================
bool initializeAW9523(uint8_t address) {
  Wire.beginTransmission(address);
  Wire.write(AW9523_REG_SOFTWARE_RESET);
  Wire.write(0x00);
  if (Wire.endTransmission() != 0) return false;

  delay(10);

  // All pins as outputs
  Wire.beginTransmission(address);
  Wire.write(AW9523_REG_CONFIG);
  Wire.write(0x00);
  Wire.write(0x00);
  if (Wire.endTransmission() != 0) return false;

  // All outputs low
  Wire.beginTransmission(address);
  Wire.write(AW9523_REG_OUTPUT);
  Wire.write(0x00);
  Wire.write(0x00);
  if (Wire.endTransmission() != 0) return false;

  return true;
}

void configurePushPull(uint8_t address) {
  Wire.beginTransmission(address);
  Wire.write(AW9523_REG_GLOBAL_CTRL);
  Wire.endTransmission(false);

  Wire.requestFrom(address, (uint8_t)1);
  uint8_t gcrValue = 0x00;
  if (Wire.available()) gcrValue = Wire.read();

  gcrValue |= (1 << 4);  // Port 0 push-pull

  Wire.beginTransmission(address);
  Wire.write(AW9523_REG_GLOBAL_CTRL);
  Wire.write(gcrValue);
  Wire.endTransmission();
}

void initLeds() {
  Wire.setClock(100000);
  Wire.begin();
  for (int chip = 0; chip < NUM_CHIPS; chip++) {
    chipPresent[chip] = initializeAW9523(aw9523_addresses[chip]);
    if (chipPresent[chip]) configurePushPull(aw9523_addresses[chip]);
#if WEASEL_DEBUG
    if (!chipPresent[chip]) {
      Serial.print("AW9523 not found at 0x");
      Serial.println(aw9523_addresses[chip], HEX);
    }
#endif
  }
  ledForceWrite = true;
}

inline void setLed(const LedLocation* group, int size, int index) {
  if (index < 0 || index >= size) return;
  const LedLocation& l = group[index];
  ledFrame[l.chip][l.port] |= (1 << l.bit);
}
#define SET_LED(group, index) setLed(group, sizeof(group) / sizeof(group[0]), (int)(index))

// Builds the LED frame from the real synth state, so LEDs always reflect firmware state
void buildLedFrame() {
  memset(ledFrame, 0, sizeof(ledFrame));

  SET_LED(LED_SEQ_TRIG, currentSeqTriggerMode);
  SET_LED(LED_SEQ_STAGES, seqMaxSteps - 3);
  SET_LED(LED_ENV_TRIG, currentEnvTriggerMode);
  SET_LED(LED_ENV_MODE, currentEnvMode);
  SET_LED(LED_PULSAR_MODE, currentPulsarMode);
  SET_LED(LED_PULSAR_ACT, pulsarRunning ? 1 : 0);
  SET_LED(LED_MODOSC_AMFM, useAmplitudeMod ? 1 : 0);
  SET_LED(LED_MODOSC_WAVE, modWaveformIndex);
  SET_LED(LED_COMP_POL, complexOscInverted ? 1 : 0);
  SET_LED(LED_COMP_WAVE, morphWaveformIndex);
  SET_LED(LED_QUANTIZE, quantizeToMidi ? 1 : 0);
  SET_LED(LED_RANDOM_MODE, currentRandomMode);
  SET_LED(LED_CLOCK_SYNC, midiClockSync ? 1 : 0);
}

// Only writes chips whose outputs changed
void updateLeds() {
  buildLedFrame();
  for (int chip = 0; chip < NUM_CHIPS; chip++) {
    if (!chipPresent[chip]) continue;
    if (!ledForceWrite && ledFrame[chip][0] == ledWritten[chip][0] && ledFrame[chip][1] == ledWritten[chip][1]) continue;

    Wire.beginTransmission(aw9523_addresses[chip]);
    Wire.write(AW9523_REG_OUTPUT);
    Wire.write(ledFrame[chip][0]);
    Wire.write(ledFrame[chip][1]);
    if (Wire.endTransmission() == 0) {
      ledWritten[chip][0] = ledFrame[chip][0];
      ledWritten[chip][1] = ledFrame[chip][1];
    }
  }
  ledForceWrite = false;
}

// ============================================
// BUTTON FUNCTIONS
// ============================================
void handleButtonPress(uint8_t bit) {
  switch (bit) {
    // --- SR1 ---
    case BTN_SEQ_TRIG_MODE:
      // Cycles trigger mode: CLOCK -> PULSAR -> MIDI
      currentSeqTriggerMode = static_cast<SeqTriggerMode>((currentSeqTriggerMode + 1) % 3);
      break;

    case BTN_SEQ_STAGES:
      // Cycles between 5, 4 and 3 steps
      if (seqMaxSteps == 5) seqMaxSteps = 4;
      else if (seqMaxSteps == 4) seqMaxSteps = 3;
      else seqMaxSteps = 5;
      seqCurrentStep %= seqMaxSteps;
      break;

    case BTN_ENV_TRIG_MODE:
      // Cycles: SEQ -> PULSAR -> MIDI
      currentEnvTriggerMode = static_cast<EnvTriggerMode>((currentEnvTriggerMode + 1) % 3);
      break;

    // --- SR2 ---
    case BTN_ENV_MODE:
      // Cycles: TRANSIENT -> SUSTAIN -> OSCILLATE
      currentEnvMode = static_cast<EnvelopeMode>((currentEnvMode + 1) % 3);
      if (currentEnvMode == ENV_MODE_OSCILLATE) envRetriggerFlag = false;
      break;

    case BTN_PULSAR_MANUAL:
      manualPulsarTrigger = true;  // fires on next audio sample
      break;

    case BTN_PULSAR_MODE:
      // Cycles: SEQ -> MIDI -> OSC (free running)
      currentPulsarMode = static_cast<PulsarMode>((currentPulsarMode + 1) % 3);
      break;

    case BTN_MODOSC_AM_FM:
      useAmplitudeMod = !useAmplitudeMod;
      break;

    case BTN_MODOSC_WAVE:
      // SIN -> TRI -> SAW -> SQUARE
      modWaveformIndex = (modWaveformIndex + 1) % 4;
      if (modWaveformIndex == 0) modulationOsc.SetWaveform(modulationOsc.WAVE_SIN);
      else if (modWaveformIndex == 1) modulationOsc.SetWaveform(modulationOsc.WAVE_TRI);
      else if (modWaveformIndex == 2) modulationOsc.SetWaveform(modulationOsc.WAVE_SAW);
      else modulationOsc.SetWaveform(modulationOsc.WAVE_SQUARE);
      break;

    case BTN_COMPOSC_POLARITY:
      complexOscInverted = !complexOscInverted;
      break;

    case BTN_COMPOSC_WAVE:
      // TRI -> SAW -> SQUARE
      morphWaveformIndex = (morphWaveformIndex + 1) % 3;
      if (morphWaveformIndex == 0) complexOscMorph.SetWaveform(complexOscMorph.WAVE_TRI);
      else if (morphWaveformIndex == 1) complexOscMorph.SetWaveform(complexOscMorph.WAVE_SAW);
      else complexOscMorph.SetWaveform(complexOscMorph.WAVE_SQUARE);
      break;

    case BTN_QUANTIZE_MIDI:
      quantizeToMidi = !quantizeToMidi;
      if (quantizeToMidi) midiNoteReceived = false;
      break;

    // --- SR3 ---
    case BTN_LPG1_MODE:
      // VCA -> LP -> COMBI
      foldedLpgMode = static_cast<LpgMode>((foldedLpgMode + 1) % 3);
      break;

    case BTN_LPG2_MODE:
      modOscLpgMode = static_cast<LpgMode>((modOscLpgMode + 1) % 3);
      break;

    case BTN_RANDOM_MODE:
      // SEQ -> PULSAR -> MIDI
      currentRandomMode = static_cast<RandomMode>((currentRandomMode + 1) % 3);
      break;

    case BTN_CLOCK_SYNC:
      midiClockSync = !midiClockSync;
      if (midiClockSync) {
        seqCurrentStep = 0;
        seqSampleCounter = 0;
        midiClockTicks = 0;
        updateClockDivision();
      }
      break;

    default:
#if !SEQ_STEP_SWITCHES_LATCHING
      for (int i = 0; i < 5; i++) {
        if (bit == SEQ_STEP_BITS[i]) seqStepEnabled[i] = !seqStepEnabled[i];
      }
#endif
      break;
  }

#if WEASEL_DEBUG
  Serial.print("Button bit ");
  Serial.println(bit);
#endif
}

void readButtons() {
  uint32_t raw = readAllShiftRegisters() & BUTTON_MASK;

  // Accept a new state only once two consecutive reads agree
  if (raw == buttonRawPrev) buttonDebounced = raw;
  buttonRawPrev = raw;

  uint32_t pressed = buttonDebounced & ~buttonLastHandled;  // rising edges
  buttonLastHandled = buttonDebounced;

  while (pressed) {
    uint8_t bit = __builtin_ctz(pressed);
    pressed &= pressed - 1;
    handleButtonPress(bit);
  }

#if SEQ_STEP_SWITCHES_LATCHING
  // Level-based, as in v0.9: step is disabled while its switch reads HIGH
  for (int i = 0; i < 5; i++) {
    seqStepEnabled[i] = !buttonIsDown(SEQ_STEP_BITS[i]);
  }
#endif
}

// ============================================
// PATCH BAY MODULATION
// ============================================
void updateMatrixModulation(float envSig, float pulsarEnvSig) {
  for (int i = 0; i < 7; i++) rawModulationValues[i] = 0.0f;

  for (int src = 0; src < NUM_SOURCES; src++) {
    float v = 0.0f;
    switch (src) {
      case 0: v = activeSeqCV; break;
      case 1: v = envSig; break;
      case 2: v = pulsarEnvSig; break;
      case 3: v = currentRandomValue; break;
    }

    // Different scaling for the pulsar source
    float scaleFactor = (src == 2) ? 100.0f : 1.0f;
    float freqScaleFactor = (src == 2) ? 200.0f : 1.0f;

    if (patchMatrix[src][0]) rawModulationValues[0] += v * scaleFactor;
    if (patchMatrix[src][1]) rawModulationValues[1] += v * freqScaleFactor;
    if (patchMatrix[src][2]) rawModulationValues[2] += v * scaleFactor;
    if (patchMatrix[src][3]) rawModulationValues[3] += v * freqScaleFactor;
    if (patchMatrix[src][4]) rawModulationValues[4] += v * scaleFactor;
    if (patchMatrix[src][5]) rawModulationValues[5] += v * freqScaleFactor;
    if (patchMatrix[src][6]) rawModulationValues[6] += v * freqScaleFactor;
  }
}

// ============================================
// AUDIO PROCESSING
// ============================================
void AudioCallback(float** in, float** out, size_t size) {

  for (size_t i = 0; i < size; i++) {

    // --- 1. TRIGGERS & SEQUENCER ---
    bool seqStepTriggered = false;

    if (currentSeqTriggerMode == SEQ_TRIGGER_CLOCK) {
      if (midiClockSync && midiClockActive && midiClockSamplesPerTick > 0) {
        // MIDI clock sync - step on MIDI-derived interval
        static uint32_t midiSampleCounter = 0;
        midiSampleCounter++;
        if (midiSampleCounter >= midiClockSamplesPerTick) {
          seqStepTriggered = true;
          midiSampleCounter = 0;
        }
      } else if (midiClockSync && !midiClockActive) {
        // Sync enabled but no clock - sequencer stops
      } else {
        // Free-running clock
        seqSampleCounter++;
        if (seqSampleCounter >= samplesPerTick) {
          seqStepTriggered = true;
          seqSampleCounter = 0;
        }
      }
    } else if (currentSeqTriggerMode == SEQ_TRIGGER_PULSAR) {
      bool isPulsarActive = pulsar.IsRunning();
      if (isPulsarActive && !lastPulsarEnvActive) seqStepTriggered = true;
      lastPulsarEnvActive = isPulsarActive;
    } else if (currentSeqTriggerMode == SEQ_TRIGGER_MIDI) {
      if (midiTriggerPending) {
        seqStepTriggered = true;
        midiTriggerPending = false;
      }
    }

    if (seqStepTriggered) {
      seqCurrentStep = (seqCurrentStep + 1) % seqMaxSteps;
      if (seqStepEnabled[seqCurrentStep]) {
        activeSeqCV = seqStepCV[seqCurrentStep];
      } else {
        gateRemainingSamples = 0;
      }
    }

    // --- 2. PULSAR TRIGGER LOGIC ---
    bool pulsarTrigger = false;

    if (manualPulsarTrigger) {
      pulsarTrigger = true;
      manualPulsarTrigger = false;
    } else if (currentPulsarMode == PULSAR_MODE_SEQ) {
      pulsarTrigger = seqStepTriggered;
    } else if (currentPulsarMode == PULSAR_MODE_OSC) {
      if (!pulsar.IsRunning()) pulsarTrigger = true;
    } else if (currentPulsarMode == PULSAR_MODE_MIDI) {
      if (pulsarMidiTrigger) {
        pulsarTrigger = true;
        pulsarMidiTrigger = false;
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
      if (randomMidiTrigger) {
        randomTrigger = true;
        randomMidiTrigger = false;
      }
    }

    if (randomTrigger) {
      currentRandomValue = (float)rand() * (1.0f / RAND_MAX);
    }

    // --- 4. ENVELOPE GENERATION ---
    bool envTrigger = false;

    if (currentEnvTriggerMode == ENV_TRIGGER_SEQ) {
      if (seqStepTriggered) envTrigger = true;
    } else if (currentEnvTriggerMode == ENV_TRIGGER_MIDI) {
      if (envMidiTrigger) {
        envTrigger = true;
        envMidiTrigger = false;
      }
    } else if (currentEnvTriggerMode == ENV_TRIGGER_PULSAR) {
      if (pulsarTrigger) envTrigger = true;
    }

    bool gate = false;

    if (currentEnvMode == ENV_MODE_TRANSIENT) {
      if (envTrigger) {
        gateRemainingSamples = transientGateSamples;
        env.Retrigger(true);
      }
      if (gateRemainingSamples > 0) {
        gate = true;
        gateRemainingSamples--;
      }

    } else if (currentEnvMode == ENV_MODE_SUSTAIN) {
      if (envTrigger) {
        env.Retrigger(true);
        gateRemainingSamples = sustainGateSamples;
      }

      if (currentEnvTriggerMode == ENV_TRIGGER_MIDI) {
        // Gate stays open while the MIDI note is held
        gate = midiNoteHeld;
        gateRemainingSamples = 0;
      } else {
        if (gateRemainingSamples > 0) {
          gate = true;
          gateRemainingSamples--;
        }
      }

    } else if (currentEnvMode == ENV_MODE_OSCILLATE) {
      static uint32_t totalCycleSamples = 0;
      const float SPEED_MULTIPLIER = 0.5f;

      if (!env.IsRunning() && oscGateSamples == 0) {
        env.Retrigger(true);
        totalCycleSamples = (uint32_t)((attackTime + sustainDuration + releaseTime) * sampleRate * SPEED_MULTIPLIER);
        oscGateSamples = totalCycleSamples;
      }

      if (oscGateSamples > 0) {
        gate = true;
        oscGateSamples--;
      } else {
        gate = false;
      }
    }

    float envSig = env.Process(gate);
    float pulsarEnvSig = pulsar.Process(pulsarTrigger);

    // --- 5. UPDATE PATCH BAY MODULATION (less frequently) ---
    if (++modUpdateCounter >= MOD_UPDATE_RATE) {
      updateMatrixModulation(envSig, pulsarEnvSig);
      modUpdateCounter = 0;
    }

    // --- 6. APPLY ATTENUATORS ---
    float pulsarModDepth = potNorm(POT_PULSAR_CTRL);
    pulsarPeriodModCoeff = rawModulationValues[0] * pulsarModDepth;
    modulationFreqCoeff = rawModulationValues[1] * modulationFreqModDepth;
    modulationOscModCoeff = rawModulationValues[2] * modulationOscModCoeffDepth;
    complexOscFreqCoeff = rawModulationValues[3] * complexOscFreqModDepth;
    complexOscWFCoeff = rawModulationValues[4] * complexOscWFModDepth;
    vcaComplexOsc = rawModulationValues[5] * foldedLpgModAmount;
    vcaModulationOsc = rawModulationValues[6] * modOscLpgModAmount;

    // --- 7. OSCILLATORS ---

    // MODULATION OSCILLATOR
    float modFinalFreq;
    if (quantizeToMidi && midiNoteReceived) {
      // Pot frequency is the C4 reference, transposed by the MIDI note
      modFinalFreq = modulationOscFreq * lastMidiNoteFreq * fast_exp2_approx((modulationFreqCoeff * 5.0f) + modulationFine);
    } else {
      modFinalFreq = modulationOscFreq * fast_exp2_approx((modulationFreqCoeff * 5.0f) + modulationFine);
    }
    modulationOsc.SetFreq(modFinalFreq);
    float modOscSig = modulationOsc.Process();

    // Protect SVF from harsh waveforms (saw and square)
    if (modWaveformIndex == 2 || modWaveformIndex == 3) {
      if (modOscSig > 1.2f) modOscSig = 1.2f;
      if (modOscSig < -1.2f) modOscSig = -1.2f;
    }
    if (modOscSig > 1.5f) modOscSig = 1.5f;
    if (modOscSig < -1.5f) modOscSig = -1.5f;

    float totalModDepth = modulationOscMod + (modulationOscModCoeff * 2.0f);
    if (totalModDepth < 0.0f) totalModDepth = 0.0f;
    if (totalModDepth > 2.0f) totalModDepth = 2.0f;

    float fmSignal = 0.0f;
    if (!useAmplitudeMod) fmSignal = modOscSig * totalModDepth;

    // COMPLEX OSCILLATOR
    float compFinalFreq;
    float totalFreqMod = complexOscFreqCoeff + fmSignal;

    if (quantizeToMidi && midiNoteReceived) {
      compFinalFreq = complexOscFreq * lastMidiNoteFreq * fast_exp2_approx(totalFreqMod * 5.0f + complexOscFine);
    } else if (!quantizeToMidi && complexOscMidiEnabled) {
      float rawNote = 24.0f + (potNorm(POT_COMPOSC_FREQ) * 72.0f);
      int quantizedNote = (int)(rawNote + 0.5f);
      compFinalFreq = mtof(quantizedNote) * fast_exp2_approx(totalFreqMod * 5.0f + complexOscFine);
    } else {
      compFinalFreq = complexOscFreq * fast_exp2_approx(totalFreqMod * 5.0f + complexOscFine);
    }

    complexOsc.SetFreq(compFinalFreq);
    complexOscMorph.SetFreq(compFinalFreq);

    float complexSig = complexOsc.Process();
    float morphSig = complexOscMorph.Process();

    float mixedSig = complexSig + (morphSig - complexSig) * complexOscMorphMix;
    if (complexOscInverted) mixedSig = -mixedSig;

    // --- 8. WAVEFOLDER & AM ---
    float wfGain = complexOscWF + (complexOscWFCoeff * 20.0f);
    if (wfGain < 1.0f) wfGain = 1.0f;
    if (wfGain > 20.0f) wfGain = 20.0f;

    waveFolder.SetGain(wfGain);
    float foldedSig = waveFolder.Process(mixedSig);

    if (useAmplitudeMod) {
      foldedSig *= (1.0f + (modOscSig * totalModDepth));
    }

    // --- 9. BUCHLA LPG MIXER ---
    float finalMix = 0.0f;

    // Channel 1: folded complex oscillator
    float ctrl0 = complexOscSigLevel + vcaComplexOsc;
    if (ctrl0 < 0.0f) ctrl0 = 0.0f;
    if (ctrl0 > 0.90f) ctrl0 = 0.90f;

    float processedSig0 = foldedSig;
    if (foldedLpgMode == LPG_MODE_VCA) {
      processedSig0 *= ctrl0 * ctrl0;
    } else {
      if (ctrl0 < 0.0005f) {
        processedSig0 = 0.0f;
      } else {
        int currentMode = (int)foldedLpgMode;
        if (fabsf(ctrl0 - lastCtrl0) > 0.002f || currentMode != lastMode0) {
          lpGateFilter1.SetFreq(fast_lpg_freq(ctrl0));
          float res0 = (foldedLpgMode == LPG_MODE_LP) ? ctrl0 * ctrl0 * 0.2f : 0.1f;
          if (res0 > 0.95f) res0 = 0.95f;
          lpGateFilter1.SetRes(res0);
          lastCtrl0 = ctrl0;
          lastMode0 = currentMode;
        }

        lpGateFilter1.Process(processedSig0);
        processedSig0 = lpGateFilter1.Low();

        if (isBadFloat(processedSig0)) {
          lpGateFilter1.Init(sampleRate);
          processedSig0 = 0.0f;
          lastCtrl0 = -1.0f;
        } else {
          if (processedSig0 > 1.0f) processedSig0 = 1.0f;
          if (processedSig0 < -1.0f) processedSig0 = -1.0f;
        }

        if (foldedLpgMode == LPG_MODE_COMBI) processedSig0 *= ctrl0;
      }
    }
    finalMix += processedSig0;

    // Channel 2: modulation oscillator
    float signal1 = modOscSig;
    if (modWaveformIndex == 2 || modWaveformIndex == 3) {
      if (signal1 > 1.0f) signal1 = 1.0f;
      if (signal1 < -1.0f) signal1 = -1.0f;
    }

    float ctrl1 = modOscSigLevel + vcaModulationOsc;
    if (ctrl1 < 0.0f) ctrl1 = 0.0f;
    if (ctrl1 > 0.90f) ctrl1 = 0.90f;

    float processedSig1 = signal1;
    if (modOscLpgMode == LPG_MODE_VCA) {
      processedSig1 *= ctrl1;
    } else {
      if (ctrl1 < 0.0001f) {
        processedSig1 = 0.0f;
      } else {
        int currentMode = (int)modOscLpgMode;
        if (fabsf(ctrl1 - lastCtrl1) > 0.002f || currentMode != lastMode1) {
          lpGateFilter2.SetFreq(fast_lpg_freq(ctrl1));
          float res1 = (modOscLpgMode == LPG_MODE_LP) ? ctrl1 * ctrl1 * 0.2f : 0.0f;
          if (res1 > 0.95f) res1 = 0.95f;
          lpGateFilter2.SetRes(res1);
          lastCtrl1 = ctrl1;
          lastMode1 = currentMode;
        }

        lpGateFilter2.Process(processedSig1);
        processedSig1 = lpGateFilter2.Low();

        if (isBadFloat(processedSig1)) {
          lpGateFilter2.Init(sampleRate);
          processedSig1 = 0.0f;
          lastCtrl1 = -1.0f;
        } else {
          if (processedSig1 > 1.0f) processedSig1 = 1.0f;
          if (processedSig1 < -1.0f) processedSig1 = -1.0f;
        }

        if (modOscLpgMode == LPG_MODE_COMBI) processedSig1 *= ctrl1;
      }
    }
    finalMix += processedSig1;

    // --- 9.5 CLAMP FINAL MIX ---
    if (isBadFloat(finalMix)) finalMix = 0.0f;
    if (finalMix > 1.0f) finalMix = 1.0f;
    if (finalMix < -1.0f) finalMix = -1.0f;

    // --- 10. OUTPUT ANALOGUE FILTER ---
    finalMix = outputFilter.Process(finalMix);

    // --- 11. REVERB, MASTER VOLUME & OUTPUT ---
    float revL, revR;
    reverb.Process(finalMix, finalMix, &revL, &revR);

    // One-pole smoothing (~20ms) to avoid zipper noise from the 5ms pot scan
    masterVolumeSmoothed += (masterVolume - masterVolumeSmoothed) * 0.001f;
    float outGain = OUTPUT_HEADROOM * masterVolumeSmoothed;

    float dryWet = 1.0f - reverbMix;
    out[0][i] = (finalMix * dryWet + revL * reverbMix) * outGain;
    out[1][i] = (finalMix * dryWet + revR * reverbMix) * outGain;

    pulsarRunning = pulsar.IsRunning();
  }
}

// ============================================
// SETUP
// ============================================
void setup() {
#if WEASEL_DEBUG
  Serial.begin(115200);
#endif
  delay(100);

  // -- PANEL HARDWARE --
  initShiftRegisters();
  initPatchBay();
  initLeds();

  // -- POTS --
  analogReadResolution(16);  // 16-BIT ADC
  initPotentiometers();

  // -- DAISY SEED INIT AT 48kHz --
  hw = DAISY.init(DAISY_SEED, AUDIO_SR_48K);
  sampleRate = DAISY.get_samplerate();
  DAISY.SetAudioBlockSize(64);

  // -- LPG LUT
  for (int i = 0; i < LPG_TABLE_SIZE; i++) {
    float ctrl = (float)i / (float)(LPG_TABLE_SIZE - 1);
    float freq = 20.0f * powf(2.0f, ctrl * 9.81f);
    if (freq < 30.0f) freq = 30.0f;
    if (freq > 18000.0f) freq = 18000.0f;
    lpgFreqTable[i] = freq;
  }

  // -- DSP OBJECTS --
  // COMPLEX OSC // SINE + MORPH
  complexOsc.Init(sampleRate);
  complexOsc.SetWaveform(complexOsc.WAVE_SIN);
  complexOsc.SetFreq(440.0f);
  complexOsc.SetAmp(1.0f);
  complexOscMorph.Init(sampleRate);
  complexOscMorph.SetWaveform(complexOscMorph.WAVE_SAW);  // matches morphWaveformIndex = 1
  complexOscMorph.SetFreq(440.0f);
  complexOscMorph.SetAmp(1.0f);
  // WAVEFOLDER
  waveFolder.Init();
  // MODULATION OSC
  modulationOsc.Init(sampleRate);
  modulationOsc.SetWaveform(modulationOsc.WAVE_SIN);
  modulationOsc.SetFreq(440.0f);
  modulationOsc.SetAmp(1.0f);
  // LPG FILTERS
  lpGateFilter1.Init(sampleRate);
  lpGateFilter2.Init(sampleRate);
  // ENVELOPE
  env.Init(sampleRate);
  gateRemainingSamples = 0;
  // PULSAR ENV
  pulsar.Init(sampleRate);
  pulsar.SetAttackTime(0.02f);
  pulsar.SetDecayTime(0.02f);
  pulsar.SetSustainLevel(1.0f);
  // ANALOGUE OUTPUT FILTER
  outputFilter.Init(sampleRate);
  outputFilter.SetFreq(8000.0f);
  outputFilter.SetRes(0.2f);
  // REVERB
  reverb.Init(sampleRate);
  reverb.SetFeedback(0.85f);
  reverb.SetLpFreq(17000.0f);

  // -- MIDI INIT --
  Serial1.setRx(MIDI_RX_PIN);
  Serial1.begin(31250);
  Serial1.flush();

  MIDI.setHandleNoteOn(handleNoteOn);
  MIDI.setHandleNoteOff(handleNoteOff);
  MIDI.setHandleClock(handleMidiClock);
  MIDI.setHandleStart(handleMidiStart);
  MIDI.setHandleStop(handleMidiStop);
  MIDI.begin(MIDI_CHANNEL_OMNI);
  MIDI.turnThruOff();
  currentMidiNote = 60;
  midiTriggerPending = false;

  // -- INITIAL PANEL STATE (before audio starts) --
  readPotentiometers();
  updateParameters();
  for (int i = 0; i < PATCH_DEBOUNCE_READS + 2; i++) {
    scanPatchMatrix();
  }
  readButtons();
  readButtons();
  buttonLastHandled = buttonDebounced;  // don't treat buttons held at boot as presses
  updateLeds();

  // -- START AUDIO (last, so every DSP object is initialised first) --
  DAISY.begin(AudioCallback);

#if WEASEL_DEBUG
  Serial.print("WEASEL firmware ");
  Serial.println(FIRMWARE_VERSION);
#endif
}

// ============================================
// LOOP
// ============================================
void loop() {
  unsigned long now = millis();

  // MIDI
  while (MIDI.read()) {}
  checkMidiClockActivity();

  // BUTTONS
  if (now - lastButtonRead >= BUTTON_READ_INTERVAL) {
    readButtons();
    lastButtonRead = now;
  }

  // PATCH BAY
  if (now - lastPatchScan >= PATCH_SCAN_INTERVAL) {
    scanPatchMatrix();
    lastPatchScan = now;
  }

  // POTS
  if (now - lastPotRead >= POT_READ_INTERVAL) {
    readPotentiometers();
    updateParameters();
    lastPotRead = now;
  }

  // LEDS
  if (now - lastLedUpdate >= LED_UPDATE_INTERVAL) {
    updateLeds();
    lastLedUpdate = now;
  }
}

// ============================================
// UPDATE PARAMETERS (from pots)
// ============================================
void updateParameters() {
  // FINE TUNE range
  float fineRange = 3.0f / 12.0f;

  // -- COMPLEX OSCILLATOR --
  complexOscFreq = fmap(potNorm(POT_COMPOSC_FREQ), 25.0f, 1760.0f, Mapping::EXP);
  complexOscFreqModDepth = potNorm(POT_COMPOSC_FREQ_CTRL);
  complexOscFine = (potNorm(POT_COMPOSC_FINE) * 2.0f - 1.0f) * fineRange;
  complexOscMorphMix = potNorm(POT_COMPOSC_TIMBRE);
  complexOscWF = 1.0f + (19.0f * potNorm(POT_COMPOSC_WF));
  complexOscWFModDepth = potNorm(POT_COMPOSC_WF_CTRL);

  // LPG 1
  foldedLpgModAmount = fmap(potNorm(POT_LPG1_CTRL), 0.0f, 0.98f, Mapping::LINEAR);
  complexOscSigLevel = fmap(potNorm(POT_LPG1_LEVEL), 0.0f, 0.98f, Mapping::EXP);

  // -- MODULATION OSCILLATOR --
  modulationOscFreq = fmap(potNorm(POT_MODOSC_FREQ), 0.1f, 1000.0f, Mapping::EXP);
  modulationFreqModDepth = potNorm(POT_MODOSC_FREQ_CTRL);
  modulationFine = (potNorm(POT_MODOSC_FINE) * 2.0f - 1.0f) * fineRange;
  modulationOscMod = potNorm(POT_MODOSC_MOD);
  modulationOscModCoeffDepth = potNorm(POT_MODOSC_MOD_CTRL);

  // LPG 2
  modOscLpgModAmount = fmap(potNorm(POT_LPG2_CTRL), 0.0f, 0.98f, Mapping::LINEAR);
  modOscSigLevel = fmap(potNorm(POT_LPG2_LEVEL), 0.0f, 0.98f, Mapping::EXP);

  // -- ENVELOPE [Buchla ASD config] --
  float minTime = 0.002f;
  float maxTime = 5.0f;

  attackTime = fmap(potNorm(POT_ENV_ATTACK), minTime, maxTime, Mapping::EXP);
  env.SetAttackTime(attackTime);

  sustainDuration = fmap(potNorm(POT_ENV_SUSTAIN), 0.02f, maxTime, Mapping::EXP);

  // TRANSIENT mode: gate pulse length = sustain pot value (minimum 20ms)
  transientGateSamples = (uint32_t)(sustainDuration * sampleRate);
  uint32_t minTransientSamples = (uint32_t)(0.02f * sampleRate);
  if (transientGateSamples < minTransientSamples) transientGateSamples = minTransientSamples;

  // SUSTAIN mode: gate length = sustain pot value (minimum 100ms)
  uint32_t minSustainGate = (uint32_t)(0.1f * sampleRate);
  sustainGateSamples = (uint32_t)(sustainDuration * sampleRate);
  if (sustainGateSamples < minSustainGate) sustainGateSamples = minSustainGate;

  // Release Time - reduced range in oscillate mode
  float releasePotNorm = potNorm(POT_ENV_DECAY);
  if (currentEnvMode == ENV_MODE_OSCILLATE) {
    releaseTime = fmap(releasePotNorm, minTime, maxTime * 0.2f, Mapping::EXP);
  } else {
    releaseTime = fmap(releasePotNorm, minTime, maxTime, Mapping::EXP);
  }
  env.SetReleaseTime(releaseTime);
  env.SetDecayTime(0.0f);
  env.SetSustainLevel(1.0f);

  // -- PULSAR --
  float pulsarReleasePot = potNorm(POT_PULSAR_PERIOD);
  float pulsarMinTime = 0.02f;
  float pulsarMaxTime = 5.0f;
  float basePulsarRelease = pulsarMinTime * powf(pulsarMaxTime / pulsarMinTime, pulsarReleasePot);

  float adjustedPulsarTime = basePulsarRelease - (2.0f * pulsarPeriodModCoeff);
  if (adjustedPulsarTime < pulsarMinTime) adjustedPulsarTime = pulsarMinTime;
  if (adjustedPulsarTime > pulsarMaxTime) adjustedPulsarTime = pulsarMaxTime;

  pulsarReleaseTime = adjustedPulsarTime;
  pulsar.SetReleaseTime(pulsarReleaseTime);

  // -- SEQUENCER STEPS --
  for (int i = 0; i < 5; i++) {
    seqStepCV[i] = fmap(potNorm(POT_SEQ_STEP1 + i), 0.0f, 1.0f, Mapping::EXP);
  }

  // -- CLOCK --
  if (midiClockSync) {
    // Pot selects clock division in MIDI sync mode
    updateClockDivision();
  } else {
    float clockPot = potNorm(POT_CLOCK);
    seqClockSpeed = 0.1f * powf(40000.0f, (1.0f - clockPot));
    if (seqClockSpeed < 0.1f) seqClockSpeed = 0.1f;
    if (seqClockSpeed > 4000.0f) seqClockSpeed = 4000.0f;

    samplesPerTick = (uint32_t)((seqClockSpeed / 1000.0f) * sampleRate);
    if (samplesPerTick < 1) samplesPerTick = 1;
  }

  // -- REVERB --
  reverbMix = potNorm(POT_REVERB_MIX);

  // -- MASTER VOLUME --
  // Squared for an audio-taper feel on a linear pot; dead zone at the bottom for true silence
  float vol = potNorm(POT_MASTER_VOLUME);
  if (vol < 0.01f) vol = 0.0f;
  masterVolume = vol * vol;
}

// ============================================
// MIDI
// ============================================
void handleNoteOn(byte channel, byte note, byte velocity) {
  if (velocity > 0) {
    currentMidiNote = note;
    midiTriggerPending = true;
    lastMidiNoteFreq = getMidiTransposeRatio(note);
    midiNoteReceived = true;
    midiNoteHeld = true;
    pulsarMidiTrigger = true;
    randomMidiTrigger = true;
    envMidiTrigger = true;
  } else {
    midiNoteHeld = false;  // Note-on with velocity 0 = note-off
  }
}

void handleNoteOff(byte channel, byte note, byte velocity) {
  midiNoteHeld = false;
}

void handleMidiClock() {
  // 24 ticks per quarter note
  uint32_t now = millis();
  midiClockActive = true;

  if (midiClockSync && lastMidiClockTime > 0) {
    uint32_t tickInterval = now - lastMidiClockTime;
    float quarterNoteInterval = tickInterval * 24.0f;
    if (quarterNoteInterval > 0) {
      midiTempo = 60000.0f / quarterNoteInterval;
      if (midiTempo < 40.0f) midiTempo = 40.0f;
      if (midiTempo > 240.0f) midiTempo = 240.0f;

      float secondsPerTick = 60.0f / (midiTempo * 24.0f);
      midiClockSamplesPerTick = (uint32_t)(secondsPerTick * sampleRate * currentDivision);
      if (midiClockSamplesPerTick < 1) midiClockSamplesPerTick = 1;
    }
  }
  lastMidiClockTime = now;
  midiClockTicks++;
}

void checkMidiClockActivity() {
  if (midiClockSync && (millis() - lastMidiClockTime > MIDI_CLOCK_TIMEOUT)) {
    midiClockActive = false;
    midiClockSamplesPerTick = 0;
  }
}

void updateClockDivision() {
  // REVERSED: 0% = fastest, 100% = slowest
  float p = potNorm(POT_CLOCK);
  currentDivisionIndex = (int)((1.0f - p) * NUM_DIVISIONS);
  if (currentDivisionIndex >= NUM_DIVISIONS) currentDivisionIndex = NUM_DIVISIONS - 1;
  if (currentDivisionIndex < 0) currentDivisionIndex = 0;
  currentDivision = clockDivisions[currentDivisionIndex];
}

void handleMidiStart() {
  seqCurrentStep = 0;
  seqSampleCounter = 0;
  midiClockTicks = 0;
}

void handleMidiStop() {
  seqSampleCounter = 0;
  midiClockTicks = 0;
}