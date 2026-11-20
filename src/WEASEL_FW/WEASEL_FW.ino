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
// 1 = print button presses, patch changes and MIDI input over USB serial (115200 baud)
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
// Prototype boards only. On the current schematic U5 shares MUX1_S0..S3 and these four pins
// are unconnected (free for other uses - drop them from readPotentiometers() if reused).
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
  BTN_PULSAR_MODE = SR_BIT(2, 1),
  BTN_PULSAR_MANUAL = SR_BIT(2, 2),
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
// 1 = latching switches, step enabled/disabled by switch level (see below)
// 0 = momentary buttons, each press toggles the step on/off
#define SEQ_STEP_SWITCHES_LATCHING 1

// Latching mode only: 1 = step ENABLED while its switch reads HIGH, disabled when LOW
//                     0 = step ENABLED while its switch reads LOW (v0.9 behaviour)
#define SEQ_STEP_ENABLED_WHEN_HIGH 0

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
#define AW9523_REG_LEDMODE_P0 0x12  // 0 = LED (constant-current sink) mode, 1 = GPIO
#define AW9523_REG_LEDMODE_P1 0x13
#define AW9523_REG_DIM_BASE 0x20    // 0x20..0x2F, one 0-255 current level per pin
#define AW9523_REG_SOFTWARE_RESET 0x7F

// Board revision LED wiring:
//   0 = prototype boards: AW9523 pin -> LED anode, cathode -> resistor -> GND.
//       Pins run as push-pull GPIO (HIGH = on). LED mode can't be used (it only sinks current),
//       so brightness is set by the series resistors.
//   1 = current schematic: 3V3 -> resistor -> LED anode, cathode -> AW9523 pin.
//       Pins run in constant-current LED mode, brightness set by AW9523_ISEL / LED_BRIGHTNESS.
#define WEASEL_LED_SINK_WIRING 0

// LED current (LED mode only). The AW9523 sinks a regulated current in LED mode, so brightness is set here,
// not by the series resistors. Max per pin = 37mA scaled by ISEL:
//   0 = 37mA, 1 = 27.75mA, 2 = 18.5mA, 3 = 9.25mA
// Pin current = Imax * LED_BRIGHTNESS / 256. ISEL 3 + 128 gives ~4.6mA, roughly half brightness.
const uint8_t AW9523_ISEL = 3;
uint8_t LED_BRIGHTNESS = 128;  // 0-255, can be changed at runtime (set ledForceWrite = true after)

const uint8_t aw9523_addresses[] = { 0x58, 0x59, 0x5A, 0x5B };
const int NUM_CHIPS = 4;
// 400kHz cuts the time LED updates block the main loop by 4x. AW9523 supports 400kHz;
// drop back to 100000 if the LEDs misbehave (long wires / weak pull-ups).
const uint32_t I2C_CLOCK_HZ = 400000;
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
// Sequencer step LEDs (U1 @ 0x58): D9 = P0_6, D14 = P0_7, D15 = P1_0, D16 = P1_1, D17 = P1_2
const LedLocation LED_SEQ_STEP[] = { { 0, 0, 6 }, { 0, 0, 7 }, { 0, 1, 0 }, { 0, 1, 1 }, { 0, 1, 2 } };  // STEP 1..5
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
// Held MIDI notes, oldest first. The gate only closes when the LAST held note is released,
// and releasing the newest note falls back to the previous one (last-note priority).
const uint8_t MAX_HELD_NOTES = 16;
uint8_t heldNotes[MAX_HELD_NOTES];
uint8_t heldNoteCount = 0;

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
const float OUTPUT_HEADROOM = 0.8f;  // doubled from 0.4 to make up for MIX_HEADROOM - same loudness as before

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
volatile int activeSeqStep = 0;  // last ENABLED step reached - the step whose CV is output (drives step LEDs)
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

// -- PARAMETER SMOOTHING (audio-rate copies of pot-driven values) --
// -- LPG STATE --
// Vactrol-style response on the LPG control: fast rise, slower fall. Smooths the
// patch-bay modulation so the gate no longer chatters or steps.
const float LPG_RISE_COEFF = 0.04f;   // ~0.5ms
const float LPG_FALL_COEFF = 0.004f;  // ~5ms
const float LPG_CTRL_MAX = 0.90f;
const uint8_t LPG_PARAM_INTERVAL = 4;  // recompute SVF coefficients every 4 samples
// SVF cubic drive. DaisySP defaults this to 0.5, which audibly saturates near full level.
// 0 = clean. Raise a little (e.g. 0.05-0.1) if you want some grit back.
const float LPG_SVF_DRIVE = 0.0f;

struct LpgChannel {
  Svf* filter;
  float ctrl;
  uint8_t paramCounter;
};
LpgChannel lpg1 = { &lpGateFilter1, 0.0f, 0 };
LpgChannel lpg2 = { &lpGateFilter2, 0.0f, 0 };

// POT SMOOTHING, two stages per parameter, run at audio rate:
//  1. Linear ramp from the previous pot reading to the new one, spread over the time
//     the last pot scan actually took. This removes the scan-rate (~200Hz) ripple a
//     plain one-pole leaves behind - the main cause of audible stepping.
//  2. A short one-pole (PARAM_SMOOTH_MS) that rounds off the corners between ramps.
// Shorter PARAM_SMOOTH_MS = snappier response. 1-3ms is a good range now.
const float PARAM_SMOOTH_MS = 2.0f;
float paramSmoothCoeff = 0.01f;  // computed from PARAM_SMOOTH_MS in setup()

// Pitch pots are smoothed in octaves (log2 Hz) so glides are even across the range
float complexOscPitchOct = 8.0f;
float modulationOscPitchOct = 5.0f;

struct SmoothedParam {
  const float* source;  // pot-derived value written by updateParameters()
  float ramp;           // linearly ramped value
  float inc;            // ramp increment per sample
  float target;         // value this ramp is heading to (latched at ramp start)
  float out;            // final smoothed value used by the DSP
};

enum SmoothedIndex {
  SM_COMP_PITCH, SM_COMP_FINE, SM_MOD_PITCH, SM_MOD_FINE,
  SM_MORPH, SM_WF, SM_MOD_DEPTH, SM_REVERB,
  SM_LPG1_LEVEL, SM_LPG2_LEVEL, SM_LPG1_MOD, SM_LPG2_MOD,
  SM_COMP_FREQ_DEPTH, SM_COMP_WF_DEPTH, SM_MOD_FREQ_DEPTH, SM_MOD_MOD_DEPTH,
  SM_COUNT
};

SmoothedParam smoothed[SM_COUNT] = {
  { &complexOscPitchOct, 0, 0, 0, 0 },
  { &complexOscFine, 0, 0, 0, 0 },
  { &modulationOscPitchOct, 0, 0, 0, 0 },
  { &modulationFine, 0, 0, 0, 0 },
  { &complexOscMorphMix, 0, 0, 0, 0 },
  { &complexOscWF, 0, 0, 0, 0 },
  { &modulationOscMod, 0, 0, 0, 0 },
  { &reverbMix, 0, 0, 0, 0 },
  { &complexOscSigLevel, 0, 0, 0, 0 },
  { &modOscSigLevel, 0, 0, 0, 0 },
  { &foldedLpgModAmount, 0, 0, 0, 0 },
  { &modOscLpgModAmount, 0, 0, 0, 0 },
  { &complexOscFreqModDepth, 0, 0, 0, 0 },
  { &complexOscWFModDepth, 0, 0, 0, 0 },
  { &modulationFreqModDepth, 0, 0, 0, 0 },
  { &modulationOscModCoeffDepth, 0, 0, 0, 0 },
};

// Named views of the smoothed outputs used in the audio callback
float& smComplexOscPitchOct = smoothed[SM_COMP_PITCH].out;
float& smComplexOscFine = smoothed[SM_COMP_FINE].out;
float& smModulationOscPitchOct = smoothed[SM_MOD_PITCH].out;
float& smModulationFine = smoothed[SM_MOD_FINE].out;
float& smComplexOscMorphMix = smoothed[SM_MORPH].out;
float& smComplexOscWF = smoothed[SM_WF].out;
float& smModulationOscMod = smoothed[SM_MOD_DEPTH].out;
float& smReverbMix = smoothed[SM_REVERB].out;
float& smComplexOscSigLevel = smoothed[SM_LPG1_LEVEL].out;
float& smModOscSigLevel = smoothed[SM_LPG2_LEVEL].out;
float& smFoldedLpgModAmount = smoothed[SM_LPG1_MOD].out;
float& smModOscLpgModAmount = smoothed[SM_LPG2_MOD].out;
float& smComplexOscFreqModDepth = smoothed[SM_COMP_FREQ_DEPTH].out;
float& smComplexOscWFModDepth = smoothed[SM_COMP_WF_DEPTH].out;
float& smModulationFreqModDepth = smoothed[SM_MOD_FREQ_DEPTH].out;
float& smModulationOscModCoeffDepth = smoothed[SM_MOD_MOD_DEPTH].out;

// Handshake between the pot scan (loop) and the audio callback
volatile uint32_t paramVersion = 0;        // bumped after each updateParameters()
uint32_t paramVersionSeen = 0;             // last version the callback ramped towards
volatile uint32_t paramRampSamples = 240;  // measured pot-scan interval, in samples
unsigned long lastPotScanMicros = 0;

// Start every smoothed parameter exactly at its pot position (no glide at power-up)
void snapSmoothedParams() {
  for (int i = 0; i < SM_COUNT; i++) {
    smoothed[i].ramp = *smoothed[i].source;
    smoothed[i].out = smoothed[i].ramp;
    smoothed[i].inc = 0.0f;
    smoothed[i].target = smoothed[i].ramp;
  }
  paramVersionSeen = paramVersion;
}

// Called at the start of each audio block: if new pot values arrived, set up ramps to them
inline void startParamRamps() {
  uint32_t v = paramVersion;
  if (v == paramVersionSeen) return;
  paramVersionSeen = v;
  float invLen = 1.0f / (float)paramRampSamples;
  for (int i = 0; i < SM_COUNT; i++) {
    smoothed[i].target = *smoothed[i].source;
    smoothed[i].inc = (smoothed[i].target - smoothed[i].ramp) * invLen;
  }
}

// Called every sample
inline void tickSmoothedParams() {
  const float k = paramSmoothCoeff;
  for (int i = 0; i < SM_COUNT; i++) {
    SmoothedParam& p = smoothed[i];
    if (p.inc != 0.0f) {
      p.ramp += p.inc;
      // stop exactly at the target instead of overshooting
      if ((p.inc > 0.0f && p.ramp >= p.target) || (p.inc < 0.0f && p.ramp <= p.target)) {
        p.ramp = p.target;
        p.inc = 0.0f;
      }
    }
    p.out += k * (p.ramp - p.out);
  }
}


// -- MIX --
const float MIX_HEADROOM = 0.5f;  // the two LPG channels are summed at half level

// -- FORWARD DECLARATIONS --
void updateClockDivision();
void updateParameters();
void checkMidiClockActivity();
void handleNoteOn(byte channel, byte note, byte velocity);
void handleNoteOff(byte channel, byte note, byte velocity);
void handleMidiClock();
void handleMidiStart();
void handleMidiStop();
void handleControlChange(byte channel, byte number, byte value);
void configureMidiInterruptPriority();
void drainMidi();
#if WEASEL_DEBUG
void handleMidiSysExDebug(byte* data, unsigned size);
#endif

// -- MATH HELPERS --
// Exponential pitch scaling. The old Taylor approximation went NEGATIVE for exponents
// between about -1.4 and -2.9 (and grew again below that), so deep FM produced negative
// frequencies. DaisySP oscillators never wrap a negative phase, so the phase ran away:
// saw/tri outputs went out of range and the sine lost precision over time (noise/flutter).
// exp2f is exact and cheap on the M7 FPU. Takes absolute pitch in octaves (log2 Hz).
inline float pitchExp2(float octaves) {
  if (octaves > 15.0f) octaves = 15.0f;    // 32kHz - clampOscFreq() trims to below Nyquist
  if (octaves < -10.0f) octaves = -10.0f;
  return exp2f(octaves);
}

// Keeps oscillator frequency positive and below Nyquist
inline float clampOscFreq(float f) {
  if (f < 0.0f) return 0.0f;
  float maxF = sampleRate * 0.45f;
  return (f > maxF) ? maxF : f;
}

// Transparent below 0.8, then a smooth knee that approaches +/-1.
// Replaces the hard +/-1 clamp on the mix, which squared off peaks whenever
// both LPG channels were loud at once.
inline float softKnee(float x) {
  const float t = 0.8f;
  float a = fabsf(x);
  if (a <= t) return x;
  float over = (a - t) / (1.0f - t);
  float y = t + (1.0f - t) * (over / (1.0f + over));
  return (x < 0.0f) ? -y : y;
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

// Pot noise filtering: one-pole low-pass on the raw ADC value, then hysteresis so a pot
// at rest stops twitching, and snapping at both ends so minimum/maximum are exact
// (e.g. wavefolder gain sits at exactly 1.0 and morph mix at exactly 0 with the pot down).
// POT_FILTER_COEFF: noise filter on the raw ADC, per 5ms scan. Higher = less lag, more noise.
//   0.5 settles in ~15ms. (Total glide = this + PARAM_SMOOTH_MS.)
// Hysteresis only applies to a pot at rest: once a pot moves past POT_HYSTERESIS it tracks
// every scan (no stepping) until it has been still for POT_ACTIVE_MS, then freezes again.
const float POT_FILTER_COEFF = 0.5f;
const int POT_HYSTERESIS = 48;            // in 16-bit LSBs (~0.07% of travel)
const unsigned long POT_ACTIVE_MS = 300;  // keep tracking this long after the last movement
unsigned long potActiveUntil[TOTAL_POTS] = { 0 };
const int POT_END_SNAP = 200;          // dead zone at each end of travel
float potFiltered[TOTAL_POTS] = { 0 };
bool potFilterPrimed = false;

inline void filterPot(int i, int raw) {
  if (!potFilterPrimed) potFiltered[i] = (float)raw;
  else potFiltered[i] += POT_FILTER_COEFF * ((float)raw - potFiltered[i]);

  int v = (int)(potFiltered[i] + 0.5f);
  if (v < POT_END_SNAP) v = 0;
  else if (v > 65535 - POT_END_SNAP) v = 65535;

  unsigned long now = millis();
  if (abs(v - potValues[i]) >= POT_HYSTERESIS) {
    potActiveUntil[i] = now + POT_ACTIVE_MS;  // pot is being turned
  }
  if (v == 0 || v == 65535 || (long)(potActiveUntil[i] - now) > 0) {
    potValues[i] = v;  // track continuously while moving
  }
}

// Both muxes are switched to the same channel together and then both ADCs are read.
// Current schematic: U4 and U5 share MUX1_S0..S3 (D4/D5/D6/D10 are unconnected on the board).
// Prototype boards: U5 still has its own MUX2_S0..S3 lines, so those are driven to the same
// channel as well - the same code works on both boards.
void readPotentiometers() {
  for (int ch = 0; ch < NUM_POTS_PER_MUX; ch++) {
    selectMuxChannel(ch, MUX1_S0, MUX1_S1, MUX1_S2, MUX1_S3);
    selectMuxChannel(ch, MUX2_S0, MUX2_S1, MUX2_S2, MUX2_S3);  // prototype only, harmless otherwise
    delayMicroseconds(20);  // settle after switching the mux
    int raw1 = analogRead(MUX1_SIG);
    int raw2 = analogRead(MUX2_SIG);
    filterPot(ch, raw1);
    filterPot(ch + NUM_POTS_PER_MUX, raw2);
  }
  potFilterPrimed = true;
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
    drainMidi();
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

#if !WEASEL_LED_SINK_WIRING
  // Prototype: all pins GPIO outputs (default after reset), Port 0 push-pull, all off
  Wire.beginTransmission(address);
  Wire.write(AW9523_REG_GLOBAL_CTRL);
  Wire.write(1 << 4);  // GPOMD: Port 0 push-pull (Port 1 is always push-pull)
  if (Wire.endTransmission() != 0) return false;

  Wire.beginTransmission(address);
  Wire.write(AW9523_REG_CONFIG);
  Wire.write(0x00);
  Wire.write(0x00);
  if (Wire.endTransmission() != 0) return false;

  Wire.beginTransmission(address);
  Wire.write(AW9523_REG_OUTPUT);
  Wire.write(0x00);
  Wire.write(0x00);
  if (Wire.endTransmission() != 0) return false;

  return true;
#else
  // Global control: current range (ISEL bits 1:0). GPOMD is irrelevant in LED mode.
  Wire.beginTransmission(address);
  Wire.write(AW9523_REG_GLOBAL_CTRL);
  Wire.write(AW9523_ISEL & 0x03);
  if (Wire.endTransmission() != 0) return false;

  // All 16 pins to LED mode (0x12 and 0x13, auto-increment)
  Wire.beginTransmission(address);
  Wire.write(AW9523_REG_LEDMODE_P0);
  Wire.write(0x00);
  Wire.write(0x00);
  if (Wire.endTransmission() != 0) return false;

  // All LEDs off
  Wire.beginTransmission(address);
  Wire.write(AW9523_REG_DIM_BASE);
  for (int i = 0; i < 16; i++) Wire.write(0x00);
  if (Wire.endTransmission() != 0) return false;

  return true;
#endif
}

void initLeds() {
  Wire.begin();
  Wire.setClock(I2C_CLOCK_HZ);  // must come after begin() - begin() resets it to 100kHz
  for (int chip = 0; chip < NUM_CHIPS; chip++) {
    chipPresent[chip] = initializeAW9523(aw9523_addresses[chip]);
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
  SET_LED(LED_SEQ_STEP, activeSeqStep);
}

// Only writes chips whose outputs changed
void updateLeds() {
  buildLedFrame();
  for (int chip = 0; chip < NUM_CHIPS; chip++) {
    if (!chipPresent[chip]) continue;
    if (!ledForceWrite && ledFrame[chip][0] == ledWritten[chip][0] && ledFrame[chip][1] == ledWritten[chip][1]) continue;

    const uint8_t p0 = ledFrame[chip][0];
    const uint8_t p1 = ledFrame[chip][1];
    Wire.beginTransmission(aw9523_addresses[chip]);
#if !WEASEL_LED_SINK_WIRING
    Wire.write(AW9523_REG_OUTPUT);
    Wire.write(p0);
    Wire.write(p1);
#else
    // DIM register order: 0x20-0x23 = P1_0..P1_3, 0x24-0x2B = P0_0..P0_7, 0x2C-0x2F = P1_4..P1_7
    Wire.write(AW9523_REG_DIM_BASE);
    for (int b = 0; b < 4; b++) Wire.write((p1 >> b) & 1 ? LED_BRIGHTNESS : 0);
    for (int b = 0; b < 8; b++) Wire.write((p0 >> b) & 1 ? LED_BRIGHTNESS : 0);
    for (int b = 4; b < 8; b++) Wire.write((p1 >> b) & 1 ? LED_BRIGHTNESS : 0);
#endif
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
      else if (modWaveformIndex == 2) modulationOsc.SetWaveform(modulationOsc.WAVE_POLYBLEP_SAW);
      else modulationOsc.SetWaveform(modulationOsc.WAVE_POLYBLEP_SQUARE);
      break;

    case BTN_COMPOSC_POLARITY:
      complexOscInverted = !complexOscInverted;
      break;

    case BTN_COMPOSC_WAVE:
      // TRI -> SAW -> SQUARE
      morphWaveformIndex = (morphWaveformIndex + 1) % 3;
      if (morphWaveformIndex == 0) complexOscMorph.SetWaveform(complexOscMorph.WAVE_TRI);
      else if (morphWaveformIndex == 1) complexOscMorph.SetWaveform(complexOscMorph.WAVE_POLYBLEP_SAW);
      else complexOscMorph.SetWaveform(complexOscMorph.WAVE_POLYBLEP_SQUARE);
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
  // Level-based step enable
  for (int i = 0; i < 5; i++) {
#if SEQ_STEP_ENABLED_WHEN_HIGH
    seqStepEnabled[i] = buttonIsDown(SEQ_STEP_BITS[i]);
#else
    seqStepEnabled[i] = !buttonIsDown(SEQ_STEP_BITS[i]);
#endif
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
// One Buchla-style LPG channel. Replaces the old per-channel code, which:
//  - hard-muted below a tiny threshold (chattering on/off as modulation hovered near zero),
//  - only retuned the filter when the control moved by > 0.002 (audible steps),
//  - and ran the SVF at its default drive of 0.5 (distortion).
float processLpg(LpgChannel& ch, float in, float ctrlTarget, LpgMode mode,
                 bool squaredVca, float combiRes) {
  if (ctrlTarget < 0.0f) ctrlTarget = 0.0f;
  if (ctrlTarget > LPG_CTRL_MAX) ctrlTarget = LPG_CTRL_MAX;

  // Vactrol-style smoothing
  float coeff = (ctrlTarget > ch.ctrl) ? LPG_RISE_COEFF : LPG_FALL_COEFF;
  ch.ctrl += coeff * (ctrlTarget - ch.ctrl);
  float ctrl = ch.ctrl;

  if (mode == LPG_MODE_VCA) {
    return in * (squaredVca ? ctrl * ctrl : ctrl);
  }

  if (++ch.paramCounter >= LPG_PARAM_INTERVAL) {
    ch.paramCounter = 0;
    float res = (mode == LPG_MODE_LP) ? ctrl * ctrl * 0.2f : combiRes;
    ch.filter->SetRes(res);
    ch.filter->SetFreq(fast_lpg_freq(ctrl));
  }

  ch.filter->Process(in);
  float out = ch.filter->Low();

  if (isBadFloat(out)) {
    ch.filter->Init(sampleRate);
    ch.filter->SetDrive(LPG_SVF_DRIVE);
    ch.paramCounter = LPG_PARAM_INTERVAL;  // force a coefficient update next sample
    return 0.0f;
  }

  if (mode == LPG_MODE_COMBI) {
    out *= ctrl;
  } else {
    // LP mode: short fade to silence at the very bottom instead of a hard mute
    float fade = ctrl * 50.0f;
    if (fade < 1.0f) out *= fade;
  }
  return out;
}

void AudioCallback(float** in, float** out, size_t size) {
  startParamRamps();  // pick up any new pot values for this block

  // A note-on sets every module's MIDI trigger flag, but only modules in MIDI mode use theirs.
  // Clear the rest so switching a module to MIDI later doesn't fire one stale trigger.
  if (currentSeqTriggerMode != SEQ_TRIGGER_MIDI) midiTriggerPending = false;
  if (currentEnvTriggerMode != ENV_TRIGGER_MIDI) envMidiTrigger = false;
  if (currentPulsarMode != PULSAR_MODE_MIDI) pulsarMidiTrigger = false;
  if (currentRandomMode != RANDOM_MODE_MIDI) randomMidiTrigger = false;


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
        activeSeqStep = seqCurrentStep;  // disabled steps leave the previous step lit
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

    // --- 5. UPDATE PATCH BAY MODULATION (every sample - no 16-sample staircase) ---
    updateMatrixModulation(envSig, pulsarEnvSig);

    // --- 5.5 SMOOTH POT-DRIVEN PARAMETERS ---
    tickSmoothedParams();

    // --- 6. APPLY ATTENUATORS ---
    float pulsarModDepth = potNorm(POT_PULSAR_CTRL);
    pulsarPeriodModCoeff = rawModulationValues[0] * pulsarModDepth;
    modulationFreqCoeff = rawModulationValues[1] * smModulationFreqModDepth;
    modulationOscModCoeff = rawModulationValues[2] * smModulationOscModCoeffDepth;
    complexOscFreqCoeff = rawModulationValues[3] * smComplexOscFreqModDepth;
    complexOscWFCoeff = rawModulationValues[4] * smComplexOscWFModDepth;
    vcaComplexOsc = rawModulationValues[5] * smFoldedLpgModAmount;
    vcaModulationOsc = rawModulationValues[6] * smModOscLpgModAmount;

    // --- 7. OSCILLATORS ---

    // MODULATION OSCILLATOR
    float modExp = (modulationFreqCoeff * 5.0f) + smModulationFine;
    float modFinalFreq = pitchExp2(smModulationOscPitchOct + modExp);
    if (quantizeToMidi && midiNoteReceived) {
      // Pot frequency is the C4 reference, transposed by the MIDI note
      modFinalFreq *= lastMidiNoteFreq;
    }
    modulationOsc.SetFreq(clampOscFreq(modFinalFreq));
    float modOscSig = modulationOsc.Process();

    // Safety clamp (band-limited saw/square overshoot slightly)
    if (modOscSig > 1.2f) modOscSig = 1.2f;
    if (modOscSig < -1.2f) modOscSig = -1.2f;

    float totalModDepth = smModulationOscMod + (modulationOscModCoeff * 2.0f);
    if (totalModDepth < 0.0f) totalModDepth = 0.0f;
    if (totalModDepth > 2.0f) totalModDepth = 2.0f;

    float fmSignal = 0.0f;
    if (!useAmplitudeMod) fmSignal = modOscSig * totalModDepth;

    // COMPLEX OSCILLATOR
    float compExp = (complexOscFreqCoeff + fmSignal) * 5.0f + smComplexOscFine;
    float compFinalFreq;

    if (quantizeToMidi && midiNoteReceived) {
      compFinalFreq = pitchExp2(smComplexOscPitchOct + compExp) * lastMidiNoteFreq;
    } else if (!quantizeToMidi && complexOscMidiEnabled) {
      float rawNote = 24.0f + (potNorm(POT_COMPOSC_FREQ) * 72.0f);
      int quantizedNote = (int)(rawNote + 0.5f);
      compFinalFreq = mtof(quantizedNote) * pitchExp2(compExp);
    } else {
      compFinalFreq = pitchExp2(smComplexOscPitchOct + compExp);
    }
    compFinalFreq = clampOscFreq(compFinalFreq);

    complexOsc.SetFreq(compFinalFreq);
    complexOscMorph.SetFreq(compFinalFreq);

    float complexSig = complexOsc.Process();
    float morphSig = complexOscMorph.Process();

    float mixedSig = complexSig + (morphSig - complexSig) * smComplexOscMorphMix;
    if (complexOscInverted) mixedSig = -mixedSig;

    // --- 8. WAVEFOLDER & AM ---
    float wfGain = smComplexOscWF + (complexOscWFCoeff * 20.0f);
    if (wfGain < 1.0f) wfGain = 1.0f;
    if (wfGain > 20.0f) wfGain = 20.0f;

    waveFolder.SetGain(wfGain);
    float foldedSig = waveFolder.Process(mixedSig);

    if (useAmplitudeMod) {
      // Normalised so full AM depth no longer drives the LPG up to 3x over level
      foldedSig *= (1.0f + (modOscSig * totalModDepth)) / (1.0f + totalModDepth);
    }

    // --- 9. BUCHLA LPG MIXER ---
    // Channel 1: folded complex oscillator (squared VCA law, light resonance in COMBI)
    float processedSig0 = processLpg(lpg1, foldedSig, smComplexOscSigLevel + vcaComplexOsc,
                                     foldedLpgMode, true, 0.1f);

    // Channel 2: modulation oscillator (linear VCA law, no resonance in COMBI)
    float signal1 = modOscSig;
    if (signal1 > 1.0f) signal1 = 1.0f;
    if (signal1 < -1.0f) signal1 = -1.0f;
    float processedSig1 = processLpg(lpg2, signal1, smModOscSigLevel + vcaModulationOsc,
                                     modOscLpgMode, false, 0.0f);

    // --- 9.5 MIX WITH HEADROOM ---
    // Sum at half level so two full channels don't clip, then a soft knee as a safety net
    float finalMix = (processedSig0 + processedSig1) * MIX_HEADROOM;
    if (isBadFloat(finalMix)) finalMix = 0.0f;
    finalMix = softKnee(finalMix);

    // --- 10. OUTPUT ANALOGUE FILTER ---
    finalMix = outputFilter.Process(finalMix);

    // --- 11. REVERB, MASTER VOLUME & OUTPUT ---
    float revL, revR;
    reverb.Process(finalMix, finalMix, &revL, &revR);

    // One-pole smoothing (~20ms) to avoid zipper noise from the 5ms pot scan
    masterVolumeSmoothed += (masterVolume - masterVolumeSmoothed) * 0.001f;
    float outGain = OUTPUT_HEADROOM * masterVolumeSmoothed;

    float dryWet = 1.0f - smReverbMix;
    out[0][i] = (finalMix * dryWet + revL * smReverbMix) * outGain;
    out[1][i] = (finalMix * dryWet + revR * smReverbMix) * outGain;

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

  // One-pole coefficient for the requested glide time (time constant = PARAM_SMOOTH_MS)
  paramSmoothCoeff = 1.0f - expf(-1.0f / (PARAM_SMOOTH_MS * 0.001f * sampleRate));

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
  complexOscMorph.SetWaveform(complexOscMorph.WAVE_POLYBLEP_SAW);  // matches morphWaveformIndex = 1
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
  lpGateFilter1.SetDrive(LPG_SVF_DRIVE);
  lpGateFilter2.SetDrive(LPG_SVF_DRIVE);
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
  MIDI.setHandleControlChange(handleControlChange);
#if WEASEL_DEBUG
  MIDI.setHandleSystemExclusive(handleMidiSysExDebug);
#endif
  MIDI.begin(MIDI_CHANNEL_OMNI);
  MIDI.turnThruOff();
  currentMidiNote = 60;
  midiTriggerPending = false;

  // -- INITIAL PANEL STATE (before audio starts) --
  readPotentiometers();
  updateParameters();

  // Start the smoothed parameters at the pot positions so nothing glides at power-up
  snapSmoothedParams();

  for (int i = 0; i < PATCH_DEBOUNCE_READS + 2; i++) {
    scanPatchMatrix();
  }
  readButtons();
  readButtons();
  buttonLastHandled = buttonDebounced;  // don't treat buttons held at boot as presses
  updateLeds();

  // -- START AUDIO (last, so every DSP object is initialised first) --
  DAISY.begin(AudioCallback);
  configureMidiInterruptPriority();  // after DAISY.begin(), which sets the audio DMA priorities

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
  drainMidi();
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

    // Ramp the audio-rate parameters over the time this scan actually took to come round,
    // so the ramps join up end to end even when the loop is held up by other work
    unsigned long nowUs = micros();
    float intervalSamples = (float)(nowUs - lastPotScanMicros) * 1e-6f * sampleRate;
    lastPotScanMicros = nowUs;
    if (intervalSamples < 48.0f) intervalSamples = 48.0f;    // 1ms
    if (intervalSamples > 960.0f) intervalSamples = 960.0f;  // 20ms
    paramRampSamples = (uint32_t)intervalSamples;
    paramVersion = paramVersion + 1;
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
  complexOscPitchOct = log2f(complexOscFreq);  // smoothed in octaves in the audio callback
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
  modulationOscPitchOct = log2f(modulationOscFreq);
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
#if WEASEL_DEBUG
const char* DEBUG_NOTE_NAMES[12] = { "C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B" };

void debugPrintNote(const char* label, byte channel, byte note, byte velocity) {
  Serial.print(label);
  Serial.print(" ch");
  Serial.print(channel);
  Serial.print(" ");
  Serial.print(DEBUG_NOTE_NAMES[note % 12]);
  Serial.print(note / 12 - 1);
  Serial.print(" (");
  Serial.print(note);
  Serial.print(") vel ");
  Serial.println(velocity);
}

// WEASEL ignores SysEx. Seeing it here with a controller that doesn't send SysEx means
// bytes are being corrupted on the MIDI input (e.g. clock F8 arriving as F0).
void handleMidiSysExDebug(byte* data, unsigned size) {
  Serial.print("MIDI SysEx (");
  Serial.print(size);
  Serial.println(" bytes) - if your controller doesn't send SysEx, the MIDI input is corrupting bytes");
}
#endif

// ============================================
// MIDI INPUT
// ============================================
// The audio callback runs inside the SAI DMA interrupt for a whole block (~1.3ms at 64
// samples), which DaisyDuino sets to priority 0. The MIDI UART interrupt was priority 1, so
// it couldn't run during the callback, and the UART only holds one received byte. At
// 31250 baud a new byte arrives every 320us, so bytes landing mid-callback overwrote each
// other (UART overrun) and were lost: missed note-ons, missed note-offs (stuck notes).
// Give the UART the top priority and drop the audio DMA one step. The UART interrupt
// only copies one byte into a buffer, so audio timing is unaffected.
void configureMidiInterruptPriority() {
  const IRQn_Type audioDma[] = { DMA1_Stream0_IRQn, DMA1_Stream1_IRQn,    // SAI1
                                 DMA1_Stream3_IRQn, DMA1_Stream4_IRQn };  // SAI2
  for (unsigned i = 0; i < sizeof(audioDma) / sizeof(audioDma[0]); i++) {
    HAL_NVIC_SetPriority(audioDma[i], 1, 0);
  }
  HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
}

// MIDI.read() parses one byte per call, and returns false until a message is complete, so
// `while (MIDI.read())` stopped at the first byte of every note and took three passes of the
// main loop to read one. Read everything that has arrived instead.
void drainMidi() {
  while (Serial1.available()) {
    MIDI.read();
  }
}

void heldNoteRemove(byte note) {
  for (uint8_t i = 0; i < heldNoteCount; i++) {
    if (heldNotes[i] == note) {
      for (uint8_t j = i; j + 1 < heldNoteCount; j++) heldNotes[j] = heldNotes[j + 1];
      heldNoteCount--;
      return;
    }
  }
}

void midiNoteOn(byte note) {
  heldNoteRemove(note);  // a repeated note moves to the top
  if (heldNoteCount == MAX_HELD_NOTES) heldNoteRemove(heldNotes[0]);  // forget the oldest
  heldNotes[heldNoteCount++] = note;

  currentMidiNote = note;
  midiTriggerPending = true;
  lastMidiNoteFreq = getMidiTransposeRatio(note);
  midiNoteReceived = true;
  midiNoteHeld = true;
  pulsarMidiTrigger = true;
  randomMidiTrigger = true;
  envMidiTrigger = true;
}

void midiNoteOff(byte note) {
  bool wasNewest = (heldNoteCount > 0 && heldNotes[heldNoteCount - 1] == note);
  heldNoteRemove(note);
  if (heldNoteCount == 0) {
    midiNoteHeld = false;
  } else if (wasNewest) {
    // Fall back to the previous held note: pitch only, the gate stays open, no retrigger
    currentMidiNote = heldNotes[heldNoteCount - 1];
    lastMidiNoteFreq = getMidiTransposeRatio(currentMidiNote);
  }
}

void allMidiNotesOff() {
  heldNoteCount = 0;
  midiNoteHeld = false;
}

void handleNoteOn(byte channel, byte note, byte velocity) {
#if WEASEL_DEBUG
  debugPrintNote(velocity > 0 ? "MIDI note on " : "MIDI note off", channel, note, velocity);
#endif
  if (velocity > 0) {
    midiNoteOn(note);
  } else {
    midiNoteOff(note);  // Note-on with velocity 0 = note-off
  }
}

void handleNoteOff(byte channel, byte note, byte velocity) {
#if WEASEL_DEBUG
  debugPrintNote("MIDI note off", channel, note, velocity);
#endif
  midiNoteOff(note);
}

// CC 120 (All Sound Off) and CC 123 (All Notes Off): clear any stuck notes
void handleControlChange(byte channel, byte number, byte value) {
  if (number == 120 || number == 123) {
    allMidiNotesOff();
#if WEASEL_DEBUG
    Serial.println("MIDI all notes off");
#endif
  }
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
#if WEASEL_DEBUG
  // Report clock starting/stopping, whether or not clock sync is switched on
  static bool debugClockRunning = false;
  bool running = lastMidiClockTime > 0 && (millis() - lastMidiClockTime) <= MIDI_CLOCK_TIMEOUT;
  if (running != debugClockRunning) {
    debugClockRunning = running;
    Serial.println(running ? "MIDI clock started" : "MIDI clock stopped");
  }
#endif
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