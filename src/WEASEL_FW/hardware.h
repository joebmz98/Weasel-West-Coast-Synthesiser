// =============================================================================
// hardware.h — pin map, pot map, button map and LED map for the v1.0 board
//
// Daisy D-number = physical pin - 1. The STM32duino Daisy Seed variant uses the
// D numbers as Arduino pin numbers.
//
// v1.0 hardware changes vs v0.9:
//  - Button matrices replaced by 4x 74HC165 shift registers (32-bit chain):
//    SR1..SR3 = panel buttons, SR4 = patch bay destinations
//  - Patch bay sources driven from D17..D20, destinations read on SR4
//  - Mode LEDs driven by 4x AW9523 (0x58..0x5B) over I2C (D11 SCL / D12 SDA)
//  - MUX2_S3 moved from D7 to D10 (D7 is now the 74HC165 SH/LD line)
//  - Master volume pot on MUX2 C11
// =============================================================================
#pragma once
#include <stdint.h>

namespace weasel
{
namespace pins
{
// --- 2x CD74HC4067 pot muxes ---------------------------------------------------
// Current schematic: U4 and U5 share MUX1_S0..S3. The MUX2 select pins are only
// wired on the prototype boards; driving them anyway is harmless.
const uint32_t kMux1Sel[4] = { D0, D1, D2, D3 };
const uint32_t kMux2Sel[4] = { D4, D5, D6, D10 };  // S3 moved from D7 (SPI1_MOSI)
const uint32_t kMux1Sig    = A0;
const uint32_t kMux2Sig    = A1;

// --- 74HC165 chain --------------------------------------------------------------
const uint32_t kShiftLoad  = D7;  // SH/LD, physical pin 8
const uint32_t kShiftClock = D8;  // physical pin 9
const uint32_t kShiftData  = D9;  // QH, physical pin 10

// --- Patch bay source drive pins (SEQ, EG, PLSR, RNDM) ----------------------------
const uint32_t kPatchSources[4] = { D17, D18, D19, D20 };  // physical pins 24..27

// --- MIDI in: 6N138 output on USART1 RX ---------------------------------------------
const uint32_t kMidiRx = 14;  // D14, physical pin 15

// I2C uses the default Wire pins: D11 (SCL) / D12 (SDA)
}  // namespace pins

// =============================================================================
// POTS (index into Pots: MUX1 = 0..15, MUX2 = 16..31)
// =============================================================================
enum PotIndex : uint8_t
{
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

// =============================================================================
// BUTTONS (bit in the 32-bit shift register read)
// SR1 = bits 31..24, SR2 = 23..16, SR3 = 15..8, SR4 = 7..0;  SRn Dk -> bit 32 - 8n + k
// =============================================================================
constexpr uint8_t srBit(int sr, int d) { return (uint8_t)(32 - 8 * sr + d); }

enum ButtonBit : uint8_t
{
  // SR1
  BTN_SEQ_TRIG_MODE = srBit(1, 0),
  BTN_SEQ_STAGES = srBit(1, 1),
  BTN_SEQ_STEP1 = srBit(1, 2),
  BTN_SEQ_STEP2 = srBit(1, 3),
  BTN_SEQ_STEP3 = srBit(1, 4),
  BTN_SEQ_STEP4 = srBit(1, 5),
  BTN_SEQ_STEP5 = srBit(1, 6),
  BTN_ENV_TRIG_MODE = srBit(1, 7),
  // SR2
  BTN_ENV_MODE = srBit(2, 0),
  BTN_PULSAR_MODE = srBit(2, 1),
  BTN_PULSAR_MANUAL = srBit(2, 2),
  BTN_MODOSC_AM_FM = srBit(2, 3),
  BTN_MODOSC_WAVE = srBit(2, 4),
  BTN_COMPOSC_POLARITY = srBit(2, 5),
  BTN_COMPOSC_WAVE = srBit(2, 6),
  BTN_QUANTIZE_MIDI = srBit(2, 7),
  // SR3
  BTN_LPG1_MODE = srBit(3, 0),
  BTN_LPG2_MODE = srBit(3, 1),
  BTN_RANDOM_MODE = srBit(3, 2),
  BTN_CLOCK_SYNC = srBit(3, 3)
};

// Only the button bits (SR1..SR3, SR3 D0-D3) are considered for edge detection
const uint32_t kButtonMask = 0xFFFF0F00UL;

const uint8_t kSeqStepBits[5] = { BTN_SEQ_STEP1, BTN_SEQ_STEP2, BTN_SEQ_STEP3, BTN_SEQ_STEP4, BTN_SEQ_STEP5 };

// Hold MIDI for a moment to enter the MIDI shift (buttons.h)
const uint8_t kShiftButton = BTN_QUANTIZE_MIDI;
// Hold this button while powering up to forget the saved panel modes
const uint8_t kPanelResetButton = BTN_PULSAR_MANUAL;

// =============================================================================
// PATCH BAY (4 sources x 7 destinations, read on SR4 D0-D6)
// =============================================================================
const int kPatchSourceCount = 4;  // 0 SEQ, 1 EG, 2 PLSR, 3 RNDM
const int kPatchDestCount = 7;    // see PatchDest
enum PatchDest : uint8_t
{
  DEST_PULSAR_PERIOD = 0,
  DEST_MODOSC_FREQ,
  DEST_MODOSC_MOD,
  DEST_COMP_PITCH,
  DEST_COMP_WAVE,
  DEST_LPG1_LEVEL,
  DEST_LPG2_LEVEL
};

// =============================================================================
// LEDS (AW9523 chip 0..3 = 0x58..0x5B, port 0/1, bit)
// Entry N of a group lights when that function is in state N. Reorder entries
// here if the panel order differs from the firmware state order.
// =============================================================================
const uint8_t kLedChipAddress[4] = { 0x58, 0x59, 0x5A, 0x5B };
const int kLedChipCount = 4;

struct LedLocation
{
  uint8_t chip;
  uint8_t port;
  uint8_t bit;
};

namespace leds
{
const LedLocation SEQ_TRIG[] = { { 0, 0, 0 }, { 0, 0, 1 }, { 0, 0, 2 } };    // CLOCK, PULSAR, MIDI
const LedLocation SEQ_STAGES[] = { { 0, 0, 3 }, { 0, 0, 4 }, { 0, 0, 5 } };  // 3, 4, 5 steps
const LedLocation ENV_TRIG[] = { { 0, 1, 3 }, { 0, 1, 4 }, { 0, 1, 5 } };    // SEQ, PULSAR, MIDI
const LedLocation ENV_MODE[] = { { 0, 1, 6 }, { 0, 1, 7 }, { 1, 0, 0 } };    // TRANSIENT, SUSTAIN, OSCILLATE
const LedLocation PULSAR_MODE[] = { { 1, 0, 2 }, { 1, 0, 3 }, { 1, 0, 4 } };  // SEQ, MIDI, OSC
// Pulsar trigger LED: D35 (U2 P0_5) lights while the pulsar fires. D36 (P0_6) was removed.
const LedLocation PULSAR_ACT[] = { { 1, 0, 5 } };
const LedLocation MODOSC_AMFM[] = { { 2, 0, 4 }, { 2, 0, 5 } };                         // FM, AM
const LedLocation MODOSC_WAVE[] = { { 2, 0, 0 }, { 2, 0, 1 }, { 2, 0, 2 }, { 2, 0, 3 } };  // SIN, TRI, SAW, SQUARE
const LedLocation COMP_POL[] = { { 2, 0, 6 }, { 2, 0, 7 } };                            // NORMAL, INVERTED
const LedLocation COMP_WAVE[] = { { 2, 1, 0 }, { 2, 1, 1 }, { 2, 1, 2 } };              // TRI, SAW, SQUARE
const LedLocation QUANTIZE[] = { { 2, 1, 3 }, { 2, 1, 4 } };                            // OFF, ON
const LedLocation RANDOM_MODE[] = { { 2, 1, 5 }, { 2, 1, 6 }, { 2, 1, 7 } };            // SEQ, PULSAR, MIDI
const LedLocation CLOCK_SYNC[] = { { 3, 0, 1 }, { 3, 0, 2 } };                          // FREE, MIDI SYNC
// Sequencer step LEDs (U1 @ 0x58): D9 = P0_6, D14 = P0_7, D15 = P1_0, D16 = P1_1, D17 = P1_2
const LedLocation SEQ_STEP[] = { { 0, 0, 6 }, { 0, 0, 7 }, { 0, 1, 0 }, { 0, 1, 1 }, { 0, 1, 2 } };
// LPG mode LEDs: U2 @ 0x59 port 1 - the same driver as the EG oscillating / trigger and
// pulsar LEDs on its port 0, so if those light, U2 is answering.
// LPG 1 = D38 LO PASS (P1_0), D39 COMBI (P1_1), D40 VCA (P1_2);
// LPG 2 = D42 LO PASS (P1_4), D43 COMBI (P1_5), D44 VCA (P1_6).
// D41 / D45 (P1_3 / P1_7) are the LPG trigger indicators, not used yet.
const LedLocation LPG1_MODE[] = { { 1, 1, 2 }, { 1, 1, 0 }, { 1, 1, 1 } };  // VCA, LP, COMBI
const LedLocation LPG2_MODE[] = { { 1, 1, 6 }, { 1, 1, 4 }, { 1, 1, 5 } };  // VCA, LP, COMBI
}  // namespace leds

}  // namespace weasel
