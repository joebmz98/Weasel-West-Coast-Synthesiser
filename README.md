# WEASEL
### Buchla Music Easel Inspired Digital Synthesizer

**firmware version 0.1** | **hardware version 1.0**

---

Designed by **[.axs instruments](https://github.com/axsinstruments)**

---

## Overview

WEASEL is a digital synthesizer inspired by the iconic Buchla Music Easel, built around the Daisy Seed platform. It features a virtual patch bay that simulates a modular experience, allowing you to route modulation sources to destinations through an intuitive button matrix interface.

---

## Features

- **Complex Oscillator** with sine wave and morphable waveform (saw/triangle/square)
- **Modulation Oscillator** with sine, triangle, saw, and square waveforms
- **FM/AM Modulation** between oscillators
- **Wavefolder** for complex timbre shaping
- **Dual-channel Low Pass Gate (LPG)** with VCA/LP/COMBI modes
- **Buchla-style ASR Envelope Generator**
- **Pulsar Generator** (decaying envelope for rhythmic triggering)
- **Sample & Hold / Random Voltage Generator**
- **5-Step Sequencer** with per-step enable toggles
- **Virtual Patch Bay** (4x7 button matrix) for routing:
  - Sequencer CV → 7 destinations
  - Envelope → 7 destinations
  - Pulsar → 7 destinations
  - Random Voltage → 7 destinations
- **Internal Reverb**
- **MIDI Input** for note control and triggering

---

## Hardware Requirements

| Component | Details |
|-----------|---------|
| **Microcontroller** | Daisy Seed |
| **ADC Resolution** | 16-bit |
| **Potentiometers** | 32 total (via 2x 16-channel multiplexers) |
| **Button Matrix 1** | 4x7 (virtual patch bay) |
| **Button Matrix 2** | 4x5 (mode selection & controls) |
| **Audio Output** | Stereo line out |
| **MIDI Input** | 3.5mm TRS (pin D14 / USART1 Rx) |

### Pin Mapping

**MUX1 (Pots 0-15)** – A0

| MUX Pin | Function |
|---------|----------|
| S0 | D0 |
| S1 | D1 |
| S2 | D2 |
| S3 | D3 |

**MUX2 (Pots 16-31)** – A1

| MUX Pin | Function |
|---------|----------|
| S0 | D4 |
| S1 | D5 |
| S2 | D6 |
| S3 | D7 |

---

## Potentiometer Functions

### MUX1 (Channels 0-15)

| CH | Function |
|----|----------|
| 0 | Sequencer Step 1 CV |
| 1 | Sequencer Step 2 CV |
| 2 | Sequencer Step 3 CV |
| 3 | Sequencer Step 4 CV |
| 4 | Sequencer Step 5 CV |
| 5 | Attack Time |
| 6 | Sustain Duration |
| 7 | Decay/Release Time |
| 8 | Pulsar Period Modulation Control |
| 9 | Pulsar Period |
| 10 | Modulation Osc Freq Modulation Amount |
| 11 | Modulation Osc Frequency |
| 12 | Modulation Osc Fine Tune (±3 cents) |
| 13 | Modulation Osc Modulation Amount Depth |
| 14 | Modulation Osc Modulation Amount |
| 15 | Complex Osc Frequency Modulation Amount |

### MUX2 (Channels 16-31)

| CH | Function |
|----|----------|
| 16 | Complex Osc Frequency |
| 17 | Complex Osc Fine Tune (±3 cents) |
| 18 | Complex Osc Wavefolder Modulation Amount |
| 19 | Complex Osc Wavefolder |
| 20 | Complex Osc Timbre (morph) |
| 21 | LPG1 Modulation Amount |
| 22 | LPG1 Level |
| 23 | LPG2 Modulation Amount |
| 24 | LPG2 Level |
| 25 | Clock Speed |
| 26 | Reverb Mix |
| 27-31 | Not Connected |

---

## Button Matrix Functions

### Matrix 1 (4x7) – Virtual Patch Bay

**Sources (Columns):**
- Col 0: Sequencer CV
- Col 1: Envelope
- Col 2: Pulsar
- Col 3: Random Voltage

**Destinations (Rows):**
- Row 0: Pulsar Period
- Row 1: Modulation Osc Frequency
- Row 2: Modulation Osc Modulation Amount
- Row 3: Complex Osc Pitch
- Row 4: Complex Osc Wavefolder Amount
- Row 5: LPG Channel 1 Level
- Row 6: LPG Channel 2 Level

### Matrix 2 (4x5) – Mode Controls

| Position | Function |
|----------|----------|
| X0+Y0 | Sequencer Length Cycle (3/4/5 steps) |
| X0+Y1 | Pulsar Mode Toggle (SEQ/MIDI/OSC) |
| X0+Y2 | Complex Osc Polarity Invert |
| X0+Y3 | Mod Osc LPG Mode Toggle (VCA/LP/COMBI) |
| X0+Y4 | Sequencer Step 2 Toggle |
| X1+Y0 | Sequencer Trigger Mode (CLOCK/PULSAR/MIDI) |
| X1+Y1 | Pulsar Clock Mode |
| X1+Y2 | MIDI Toggle (Complex + Mod Osc) |
| X1+Y3 | Random Trigger Mode (SEQ/PULSAR/MIDI) |
| X1+Y4 | Sequencer Step 3 Toggle |
| X2+Y1 | Mod Waveform Cycle |
| X2+Y2 | Complex Waveform Cycle |
| X2+Y4 | Sequencer Step 4 Toggle |
| X3+Y1 | FM/AM Toggle |
| X3+Y2 | LPG Mode Toggle (VCA/LP/COMBI) |
| X3+Y3 | Sequencer Step 1 Toggle |
| X3+Y4 | Sequencer Step 5 Toggle |

---

## Installation & Setup

### 1. Prerequisites

- [Arduino IDE](https://www.arduino.cc/en/software)
- [DaisyDuino](https://github.com/electro-smith/DaisyDuino) – Daisy platform for Arduino
- [MIDI Library](https://github.com/FortySevenEffects/arduino_midi_library)

### 2. Flashing the Firmware

1. Install DaisyDuino following the [official guide](https://github.com/electro-smith/DaisyDuino#installation)
2. Clone this repository
3. Open `weasel.ino` in Arduino IDE
4. Select **Daisy Seed** as the board
5. Upload the firmware

### 3. First Boot

- Connect audio output to the Daisy Seed's audio jack
- Connect a MIDI source to the TRS input (optional)
- Power via USB or external 5V
- The synthesizer will start with default settings

---

## Usage

### Basic Signal Flow

[Complex Osc] → [Wavefolder] → [LPG Ch1] → [Mixer] → [Output Filter] → [Reverb]
[Mod Osc] ──────────────────→ [LPG Ch2] ↗
     ↓
  (FM/AM to Complex Osc)

### Sequencer Operation

- The 5-step sequencer advances based on the selected trigger mode (Clock/Pulsar/MIDI)
- Each step's CV is set by its corresponding potentiometer
- Per-step toggles allow disabling individual steps
- Sequence length can be set to 3, 4, or 5 steps

### Envelope (Buchla ASR Style)

- **Attack**: Sets rise time
- **Sustain**: Sets gate duration
- **Release**: Sets fall time after gate ends

### Pulsar Generator

- Creates decaying envelopes for rhythmic triggering
- Can be synced to sequencer, MIDI note-on, or run freely as an LFO

### Low Pass Gate Modes

- **VCA**: Amplitude only (no filtering)
- **LP**: Low-pass filtering only
- **COMBI**: Both VCA and filtering (classic Buchla LPG behavior)

---

## MIDI Implementation

| Function | Details |
|----------|---------|
| **Input** | 3.5mm TRS (Type A) |
| **Baud Rate** | 31250 |
| **Note On** | Triggers sequencer, pulsar, and random (depending on mode) |
| **Note Pitch** | Controls Complex and/or Mod Osc frequencies when MIDI mode is enabled |

**MIDI Pin:** Digital pin 30 (D14 / USART1 Rx)

---

## Building from Source

### Dependencies

- `DaisyDuino` – Daisy hardware abstraction
- `MIDI` – MIDI input handling

### Compilation

The firmware is compatible with Arduino IDE. Select **Daisy Seed** from the boards menu and upload.

---

## License

**MIT License** – Copyright (c) 2024 .axs instruments

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files, to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED.

---

## Support & Contributions

- **Issues:** Open a GitHub ticket
- **Custom Requests:** [axs.instruments@gmail.com](mailto:axs.instruments@gmail.com)
- **Contributions:** PRs welcome!

---

## Acknowledgments

Built with the [Daisy Platform](https://www.electro-smith.com/daisy) by Electro-Smith.