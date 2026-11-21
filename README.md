# WEASEL
### Buchla Music Easel inspired digital west coast synthesiser

**firmware 1.0** | **hardware 1.0**

Designed by **[.axs instruments](https://github.com/axsinstruments)**

> ### 🔌 [Open the WEASEL web flasher & editor](https://joebmz98.github.io/Weasel-Flasher/)
> Update the firmware, set the MIDI channel, store and recall presets, or play WEASEL in the browser.
> Runs in Chrome or Edge on a computer, needs a micro-USB **data** cable, nothing to install.
> Source and instructions: [joebmz98/Weasel-Flasher](https://github.com/joebmz98/Weasel-Flasher)

---

## Overview

WEASEL is a digital synthesiser inspired by the Buchla Music Easel, built around the
Electro-Smith Daisy Seed. A front panel of faders, knobs, buttons and toggles sits over a
single mainboard, and a **patch bay of 3.5 mm jacks** routes the four modulation sources to
seven destinations with real patch cables, just like the Easel.

Every mode is shown by a 3 mm LED shining straight up through the front panel, so the
panel always tells you what the synth is doing.

---

## Features

- **Complex oscillator**: sine plus a morphing saw / square / triangle, with timbre and polarity
- **Wavefolder** on the complex oscillator
- **Modulation oscillator**: sine, triangle, sawtooth and square, with FM or AM into the complex oscillator
- **Dual low pass gate**: each channel switches between VCA, low pass and combination. Since firmware 1.1.2 this is a circuit model of the Buchla 292 (vactrol plus audio circuit, after Parker & D'Angelo, DAFx-13), with the mode switch reconfiguring the modelled circuit as on the original
- **Envelope generator**: attack / sustain / decay, in transient, sustain or oscillating mode
- **Pulsar**: decaying pulse generator, triggered by the sequencer, MIDI, or free running
- **Sequential voltage source**: 5-step sequencer with 3, 4 or 5 stages and a toggle per step. A step that is off is a rest: it holds the previous CV and sends no trigger to the envelope, pulsar or random source
- **Random voltage**: sample & hold, triggered by the sequencer, pulsar or MIDI
- **Clock**: free running or synced to incoming MIDI clock
- **Patch bay**: 12 source jacks (3 each for sequencer, envelope, pulsar, random) and 7 destination jacks
- **Reverb**, master volume and stereo line out
- **MIDI in** (3.5 mm TRS): notes, triggers, clock and start/stop
- **Web editor** over USB: MIDI channel, presets of every panel mode, firmware updates

---

## Web flasher & editor

The **[WEASEL web tools](https://joebmz98.github.io/Weasel-Flasher/)** have four pages:

| Page | What it does | WEASEL needs to be |
| --- | --- | --- |
| **Update firmware** | Flashes a release or your own `.bin` over WebUSB | In DFU mode |
| **MIDI settings** | MIDI in channel (1-16 or omni) and a MIDI in monitor | Running, with the USB editor firmware |
| **Presets** | Stores and recalls every panel mode by name; import / export as JSON | Running, with the USB editor firmware |
| **Play** | The WEASEL sound engine running in the browser | Not needed |

**Putting WEASEL in DFU mode:** hold **BOOT**, tap **RESET**, release **BOOT** on the Daisy Seed.
Once the editor firmware is running you can also restart into DFU mode from the flasher page.

The MIDI settings and Presets pages need the firmware build with the USB editor link
(`WeaselEditor.h`), which currently lives in the flasher repo under `firmware-src/`.
Add `?demo` to any page's address to try it against a simulated WEASEL.

---

## Hardware

### Boards

| Board | Folder | Notes |
| --- | --- | --- |
| **Mainboard** | `build/WEASEL_KiCad` | Daisy Seed, all pots, buttons, jacks, LEDs and power |
| **Front panel** | `build/WEASEL_KiCad` | PCB panel: silkscreen artwork, cut-outs and 3.2 mm LED light holes |
| **Back panel** | `build/WEASEL_KiCad` | PCB back plate |
| **Keys board** | `build/WEASEL_KEYS_KiCad` | Capacitive touch keyboard (4x MPR121), connects over I2C. Firmware support is still to come |

The mainboard, front panel and back panel are drawn in one KiCad file and joined with
**mouse bites**, so they are ordered as a single panel (about 335 x 604 mm) and snapped apart.
All three share the same **M3** mounting hole pattern (3.2 mm plated holes), so the stack
bolts together with M3 screws and standoffs.

### Mainboard parts

| Part | Details |
| --- | --- |
| **Microcontroller** | Electro-Smith Daisy Seed (STM32H750) |
| **Potentiometers** | 22 Bourns PTA2043 slide faders + 6 RV09 9 mm rotary pots, read by 2x CD74HC4067 16-channel multiplexers |
| **Buttons** | 15 x 6x6 mm tactile switches, 13 mm tall so they reach through the front panel |
| **Toggles** | 5 x MTS-102 mini toggles (sequencer step on/off) |
| **Button inputs** | 4x 74HC165 shift registers (32-bit chain): 3 for panel buttons, 1 for the patch bay destinations |
| **LEDs** | 52 x 3 mm through-hole LEDs, driven by 4x AW9523B (I2C 0x58-0x5B) in constant-current mode |
| **Patch bay** | 19 x PJ398SM-style vertical mono jacks (tip / tip-normal / sleeve) |
| **Audio out** | 3.5 mm stereo jack |
| **MIDI in** | 3.5 mm TRS (Type A), 6N137 optocoupler |
| **Power** | DC barrel jack, centre positive. AP63205 5 V buck regulator with reverse-polarity protection (P-FET), SMBJ24A TVS and a resettable fuse, so a wrong or centre-negative plug won't damage the Seed. Use 9-12 V DC (24 V absolute max) |
| **Keys connector** | J23 / J24: 2x5 2.54 mm right-angle pin headers carrying 3.3 V, GND, I2C (SCL / SDA) and IRQ |

#### LEDs

The LEDs sit on the mainboard and shine up through 3.2 mm holes in the front panel. Fit them
standing tall enough (or on spacers) that the lens reaches the panel. Brightness is set in
firmware rather than by the resistors: the AW9523 sinks a regulated current from each LED
(about half brightness by default). The 47R resistors to 3.3 V only limit the current.

### Pin map

| Function | Daisy pin |
| --- | --- |
| MUX select S0-S3 (both multiplexers share these) | D0, D1, D2, D3 |
| MUX1 signal (pots 0-15) | A0 |
| MUX2 signal (pots 16-31) | A1 |
| 74HC165 SH/LD, CLK, QH | D7, D8, D9 |
| I2C SCL / SDA (AW9523 LED drivers, keys board) | D11 / D12 |
| MIDI RX (USART1) | D14 |
| Patch bay sources: sequencer, envelope, pulsar, random | D17, D18, D19, D20 |

> **Prototype boards:** the first prototypes drove the second multiplexer from its own select
> pins (D4, D5, D6, D10) and wired the LED anodes to the AW9523. The firmware still supports
> both, see [Building the firmware](#building-the-firmware).

---

## Controls

### Pots and faders

| MUX1 | Function | MUX2 | Function |
| --- | --- | --- | --- |
| 0 | Sequencer stage 1 | 16 | Complex osc frequency |
| 1 | Sequencer stage 2 | 17 | Complex osc fine tune (±3 cents) |
| 2 | Sequencer stage 3 | 18 | Wavefolder modulation amount |
| 3 | Sequencer stage 4 | 19 | Wavefolder |
| 4 | Sequencer stage 5 | 20 | Complex osc timbre (morph) |
| 5 | Envelope attack | 21 | LPG 1 modulation amount |
| 6 | Envelope sustain | 22 | LPG 1 level |
| 7 | Envelope decay | 23 | LPG 2 modulation amount |
| 8 | Pulsar period modulation amount | 24 | LPG 2 level |
| 9 | Pulsar period | 25 | Clock speed |
| 10 | Mod osc frequency modulation amount | 26 | Reverb mix |
| 11 | Mod osc frequency | 27 | Master volume |
| 12 | Mod osc fine tune (±3 cents) | 28-31 | Not connected |
| 13 | Mod osc modulation depth modulation amount | | |
| 14 | Mod osc modulation depth | | |
| 15 | Complex osc frequency modulation amount | | |

### Buttons and toggles

Each button steps through its modes when you press **and release** it, and the LEDs beside
it show the current one (`*` = power-on default). The pulsar's manual button fires as soon as
it's pressed.

| Section | Button | Modes |
| --- | --- | --- |
| Sequential voltage source | Trigger select | clock\*, pulsar, midi |
| | Stages | 5\*, 4, 3 |
| | Step 1-5 (toggles) | each step on / off |
| Envelope generator | Trigger select | sequencer\*, pulsar, midi |
| | Mode | transient\*, sustain, oscillating |
| Pulsar | Trigger select | sequencer\*, midi, free |
| | Manual | fires a pulse |
| Modulation oscillator | Waveform | sine\*, triangle, sawtooth, square |
| | Modulation | fm\*, am |
| Complex oscillator | Waveform | saw\*, square, triangle |
| | Polarity | +\*, - |
| | MIDI | off\*, on (follow MIDI notes) |
| Dual low pass gate | Level 1 / level 2 mode | voltage controlled amp, lo pass filter, combination\* |
| Random voltage | Trigger select | sequencer\*, pulsar, midi |
| Clock | Mode | free\*, sync (MIDI clock) |

### Shift functions (firmware 1.1.3+)

**MIDI pitch per oscillator.** By default the MIDI note sets the pitch of both oscillators. To
choose which ones follow it:

1. Hold the complex oscillator's **MIDI** button for 1 second. The LPG 1 and LPG 2 mode LEDs flash.
2. Press **LPG 1** (complex oscillator) or **LPG 2** (modulation oscillator) to switch that
   oscillator in or out of MIDI pitch. Flashing = follows the MIDI note; steady (showing the
   channel's mode as normal) = keeps its own pitch. Press again to switch it back.
3. Let go of **MIDI**. The choice applies then, and is kept after power-off. Letting go after a
   shift never toggles MIDI on or off; press and release MIDI again for that.

While MIDI is held, other buttons are ignored. The same setting is in the
[web editor](https://joebmz98.github.io/Weasel-Flasher/editor.html) under **MIDI pitch**.

### Patch bay

Plug a cable from any source jack to any destination jack. Each source has three jacks, so one
source can drive several destinations at once.

| Sources (3 jacks each) | Destinations |
| --- | --- |
| Sequencer, envelope, pulsar, random | Pulsar period, mod osc frequency, mod osc modulation, complex osc pitch, wavefolder, LPG 1 level, LPG 2 level |

---

## Signal flow

```
[Complex Osc] → [Wavefolder] → [LPG 1] ┐
       ↑ FM / AM                        ├→ [Mixer] → [Output filter] → [Reverb] → [Master volume] → Out
[Mod Osc] ───────────────────→ [LPG 2] ┘

Sequencer · Envelope · Pulsar · Random ──(patch bay)──→ modulation destinations
```

---

## MIDI

| Function | Details |
| --- | --- |
| **Input** | 3.5 mm TRS (Type A), 31250 baud |
| **Channel** | Omni by default; set 1-16 in the [web editor](https://joebmz98.github.io/Weasel-Flasher/editor.html) |
| **Note on** | Triggers the sequencer, envelope, pulsar and random voltage when they're set to midi |
| **Note pitch** | Plays the complex and mod oscillators when MIDI is on; each can be left out (see [Shift functions](#shift-functions-firmware-113) or the web editor) |
| **Clock, start / stop** | Syncs the sequencer when the clock is set to sync (received on any channel) |

---

## Building the firmware

The easiest way to update is the **[web flasher](https://joebmz98.github.io/Weasel-Flasher/#flash)**.
To build it yourself:

1. Install the [Arduino IDE](https://www.arduino.cc/en/software), [DaisyDuino](https://github.com/electro-smith/DaisyDuino#installation) and the [MIDI Library](https://github.com/FortySevenEffects/arduino_midi_library).
2. Open `src/WEASEL_FW/WEASEL_FW.ino`.
3. Set the board options at the top of the sketch:
   - `WEASEL_LED_SINK_WIRING`: `1` for the current board (LED cathodes on the AW9523, dimmable), `0` for the prototype boards.
   - `LED_BRIGHTNESS` (0-255) and `AW9523_ISEL` set the LED current.
   - `SEQ_STEP_SWITCHES_LATCHING`: `1` for toggle switches on the sequencer steps, `0` for momentary buttons.
4. Select **Daisy Seed** as the board. For the web editor, also set **Tools > USB support > CDC (generic 'Serial' supersede U(S)ART)**.
5. Put the Seed in DFU mode and upload, or use **Sketch > Export Compiled Binary** and flash the `.bin` with the web flasher.

`src/tests/potentiometerTests` is a small sketch for checking the pots and multiplexers.

---

## Ordering the PCBs

Production files are in `build/WEASEL_KiCad/production/`, made with the KiCad Fabrication Toolkit for JLCPCB:

| File | Contents |
| --- | --- |
| `WEASEL.zip` | Gerbers and drill files for the mainboard, front panel and back panel |
| `WEASEL_bom.csv` | BOM with LCSC part numbers |
| `WEASEL_positions.csv` | Pick and place positions |

The BOM leaves out parts that are fitted by hand: the **LEDs**, the **Daisy Seed**, the
**MIDI / audio jacks (J1, J2)**, the **DC jack (J22)** and the **six 9 mm rotary pots**.
Regenerate the production files from KiCad after any board change.

---

## Repository layout

| Path | Contents |
| --- | --- |
| `src/WEASEL_FW/` | Firmware (Arduino / DaisyDuino) |
| `src/tests/` | Hardware test sketches |
| `tools/lpg_test/` | Desktop test harness for the LPG model |
| `tools/button_test/` | Desktop test of the panel buttons and the MIDI shift (`sh tools/button_test/run.sh`) |
| `build/WEASEL_KiCad/` | Mainboard and panels: schematic, PCB, footprints, DRC report, production files |
| `build/WEASEL_KEYS_KiCad/` | Keys board |
| `lib/` | KiCad libraries (Daisy Seed) |
| `docs/` | Images, audio references and Buchla LPG research |

---

## License

**MIT License**: Copyright (c) 2024 .axs instruments

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files, to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED.

---

## Support & contributions

- **Issues:** open a GitHub issue
- **Custom requests:** [axs.instruments@gmail.com](mailto:axs.instruments@gmail.com)
- **Contributions:** PRs welcome!

---

## Acknowledgments

Built with the [Daisy Platform](https://www.electro-smith.com/daisy) by Electro-Smith.

### Low pass gate model

The low pass gates (firmware 1.1.2 and later, `src/WEASEL_FW/buchla_lpg.cpp`) are a model of the Buchla 292, built on:

- **Julian Parker & Stefano D'Angelo**, [*A Digital Model of the Buchla Lowpass-Gate*](https://dafx.de/paper-archive/details/uqLQLFI9j52bBmv10UKKMg), Proc. DAFx-13, Maynooth, 2013: the circuit analysis, the three-mode component values and the vactrol model
- **Josh Rohs**, *A Matlab Implementation of the Buchla Lopass Gate*, McGill University MUMT618, 2018 (supervised by Gary Scavone; copy in `docs/references/`): the level-dependent vactrol response (rho)
- **Segfault1602**, [`sfdsp::BuchlaLPG` in libdsp](https://github.com/Segfault1602/libdsp/blob/main/src/buchla_lpg.cpp) (MIT License, Copyright (c) 2023 Segfault1602): the C++ implementation the WEASEL version is adapted from
- **Don Buchla**, for the original 292 Lopass Gate and the Music Easel that WEASEL is inspired by

PCBway very kindly offered to sponsor this project and have sent me some more stock for the shop. Thank you to PCBway for supporting this passion project of mine. Emily from the marketing department reached out to me when I first launched the product and store and very kindly offered to sponsor the project. Both working with them and using their services was really easy. All I needed to do was export my PCB gerber files, drag and drop them on the 'PCB Instant Quote' page and send it off for approval. I received my order a week later and was very happy with the quality of the PCBs. Thank you again PCBway for your support.

You can find their website here: https://www.pcbway.com/
