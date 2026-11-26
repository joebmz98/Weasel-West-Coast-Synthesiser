// =============================================================================
// config.h — firmware version and build options
//
// Everything you might change when building for a different board revision
// lives here. Pin numbers and the panel map are in hardware.h.
// =============================================================================
#pragma once

// Reported to the web pages (display only: WeaselEditor.h's PROTOCOL decides
// compatibility). Must be defined before WeaselEditor.h is included.
#define WEASEL_FIRMWARE_VERSION "1.2.0"

// 1 = print button presses, patch changes, pot glitches and MIDI input over USB
// serial (115200 baud). The web editor uses the same USB port: leave this at 0
// when using the web pages.
#define WEASEL_DEBUG 0

// AW9523 LED wiring:
//   0 = prototype boards: AW9523 pin -> LED anode, cathode -> resistor -> GND.
//       Pins run as push-pull GPIO (HIGH = on). LED mode can't be used (it only
//       sinks current), so brightness is set by the series resistors.
//   1 = current schematic: 3V3 -> resistor -> LED anode, cathode -> AW9523 pin.
//       Pins run in constant-current LED mode, brightness set by
//       kLedCurrentRange / ledBrightness in leds.h.
#define WEASEL_LED_SINK_WIRING 0

// Sequencer step buttons:
//   1 = latching switches, step enabled/disabled by switch level (see below)
//   0 = momentary buttons, each press toggles the step on/off
#define SEQ_STEP_SWITCHES_LATCHING 1

// Latching mode only: 1 = step ENABLED while its switch reads HIGH, disabled when LOW
//                     0 = step ENABLED while its switch reads LOW (v0.9 behaviour)
#define SEQ_STEP_ENABLED_WHEN_HIGH 0

// Main-loop code that is called from more than one place is kept out of line:
// the sketch is built at -O2, which would otherwise copy each of these into every
// caller and cost several KB of the 128 KB flash.
#define WEASEL_NOINLINE __attribute__((noinline))

// Complex oscillator oversampling (oscillators, wavefolder and AM run at this
// multiple of 48 kHz). 1 = off (the 1.1.x sound, aliasing included), 2 or 4.
#ifndef WEASEL_COMPLEX_OVERSAMPLING
#define WEASEL_COMPLEX_OVERSAMPLING 4
#endif
