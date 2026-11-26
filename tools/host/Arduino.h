// Minimal Arduino stub for the desktop tests in tools/: just the time functions
// the firmware headers use, driven by the test (fakeMs / fakeUs).
#pragma once
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
inline unsigned long fakeMs = 0;
inline unsigned long fakeUs = 0;
inline unsigned long millis() { return fakeMs; }
inline unsigned long micros() { return fakeUs; }
// Pin names used by hardware.h
enum { D0 = 0, D1, D2, D3, D4, D5, D6, D7, D8, D9, D10, D11, D12, D13, D14, D15, D16, D17, D18, D19, D20 };
#define A0 100
#define A1 101
