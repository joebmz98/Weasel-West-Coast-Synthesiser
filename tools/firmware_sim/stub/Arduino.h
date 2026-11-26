// Host stub of the Arduino API: just enough for the WEASEL sketch. Time only moves
// when the harness advances it (one audio block at a time), so runs are deterministic.
#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <algorithm>
typedef uint8_t byte;
typedef bool boolean;
#define HIGH 1
#define LOW 0
#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP 2
enum { D0=0,D1,D2,D3,D4,D5,D6,D7,D8,D9,D10,D11,D12,D13,D14,D15,D16,D17,D18,D19,D20,D21,D22,D23,D24,D25,D26,D27,D28,D29,D30 };
#define A0 100
#define A1 101
#define bitRead(v, b) (((v) >> (b)) & 1)
namespace sim {
extern uint64_t nowUs;
void pinWrite(int pin, int v);
int pinRead(int pin);
int adcRead(int pin);
}
inline unsigned long millis() { return (unsigned long)(sim::nowUs / 1000); }
inline unsigned long micros() { return (unsigned long)sim::nowUs; }
inline void delay(unsigned long) {}
inline void delayMicroseconds(unsigned int) {}
inline void pinMode(int, int) {}
inline void digitalWrite(int p, int v) { sim::pinWrite(p, v); }
inline int digitalRead(int p) { return sim::pinRead(p); }
inline int analogRead(int p) { return sim::adcRead(p); }
inline void analogReadResolution(int) {}
using std::min; using std::max;

class Stream {
public:
  virtual int available() { return 0; }
  virtual int read() { return -1; }
  virtual size_t write(uint8_t) { return 1; }
  size_t write(const uint8_t* b, size_t n) { for (size_t i = 0; i < n; i++) write(b[i]); return n; }
  void begin(unsigned long) {}
  void flush() {}
  void print(...) {}
  void println(...) {}
  operator bool() const { return true; }
};
class HardwareSerial : public Stream {
public:
  uint8_t rx[4096]; int head = 0, tail = 0;
  void setRx(int) {}
  void begin(unsigned long) {}
  void end() {}
  int available() override { return (head - tail + 4096) % 4096; }
  int read() override { if (head == tail) return -1; int c = rx[tail]; tail = (tail + 1) % 4096; return c; }
  int peek() { return head == tail ? -1 : rx[tail]; }
  void push(uint8_t b) { rx[head] = b; head = (head + 1) % 4096; }
};
extern HardwareSerial Serial1;
extern Stream Serial;
extern Stream SerialUSB;
// NVIC / HAL bits used by configureMidiInterruptPriority()
typedef int IRQn_Type;
enum { DMA1_Stream0_IRQn, DMA1_Stream1_IRQn, DMA1_Stream3_IRQn, DMA1_Stream4_IRQn, USART1_IRQn };
inline void HAL_NVIC_SetPriority(int, int, int) {}
