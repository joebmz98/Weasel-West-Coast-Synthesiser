// ********************** //
// WEASEL MUX TEST        //
// Reads every channel of MUX1 (U4) and MUX2 (U5) and prints them over USB serial.
// Pin map matches WEASEL_FW v1.0 (dev branch).
//
// Serial monitor @ 115200. Send:
//   't' -> table mode (full table every 250 ms)        [default]
//   'c' -> change mode (only prints channels that move)
//   's' -> select-line test: holds MUX2 on each channel in turn for 2 s
//          so you can meter U5 pins 10/11/14/13 (S0/S1/S2/S3)
//   'p' -> pause / resume
// ********************** //

#include "DaisyDuino.h"

// -- MUX1 (U4)
#define MUX1_S0 D0
#define MUX1_S1 D1
#define MUX1_S2 D2
#define MUX1_S3 D3
#define MUX1_SIG A0

// -- MUX2 (U5)
#define MUX2_S0 D4
#define MUX2_S1 D5
#define MUX2_S2 D6
#define MUX2_S3 D10
#define MUX2_SIG A1

// -- Other panel lines held in a safe idle state
#define SH_LD_PIN D7
#define CLK_PIN D8
const int sourcePins[4] = { D17, D18, D19, D20 };

const int SETTLE_US = 10;   // same as firmware; raise to e.g. 200 to test settling
const int AVG_SAMPLES = 4;  // averaging per channel for a steadier reading
const int CHANGE_THRESHOLD = 512;  // 16-bit counts (~0.8%) for change mode

const char* mux1Names[16] = {
  "Seq step 1", "Seq step 2", "Seq step 3", "Seq step 4",
  "Seq step 5", "Env attack", "Env sustain", "Env decay",
  "Pulsar ctrl", "Pulsar period", "ModOsc freq ctrl", "ModOsc freq",
  "ModOsc fine", "ModOsc mod ctrl", "ModOsc mod", "CompOsc freq ctrl"
};

const char* mux2Names[16] = {
  "CompOsc freq", "CompOsc fine", "CompOsc WF ctrl", "CompOsc WF",
  "CompOsc timbre", "LPG1 ctrl (RV22)", "LPG1 level (RV23)", "LPG2 ctrl (RV24)",
  "LPG2 level (RV25)", "Clock", "Reverb mix", "Master",
  "N/C (GND)", "N/C (GND)", "N/C (GND)", "N/C (GND)"
};

uint16_t vals[2][16];
uint16_t lastPrinted[2][16];

enum TestMode { MODE_TABLE, MODE_CHANGE };
TestMode testMode = MODE_TABLE;
bool paused = false;

void selectChannel(int ch, int s0, int s1, int s2, int s3) {
  digitalWrite(s0, bitRead(ch, 0));
  digitalWrite(s1, bitRead(ch, 1));
  digitalWrite(s2, bitRead(ch, 2));
  digitalWrite(s3, bitRead(ch, 3));
}

uint16_t readChannel(int mux, int ch) {
  if (mux == 0) selectChannel(ch, MUX1_S0, MUX1_S1, MUX1_S2, MUX1_S3);
  else selectChannel(ch, MUX2_S0, MUX2_S1, MUX2_S2, MUX2_S3);
  delayMicroseconds(SETTLE_US);
  uint32_t acc = 0;
  for (int i = 0; i < AVG_SAMPLES; i++) acc += analogRead(mux == 0 ? MUX1_SIG : MUX2_SIG);
  return acc / AVG_SAMPLES;
}

void readAll() {
  for (int m = 0; m < 2; m++)
    for (int c = 0; c < 16; c++) vals[m][c] = readChannel(m, c);
}

void printBar(uint16_t v) {
  int n = (v * 20UL) / 65535UL;
  Serial.print('|');
  for (int i = 0; i < 20; i++) Serial.print(i < n ? '#' : ' ');
  Serial.print('|');
}

void printPadded(const char* s, int width) {
  Serial.print(s);
  for (int i = strlen(s); i < width; i++) Serial.print(' ');
}

void printNum(long v, int width) {
  char buf[12];
  snprintf(buf, sizeof(buf), "%*ld", width, v);
  Serial.print(buf);
}

void printTable() {
  Serial.println();
  Serial.println("CH  MUX1                         raw   %                          MUX2                         raw   %");
  for (int c = 0; c < 16; c++) {
    printNum(c, 2);
    Serial.print("  ");
    printPadded(mux1Names[c], 18);
    printNum(vals[0][c], 6);
    printNum((vals[0][c] * 100UL) / 65535UL, 4);
    Serial.print(' ');
    printBar(vals[0][c]);
    Serial.print("   ");
    printPadded(mux2Names[c], 18);
    printNum(vals[1][c], 6);
    printNum((vals[1][c] * 100UL) / 65535UL, 4);
    Serial.print(' ');
    printBar(vals[1][c]);
    Serial.println();
  }
}

void printChanges() {
  for (int m = 0; m < 2; m++) {
    for (int c = 0; c < 16; c++) {
      if (abs((int)vals[m][c] - (int)lastPrinted[m][c]) > CHANGE_THRESHOLD) {
        Serial.print(millis());
        Serial.print(m == 0 ? "  MUX1 C" : "  MUX2 C");
        printNum(c, 2);
        Serial.print("  ");
        printPadded(m == 0 ? mux1Names[c] : mux2Names[c], 18);
        printNum(vals[m][c], 6);
        printNum((vals[m][c] * 100UL) / 65535UL, 4);
        Serial.println("%");
        lastPrinted[m][c] = vals[m][c];
      }
    }
  }
}

void selectLineTest() {
  Serial.println();
  Serial.println("MUX2 select-line test. Expected logic levels on U5:");
  Serial.println("  pin10=S0  pin11=S1  pin14=S2  pin13=S3   (send any key to abort)");
  for (int c = 0; c < 16; c++) {
    selectChannel(c, MUX2_S0, MUX2_S1, MUX2_S2, MUX2_S3);
    Serial.print("C");
    printNum(c, 2);
    Serial.print("  S3..S0 = ");
    for (int b = 3; b >= 0; b--) Serial.print(bitRead(c, b));
    Serial.print("   reading: ");
    unsigned long t0 = millis();
    while (millis() - t0 < 2000) {
      if (Serial.available()) {
        while (Serial.available()) Serial.read();
        Serial.println("\naborted");
        return;
      }
    }
    delayMicroseconds(SETTLE_US);
    uint16_t v = analogRead(MUX2_SIG);
    printNum(v, 6);
    Serial.print("  (");
    Serial.print(mux2Names[c]);
    Serial.println(")");
  }
  Serial.println("done");
}

void handleSerial() {
  while (Serial.available()) {
    char k = Serial.read();
    switch (k) {
      case 't': testMode = MODE_TABLE; Serial.println("\n[table mode]"); break;
      case 'c':
        testMode = MODE_CHANGE;
        memcpy(lastPrinted, vals, sizeof(vals));
        Serial.println("\n[change mode - move a pot]");
        break;
      case 's': selectLineTest(); break;
      case 'p': paused = !paused; Serial.println(paused ? "\n[paused]" : "\n[running]"); break;
      default: break;
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(1500);

  DAISY.init(DAISY_SEED, AUDIO_SR_48K);  // init system first, then claim pins

  const int muxPins[] = { MUX1_S0, MUX1_S1, MUX1_S2, MUX1_S3,
                          MUX2_S0, MUX2_S1, MUX2_S2, MUX2_S3 };
  for (int i = 0; i < 8; i++) {
    pinMode(muxPins[i], OUTPUT);
    digitalWrite(muxPins[i], LOW);
  }
  pinMode(MUX1_SIG, INPUT);
  pinMode(MUX2_SIG, INPUT);
  analogReadResolution(16);

  // Keep the 74HC165 chain and patch-bay sources idle
  pinMode(SH_LD_PIN, OUTPUT);
  digitalWrite(SH_LD_PIN, HIGH);
  pinMode(CLK_PIN, OUTPUT);
  digitalWrite(CLK_PIN, LOW);
  for (int i = 0; i < 4; i++) {
    pinMode(sourcePins[i], OUTPUT);
    digitalWrite(sourcePins[i], LOW);
  }

  Serial.println("WEASEL MUX TEST");
  Serial.println("t = table, c = changes only, s = MUX2 select-line test, p = pause");
  readAll();
  memcpy(lastPrinted, vals, sizeof(vals));
}

void loop() {
  handleSerial();
  if (paused) return;

  readAll();
  if (testMode == MODE_TABLE) {
    static unsigned long last = 0;
    if (millis() - last >= 250) {
      printTable();
      last = millis();
    }
  } else {
    printChanges();
    delay(5);
  }
}