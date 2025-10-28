#include "DaisyDuino.h"

// Matrix 1: A0-A6 (rows), B0-B3 (columns)
const int MATRIX1_ROWS = 7;  // Changed from 6 to 7 for A6
const int MATRIX1_COLS = 4;
const int matrix1RowPins[MATRIX1_ROWS] = { 23, 24, 25, 26, 27, 28, 29 };  // A0-A6
const int matrix1ColPins[MATRIX1_COLS] = { 19, 20, 21, 22 };              // B0-B3

// Matrix 2: X0-X3 (columns), Y0-Y4 (rows)
const int MATRIX2_ROWS = 5;
const int MATRIX2_COLS = 4;
const int matrix2RowPins[MATRIX2_ROWS] = { 8, 9, 10, 11, 14 };  // Y0-Y4
const int matrix2ColPins[MATRIX2_COLS] = { 12, 13, 17, 18 };    // X0-X3 (X0=12, X1=13)

// Button state arrays
bool matrix1State[MATRIX1_ROWS][MATRIX1_COLS] = { 0 };
bool matrix1PrevState[MATRIX1_ROWS][MATRIX1_COLS] = { 0 };

bool matrix2State[MATRIX2_ROWS][MATRIX2_COLS] = { 0 };
bool matrix2PrevState[MATRIX2_ROWS][MATRIX2_COLS] = { 0 };

// Variables to store last changed button
int lastChangedMatrix = 0;  // 1 or 2
int lastChangedRow = -1;
int lastChangedCol = -1;
bool lastChangedState = false;  // true = pressed, false = released

DaisyHardware hw;

void AudioCallback(float **in, float **out, size_t size) {
  // Silent audio callback - output zeros
  for (size_t i = 0; i < size; i++) {
    out[0][i] = 0.0f;
    out[1][i] = 0.0f;
  }
}

void setup() {

  // Initialize Daisy Seed with higher sample rate for better responsiveness
  hw = DAISY.init(DAISY_SEED, AUDIO_SR_48K);
  float sample_rate = DAISY.get_samplerate();

  // Set audio callback
  DAISY.begin(AudioCallback);

  // Initialize matrix 1 pins
  for (int i = 0; i < MATRIX1_ROWS; i++) {
    pinMode(matrix1RowPins[i], INPUT_PULLDOWN);
  }
  for (int i = 0; i < MATRIX1_COLS; i++) {
    pinMode(matrix1ColPins[i], INPUT_PULLDOWN);
  }

  // Initialize matrix 2 pins
  for (int i = 0; i < MATRIX2_ROWS; i++) {
    pinMode(matrix2RowPins[i], INPUT_PULLDOWN);
  }
  for (int i = 0; i < MATRIX2_COLS; i++) {
    pinMode(matrix2ColPins[i], INPUT_PULLDOWN);
  }

  Serial.begin(115200);  // Higher baud rate for better responsiveness
  delay(1000);           // Give serial time to initialize

  Serial.println("Daisy Seed Button Matrix Scanner Ready");
  Serial.println("Matrix 1: A0-A6 (rows 0-6), B0-B3 (cols 0-3)");
  Serial.println("Matrix 2: Y0-Y4 (rows 0-4), X0-X3 (cols 0-3)");
  Serial.println("Waiting for button changes...");
}

void scanMatrix1() {
  // Scan Matrix 1 (A rows, B columns)
  for (int col = 0; col < MATRIX1_COLS; col++) {
    // Set current column as output high
    pinMode(matrix1ColPins[col], OUTPUT);
    digitalWrite(matrix1ColPins[col], HIGH);

    // Small delay for signal stabilization
    delayMicroseconds(10);

    // Read all rows
    for (int row = 0; row < MATRIX1_ROWS; row++) {
      matrix1PrevState[row][col] = matrix1State[row][col];
      matrix1State[row][col] = digitalRead(matrix1RowPins[row]);

      // Check for state change
      if (matrix1State[row][col] != matrix1PrevState[row][col]) {
        lastChangedMatrix = 1;
        lastChangedRow = row;
        lastChangedCol = col;
        lastChangedState = matrix1State[row][col];

        // Print immediately when state changes
        Serial.print("Matrix 1 - ");
        Serial.print("A");
        Serial.print(row);
        Serial.print(" B");
        Serial.print(col);
        Serial.print(": ");
        Serial.println(lastChangedState ? "PRESSED" : "RELEASED");
      }
    }

    // Set column back to input
    digitalWrite(matrix1ColPins[col], LOW);
    pinMode(matrix1ColPins[col], INPUT_PULLDOWN);
  }
}

void scanMatrix2() {
  // Scan Matrix 2 (Y rows, X columns)
  for (int col = 0; col < MATRIX2_COLS; col++) {
    // Set current column as output high
    pinMode(matrix2ColPins[col], OUTPUT);
    digitalWrite(matrix2ColPins[col], HIGH);

    // Small delay for signal stabilization
    delayMicroseconds(10);

    // Read all rows
    for (int row = 0; row < MATRIX2_ROWS; row++) {
      matrix2PrevState[row][col] = matrix2State[row][col];
      matrix2State[row][col] = digitalRead(matrix2RowPins[row]);

      // Check for state change
      if (matrix2State[row][col] != matrix2PrevState[row][col]) {
        lastChangedMatrix = 2;
        lastChangedRow = row;
        lastChangedCol = col;
        lastChangedState = matrix2State[row][col];

        // Print immediately when state changes
        Serial.print("Matrix 2 - ");
        Serial.print("Y");
        Serial.print(row);
        Serial.print(" X");
        Serial.print(col);
        Serial.print(": ");
        Serial.println(lastChangedState ? "PRESSED" : "RELEASED");
      }
    }

    // Set column back to input
    digitalWrite(matrix2ColPins[col], LOW);
    pinMode(matrix2ColPins[col], INPUT_PULLDOWN);
  }
}

void loop() {
  // Scan both matrices
  scanMatrix1();
  scanMatrix2();

  // Small delay to prevent flooding serial
  delay(50);
}