#pragma once
#include <stdint.h>
#include <stddef.h>
constexpr int LOW = 0, HIGH = 1, INPUT = 0, OUTPUT = 1, CHANGE = 2;
uint32_t millis();
uint32_t micros();
void noInterrupts();
void interrupts();
int digitalRead(uint8_t pin);
void digitalWrite(uint8_t pin, int level);
void pinMode(uint8_t pin, int mode);
inline int digitalPinToInterrupt(int pin) { return pin; }
void attachInterrupt(int pin, void (*callback)(), int mode);
struct MockSerial {
  char input[1024] = {}, output[32768] = {};
  size_t readAt = 0, inputSize = 0, outputSize = 0, writes = 0;
  bool stopDuringTelemetry = false;
  int writeCapacity = 0;  // Exact UNO NO_USB availableForWrite behavior.
  void begin(int baud);
  int available() { return static_cast<int>(inputSize - readAt); }
  int read() { return available() ? input[readAt++] : -1; }
  int availableForWrite() { return writeCapacity; }
  size_t write(uint8_t* data, size_t length);
  void receive(const char* text);
};
extern MockSerial Serial;
