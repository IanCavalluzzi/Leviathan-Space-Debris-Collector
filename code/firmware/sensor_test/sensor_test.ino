// UNO R4 WiFi sensor bench test. Uses the normal 5 V Arduino headers.
// Ultrasonic settings assume a standard HC-SR04; confirm the actual module.
// IR values are raw digital levels, not distance or grasp confirmation.
#include <Arduino.h>

#if !defined(ARDUINO_UNOR4_WIFI)
#error "Select Arduino UNO R4 WiFi as the board."
#endif

// Latest user wiring: reserve D2=PUL+ and D3=DIR+ for future motor firmware.
// This sensor-only sketch does not configure or drive those motor pins.
constexpr uint8_t TRIG_PIN = 7;
constexpr uint8_t ECHO_PIN = 6;
constexpr uint8_t IR_PINS[] = {9, 10, 11, 12, 13};  // OUT1 through OUT5.
constexpr size_t IR_COUNT = sizeof(IR_PINS) / sizeof(IR_PINS[0]);
constexpr unsigned long ECHO_TIMEOUT_US = 30000UL;
constexpr unsigned long SAMPLE_PAUSE_MS = 200UL;

struct EchoSample {
  unsigned long widthUs;
  const char* phase;
  int before;
  int after;
};

EchoSample measureEcho() {
  const int before = digitalRead(ECHO_PIN);
  if (before == HIGH) {
    // Echo is always an input. Do not try to reset a sensor by driving it low.
    return {0, "HIGH_BEFORE_TRIGGER", before, digitalRead(ECHO_PIN)};
  }

  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  const unsigned long started = micros();
  while (digitalRead(ECHO_PIN) == LOW) {
    if (micros() - started >= ECHO_TIMEOUT_US) {
      return {0, "NO_RISE", before, digitalRead(ECHO_PIN)};
    }
  }
  const unsigned long rose = micros();
  while (digitalRead(ECHO_PIN) == HIGH) {
    if (micros() - started >= ECHO_TIMEOUT_US) {
      return {0, "NO_FALL", before, digitalRead(ECHO_PIN)};
    }
  }
  return {micros() - rose, "PULSE", before, digitalRead(ECHO_PIN)};
}

void setup() {
  digitalWrite(TRIG_PIN, LOW);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  for (size_t i = 0; i < IR_COUNT; ++i) {
    pinMode(IR_PINS[i], INPUT);  // No pull-ups: output circuitry is unconfirmed.
  }

  Serial.begin(115200);
  delay(1000);
  Serial.println(F("# Sensor test: TRIG=D7 ECHO=D6 OUT1..5=D9..13"));
  Serial.println(F("# Sensor-only: D2=PUL+ D3=DIR+ reserved; motor pins are not configured."));
  Serial.println(F("# HC-SR04 assumed; IR 0/1 levels have no assigned detection polarity."));
  Serial.println(F("# Echo diagnostics follow each CSV row; # lines are ignored by the camera."));
  Serial.println(F("ms,echo_us,ultrasonic_status,distance_cm,ir1_d9,ir2_d10,ir3_d11,ir4_d12,ir5_d13"));
}

void loop() {
  const unsigned long sampleMs = millis();

  const EchoSample echo = measureEcho();
  const unsigned long echoUs = echo.widthUs;
  const float distanceCm = echoUs / 58.0f;  // Nominal HC-SR04 conversion, uncalibrated.
  const bool inNominalRange = echoUs != 0 && distanceCm >= 2.0f && distanceCm <= 400.0f;

  int irLevels[IR_COUNT];
  for (size_t i = 0; i < IR_COUNT; ++i) {
    irLevels[i] = digitalRead(IR_PINS[i]);
  }

  Serial.print(sampleMs);
  Serial.print(',');
  Serial.print(echoUs);
  Serial.print(',');
  if (echoUs == 0) {
    Serial.print(F("TIMEOUT,NA"));
  } else if (!inNominalRange) {
    Serial.print(F("OUT_OF_RANGE,NA"));
  } else {
    Serial.print(F("OK,"));
    Serial.print(distanceCm, 1);
  }
  for (size_t i = 0; i < IR_COUNT; ++i) {
    Serial.print(',');
    Serial.print(irLevels[i]);
  }
  Serial.println();

  Serial.print(F("# echo_debug phase="));
  Serial.print(echo.phase);
  Serial.print(F(" before="));
  Serial.print(echo.before);
  Serial.print(F(" after="));
  Serial.print(echo.after);
  Serial.print(F(" width_us="));
  Serial.println(echo.widthUs);

  // At least 200 ms between triggers. A missing echo has a finite timeout.
  delay(SAMPLE_PAUSE_MS);
}
