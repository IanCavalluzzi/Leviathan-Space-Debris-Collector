#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>

namespace sensors {
constexpr uint8_t TRIG_PIN = 7, ECHO_PIN = 6;
constexpr uint8_t IR_PINS[] = {9, 10, 11, 12, 13};
constexpr uint32_t SAMPLE_PERIOD_MS = 200, ECHO_TIMEOUT_US = 30000;
constexpr size_t ROW_CAPACITY = 64;
enum class Phase : uint8_t { Idle, TriggerLow, TriggerHigh, WaitRise, WaitFall, Done };

// Main-loop changes/snapshots must be interrupt-guarded. Only edge() runs in
// the Echo ISR; it records one pulse, never prints, waits or touches motors.
struct Acquisition {
  volatile Phase phase = Phase::Idle;
  volatile uint32_t phaseStartUs = 0, windowStartUs = 0, riseUs = 0, widthUs = 0;

  void begin(uint32_t nowUs, bool echoHigh) {
    widthUs = 0;
    phaseStartUs = nowUs;
    phase = echoHigh ? Phase::Done : Phase::TriggerLow;
  }
  bool triggering() const {
    return phase == Phase::TriggerLow || phase == Phase::TriggerHigh;
  }
  bool triggerHigh() const { return phase == Phase::TriggerHigh; }
  void advance(uint32_t nowUs, bool echoHigh) {
    const Phase current = phase;
    const uint32_t elapsed = nowUs - phaseStartUs;
    if (current == Phase::TriggerLow && elapsed >= 2) {
      // A late trigger service is invalid; do not accept a stretched pulse.
      if (echoHigh || elapsed >= 1000) { phase = Phase::Done; return; }
      phaseStartUs = nowUs;
      phase = Phase::TriggerHigh;
    } else if (current == Phase::TriggerHigh && elapsed >= 10) {
      windowStartUs = nowUs;
      phase = echoHigh || elapsed >= 1000 ? Phase::Done : Phase::WaitRise;
    } else if ((current == Phase::WaitRise || current == Phase::WaitFall) &&
               uint32_t(nowUs - windowStartUs) >= ECHO_TIMEOUT_US) {
      phase = Phase::Done;  // width stays zero: absent/stuck Echo is unknown.
    }
  }
  void edge(uint32_t nowUs, bool high) {
    const Phase current = phase;
    if (current != Phase::WaitRise && current != Phase::WaitFall) return;
    if (uint32_t(nowUs - windowStartUs) >= ECHO_TIMEOUT_US) {
      phase = Phase::Done;
    } else if (current == Phase::WaitRise && high) {
      riseUs = nowUs;
      phase = Phase::WaitFall;
    } else if (current == Phase::WaitFall && !high) {
      widthUs = nowUs - riseUs;
      phase = Phase::Done;
    }
  }
};

// Same nominal HC-SR04 conversion/range and nine CSV columns as sensor_test.
// Integer tenths avoid floating printf support; invalid range is always NA.
inline int formatRow(char* row, size_t capacity, uint32_t sampleMs,
                     uint32_t echoUs, const uint8_t ir[5]) {
  char range[12];
  const bool valid = echoUs >= 116 && echoUs <= 23200;
  const char* status = !echoUs ? "TIMEOUT" : valid ? "OK" : "OUT_OF_RANGE";
  if (valid) {
    const uint32_t tenths = (echoUs * 10 + 29) / 58;
    snprintf(range, sizeof(range), "%lu.%lu", static_cast<unsigned long>(tenths / 10),
             static_cast<unsigned long>(tenths % 10));
  } else {
    snprintf(range, sizeof(range), "NA");
  }
  return snprintf(row, capacity, "%lu,%lu,%s,%s,%u,%u,%u,%u,%u\n",
      static_cast<unsigned long>(sampleMs), static_cast<unsigned long>(echoUs), status, range,
      static_cast<unsigned>(ir[0]), static_cast<unsigned>(ir[1]), static_cast<unsigned>(ir[2]),
      static_cast<unsigned>(ir[3]), static_cast<unsigned>(ir[4]));
}
}  // namespace sensors
