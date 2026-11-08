// Runtime tests include the ACTUAL sketch, not a duplicate integration model.
#include <Arduino.h>
#include <FspTimer.h>
#include <stdio.h>
#include <string.h>
#include "../../../firmware/grip_control/grip_control.ino"

MockSerial Serial;
void (*mockTick)(timer_callback_args_t*) = nullptr;
static void (*echoCallback)() = nullptr;
static uint64_t clockUs = 0, nextTickUs = 1000, echoRiseAt = 0, echoFallAt = 0;
static bool irqEnabled = true;
static int levels[32] = {}, modes[32] = {};
static unsigned checks = 0, failures = 0, disabledWrites = 0, triggerStarts = 0;

void check(bool condition, const char* label) {
  ++checks;
  if (!condition) { ++failures; printf("FAIL: %s\n", label); }
}
uint32_t millis() { return static_cast<uint32_t>(clockUs / 1000); }
uint32_t micros() { return static_cast<uint32_t>(clockUs); }
void noInterrupts() { irqEnabled = false; }
void interrupts() { irqEnabled = true; }
int digitalRead(uint8_t pin) { return levels[pin]; }
void digitalWrite(uint8_t pin, int value) {
  if (pin == sensors::TRIG_PIN && !levels[pin] && value) ++triggerStarts;
  levels[pin] = value;
}
void pinMode(uint8_t pin, int mode) { modes[pin] = mode; }
void attachInterrupt(int pin, void (*callback)(), int mode) {
  check(pin == 6 && mode == CHANGE, "Echo CHANGE interrupt uses D6");
  echoCallback = callback;
}
void advanceTime(uint32_t deltaUs) {
  const uint64_t end = clockUs + deltaUs;
  while (clockUs < end) {
    ++clockUs;
    if (clockUs == nextTickUs) {
      nextTickUs += 1000;
      if (irqEnabled && mockTick) mockTick(nullptr);
    }
    if (clockUs == echoRiseAt || clockUs == echoFallAt) {
      levels[sensors::ECHO_PIN] = clockUs == echoRiseAt ? HIGH : LOW;
      if (irqEnabled && echoCallback) echoCallback();
    }
  }
}
void MockSerial::begin(int baud) { check(baud == 115200, "Baud stays 115200"); }
void MockSerial::receive(const char* text) {
  while (*text && inputSize < sizeof(input)) input[inputSize++] = *text++;
}
size_t MockSerial::write(uint8_t* data, size_t length) {
  if (!irqEnabled) ++disabledWrites;
  ++writes;
  const bool telemetry = length && data[0] >= '0' && data[0] <= '9';
  if (telemetry && stopDuringTelemetry) { receive("STOP\n"); stopDuringTelemetry = false; }
  for (size_t i = 0; i < length; ++i) {
    if (outputSize + 1 < sizeof(output)) output[outputSize++] = data[i];
    advanceTime(87);  // Slower rounded-up 115200-baud 8N1 UART wire cost.
  }
  output[outputSize] = '\0';
  return length;
}
void resetSketch(uint64_t startUs = 0) {
  clockUs = startUs; nextTickUs = (clockUs / 1000 + 1) * 1000;
  echoRiseAt = echoFallAt = 0;
  Serial = MockSerial{};
  motor = grip::Controller(MOTOR_CONFIG);
  echoCapture = sensors::Acquisition{};
  outputsReady = false;
  lastStep[0] = lastStep[1] = lastDir[0] = lastDir[1] = -1;
  receiveLength = 0; discardLine = sawCarriageReturn = false;
  acknowledgment[0] = sensorRow[0] = '\0'; pendingError = nullptr;
  reportedRevision = 0xFFFFFFFF;
  triggerStarts = disabledWrites = 0;
  irqEnabled = true;
  for (unsigned i = 0; i < 32; ++i) { levels[i] = 0; modes[i] = -1; }
  setup();
}
void runFor(uint32_t durationUs) {
  const uint64_t end = clockUs + durationUs;
  while (clockUs < end) { loop(); advanceTime(1); }
}
void command(const char* text) {
  Serial.receive(text);
  runFor(100);
}
void clearOutput() { Serial.outputSize = Serial.writes = 0; Serial.output[0] = '\0'; }
void prepareRest() {
  command("REFERENCE_NEUTRAL\n");
  check(motor.referenced && !motor.moving && motor.state == grip::State::Positioned &&
        motor.axis[0].position == 0 && motor.axis[1].position == 0,
        "Physical-zero declaration does not create Rest or move either axis");
  command("ARM\n"); command("GRAB\n");
  check(!motor.moving && motor.result == grip::Reply::Ok && motor.jobId == 0 &&
        strstr(Serial.output, "NOT_NEUTRAL"), "Actual sketch rejects Grab before preparation");
  command("NEUTRAL\n");
  for (unsigned i = 0; i < 6; ++i) { runFor(300000); command("STATUS\n"); }
  check(!motor.moving && motor.referenced && motor.state == grip::State::Neutral &&
        motor.axis[0].position == 900 && motor.axis[1].position == 0,
        "Explicit preparation reaches Rest by pulling A only");
}

void acquisitionTests() {
  using sensors::Phase;
  sensors::Acquisition a;
  a.begin(100, false);
  check(a.phase == Phase::TriggerLow && !a.triggerHigh(), "Trigger starts low");
  a.advance(101, false); check(a.phase == Phase::TriggerLow, "Two us low minimum");
  a.advance(102, false); check(a.triggerHigh(), "Trigger goes high after low phase");
  a.advance(111, false); check(a.triggerHigh(), "Ten us high minimum");
  a.advance(112, false); check(a.phase == Phase::WaitRise, "Trigger ends without waiting for Echo");
  a.edge(150, false); check(a.phase == Phase::WaitRise, "Unmatched falling edge ignored");
  a.edge(200, true); a.edge(201, true); a.edge(780, false);
  check(a.phase == Phase::Done && a.widthUs == 580, "First rise/fall captured despite delayed service");
  a.advance(40000, false); a.edge(41000, true);
  check(a.widthUs == 580, "Completed pulse immutable until next sample");
  a.begin(0, false); a.advance(2, false); a.advance(12, false);
  a.advance(30011, false); check(a.phase == Phase::WaitRise, "No rise deadline is finite and exclusive");
  a.advance(30012, false); check(a.phase == Phase::Done && !a.widthUs, "Absent rise times out");
  a.begin(0, false); a.advance(2, false); a.advance(12, false); a.edge(50, true);
  a.advance(30012, true); check(a.phase == Phase::Done && !a.widthUs, "Absent fall times out");
  a.begin(0, true); check(a.phase == Phase::Done && !a.widthUs, "High before trigger is unknown");
  a.begin(0, false); a.advance(1000, false);
  check(a.phase == Phase::Done, "Overdue trigger-low phase aborts");
  a.begin(0, false); a.advance(2, false); a.advance(1002, false);
  check(a.phase == Phase::Done && !a.triggerHigh(), "Overdue trigger-high phase aborts low");
  a.begin(0, false); a.advance(2, false); a.advance(12, false); a.edge(30012, true);
  check(a.phase == Phase::Done && !a.widthUs, "Late rise cannot revive expired acquisition");
  a.begin(0, false); a.advance(2, false); a.advance(12, false); a.edge(50, true); a.edge(30012, false);
  check(a.phase == Phase::Done && !a.widthUs, "Late fall is TIMEOUT, not a stale range");
  const uint32_t base = 0xFFFFFF00u;
  a.begin(base, false); a.advance(base + 2, false); a.advance(base + 12, false);
  a.edge(base + 100, true); a.edge(base + 680, false);
  check(a.widthUs == 580, "Echo width survives micros rollover");
  a.begin(base, false); a.advance(base + 2, false); a.advance(base + 12, false);
  a.advance(base + 30012, false);
  check(a.phase == Phase::Done && !a.widthUs, "Timeout survives micros rollover");
  a.begin(0xFFFFFFFEu, false); a.advance(0, false); a.advance(10, false);
  check(a.phase == Phase::WaitRise, "Trigger phases survive micros rollover");
}

void formatTests() {
  char row[sensors::ROW_CAPACITY]; const uint8_t ir[] = {0, 1, 0, 1, 1};
  const uint32_t widths[] = {0, 1, 115, 116, 580, 23200, 23201, 29999};
  const char* expected[] = {
    "4294967295,0,TIMEOUT,NA,0,1,0,1,1\n", "4294967295,1,OUT_OF_RANGE,NA,0,1,0,1,1\n",
    "4294967295,115,OUT_OF_RANGE,NA,0,1,0,1,1\n", "4294967295,116,OK,2.0,0,1,0,1,1\n",
    "4294967295,580,OK,10.0,0,1,0,1,1\n", "4294967295,23200,OK,400.0,0,1,0,1,1\n",
    "4294967295,23201,OUT_OF_RANGE,NA,0,1,0,1,1\n",
    "4294967295,29999,OUT_OF_RANGE,NA,0,1,0,1,1\n"};
  for (unsigned i = 0; i < sizeof(widths) / sizeof(widths[0]); ++i) {
    const int n = sensors::formatRow(row, sizeof(row), 0xFFFFFFFFu, widths[i], ir);
    check(n > 0 && n < int(sizeof(row)) && strcmp(row, expected[i]) == 0,
          "Exact nine-column CSV, nominal bounds, NA and raw IR ordering");
    printf("CSV_FIXTURE:%s", row);
  }
}

void integrationTests() {
  resetSketch();
  check(!motor.armed && !motor.referenced && !motor.moving, "Boot disarmed/unreferenced, no movement");
  check(modes[6] == INPUT && modes[7] == OUTPUT && modes[9] == INPUT && modes[13] == INPUT,
        "Sensor pins configured without pullups");
  runFor(650000);
  check(strstr(Serial.output, ",0,TIMEOUT,NA,0,0,0,0,0\n") != nullptr, "Missing Echo reports unknown");
  check(triggerStarts == 4, "Absent Echo sampling bounded to one start per 200 ms");
  check(strstr(Serial.output, "# motor UNREFERENCED 1 0 0 0 0 OK 0 0 -1\n") != nullptr,
        "NO_USB zero availableForWrite does not suppress motor state");
  check(disabledWrites == 0, "All serial writes retain interrupts");
  prepareRest(); command("ARM\n"); command("GRAB\n"); runFor(20000);
  check(motor.moving && motor.axis[1].position > 0, "Motor pulses continue with no Echo");
  clearOutput(); command("STOP\n"); runFor(10000);
  check(!motor.moving && !motor.referenced && !motor.armed && motor.result == grip::Reply::Stopped,
        "Missing Echo cannot block STOP or pulse tails");
  check(strstr(Serial.output, "# motor ACK OK 2\n") && strstr(Serial.output, "STOPPED"),
        "STOP ACK/state contract preserved");

  resetSketch(); runFor(40000); clearOutput();
  prepareRest(); command("ARM\n"); command("GRAB\n");
  runFor(18000); clearOutput();
  const int32_t before = motor.axis[1].position;
  // Prepare a ready sample: main loop's row sender is exercised directly so a
  // STOP can arrive at the first byte, the worst position within a sensor row.
  const uint8_t raw[] = {1, 0, 1, 0, 1};
  sensors::formatRow(sensorRow, sizeof(sensorRow), millis(), 580, raw);
  Serial.stopDuringTelemetry = true;
  const uint64_t sentAt = clockUs;
  reportSensors();
  check(clockUs - sentAt < 5600 && Serial.available() == 5, "At most one bounded row precedes arriving STOP");
  check(motor.axis[1].position > before, "Motor timer ticks during synchronous telemetry write");
  loop();
  check(motor.result == grip::Reply::Stopped && clockUs - sentAt < 20000,
        "STOP executes on next loop without a telemetry backlog");
  check(Serial.output[0] >= '0' && strstr(Serial.output, ",OK,10.0,1,0,1,0,1\n# motor ACK OK 2\n"),
        "Telemetry and ACK are complete separate lines");
  check(disabledWrites == 0, "Motor IRQ remains enabled during mixed output");

  resetSketch(); runFor(40000); clearOutput();
  // A whole valid Echo pulse arrives during a motor STATUS UART write.
  echoCapture.begin(micros(), false);
  echoCapture.advance(micros() + 2, false); echoCapture.advance(micros() + 12, false);
  advanceTime(12); echoRiseAt = clockUs + 100; echoFallAt = echoRiseAt + 580;
  command("STATUS\n");
  runFor(100);
  check(strstr(Serial.output, ",580,OK,10.0,0,0,0,0,0\n"), "Interrupt captures Echo through UART activity");

  resetSketch(); runFor(40000); clearOutput();
  sensors::formatRow(sensorRow, sizeof(sensorRow), millis(), 580, raw);
  Serial.receive("STATUS\nSTOP\n");
  loop(); loop();
  check(!sensorRow[0] || Serial.output[0] == '#', "Queued commands/ACKs precede telemetry");
  check(strstr(Serial.output, "# motor ACK OK 0\n"), "Queued STATUS/STOP remain responsive");
  clearOutput(); levels[6] = HIGH; runFor(450000);
  check(strstr(Serial.output, ",0,TIMEOUT,NA,"), "Stuck-high Echo remains unknown in actual sketch");
  check(!motor.moving && !motor.armed, "Stuck-high Echo never starts motors");

  resetSketch(uint64_t(0xFFFFFFF0u) * 1000); runFor(450000);
  check(triggerStarts == 3 && strstr(Serial.output, "TIMEOUT,NA"), "Sampling cadence survives millis rollover");
}

int main() {
  acquisitionTests(); formatTests(); integrationTests();
  resetSketch(); runFor(40000); clearOutput();
  command("REFERENCE_REST\n");
  check(motor.referenced && !motor.moving && !motor.armed && motor.jobId == 0 &&
        motor.axis[0].position == 900 && motor.axis[1].position == 0 &&
        motor.state == grip::State::Neutral && !motor.axis[0].stepActive && !motor.axis[1].stepActive,
        "Actual retracted reference declares Rest without a move or ARM");
  command("ARM\n"); command("GRAB\n"); runFor(2000000);
  check(!motor.moving && motor.axis[0].position == 0 && motor.axis[1].position == 900 &&
        motor.state == grip::State::HoldUnconfirmed, "Grab works from an already-retracted startup declaration");
  resetSketch(); runFor(40000); clearOutput();
  command("SET_STROKES -900 +700\n");
  check(!motor.moving && !motor.armed && !motor.referenced && motor.jobId == 0 &&
        motor.axis[0].position == 0 && motor.axis[1].position == 0 &&
        strstr(Serial.output, "# motor CONTROLS 1\n") &&
        strstr(Serial.output, "# motor PROFILE 0 0 -900 0 0 700\n"),
        "Actual signed settings apply without movement and report live endpoints");
  command("REFERENCE_REST\n"); command("ARM\n"); command("GRAB\n"); runFor(2000000);
  check(!motor.moving && motor.axis[0].position == 0 && motor.axis[1].position == 700,
        "Actual sketch uses edited signed pulse counts for coordinated Grab");
  resetSketch(); runFor(40000);
  check(strstr(Serial.output, "# motor SAFETY 0 0 1500 10000\n"),
        "Boot advertises physical readiness separately from unloaded override and guard off");
  check(strstr(Serial.output, "# motor PROFILE 0 0 900 0 0 900\n"),
        "Installed PROFILE advertises distinct physical zero, Rest and Grasp");
  clearOutput(); command("GUARD\n");
  check(motor.guardActive && strstr(Serial.output, "# motor SAFETY 0 1 1500 10000\n"),
        "Exact GUARD command acknowledges and advertises its active lease");
  prepareRest(); command("ARM\n"); command("GRAB\n");
  runFor(1600000);
  check(!motor.moving && !motor.guardActive && !motor.armed && !motor.referenced &&
        motor.result == grip::Reply::GuardTimeout && motor.state == grip::State::Unreferenced,
        "Actual timer stops an interrupted guarded job on lost communication");
  check(strstr(Serial.output, "GUARD_TIMEOUT") && !motor.axis[0].stepActive && !motor.axis[1].stepActive,
        "Guard timeout is reported after both pulse tails complete");
  clearOutput(); command("STATUS\n");
  check(!motor.guardActive && strstr(Serial.output, "# motor SAFETY 0 0 1500 10000\n"),
        "Reconnected STATUS cannot reactivate an expired guarded session");
  command("REFERENCE_NEUTRAL\n"); clearOutput(); command("ARM\n");
  check(!motor.armed && strstr(Serial.output, "# motor ERR GUARD_TIMEOUT 2\n"),
        "A guarded session needs explicit fresh GUARD before another ARM");
  command("GUARD\n"); command("ARM\n"); command("JOG A 1\n"); runFor(20000);
  check(motor.guardActive && motor.jobId == 3 && !motor.moving && motor.axis[0].position == 1,
        "Fresh explicit GUARD allows a deliberately referenced new finite job");

  resetSketch(); runFor(40000);
  command("GUARD\n"); prepareRest(); command("ARM\n"); command("GRAB\n");
  for (unsigned i = 0; i < 20; ++i) { runFor(250000); command("STATUS\n"); }
  check(motor.guardActive && !motor.moving && motor.referenced &&
        motor.state == grip::State::HoldUnconfirmed && motor.axis[0].position == 0 && motor.axis[1].position == 900,
        "Actual STATUS polling sustains a full coordinated guarded stroke with sensor output");
  const int32_t heldA = motor.axis[0].position, heldB = motor.axis[1].position;
  runFor(1600000);
  check(!motor.guardActive && !motor.armed && motor.referenced && motor.state == grip::State::HoldUnconfirmed &&
        motor.axis[0].position == heldA && motor.axis[1].position == heldB,
        "Loss after completion never unwinds or invalidates the held commanded reference");

  resetSketch(); runFor(40000);
  command("GUARD\n"); command("REFERENCE_NEUTRAL\n"); command("ARM\n");
  for (unsigned i = 0; i < 42; ++i) { runFor(250000); command("STATUS\n"); }
  check(motor.guardActive && !motor.armed && motor.referenced && !motor.moving &&
        motor.result == grip::Reply::ArmExpired && strstr(Serial.output, "ARM_EXPIRED"),
        "Actual idle timer expires ARM even while valid STATUS polls sustain communication");

  resetSketch(); runFor(40000);
  command("GUARD\n"); runFor(1000000);
  command("JOG A 1\n");  // NO_REFERENCE rejection cannot renew the lease.
  command("GUARD 1\n"); // Syntactically invalid payload cannot renew it either.
  runFor(550000);
  check(!motor.guardActive && motor.result == grip::Reply::GuardTimeout,
        "Malformed and rejected actual wire commands never refresh the guard lease");
  command("STOP\n"); command("REFERENCE_NEUTRAL\n"); command("ARM\n"); command("JOG A 1\n"); runFor(20000);
  check(!motor.guardActive && !motor.moving && motor.axis[0].position == 1 && motor.result == grip::Reply::Complete,
        "Explicit STOP exits expired guard mode for normal deliberate manual commands");
  printf("Sensor checks: %u, failures: %u\n", checks, failures);
  return failures ? 1 : 0;
}
