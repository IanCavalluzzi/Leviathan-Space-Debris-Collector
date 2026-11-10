// Two-axis motor control and sensor telemetry for the UNO R4 WiFi.
#include <Arduino.h>
#include <FspTimer.h>
#include <stdio.h>
#include <string.h>
#include "grip_config.h"
#include "sensor_logic.h"

#if !defined(ARDUINO_UNOR4_WIFI)
#error "Select Arduino UNO R4 WiFi. This sketch uses the RA4M1 normal headers."
#endif

FspTimer pulseTimer;
grip::Controller motor(MOTOR_CONFIG);
volatile bool outputsReady = false;
int lastStep[2] = {-1, -1};
int lastDir[2] = {-1, -1};
char receiveLine[40];
size_t receiveLength = 0;
bool discardLine = false;
bool sawCarriageReturn = false;
uint32_t reportedRevision = 0xFFFFFFFF;
const char* pendingError = nullptr;
char acknowledgment[80] = {};
sensors::Acquisition echoCapture;
uint32_t lastSensorStartMs = 0, sensorSampleMs = 0;
char sensorRow[sensors::ROW_CAPACITY] = {};

void echoEdge() {
  const uint32_t nowUs = micros();
  echoCapture.edge(nowUs, digitalRead(sensors::ECHO_PIN) == HIGH);
}

void setupSensors() {
  digitalWrite(sensors::TRIG_PIN, LOW);
  pinMode(sensors::TRIG_PIN, OUTPUT);
  pinMode(sensors::ECHO_PIN, INPUT);
  for (uint8_t pin : sensors::IR_PINS) pinMode(pin, INPUT);  // Raw, no pull-ups.
  attachInterrupt(digitalPinToInterrupt(sensors::ECHO_PIN), echoEdge, CHANGE);
  lastSensorStartMs = millis() - sensors::SAMPLE_PERIOD_MS;
}

void serviceSensors() {
  const uint32_t nowMs = millis();
  uint32_t echoUs = 0;
  bool complete = false;
  noInterrupts();
  const uint32_t nowUs = micros();
  if (echoCapture.phase == sensors::Phase::Idle &&
      uint32_t(nowMs - lastSensorStartMs) >= sensors::SAMPLE_PERIOD_MS) {
    lastSensorStartMs = sensorSampleMs = nowMs;
    sensorRow[0] = '\0';  // No telemetry queue or stale retry across samples.
    echoCapture.begin(nowUs, digitalRead(sensors::ECHO_PIN) == HIGH);
  } else {
    echoCapture.advance(nowUs, digitalRead(sensors::ECHO_PIN) == HIGH);
  }
  digitalWrite(sensors::TRIG_PIN, echoCapture.triggerHigh() ? HIGH : LOW);
  if (echoCapture.phase == sensors::Phase::Done) {
    echoUs = echoCapture.widthUs;
    echoCapture.phase = sensors::Phase::Idle;
    complete = true;
  }
  interrupts();
  if (!complete) return;
  uint8_t ir[5];
  for (uint8_t i = 0; i < 5; ++i) ir[i] = digitalRead(sensors::IR_PINS[i]) == HIGH;
  const int length = sensors::formatRow(sensorRow, sizeof(sensorRow), sensorSampleMs, echoUs, ir);
  if (length <= 0 || length >= static_cast<int>(sizeof(sensorRow))) sensorRow[0] = '\0';
}

// motor is shared only across this ISR and interrupt-guarded main-loop sections.
void applyOutputs() {
  if (!outputsReady) return;
  for (uint8_t i = 0; i < 2; ++i) {
    const int direction = motor.dirLevel(i);
    const int step = motor.stepLevel(i);
    if (direction != lastDir[i]) { digitalWrite(MOTOR_CONFIG.axis[i].dirPin, direction); lastDir[i] = direction; }
    if (step != lastStep[i]) { digitalWrite(MOTOR_CONFIG.axis[i].stepPin, step); lastStep[i] = step; }
  }
}

void pulseTick(timer_callback_args_t*) {
  motor.tick(millis());
  applyOutputs();
}

const char* replyName(grip::Reply r) {
  using grip::Reply;
  switch (r) {
    case Reply::Ok: return "OK";
    case Reply::Accepted: return "ACCEPTED";
    case Reply::Complete: return "COMPLETE";
    case Reply::BadCommand: return "BAD_COMMAND";
    case Reply::Locked: return "CONFIG_LOCKED";
    case Reply::Busy: return "BUSY";
    case Reply::Disarmed: return "DISARMED";
    case Reply::MustDisarm: return "DISARM_FIRST";
    case Reply::NoReference: return "NO_REFERENCE";
    case Reply::Limit: return "TRAVEL_LIMIT";
    case Reply::NotNeutral: return "NOT_NEUTRAL";
    case Reply::AlreadyNeutral: return "ALREADY_NEUTRAL";
    case Reply::ReferenceChanged: return "USE_REFERENCE_NEUTRAL";
    case Reply::Deadline: return "MOVE_TOO_LONG";
    case Reply::Stopped: return "STOPPED";
    case Reply::TimedOut: return "MOVE_TIMEOUT";
    case Reply::AxisFault: return "AXIS_FAULT";
    case Reply::GuardTimeout: return "GUARD_TIMEOUT";
    case Reply::ArmExpired: return "ARM_EXPIRED";
    default: return "TIMER_FAULT";
  }
}

const char* stateName(grip::State s) {
  using grip::State;
  switch (s) {
    case State::Neutral: return "NEUTRAL";
    case State::Positioned: return "POSITIONED";
    case State::Jogging: return "JOGGING";
    case State::Grabbing: return "GRABBING";
    case State::HoldUnconfirmed: return "HOLD_UNCONFIRMED";
    case State::Returning: return "RETURNING";
    case State::Stopping: return "STOPPING";
    default: return "UNREFERENCED";
  }
}

bool sendLine(char* text) {
  const size_t length = strlen(text);
  // UNO R4 WiFi uses the UART-to-ESP32 USB bridge (NO_USB). Its UART
  // inherits availableForWrite()==0, so gating on it suppresses all output.
  // A bounded line transmits locally; motor pulse/deadline IRQs stay enabled.
#if !defined(NO_USB)
  if (Serial.availableForWrite() < static_cast<int>(length)) return false;
#endif
  return Serial.write(reinterpret_cast<uint8_t*>(text), length) == length;
}

void reportSensors() {
  // Commands/ACK/errors/state precede expendable telemetry. The UART write is
  // synchronous on NO_USB: send at most one <64-byte row (~5.5 ms at 115200),
  // never a backlog, with timer/Echo interrupts enabled throughout the write.
  if (!sensorRow[0] || acknowledgment[0] || pendingError || receiveLength ||
      discardLine || sawCarriageReturn || Serial.available()) return;
  noInterrupts();
  const bool statePending = motor.revision != reportedRevision;
  interrupts();
  if (!statePending) sendLine(sensorRow);
  sensorRow[0] = '\0';  // Drop rather than queue/retry telemetry under pressure.
}

void handleLine() {
  receiveLine[receiveLength] = '\0';
  const grip::Parsed parsed = grip::parse(receiveLine);
  const uint32_t nowMs = millis();
  noInterrupts();
  const grip::Reply reply = motor.command(parsed.command, parsed.amount, nowMs, parsed.axis, parsed.other);
  const uint32_t jobId = motor.jobId;
  applyOutputs();
  interrupts();
  // One bounded acknowledgment per input; queued output never masks a rejection.
  snprintf(acknowledgment, sizeof(acknowledgment), "# motor %s %s %lu\n",
      reply == grip::Reply::Ok ? "ACK" : "ERR", replyName(reply), static_cast<unsigned long>(jobId));
  reportedRevision = 0xFFFFFFFF;  // STATUS/command acknowledgment uses a snapshot.
}

void readCommands() {
  if (acknowledgment[0] || pendingError) return;
  // Bound input work so a continuously transmitting host cannot monopolize loop().
  for (uint8_t count = 0; count < 32 && Serial.available(); ++count) {
    const int byte = Serial.read();
    if (byte < 0) break;
    if (byte == '\r') { sawCarriageReturn = true; continue; }
    if (byte == '\n') {
      if (!discardLine && receiveLength) handleLine();
      discardLine = false;
      receiveLength = 0;
      sawCarriageReturn = false;
      return;  // Report this command before accepting another one.
    } else if (!discardLine) {
      if (sawCarriageReturn || byte < 32 || byte > 126 || receiveLength >= sizeof(receiveLine) - 1) {
        noInterrupts();
        motor.halt(grip::Reply::BadCommand);
        interrupts();
        pendingError = "BAD_LINE";
        discardLine = true;
        receiveLength = 0;
      } else receiveLine[receiveLength++] = static_cast<char>(byte);
    }
  }
}

void reportState() {
  char text[128];
  if (acknowledgment[0]) {
    if (!sendLine(acknowledgment)) return;
    acknowledgment[0] = '\0';
  }
  if (pendingError) {
    snprintf(text, sizeof(text), "# motor ERR %s\n", pendingError);
    if (!sendLine(text)) return;
    pendingError = nullptr;
  }
  noInterrupts();
  const uint32_t revision = motor.revision;
  const grip::State state = motor.state;
  const bool armed = motor.armed;
  const bool referenced = motor.referenced;
  const int32_t positionA = motor.axis[0].position;
  const int32_t positionB = motor.axis[1].position;
  const uint32_t jobId = motor.jobId;
  const int8_t faultAxis = motor.faultAxis;
  const grip::Job job = motor.job;
  const grip::Reply result = motor.result;
  const bool configured = motor.configured();
  const bool physicalReady = motor.physicalReady();
  const bool guardActive = motor.guardActive;
  interrupts();
  if (revision == reportedRevision) return;
  // Report the limits enforced by this board.
  const int limitsLength = snprintf(text, sizeof(text), "# motor LIMITS %ld %ld %ld %ld %ld %ld\n",
      static_cast<long>(MOTOR_CONFIG.axis[0].minPosition), static_cast<long>(MOTOR_CONFIG.axis[0].maxPosition),
      static_cast<long>(MOTOR_CONFIG.axis[0].maxJogSteps), static_cast<long>(MOTOR_CONFIG.axis[1].minPosition),
      static_cast<long>(MOTOR_CONFIG.axis[1].maxPosition), static_cast<long>(MOTOR_CONFIG.axis[1].maxJogSteps));
  if (limitsLength <= 0 || limitsLength >= static_cast<int>(sizeof(text)) || !sendLine(text)) return;
  char controls[] = "# motor CONTROLS 1\n";  // REFERENCE_REST and signed SET_STROKES supported.
  if (!sendLine(controls)) return;
  // Report the live profile because SET_STROKES can change it.
  const int profileLength = snprintf(text, sizeof(text), "# motor PROFILE %ld %ld %ld %ld %ld %ld\n",
      static_cast<long>(motor.config.reference[0]), static_cast<long>(motor.config.reference[1]),
      static_cast<long>(motor.config.neutral[0]), static_cast<long>(motor.config.neutral[1]),
      static_cast<long>(motor.config.grasp[0]), static_cast<long>(motor.config.grasp[1]));
  if (profileLength <= 0 || profileLength >= static_cast<int>(sizeof(text)) || !sendLine(text)) return;
  const int safetyLength = snprintf(text, sizeof(text), "# motor SAFETY %u %u %lu %lu\n",
      physicalReady, guardActive, static_cast<unsigned long>(grip::GUARD_TIMEOUT_MS),
      static_cast<unsigned long>(grip::ARM_TIMEOUT_MS));
  if (safetyLength <= 0 || safetyLength >= static_cast<int>(sizeof(text)) || !sendLine(text)) return;
  // Config, armed, reference, commanded position. All events are CSV comments.
  const int length = snprintf(text, sizeof(text), "# motor %s %u %u %u %ld %ld %s %lu %u %d\n",
      stateName(state), configured, armed, referenced, static_cast<long>(positionA),
      static_cast<long>(positionB), replyName(result), static_cast<unsigned long>(jobId),
      static_cast<unsigned>(job), static_cast<int>(faultAxis));
  if (length > 0 && length < static_cast<int>(sizeof(text)) && sendLine(text)) reportedRevision = revision;
}

void setup() {
  Serial.begin(115200);
  setupSensors();  // Sensors still report unknown if the motor config/timer locks.
  if (!motor.configured()) {
    motor.result = grip::Reply::Locked;
    pendingError = "CONFIG_LOCKED";
    return;
  }

  uint8_t timerType = GPT_TIMER;
  const int8_t channel = FspTimer::get_available_timer(timerType);
  const bool timerOk = channel >= 0 &&
      pulseTimer.begin(TIMER_MODE_PERIODIC, timerType, channel, 1000.0f, 50.0f, pulseTick) &&
      pulseTimer.setup_overflow_irq() && pulseTimer.open() && pulseTimer.start();
  if (!timerOk) {
    motor.failTimer();
    pendingError = "TIMER_FAULT";
    return;
  }
  // No outputs until explicit interface/config validation and timer initialization.
  for (uint8_t i = 0; i < 2; ++i) {
    digitalWrite(MOTOR_CONFIG.axis[i].stepPin, motor.stepLevel(i));
    digitalWrite(MOTOR_CONFIG.axis[i].dirPin, motor.dirLevel(i));
    pinMode(MOTOR_CONFIG.axis[i].stepPin, OUTPUT);
    pinMode(MOTOR_CONFIG.axis[i].dirPin, OUTPUT);
  }
  noInterrupts();
  outputsReady = true;
  applyOutputs();
  interrupts();
}

void loop() {
  readCommands();
  serviceSensors();
  // Keep UART writes out of the 2/10 us trigger phases. No busy wait or delay;
  // command parsing and the independent motor timer continue on every loop.
  if (echoCapture.triggering()) return;
  reportState();
  readCommands();  // A STOP arriving during a state write precedes telemetry.
  reportSensors();
}
