#pragma once
#include <stdint.h>

namespace grip {
// Conservative 1 ms scheduling quantum; physical waveforms remain unverified.
constexpr uint32_t TICK_US = 1000;
// Software session defaults, not measured physical stop/holding limits.
constexpr uint32_t GUARD_TIMEOUT_MS = 1500;
constexpr uint32_t ARM_TIMEOUT_MS = 10000;
constexpr uint32_t ticks(uint32_t us) { return (us + TICK_US - 1) / TICK_US; }
constexpr uint32_t larger(uint32_t a, uint32_t b) { return a > b ? a : b; }
constexpr uint32_t magnitude(int32_t n) { return n < 0 ? uint32_t(-int64_t(n)) : uint32_t(n); }
struct AxisConfig {
  int8_t stepPin, dirPin;
  bool interfaceVerified;
  int8_t stepActiveLevel, positiveDirLevel;
  int32_t minPosition, maxPosition, maxJogSteps;
  uint32_t stepsPerSecond, minPulseActiveUs, minPulseIdleUs, dirSetupUs, dirHoldUs;
};
struct Config {
  AxisConfig axis[2];
  bool profileVerified;
  int32_t neutral[2], grasp[2];  // neutral is prepared Rest, not physical zero.
  uint32_t jobTimeoutMs;
  // Explicit user override for unloaded tests, not electrical/mechanism verification.
  bool unloadedTestAuthorized = false;
  int32_t reference[2] = {0, 0};  // Coordinates assigned by REFERENCE_NEUTRAL.
};
// Normal UNO headers; reserve serial D0/D1, ultrasound D6/D7, IR D9-D13.
constexpr bool motorPin(int8_t pin) {
  return pin >= 2 && pin <= 19 && pin != 6 && pin != 7 && !(pin >= 9 && pin <= 13);
}
constexpr bool valid(const AxisConfig& c, bool unloadedTestAuthorized = false) {
  return motorPin(c.stepPin) && motorPin(c.dirPin) && c.stepPin != c.dirPin &&
         (c.interfaceVerified || unloadedTestAuthorized) && c.stepActiveLevel >= 0 && c.stepActiveLevel <= 1 &&
         c.positiveDirLevel >= 0 && c.positiveDirLevel <= 1 &&
         c.minPosition >= -1000000 && c.maxPosition <= 1000000 && c.minPosition < c.maxPosition &&
         c.maxJogSteps > 0 && c.maxJogSteps <= c.maxPosition - c.minPosition &&
         c.stepsPerSecond > 0 && c.stepsPerSecond <= 500 &&
         c.minPulseActiveUs > 0 && c.minPulseActiveUs <= 1000000 &&
         c.minPulseIdleUs > 0 && c.minPulseIdleUs <= 1000000 &&
         c.dirSetupUs > 0 && c.dirSetupUs <= 1000000 &&
         c.dirHoldUs > 0 && c.dirHoldUs <= 1000000;
}
constexpr bool within(const AxisConfig& c, int32_t p) { return p >= c.minPosition && p <= c.maxPosition; }
constexpr bool valid(const Config& c) {
  if (!valid(c.axis[0], c.unloadedTestAuthorized) || !valid(c.axis[1], c.unloadedTestAuthorized) ||
      (!c.profileVerified && !c.unloadedTestAuthorized) ||
      c.jobTimeoutMs == 0 || c.jobTimeoutMs > 60000) return false;
  if (c.unloadedTestAuthorized) {
    // User-selected unloaded full-stroke test envelope, not calibrated tendon limits.
    if (c.jobTimeoutMs > 10000) return false;
    for (uint8_t i = 0; i < 2; ++i)
      if (c.axis[i].minPosition < -2500 || c.axis[i].maxPosition > 2500 || c.axis[i].stepsPerSecond > 500) return false;
  }
  if (c.axis[0].stepPin == c.axis[1].stepPin || c.axis[0].stepPin == c.axis[1].dirPin ||
      c.axis[0].dirPin == c.axis[1].stepPin || c.axis[0].dirPin == c.axis[1].dirPin) return false;
  for (uint8_t i = 0; i < 2; ++i)
    if (!within(c.axis[i], c.reference[i]) || !within(c.axis[i], c.neutral[i]) ||
        !within(c.axis[i], c.grasp[i])) return false;
  return c.neutral[0] != c.grasp[0] || c.neutral[1] != c.grasp[1];
}
enum class Command { Invalid, Status, Guard, Arm, Disarm, ReferenceNeutral, ReferenceOpen, Jog, Grab, Neutral, Stop, ReferenceRest, SetStrokes };
enum class Reply { Ok, Accepted, Complete, BadCommand, Locked, Busy, Disarmed, MustDisarm,
                   NoReference, Limit, NotNeutral, AlreadyNeutral, ReferenceChanged,
                   Deadline, Stopped, TimedOut, AxisFault, TimerFault, GuardTimeout, ArmExpired };
enum class State { Unreferenced, Neutral, Positioned, Jogging, Grabbing, HoldUnconfirmed, Returning, Stopping };
enum class Job { None, Jog, Grab, Neutral };
struct Parsed { Command command; int32_t amount; uint8_t axis; int32_t other = 0; };
constexpr bool equal(const char* a, const char* b) {
  while (*a && *a == *b) { ++a; ++b; }
  return *a == *b;
}
constexpr bool startsWith(const char* text, const char* prefix) {
  while (*prefix) if (*text++ != *prefix++) return false;
  return true;
}
constexpr bool signedNumber(const char*& text, int32_t& value) {
  const bool negative = *text == '-';
  if (*text == '-' || *text == '+') ++text;
  if (*text < '0' || *text > '9') return false;
  value = 0;
  while (*text >= '0' && *text <= '9') {
    if (value > (1000000 - (*text - '0')) / 10) return false;
    value = value * 10 + (*text++ - '0');
  }
  if (negative) value = -value;
  return true;
}
// Exact payloads; legacy JOG N retains driver-A commissioning semantics.
constexpr Parsed parse(const char* line) {
  if (equal(line, "STATUS")) return {Command::Status, 0, 0};
  if (equal(line, "GUARD")) return {Command::Guard, 0, 0};
  if (equal(line, "ARM")) return {Command::Arm, 0, 0};
  if (equal(line, "DISARM")) return {Command::Disarm, 0, 0};
  if (equal(line, "REFERENCE_NEUTRAL")) return {Command::ReferenceNeutral, 0, 0};
  if (equal(line, "REFERENCE_REST")) return {Command::ReferenceRest, 0, 0};
  if (equal(line, "REFERENCE_OPEN")) return {Command::ReferenceOpen, 0, 0};
  if (equal(line, "GRAB") || equal(line, "CAPTURE")) return {Command::Grab, 0, 0};
  if (equal(line, "NEUTRAL") || equal(line, "RELEASE")) return {Command::Neutral, 0, 0};
  if (equal(line, "STOP")) return {Command::Stop, 0, 0};
  if (startsWith(line, "SET_STROKES ")) {
    line += 12;
    int32_t a = 0, b = 0;
    if (!signedNumber(line, a) || *line++ != ' ' || !signedNumber(line, b) || *line)
      return {Command::Invalid, 0, 0};
    return {Command::SetStrokes, a, 0, b};
  }
  if (line[0] != 'J' || line[1] != 'O' || line[2] != 'G' || line[3] != ' ') return {Command::Invalid, 0, 0};
  line += 4;
  uint8_t axis = 0;
  if (*line == 'A' || *line == 'B') {
    axis = *line++ == 'B' ? 1 : 0;
    if (*line != ' ') return {Command::Invalid, 0, 0};
    ++line;
  }
  const bool negative = *line == '-';
  if (negative) ++line;
  if (!*line) return {Command::Invalid, 0, 0};
  int32_t value = 0;
  while (*line) {
    if (*line < '0' || *line > '9' || value > (1000000 - (*line - '0')) / 10) return {Command::Invalid, 0, 0};
    value = value * 10 + (*line++ - '0');
  }
  return {Command::Jog, negative ? -value : value, axis};
}
struct Axis {
  int32_t position = 0;  // Commanded pulse count, never a shaft measurement.
  bool stepActive = false, positive = true;
};
class Controller {
 public:
  Config config;
  Axis axis[2];
  State state = State::Unreferenced;
  Reply result = Reply::Ok;
  Job job = Job::None;
  bool armed = false, referenced = false, moving = false, timerHealthy = true;
  bool guardActive = false;
  int8_t faultAxis = -1;
  uint32_t revision = 0, jobId = 0;
  constexpr explicit Controller(Config c) : config(c) {}
  constexpr bool configured() const { return timerHealthy && profileReady(); }
  constexpr bool physicalReady() const {
    return config.profileVerified && config.axis[0].interfaceVerified && config.axis[1].interfaceVerified;
  }
  constexpr int stepLevel(uint8_t i) const { return axis[i].stepActive ? config.axis[i].stepActiveLevel : !config.axis[i].stepActiveLevel; }
  constexpr int dirLevel(uint8_t i) const { return axis[i].positive ? config.axis[i].positiveDirLevel : !config.axis[i].positiveDirLevel; }
  constexpr bool atNeutral() const { return axis[0].position == config.neutral[0] && axis[1].position == config.neutral[1]; }
  constexpr void halt(Reply reason) {
    armed = false;
    if (!stopping) result = reason;  // Preserve original reason while draining tails.
    if (moving) {
      referenced = false;
      stopping = true;
      state = State::Stopping;
      if (tailsDone()) finish();
    }
    ++revision;
  }
  // Local failure hook, not a claim of physical stall/force feedback.
  constexpr void failAxis(uint8_t i) {
    if (i > 1) return;
    faultAxis = static_cast<int8_t>(i);
    referenced = false;
    halt(Reply::AxisFault);
    if (!moving) state = State::Unreferenced;
  }
  constexpr void failTimer() {
    timerHealthy = false;
    referenced = false;
    halt(Reply::TimerFault);
  }
  constexpr Reply command(Command c, int32_t amount = 0, uint32_t nowMs = 0, uint8_t jogAxis = 0, int32_t other = 0) {
    expirePermissions(nowMs);
    const Reply reply = dispatch(c, amount, nowMs, jogAxis, other);
    // Valid rejected/invalid traffic cannot keep a guarded session alive.
    if (reply == Reply::Ok && guardActive) lastContactMs = nowMs;
    return reply;
  }
 private:
  constexpr Reply dispatch(Command c, int32_t amount, uint32_t nowMs, uint8_t jogAxis, int32_t other) {
    if (c == Command::Status) return Reply::Ok;
    if (c == Command::Stop || c == Command::Disarm) {
      guardRequired = guardActive = false;  // Deliberate exit restores legacy manual operation.
      halt(c == Command::Stop ? Reply::Stopped : Reply::Disarmed);
      return Reply::Ok;
    }
    if (c == Command::Invalid) return Reply::BadCommand;
    if (moving) return Reply::Busy;
    if (c == Command::ReferenceOpen) return Reply::ReferenceChanged;
    if (!configured()) return Reply::Locked;
    if (c == Command::SetStrokes) {
      if (armed) return Reply::MustDisarm;
      if (!amount || !other || magnitude(amount) > 2500 || magnitude(other) > 2500) return Reply::Limit;
      Config next = config;
      next.neutral[0] = amount; next.neutral[1] = 0;
      next.grasp[0] = 0; next.grasp[1] = other;
      next.profileVerified = false;  // An edited trajectory is not physically commissioned.
      if (!valid(next) || !fitsDeadline(larger(magnitude(amount), magnitude(other)))) return Reply::Limit;
      config = next;
      state = referenced ? (atNeutral() ? State::Neutral : State::Positioned) : State::Unreferenced;
      result = Reply::Ok;
      ++revision;
      return Reply::Ok;  // Preserve actual commanded coordinates; no reference, ARM or motion.
    }
    if (c == Command::Guard) {
      if (armed) return Reply::MustDisarm;
      guardRequired = guardActive = true;
      lastContactMs = nowMs;
      result = Reply::Ok;
      ++revision;
      return Reply::Ok;
    }
    if (c == Command::ReferenceNeutral || c == Command::ReferenceRest) {
      if (armed) return Reply::MustDisarm;
      const int32_t* declared = c == Command::ReferenceRest ? config.neutral : config.reference;
      for (uint8_t i = 0; i < 2; ++i) axis[i].position = declared[i];
      referenced = true;
      state = atNeutral() ? State::Neutral : State::Positioned;
      result = Reply::Ok;
      faultAxis = -1;
      ++revision;
      return Reply::Ok;
    }
    if (!referenced) return Reply::NoReference;
    if (guardRequired && !guardActive) return Reply::GuardTimeout;
    if (c == Command::Arm) {
      if (!armed) armedSinceMs = nowMs;  // Repeated ARM is idempotent, not an extension.
      armed = true;
      result = Reply::Ok;
      ++revision;
      return Reply::Ok;
    }
    if (!armed) return Reply::Disarmed;
    int32_t target[2] = {axis[0].position, axis[1].position};
    if (c == Command::Jog) {
      if (jogAxis > 1 || !amount || magnitude(amount) > uint32_t(config.axis[jogAxis].maxJogSteps)) return Reply::Limit;
      const int64_t destination = int64_t(target[jogAxis]) + amount;
      if (destination < config.axis[jogAxis].minPosition || destination > config.axis[jogAxis].maxPosition) return Reply::Limit;
      target[jogAxis] = static_cast<int32_t>(destination);
      return start(target, Job::Jog, nowMs);
    }
    if (c == Command::Grab) {
      if (!atNeutral() || state != State::Neutral) return Reply::NotNeutral;
      return start(config.grasp, Job::Grab, nowMs);
    }
    if (c == Command::Neutral) {
      if (atNeutral()) return Reply::AlreadyNeutral;
      return start(config.neutral, Job::Neutral, nowMs);
    }
    return Reply::BadCommand;
  }
 public:
  // Local timer ISR only. No serial, sensor, cloud work, waits or allocation.
  constexpr void tick(uint32_t nowMs) {
    expirePermissions(nowMs);  // Also expires idle ARM/session while loop() is busy.
    if (!moving) return;
    if (!stopping) {
      for (uint8_t i = 0; i < 2; ++i) {
        if (!valid(config.axis[i], config.unloadedTestAuthorized) || !within(config.axis[i], axis[i].position)) { failAxis(i); break; }
      }
      if (!stopping && uint32_t(nowMs - startedMs) >= config.jobTimeoutMs) halt(Reply::TimedOut);
    }
    if (!moving) return;
    for (uint8_t i = 0; i < 2; ++i) {
      if (activeLeft[i] && --activeLeft[i] == 0) axis[i].stepActive = false;
      if (tailLeft[i]) --tailLeft[i];
    }
    if (stopping || slotsLeft == 0) {
      if (tailsDone()) finish();
      return;
    }
    if (slotWait > 1) { --slotWait; return; }
    slotWait = periodTicks;
    --slotsLeft;
    // Integer common-progress interpolation; stationary axes emit no pulses.
    // Every moving axis emits its final edge in the same final slot.
    for (uint8_t i = 0; i < 2; ++i) {
      error[i] += counts[i];
      if (error[i] < totalSlots) continue;
      error[i] -= totalSlots;
      axis[i].stepActive = true;
      activeLeft[i] = ticks(config.axis[i].minPulseActiveUs);
      // Retain the full rate interval even on the final edge so a following
      // accepted job cannot violate the rate bound across the job boundary.
      tailLeft[i] = periodTicks;
      axis[i].position += axis[i].positive ? 1 : -1;
    }
  }
 private:
  bool guardRequired = false;  // Once opted in, expiry needs an explicit fresh GUARD.
  uint32_t lastContactMs = 0, armedSinceMs = 0;
  constexpr void expirePermissions(uint32_t nowMs) {
    if (guardActive && uint32_t(nowMs - lastContactMs) >= GUARD_TIMEOUT_MS) {
      guardActive = false;
      halt(Reply::GuardTimeout);  // Idle/hold reference survives; interrupted jobs do not.
    }
    if (armed && uint32_t(nowMs - armedSinceMs) >= ARM_TIMEOUT_MS) {
      armed = false;
      result = Reply::ArmExpired;
      ++revision;
    }
  }
  bool stopping = false;
  uint32_t counts[2] = {}, error[2] = {}, activeLeft[2] = {}, tailLeft[2] = {};
  uint32_t totalSlots = 0, slotsLeft = 0, slotWait = 0, periodTicks = 0, startedMs = 0;
  constexpr uint32_t period() const {
    uint32_t p = 1;
    for (uint8_t i = 0; i < 2; ++i) {
      const AxisConfig& c = config.axis[i];
      p = larger(p, (1000 + c.stepsPerSecond - 1) / c.stepsPerSecond);
      p = larger(p, ticks(c.minPulseActiveUs) + larger(ticks(c.minPulseIdleUs), ticks(c.dirHoldUs)));
    }
    return p;
  }
  constexpr uint32_t setupTicks() const {
    return larger(ticks(config.axis[0].dirSetupUs), ticks(config.axis[1].dirSetupUs)) + 1;
  }
  constexpr bool fitsDeadline(uint32_t count) const {
    return uint64_t(setupTicks()) + uint64_t(count) * period() < config.jobTimeoutMs;
  }
  constexpr bool profileReady() const {
    if (!valid(config)) return false;
    const uint32_t n = larger(magnitude(config.grasp[0] - config.neutral[0]), magnitude(config.grasp[1] - config.neutral[1]));
    const uint32_t preparation = larger(magnitude(config.neutral[0] - config.reference[0]),
                                       magnitude(config.neutral[1] - config.reference[1]));
    return fitsDeadline(n) && fitsDeadline(preparation);
  }
  constexpr bool tailsDone() const { return tailLeft[0] == 0 && tailLeft[1] == 0; }
  constexpr Reply start(const int32_t target[2], Job next, uint32_t nowMs) {
    uint32_t requested[2] = {};
    for (uint8_t i = 0; i < 2; ++i) {
      if (!within(config.axis[i], axis[i].position) || !within(config.axis[i], target[i])) return Reply::Limit;
      requested[i] = magnitude(target[i] - axis[i].position);
    }
    const uint32_t n = larger(requested[0], requested[1]);
    if (!n) return Reply::Limit;
    if (!fitsDeadline(n)) return Reply::Deadline;
    // Atomic acceptance: both axes and complete duration checked before commit.
    for (uint8_t i = 0; i < 2; ++i) {
      counts[i] = requested[i];
      error[i] = activeLeft[i] = tailLeft[i] = 0;
      if (counts[i]) axis[i].positive = target[i] > axis[i].position;
      axis[i].stepActive = false;
    }
    totalSlots = slotsLeft = n;
    periodTicks = period();
    slotWait = setupTicks();  // Includes a possibly partial first timer interval.
    startedMs = nowMs;
    stopping = false;
    moving = true;
    armed = false;  // One ARM authorizes the whole coordinated job, not one axis.
    job = next;
    state = next == Job::Grab ? State::Grabbing : next == Job::Neutral ? State::Returning : State::Jogging;
    result = Reply::Accepted;
    ++jobId;
    ++revision;
    return Reply::Ok;
  }
  constexpr void finish() {
    moving = false;
    for (uint8_t i = 0; i < 2; ++i) axis[i].stepActive = false;
    armed = false;
    if (stopping) state = State::Unreferenced;
    else {
      state = job == Job::Grab ? State::HoldUnconfirmed : atNeutral() ? State::Neutral : State::Positioned;
      result = Reply::Complete;
    }
    stopping = false;
    ++revision;
  }
};
}  // namespace grip
