// Compiler-evaluated production logic. Synthetic fixtures are NOT hardware settings.
#include "motor_logic.h"
#include "grip_config.h"
using namespace grip;
constexpr Config FIXTURE = {
    {{2, 3, true, 1, 1, -100, 100, 20, 100, 1500, 500, 1500, 1500},
     {4, 5, true, 0, 0, -100, 100, 20, 200, 3000, 1000, 3500, 4500}},
    true, {10, 20}, {17, 7}, 2000, false, {10, 20}};
constexpr Controller ready(Config cfg = FIXTURE, uint32_t start = 0) {
  Controller c(cfg);
  c.command(Command::ReferenceNeutral, 0, start);
  c.command(Command::Arm, 0, start);
  return c;
}
constexpr void run(Controller& c, uint32_t start = 0, uint32_t duration = 400) {
  for (uint32_t i = 1; i <= duration; ++i) c.tick(start + i);
}
constexpr bool locked(Config cfg) {
  Controller c(cfg);
  if (c.configured() || c.command(Command::Arm) != Reply::Locked ||
      c.command(Command::ReferenceNeutral) != Reply::Locked || c.command(Command::Grab) != Reply::Locked ||
      c.command(Command::Neutral) != Reply::Locked || c.command(Command::Jog, 1) != Reply::Locked) return false;
  run(c);
  return !c.moving && !c.armed && !c.referenced && !c.axis[0].stepActive && !c.axis[1].stepActive &&
         c.axis[0].position == 0 && c.axis[1].position == 0;
}
constexpr bool savedBenchSettings() {
  Config cfg = MOTOR_CONFIG;
  cfg.unloadedTestAuthorized = false;
  if (!locked(cfg)) return false;
  Controller c(MOTOR_CONFIG);
  if (!c.configured() || c.physicalReady() || c.referenced || c.moving) return false;
  if (c.command(Command::ReferenceNeutral) != Reply::Ok || c.moving || c.jobId ||
      c.axis[0].position != 0 || c.axis[1].position != 0 || c.state != State::Positioned) return false;
  c.command(Command::Arm);
  if (c.command(Command::Grab) != Reply::NotNeutral || c.moving || c.jobId) return false;
  if (c.command(Command::Neutral) != Reply::Ok || !c.axis[0].positive) return false;
  // Preparation pulls only A; it must not pull B or create a fake Rest reference.
  for (uint32_t now = 1; now <= 2000; ++now) {
    c.tick(now);
    if (c.axis[1].position != 0 || c.axis[1].stepActive) return false;
  }
  if (!c.atNeutral() || c.state != State::Neutral || c.axis[0].position != 900 || c.armed) return false;
  for (uint32_t start : {2000u, 6000u, 10000u}) {
    c.command(Command::Arm, 0, start);
    if (c.command(Command::Grab, 0, start) != Reply::Ok || c.axis[0].positive || !c.axis[1].positive) return false;
    for (uint32_t i = 1; i <= 2000; ++i) {
      c.tick(start + i);
      if (c.axis[0].position < 0 || c.axis[1].position < 0 ||
          c.axis[0].position + c.axis[1].position != 900) return false;
    }
    if (c.moving || c.axis[0].position != 0 || c.axis[1].position != 900 ||
        c.state != State::HoldUnconfirmed || c.armed) return false;
    c.command(Command::Arm, 0, start + 2000);
    if (c.command(Command::Neutral, 0, start + 2000) != Reply::Ok ||
        !c.axis[0].positive || c.axis[1].positive) return false;
    run(c, start + 2000, 2000);
    if (!c.atNeutral() || c.state != State::Neutral || !c.referenced || c.moving) return false;
  }
  return c.jobId == 7 && !c.armed;
}
static_assert(savedBenchSettings(), "Physical zero requires preparation; three paper Grab/Rest cycles preserve coordinates and directions");
constexpr Controller preparedBench() {
  Controller c(MOTOR_CONFIG);
  c.command(Command::ReferenceNeutral);
  c.command(Command::Arm);
  c.command(Command::Neutral);
  run(c, 0, 2000);
  return c;
}
constexpr bool preparationStop() {
  Controller c = ready(MOTOR_CONFIG);
  c.command(Command::Neutral);
  run(c, 0, 100);
  const int32_t a = c.axis[0].position;
  c.command(Command::Stop, 0, 100);
  run(c, 100, 20);
  if (c.moving || c.referenced || c.axis[0].position != a || c.axis[1].position != 0) return false;
  return c.command(Command::Arm, 0, 120) == Reply::NoReference;
}
static_assert(preparationStop(), "Interrupted preparation stops without pull on B and requires physical-zero recovery");
constexpr bool retractedReference() {
  Controller c(MOTOR_CONFIG);
  if (c.command(Command::ReferenceRest) != Reply::Ok || !c.referenced || !c.atNeutral() ||
      c.state != State::Neutral || c.moving || c.armed || c.jobId ||
      c.axis[0].position != 900 || c.axis[1].position != 0) return false;
  run(c, 0, 100);
  if (c.axis[0].stepActive || c.axis[1].stepActive || c.axis[0].position != 900) return false;
  c.command(Command::Arm, 0, 100);
  if (c.command(Command::ReferenceRest, 0, 100) != Reply::MustDisarm ||
      c.command(Command::ReferenceNeutral, 0, 100) != Reply::MustDisarm) return false;
  if (c.command(Command::Grab, 0, 100) != Reply::Ok ||
      c.command(Command::ReferenceRest, 0, 100) != Reply::Busy) return false;
  run(c, 100, 2000);
  if (c.axis[0].position != 0 || c.axis[1].position != 900 || !c.referenced) return false;
  c.command(Command::Arm, 0, 2100); c.command(Command::Neutral, 0, 2100);
  run(c, 2100, 2000);
  return c.atNeutral() && c.state == State::Neutral && c.jobId == 2;
}
static_assert(retractedReference(), "Already-retracted declaration emits no motion; Grab/return works without resetting physical zero");
constexpr bool editableStrokes() {
  Controller c(MOTOR_CONFIG);
  if (c.command(Command::SetStrokes, -900, 0, 0, 700) != Reply::Ok || c.moving || c.armed ||
      c.referenced || c.jobId || c.axis[0].position != 0 || c.axis[1].position != 0 || c.physicalReady()) return false;
  if (c.config.neutral[0] != -900 || c.config.grasp[1] != 700) return false;
  if (c.command(Command::SetStrokes, 0, 0, 0, 900) != Reply::Limit ||
      c.command(Command::SetStrokes, 2501, 0, 0, 900) != Reply::Limit || c.config.neutral[0] != -900) return false;
  c.command(Command::ReferenceRest);
  c.command(Command::Arm);
  if (c.command(Command::SetStrokes, 1000, 0, 0, 1000) != Reply::MustDisarm) return false;
  if (c.command(Command::Grab) != Reply::Ok || !c.axis[0].positive || !c.axis[1].positive ||
      c.command(Command::SetStrokes, 1000, 0, 0, 1000) != Reply::Busy) return false;
  run(c, 0, 2000);
  if (c.moving || c.axis[0].position != 0 || c.axis[1].position != 700) return false;
  c.command(Command::Arm, 0, 2000); c.command(Command::Neutral, 0, 2000);
  run(c, 2000, 2000);
  if (!c.atNeutral() || c.axis[0].position != -900 || c.axis[1].position != 0) return false;
  if (c.command(Command::SetStrokes, -1000, 4000, 0, -800) != Reply::Ok) return false;
  return c.referenced && c.state == State::Positioned && c.axis[0].position == -900 &&
      c.axis[1].position == 0 && !c.moving && !c.armed && c.jobId == 2;
}
static_assert(editableStrokes(), "Signed stroke edits are atomic, idle/disarmed, keep coordinates and never move; flipped Grab/return works");
static_assert(parse("SET_STROKES -900 +700").command == Command::SetStrokes &&
              parse("SET_STROKES -900 +700").amount == -900 && parse("SET_STROKES -900 +700").other == 700 &&
              parse("SET_STROKES 1").command == Command::Invalid &&
              parse("SET_STROKES 1 2 extra").command == Command::Invalid &&
              parse("SET_STROKES 99999999 2").command == Command::Invalid,
              "Signed stroke parser is exact, bounded and rejects malformed commands");
constexpr bool referenceBounds() {
  Config cfg = MOTOR_CONFIG; cfg.reference[0] = 2501;
  if (!locked(cfg)) return false;
  cfg = MOTOR_CONFIG; cfg.jobTimeoutMs = 1803;
  // Both the 900-pulse preparation and 900-pulse exchange must fit the deadline.
  return locked(cfg);
}
static_assert(referenceBounds(), "Reference coordinates and preparation deadline are validated");
constexpr bool customJogBounds() {
  Controller c = ready(MOTOR_CONFIG);
  if (c.command(Command::Jog, 2501) != Reply::Limit || c.command(Command::Jog, 2500) != Reply::Ok) return false;
  run(c, 0, 6000);
  if (c.moving || c.armed || c.axis[0].position != 2500 || c.axis[1].position != 0) return false;
  c.command(Command::Arm, 0, 6000);
  if (c.command(Command::Jog, 1, 6000) != Reply::Limit || !c.armed) return false;
  if (c.command(Command::Jog, -2500, 6000) != Reply::Ok) return false;
  run(c, 6000, 6000);
  if (c.moving || c.axis[0].position != 0) return false;
  c.command(Command::Arm, 0, 12000);
  if (c.command(Command::Jog, -2500, 12000, 1) != Reply::Ok) return false;
  run(c, 12000, 6000);
  return !c.moving && c.axis[1].position == -2500 && c.axis[0].position == 0 && c.referenced;
}
static_assert(customJogBounds(), "Custom single jogs retain per-job and cumulative position limits on both axes");
constexpr bool benchOverrideBounds() {
  Config cfg = MOTOR_CONFIG; cfg.axis[1].stepsPerSecond = 501; if (!locked(cfg)) return false;
  cfg = MOTOR_CONFIG; cfg.axis[0].minPosition = -2501; if (!locked(cfg)) return false;
  cfg = MOTOR_CONFIG; cfg.axis[1].maxPosition = 2501; if (!locked(cfg)) return false;
  cfg = MOTOR_CONFIG; cfg.jobTimeoutMs = 10001; if (!locked(cfg)) return false;
  cfg = MOTOR_CONFIG; cfg.axis[0].stepPin = 6; return locked(cfg);
}
static_assert(benchOverrideBounds(), "Unloaded override preserves its travel/rate/deadline and reserved-pin bounds");
constexpr bool missingSettings() {
  for (uint8_t i = 0; i < 2; ++i) {
    Config cfg = FIXTURE; cfg.axis[i].interfaceVerified = false; if (!locked(cfg)) return false;
    cfg = FIXTURE; cfg.axis[i].stepPin = -1; if (!locked(cfg)) return false;
    cfg = FIXTURE; cfg.axis[i].dirPin = -1; if (!locked(cfg)) return false;
    cfg = FIXTURE; cfg.axis[i].stepActiveLevel = -1; if (!locked(cfg)) return false;
    cfg = FIXTURE; cfg.axis[i].positiveDirLevel = -1; if (!locked(cfg)) return false;
    cfg = FIXTURE; cfg.axis[i].stepsPerSecond = 0; if (!locked(cfg)) return false;
    cfg = FIXTURE; cfg.axis[i].minPulseActiveUs = 0; if (!locked(cfg)) return false;
    cfg = FIXTURE; cfg.axis[i].minPulseIdleUs = 0; if (!locked(cfg)) return false;
    cfg = FIXTURE; cfg.axis[i].dirSetupUs = 0; if (!locked(cfg)) return false;
    cfg = FIXTURE; cfg.axis[i].dirHoldUs = 0; if (!locked(cfg)) return false;
    cfg = FIXTURE; cfg.axis[i].minPosition = cfg.axis[i].maxPosition; if (!locked(cfg)) return false;
    cfg = FIXTURE; cfg.axis[i].maxJogSteps = 0; if (!locked(cfg)) return false;
  }
  Config cfg = FIXTURE; cfg.profileVerified = false; if (!locked(cfg)) return false;
  cfg = FIXTURE; cfg.jobTimeoutMs = 0; return locked(cfg);
}
static_assert(missingSettings(), "Either incomplete axis or profile locks the whole mechanism");
constexpr bool invalidSettings() {
  for (uint8_t i = 0; i < 2; ++i) {
    Config cfg = FIXTURE; cfg.axis[i].stepsPerSecond = 501; if (!locked(cfg)) return false;
    cfg = FIXTURE; cfg.axis[i].minPosition = -1000001; if (!locked(cfg)) return false;
    cfg = FIXTURE; cfg.axis[i].maxPosition = 1000001; if (!locked(cfg)) return false;
    cfg = FIXTURE; cfg.axis[i].maxJogSteps = 201; if (!locked(cfg)) return false;
    cfg = FIXTURE; cfg.axis[i].minPulseActiveUs = 1000001; if (!locked(cfg)) return false;
    cfg = FIXTURE; cfg.axis[i].minPulseIdleUs = 1000001; if (!locked(cfg)) return false;
    cfg = FIXTURE; cfg.axis[i].dirSetupUs = 1000001; if (!locked(cfg)) return false;
    cfg = FIXTURE; cfg.axis[i].dirHoldUs = 1000001; if (!locked(cfg)) return false;
    cfg = FIXTURE; cfg.grasp[i] = 101; if (!locked(cfg)) return false;
    cfg = FIXTURE; cfg.neutral[i] = -101; if (!locked(cfg)) return false;
  }
  Config cfg = FIXTURE; cfg.jobTimeoutMs = 60001; if (!locked(cfg)) return false;
  cfg = FIXTURE; cfg.grasp[0] = 10; cfg.grasp[1] = 20; return locked(cfg);
}
static_assert(invalidSettings(), "Rate, travel, timing and full-profile bounds apply to both axes");
constexpr bool pinReservations() {
  for (int8_t pin = 0; pin <= 20; ++pin) {
    if (pin == 2 || pin == 3 || pin == 4 || pin == 5 || pin == 8 || (pin >= 14 && pin <= 19)) continue;
    Config cfg = FIXTURE; cfg.axis[1].stepPin = pin; if (!locked(cfg)) return false;
  }
  Config cfg = FIXTURE; cfg.axis[1].stepPin = 2; if (!locked(cfg)) return false;
  cfg = FIXTURE; cfg.axis[1].dirPin = 3; if (!locked(cfg)) return false;
  cfg = FIXTURE; cfg.axis[1].stepPin = 3; if (!locked(cfg)) return false;
  cfg = FIXTURE; cfg.axis[1].dirPin = 2; if (!locked(cfg)) return false;
  cfg = FIXTURE; cfg.axis[1].dirPin = 4; return locked(cfg);
}
static_assert(pinReservations(), "Unknown, sensor, UART and conflicting motor pins are rejected");
constexpr bool referenceAndArm() {
  Controller c(FIXTURE);
  if (c.command(Command::Arm) != Reply::NoReference || c.command(Command::Grab) != Reply::NoReference) return false;
  if (c.command(Command::ReferenceOpen) != Reply::ReferenceChanged || c.referenced) return false;
  if (c.command(Command::ReferenceNeutral) != Reply::Ok || !c.atNeutral() || c.axis[0].position != 10) return false;
  if (c.command(Command::Grab) != Reply::Disarmed) return false;
  c.command(Command::Arm);
  return c.command(Command::ReferenceNeutral) == Reply::MustDisarm;
}
static_assert(referenceAndArm(), "Curled Rest is explicitly referenced; legacy OPEN is never reinterpreted");
constexpr bool coordinated(int32_t a, int32_t b) {
  Config cfg = FIXTURE; cfg.grasp[0] = cfg.neutral[0] + a; cfg.grasp[1] = cfg.neutral[1] + b;
  Controller c = ready(cfg);
  if (c.command(Command::Grab) != Reply::Ok || c.armed || c.result != Reply::Accepted) return false;
  uint32_t pulses[2] = {}, finalEdge[2] = {};
  bool previous[2] = {};
  const uint32_t total = larger(magnitude(a), magnitude(b));
  for (uint32_t now = 1; now <= 400; ++now) {
    c.tick(now);
    for (uint8_t i = 0; i < 2; ++i) {
      if (c.axis[i].stepActive && !previous[i]) { ++pulses[i]; finalEdge[i] = now; }
      previous[i] = c.axis[i].stepActive;
    }
    const uint32_t progressed = magnitude(c.axis[magnitude(a) >= magnitude(b) ? 0 : 1].position - cfg.neutral[magnitude(a) >= magnitude(b) ? 0 : 1]);
    if (magnitude(c.axis[0].position - cfg.neutral[0]) != progressed * magnitude(a) / total ||
        magnitude(c.axis[1].position - cfg.neutral[1]) != progressed * magnitude(b) / total) return false;
  }
  return pulses[0] == magnitude(a) && pulses[1] == magnitude(b) &&
      (!a || !b || finalEdge[0] == finalEdge[1]) && !c.moving && !c.armed && c.referenced &&
      !c.axis[0].stepActive && !c.axis[1].stepActive && c.state == State::HoldUnconfirmed &&
      c.axis[0].position == cfg.grasp[0] && c.axis[1].position == cfg.grasp[1] && c.result == Reply::Complete;
}
static_assert(coordinated(7, -13), "Unequal opposite motion has exact counts, shared progress and final edge");
static_assert(coordinated(-13, 7), "Either axis can lead with independently signed direction");
static_assert(coordinated(13, 13), "Equal same-direction motion completes both axes");
static_assert(coordinated(0, -13), "Stationary A emits zero pulses");
static_assert(coordinated(7, 0), "Stationary B emits zero pulses");
constexpr bool roundTrip() {
  Controller c = ready(); c.command(Command::Grab); run(c);
  if (c.command(Command::Neutral) != Reply::Disarmed || c.state != State::HoldUnconfirmed) return false;
  run(c, 400);
  if (c.state != State::HoldUnconfirmed || c.axis[1].position != 7) return false;
  c.command(Command::Arm);
  if (c.command(Command::Grab) != Reply::NotNeutral || c.command(Command::Neutral, 0, 800) != Reply::Ok) return false;
  if (c.axis[0].positive || !c.axis[1].positive) return false;
  run(c, 800);
  return !c.moving && !c.armed && c.referenced && c.atNeutral() && c.state == State::Neutral && c.jobId == 2;
}
static_assert(roundTrip(), "One ARM per complete job; hold persists until explicit armed neutral return");
constexpr bool busyAndRepeated() {
  Controller c = ready();
  if (c.command(Command::Neutral) != Reply::AlreadyNeutral || !c.armed) return false;
  c.command(Command::Grab);
  const uint32_t revision = c.revision;
  for (Command cmd : {Command::Grab, Command::Neutral, Command::Jog, Command::Arm, Command::ReferenceNeutral})
    if (c.command(cmd, 1) != Reply::Busy || c.revision != revision || c.jobId != 1) return false;
  if (c.command(Command::Status) != Reply::Ok) return false;
  run(c);
  return c.command(Command::Grab) == Reply::Disarmed && c.command(Command::Neutral) == Reply::Disarmed && c.jobId == 1;
}
static_assert(busyAndRepeated(), "Busy/repeated commands do not queue, restart or partially rearm");
constexpr bool completionTails() {
  Controller c = ready(); c.command(Command::Grab);
  for (uint32_t now = 1; now <= 125; ++now) c.tick(now);
  if (c.axis[0].position != 17 || c.axis[1].position != 7 || !c.moving) return false;
  for (uint32_t now = 126; now <= 134; ++now) {
    c.tick(now);
    if (!c.moving || c.state != State::Grabbing) return false;
  }
  c.tick(135);
  return !c.moving && c.result == Reply::Complete;
}
static_assert(completionTails(), "Completion waits for the slower axis's final pulse/idle/hold tail");
constexpr bool pulseTiming() {
  Controller c = ready(); c.command(Command::Grab);
  bool prev[2] = {};
  uint32_t lastRise[2] = {}, lastFall[2] = {};
  for (uint32_t now = 1; now <= 150; ++now) {
    c.tick(now);
    for (uint8_t i = 0; i < 2; ++i) {
      const AxisConfig ac = FIXTURE.axis[i];
      if (c.axis[i].stepActive && !prev[i]) {
        if (now < ticks(ac.dirSetupUs) + 1) return false;
        if (lastRise[i] && now - lastRise[i] < (1000 + ac.stepsPerSecond - 1) / ac.stepsPerSecond) return false;
        if (lastFall[i] && now - lastFall[i] < ticks(ac.minPulseIdleUs)) return false;
        lastRise[i] = now;
      }
      if (!c.axis[i].stepActive && prev[i]) {
        if (now - lastRise[i] < ticks(ac.minPulseActiveUs)) return false;
        lastFall[i] = now;
      }
      prev[i] = c.axis[i].stepActive;
    }
  }
  return lastRise[0] == 125 && lastRise[1] == 125 && lastFall[0] == 127 && lastFall[1] == 128;
}
static_assert(pulseTiming(), "Independent active/idle/setup/rate requirements survive common scheduling");
constexpr bool stopped(Command cmd, int8_t fault = -1) {
  Controller c = ready(); c.command(Command::Grab);
  for (uint32_t now = 1; now <= 15; ++now) c.tick(now);
  if (!c.axis[0].stepActive || !c.axis[1].stepActive) return false;
  const int32_t a = c.axis[0].position, b = c.axis[1].position;
  if (fault >= 0) c.failAxis(fault); else c.command(cmd);
  if (c.referenced || c.armed || !c.axis[0].stepActive || !c.axis[1].stepActive) return false;
  if (c.command(Command::ReferenceNeutral) != Reply::Busy) return false;
  c.tick(16); if (!c.axis[0].stepActive || !c.axis[1].stepActive) return false;
  c.tick(17); if (c.axis[0].stepActive || !c.axis[1].stepActive) return false;
  c.tick(18); if (c.axis[1].stepActive || !c.moving) return false;
  for (uint32_t now = 19; now <= 24; ++now) { c.tick(now); if (!c.moving) return false; }
  c.tick(25); run(c, 25);
  return !c.moving && !c.referenced && c.state == State::Unreferenced &&
      c.axis[0].position == a && c.axis[1].position == b && c.command(Command::Arm) == Reply::NoReference &&
      c.result == (fault >= 0 ? Reply::AxisFault : cmd == Command::Stop ? Reply::Stopped : Reply::Disarmed);
}
static_assert(stopped(Command::Stop), "STOP drains both active pulses and hold intervals without more edges");
static_assert(stopped(Command::Disarm), "DISARM interrupts both axes and invalidates the reference");
static_assert(stopped(Command::Stop, 0), "A failure stops both axes");
static_assert(stopped(Command::Stop, 1), "B failure stops both axes");
constexpr bool stopDuringSetup() {
  Controller c = ready(); c.command(Command::Grab); c.command(Command::Stop); run(c);
  return !c.moving && !c.referenced && c.axis[0].position == 10 && c.axis[1].position == 20;
}
static_assert(stopDuringSetup(), "STOP before first edge prevents both axes from starting");
constexpr bool timeout(uint32_t start) {
  Controller c = ready(FIXTURE, start); c.command(Command::Grab, 0, start);
  for (uint32_t i = 1; i <= 15; ++i) c.tick(start + i);
  c.tick(start + FIXTURE.jobTimeoutMs);
  for (uint32_t i = 1; i <= 20; ++i) c.tick(start + FIXTURE.jobTimeoutMs + i);
  return !c.moving && !c.referenced && c.axis[0].position == 11 && c.axis[1].position == 18 && c.result == Reply::TimedOut;
}
static_assert(timeout(0), "Local timeout stops both axes while respecting pulse tails");
static_assert(timeout(0xFFFFFFF0u), "Timeout arithmetic survives millis rollover");
constexpr bool rolloverComplete() {
  const uint32_t start = 0xFFFFFFF0u; Controller c = ready(FIXTURE, start);
  c.command(Command::Grab, 0, start); run(c, start);
  return c.state == State::HoldUnconfirmed && c.result == Reply::Complete && c.referenced;
}
static_assert(rolloverComplete(), "Normal coordinated completion survives rollover");
constexpr bool atomicRejection() {
  Controller c = ready(); c.config.grasp[1] = -101;
  if (c.command(Command::Grab) != Reply::Locked || !c.armed || c.moving || c.jobId ||
      c.axis[0].position != 10 || c.axis[1].position != 20 || !c.axis[1].positive) return false;
  c.config = FIXTURE; c.config.jobTimeoutMs = 140;
  if (!c.configured() || c.command(Command::Jog, 20) != Reply::Deadline || !c.armed || c.moving || c.jobId) return false;
  c.config.jobTimeoutMs = 130;
  return c.command(Command::Grab) == Reply::Locked && !c.moving && c.armed;
}
static_assert(atomicRejection(), "Bad second target or complete-job deadline rejects before any axis changes");
constexpr bool jogBounds() {
  Controller c = ready();
  if (c.command(Command::Jog, 0) != Reply::Limit || c.command(Command::Jog, 21) != Reply::Limit ||
      c.command(Command::Jog, 1, 0, 2) != Reply::Limit) return false;
  c.axis[0].position = 95; if (c.command(Command::Jog, 6) != Reply::Limit) return false;
  c.axis[0].position = -95; if (c.command(Command::Jog, -6) != Reply::Limit) return false;
  c.axis[0].position = 10;
  if (c.command(Command::Jog, -5, 0, 1) != Reply::Ok) return false;
  run(c);
  if (c.axis[0].position != 10 || c.axis[1].position != 15 || c.state != State::Positioned) return false;
  c.command(Command::Arm);
  if (c.command(Command::Grab) != Reply::NotNeutral || c.command(Command::Neutral, 0, 400) != Reply::Ok) return false;
  run(c, 400);
  return c.atNeutral() && c.state == State::Neutral;
}
static_assert(jogBounds(), "Single-axis commissioning is finite, signed, bounded and can return to neutral");
constexpr bool polarity() {
  Controller c = ready();
  if (c.stepLevel(0) != 0 || c.stepLevel(1) != 1) return false;
  c.command(Command::Grab);
  for (uint32_t i = 1; i <= 15; ++i) c.tick(i);
  return c.stepLevel(0) == 1 && c.stepLevel(1) == 0 && c.dirLevel(0) == 1 && c.dirLevel(1) == 1;
}
static_assert(polarity(), "Independent STEP and signed DIR polarity are explicit");
constexpr bool rateAcrossJobs() {
  Controller c = ready();
  if (c.command(Command::Jog, 1) != Reply::Ok) return false;
  for (uint32_t now = 1; now <= 14; ++now) {
    c.tick(now);
    if (!c.moving) return false;
  }
  c.tick(15);
  if (c.moving) return false;
  c.command(Command::Arm);
  if (c.command(Command::Jog, -1, 15) != Reply::Ok) return false;
  for (uint32_t now = 16; now <= 19; ++now) {
    c.tick(now);
    if (c.axis[0].stepActive) return false;
  }
  c.tick(20);
  return c.axis[0].stepActive && c.axis[0].position == 10;
}
static_assert(rateAcrossJobs(), "A stationary partner cannot bypass the final rate interval between jobs");
constexpr bool runtimeFault() {
  for (uint8_t i = 0; i < 2; ++i) {
    Controller c = ready(); c.command(Command::Grab);
    for (uint32_t now = 1; now <= 15; ++now) c.tick(now);
    c.config.axis[i].interfaceVerified = false; run(c, 15);
    if (c.moving || c.referenced || c.result != Reply::AxisFault || c.faultAxis != i ||
        c.axis[0].position != 11 || c.axis[1].position != 18) return false;
  }
  Controller c = ready(); c.failTimer();
  return !c.configured() && !c.referenced && c.command(Command::Arm) == Reply::Locked;
}
static_assert(runtimeFault(), "Invalid local axis state and timer-start failure lock out work");
constexpr Controller guarded(Config cfg = FIXTURE, uint32_t start = 0) {
  Controller c(cfg);
  c.command(Command::Guard, 0, start);
  c.command(Command::ReferenceNeutral, 0, start);
  c.command(Command::Arm, 0, start);
  return c;
}
constexpr bool physicalReadiness() {
  Controller bench(MOTOR_CONFIG);
  if (!bench.configured() || bench.physicalReady()) return false;
  Controller measured(FIXTURE);
  if (!measured.physicalReady()) return false;
  measured.config.axis[0].interfaceVerified = false;
  if (measured.physicalReady()) return false;
  measured.config.axis[0].interfaceVerified = true;
  measured.config.axis[1].interfaceVerified = false;
  if (measured.physicalReady()) return false;
  measured.config.axis[1].interfaceVerified = true;
  measured.config.profileVerified = false;
  return !measured.physicalReady();
}
static_assert(physicalReadiness(), "The unloaded override never advertises physical readiness");
constexpr bool guardAdmission() {
  Controller c(FIXTURE);
  if (c.guardActive || c.command(Command::Guard, 0, 0) != Reply::Ok || !c.guardActive || c.referenced) return false;
  c.command(Command::ReferenceNeutral);
  c.command(Command::Arm);
  if (c.command(Command::Guard, 0, 100) != Reply::MustDisarm) return false;
  c.command(Command::Grab);
  return c.command(Command::Guard, 0, 100) == Reply::Busy;
}
static_assert(guardAdmission(), "GUARD is explicit, idle/disarmed and does not reference or move");
constexpr bool guardExpires(uint32_t start) {
  Controller c = guarded(FIXTURE, start);
  c.tick(start + GUARD_TIMEOUT_MS - 1);
  if (!c.guardActive || !c.armed || !c.referenced) return false;
  c.tick(start + GUARD_TIMEOUT_MS);
  if (c.guardActive || c.armed || !c.referenced || c.state != State::Neutral || c.result != Reply::GuardTimeout) return false;
  c.command(Command::Status, 0, start + GUARD_TIMEOUT_MS + 1);
  if (c.guardActive || c.command(Command::Arm, 0, start + GUARD_TIMEOUT_MS + 1) != Reply::GuardTimeout) return false;
  if (c.command(Command::Guard, 0, start + GUARD_TIMEOUT_MS + 2) != Reply::Ok) return false;
  return c.guardActive && c.command(Command::Arm, 0, start + GUARD_TIMEOUT_MS + 2) == Reply::Ok;
}
static_assert(guardExpires(0), "Session expiry disarms idle state and requires explicit GUARD, not STATUS");
static_assert(guardExpires(0xFFFFFFF0u), "Guard deadlines survive millis rollover");
constexpr bool guardMovingExpiry() {
  Controller c = preparedBench();
  c.command(Command::Guard, 0, 2000); c.command(Command::Arm, 0, 2000);
  c.command(Command::Grab, 0, 2000);
  run(c, 2000, GUARD_TIMEOUT_MS + 10);
  if (c.guardActive || c.moving || c.referenced || c.armed || c.state != State::Unreferenced ||
      c.result != Reply::GuardTimeout || c.axis[0].position <= 0 || c.axis[0].position >= 900 ||
      c.axis[1].position <= 0 || c.axis[1].position >= 900 || c.axis[0].stepActive || c.axis[1].stepActive) return false;
  const int32_t a = c.axis[0].position, b = c.axis[1].position;
  run(c, 2000 + GUARD_TIMEOUT_MS + 10, 100);
  return c.axis[0].position == a && c.axis[1].position == b;
}
static_assert(guardMovingExpiry(), "Guard loss stops both moving axes with pulse tails and no return/replay");
constexpr bool guardedFullStroke() {
  Controller c = preparedBench();
  c.command(Command::Guard, 0, 2000); c.command(Command::Arm, 0, 2000);
  c.command(Command::Grab, 0, 2000);
  for (uint32_t now = 2001; now <= 5000; ++now) {
    if (now % 300 == 0 && c.command(Command::Status, 0, now) != Reply::Ok) return false;
    c.tick(now);
  }
  return c.guardActive && !c.moving && c.referenced && !c.armed &&
      c.state == State::HoldUnconfirmed && c.result == Reply::Complete &&
      c.axis[0].position == 0 && c.axis[1].position == 900;
}
static_assert(guardedFullStroke(), "Existing STATUS cadence renews the lease throughout the saved finite job");
constexpr bool guardHoldExpiry() {
  Controller c = guarded();
  c.command(Command::Grab);
  run(c);
  if (c.state != State::HoldUnconfirmed || c.result != Reply::Complete) return false;
  c.tick(GUARD_TIMEOUT_MS);
  if (c.guardActive || c.moving || c.armed || !c.referenced || c.state != State::HoldUnconfirmed ||
      c.axis[0].position != 17 || c.axis[1].position != 7 || c.result != Reply::GuardTimeout) return false;
  c.command(Command::Guard, 0, GUARD_TIMEOUT_MS + 1);
  c.command(Command::Arm, 0, GUARD_TIMEOUT_MS + 1);
  if (c.command(Command::Neutral, 0, GUARD_TIMEOUT_MS + 1) != Reply::Ok) return false;
  run(c, GUARD_TIMEOUT_MS + 1);
  return c.atNeutral() && c.state == State::Neutral;
}
static_assert(guardHoldExpiry(), "Guard loss preserves completed hold/reference without unwind; deliberate return remains possible");
constexpr bool rejectedTrafficDoesNotRenew() {
  Controller c = guarded();
  if (c.command(Command::Invalid, 0, 1000) != Reply::BadCommand ||
      c.command(Command::Jog, 21, 1100) != Reply::Limit ||
      c.command(Command::Guard, 0, 1200) != Reply::MustDisarm) return false;
  c.tick(GUARD_TIMEOUT_MS);
  return !c.guardActive && !c.armed && c.result == Reply::GuardTimeout;
}
static_assert(rejectedTrafficDoesNotRenew(), "Rejected/invalid commands cannot refresh a communication lease");
constexpr bool armExpires(uint32_t start) {
  Controller c(FIXTURE);
  c.command(Command::ReferenceNeutral, 0, start);
  c.command(Command::Arm, 0, start);
  c.command(Command::Arm, 0, start + 5000);  // Must not extend the existing permission.
  c.tick(start + ARM_TIMEOUT_MS - 1);
  if (!c.armed) return false;
  c.tick(start + ARM_TIMEOUT_MS);
  return !c.armed && c.referenced && c.state == State::Neutral && c.result == Reply::ArmExpired &&
      c.command(Command::Jog, 1, start + ARM_TIMEOUT_MS) == Reply::Disarmed;
}
static_assert(armExpires(0), "ARM expires locally without moving or invalidating an idle reference");
static_assert(armExpires(0xFFFFFFF0u), "ARM expiration survives millis rollover");
constexpr bool pollingDoesNotExtendArm() {
  Controller c = guarded();
  for (uint32_t now = 1; now <= ARM_TIMEOUT_MS; ++now) {
    if (now % 300 == 0) c.command(Command::Status, 0, now);
    c.tick(now);
  }
  return c.guardActive && !c.armed && c.referenced && c.result == Reply::ArmExpired;
}
static_assert(pollingDoesNotExtendArm(), "STATUS renews communication but never keeps stale ARM permission alive");
constexpr bool commandChecksArmDeadline() {
  Controller c = ready();
  return c.command(Command::Jog, 1, ARM_TIMEOUT_MS) == Reply::Disarmed &&
      !c.moving && !c.armed && c.result == Reply::ArmExpired;
}
static_assert(commandChecksArmDeadline(), "A late movement command checks ARM expiration before accepting a job");
constexpr bool deliberateGuardExit(Command exit) {
  Controller c = guarded();
  c.tick(GUARD_TIMEOUT_MS);
  if (c.command(exit, 0, GUARD_TIMEOUT_MS + 1) != Reply::Ok || c.guardActive || c.armed || !c.referenced) return false;
  if (c.command(Command::Arm, 0, GUARD_TIMEOUT_MS + 2) != Reply::Ok) return false;
  if (c.command(Command::Jog, 1, GUARD_TIMEOUT_MS + 2) != Reply::Ok) return false;
  run(c, GUARD_TIMEOUT_MS + 2);
  return !c.moving && c.result == Reply::Complete;
}
static_assert(deliberateGuardExit(Command::Stop), "Explicit STOP exits guarded mode for deliberate legacy manual recovery");
static_assert(deliberateGuardExit(Command::Disarm), "Explicit DISARM exits guarded mode without rearming or movement");
constexpr bool exitWhileHolding() {
  Controller c = guarded(); c.command(Command::Grab); run(c);
  if (c.command(Command::Stop, 0, 400) != Reply::Ok || c.guardActive || c.armed || !c.referenced ||
      c.state != State::HoldUnconfirmed || c.axis[0].position != 17 || c.axis[1].position != 7) return false;
  c.command(Command::Arm, 0, 500);
  if (c.command(Command::Neutral, 0, 500) != Reply::Ok) return false;
  run(c, 500);
  return c.atNeutral() && c.state == State::Neutral;
}
static_assert(exitWhileHolding(), "Deliberate guarded STOP never unwinds hold; manual return remains explicit");
constexpr bool exitDuringMotion() {
  Controller c = guarded(); c.command(Command::Grab);
  for (uint32_t now = 1; now <= 15; ++now) c.tick(now);
  c.command(Command::Stop, 0, 15);
  run(c, 15);
  return !c.guardActive && !c.moving && !c.armed && !c.referenced && c.state == State::Unreferenced &&
      c.command(Command::Arm, 0, 500) == Reply::NoReference;
}
static_assert(exitDuringMotion(), "Guard exit during motion still requires physical reference recovery");
static_assert(parse("GUARD").command == Command::Guard && parse("GUARD 1").command == Command::Invalid,
              "GUARD is an exact additive command without hidden parameters");
static_assert(parse("GRAB").command == Command::Grab && parse("CAPTURE").command == Command::Grab &&
              parse("NEUTRAL").command == Command::Neutral && parse("RELEASE").command == Command::Neutral, "Explicit high-level aliases");
static_assert(parse("REFERENCE_OPEN").command == Command::ReferenceOpen &&
              parse("REFERENCE_NEUTRAL").command == Command::ReferenceNeutral &&
              parse("REFERENCE_REST").command == Command::ReferenceRest, "Reference contracts stay distinct");
static_assert(parse("JOG -20").axis == 0 && parse("JOG -20").amount == -20 &&
              parse("JOG B -20").axis == 1 && parse("JOG A 1").command == Command::Jog, "Signed axis jog parser");
constexpr bool malformed() {
  for (const char* text : {"", "J", "JO", "JOG", "JOG ", "JOG B", "JOG B ", "JOG C 1", "JOG 2.0",
      "JOG 2147483648", "JOG -2147483648", "JOG +1", "JOG 1 ARM", "CAPTURE ", "GRAB 1"})
    if (parse(text).command != Command::Invalid) return false;
  return true;
}
static_assert(malformed(), "Malformed, overflowing and appended commands are rejected");
void setup() {}
void loop() {}
