"""Exact grip_control comment protocol and a single-session command sequencer.

No device access here. SensorReader alone owns the USB connection.
"""

from collections import deque
from dataclasses import dataclass, replace
import re
from threading import RLock
import time


FRESH_SECONDS = 1.0
REPLY_SECONDS = 1.0
POLL_SECONDS = 0.3
CONNECT_REPLY_SECONDS = 3.0  # Allow the board's serial-open startup, without retrying commands.
MOVING = frozenset(("JOGGING", "GRABBING", "RETURNING", "STOPPING"))
STATES = MOVING | {"UNREFERENCED", "NEUTRAL", "POSITIONED", "HOLD_UNCONFIRMED"}
RESULTS = frozenset((
    "OK", "ACCEPTED", "COMPLETE", "BAD_COMMAND", "CONFIG_LOCKED", "BUSY",
    "DISARMED", "DISARM_FIRST", "NO_REFERENCE", "TRAVEL_LIMIT", "NOT_NEUTRAL",
    "ALREADY_NEUTRAL", "USE_REFERENCE_NEUTRAL", "MOVE_TOO_LONG", "STOPPED",
    "MOVE_TIMEOUT", "AXIS_FAULT", "TIMER_FAULT",
    "GUARD_TIMEOUT", "ARM_EXPIRED",
))


@dataclass(frozen=True)
class MotorReply:
    kind: str
    result: str
    job_id: int | None


@dataclass(frozen=True)
class MotorStatus:
    state: str
    configured: bool
    armed: bool
    referenced: bool
    position_a: int
    position_b: int
    result: str
    job_id: int
    job: int
    fault_axis: int
    received_at: float

    @property
    def moving(self):
        return self.state in MOVING


@dataclass(frozen=True)
class MotorLimits:
    min_a: int
    max_a: int
    jog_a: int
    min_b: int
    max_b: int
    jog_b: int


@dataclass(frozen=True)
class MotorProfile:
    reference_a: int
    reference_b: int
    rest_a: int
    rest_b: int
    grasp_a: int
    grasp_b: int

    @property
    def manual_valid(self):
        return (self.reference_a == self.reference_b == self.rest_b == self.grasp_a == 0
                and 1 <= abs(self.rest_a) <= 2500 and 1 <= abs(self.grasp_b) <= 2500)


@dataclass(frozen=True)
class MotorControls:
    editable: bool = True


# User-confirmed A outer / B inner; user-selected 900-pulse pulling strokes.
PAPER_PROFILE = MotorProfile(0, 0, 900, 0, 0, 900)


@dataclass(frozen=True)
class MotorSafety:
    physical_ready: bool
    guard_active: bool
    guard_timeout_ms: int
    arm_timeout_ms: int
    received_at: float


def parse_motor_line(line, received_at):
    """ACK OK job; ERR reason [job]; or the ten-field state, as emitted."""
    fields = line.strip().split()
    if fields[:2] != ["#", "motor"]:
        return None
    payload = fields[2:]
    if payload == ["CONTROLS", "1"]:
        return MotorControls()
    if len(payload) == 7 and payload[0] == "PROFILE":
        if any(not re.fullmatch(r"-?[0-9]{1,10}", value) for value in payload[1:]):
            return None
        values = tuple(map(int, payload[1:]))
        if any(abs(value) > 1000000 for value in values) or values[2:4] == values[4:6]:
            return None
        return MotorProfile(*values)
    if len(payload) == 5 and payload[0] == "SAFETY":
        if (any(value not in ("0", "1") for value in payload[1:3])
                or any(not re.fullmatch(r"[0-9]{1,5}", value) for value in payload[3:])):
            return None
        guard_timeout, arm_timeout = map(int, payload[3:])
        if not (0 < guard_timeout <= 60000 and 0 < arm_timeout <= 60000):
            return None
        return MotorSafety(payload[1] == "1", payload[2] == "1",
                           guard_timeout, arm_timeout, received_at)
    if len(payload) == 7 and payload[0] == "LIMITS":
        if any(not re.fullmatch(r"-?[0-9]{1,10}", value) for value in payload[1:]):
            return None
        values = tuple(map(int, payload[1:]))
        if any(not (-1000000 <= low < high <= 1000000 and 0 < jog <= high - low)
               for low, high, jog in (values[:3], values[3:])):
            return None
        return MotorLimits(*values)
    if len(payload) in (2, 3) and payload[0] in ("ACK", "ERR"):
        kind, result = payload[:2]
        if kind == "ACK" and (result != "OK" or len(payload) != 3):
            return None
        if kind == "ERR" and (result not in (RESULTS - {"OK"}) | {"BAD_LINE"}
                              or (result == "BAD_LINE" and len(payload) != 2)):
            return None
        # BAD_LINE and startup faults omit the job ID; all other replies carry it.
        if len(payload) == 2 and result not in {"BAD_LINE", "CONFIG_LOCKED", "TIMER_FAULT"}:
            return None
        job = payload[2] if len(payload) == 3 else None
        if job is not None and (not re.fullmatch(r"[0-9]+", job) or int(job) > 0xFFFFFFFF):
            return None
        return MotorReply(kind, result, int(job) if job is not None else None)
    if len(payload) != 10 or payload[0] not in STATES or payload[6] not in RESULTS:
        return None
    if any(value not in ("0", "1") for value in payload[1:4]):
        return None
    if any(not re.fullmatch(r"-?[0-9]+", payload[i]) for i in (4, 5, 7, 8, 9)):
        return None
    a, b, job_id, job, fault = (int(payload[i]) for i in (4, 5, 7, 8, 9))
    if not (-0x80000000 <= a <= 0x7FFFFFFF and -0x80000000 <= b <= 0x7FFFFFFF
            and 0 <= job_id <= 0xFFFFFFFF and 0 <= job <= 3 and -1 <= fault <= 1):
        return None
    return MotorStatus(payload[0], *(value == "1" for value in payload[1:4]),
                       a, b, payload[6], job_id, job, fault, received_at)


@dataclass(frozen=True)
class MotorSnapshot:
    connected: bool = False
    status: MotorStatus | None = None
    pending: str | None = None
    message: str = "Controller unavailable"
    locally_armed: bool = False
    synchronized: bool = False
    limits: MotorLimits | None = None
    safety: MotorSafety | None = None
    session_id: int = 0
    profile: MotorProfile | None = None
    controls: bool = False

    @property
    def manual_profile_ready(self):
        return self.controls and self.profile is not None and self.profile.manual_valid

    def fresh(self, now):
        return (self.connected and self.synchronized and self.status is not None
                and 0 <= now - self.status.received_at <= FRESH_SECONDS)

    def blocks_dialog(self, now):
        # Blocking ROI tools hide STOP: require fresh, idle, disarmed status.
        return self.connected and (not self.fresh(now) or self.pending not in (None, "STATUS")
                                   or self.status.moving or self.status.armed)


class MotorLink:
    """One normal transaction (write -> ACK -> state), plus priority STOP.

    Replies have no command identifier. Keep sent replies in FIFO order even
    when STOP overtakes the normal transaction. Never retry a write or a job.
    """

    def __init__(self):
        self.lock = RLock()
        self.snapshot = MotorSnapshot()
        self._queued = None
        self._queued_condition = None
        self._queued_entitlement = False
        self._sent = deque()
        self._awaiting = None
        self._last_poll = float("-inf")
        self._session_id = 0
        self._invalidated = False

    def _update(self, **changes):
        self.snapshot = replace(self.snapshot, **changes)

    def _clear_queued(self):
        self._queued = None
        self._queued_condition = None
        self._queued_entitlement = False

    def _protocol_fault(self, message):
        # Keep already-sent replies in FIFO order, but never revive an unsent
        # action or an earlier ARM grant when a later STATUS resynchronizes.
        if self._queued != "STOP":
            self._clear_queued()
        self._invalidated = True
        pending = self._queued or (self._sent[0][0] if self._sent else
                                  self._awaiting[0] if self._awaiting else None)
        self._update(synchronized=False, locally_armed=False, pending=pending, message=message)

    def _command_reason(self, action, now, entitlement):
        s = self.snapshot
        if not s.fresh(now):
            return "fresh controller STATUS required"
        status = s.status
        if status.moving:
            return "controller is moving"
        if action != "DISARM" and not status.configured:
            return "Controller CONFIG_LOCKED/fault; check firmware configuration"
        if action == "GUARD":
            if s.safety is None:
                return "Updated controller SAFETY metadata required for GUARD"
            if status.armed:
                return "Disarm before enabling communication GUARD"
        elif action in {"REFERENCE_NEUTRAL", "REFERENCE_REST"} and status.armed:
            return "Stop/disarm before declaring a physical reference"
        elif action == "REFERENCE_REST" and not s.manual_profile_ready:
            return "Upload latest grip_control before declaring retracted Rest"
        elif action == "SET_STROKES":
            if status.armed:
                return "Disarm before applying pulse settings"
            if not s.controls:
                return "Upload latest grip_control for editable pulse settings"
        elif action in {"ARM", "GRAB", "NEUTRAL", "JOG"} and not status.referenced:
            return "Physical neutral reference required"
        elif action in {"GRAB", "NEUTRAL", "JOG"} and not (status.armed and entitlement):
            return "Manual arm is off or the move's ARM was not acknowledged"
        elif action in {"GRAB", "NEUTRAL"} and not s.manual_profile_ready:
            return "Upload corrected grip_control: physical-zero/Rest/Grasp PROFILE required"
        elif action == "GRAB" and status.state != "NEUTRAL":
            return "Grab requires prepared Rest; use Prepare / Return Rest first"
        elif action == "NEUTRAL" and status.state == "NEUTRAL":
            return "Already at Rest; no movement sent"
        return None

    def connect(self):
        with self.lock:
            self._clear_queued()
            self._sent.clear()
            self._awaiting = None
            self._last_poll = float("-inf")
            self._session_id += 1
            self._invalidated = False
            self.snapshot = MotorSnapshot(True, message="Connected; awaiting STATUS (no replay)",
                                          session_id=self._session_id)

    def disconnect(self):
        with self.lock:
            self._clear_queued()
            self._sent.clear()
            self._awaiting = None
            self._invalidated = False
            self.snapshot = MotorSnapshot(message="USB unavailable; outcome unknown; no replay",
                                          session_id=self._session_id)

    def request(self, command, now=None, condition=None):
        now = time.monotonic() if now is None else now
        with self.lock:
            s = self.snapshot
            if command == "STOP":
                if not s.connected:
                    self._update(message="STOP unavailable: no USB connection; use physical cutoff")
                    return False
                if (self._queued == "STOP" or any(c == "STOP" for c, _ in self._sent)
                        or (self._awaiting and self._awaiting[0] == "STOP")):
                    return False
                self._queued = "STOP"  # Discard an unsent normal command.
                self._queued_condition = None
                self._queued_entitlement = False
                self._update(pending="STOP", locally_armed=False, message="STOP requested; awaiting acknowledgment")
                return True
            jog = re.fullmatch(r"JOG ([AB]) (-?[0-9]+)", command)
            strokes = re.fullmatch(r"SET_STROKES ([+-]?[0-9]{1,4}) ([+-]?[0-9]{1,4})", command)
            action = "JOG" if jog else "SET_STROKES" if strokes else command
            if action not in {"REFERENCE_NEUTRAL", "REFERENCE_REST", "SET_STROKES", "ARM", "DISARM", "GRAB", "NEUTRAL", "JOG", "GUARD"}:
                self._update(message="Unsupported controller command")
                return False
            if action == "JOG" and (jog is None or not -0x80000000 <= int(jog[2]) <= 0x7FFFFFFF or int(jog[2]) == 0):
                self._update(message="Jog requires an axis and a nonzero signed pulse count")
                return False
            if action == "SET_STROKES" and (strokes is None or any(not 1 <= abs(int(n)) <= 2500 for n in strokes.groups())):
                self._update(message="Each signed stroke must be -2500..-1 or 1..2500 pulses")
                return False
            if not s.fresh(now):
                self._update(message="Command unavailable: fresh controller STATUS required")
                return False
            # One user action may wait behind a read-only STATUS poll. Otherwise
            # routine polling makes buttons intermittently reject valid clicks.
            polling = (all(c == "STATUS" for c, _ in self._sent)
                       and (self._awaiting is None or self._awaiting[0] == "STATUS"))
            if self._queued or ((self._sent or self._awaiting) and not polling) or s.status.moving:
                self._update(message="Busy; action ignored (no movement queue)")
                return False
            reason = self._command_reason(action, now, s.locally_armed)
            if reason:
                self._update(message=reason)
                return False
            self._queued = command
            self._queued_condition = condition
            self._queued_entitlement = s.locally_armed
            self._update(pending=command, message=f"{command} requested; awaiting acknowledgment",
                         locally_armed=False if action in {"GRAB", "NEUTRAL", "JOG", "DISARM", "REFERENCE_NEUTRAL", "REFERENCE_REST"}
                         else s.locally_armed)
            return True

    def next_write(self, now):
        """Called only by the USB worker; finite ACK/state deadline resets session."""
        with self.lock:
            if not self.snapshot.connected:
                return None
            # STOP must leave even when the older command has timed out.
            if self._queued == "STOP":
                command = "STOP"
                self._clear_queued()
            else:
                deadlines = [sent for _, sent in self._sent]
                if self._awaiting:
                    deadlines.append(self._awaiting[2])
                reply_limit = REPLY_SECONDS if self.snapshot.synchronized else CONNECT_REPLY_SECONDS
                if deadlines and now - min(deadlines) > reply_limit:
                    raise TimeoutError("Controller reply missing; command outcome unknown")
                if self._sent or self._awaiting:
                    return None
                command = self._queued
                condition = self._queued_condition
                entitlement = self._queued_entitlement
                self._clear_queued()
                if command is None and now - self._last_poll >= POLL_SECONDS:
                    command = "STATUS"
                if command is None:
                    return None
                if command != "STATUS":
                    action = "JOG" if command.startswith("JOG ") else "SET_STROKES" if command.startswith("SET_STROKES ") else command
                    reason = self._command_reason(action, now, entitlement)
                    if reason is None and condition is not None:
                        try:
                            if not condition():
                                reason = "automatic conditions changed"
                        except Exception:
                            reason = "automatic condition check failed"
                    if reason is not None:
                        self._update(pending=None, locally_armed=False,
                                     message=f"{command} canceled before write: {reason}")
                        if self.snapshot.status is None or not self.snapshot.status.armed:
                            return None
                        # Canceling an automatic GRAB must not leave its ARM live.
                        command = "STOP"
            self._sent.append((command, now))
            if command == "STATUS":
                self._last_poll = now
            self._update(pending=command)
            return (command + "\n").encode("ascii")

    def receive(self, event, now):
        with self.lock:
            if isinstance(event, MotorControls):
                self._update(controls=event.editable)
                return
            if isinstance(event, MotorProfile):
                self._update(profile=event)
                return  # Installed endpoints never acknowledge, reference or arm.
            if isinstance(event, MotorSafety):
                self._update(safety=event)
                return  # Read-only metadata never acknowledges or authorizes work.
            if isinstance(event, MotorLimits):
                self._update(limits=event)
                return  # Configuration metadata cannot acknowledge a command.
            if isinstance(event, MotorReply):
                if event.job_id is None:
                    # Unsolicited startup/framing fault cannot acknowledge a command.
                    self._protocol_fault(f"Controller error: {event.result}")
                    return
                if not self._sent or self._awaiting:
                    self._protocol_fault("Unexpected reply; controller state unavailable")
                    return
                command, sent = self._sent.popleft()
                self._awaiting = (command, event, sent)
                if event.kind == "ERR":
                    self._update(message=f"{command} rejected: {event.result}", locally_armed=False)
                elif command != "STATUS":
                    self._update(message=f"{command} acknowledged; awaiting state")
                return
            if not isinstance(event, MotorStatus):
                return
            self._update(status=event)
            if self._awaiting:
                command, reply, _ = self._awaiting
                self._awaiting = None
                if event.job_id != reply.job_id:
                    self._protocol_fault("Reply/state job mismatch; outcome unknown")
                else:
                    stopping = self._queued == "STOP" or any(c == "STOP" for c, _ in self._sent)
                    if command == "STATUS":
                        self._invalidated = False
                    self._update(synchronized=not self._invalidated,
                                 locally_armed=(reply.kind == "ACK" and command == "ARM"
                                                and event.armed and not stopping and not self._invalidated)
                                 if command != "STATUS" else self.snapshot.locally_armed)
                    if reply.kind == "ACK" and command != "STATUS":
                        self._update(message=f"{command} acknowledged | {event.state} / {event.result}")
                self._update(pending=self._queued or (self._sent[0][0] if self._sent else None))
            if not event.armed or event.moving:
                self._update(locally_armed=False)


def motor_lines(snapshot, now=None):
    now = time.monotonic() if now is None else now
    if not snapshot.connected:
        return ("Motor: USB unavailable | " + snapshot.message, "No motor state / reference / arm available")
    if not snapshot.fresh(now):
        return ("Motor: state unknown/stale | STOP available", snapshot.message)
    s = snapshot.status
    posture = "REST" if s.state == "NEUTRAL" and snapshot.manual_profile_ready else s.state
    detail = (f"Motor: {posture} / {s.result} | job {s.job_id} | config {int(s.configured)} "
              f"ref {int(s.referenced)} arm {int(s.armed)}")
    if s.state == "HOLD_UNCONFIRMED" and s.result == "COMPLETE":
        detail = f"Motor: completed GRAB; HOLD UNCONFIRMED | job {s.job_id}"
    entitlement = " | next move armed" if snapshot.locally_armed else " | next move unarmed"
    return (detail, snapshot.message[:80] + entitlement)
