"""Explicitly enabled, measured-window, one-shot capture coordination.

No device access or movement retry lives here. The existing MotorLink performs
GUARD/ARM/GRAB transactions; its write predicate rechecks this cycle immediately
before each write. Dwell resets like a PLC on-delay timer, and an enable produces
only one capture, following the simple TON/R_TRIG semantics documented by
Beckhoff: https://infosys.beckhoff.com/content/1033/tcplclib_tc2_standard/74406539.html
and https://infosys.beckhoff.com/content/1033/tcplclib_tc2_standard/74391563.html .
"""

from dataclasses import dataclass, replace
import json
import math
from pathlib import Path
from threading import RLock

from motor_control import FRESH_SECONDS, MotorSnapshot, PAPER_PROFILE
from operator_guidance import Calibration
from sensor_telemetry import STALE_SECONDS, SensorReading


MINIMUM_SAMPLES = 3
_UNSET = object()
FAULT_RESULTS = frozenset(("CONFIG_LOCKED", "AXIS_FAULT", "TIMER_FAULT",
                           "MOVE_TIMEOUT", "GUARD_TIMEOUT", "ARM_EXPIRED"))


@dataclass(frozen=True)
class AutoCaptureConfig:
    """Dwell is supplied explicitly after the real capture window is measured."""

    dwell_seconds: float

    def __post_init__(self):
        if (type(self.dwell_seconds) not in (int, float)
                or not math.isfinite(self.dwell_seconds)
                or not 0 < self.dwell_seconds <= 5):
            raise ValueError("Auto-capture dwell_seconds must be positive and at most 5 seconds")


def load_auto_capture_config(path):
    """No default file or operational values; malformed/unset config stays off."""
    try:
        data = json.loads(Path(path).read_text(encoding="utf-8"))
        if not isinstance(data, dict) or set(data) != {"dwell_seconds"}:
            raise ValueError("Expected only dwell_seconds")
        return AutoCaptureConfig(**data)
    except (OSError, TypeError, ValueError) as exc:
        raise ValueError("Cannot load auto-capture configuration; supply an explicit valid dwell_seconds") from exc


@dataclass(frozen=True)
class CaptureContext:
    calibration: Calibration | None
    frame_shape: tuple
    frame_received_at: float
    target_number: int | None
    target_center: tuple[float, float] | None
    tracking: bool
    reading: SensorReading | None
    correspondence: bool
    motor: MotorSnapshot


class AutoCapture:
    """One explicit enable -> GUARD -> stable window -> ARM -> one GRAB.

    Pass the token captured when tick returns a command to write_allowed. A
    canceled cycle cannot authorize its queued command after another enable.
    cancel returns whether the caller must request one priority STOP. Internal
    cancellation returns STOP from tick instead. command_result reports queue
    acceptance, not physical completion. No state claims a confirmed grasp.
    """

    def __init__(self, config=None):
        if config is not None and not isinstance(config, AutoCaptureConfig):
            raise TypeError("Expected AutoCaptureConfig or None")
        self.config = config
        self.state = "OFF"
        self.message = "Auto capture off; measured configuration required" if config is None else "Auto capture off"
        self.token = 0
        self._lock = RLock()
        self._context = None
        self._identity = None
        self._issued = set()
        self._written = set()
        self._submitted = set()
        self._action_at = 0.0
        self._arm_at = None
        self._arm_requested = False
        self._grab_written = False
        self._stop_pending = False
        self._last_now = None
        self._reset_dwell()

    @property
    def enabled(self):
        return self.state in ("GUARDING", "WAITING", "ARMING")

    def _reset_dwell(self):
        self._since = None
        self._sample_ids = set()
        self._first_sample_at = None
        self._last_sample_at = None

    def _base_reason(self, context, now, *, guard=False, command=None, locked=True):
        if self.config is None:
            return "Measured auto-capture configuration required"
        if not math.isfinite(now):
            return "Clock unavailable"
        c = context.calibration
        if c is None or not c.matches(context.frame_shape) or c.range_window_cm is None:
            return "Measured image calibration and range window required"
        if not (math.isfinite(context.frame_received_at)
                and 0 <= now - context.frame_received_at <= FRESH_SECONDS):
            return "Camera observation stale"
        center = context.target_center
        if (context.tracking is not True or type(context.target_number) is not int
                or context.target_number <= 0 or center is None or len(center) != 2
                or not all(math.isfinite(v) for v in center)
                or not 0 <= center[0] < c.width or not 0 <= center[1] < c.height):
            return "Selected target lost or unavailable"
        if context.correspondence is not True:
            return "Echo/selected-target correspondence required"
        r = context.reading
        if (r is None or r.status != "OK" or r.distance_cm is None
                or not math.isfinite(r.distance_cm) or not 2 <= r.distance_cm <= 400
                or not math.isfinite(r.received_at)
                or not 0 <= now - r.received_at <= STALE_SECONDS):
            return "Fresh valid target range required"
        m = context.motor
        if not m.fresh(now):
            return "Fresh synchronized controller required"
        if type(m.session_id) is not int or m.session_id <= 0:
            return "Controller session unavailable"
        s = m.status
        if m.profile != PAPER_PROFILE:
            return "Corrected physical-zero/Rest/Grasp controller PROFILE required"
        if (not s.configured or not s.referenced or s.state != "NEUTRAL"
                or s.moving or s.fault_axis != -1 or s.result in FAULT_RESULTS):
            return "Referenced, fault-free prepared Rest required"
        safety = m.safety
        if (safety is None or safety.physical_ready is not True
                or type(safety.guard_timeout_ms) is not int or safety.guard_timeout_ms <= 0
                or type(safety.arm_timeout_ms) is not int or safety.arm_timeout_ms <= 0
                or not math.isfinite(safety.received_at)
                or not 0 <= now - safety.received_at <= FRESH_SECONDS):
            return "Commissioned physical profile and controller safety metadata required"
        if guard and safety.guard_active is not True:
            return "Controller communication guard unavailable"
        if m.pending not in (None, "STATUS", command):
            return "Another controller action is pending"
        if locked and self._identity is not None:
            expected = (c, tuple(context.frame_shape[:2]), context.target_number, m.session_id)
            if expected != self._identity:
                return "Target, calibration or controller session changed"
        return None

    @staticmethod
    def _in_window(context):
        c, center = context.calibration, context.target_center
        return (all(abs(center[i] - c.crosshair[i]) <= c.tolerance[i] for i in (0, 1))
                and c.range_window_cm[0] <= context.reading.distance_cm <= c.range_window_cm[1])

    def enable(self, context, now):
        with self._lock:
            if self.enabled:
                self.message = "Auto capture already enabled"
                return False
            reason = self._base_reason(context, now, locked=False)
            if reason is None and (context.motor.status.armed or context.motor.locally_armed):
                reason = "Disarm before enabling auto capture"
            if reason:
                self.message = reason
                return False
            self.token += 1
            self._context = context
            self._identity = (context.calibration, tuple(context.frame_shape[:2]),
                              context.target_number, context.motor.session_id)
            self._issued.clear()
            self._written.clear()
            self._submitted.clear()
            self._arm_requested = self._grab_written = self._stop_pending = False
            self._arm_at = None
            self._last_now = now
            self._reset_dwell()
            self.state, self.message = "GUARDING", "Auto enabled; establishing controller guard"
            return True

    def cancel(self, reason="Auto capture canceled"):
        with self._lock:
            stop = self._arm_requested and not self._grab_written
            self.token += 1
            self.state, self.message = "OFF", reason
            self._identity = None
            self._issued.clear()
            self._reset_dwell()
            self._arm_requested = False
            return stop

    def _stop_needed(self, context, now):
        # The worker may already have substituted STOP for a denied queued
        # GRAB. Suppress a duplicate only after new, synchronized idle/disarmed
        # state proves the ARM is no longer live and no command is pending.
        m = context.motor
        return not (self._arm_at is not None and m.fresh(now)
                    and m.pending in (None, "STATUS") and not m.status.armed
                    and not m.status.moving and m.status.received_at > self._arm_at)

    def _cancel_command(self, reason, context, now):
        stop = self.cancel(reason)
        return "STOP" if stop and self._stop_needed(context, now) else None

    def _issue(self, command, now):
        self._issued.add(command)
        self._action_at = now
        return command

    def command_result(self, command, accepted):
        with self._lock:
            if command == "STOP":
                self._stop_pending = False
                return
            if command not in self._issued:
                return
            if accepted:
                self._submitted.add(command)
            else:
                self._stop_pending = self.cancel(f"Auto {command} request denied; no retry")

    def tick(self, context, now):
        with self._lock:
            self._context = context
            if self._stop_pending:
                self._stop_pending = False
                return "STOP" if self._stop_needed(context, now) else None
            if self.state == "OFF":
                return None
            if self.state == "FIRED" and self._grab_written:
                return None  # Expected wrapping occlusion never releases or repeats.
            if self._last_now is not None and now < self._last_now:
                return self._cancel_command("Clock moved backwards; auto capture canceled", context, now)
            self._last_now = now
            command = {"GUARDING": "GUARD", "ARMING": "ARM", "FIRED": "GRAB"}.get(self.state)
            reason = self._base_reason(context, now, guard=self.state != "GUARDING", command=command)
            if reason:
                return self._cancel_command(reason, context, now)
            m = context.motor
            if self.state == "GUARDING":
                if m.status.armed:
                    return self._cancel_command("Unexpected ARM while establishing guard", context, now)
                if "GUARD" not in self._issued:
                    return self._issue("GUARD", now)
                if ("GUARD" in self._submitted and m.pending is None
                        and m.status.received_at > self._action_at):
                    if not m.safety.guard_active:
                        return self._cancel_command("Controller did not establish GUARD; no retry", context, now)
                    self.state, self.message = "WAITING", "Auto enabled; waiting for stable capture window"
                else:
                    return None
            if self.state == "WAITING":
                if m.status.armed:
                    return self._cancel_command("Unexpected ARM before capture dwell", context, now)
                if not self._in_window(context):
                    self._reset_dwell()
                    self.message = "Auto enabled; waiting for alignment and measured range"
                    return None
                if self._since is None:
                    self._since = now
                    self._first_sample_at = context.reading.received_at
                self._sample_ids.add(context.reading.uptime_ms)
                self._last_sample_at = context.reading.received_at
                dwell = self.config.dwell_seconds
                if (now - self._since < dwell or len(self._sample_ids) < MINIMUM_SAMPLES
                        or self._last_sample_at - self._first_sample_at < dwell):
                    self.message = "Auto enabled; qualifying stable target range"
                    return None
                self.state, self.message = "ARMING", "Capture window stable; requesting one ARM"
                self._arm_requested = True
                self._arm_at = now
                return self._issue("ARM", now)
            if self.state == "ARMING":
                if not self._in_window(context):
                    return self._cancel_command("Capture window changed while arming", context, now)
                if ("ARM" in self._submitted and m.pending is None
                        and m.status.received_at > self._action_at):
                    if not m.locally_armed or not m.status.armed:
                        return self._cancel_command("Controller did not acknowledge ARM; no retry", context, now)
                    self.state, self.message = "FIRED", "One GRAB requested; hold remains unconfirmed"
                    return self._issue("GRAB", now)
            elif self.state == "FIRED" and not self._in_window(context):
                return self._cancel_command("Capture window changed before GRAB write", context, now)
            return None

    def write_allowed(self, command, now, token=None, *, motor=_UNSET, reading=_UNSET):
        """Validate current worker snapshots plus the latest real camera frame.

        None is actual missing metadata/range, not permission to reuse cached
        data. Omitting an override is useful only in pure state-machine tests.
        """
        with self._lock:
            if token is not None and token != self.token:
                return False
            if (command not in self._issued or command in self._written
                    or self._context is None or self.state == "OFF"):
                return False
            if self.state != {"GUARD": "GUARDING", "ARM": "ARMING", "GRAB": "FIRED"}.get(command):
                return False
            context = self._context
            if motor is not _UNSET:
                context = replace(context, motor=motor)
            if reading is not _UNSET:
                context = replace(context, reading=reading)
            reason = ("Controller snapshot unavailable" if context.motor is None else
                      self._base_reason(context, now, guard=command != "GUARD", command=command))
            if (reason is None and reading is not _UNSET and self._context.reading is not None
                    and abs(reading.distance_cm - self._context.reading.distance_cm)
                    > (context.calibration.range_window_cm[1] - context.calibration.range_window_cm[0]) / 2):
                reason = "Echo changed beyond confirmed correspondence before write"
            if reason is None and command in ("ARM", "GRAB") and not self._in_window(context):
                reason = "Capture window changed before automatic command write"
            if reason is None and command == "GRAB" and not context.motor.status.armed:
                reason = "ARM no longer valid before GRAB write"
            if reason is None and command in ("GUARD", "ARM") and context.motor.status.armed:
                reason = "Controller already armed before automatic setup write"
            if reason:
                self._stop_pending = self.cancel(reason)
                return False
            self._written.add(command)
            if command == "GRAB":
                self._grab_written = True
            return True
