"""Guide approach using camera calibration and a confirmed range reading."""

from dataclasses import dataclass
import json
import math
from pathlib import Path

from sensor_telemetry import STALE_SECONDS


@dataclass(frozen=True)
class Calibration:
    width: int
    height: int
    crosshair: tuple[float, float]
    tolerance: tuple[float, float]
    right_sign: int
    down_sign: int
    axes_verified: bool
    range_window_cm: tuple[float, float] | None = None

    def __post_init__(self):
        numbers = (*self.crosshair, *self.tolerance)
        if (type(self.width) is not int or type(self.height) is not int
                or self.width <= 0 or self.height <= 0 or len(self.crosshair) != 2 or len(self.tolerance) != 2
                or not all(math.isfinite(n) for n in numbers)
                or not 0 <= self.crosshair[0] < self.width or not 0 <= self.crosshair[1] < self.height
                or any(n <= 0 for n in self.tolerance)
                or type(self.right_sign) is not int or type(self.down_sign) is not int
                or self.right_sign not in (-1, 1) or self.down_sign not in (-1, 1)
                or type(self.axes_verified) is not bool):
            raise ValueError("Invalid measured camera/gripper calibration")
        if self.range_window_cm is not None:
            if (len(self.range_window_cm) != 2 or not all(math.isfinite(n) for n in self.range_window_cm)
                    or not 2 <= self.range_window_cm[0] < self.range_window_cm[1] <= 400):
                raise ValueError("Invalid measured sensor-face range window")

    def matches(self, shape):
        return self.axes_verified and tuple(shape[:2]) == (self.height, self.width)


def load_calibration(path):
    """Load measured camera and range calibration from JSON."""
    try:
        data = json.loads(Path(path).read_text(encoding="utf-8"))
        return Calibration(**data)
    except (OSError, TypeError, ValueError) as exc:
        raise ValueError("Cannot load guidance calibration; check measured JSON fields") from exc


@dataclass(frozen=True)
class Guidance:
    alignment: str
    range_advice: str


def guidance(calibration, shape, target_center, reading=None, now=0.0, correspondence=False):
    if (target_center is None or len(target_center) != 2
            or not all(math.isfinite(n) for n in target_center)
            or not 0 <= target_center[0] < shape[1] or not 0 <= target_center[1] < shape[0]):
        return Guidance("Alignment unavailable: target lost/unselected", "Range guidance unavailable: target unknown")
    if calibration is None or not calibration.matches(shape):
        return Guidance("Alignment unavailable: calibration unset/unverified or image size changed",
                        "Range guidance unavailable: calibration required")
    dx = target_center[0] - calibration.crosshair[0]
    dy = target_center[1] - calibration.crosshair[1]
    moves = []
    if abs(dx) > calibration.tolerance[0]:
        moves.append("move right" if dx * calibration.right_sign > 0 else "move left")
    if abs(dy) > calibration.tolerance[1]:
        moves.append("move down" if dy * calibration.down_sign > 0 else "move up")
    alignment = " / ".join(moves) if moves else "Aligned (measured image tolerance)"
    if calibration.range_window_cm is None:
        advice = "Range guidance unavailable: sensor-to-gripper window unset"
    elif not correspondence:
        advice = "Range guidance unavailable: establish echo/selected-target correspondence"
    elif (reading is None or reading.status != "OK" or reading.distance_cm is None
          or not 0 <= now - reading.received_at <= STALE_SECONDS):
        advice = "Range guidance unavailable: fresh valid echo required"
    elif reading.distance_cm < calibration.range_window_cm[0]:
        advice = "Move back (measured range window)"
    elif reading.distance_cm > calibration.range_window_cm[1]:
        advice = "Move closer (measured range window)"
    else:
        advice = "Distance OK (operator-confirmed echo; hold unconfirmed)"
    return Guidance(alignment, advice)


class Correspondence:
    """Explicit operator evidence is per target and expires with loss/stale range.

    Camera or object movement can change the echo: confirmation also expires
    when its image box or measured range changes beyond calibration tolerance.
    """

    def __init__(self):
        self.clear()

    def clear(self):
        self.target = None
        self.box = None
        self.distance = None

    def confirm(self, calibration, shape, selection, reading, now):
        self.clear()
        if (calibration is None or not calibration.matches(shape) or calibration.range_window_cm is None
                or selection.state != "tracking" or selection.current is None or reading is None
                or reading.status != "OK" or reading.distance_cm is None
                or not 0 <= now - reading.received_at <= STALE_SECONDS):
            return False
        self.target, self.box = selection.number, selection.current.box
        self.distance = reading.distance_cm
        return True

    def valid(self, calibration, shape, selection, reading, now):
        if not self.matches(calibration, shape, selection, reading, now):
            self.clear()
            return False
        return True

    def matches(self, calibration, shape, selection, reading, now):
        """Read-only write-boundary check against the operator's baseline.

        The USB worker may inspect current range without clearing UI evidence.
        Copy the evidence/selection fields so a concurrent clear fails closed.
        """
        target, box, distance = self.target, self.box, self.distance
        current, number, state = selection.current, selection.number, selection.state
        if (target is None or box is None or distance is None or calibration is None
                or not calibration.matches(shape) or calibration.range_window_cm is None
                or state != "tracking" or current is None or number != target
                or reading is None or reading.status != "OK" or reading.distance_cm is None
                or not 0 <= now - reading.received_at <= STALE_SECONDS):
            return False
        # Expire the confirmation after half the calibrated range window.
        return (all(abs(a - b) <= calibration.tolerance[i % 2]
                    for i, (a, b) in enumerate(zip(current.box, box)))
                and abs(reading.distance_cm - distance) <=
                (calibration.range_window_cm[1] - calibration.range_window_cm[0]) / 2)
