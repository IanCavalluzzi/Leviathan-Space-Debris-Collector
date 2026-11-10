"""Manual jog, Grab, and Rest controls without a camera."""

import argparse
import re
import time

import cv2
import numpy as np

from preview_layout import text_lines
from sensor_telemetry import SensorReader

WINDOW = "Debris Capture - Manual Motors"
SIZE = (900, 770)
JOG_COUNTS = (1, 5, 10)
PULSE_CEILING = 2500  # Firmware also enforces cumulative travel limits.
KEYS = {ord("1"): "A+", ord("2"): "A-", ord("3"): "B+", ord("4"): "B-"}


class ManualControl:
    """Wait for an ARM acknowledgment before sending one movement command."""

    def __init__(self, reader):
        self.reader = reader
        self.steps = 10
        self.entry = "10"
        self.editing = False
        self.replace_entry = False
        self.after_arm = None
        self.reference_until = 0.0
        self.reference_action = None
        self.action_session = None
        self.stroke_entries = {"A": "900", "B": "900"}
        self.stroke_edit = None
        self.stroke_replace = False
        self.last_profile = None
        self.notice = "Declare physical neutral OR already-retracted Rest before moving. Neither reference button moves motors."

    def clear_action(self):
        self.after_arm = None
        self.reference_until = 0.0
        self.reference_action = None

    def stop(self):
        self.clear_action()
        self.editing = False
        self.stroke_edit = None
        self.entry = str(self.steps)
        self.reader.command("STOP")
        self.notice = "STOP requested. Check the controller reply; STOP does not cut power."

    def reference(self, now, retracted=False):
        s = self.reader.motor_snapshot
        command = "REFERENCE_REST" if retracted else "REFERENCE_NEUTRAL"
        if (self.after_arm is not None or not s.fresh(now) or s.status.moving
                or s.status.armed or s.pending not in (None, "STATUS") or not s.status.configured):
            self.reference_until = 0.0
            self.notice = "Need connected, configured, idle/disarmed controller. STOP can disarm."
            return False
        if retracted and not s.manual_profile_ready:
            self.reference_until = 0.0
            self.notice = "Upload latest grip_control before declaring retracted Rest."
            return False
        if self.reference_action != command or not self.reference_until or now > self.reference_until:
            self.reference_until = now + 5.0
            self.reference_action = command
            posture = "retracted Rest (A 900 / B 0)" if retracted else "pre-tensioned neutral (A 0 / B 0)"
            self.notice = f"Physically establish {posture}, then confirm within 5 seconds. No movement."
            return False
        self.reference_until = 0.0
        self.reference_action = None
        accepted = self.reader.command(command)
        self.notice = "Physical reference declaration requested; no movement." if accepted else self.reader.motor_snapshot.message
        return accepted

    def jog(self, axis, amount, now):
        s = self.reader.motor_snapshot
        self.reference_until = 0.0
        if axis not in ("A", "B") or type(amount) is not int or not 1 <= abs(amount) <= PULSE_CEILING:
            self.notice = "Choose A/B and 1 to 2500 pulses; use +/- for direction."
            return False
        if (self.after_arm is not None or not s.fresh(now) or s.status.moving
                or s.pending not in (None, "STATUS")):
            self.notice = "Wait for a fresh idle controller; repeated clicks are not queued."
            return False
        if not s.status.configured or not s.status.referenced:
            self.notice = "Set physical neutral first; a locked controller also needs its configuration checked."
            return False
        if s.limits is None:
            if abs(amount) > 10:
                self.notice = "Upload updated grip_control first: installed firmware has not reported larger jog limits."
                return False
        else:
            limits = ((s.limits.min_a, s.limits.max_a, s.limits.jog_a, s.status.position_a)
                      if axis == "A" else (s.limits.min_b, s.limits.max_b, s.limits.jog_b, s.status.position_b))
            low, high, maximum, position = limits
            if abs(amount) > maximum or not low <= position + amount <= high:
                self.notice = f"{axis} move rejected: jog cap {maximum}; position {position}, allowed {low} to {high}."
                return False
        return self.arm_job(f"JOG {axis} {amount}")

    def move(self, command, now):
        s = self.reader.motor_snapshot
        self.reference_until = 0.0
        if (self.after_arm is not None or not s.fresh(now) or s.status.moving
                or s.pending not in (None, "STATUS")):
            self.notice = "Wait for a fresh idle controller; repeated clicks are not queued."
            return False
        if not s.status.configured or not s.status.referenced:
            self.notice = "Declare physical neutral or already-retracted Rest first."
            return False
        if not s.manual_profile_ready:
            self.notice = "Upload corrected grip_control first: installed Rest/Grasp PROFILE is missing or different."
            return False
        if command == "GRAB" and s.status.state != "NEUTRAL":
            self.notice = f"Prepare Rest first: A {s.profile.rest_a} / B 0."
            return False
        if command == "NEUTRAL" and s.status.state == "NEUTRAL":
            self.notice = "Already at Rest; no movement requested."
            return False
        return self.arm_job(command)

    def arm_job(self, command):
        if not self.reader.command("ARM"):
            self.notice = self.reader.motor_snapshot.message
            return False
        self.after_arm = command
        self.action_session = self.reader.motor_snapshot.session_id
        self.notice = f"{command} requested; waiting for ARM acknowledgment."
        return True

    def tick(self, now):
        profile = self.reader.motor_snapshot.profile
        if profile is not None and profile != self.last_profile and self.stroke_edit is None:
            self.stroke_entries = {"A": str(profile.rest_a), "B": str(profile.grasp_b)}
            self.last_profile = profile
        if not self.reader.motor_snapshot.connected:
            self.clear_action()
            return
        if self.after_arm is None:
            return
        s = self.reader.motor_snapshot
        if not s.fresh(now) or s.session_id != self.action_session:
            self.after_arm = None
            self.notice = "Controller status unavailable; movement canceled without replay."
        elif s.pending is None:
            command, self.after_arm = self.after_arm, None
            if s.locally_armed and s.status.armed and s.status.referenced and not s.status.moving:
                accepted = self.reader.command(command)
                self.notice = f"{command} requested; wait for COMPLETE." if accepted else self.reader.motor_snapshot.message
            else:
                self.notice = "Arm was not confirmed; no movement sent. Check the controller reply."

    def action(self, action, now):
        if action == "STOP":
            self.stop()
        elif action == "REFERENCE":
            self.reference(now)
        elif action == "REFERENCE_REST":
            self.reference(now, retracted=True)
        elif action in ("STROKE_A", "STROKE_B"):
            self.stroke_edit = action[-1]
            self.stroke_replace = True
            self.editing = False
            self.notice = "Type signed pulses, then Apply pulses or Enter. Editing does not move motors."
        elif action == "APPLY_STROKES":
            self.apply_strokes(now)
        elif action in ("JOG_A_PLUS", "JOG_A_MINUS", "JOG_B_PLUS", "JOG_B_MINUS"):
            self.jog(action[4], 100 if action.endswith("PLUS") else -100, now)
        elif action in ("GRAB", "NEUTRAL"):
            self.move(action, now)
        elif action.startswith("STEP") and action[4:].isdigit() and int(action[4:]) in JOG_COUNTS:
            self.steps = int(action[4:])
            self.entry, self.editing = str(self.steps), False
        elif action == "PULSES":
            self.editing, self.replace_entry = True, True
            self.notice = "Type a positive pulse count, then Enter or Set. Typing does not move a motor."
        elif action == "APPLY":
            self.apply_entry()
        elif action in ("A+", "A-", "B+", "B-"):
            if self.editing and not self.apply_entry():
                return
            self.jog(action[0], self.steps if action[1] == "+" else -self.steps, now)

    def apply_entry(self):
        if not self.entry.isdigit() or not 1 <= int(self.entry) <= PULSE_CEILING:
            self.notice = "Pulse count must be 1 to 2500. No movement requested."
            return False
        self.steps, self.editing = int(self.entry), False
        self.entry = str(self.steps)
        self.notice = f"Set {self.steps} pulses. Choose an A/B direction button to move."
        return True

    def apply_strokes(self, now):
        values = [self.stroke_entries[axis] for axis in ("A", "B")]
        if any(not re.fullmatch(r"[+-]?[0-9]{1,4}", n) or not 1 <= abs(int(n)) <= 2500 for n in values):
            self.notice = "Each signed stroke must be -2500..-1 or 1..2500. No movement."
            return False
        s = self.reader.motor_snapshot
        if (self.after_arm is not None or not s.fresh(now) or s.status.moving or s.status.armed
                or s.pending not in (None, "STATUS") or not s.controls):
            self.notice = "Apply needs latest firmware and a fresh idle/disarmed controller. STOP can disarm."
            return False
        accepted = self.reader.command(f"SET_STROKES {int(values[0])} {int(values[1])}")
        self.stroke_edit = None
        self.notice = "Pulse settings requested; no movement. Saved until controller reset." if accepted else self.reader.motor_snapshot.message
        return accepted

    def stroke_key(self, key, now):
        if key in (10, 13):
            self.apply_strokes(now)
        elif key == 27:
            self.stroke_edit = None
        elif key in (8, 127):
            self.stroke_entries[self.stroke_edit] = "" if self.stroke_replace else self.stroke_entries[self.stroke_edit][:-1]
            self.stroke_replace = False
        elif key in (ord("+"), ord("-")) or ord("0") <= key <= ord("9"):
            entry = "" if self.stroke_replace else self.stroke_entries[self.stroke_edit]
            self.stroke_replace = False
            character = chr(key)
            if (character in "+-" and not entry) or (character.isdigit() and len(entry) < 5):
                self.stroke_entries[self.stroke_edit] = entry + character

    def edit_key(self, key):
        if key in (10, 13):
            self.apply_entry()
        elif key in (8, 127):
            self.entry = "" if self.replace_entry else self.entry[:-1]
            self.replace_entry = False
        elif ord("0") <= key <= ord("9"):
            if self.replace_entry:
                self.entry, self.replace_entry = "", False
            if len(self.entry) < 4:
                self.entry += chr(key)


def connection_lines(reader, motor, now):
    """Describe the current USB and controller state."""
    if motor.fresh(now):
        return "Controller responding", ""
    state = getattr(reader.snapshot, "state", "starting")
    messages = {
        "off": ("Display only; USB off", "Launch without --port off to connect to the UNO."),
        "starting": ("Finding the UNO", "Waiting for USB discovery and compatible controller replies."),
        "not_found": ("UNO USB port not detected", "Check the UNO USB connection; no serial port is currently detected."),
        "ambiguous": ("Multiple UNO boards detected", "Choose the intended --port explicitly."),
        "waiting": ("USB open; waiting for controller", "Load grip_control.ino; the loop test has no serial command reader."),
        "receiving": ("USB open; motor status unavailable", "Sensor data is arriving; check the command-capable grip_control sketch."),
        "access_denied": ("Port access denied", "Close Serial Monitor or another owner of this port; also check permissions."),
        "reply_timeout": ("Controller reply timed out", "USB opened but grip_control did not answer. Check the uploaded sketch."),
        "disconnected": ("USB connection interrupted", "Waiting to reconnect. Pending movements are discarded."),
    }
    return messages.get(state, ("Controller status unavailable", motor.message))


def buttons(control, now):
    confirming = now <= control.reference_until
    reference = "Confirm physical neutral" if confirming and control.reference_action == "REFERENCE_NEUTRAL" else "Set physical neutral"
    retracted = "Confirm retracted" if confirming and control.reference_action == "REFERENCE_REST" else "Set retracted (no movement)"
    result = [
        ("REFERENCE", reference, (20, 92, 420, 44)),
        ("STOP", "STOP  [Space / Esc]", (460, 92, 420, 88)),
        ("STEP1", "1 pulse", (20, 160, 130, 38)),
        ("STEP5", "5 pulses", (165, 160, 130, 38)),
        ("STEP10", "10 pulses", (310, 160, 130, 38)),
        ("PULSES", "Custom: " + control.entry + ("_" if control.editing else ""), (460, 160, 270, 38)),
        ("APPLY", "Set [Enter]", (745, 160, 135, 38)),
        ("A+", "A +   [1]", (20, 225, 420, 65)),
        ("A-", "A -   [2]", (20, 305, 420, 65)),
        ("B+", "B +   [3]", (460, 225, 420, 65)),
        ("B-", "B -   [4]", (460, 305, 420, 65)),
        ("GRAB", "Grab: A pays out / B pulls", (20, 385, 420, 50)),
        ("NEUTRAL", "Curl outers: Prepare / Return Rest", (460, 385, 420, 50)),
    ]
    return [(action, label, (x, y + 50 if y >= 160 else y, w, h))
            for action, label, (x, y, w, h) in result] + [
                ("REFERENCE_REST", retracted, (20, 142, 420, 38)),
                ("STROKE_A", "A: " + control.stroke_entries["A"] + ("_" if control.stroke_edit == "A" else ""), (20, 495, 130, 32)),
                ("STROKE_B", "B: " + control.stroke_entries["B"] + ("_" if control.stroke_edit == "B" else ""), (165, 495, 130, 32)),
                ("APPLY_STROKES", "Apply pulses", (310, 495, 130, 32)),
                ("JOG_A_PLUS", "A +100", (460, 495, 95, 32)),
                ("JOG_A_MINUS", "A -100", (565, 495, 95, 32)),
                ("JOG_B_PLUS", "B +100", (670, 495, 95, 32)),
                ("JOG_B_MINUS", "B -100", (775, 495, 105, 32))]


def control_at(x, y, control, now):
    return next((action for action, _, (left, top, width, height) in buttons(control, now)
                 if left <= x < left + width and top <= y < top + height), None)


def draw_panel(control, now):
    canvas = np.full((SIZE[1], SIZE[0], 3), (23, 27, 32), np.uint8)

    def label(text, x, y, width=860, scale=.48, color=(225, 228, 232), limit=1):
        for i, line in enumerate(text_lines(text, width, scale, limit)):
            cv2.putText(canvas, line, (x, y + i * 21), cv2.FONT_HERSHEY_SIMPLEX,
                        scale, color, 1, cv2.LINE_AA)

    s = control.reader.motor_snapshot
    port = control.reader.snapshot.port or control.reader.requested_port
    fresh = s.fresh(now)
    connection, explanation = connection_lines(control.reader, s, now)
    label("MANUAL MOTOR CONTROL", 20, 30, scale=.68)
    label(f"USB: {port} | {connection}", 20, 61)
    for action, text, (x, y, w, h) in buttons(control, now):
        color = (45, 45, 170) if action == "STOP" else (65, 75, 88)
        if action == f"STEP{control.steps}":
            color = (65, 110, 65)
        if (action == "PULSES" and control.editing) or action == f"STROKE_{control.stroke_edit}":
            color = (110, 90, 45)
        cv2.rectangle(canvas, (x, y), (x + w - 1, y + h - 1), color, -1)
        label(text, x + 12, y + h // 2 + 6, width=w - 24, scale=.60 if action in ("A+", "A-", "B+", "B-") else .48)
    label(f"Jog click = {control.steps} pulses. A = outers, B = inners; signed pulse settings choose pull directions.", 20, 258, scale=.43)
    if fresh:
        status = s.status
        posture = "REST" if status.state == "NEUTRAL" and s.manual_profile_ready else status.state
        label(f"{posture} / {status.result} | Commanded pulses A {status.position_a:+d}, B {status.position_b:+d}", 20, 572)
        label(s.message, 20, 599, limit=2)
    else:
        label(explanation, 20, 572, limit=2)
        label("Requires grip_control firmware; the loop test ignores serial STOP. This panel stays open on errors.", 20, 617, scale=.43)
    label(control.notice, 20, 650, limit=2)
    if s.limits is not None:
        label(f"Firmware jog caps A/B: {s.limits.jog_a}/{s.limits.jog_b}; position limits A {s.limits.min_a}..{s.limits.max_a}, B {s.limits.min_b}..{s.limits.max_b}", 20, 694, scale=.42)
    else:
        label("Firmware limits not reported: fine jogs only; upload updated grip_control for counts over 10.", 20, 694, scale=.42)
    label("Reference buttons declare posture without movement. Apply pulses changes settings until reset.", 20, 718, scale=.42)
    label("Space / Esc: STOP, stay open.  R twice: confirm physical neutral.  Q: stop and quit.", 20, 745, scale=.42)
    return canvas


def run_manual(port="auto"):
    reader = SensorReader(port)
    control = ManualControl(reader)
    try:
        # Keep button hit testing in the panel's fixed coordinate system.
        cv2.namedWindow(WINDOW, cv2.WINDOW_AUTOSIZE)

        def mouse(event, x, y, flags, param):
            if event == cv2.EVENT_LBUTTONDOWN:
                now = time.monotonic()
                action = control_at(x, y, control, now)
                if action is not None:
                    control.action(action, now)

        cv2.setMouseCallback(WINDOW, mouse)
        reader.start()
        while True:
            now = time.monotonic()
            control.tick(now)
            cv2.imshow(WINDOW, draw_panel(control, now))
            key = cv2.waitKey(20) & 0xFF
            if cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) < 1:
                return 0
            if key in (ord("q"), ord("Q")):
                control.stop()
                return 0
            if key in (27, ord(" ")):
                control.stop()
            elif control.stroke_edit is not None:
                control.stroke_key(key, now)
            elif control.editing:
                control.edit_key(key)
            elif key in (ord("r"), ord("R")):
                control.reference(now)
            elif key in KEYS:
                control.action(KEYS[key], now)
    finally:
        control.clear_action()
        reader.stop()
        try:
            cv2.destroyWindow(WINDOW)
        except cv2.error:
            pass


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="auto", help="UNO USB auto-discovery, explicit port, or off for display only")
    args = parser.parse_args(argv)
    return run_manual(args.port)


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (RuntimeError, OSError, cv2.error, KeyboardInterrupt) as exc:
        print(f"Manual control ended: {exc}")
