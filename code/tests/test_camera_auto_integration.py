"""Actual camera event loop + capture FSM + MotorLink, with simulated devices."""

from contextlib import ExitStack, redirect_stdout
from io import StringIO
import unittest
from unittest.mock import MagicMock, patch

import numpy as np

import camera_preview
from auto_capture import AutoCaptureConfig
from motor_control import MotorLink, parse_motor_line
from operator_guidance import Calibration
from preview_layout import DEFAULT_SIZE, PreviewLayout
from sensor_telemetry import SensorSnapshot, parse_line


CALIBRATION = Calibration(640, 480, (300, 200), (10, 15), 1, 1, True, (8, 12))


class Clock:
    def __init__(self):
        self.frame = 0

    def now(self):
        return 1 + self.frame / 5


class SimulatedReader:
    """Keep real command queuing, ACK/state parsing and write predicates."""

    def __init__(self, clock):
        self.clock = clock
        self.motor = MotorLink()
        self.snapshot = SensorSnapshot("waiting", "SIM")
        self.guard = self.armed = False
        self.physical_ready = self.referenced = True
        self.state = "NEUTRAL"
        self.job = 0
        self.job_kind = 0
        self.position_a, self.position_b = 900, 0
        self.stroke_a = self.stroke_b = 900
        self.requests, self.writes, self.predicates = [], [], []
        self.timeout = False
        self.distance = 10.0
        self.stopped = False

    @property
    def motor_snapshot(self):
        return self.motor.snapshot

    def start(self):
        self.motor.connect()
        self.sample()
        self.pump()

    def stop(self):
        self.stopped = True

    def sample(self):
        now = self.clock.now()
        payload = (f"{round(now * 1000)},0,TIMEOUT,NA,0,0,0,0,0" if self.timeout else
                   f"{round(now * 1000)},{round(self.distance * 58)},OK,{self.distance},0,0,0,0,0")
        self.snapshot = SensorSnapshot("receiving", "SIM", parse_line(payload, now))

    def command(self, command, condition=None):
        self.requests.append(command)
        checked = None
        if condition is not None:
            def checked():
                result = condition()
                self.predicates.append((command, result))
                return result
        return self.motor.request(command, self.clock.now(), condition=checked)

    def pump(self):
        now = self.clock.now()
        outgoing = self.motor.next_write(now)
        if outgoing is None:
            return
        command = outgoing.decode("ascii").strip()
        self.writes.append(command)
        if command == "GUARD":
            assert not self.armed
            self.guard = True
        elif command == "ARM":
            self.armed = True
        elif command == "GRAB":
            assert self.armed and self.referenced and self.state == "NEUTRAL"
            self.armed = False
            self.state = "HOLD_UNCONFIRMED"
            self.job += 1
            self.job_kind = 2
            self.position_a, self.position_b = 0, self.stroke_b
        elif command == "NEUTRAL":
            assert self.armed and self.referenced
            self.armed = False
            self.state = "NEUTRAL"
            self.job += 1
            self.job_kind = 3
            self.position_a, self.position_b = self.stroke_a, 0
        elif command == "REFERENCE_REST":
            self.referenced, self.state = True, "NEUTRAL"
            self.position_a, self.position_b = self.stroke_a, 0
        elif command.startswith("SET_STROKES "):
            _, a, b = command.split()
            self.stroke_a, self.stroke_b = int(a), int(b)
            self.physical_ready = False
            self.state = "NEUTRAL" if (self.position_a, self.position_b) == (self.stroke_a, 0) else "POSITIONED"
        elif command.startswith("JOG "):
            assert self.armed and self.referenced
            _, axis, amount = command.split()
            self.armed = False
            if axis == "A":
                self.position_a += int(amount)
            else:
                self.position_b += int(amount)
            self.state = "POSITIONED"
            self.job += 1
            self.job_kind = 1
        elif command == "STOP":
            self.guard = self.armed = False
        elif command != "STATUS":
            raise AssertionError(f"Unexpected automatic command: {command}")
        result = "COMPLETE" if self.job else "STOPPED" if command == "STOP" else "OK"
        positions = f"{self.position_a} {self.position_b}"
        lines = [f"# motor ACK OK {self.job}",
                 f"# motor SAFETY {int(self.physical_ready)} {int(self.guard)} 1500 10000",
                 "# motor LIMITS -2500 2500 2500 -2500 2500 2500",
                 "# motor CONTROLS 1",
                 f"# motor PROFILE 0 0 {self.stroke_a} 0 0 {self.stroke_b}",
                 f"# motor {self.state} 1 {int(self.armed)} {int(self.referenced)} {positions} {result} {self.job} {self.job_kind} -1"]
        for line in lines:
            event = parse_motor_line(line, now)
            assert event is not None, line
            self.motor.receive(event, now)


class CameraAutoIntegrationTests(unittest.TestCase):
    def run_scene(self, scenario):
        clock = Clock()
        reader = SimulatedReader(clock)
        manual = scenario.startswith("manual_")
        if manual:
            reader.physical_ready = False  # Manual mode does not bypass or set auto qualification.
            reader.timeout = True
        if scenario == "manual_no_reference":
            reader.referenced, reader.state = False, "UNREFERENCED"
        elif scenario == "manual_not_rest":
            reader.state = "POSITIONED"
        if scenario == "echo_baseline":
            reader.distance = 8.0
        frame = np.zeros((480, 640, 3), np.uint8)
        frame[150:250, 250:350] = 255
        callback = None
        views = []
        enabled_frame = 0
        acted = False
        after_action = 0
        capture = MagicMock()
        capture.isOpened.return_value = True

        def read_frame():
            clock.frame += 1
            reader.sample()  # Fresh serial range may change before a queued write.
            reader.pump()
            return True, frame.copy()

        def install_callback(window, handler):
            nonlocal callback
            callback = handler

        def click(action):
            box = next(box for name, _, box in camera_preview.control_buttons(frame.shape) if name == action)
            x, y, width, height = box
            callback(camera_preview.cv2.EVENT_LBUTTONDOWN, x + width // 2,
                     y + height // 2, 0, None)

        def select_target():
            x, y = PreviewLayout(frame.shape).display_point(300, 200)
            callback(camera_preview.cv2.EVENT_LBUTTONDOWN, round(x), round(y), 0, None)

        def record_view(*args, **kwargs):
            views.append((args[3].state, kwargs["auto_status"]))
            return args[0]

        def event(delay):
            nonlocal enabled_frame, acted, after_action
            if scenario == "manual_pulse_edit":
                if clock.frame == 1:
                    click("STROKE_A")
                    return ord("-")
                if clock.frame in (2, 3, 4):
                    return ord("9") if clock.frame == 2 else ord("0")
                if clock.frame == 5:
                    acted = True
                    return 13
            elif scenario in ("manual_toggle_cycle", "manual_toggle_off", "manual_stop_off", "manual_session_off", "manual_live_jog"):
                if clock.frame == 1:
                    click("ARM")
                    self.assertNotIn("ARM", reader.requests)  # Enabling itself sends nothing.
                elif clock.frame == 3:
                    if scenario in ("manual_toggle_off", "manual_stop_off"):
                        click("ARM" if scenario == "manual_toggle_off" else "STOP")
                    elif scenario == "manual_session_off":
                        reader.motor.disconnect(); reader.motor.connect(); reader.pump()
                    else:
                        click("JOG_A_PLUS" if scenario == "manual_live_jog" else "GRAB")
                elif clock.frame == 6:
                    click("NEUTRAL" if scenario == "manual_toggle_cycle" else "GRAB")
                    if scenario != "manual_toggle_cycle":
                        acted = True
                elif clock.frame == 9:
                    click("GRAB")
                elif clock.frame == 12:
                    acted = True
            elif manual:
                if clock.frame == 1 and scenario != "manual_no_arm":
                    click("ARM")
                elif clock.frame == 3:
                    click("GRAB")
                    acted = True
            elif clock.frame == 1:
                select_target()
            elif clock.frame == 2:
                click("ECHO")
                if scenario == "echo_baseline":
                    reader.distance = 9.9  # Still within 2 cm of the confirmed 8 cm baseline.
            elif clock.frame == 4:
                self.assertEqual([c for c in reader.writes if c != "STATUS"], [])
                click("AUTO")
                enabled_frame = clock.frame
            elif scenario in ("stop", "reselect", "cancel_auto_manual_grab") and not acted and reader.motor_snapshot.pending == "ARM":
                acted = True
                if scenario == "cancel_auto_manual_grab":
                    click("GRAB")
                elif scenario == "stop":
                    click("STOP")
                else:
                    select_target()
            elif scenario == "cancel_auto_queued_grab" and not acted and reader.motor_snapshot.pending == "GRAB":
                acted = True
                click("GRAB")
            elif scenario == "range_loss" and not acted and reader.motor_snapshot.pending == "GRAB":
                acted = True
                reader.timeout = True
            elif scenario == "echo_baseline" and not acted and reader.motor_snapshot.pending == "GRAB":
                acted = True
                reader.distance = 10.1  # In capture window, beyond the original echo confirmation.
            elif scenario == "capture" and not acted and "GRAB" in reader.writes:
                acted = True
                frame[:] = 0  # Wrapping/occlusion after actual dispatch.
            if acted:
                after_action += 1
                if after_action >= 5:
                    return ord("q")
            if clock.frame > 30:
                self.fail(f"Scenario did not reach its expected action: {scenario}; {reader.writes}; {views[-3:]}")
            return 255

        capture.read.side_effect = read_frame
        with ExitStack() as stack:
            stack.enter_context(patch.object(camera_preview, "SensorReader", return_value=reader))
            selector = stack.enter_context(patch.object(camera_preview, "GeminiSelector"))
            selector.return_value.poll.return_value = None
            stack.enter_context(patch.object(camera_preview.time, "monotonic", side_effect=clock.now))
            stack.enter_context(patch.object(camera_preview, "draw_view", side_effect=record_view))
            stack.enter_context(patch.object(camera_preview.cv2, "VideoCapture", return_value=capture))
            stack.enter_context(patch.object(camera_preview.cv2, "getWindowImageRect", return_value=(0, 0, *DEFAULT_SIZE)))
            stack.enter_context(patch.object(camera_preview.cv2, "getTrackbarPos", side_effect=lambda name, _: 180 if name == "Threshold" else 80))
            stack.enter_context(patch.object(camera_preview.cv2, "setMouseCallback", side_effect=install_callback))
            stack.enter_context(patch.object(camera_preview.cv2, "waitKey", side_effect=event))
            stack.enter_context(patch.object(camera_preview.cv2, "getWindowProperty", return_value=1))
            for name in ("namedWindow", "resizeWindow", "createTrackbar", "setTrackbarPos", "imshow", "destroyAllWindows"):
                stack.enter_context(patch.object(camera_preview.cv2, name))
            stack.enter_context(redirect_stdout(StringIO()))
            self.assertEqual(camera_preview.run_preview(0, calibration=None if manual else CALIBRATION,
                                                       auto_config=None if manual else AutoCaptureConfig(.3)), 0)
            selector.return_value.submit.assert_not_called()
        if not manual:
            self.assertGreater(enabled_frame, 0)
        self.assertTrue(reader.stopped)
        capture.release.assert_called_once()
        return reader, views

    def test_manual_toggle_persists_across_three_deliberate_moves(self):
        reader, _ = self.run_scene("manual_toggle_cycle")
        self.assertEqual([c for c in reader.writes if c != "STATUS"],
                         ["ARM", "GRAB", "ARM", "NEUTRAL", "ARM", "GRAB"])

    def test_manual_toggle_off_stop_and_reconnect_prevent_further_moves(self):
        for scenario in ("manual_toggle_off", "manual_stop_off", "manual_session_off"):
            reader, _ = self.run_scene(scenario)
            self.assertNotIn("GRAB", reader.writes)

    def test_camera_signed_field_typing_and_apply_never_start_a_motor(self):
        reader, _ = self.run_scene("manual_pulse_edit")
        self.assertEqual([c for c in reader.writes if c != "STATUS"], ["SET_STROKES -900 900"])

    def test_camera_live_adjustment_moves_only_selected_axis_by_100(self):
        reader, _ = self.run_scene("manual_live_jog")
        self.assertEqual([c for c in reader.writes if c != "STATUS"], ["ARM", "JOG A 100"])

    def test_manual_grab_without_target_calibration_range_or_auto_configuration(self):
        reader, views = self.run_scene("manual_fallback")
        self.assertEqual([c for c in reader.writes if c != "STATUS"], ["ARM", "GRAB"])
        self.assertEqual(reader.writes.count("GRAB"), 1)
        self.assertFalse(reader.physical_ready)
        self.assertTrue(all(selection == "unselected" for selection, _ in views))

    def test_manual_grab_still_requires_reference_arm_and_prepared_rest(self):
        for scenario in ("manual_no_reference", "manual_no_arm", "manual_not_rest"):
            reader, _ = self.run_scene(scenario)
            self.assertNotIn("GRAB", reader.writes)

    def test_manual_grab_cancels_auto_arm_without_delayed_second_capture(self):
        for scenario, expected in (("cancel_auto_manual_grab", ["GUARD", "STOP"]),
                                   ("cancel_auto_queued_grab", ["GUARD", "ARM", "STOP"])):
            reader, views = self.run_scene(scenario)
            self.assertEqual([c for c in reader.writes if c != "STATUS"], expected)
            self.assertNotIn("GRAB", reader.writes)
            self.assertTrue(all("Auto OFF" in state for _, state in views[-3:]))

    def test_explicit_auto_click_runs_real_guard_arm_grab_once_and_occlusion_never_releases(self):
        reader, views = self.run_scene("capture")
        self.assertEqual([c for c in reader.writes if c != "STATUS"], ["GUARD", "ARM", "GRAB"])
        self.assertEqual(reader.predicates, [("GUARD", True), ("ARM", True), ("GRAB", True)])
        self.assertIn("lost", [selection for selection, _ in views])
        self.assertTrue(all("Auto FIRED" in state for _, state in views[-3:]))
        self.assertNotIn("NEUTRAL", reader.requests)

    def test_stop_button_cancels_queued_arm_before_worker_can_write_it(self):
        reader, views = self.run_scene("stop")
        self.assertEqual([c for c in reader.writes if c != "STATUS"], ["GUARD", "STOP"])
        self.assertIn("ARM", reader.requests)
        self.assertNotIn("GRAB", reader.requests)
        self.assertTrue(all("Auto OFF" in state for _, state in views[-3:]))

    def test_target_reselection_cancels_queued_arm_and_does_not_resume(self):
        reader, views = self.run_scene("reselect")
        self.assertEqual([c for c in reader.writes if c != "STATUS"], ["GUARD", "STOP"])
        self.assertNotIn("GRAB", reader.requests)
        self.assertTrue(all("Auto OFF" in state for _, state in views[-3:]))

    def test_fresh_timeout_before_grab_write_uses_current_worker_reading_and_stops(self):
        reader, views = self.run_scene("range_loss")
        self.assertEqual([c for c in reader.writes if c != "STATUS"], ["GUARD", "ARM", "STOP"])
        self.assertIn(("GRAB", False), reader.predicates)
        self.assertNotIn("GRAB", reader.writes)
        self.assertTrue(all("Auto OFF" in state for _, state in views[-3:]))

    def test_write_time_range_is_checked_against_original_operator_echo_confirmation(self):
        reader, views = self.run_scene("echo_baseline")
        self.assertEqual([c for c in reader.writes if c != "STATUS"], ["GUARD", "ARM", "STOP"])
        self.assertIn(("GRAB", False), reader.predicates)
        self.assertNotIn("GRAB", reader.writes)
        self.assertTrue(all("Auto OFF" in state for _, state in views[-3:]))


if __name__ == "__main__":
    unittest.main()
