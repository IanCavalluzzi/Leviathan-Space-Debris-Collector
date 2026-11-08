"""Synthetic one-shot capture checks; no controller/camera or cloud access."""

from dataclasses import replace
import json
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

from auto_capture import AutoCapture, AutoCaptureConfig, CaptureContext, load_auto_capture_config
from motor_control import PAPER_PROFILE, MotorSafety, MotorSnapshot, MotorStatus
from operator_guidance import Calibration
from sensor_telemetry import parse_line


CALIBRATION = Calibration(640, 480, (300, 200), (10, 15), 1, 1, True, (8, 12))


def context(now=1.0, *, guard=False, armed=False, pending=None, local=None,
            center=(300, 200), distance=10, sample_id=None, target=1):
    sample_id = round(now * 1000) if sample_id is None else sample_id
    reading = parse_line(f"{sample_id},580,OK,{distance},0,0,0,0,0", now)
    status = MotorStatus("NEUTRAL", True, armed, True, 900, 0, "OK", 0, 0, -1, now)
    motor = MotorSnapshot(connected=True, status=status, synchronized=True, session_id=1, profile=PAPER_PROFILE, controls=True,
                          safety=MotorSafety(True, guard, 1500, 10000, now),
                          pending=pending, locally_armed=armed if local is None else local)
    return CaptureContext(CALIBRATION, (480, 640, 3), now, target, center, True,
                          reading, True, motor)


def arm_request(auto):
    """Reach a single ARM request after guard ACK and three dwell samples."""
    assert auto.enable(context(), 1.0)
    assert auto.tick(context(), 1.0) == "GUARD"
    assert auto.write_allowed("GUARD", 1.0, auto.token)
    auto.command_result("GUARD", True)
    assert auto.tick(context(1.1, guard=True), 1.1) is None
    assert auto.tick(context(1.3, guard=True), 1.3) is None
    assert auto.tick(context(1.6, guard=True), 1.6) == "ARM"


def grab_request(auto):
    arm_request(auto)
    assert auto.write_allowed("ARM", 1.6, auto.token)
    auto.command_result("ARM", True)
    assert auto.tick(context(1.7, guard=True, armed=True), 1.7) == "GRAB"


class ConfigTests(unittest.TestCase):
    def test_explicit_configuration_and_exact_json_contract(self):
        auto = AutoCapture()
        self.assertFalse(auto.enable(context(), 1))
        self.assertIsNone(auto.tick(context(), 1))
        with TemporaryDirectory() as folder:
            path = Path(folder) / "synthetic.json"
            path.write_text(json.dumps({"dwell_seconds": .5}))
            self.assertEqual(load_auto_capture_config(path), AutoCaptureConfig(.5))
            for data in ({}, {"dwell_seconds": .5, "commissioned": True}, [], None):
                path.write_text(json.dumps(data))
                with self.assertRaises(ValueError):
                    load_auto_capture_config(path)
            with self.assertRaises(ValueError):
                load_auto_capture_config(Path(folder) / "absent.json")

    def test_invalid_dwell_cannot_enable(self):
        for value in (True, None, 0, -1, 5.01, float("nan"), float("inf"), "0.5"):
            with self.subTest(value=value), self.assertRaises(ValueError):
                AutoCaptureConfig(value)
        self.assertEqual(AutoCaptureConfig(5).dwell_seconds, 5)


class EnableGateTests(unittest.TestCase):
    def test_physical_zero_or_missing_profile_cannot_enable_auto(self):
        good = context()
        for motor in (replace(good.motor, profile=None),
                      replace(good.motor, status=replace(good.motor.status, state="POSITIONED",
                                                        position_a=0, position_b=0))):
            auto = AutoCapture(AutoCaptureConfig(.5))
            self.assertFalse(auto.enable(replace(good, motor=motor), 1))
            self.assertEqual(auto.state, "OFF")

    def test_no_startup_commands_and_no_double_enable(self):
        auto = AutoCapture(AutoCaptureConfig(.5))
        self.assertIsNone(auto.tick(context(), 1))
        self.assertTrue(auto.enable(context(), 1))
        token = auto.token
        self.assertFalse(auto.enable(context(), 1))
        self.assertEqual(auto.token, token)

    def test_all_measurement_and_controller_enable_gates(self):
        good = context()
        invalid = [
            replace(good, calibration=None),
            replace(good, calibration=replace(CALIBRATION, axes_verified=False)),
            replace(good, calibration=replace(CALIBRATION, range_window_cm=None)),
            replace(good, frame_shape=(720, 1280, 3)),
            replace(good, frame_received_at=-1),
            replace(good, frame_received_at=2),
            replace(good, tracking=False),
            replace(good, target_number=None),
            replace(good, target_center=(float("nan"), 200)),
            replace(good, target_center=(-1, 200)),
            replace(good, correspondence=False),
            replace(good, reading=None),
            replace(good, reading=parse_line("1000,0,TIMEOUT,NA,0,0,0,0,0", 1)),
            replace(good, reading=replace(good.reading, received_at=-1)),
            replace(good, motor=replace(good.motor, connected=False)),
            replace(good, motor=replace(good.motor, synchronized=False)),
            replace(good, motor=replace(good.motor, pending="JOG A 100")),
            replace(good, motor=replace(good.motor, session_id=0)),
            replace(good, motor=replace(good.motor, status=replace(good.motor.status, configured=False))),
            replace(good, motor=replace(good.motor, status=replace(good.motor.status, referenced=False))),
            replace(good, motor=replace(good.motor, status=replace(good.motor.status, fault_axis=0))),
            context(armed=True),
        ]
        for item in invalid:
            with self.subTest(context=item):
                auto = AutoCapture(AutoCaptureConfig(.5))
                self.assertFalse(auto.enable(item, 1))
                self.assertIsNone(auto.tick(item, 1))

    def test_configured_unloaded_override_does_not_qualify_physical_ready(self):
        good = context()
        for safety in (None, replace(good.motor.safety, physical_ready=False),
                       replace(good.motor.safety, guard_timeout_ms=0),
                       replace(good.motor.safety, arm_timeout_ms=0),
                       replace(good.motor.safety, received_at=-1)):
            auto = AutoCapture(AutoCaptureConfig(.5))
            self.assertFalse(auto.enable(replace(good, motor=replace(good.motor, safety=safety)), 1))
            self.assertIn("physical profile", auto.message)


class OneShotTests(unittest.TestCase):
    def setUp(self):
        self.auto = AutoCapture(AutoCaptureConfig(.5))

    def test_guard_requires_ack_state_then_dwell_and_local_arm_ack(self):
        auto = self.auto
        self.assertTrue(auto.enable(context(), 1))
        self.assertEqual(auto.tick(context(), 1), "GUARD")
        auto.command_result("GUARD", True)
        self.assertIsNone(auto.tick(context(1.1, guard=True, pending="GUARD"), 1.1))
        self.assertEqual(auto.state, "GUARDING")
        self.assertIsNone(auto.tick(context(1.2, guard=True), 1.2))
        self.assertEqual(auto.state, "WAITING")
        self.assertIsNone(auto.tick(context(1.4, guard=True), 1.4))
        self.assertEqual(auto.tick(context(1.8, guard=True), 1.8), "ARM")
        auto.command_result("ARM", True)
        self.assertIsNone(auto.tick(context(1.9, guard=True, armed=True, local=False, pending="ARM"), 1.9))
        self.assertEqual(auto.tick(context(2, guard=True, armed=True), 2), "GRAB")

    def test_completed_guard_without_guard_active_cancels_without_retry(self):
        self.auto.enable(context(), 1)
        self.auto.tick(context(), 1)
        self.auto.command_result("GUARD", True)
        self.assertIsNone(self.auto.tick(context(1.1, guard=False), 1.1))
        self.assertEqual(self.auto.state, "OFF")
        self.assertIsNone(self.auto.tick(context(1.2, guard=True), 1.2))

    def test_completed_arm_without_local_ack_cancels_with_stop(self):
        arm_request(self.auto)
        self.auto.command_result("ARM", True)
        self.assertEqual(self.auto.tick(context(1.7, guard=True, armed=True, local=False), 1.7), "STOP")
        self.assertEqual(self.auto.state, "OFF")

    def test_one_grab_even_after_complete_occlusion_loss_or_reconnect(self):
        auto = self.auto
        grab_request(auto)
        token = auto.token
        self.assertTrue(auto.write_allowed("GRAB", 1.7, token))
        auto.command_result("GRAB", True)
        self.assertFalse(auto.write_allowed("GRAB", 1.7, token))
        for item in (replace(context(1.8), tracking=False, correspondence=False),
                     replace(context(1.9), motor=replace(context(1.9).motor, connected=False)),
                     replace(context(2), motor=replace(context(2).motor, session_id=2)),
                     context(3)):
            self.assertIsNone(auto.tick(item, item.frame_received_at))
        self.assertEqual(auto.state, "FIRED")
        self.assertFalse(auto.cancel("Explicit disable after capture"))

    def test_dwell_requires_distinct_range_samples_not_repeated_video_frames(self):
        auto = self.auto
        auto.enable(context(), 1)
        auto.tick(context(), 1)
        auto.command_result("GUARD", True)
        for now in (1.1, 1.3, 1.6, 1.8, 2):
            self.assertIsNone(auto.tick(context(now, guard=True, sample_id=1100), now))
        self.assertEqual(auto.state, "WAITING")
        self.assertIsNone(auto.tick(context(2.1, guard=True, sample_id=2100), 2.1))
        self.assertEqual(auto.tick(context(2.3, guard=True, sample_id=2300), 2.3), "ARM")

    def test_dwell_requires_sample_time_span_and_continuous_in_window(self):
        auto = self.auto
        auto.enable(context(), 1)
        auto.tick(context(), 1)
        auto.command_result("GUARD", True)
        auto.tick(context(1.1, guard=True), 1.1)
        auto.tick(context(1.3, guard=True), 1.3)
        self.assertIsNone(auto.tick(context(1.4, guard=True, center=(320, 200)), 1.4))
        self.assertEqual(auto.state, "WAITING")
        self.assertIsNone(auto.tick(context(1.5, guard=True), 1.5))
        self.assertIsNone(auto.tick(context(1.7, guard=True), 1.7))
        self.assertEqual(auto.tick(context(2, guard=True), 2), "ARM")

    def test_outside_range_waits_without_arming_then_qualifies_boundaries(self):
        auto = self.auto
        auto.enable(context(distance=20), 1)
        self.assertEqual(auto.tick(context(distance=20), 1), "GUARD")
        auto.command_result("GUARD", True)
        self.assertIsNone(auto.tick(context(1.1, guard=True, distance=20), 1.1))
        self.assertEqual(auto.state, "WAITING")
        auto.tick(context(1.2, guard=True, center=(310, 215), distance=8), 1.2)
        auto.tick(context(1.4, guard=True, distance=12), 1.4)
        self.assertEqual(auto.tick(context(1.8, guard=True, distance=10), 1.8), "ARM")

    def test_hard_invalidations_consume_enable_without_auto_resume(self):
        for alter in (
            lambda c: replace(c, tracking=False),
            lambda c: replace(c, correspondence=False),
            lambda c: replace(c, target_number=2),
            lambda c: replace(c, calibration=replace(CALIBRATION, crosshair=(301, 200))),
            lambda c: replace(c, motor=replace(c.motor, session_id=2)),
            lambda c: replace(c, motor=replace(c.motor, safety=replace(c.motor.safety, guard_active=False))),
        ):
            auto = AutoCapture(AutoCaptureConfig(.5))
            auto.enable(context(), 1)
            auto.tick(context(), 1)
            auto.command_result("GUARD", True)
            auto.tick(context(1.1, guard=True), 1.1)
            self.assertIsNone(auto.tick(alter(context(1.2, guard=True)), 1.2))
            self.assertEqual(auto.state, "OFF")
            self.assertIsNone(auto.tick(context(1.3, guard=True), 1.3))

    def test_cancel_after_arm_or_queued_grab_requires_one_priority_stop(self):
        for stage in (arm_request, grab_request):
            auto = AutoCapture(AutoCaptureConfig(.5))
            stage(auto)
            self.assertTrue(auto.cancel("Operator changed selection"))
            self.assertFalse(auto.cancel("Still canceled"))
            self.assertIsNone(auto.tick(context(2, guard=True), 2))

    def test_window_loss_while_arming_returns_stop_not_grab(self):
        arm_request(self.auto)
        self.auto.command_result("ARM", True)
        self.assertEqual(self.auto.tick(context(1.7, guard=True, armed=True, distance=15), 1.7), "STOP")
        self.assertIsNone(self.auto.tick(context(1.8, guard=True, armed=True), 1.8))

    def test_denied_grab_does_not_retry_and_requests_stop(self):
        grab_request(self.auto)
        self.auto.command_result("GRAB", False)
        self.assertEqual(self.auto.tick(context(1.8, guard=True, armed=True), 1.8), "STOP")
        self.assertIsNone(self.auto.tick(context(2, guard=True, armed=True), 2))

    def test_no_guard_retry_when_request_denied(self):
        self.auto.enable(context(), 1)
        self.auto.tick(context(), 1)
        self.auto.command_result("GUARD", False)
        self.assertIsNone(self.auto.tick(context(1.1), 1.1))
        self.assertEqual(self.auto.state, "OFF")

    def test_write_time_gate_refuses_stale_or_newly_lost_target(self):
        for change in (lambda c: replace(c, tracking=False), lambda c: replace(c, correspondence=False)):
            auto = AutoCapture(AutoCaptureConfig(.5))
            grab_request(auto)
            # Refresh context after the GRAB was queued, before its worker write.
            self.assertEqual(auto.tick(change(context(1.8, guard=True, armed=True, pending="GRAB", local=False)), 1.8), "STOP")
            self.assertFalse(auto.write_allowed("GRAB", 1.8, auto.token))
        grab_request(self.auto)
        self.assertFalse(self.auto.write_allowed("GRAB", 3, self.auto.token))
        self.assertEqual(self.auto.tick(context(3, guard=True, armed=True), 3), "STOP")

    def test_write_time_fresh_context_tolerates_consumed_local_entitlement(self):
        grab_request(self.auto)
        self.auto.command_result("GRAB", True)
        self.assertIsNone(self.auto.tick(context(1.8, guard=True, armed=True, pending="GRAB", local=False), 1.8))
        self.assertTrue(self.auto.write_allowed("GRAB", 1.8, self.auto.token))

    def test_current_worker_motor_snapshot_supersedes_cached_frame_status(self):
        for change in (
            lambda m: None,
            lambda m: replace(m, connected=False),
            lambda m: replace(m, session_id=2),
            lambda m: replace(m, safety=replace(m.safety, physical_ready=False)),
            lambda m: replace(m, safety=replace(m.safety, guard_active=False)),
            lambda m: replace(m, status=replace(m.status, armed=False)),
        ):
            auto = AutoCapture(AutoCaptureConfig(.5))
            grab_request(auto)
            current = change(context(1.8, guard=True, armed=True).motor)
            self.assertFalse(auto.write_allowed("GRAB", 1.8, auto.token, motor=current))
            self.assertEqual(auto.state, "OFF")

    def test_current_worker_range_including_none_supersedes_cached_valid_range(self):
        for reading in (None, context(1.8, distance=20).reading,
                        parse_line("1800,0,TIMEOUT,NA,0,0,0,0,0", 1.8)):
            auto = AutoCapture(AutoCaptureConfig(.5))
            grab_request(auto)
            self.assertFalse(auto.write_allowed("GRAB", 1.8, auto.token, reading=reading))
            self.assertEqual(auto.state, "OFF")
        auto = AutoCapture(AutoCaptureConfig(.5))
        grab_request(auto)
        current = context(1.8, guard=True, armed=True)
        self.assertTrue(auto.write_allowed("GRAB", 1.8, auto.token,
                                           motor=current.motor, reading=current.reading))

    def test_worker_stop_ack_suppresses_module_duplicate_stop(self):
        grab_request(self.auto)
        self.assertFalse(self.auto.write_allowed("GRAB", 1.8, self.auto.token, reading=None))
        stopped = context(1.9)
        stopped = replace(stopped, motor=replace(stopped.motor,
                          status=replace(stopped.motor.status, result="STOPPED")))
        self.assertIsNone(self.auto.tick(stopped, 1.9))
        self.assertEqual(self.auto.state, "OFF")

    def test_pending_stale_unknown_or_prearm_disarm_does_not_suppress_stop(self):
        good = context(1.9)
        invalid = [replace(good, motor=replace(good.motor, pending=command))
                   for command in ("ARM", "GRAB", "STOP")]
        invalid.extend([
            replace(good, motor=replace(good.motor, connected=False)),
            replace(good, motor=replace(good.motor, synchronized=False)),
            replace(good, motor=replace(good.motor, status=replace(good.motor.status, received_at=0))),
            replace(good, motor=replace(good.motor, status=replace(good.motor.status, received_at=1.5))),
            replace(good, motor=replace(good.motor, status=replace(good.motor.status, state="GRABBING"))),
        ])
        for item in invalid:
            auto = AutoCapture(AutoCaptureConfig(.5))
            grab_request(auto)
            self.assertFalse(auto.write_allowed("GRAB", 1.8, auto.token, reading=None))
            self.assertEqual(auto.tick(item, 1.9), "STOP")

    def test_old_cycle_predicate_cannot_authorize_reenabled_command(self):
        auto = self.auto
        auto.enable(context(), 1)
        self.assertEqual(auto.tick(context(), 1), "GUARD")
        old_token = auto.token
        self.assertFalse(auto.cancel("New cycle requested"))
        self.assertTrue(auto.enable(context(1.1), 1.1))
        self.assertEqual(auto.tick(context(1.1), 1.1), "GUARD")
        self.assertFalse(auto.write_allowed("GUARD", 1.1, old_token))
        self.assertTrue(auto.write_allowed("GUARD", 1.1, auto.token))

    def test_explicit_new_enable_after_return_can_select_another_target(self):
        grab_request(self.auto)
        self.assertTrue(self.auto.write_allowed("GRAB", 1.7, self.auto.token))
        self.assertTrue(self.auto.enable(context(2, target=2), 2))
        self.assertEqual(self.auto.tick(context(2, target=2), 2), "GUARD")

    def test_expired_arm_fault_and_backward_clock_cancel(self):
        arm_request(self.auto)
        item = context(1.7, guard=True)
        item = replace(item, motor=replace(item.motor, status=replace(item.motor.status, result="ARM_EXPIRED")))
        self.assertIsNone(self.auto.tick(item, 1.7))  # New state already proves ARM expired/disarmed.
        auto = AutoCapture(AutoCaptureConfig(.5))
        arm_request(auto)
        self.assertEqual(auto.tick(context(1.5, guard=True), 1.5), "STOP")


if __name__ == "__main__":
    unittest.main()
