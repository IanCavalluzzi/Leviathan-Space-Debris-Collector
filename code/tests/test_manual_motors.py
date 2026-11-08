"""Camera-free manual controls exercise the actual shared command sequencer."""

from dataclasses import replace
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import cv2

from manual_motors import ManualControl, SIZE, buttons, connection_lines, control_at, draw_panel, run_manual
from motor_control import PAPER_PROFILE, MotorControls, MotorLimits, MotorLink, MotorReply, MotorStatus


def state(now=1.0, armed=False, referenced=True, configured=True, name="NEUTRAL",
          result="OK", job_id=0, job=0):
    return MotorStatus(name, configured, armed, referenced, 0, 0, result, job_id, job, -1, now)


class Reader:
    def __init__(self, status=None):
        self.motor = MotorLink()
        self.snapshot = SimpleNamespace(port="COM22")
        self.requested_port = "auto"
        self.commands = []
        self.now = 1.0
        self.started = self.stopped = False
        self.motor.connect()
        self.complete("STATUS", status or state())
        self.motor.receive(PAPER_PROFILE, self.now)
        self.motor.receive(MotorControls(), self.now)

    @property
    def motor_snapshot(self):
        return self.motor.snapshot

    def command(self, command):
        self.commands.append(command)
        return self.motor.request(command, self.now)

    def complete(self, command, status, error=None):
        assert self.motor.next_write(self.now) == (command + "\n").encode()
        self.motor.receive(MotorReply("ERR" if error else "ACK", error or "OK", status.job_id), self.now)
        self.motor.receive(status, self.now)

    def start(self):
        self.started = True

    def stop(self):
        self.stopped = True


class ManualMotorTests(unittest.TestCase):
    def test_retracted_and_neutral_references_have_separate_confirmations(self):
        reader = Reader(state(referenced=False, name="UNREFERENCED"))
        control = ManualControl(reader)
        self.assertFalse(control.reference(1))
        self.assertFalse(control.reference(1, retracted=True))
        self.assertEqual(reader.commands, [])
        self.assertTrue(control.reference(1.1, retracted=True))
        self.assertEqual(reader.commands, ["REFERENCE_REST"])
        self.assertIsNone(control.after_arm)

    def test_signed_fields_apply_without_arming_or_motion_and_validate_bounds(self):
        reader = Reader()
        control = ManualControl(reader)
        control.action("STROKE_A", 1)
        for key in map(ord, "-900"):
            control.stroke_key(key, 1)
        self.assertEqual(reader.commands, [])
        control.stroke_key(13, 1)
        self.assertEqual(reader.commands, ["SET_STROKES -900 900"])
        self.assertIsNone(control.after_arm)
        for invalid in ("0", "2501", "-2501", "+", "1.5"):
            other = Reader()
            ui = ManualControl(other)
            ui.stroke_entries["A"] = invalid
            self.assertFalse(ui.apply_strokes(1))
            self.assertEqual(other.commands, [])

    def test_live_100_adjustment_uses_existing_bounded_arm_handshake(self):
        reader = Reader()
        reader.motor.receive(MotorLimits(-2500, 2500, 2500, -2500, 2500, 2500), 1)
        control = ManualControl(reader)
        control.action("JOG_B_MINUS", 1)
        self.assertEqual(reader.commands, ["ARM"])
        reader.complete("ARM", state(armed=True))
        control.tick(1)
        self.assertEqual(reader.commands, ["ARM", "JOG B -100"])

    def test_prepare_grab_and_return_each_wait_for_their_own_arm_ack(self):
        reader = Reader(state(name="POSITIONED"))
        control = ManualControl(reader)
        self.assertFalse(control.move("GRAB", 1))
        self.assertEqual(reader.commands, [])
        for command, start, end, a, b, job in (
                ("NEUTRAL", "POSITIONED", "NEUTRAL", 900, 0, 1),
                ("GRAB", "NEUTRAL", "HOLD_UNCONFIRMED", 0, 900, 2),
                ("NEUTRAL", "HOLD_UNCONFIRMED", "NEUTRAL", 900, 0, 3)):
            self.assertTrue(control.move(command, 1))
            self.assertFalse(control.move(command, 1))
            self.assertEqual(reader.commands[-1], "ARM")
            reader.complete("ARM", state(armed=True, name=start, job_id=job - 1))
            control.tick(1)
            self.assertEqual(reader.commands[-1], command)
            reader.complete(command, replace(state(name=end, job_id=job, result="COMPLETE"),
                                             position_a=a, position_b=b))
        self.assertEqual(reader.commands, ["ARM", "NEUTRAL", "ARM", "GRAB", "ARM", "NEUTRAL"])
        self.assertFalse(control.move("NEUTRAL", 1))

    def test_missing_profile_blocks_coordinated_buttons_and_stop_cancels_pending_job(self):
        reader = Reader(state(name="POSITIONED"))
        control = ManualControl(reader)
        reader.motor._update(profile=None)
        self.assertFalse(control.move("NEUTRAL", 1))
        self.assertEqual(reader.commands, [])
        reader.motor.receive(PAPER_PROFILE, 1)
        self.assertTrue(control.move("NEUTRAL", 1))
        control.stop()
        control.tick(1)
        self.assertEqual(reader.commands, ["ARM", "STOP"])
        self.assertIsNone(control.after_arm)

    def test_custom_count_typing_and_enter_never_move_a_motor(self):
        reader = Reader()
        control = ManualControl(reader)
        control.action("PULSES", 1)
        for key in map(ord, "600"):
            control.edit_key(key)
        control.edit_key(13)
        self.assertEqual(control.steps, 600)
        self.assertFalse(control.editing)
        self.assertEqual(reader.commands, [])

    def test_typing_in_the_live_field_does_not_trigger_numeric_jog_shortcuts(self):
        reader = Reader()
        control = ManualControl(reader)
        keys = iter([ord("1"), ord("0"), ord("0"), 13, ord("q")])
        callbacks = []
        clicked = [False]

        def key(_):
            if not clicked[0]:
                callbacks[0](cv2.EVENT_LBUTTONDOWN, 550, 230, 0, None)
                clicked[0] = True
            return next(keys)

        with patch("manual_motors.SensorReader", return_value=reader), \
                patch("manual_motors.ManualControl", return_value=control), \
                patch("manual_motors.cv2.namedWindow"), \
                patch("manual_motors.cv2.setMouseCallback", side_effect=lambda name, cb: callbacks.append(cb)), \
                patch("manual_motors.cv2.imshow"), \
                patch("manual_motors.cv2.waitKey", side_effect=key), \
                patch("manual_motors.cv2.getWindowProperty", return_value=1), \
                patch("manual_motors.cv2.destroyWindow"):
            self.assertEqual(run_manual(), 0)
        self.assertEqual(control.steps, 100)
        self.assertEqual(reader.commands, ["STOP"])

    def test_custom_jog_waits_for_arm_and_respects_reported_limits(self):
        reader = Reader()
        reader.motor.receive(MotorLimits(-2500, 2500, 2500, -2500, 2500, 2500), 1)
        control = ManualControl(reader)
        control.entry = "600"
        control.apply_entry()
        control.action("B-", 1)
        self.assertEqual(reader.commands, ["ARM"])
        reader.complete("ARM", state(armed=True))
        control.tick(1)
        self.assertEqual(reader.commands, ["ARM", "JOG B -600"])

    def test_old_firmware_does_not_silently_accept_a_large_custom_jog(self):
        reader = Reader()
        control = ManualControl(reader)
        self.assertFalse(control.jog("A", 100, 1))
        self.assertIn("Upload updated grip_control", control.notice)
        self.assertEqual(reader.commands, [])

    def test_position_and_individual_jog_caps_are_checked_before_arm(self):
        reader = Reader()
        reader.motor.receive(MotorLimits(-2500, 2500, 2500, -2500, 2500, 100), 1)
        reader.motor.receive(replace(state(), position_a=2490), 1)
        control = ManualControl(reader)
        self.assertFalse(control.jog("A", 20, 1))
        self.assertFalse(control.jog("B", -101, 1))
        self.assertEqual(reader.commands, [])
        self.assertTrue(control.jog("A", 10, 1))

    def test_invalid_custom_count_does_not_replace_count_or_move(self):
        reader = Reader()
        control = ManualControl(reader)
        for value in ("", "0", "-1", "2501", "1.5", "ARM"):
            control.entry = value
            self.assertFalse(control.apply_entry())
            self.assertEqual(control.steps, 10)
        self.assertEqual(reader.commands, [])

    def test_connection_messages_distinguish_missing_usb_access_and_firmware(self):
        reader = Reader()
        reader.motor.disconnect()
        for state_name, expected in (("not_found", "not detected"), ("access_denied", "access denied"),
                                     ("reply_timeout", "timed out"), ("waiting", "waiting for controller")):
            reader.snapshot = SimpleNamespace(port="COM22", state=state_name)
            title, _ = connection_lines(reader, reader.motor_snapshot, 1)
            self.assertIn(expected, title)

    def test_constructing_controls_never_references_arms_or_moves(self):
        reader = Reader()
        control = ManualControl(reader)
        control.tick(1)
        self.assertEqual(reader.commands, [])
        self.assertIsNone(control.after_arm)

    def test_one_click_waits_for_arm_then_sends_one_finite_jog(self):
        reader = Reader()
        control = ManualControl(reader)
        self.assertTrue(control.jog("A", 10, 1))
        control.tick(1)
        self.assertEqual(reader.commands, ["ARM"])
        self.assertFalse(control.jog("B", -10, 1))  # Never queue another move.
        reader.complete("ARM", state(armed=True))
        control.tick(1)
        self.assertEqual(reader.commands, ["ARM", "JOG A 10"])
        reader.complete("JOG A 10", state(name="JOGGING", result="ACCEPTED", job_id=1, job=1))
        control.tick(1)
        self.assertEqual(reader.commands, ["ARM", "JOG A 10"])
        self.assertIsNone(control.after_arm)

    def test_signed_b_jog_uses_same_controller_and_one_arm(self):
        reader = Reader()
        control = ManualControl(reader)
        self.assertTrue(control.jog("B", -5, 1))
        reader.complete("ARM", state(armed=True))
        control.tick(1)
        self.assertEqual(reader.commands, ["ARM", "JOG B -5"])

    def test_stop_cancels_unsent_jog_without_closing_controls(self):
        reader = Reader()
        control = ManualControl(reader)
        control.jog("A", 10, 1)
        control.stop()
        self.assertEqual(reader.motor.next_write(1), b"STOP\n")
        self.assertIsNone(control.after_arm)
        control.tick(1)
        self.assertNotIn("JOG A 10", reader.commands)
        self.assertFalse(reader.stopped)

    def test_disconnect_during_arm_does_not_replay_a_jog_after_reconnect(self):
        reader = Reader()
        control = ManualControl(reader)
        control.jog("A", 10, 1)
        reader.motor.disconnect()
        control.tick(1)
        reader.motor.connect()
        reader.complete("STATUS", state(armed=True))
        control.tick(1)
        self.assertEqual(reader.commands, ["ARM"])
        self.assertIsNone(control.after_arm)
        self.assertFalse(reader.motor_snapshot.locally_armed)

    def test_rejected_arm_never_sends_jog(self):
        reader = Reader()
        control = ManualControl(reader)
        control.jog("A", 10, 1)
        reader.complete("ARM", state(referenced=False), error="NO_REFERENCE")
        control.tick(1)
        self.assertEqual(reader.commands, ["ARM"])
        self.assertIsNone(control.after_arm)
        self.assertIn("no movement", control.notice)

    def test_unconfigured_unreferenced_stale_and_moving_controllers_do_not_jog(self):
        for status in (state(configured=False), state(referenced=False),
                       state(now=0), state(name="JOGGING", result="ACCEPTED", job=1)):
            with self.subTest(status=status):
                reader = Reader(status)
                control = ManualControl(reader)
                self.assertFalse(control.jog("A", 10, 1.1))
                self.assertEqual(reader.commands, [])

    def test_physical_reference_needs_two_clicks_and_idle_disarmed_status(self):
        reader = Reader(state(referenced=False))
        control = ManualControl(reader)
        self.assertFalse(control.reference(1))
        self.assertEqual(reader.commands, [])
        self.assertTrue(control.reference(1.1))
        self.assertEqual(reader.commands, ["REFERENCE_NEUTRAL"])
        armed = Reader(state(armed=True))
        self.assertFalse(ManualControl(armed).reference(1))
        self.assertEqual(armed.commands, [])

    def test_jog_click_during_background_poll_is_not_dropped(self):
        reader = Reader()
        reader.now = 1.4
        self.assertEqual(reader.motor.next_write(1.4), b"STATUS\n")
        control = ManualControl(reader)
        self.assertTrue(control.jog("B", -10, 1.4))
        control.tick(1.4)
        self.assertEqual(reader.commands, ["ARM"])
        reader.motor.receive(MotorReply("ACK", "OK", 0), 1.4)
        reader.motor.receive(state(now=1.4), 1.4)
        reader.complete("ARM", state(now=1.4, armed=True))
        control.tick(1.4)
        self.assertEqual(reader.commands, ["ARM", "JOG B -10"])

    def test_reference_confirmation_expires(self):
        reader = Reader(state(referenced=False))
        control = ManualControl(reader)
        control.reference(1)
        reader.motor.receive(state(now=7, referenced=False), 7)
        self.assertFalse(control.reference(7))
        self.assertEqual(reader.commands, [])

    def test_attached_jogs_keep_existing_small_counts(self):
        reader = Reader()
        control = ManualControl(reader)
        for axis, amount in (("C", 1), ("A", 0), ("A", 11), ("B", -100), ("A", True)):
            self.assertFalse(control.jog(axis, amount, 1))
        self.assertEqual(reader.commands, [])
        for count in (1, 5, 10):
            control.action(f"STEP{count}", 1)
            self.assertEqual(control.steps, count)

    def test_controller_limit_error_is_displayed_without_retry(self):
        reader = Reader()
        control = ManualControl(reader)
        control.jog("A", 10, 1)
        reader.complete("ARM", state(armed=True))
        control.tick(1)
        reader.complete("JOG A 10", state(), error="TRAVEL_LIMIT")
        control.tick(1)
        self.assertIn("TRAVEL_LIMIT", reader.motor_snapshot.message)
        self.assertEqual(reader.commands, ["ARM", "JOG A 10"])
        self.assertEqual(draw_panel(control, 1).shape, (SIZE[1], SIZE[0], 3))

    def test_buttons_map_to_finite_actions_and_padding_is_inactive(self):
        control = ManualControl(Reader())
        for action, _, (x, y, w, h) in buttons(control, 1):
            self.assertEqual(control_at(x + w // 2, y + h // 2, control, 1), action)
        self.assertIsNone(control_at(5, 5, control, 1))
        self.assertIsNone(control_at(450, 250, control, 1))

    def test_space_stops_but_only_q_closes_window(self):
        reader = Reader()
        with patch("manual_motors.SensorReader", return_value=reader), \
                patch("manual_motors.cv2.namedWindow"), \
                patch("manual_motors.cv2.setMouseCallback"), \
                patch("manual_motors.cv2.imshow") as draw, \
                patch("manual_motors.cv2.waitKey", side_effect=[ord(" "), ord("q")]), \
                patch("manual_motors.cv2.getWindowProperty", return_value=1), \
                patch("manual_motors.cv2.destroyWindow"):
            self.assertEqual(run_manual(), 0)
        self.assertEqual(draw.call_count, 2)
        self.assertTrue(reader.started)
        self.assertTrue(reader.stopped)
        self.assertEqual(reader.commands, ["STOP", "STOP"])


if __name__ == "__main__":
    unittest.main()
