"""Real host classes across fragmented production wire fixtures; no devices.

These synthetic positions/calibration qualify software flow only. Explicit ACK,
SAFETY, state and sensor fixtures are not a second motor-controller model.
"""

import unittest

from auto_capture import AutoCapture, AutoCaptureConfig, CaptureContext
from motor_control import MotorLink
from operator_guidance import Calibration
from sensor_telemetry import LineDecoder, SensorReading


CALIBRATION = Calibration(640, 480, (300, 200), (10, 15), 1, 1, True, (8, 12))


class HostWire:
    """A finite in-memory transport around the actual decoder and sequencer."""

    def __init__(self, *, physical_ready=True, referenced=True):
        self.now = 1.0
        self.link = MotorLink()
        self.decoder = LineDecoder(include_motor=True)
        self.auto = AutoCapture(AutoCaptureConfig(.5))
        self.writes = []
        self.reading = None
        self.frame_at = self.now
        self.tracking = True
        self.correspondence = True
        self.physical_ready = physical_ready
        self.guard = False
        self.armed = False
        self.referenced = referenced
        self.state = "NEUTRAL" if referenced else "UNREFERENCED"
        self.result = "OK"
        self.job_id = self.job = 0
        self.link.connect()
        assert self.write() == b"STATUS\n"
        self.reply(1.0)

    def feed(self, text):
        data = text.encode("ascii")
        # Split inside tags, numbers and newlines to exercise actual buffering.
        sizes = (1, 7, 2, 13)
        offset = turn = 0
        while offset < len(data):
            size = sizes[turn % len(sizes)]
            for event in self.decoder.feed(data[offset:offset + size], self.now):
                if isinstance(event, SensorReading):
                    self.reading = event
                else:
                    self.link.receive(event, self.now)
            offset += size
            turn += 1

    def sample(self, now, sample_id=None, *, timeout=False):
        self.now = now
        sample_id = round(now * 1000) if sample_id is None else sample_id
        payload = "0,TIMEOUT,NA" if timeout else "580,OK,10.0"
        self.feed(f"{sample_id},{payload},0,0,0,0,0\n")

    def reply(self, now, *, guard=None, armed=None, referenced=None, state=None,
              result=None, job_id=None, job=None, physical_ready=None):
        self.now = now
        for name, value in (("guard", guard), ("armed", armed), ("referenced", referenced),
                            ("state", state), ("result", result), ("job_id", job_id),
                            ("job", job), ("physical_ready", physical_ready)):
            if value is not None:
                setattr(self, name, value)
        self.feed(
            f"# motor ACK OK {self.job_id}\n"
            f"# motor SAFETY {int(self.physical_ready)} {int(self.guard)} 1500 10000\n"
            "# motor LIMITS -2500 2500 2500 -2500 2500 2500\n"
            "# motor CONTROLS 1\n# motor PROFILE 0 0 900 0 0 900\n"
            f"# motor {self.state} 1 {int(self.armed)} {int(self.referenced)} "
            f"0 0 {self.result} {self.job_id} {self.job} -1\n"
        )
        self.sample(now)

    def context(self, now=None):
        if now is not None:
            self.now = self.frame_at = now
        return CaptureContext(CALIBRATION, (480, 640, 3), self.frame_at, 1,
                              (300, 200) if self.tracking else None, self.tracking,
                              self.reading, self.correspondence, self.link.snapshot)

    def tick(self, now):
        return self.auto.tick(self.context(now), now)

    def queue(self, command):
        token = self.auto.token
        accepted = self.link.request(
            command, self.now,
            condition=lambda: self.auto.write_allowed(
                command, self.now, token, motor=self.link.snapshot, reading=self.reading),
        )
        self.auto.command_result(command, accepted)
        return accepted

    def write(self, now=None):
        if now is not None:
            self.now = now
        outgoing = self.link.next_write(self.now)
        if outgoing is not None:
            self.writes.append(outgoing)
        return outgoing

    def request_grab(self):
        """Actual guard/arm ACKs, three distinct samples and a pending poll."""
        assert self.auto.enable(self.context(1.0), 1.0)
        assert self.tick(1.0) == "GUARD"
        assert self.queue("GUARD")
        assert self.write() == b"GUARD\n"
        self.reply(1.05, guard=True)
        self.sample(1.1)
        assert self.tick(1.1) is None
        assert self.write(1.3) == b"STATUS\n"
        self.reply(1.3)
        assert self.tick(1.3) is None
        assert self.write(1.6) == b"STATUS\n"
        self.sample(1.6)
        assert self.tick(1.6) == "ARM"
        assert self.queue("ARM")
        assert self.write() is None  # The read-only poll must finish first.
        self.reply(1.61)
        assert self.write() == b"ARM\n"
        self.reply(1.62, armed=True)
        assert self.tick(1.63) == "GRAB"
        assert self.queue("GRAB")


class SystemIntegrationTests(unittest.TestCase):
    def test_measured_ready_flow_sends_one_guard_arm_grab_and_never_auto_releases(self):
        host = HostWire()
        host.request_grab()
        self.assertEqual(host.write(), b"GRAB\n")
        host.reply(1.64, armed=False, state="GRABBING", result="ACCEPTED", job_id=1, job=2)
        host.tracking = host.correspondence = False
        self.assertIsNone(host.tick(1.7))
        self.assertIsNone(host.tick(1.8))
        self.assertEqual(host.write(2.0), b"STATUS\n")
        host.reply(2.0, state="HOLD_UNCONFIRMED", result="COMPLETE")
        self.assertIsNone(host.tick(2.0))
        motion = [item for item in host.writes if item != b"STATUS\n"]
        self.assertEqual(motion, [b"GUARD\n", b"ARM\n", b"GRAB\n"])
        self.assertEqual(host.auto.state, "FIRED")

    def test_unloaded_configured_controller_cannot_enable_automatic_capture(self):
        host = HostWire(physical_ready=False)
        self.assertTrue(host.link.snapshot.status.configured)
        self.assertFalse(host.link.snapshot.safety.physical_ready)
        self.assertFalse(host.auto.enable(host.context(), host.now))
        self.assertIsNone(host.tick(1.1))
        self.assertEqual(host.writes, [b"STATUS\n"])
        # Physical readiness applies to auto; keep the deliberate manual path.
        self.assertTrue(host.link.request("ARM", 1.1))
        self.assertEqual(host.write(), b"ARM\n")
        host.reply(1.11, armed=True)
        self.assertTrue(host.link.request("JOG A 100", 1.11))
        self.assertEqual(host.write(), b"JOG A 100\n")

    def test_reference_and_dialogs_survive_status_poll_without_motion(self):
        host = HostWire(physical_ready=False, referenced=False)
        self.assertEqual(host.write(1.4), b"STATUS\n")
        self.assertFalse(host.link.snapshot.blocks_dialog(1.4))
        self.assertTrue(host.link.request("REFERENCE_NEUTRAL", 1.4))
        self.assertIsNone(host.write())
        host.reply(1.41)
        self.assertEqual(host.write(), b"REFERENCE_NEUTRAL\n")
        host.reply(1.42, referenced=True, state="NEUTRAL")
        self.assertTrue(host.link.snapshot.status.referenced)
        self.assertFalse(host.link.snapshot.locally_armed)
        self.assertNotIn(b"ARM\n", host.writes)

    def test_new_invalid_sensor_after_grab_queue_sends_stop_at_write_boundary(self):
        host = HostWire()
        host.request_grab()
        host.sample(1.64, timeout=True)  # Arrives on serial before next UI frame.
        self.assertEqual(host.write(), b"STOP\n")
        self.assertNotIn(b"GRAB\n", host.writes)
        self.assertFalse(host.link.snapshot.locally_armed)
        # The write-boundary STOP finishes before the next UI frame. The fresh,
        # acknowledged disarmed state must suppress a redundant coordinator STOP.
        host.reply(1.65, armed=False, result="STOPPED")
        self.assertIsNone(host.tick(1.66))
        self.assertIsNone(host.write(1.66))
        self.assertEqual(host.writes.count(b"STOP\n"), 1)

    def test_pending_or_stale_disarmed_state_cannot_suppress_best_effort_stop(self):
        for uncertain in ("pending", "stale"):
            with self.subTest(state=uncertain):
                host = HostWire()
                host.request_grab()
                host.sample(1.64, timeout=True)
                self.assertEqual(host.write(), b"STOP\n")
                if uncertain == "pending":
                    # A state without the pending command's ACK is insufficient.
                    host.now = 1.65
                    host.feed("# motor NEUTRAL 1 0 1 0 0 STOPPED 0 0 -1\n")
                    self.assertTrue(host.link.snapshot.fresh(host.now))
                    self.assertEqual(host.link.snapshot.pending, "STOP")
                    self.assertEqual(host.tick(1.66), "STOP")
                else:
                    host.reply(1.65, armed=False, result="STOPPED")
                    host.sample(3.0)
                    self.assertFalse(host.link.snapshot.fresh(host.now))
                    self.assertEqual(host.tick(3.0), "STOP")

    def test_target_loss_after_queue_cannot_revive_grab_on_recovered_frame(self):
        host = HostWire()
        host.request_grab()
        host.tracking = host.correspondence = False
        self.assertEqual(host.tick(1.64), "STOP")
        # A write already about to dispatch still rechecks the canceled token.
        self.assertEqual(host.write(), b"STOP\n")
        self.assertNotIn(b"GRAB\n", host.writes)
        host.reply(1.65, armed=False, result="STOPPED")
        host.tracking = host.correspondence = True
        self.assertIsNone(host.tick(1.66))
        self.assertIsNone(host.write(1.66))
        self.assertEqual(host.auto.state, "OFF")

    def test_fresh_motor_metadata_cannot_mask_stale_camera_and_range_at_write(self):
        host = HostWire()
        host.request_grab()
        # Simulate fresh unsolicited firmware state but no new camera/range.
        host.now = 2.8
        host.feed("# motor SAFETY 1 1 1500 10000\n"
                  "# motor NEUTRAL 1 1 1 0 0 OK 0 0 -1\n")
        self.assertTrue(host.link.snapshot.fresh(host.now))
        self.assertEqual(host.write(), b"STOP\n")
        self.assertNotIn(b"GRAB\n", host.writes)

    def test_disconnect_after_grab_write_never_replays_on_new_session(self):
        host = HostWire()
        host.request_grab()
        self.assertEqual(host.write(), b"GRAB\n")
        first_session = host.link.snapshot.session_id
        host.link.disconnect()  # Outcome unknown; never retry a motion command.
        self.assertIsNone(host.tick(1.7))
        host.link.connect()
        self.assertGreater(host.link.snapshot.session_id, first_session)
        self.assertIsNone(host.link.snapshot.safety)
        self.assertEqual(host.write(1.8), b"STATUS\n")
        host.reply(1.81, armed=False, state="HOLD_UNCONFIRMED", result="COMPLETE", job_id=1, job=2)
        self.assertIsNone(host.tick(1.82))
        self.assertIsNone(host.write(1.82))
        self.assertEqual(host.writes.count(b"GRAB\n"), 1)
        self.assertNotIn(b"NEUTRAL\n", host.writes)

    def test_reconnect_discards_guard_metadata_and_requires_new_physical_evidence(self):
        host = HostWire()
        self.assertTrue(host.link.snapshot.safety.physical_ready)
        first_session = host.link.snapshot.session_id
        host.link.disconnect()
        host.link.connect()
        self.assertIsNone(host.link.snapshot.safety)
        self.assertGreater(host.link.snapshot.session_id, first_session)
        self.assertEqual(host.write(1.1), b"STATUS\n")
        host.reply(1.11, physical_ready=False)
        self.assertFalse(host.auto.enable(host.context(1.12), 1.12))
        self.assertFalse(host.link.snapshot.safety.physical_ready)

    def test_current_guard_and_profile_metadata_override_previously_ready_context(self):
        for safety in ("# motor SAFETY 1 0 1500 10000\n",
                       "# motor SAFETY 0 1 1500 10000\n"):
            with self.subTest(metadata=safety.strip()):
                host = HostWire()
                host.request_grab()
                host.now = 1.64
                host.feed(safety)
                self.assertEqual(host.write(), b"STOP\n")
                self.assertNotIn(b"GRAB\n", host.writes)


if __name__ == "__main__":
    unittest.main()
