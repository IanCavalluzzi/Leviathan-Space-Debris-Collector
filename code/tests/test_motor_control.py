"""Production protocol/sequencing tests; no serial device is opened."""

from pathlib import Path
import unittest

from motor_control import PAPER_PROFILE, MotorControls, MotorProfile, MotorLimits, MotorLink, MotorReply, MotorSafety, MotorStatus, motor_lines, parse_motor_line
from sensor_telemetry import LineDecoder, SensorReading


def state(now=1.0, name="NEUTRAL", armed=False, referenced=True, job_id=0,
          result="OK", job=0, configured=True):
    return parse_motor_line(
        f"# motor {name} {int(configured)} {int(armed)} {int(referenced)} 0 0 {result} {job_id} {job} -1", now)


def transact(link, command, status, now=1.0, error=None):
    assert link.request(command, now)
    assert link.next_write(now) == (command + "\n").encode()
    kind = "ERR" if error else "ACK"
    link.receive(parse_motor_line(f"# motor {kind} {error or 'OK'} {status.job_id}", now), now)
    link.receive(status, now)


def connected(status=None):
    link = MotorLink()
    link.connect()
    assert link.next_write(1.0) == b"STATUS\n"
    s = status or state()
    link.receive(MotorReply("ACK", "OK", s.job_id), 1.0)
    link.receive(s, 1.0)
    link.receive(PAPER_PROFILE, 1.0)
    link.receive(MotorControls(), 1.0)
    return link


class MotorProtocolTests(unittest.TestCase):
    def test_editable_controls_capability_and_signed_settings_contract(self):
        self.assertEqual(parse_motor_line("# motor CONTROLS 1", 1), MotorControls())
        self.assertIsNone(parse_motor_line("# motor CONTROLS 2", 1))
        link = connected(state(referenced=False, name="UNREFERENCED"))
        self.assertTrue(link.request("SET_STROKES -900 +700", 1))
        self.assertEqual(link.next_write(1), b"SET_STROKES -900 +700\n")
        for command in ("SET_STROKES 0 900", "SET_STROKES 2501 900", "SET_STROKES 900 -2501"):
            self.assertFalse(connected().request(command, 1))
        armed = connected()
        transact(armed, "ARM", state(armed=True))
        self.assertFalse(armed.request("SET_STROKES 900 900", 1))
        old = connected()
        old._update(controls=False)
        self.assertFalse(old.request("REFERENCE_REST", 1))
        self.assertFalse(old.request("SET_STROKES 900 900", 1))

    def test_profile_metadata_is_strict_and_does_not_acknowledge_or_survive_reconnect(self):
        line = "# motor PROFILE 0 0 900 0 0 900"
        self.assertEqual(parse_motor_line(line, 1), PAPER_PROFILE)
        for bad in (line + " extra", line.replace("900", "1000001", 1),
                    line.replace("900", "1.5", 1), "# motor PROFILE 0 0 0 0 0 0"):
            self.assertIsNone(parse_motor_line(bad, 1))
        decoder = LineDecoder(include_motor=True)
        self.assertEqual(decoder.feed((line + "\n").encode(), 1), [PAPER_PROFILE])
        link = MotorLink()
        link.connect()
        self.assertEqual(link.next_write(1), b"STATUS\n")
        link.receive(PAPER_PROFILE, 1)
        self.assertEqual(link.snapshot.pending, "STATUS")
        self.assertFalse(link.snapshot.synchronized)
        self.assertFalse(link.snapshot.locally_armed)
        link.disconnect()
        link.connect()
        self.assertIsNone(link.snapshot.profile)

    def test_old_or_wrong_profile_cannot_run_coordinated_jobs_but_jogs_remain(self):
        for profile in (None, MotorProfile(0, 0, 0, 0, 1022, -2033)):
            for command in ("GRAB", "NEUTRAL"):
                link = connected(state(name="POSITIONED" if command == "NEUTRAL" else "NEUTRAL"))
                transact(link, "ARM", state(armed=True, name=link.snapshot.status.state))
                link._update(profile=profile)
                self.assertFalse(link.request(command, 1))
                self.assertIn("PROFILE", link.snapshot.message)
                self.assertTrue(link.request("JOG A 1", 1))

    def test_grab_waits_for_prepared_rest_and_checks_profile_again_before_write(self):
        link = connected(state(name="POSITIONED"))
        transact(link, "ARM", state(armed=True, name="POSITIONED"))
        self.assertFalse(link.request("GRAB", 1))
        self.assertTrue(link.request("NEUTRAL", 1))
        link._update(profile=None)
        self.assertEqual(link.next_write(1), b"STOP\n")

    def test_safety_metadata_validates_flags_timeouts_and_timestamp(self):
        self.assertEqual(parse_motor_line("# motor SAFETY 1 0 1000 5000", 7),
                         MotorSafety(True, False, 1000, 5000, 7))
        for line in ("# motor SAFETY 2 0 1000 5000", "# motor SAFETY 1 0 0 5000",
                     "# motor SAFETY 1 0 1000 -1", "# motor SAFETY 1 0 60001 5000",
                     "# motor SAFETY 1 0 1000 60001", "# motor SAFETY 1 0 1000",
                     "# motor SAFETY 1 0 1.5 5000", "# motor SAFETY 1 0 1000 5000 extra"):
            self.assertIsNone(parse_motor_line(line, 7), line)
        for reason in ("GUARD_TIMEOUT", "ARM_EXPIRED"):
            self.assertEqual(parse_motor_line(f"# motor ERR {reason} 3", 7),
                             MotorReply("ERR", reason, 3))
            self.assertEqual(parse_motor_line(f"# motor NEUTRAL 1 0 1 0 0 {reason} 3 0 -1", 7).result,
                             reason)

    def test_limits_metadata_is_validated_and_matches_production_format(self):
        self.assertEqual(parse_motor_line("# motor LIMITS -2500 2500 2500 -2500 2500 100", 1),
                         MotorLimits(-2500, 2500, 2500, -2500, 2500, 100))
        for line in ("# motor LIMITS -2500 2500 0 -2500 2500 10",
                     "# motor LIMITS 5 1 1 -2500 2500 10",
                     "# motor LIMITS -2500 2500 6000 -2500 2500 10",
                     "# motor LIMITS -2500 2500 bad -2500 2500 10"):
            self.assertIsNone(parse_motor_line(line, 1))
        source = (Path(__file__).resolve().parents[1] / "firmware/grip_control/grip_control.ino").read_text()
        self.assertIn('"# motor LIMITS %ld %ld %ld %ld %ld %ld\\n"', source)

    def test_production_wire_format_is_the_source_contract(self):
        source = (Path(__file__).resolve().parents[1] / "firmware/grip_control/grip_control.ino").read_text()
        self.assertIn('"# motor %s %s %lu\\n"', source)
        self.assertIn('"# motor ERR %s\\n"', source)
        self.assertIn('"# motor %s %u %u %u %ld %ld %s %lu %u %d\\n"', source)

    def test_actual_ack_error_and_state_variants(self):
        self.assertEqual(parse_motor_line("# motor ACK OK 6\r\n", 1), MotorReply("ACK", "OK", 6))
        self.assertEqual(parse_motor_line("# motor ERR DISARMED 6", 1), MotorReply("ERR", "DISARMED", 6))
        for reason in ("BAD_LINE", "CONFIG_LOCKED", "TIMER_FAULT"):
            self.assertEqual(parse_motor_line(f"# motor ERR {reason}", 1), MotorReply("ERR", reason, None))
        s = parse_motor_line("# motor GRABBING 1 0 1 1022 -2033 ACCEPTED 4294967295 2 -1", 7)
        self.assertIsInstance(s, MotorStatus)
        self.assertTrue(s.moving)
        self.assertEqual((s.position_a, s.position_b, s.job, s.received_at), (1022, -2033, 2, 7))

    def test_malformed_or_invented_contracts_rejected(self):
        for line in ("# motor ACK GRAB 1", "# motor ACK ACCEPTED 1", "# motor ACK OK",
                     "# motor ACK OK -1", "# motor ACK OK 4294967296", "# motor ERR BUSY",
                     "# motor DONE 1", "motor ACK OK 1", "# motor GRABBING 1 0 1 0 ACCEPTED 2 2 -1",
                     "# motor NEUTRAL 2 0 1 0 0 OK 0 0 -1",
                     "# motor NEUTRAL 1 0 1 0 0 UNKNOWN 0 0 -1",
                     "# motor NEUTRAL 1 0 1 2147483648 0 OK 0 0 -1",
                     "# motor NEUTRAL 1 0 1 0 0 OK 0 4 -1"):
            with self.subTest(line=line):
                self.assertIsNone(parse_motor_line(line, 1))

    def test_partial_mixed_stream_preserves_sensor_csv_and_motor_order(self):
        decoder = LineDecoder(include_motor=True)
        self.assertEqual(decoder.feed(b"# motor ACK O", 1), [])
        items = decoder.feed(
            b"K 2\r\n1000,580,OK,10.0,0,1,0,1,0\r\n"
            b"# echo_debug phase=PULSE\n"
            b"# motor GRABBING 1 0 1 0 0 ACCEPTED 2 2 -1\n", 2)
        self.assertEqual([type(x) for x in items], [MotorReply, SensorReading, MotorStatus])
        self.assertEqual(items[1].ir, (0, 1, 0, 1, 0))


class SequencingTests(unittest.TestCase):
    def test_safety_metadata_never_acknowledges_or_arms_and_session_increments(self):
        link = MotorLink()
        link.connect()
        session = link.snapshot.session_id
        safety = MotorSafety(True, False, 1000, 5000, 1)
        link.receive(safety, 1)
        self.assertFalse(link.snapshot.fresh(1))
        self.assertFalse(link.snapshot.locally_armed)
        self.assertEqual(link.next_write(1), b"STATUS\n")
        link.receive(safety, 1)
        self.assertEqual(link.snapshot.pending, "STATUS")
        link.disconnect()
        self.assertEqual(link.snapshot.session_id, session)
        self.assertIsNone(link.snapshot.safety)
        link.connect()
        self.assertGreater(link.snapshot.session_id, session)
        self.assertIsNone(link.snapshot.safety)

    def test_guard_requires_metadata_idle_disarmed_but_no_reference(self):
        link = connected(state(name="UNREFERENCED", referenced=False))
        self.assertFalse(link.request("GUARD", 1))
        link.receive(MotorSafety(False, False, 1000, 5000, 1), 1)
        self.assertTrue(link.request("GUARD", 1))
        self.assertEqual(link.next_write(1), b"GUARD\n")
        self.assertFalse(link.snapshot.locally_armed)
        armed = connected(state(armed=True))
        armed.receive(MotorSafety(True, False, 1000, 5000, 1), 1)
        self.assertFalse(armed.request("GUARD", 1))

    def test_condition_is_checked_only_immediately_before_actual_write(self):
        checks = []
        link = connected()
        self.assertEqual(link.next_write(1.4), b"STATUS\n")
        self.assertTrue(link.request("ARM", 1.4, condition=lambda: checks.append(True) or True))
        self.assertEqual(checks, [])
        self.assertIsNone(link.next_write(1.4))
        link.receive(MotorReply("ACK", "OK", 0), 1.4)
        link.receive(state(now=1.4), 1.4)
        self.assertEqual(link.next_write(1.4), b"ARM\n")
        self.assertEqual(checks, [True])

    def test_false_condition_cancels_unarmed_action_without_replay(self):
        link = connected()
        self.assertTrue(link.request("ARM", 1, condition=lambda: False))
        self.assertIsNone(link.next_write(1))
        self.assertIsNone(link.snapshot.pending)
        self.assertFalse(link.snapshot.locally_armed)
        self.assertIn("conditions changed", link.snapshot.message)
        self.assertIsNone(link.next_write(1.1))

    def test_false_or_failed_condition_stops_instead_of_leaving_arm_live(self):
        def failed():
            raise ValueError("new camera/sensor state unavailable")
        for condition in (lambda: False, failed):
            link = connected()
            transact(link, "ARM", state(armed=True))
            self.assertTrue(link.request("GRAB", 1, condition=condition))
            self.assertEqual(link.next_write(1), b"STOP\n")
            self.assertEqual(link.snapshot.pending, "STOP")
            self.assertFalse(link.snapshot.locally_armed)
            link.receive(MotorReply("ACK", "OK", 0), 1)
            link.receive(state(result="STOPPED"), 1)
            self.assertIsNone(link.next_write(1.1))

    def test_queued_grab_retains_legitimate_arm_entitlement_behind_poll(self):
        link = connected()
        transact(link, "ARM", state(armed=True))
        self.assertEqual(link.next_write(1.4), b"STATUS\n")
        self.assertTrue(link.request("GRAB", 1.4, condition=lambda: True))
        self.assertFalse(link.snapshot.locally_armed)  # Consumed at request time.
        link.receive(MotorReply("ACK", "OK", 0), 1.4)
        link.receive(state(now=1.4, armed=True), 1.4)
        self.assertEqual(link.next_write(1.4), b"GRAB\n")

    def test_queued_movement_rechecks_current_controller_preconditions_after_poll(self):
        for updated in (state(now=1.4, armed=False),
                        state(now=1.4, armed=True, name="POSITIONED"),
                        state(now=1.4, armed=True, referenced=False),
                        state(now=1.4, armed=True, configured=False),
                        state(now=1.4, armed=False, name="GRABBING", job=2)):
            link = connected()
            transact(link, "ARM", state(armed=True))
            self.assertEqual(link.next_write(1.4), b"STATUS\n")
            self.assertTrue(link.request("GRAB", 1.4))
            link.receive(MotorReply("ACK", "OK", 0), 1.4)
            link.receive(updated, 1.4)
            outgoing = link.next_write(1.4)
            self.assertNotEqual(outgoing, b"GRAB\n")
            self.assertEqual(outgoing, b"STOP\n" if updated.armed else None)
            self.assertFalse(link.snapshot.locally_armed)

    def test_protocol_fault_discards_grab_waiting_behind_status_resynchronization(self):
        for fault in (MotorReply("ERR", "BAD_LINE", None),
                      MotorReply("ERR", "TIMER_FAULT", None)):
            link = connected()
            transact(link, "ARM", state(armed=True))
            self.assertEqual(link.next_write(1.4), b"STATUS\n")
            self.assertTrue(link.request("GRAB", 1.4))
            link.receive(fault, 1.41)
            self.assertFalse(link.snapshot.fresh(1.41))
            link.receive(MotorReply("ACK", "OK", 0), 1.42)
            link.receive(state(now=1.42, armed=True), 1.42)
            self.assertIsNone(link.next_write(1.43))
            self.assertFalse(link.snapshot.locally_armed)
            self.assertNotEqual(link.snapshot.pending, "GRAB")

    def test_unexpected_reply_or_mismatched_state_discards_unsent_normal_intent(self):
        link = connected()
        self.assertTrue(link.request("ARM", 1))
        link.receive(MotorReply("ACK", "OK", 0), 1)  # Nothing was sent yet.
        self.assertNotEqual(link.next_write(1), b"ARM\n")
        link = connected()
        self.assertEqual(link.next_write(1.4), b"STATUS\n")
        self.assertTrue(link.request("ARM", 1.4))
        link.receive(MotorReply("ACK", "OK", 0), 1.4)
        link.receive(state(now=1.4, job_id=1), 1.4)
        self.assertNotEqual(link.next_write(1.4), b"ARM\n")

    def test_protocol_fault_preserves_stop_and_does_not_restore_prior_arm_reply(self):
        link = connected()
        self.assertTrue(link.request("ARM", 1))
        self.assertEqual(link.next_write(1), b"ARM\n")
        link.receive(MotorReply("ERR", "BAD_LINE", None), 1)
        link.receive(MotorReply("ACK", "OK", 0), 1)
        link.receive(state(armed=True), 1)
        self.assertFalse(link.snapshot.fresh(1))
        self.assertFalse(link.snapshot.locally_armed)
        self.assertTrue(link.request("STOP", 1))
        link.receive(MotorReply("ERR", "BAD_LINE", None), 1)
        self.assertEqual(link.next_write(1), b"STOP\n")

    def test_fresh_idle_disarmed_status_poll_does_not_block_dialog(self):
        link = connected()
        self.assertEqual(link.next_write(1.4), b"STATUS\n")
        self.assertFalse(link.snapshot.blocks_dialog(1.4))
        self.assertTrue(link.request("ARM", 1.4))
        self.assertTrue(link.snapshot.blocks_dialog(1.4))

    def test_limits_never_acknowledge_or_arm_and_clear_on_reconnect(self):
        link = connected()
        caps = MotorLimits(-2500, 2500, 2500, -2500, 2500, 2500)
        link.receive(caps, 1)
        self.assertEqual(link.snapshot.limits, caps)
        self.assertFalse(link.snapshot.locally_armed)
        self.assertTrue(link.request("ARM", 1))
        self.assertEqual(link.next_write(1), b"ARM\n")
        link.receive(caps, 1)
        self.assertEqual(link.snapshot.pending, "ARM")
        self.assertFalse(link.snapshot.locally_armed)
        link.disconnect()
        link.connect()
        self.assertIsNone(link.snapshot.limits)

    def test_signed_jog_uses_actual_firmware_syntax_and_consumes_arm(self):
        link = connected()
        self.assertFalse(link.request("JOG B -10", 1))
        transact(link, "ARM", state(armed=True), 1)
        self.assertTrue(link.request("JOG B -10", 1))
        self.assertEqual(link.next_write(1), b"JOG B -10\n")
        self.assertFalse(link.snapshot.locally_armed)
        self.assertFalse(link.request("JOG B -10", 1))

    def test_invalid_jog_payloads_never_enter_the_wire(self):
        for command in ("JOG", "JOG C 10", "JOG A 0", "JOG A 2147483648", "JOG B -2147483649", "JOG A 10\nARM"):
            link = connected()
            self.assertFalse(link.request(command, 1))
            self.assertIsNone(link.next_write(1))

    def test_one_deliberate_action_can_wait_behind_a_status_poll(self):
        link = connected()
        self.assertEqual(link.next_write(1.4), b"STATUS\n")
        self.assertTrue(link.request("ARM", 1.4))
        self.assertFalse(link.request("ARM", 1.4))
        self.assertIsNone(link.next_write(1.4))
        link.receive(MotorReply("ACK", "OK", 0), 1.4)
        link.receive(state(now=1.4), 1.4)
        self.assertEqual(link.next_write(1.4), b"ARM\n")

    def test_stop_discards_action_waiting_behind_status(self):
        link = connected()
        self.assertEqual(link.next_write(1.4), b"STATUS\n")
        self.assertTrue(link.request("ARM", 1.4))
        self.assertTrue(link.request("STOP", 1.4))
        self.assertEqual(link.next_write(1.4), b"STOP\n")
        link.receive(MotorReply("ACK", "OK", 0), 1.4)
        link.receive(state(now=1.4), 1.4)
        link.receive(MotorReply("ACK", "OK", 0), 1.4)
        link.receive(state(now=1.4, result="STOPPED"), 1.4)
        self.assertIsNone(link.next_write(1.4))

    def test_connection_never_references_arms_or_moves(self):
        link = MotorLink()
        link.connect()
        self.assertFalse(link.request("ARM", 1))
        self.assertEqual(link.next_write(1), b"STATUS\n")
        self.assertIsNone(link.next_write(1.1))
        self.assertFalse(link.snapshot.fresh(1.1))

    def test_full_ack_state_arm_grab_and_deliberate_neutral_cycle(self):
        link = connected(state(name="UNREFERENCED", referenced=False))
        transact(link, "REFERENCE_NEUTRAL", state(), 1)
        transact(link, "ARM", state(armed=True), 1)
        self.assertTrue(link.snapshot.locally_armed)
        self.assertTrue(link.request("GRAB", 1))
        self.assertFalse(link.snapshot.locally_armed)
        self.assertEqual(link.next_write(1), b"GRAB\n")
        self.assertNotIn("completed", " ".join(motor_lines(link.snapshot, 1)).lower())
        link.receive(MotorReply("ACK", "OK", 1), 1)
        self.assertEqual(link.snapshot.pending, "GRAB")
        self.assertFalse(link.request("GRAB", 1))
        link.receive(state(name="GRABBING", job_id=1, result="ACCEPTED", job=2), 1)
        self.assertFalse(link.request("NEUTRAL", 1))
        link.receive(state(name="HOLD_UNCONFIRMED", job_id=1, result="COMPLETE", job=2), 1)
        self.assertIn("HOLD UNCONFIRMED", motor_lines(link.snapshot, 1)[0])
        self.assertFalse(link.request("NEUTRAL", 1))
        transact(link, "ARM", state(name="HOLD_UNCONFIRMED", armed=True, job_id=1, job=2), 1)
        transact(link, "NEUTRAL", state(name="RETURNING", job_id=2, result="ACCEPTED", job=3), 1)
        link.receive(state(job_id=2, result="COMPLETE", job=3), 1)
        self.assertFalse(link.snapshot.locally_armed)
        self.assertFalse(link.request("GRAB", 1))

    def test_controller_already_armed_requires_local_explicit_arm(self):
        link = connected(state(armed=True))
        self.assertFalse(link.request("GRAB", 1))
        self.assertFalse(link.snapshot.locally_armed)
        transact(link, "ARM", state(armed=True), 1)
        self.assertTrue(link.request("GRAB", 1))

    def test_no_reference_while_armed_or_grab_away_from_neutral(self):
        link = connected(state(armed=True))
        self.assertFalse(link.request("REFERENCE_NEUTRAL", 1))
        link = connected(state(name="POSITIONED"))
        transact(link, "ARM", state(name="POSITIONED", armed=True), 1)
        self.assertFalse(link.request("GRAB", 1))
        self.assertTrue(link.request("NEUTRAL", 1))

    def test_stale_config_locked_and_unreferenced_never_move(self):
        for s in (state(configured=False), state(referenced=False), state(now=0)):
            link = connected(s)
            self.assertFalse(link.request("ARM", 1.1))
            self.assertFalse(link.request("GRAB", 1.1))
            self.assertFalse(link.request("NEUTRAL", 1.1))
            self.assertTrue(link.request("STOP", 10))
            self.assertEqual(link.next_write(10), b"STOP\n")

    def test_stop_discards_unsent_movement_and_repeated_stop_is_not_queued(self):
        link = connected()
        transact(link, "ARM", state(armed=True), 1)
        self.assertTrue(link.request("GRAB", 1))
        self.assertTrue(link.request("STOP", 1))
        self.assertFalse(link.request("STOP", 1))
        self.assertEqual(link.next_write(1), b"STOP\n")
        self.assertFalse(link.request("NEUTRAL", 1))
        link.receive(MotorReply("ACK", "OK", 0), 1)
        link.receive(state(result="STOPPED"), 1)
        self.assertFalse(link.snapshot.locally_armed)
        self.assertIsNone(link.next_write(1.1))

    def test_stop_overtakes_sent_motion_and_replies_are_matched_fifo(self):
        link = connected()
        transact(link, "ARM", state(armed=True), 1)
        self.assertTrue(link.request("GRAB", 1))
        self.assertEqual(link.next_write(1), b"GRAB\n")
        self.assertTrue(link.request("STOP", 1.01))
        self.assertEqual(link.next_write(1.01), b"STOP\n")
        link.receive(MotorReply("ACK", "OK", 1), 1.02)
        link.receive(state(1.02, "GRABBING", job_id=1, result="ACCEPTED", job=2), 1.02)
        self.assertEqual(link.snapshot.pending, "STOP")
        link.receive(MotorReply("ACK", "OK", 1), 1.03)
        self.assertFalse(link.request("ARM", 1.03))
        link.receive(state(1.03, "UNREFERENCED", referenced=False, job_id=1, result="STOPPED", job=2), 1.03)
        self.assertIsNone(link.snapshot.pending)
        self.assertIn("STOP acknowledged", link.snapshot.message)
        self.assertFalse(link.snapshot.locally_armed)

    def test_stop_after_arm_sent_does_not_grant_local_arm(self):
        link = connected()
        self.assertTrue(link.request("ARM", 1))
        self.assertEqual(link.next_write(1), b"ARM\n")
        self.assertTrue(link.request("STOP", 1.01))
        self.assertEqual(link.next_write(1.01), b"STOP\n")
        link.receive(MotorReply("ACK", "OK", 0), 1.02)
        link.receive(state(1.02, armed=True), 1.02)
        self.assertFalse(link.snapshot.locally_armed)

    def test_rejection_is_shown_not_as_motion_or_completion(self):
        link = connected()
        transact(link, "ARM", state(), 1, "CONFIG_LOCKED")
        self.assertIn("rejected: CONFIG_LOCKED", link.snapshot.message)
        self.assertFalse(link.snapshot.locally_armed)

    def test_ack_without_state_and_silence_timeout_without_retry(self):
        link = connected()
        self.assertTrue(link.request("ARM", 1))
        self.assertEqual(link.next_write(1), b"ARM\n")
        link.receive(MotorReply("ACK", "OK", 0), 1.1)
        self.assertFalse(link.snapshot.locally_armed)
        self.assertFalse(link.request("GRAB", 1.2))
        with self.assertRaises(TimeoutError):
            link.next_write(2.1)

    def test_queued_action_cancels_if_status_ages_before_write(self):
        link = connected()
        self.assertTrue(link.request("ARM", 1))
        self.assertIsNone(link.next_write(2.1))
        self.assertIsNone(link.snapshot.pending)
        self.assertIn("canceled", link.snapshot.message)

    def test_disconnect_reconnect_discards_queued_and_sent_jobs_no_replay(self):
        for sent in (False, True):
            link = connected()
            transact(link, "ARM", state(armed=True), 1)
            link.request("GRAB", 1)
            if sent:
                self.assertEqual(link.next_write(1), b"GRAB\n")
            link.disconnect()
            self.assertIsNone(link.snapshot.status)
            self.assertIn("unknown", link.snapshot.message)
            self.assertFalse(link.request("STOP", 2))
            link.connect()
            self.assertEqual(link.next_write(3), b"STATUS\n")
            link.receive(MotorReply("ACK", "OK", 1), 3)
            link.receive(state(3, "HOLD_UNCONFIRMED", job_id=1, result="COMPLETE", job=2), 3)
            self.assertIsNone(link.next_write(3.1))
            self.assertFalse(link.snapshot.locally_armed)
            self.assertFalse(link.request("GRAB", 3.1))

    def test_bad_line_and_unexpected_ack_do_not_authorize_actions(self):
        for event in (MotorReply("ERR", "BAD_LINE", None), MotorReply("ACK", "OK", 0)):
            link = connected()
            link.receive(event, 1)
            self.assertFalse(link.snapshot.fresh(1))
            self.assertFalse(link.request("ARM", 1))

    def test_reply_and_state_job_mismatch_require_resynchronization(self):
        link = connected()
        self.assertTrue(link.request("ARM", 1))
        self.assertEqual(link.next_write(1), b"ARM\n")
        link.receive(MotorReply("ACK", "OK", 0), 1)
        link.receive(state(armed=True, job_id=1), 1)
        self.assertFalse(link.snapshot.fresh(1))
        self.assertFalse(link.snapshot.locally_armed)
        self.assertIn("mismatch", link.snapshot.message)
        self.assertFalse(link.request("GRAB", 1))

    def test_dialogs_require_fresh_disarmed_idle_no_pending(self):
        link = connected()
        self.assertFalse(link.snapshot.blocks_dialog(1))
        self.assertTrue(link.snapshot.blocks_dialog(2.1))
        self.assertTrue(link.request("ARM", 1))
        self.assertTrue(link.snapshot.blocks_dialog(1))
        self.assertTrue(connected(state(armed=True)).snapshot.blocks_dialog(1))
        self.assertTrue(connected(state(name="GRABBING")).snapshot.blocks_dialog(1))


if __name__ == "__main__":
    unittest.main()
