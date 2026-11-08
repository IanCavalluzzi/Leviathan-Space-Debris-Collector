"""Existing firmware CSV, stream framing, and freshness tests without hardware."""

from types import SimpleNamespace
import unittest
from unittest.mock import patch

import sensor_telemetry as telemetry
from unittest.mock import MagicMock


VALID = "1000,580,OK,10.0,0,1,0,1,0"


class SensorParsingTests(unittest.TestCase):
    def test_valid_distance_and_raw_ir(self):
        reading = telemetry.parse_line(VALID, 5.0)
        self.assertEqual(reading.distance_cm, 10.0)
        self.assertEqual(reading.ir, (0, 1, 0, 1, 0))
        self.assertEqual(reading.received_at, 5.0)

    def test_invalid_sensor_status_preserves_unknown_distance(self):
        for line in (
            "1200,0,TIMEOUT,NA,0,0,0,0,0",
            "1400,29000,OUT_OF_RANGE,NA,1,1,1,1,1",
        ):
            with self.subTest(line=line):
                reading = telemetry.parse_line(line, 5.0)
                self.assertIsNotNone(reading)
                self.assertIsNone(reading.distance_cm)

    def test_headers_corrupt_rows_and_nonfinite_values_are_rejected(self):
        rows = [
            "# Sensor test: TRIG=D2",
            "ms,echo_us,ultrasonic_status,distance_cm,ir1_d9,ir2_d10,ir3_d11,ir4_d12,ir5_d13",
            VALID.replace("10.0", "nan"),
            VALID.replace("10.0", "inf"),
            VALID.replace("10.0", "-10"),
            VALID.replace("OK", "UNKNOWN"),
            "1000,0,OK,10.0,0,1,0,1,0",
            "1000,0,TIMEOUT,0,0,1,0,1,0",
            "1000,580,OK,10.0,0,1,0,1,2",
            "1000,580,OK,10.0",
        ]
        for row in rows:
            with self.subTest(row=row):
                self.assertIsNone(telemetry.parse_line(row, 5.0))

    def test_partial_crlf_stream_is_reassembled(self):
        decoder = telemetry.LineDecoder()
        self.assertEqual(decoder.feed(VALID[:12].encode(), 1.0), [])
        readings = decoder.feed((VALID[12:] + "\r\n").encode(), 2.0)
        self.assertEqual(len(readings), 1)
        self.assertEqual(readings[0].distance_cm, 10)
        self.assertEqual(readings[0].received_at, 2.0)

    def test_echo_diagnostic_comments_do_not_interrupt_csv_readings(self):
        decoder = telemetry.LineDecoder()
        stream = (
            "1200,0,TIMEOUT,NA,0,0,0,0,0\r\n"
            "# echo_debug phase=NO_RISE before=0 after=0 width_us=0\r\n"
            + VALID + "\r\n"
            "# echo_debug phase=PULSE before=0 after=0 width_us=580\r\n"
        )
        readings = decoder.feed(stream.encode("ascii"), 5.0)
        self.assertEqual(len(readings), 2)
        self.assertIsNone(readings[0].distance_cm)
        self.assertEqual(readings[1].distance_cm, 10.0)

    def test_oversized_and_non_ascii_lines_resynchronize(self):
        decoder = telemetry.LineDecoder()
        readings = decoder.feed(b"x" * 700 + b"\n\xff\n" + VALID.encode() + b"\n", 2.0)
        self.assertEqual(len(readings), 1)
        self.assertEqual(len(decoder.buffer), 0)

    def test_stale_values_are_hidden(self):
        reading = telemetry.parse_line(VALID, 5.0)
        snapshot = telemetry.SensorSnapshot("receiving", "COM22", reading)
        fresh = telemetry.format_snapshot(snapshot, now=5.2)
        self.assertIn("10.0 cm", fresh[0])
        self.assertIn("echo target unconfirmed", fresh[1])
        self.assertNotIn("0 1 0 1 0", " ".join(fresh))
        stale = telemetry.format_snapshot(snapshot, now=7.0)
        self.assertIn("STALE", stale[0])
        self.assertNotIn("10.0 cm", stale[0])
        self.assertNotIn("0 1 0 1 0", stale[1])

    def test_timeout_remains_unknown_in_ultrasonic_only_display(self):
        reading = telemetry.parse_line("1200,0,TIMEOUT,NA,0,0,0,0,0", 5.0)
        lines = telemetry.format_snapshot(telemetry.SensorSnapshot("receiving", "COM22", reading), now=5.2)
        self.assertIn("distance unknown", lines[0])
        self.assertIn("TIMEOUT", lines[0])
        self.assertNotIn("0 0 0 0 0", " ".join(lines))

    def test_ir_is_absent_from_all_display_states_but_legacy_csv_is_readable(self):
        reading = telemetry.parse_line(VALID, 5.0)
        self.assertIsNotNone(reading)
        for snapshot, now in (
            (telemetry.SensorSnapshot("receiving", "COM22", reading), 5.2),
            (telemetry.SensorSnapshot("receiving", "COM22", reading), 7.0),
            *((telemetry.SensorSnapshot(state), 5.2) for state in (
                "off", "starting", "not_found", "ambiguous", "waiting", "disconnected",
                "access_denied", "reply_timeout", "unknown")),
        ):
            text = " ".join(telemetry.format_snapshot(snapshot, now=now))
            self.assertNotIn("IR", text)
            self.assertNotIn("OUT1", text)
            self.assertNotIn("0 1 0 1 0", text)

    def test_disabled_reader_does_not_open_a_serial_port(self):
        with patch.object(telemetry.serial, "Serial") as serial_port:
            reader = telemetry.SensorReader("off")
            reader.start()
            reader.stop()
        serial_port.assert_not_called()

    def test_automatic_write_condition_reaches_the_single_serial_owner(self):
        reader = telemetry.SensorReader("off")
        condition = lambda: False
        with patch.object(reader.motor, "request", return_value=True) as request:
            self.assertTrue(reader.command("GRAB", condition=condition))
        request.assert_called_once_with("GRAB", condition=condition)

    def test_auto_detection_ignores_other_devices_and_requires_unique_board(self):
        board = SimpleNamespace(vid=0x2341, pid=0x1002, device="COM22")
        other = SimpleNamespace(vid=0x10C4, pid=0xEA60, device="COM21")
        reader = telemetry.SensorReader()
        with patch.object(telemetry.list_ports, "comports", return_value=[board, other]):
            self.assertEqual(reader._find_port(), "COM22")
        with patch.object(telemetry.list_ports, "comports", return_value=[board, board]):
            self.assertIsNone(reader._find_port())
            self.assertEqual(reader.snapshot.state, "ambiguous")

    def test_explicit_replacement_port_bypasses_uno_discovery(self):
        reader = telemetry.SensorReader("COM7")
        with patch.object(telemetry.list_ports, "comports") as enumerate_ports:
            self.assertEqual(reader._find_port(), "COM7")
        enumerate_ports.assert_not_called()

    def test_listing_bridge_and_unknown_ports_does_not_open_or_identify_board(self):
        ports = [
            SimpleNamespace(vid=0x10C4, pid=0xEA60, device="COM7", description="USB UART"),
            SimpleNamespace(vid=None, pid=None, device="COM1", description="Serial Port"),
        ]
        with patch.object(telemetry.list_ports, "comports", return_value=ports), \
                patch.object(telemetry.serial, "Serial") as serial_port:
            lines = telemetry.serial_port_lines()
        self.assertIn("COM1", lines[0])
        self.assertIn("USB ID unavailable", lines[0])
        self.assertIn("COM7", lines[1])
        self.assertIn("10C4:EA60", lines[1])
        self.assertNotIn("ESP32", " ".join(lines))
        serial_port.assert_not_called()

    def test_listing_no_ports_explains_missing_connection(self):
        with patch.object(telemetry.list_ports, "comports", return_value=[]):
            self.assertIn("No serial ports", telemetry.serial_port_lines()[0])

    def test_connection_guidance_does_not_prescribe_uno_port_or_firmware(self):
        for state in ("not_found", "ambiguous", "waiting"):
            with self.subTest(state=state):
                lines = telemetry.format_snapshot(telemetry.SensorSnapshot(state, "COM7"))
                self.assertNotIn("COM22", " ".join(lines))
                self.assertNotIn("Upload sensor_test", " ".join(lines))

    def test_repeated_sample_does_not_refresh_received_time_and_only_polls_status(self):
        reader = telemetry.SensorReader("COM22")
        connection = MagicMock()
        connection.write.side_effect = len
        connection.in_waiting = 100
        reads = iter([(VALID + "\n").encode(), (VALID + "\n").encode()])

        def read_chunk(size):
            try:
                return next(reads)
            except StopIteration:
                reader._stop.set()
                return b""

        connection.read.side_effect = read_chunk
        with patch.object(telemetry.serial, "Serial") as serial_port, \
                patch.object(telemetry.time, "monotonic", side_effect=[10.0, 10.0, 10.1, 10.1, 10.2, 10.2]):
            serial_port.return_value.__enter__.return_value = connection
            reader._run()
        self.assertEqual(reader.snapshot.reading.received_at, 10.0)
        connection.write.assert_called_once_with(b"STATUS\n")
        serial_port.return_value.__exit__.assert_called_once()

    def test_disconnect_clears_previously_valid_sample(self):
        reader = telemetry.SensorReader("COM22")
        connection = MagicMock()
        connection.write.side_effect = len
        connection.in_waiting = 100
        connection.read.side_effect = [
            (VALID + "\n").encode(),
            telemetry.serial.SerialException("device removed"),
        ]
        with patch.object(telemetry.serial, "Serial") as serial_port, \
                patch.object(reader._stop, "wait", side_effect=lambda seconds: reader._stop.set()):
            serial_port.return_value.__enter__.return_value = connection
            reader._run()
        self.assertEqual(reader.snapshot.state, "disconnected")
        self.assertIsNone(reader.snapshot.reading)
        connection.write.assert_called_once_with(b"STATUS\n")

    def test_open_denied_and_reply_timeout_have_specific_connection_states(self):
        for error, expected in ((telemetry.serial.SerialException("PermissionError(13, Access is denied, 5)"), "access_denied"),
                                (TimeoutError("Controller reply missing"), "reply_timeout")):
            reader = telemetry.SensorReader("COM22")
            with patch.object(telemetry.serial, "Serial", side_effect=error), \
                    patch.object(reader._stop, "wait", side_effect=lambda seconds: reader._stop.set()):
                reader._run()
            self.assertEqual(reader.snapshot.state, expected)
            self.assertTrue(reader.snapshot.detail)
            self.assertIsNone(reader.snapshot.reading)

    def test_worker_mixed_csv_and_motor_status_uses_one_bounded_usb_connection(self):
        reader = telemetry.SensorReader("COM22")
        connection = MagicMock()
        connection.in_waiting = 100000
        connection.write.side_effect = len
        chunks = iter([
            (VALID + "\n# motor ACK OK 0\n# motor NEUTRAL 1 0 1 0 0 OK 0 0 -1\n").encode(),
        ])

        def read_chunk(size):
            self.assertLessEqual(size, 4096)
            try:
                return next(chunks)
            except StopIteration:
                reader._stop.set()
                return b""

        connection.read.side_effect = read_chunk
        with patch.object(telemetry.serial, "Serial") as serial_port:
            serial_port.return_value.__enter__.return_value = connection
            reader._run()
        serial_port.assert_called_once_with("COM22", 115200, timeout=0.05, write_timeout=0.1)
        self.assertEqual(reader.snapshot.reading.distance_cm, 10)
        self.assertEqual(reader.motor_snapshot.status.state, "NEUTRAL")
        connection.write.assert_called_once_with(b"STATUS\n")

    def test_worker_disconnect_after_motion_write_reconnects_with_status_only(self):
        reader = telemetry.SensorReader("COM22")
        first, second = MagicMock(), MagicMock()
        for connection in (first, second):
            connection.in_waiting = 1024
            connection.write.side_effect = len
        count = 0

        def first_read(size):
            nonlocal count
            count += 1
            if count == 1:
                # Receipt occurs after this read; request ARM at the next write opportunity.
                return b"# motor ACK OK 0\n# motor CONTROLS 1\n# motor PROFILE 0 0 900 0 0 900\n# motor NEUTRAL 1 0 1 900 0 OK 0 0 -1\n"
            if count == 2:
                return b"# motor ACK OK 0\n# motor NEUTRAL 1 1 1 0 0 OK 0 0 -1\n"
            raise telemetry.serial.SerialException("removed after GRAB write")

        first.read.side_effect = first_read
        first_writes = []

        def write_first(data):
            first_writes.append(data)
            return len(data)

        first.write.side_effect = write_first
        original_next = reader.motor.next_write

        def next_write(now):
            if reader.motor_snapshot.pending is None and reader.motor_snapshot.synchronized:
                if reader.motor_snapshot.locally_armed:
                    reader.motor.request("GRAB", now)
                else:
                    reader.motor.request("ARM", now)
            return original_next(now)

        def second_read(size):
            reader._stop.set()
            return b"# motor ACK OK 1\n# motor GRABBING 1 0 1 1 -1 ACCEPTED 1 2 -1\n"

        second.read.side_effect = second_read
        with patch.object(telemetry.serial, "Serial", side_effect=[first, second]) as serial_port, \
                patch.object(reader.motor, "next_write", side_effect=next_write), \
                patch.object(reader._stop, "wait", return_value=False):
            first.__enter__.return_value = first
            second.__enter__.return_value = second
            reader._run()
        self.assertEqual(first_writes, [b"STATUS\n", b"ARM\n", b"GRAB\n"])
        second.write.assert_called_once_with(b"STATUS\n")
        self.assertEqual(serial_port.call_count, 2)
        self.assertFalse(reader.motor_snapshot.locally_armed)
        self.assertIsNone(reader.snapshot.reading)

    def test_partial_usb_write_disconnects_without_retrying_command(self):
        reader = telemetry.SensorReader("COM22")
        connection = MagicMock()
        connection.write.return_value = 2
        with patch.object(telemetry.serial, "Serial") as serial_port, \
                patch.object(reader._stop, "wait", side_effect=lambda seconds: reader._stop.set()):
            serial_port.return_value.__enter__.return_value = connection
            reader._run()
        connection.write.assert_called_once_with(b"STATUS\n")
        connection.read.assert_not_called()
        self.assertFalse(reader.motor_snapshot.connected)
        self.assertEqual(reader.snapshot.state, "disconnected")


if __name__ == "__main__":
    unittest.main()
