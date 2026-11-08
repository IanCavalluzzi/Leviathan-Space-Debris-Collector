"""Single background 115200-baud USB owner for sensor CSV and motor commands."""

from dataclasses import dataclass
import math
from threading import Event, Thread
import time

import serial
from serial.tools import list_ports

from motor_control import MotorLink, parse_motor_line


STALE_SECONDS = 1.0  # Display freshness for the sketch's approximately 0.2 s cadence.
BAUD = 115200


def serial_port_lines():
    """List USB identities without opening ports or guessing the controller board."""
    ports = sorted(list_ports.comports(), key=lambda port: port.device)
    lines = []
    for port in ports:
        usb_id = (
            f"VID:PID={port.vid:04X}:{port.pid:04X}"
            if port.vid is not None and port.pid is not None else "USB ID unavailable"
        )
        lines.append(f"{port.device}: {port.description} | {usb_id}")
    return lines or ["No serial ports found. Connect the controller with a USB data cable."]


@dataclass(frozen=True)
class SensorReading:
    uptime_ms: int
    echo_us: int
    status: str
    distance_cm: float | None
    ir: tuple[int, ...]
    received_at: float


@dataclass(frozen=True)
class SensorSnapshot:
    state: str = "starting"
    port: str | None = None
    reading: SensorReading | None = None
    detail: str = ""


def parse_line(line, received_at):
    """Reject malformed data; preserve the firmware's explicit invalid distance."""
    fields = [field.strip() for field in line.strip().split(",")]
    if len(fields) != 9 or not fields[0].isdigit() or not fields[1].isdigit():
        return None
    if any(value not in ("0", "1") for value in fields[4:]):
        return None
    uptime, echo = int(fields[0]), int(fields[1])
    if uptime > 0xFFFFFFFF or echo > 0xFFFFFFFF:
        return None
    status = fields[2]
    if status == "OK":
        try:
            distance = float(fields[3])
        except ValueError:
            return None
        if echo == 0 or not math.isfinite(distance) or not 2 <= distance <= 400:
            return None
    elif status in ("TIMEOUT", "OUT_OF_RANGE"):
        if fields[3] != "NA" or (status == "TIMEOUT") != (echo == 0):
            return None
        distance = None
    else:
        return None
    return SensorReading(uptime, echo, status, distance, tuple(map(int, fields[4:])), received_at)


class LineDecoder:
    """Reassemble partial reads and discard oversized lines through their newline."""

    def __init__(self, include_motor=False):
        self.buffer = bytearray()
        self.discarding = False
        self.include_motor = include_motor

    def feed(self, data, received_at):
        readings = []
        for byte in data:
            if byte == 10:
                if not self.discarding:
                    try:
                        line = self.buffer.decode("ascii")
                        reading = parse_line(line, received_at)
                        if reading is None and self.include_motor:
                            reading = parse_motor_line(line, received_at)
                    except UnicodeDecodeError:
                        reading = None
                    if reading is not None:
                        readings.append(reading)
                self.buffer.clear()
                self.discarding = False
            elif not self.discarding:
                if len(self.buffer) >= 512:
                    self.buffer.clear()
                    self.discarding = True
                else:
                    self.buffer.append(byte)
        return readings


class SensorReader:
    """Keep finite serial reads and reconnection outside the camera/UI loop."""

    def __init__(self, port="auto"):
        self.requested_port = port
        self.snapshot = SensorSnapshot("off" if port.lower() == "off" else "starting")
        self._stop = Event()
        self._thread = None
        self.motor = MotorLink()

    @property
    def motor_snapshot(self):
        return self.motor.snapshot

    def command(self, command, condition=None):
        if condition is None:
            return self.motor.request(command)
        return self.motor.request(command, condition=condition)

    def start(self):
        if self.requested_port.lower() != "off" and self._thread is None:
            self._thread = Thread(target=self._run, name="sensor-telemetry", daemon=True)
            self._thread.start()

    def stop(self):
        s = self.motor.snapshot
        if s.connected and (s.pending not in (None, "STATUS") or
                            (s.status is not None and (s.status.moving or s.status.armed))):
            self.motor.request("STOP")
            # Let the sole USB owner send STOP; never claim shutdown proves a halt.
            self._stop.wait(0.35)
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        self.snapshot = SensorSnapshot("off")
        self.motor.disconnect()

    def _find_port(self):
        if self.requested_port.lower() != "auto":
            return self.requested_port
        boards = [
            port for port in list_ports.comports()
            # Match the selected UNO R4 WiFi, not unrelated generic USB bridges.
            if (port.vid, port.pid) == (0x2341, 0x1002)
        ]
        if len(boards) != 1:
            self.snapshot = SensorSnapshot("ambiguous" if boards else "not_found")
            return None
        return boards[0].device

    def _run(self):
        while not self._stop.is_set():
            port = None
            try:
                port = self._find_port()
                if port is None:
                    self._stop.wait(1.0)
                    continue
                with serial.Serial(port, BAUD, timeout=0.05, write_timeout=0.1) as connection:
                    connection.reset_input_buffer()
                    self.snapshot = SensorSnapshot("waiting", port)
                    self.motor.connect()  # STATUS only; never reference, arm or move.
                    decoder = LineDecoder(include_motor=True)
                    while not self._stop.is_set():
                        now = time.monotonic()
                        outgoing = self.motor.next_write(now)
                        if outgoing is not None:
                            if connection.write(outgoing) != len(outgoing):
                                raise serial.SerialException("Incomplete write; outcome unknown")
                        chunk = connection.read(min(4096, max(1, connection.in_waiting)))
                        received_at = time.monotonic()
                        for reading in decoder.feed(chunk, received_at):
                            if isinstance(reading, SensorReading):
                                previous = self.snapshot.reading
                                # Cached samples must not refresh display freshness.
                                if previous is None or reading.uptime_ms != previous.uptime_ms:
                                    self.snapshot = SensorSnapshot("receiving", port, reading)
                            else:
                                self.motor.receive(reading, received_at)
            except (serial.SerialException, OSError, TimeoutError) as exc:
                if isinstance(exc, TimeoutError):
                    state, detail = "reply_timeout", "USB opened but compatible controller replies timed out"
                elif (isinstance(exc, PermissionError) or getattr(exc, "winerror", None) == 5
                      or "PermissionError" in str(exc) or "Access is denied" in str(exc)):
                    state, detail = "access_denied", "Port access denied; another app or permissions may block it"
                else:
                    state, detail = "disconnected", "USB connection unavailable or interrupted"
                self.snapshot = SensorSnapshot(state, port, detail=detail)
                self.motor.disconnect()
            if not self._stop.is_set():
                self._stop.wait(1.0)


def format_snapshot(snapshot, now=None):
    """Ultrasonic-only HUD; legacy IR fields stay in the wire parser for compatibility."""
    now = time.monotonic() if now is None else now
    port = snapshot.port or "auto"
    reading = snapshot.reading
    if reading is None:
        messages = {
            "off": ("Ultrasonic off (--port off)", "Camera-only view"),
            "starting": ("Ultrasonic: connecting", "Camera continues while USB is checked"),
            "not_found": ("Ultrasonic: no controller", "Connect UNO R4 WiFi; --list-ports shows USB ports"),
            "ambiguous": ("Ultrasonic: multiple boards", "Use --list-ports, then choose --port"),
            "waiting": (f"Ultrasonic {port}: waiting for CSV", "Expect sensor CSV at 115200; close Serial Monitor"),
            "disconnected": (f"Ultrasonic {port}: connection unavailable", "Check USB / close Serial Monitor; retrying"),
            "access_denied": (f"USB {port}: port access denied", "Close the app using this port; also check permissions"),
            "reply_timeout": (f"USB {port}: controller reply timed out", "Check grip_control firmware; reconnecting without motion replay"),
        }
        return messages.get(snapshot.state, ("Ultrasonic: unavailable", "Distance unknown"))
    age = max(0.0, now - reading.received_at)
    if age > STALE_SECONDS:
        return (
            f"Ultrasonic {port}: STALE ({age:.1f} s); distance unknown",
            "Echo target unconfirmed",
        )
    distance = f"{reading.distance_cm:.1f} cm" if reading.distance_cm is not None else "distance unknown"
    return (
        f"Ultrasonic {port}: {distance} | {reading.status} | age {age:.1f} s",
        "echo target unconfirmed",
    )
