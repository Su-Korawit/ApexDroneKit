"""
apexdrone.py - A beginner-friendly Python library for the APEX GD-149 drone.

Quick start:

    from apexdrone import Drone

    with Drone("sim") as d:      # "sim" = practice mode, no real drone needed
        d.takeoff()
        d.forward(dist=50)       # centimetres
        d.rotate_cw(angle=90)    # degrees
        d.land()

Connection modes (the `link` argument):
    "sim"   Practice mode. Prints what it would do. No drone, no risk. (default)
    "ble"   Bluetooth Low Energy. Keeps your internet working. Reports sensors.
    "wifi"  Connect to the drone's Wi-Fi hotspot first. No sensor data back.

Safety:
    * Fly in an open space, at least 3 m clear of people and objects.
    * `takeoff()` climbs to about 110 cm, so you need 2 m of ceiling.
    * Ctrl+C asks the drone to land, but it is NOT an emergency stop.
      If the motors will not stop, run emergency_stop.py or unplug the battery.

This library was written against a real GD-149. Several fields in the vendor
documentation turned out to be wrong; the corrected units are used here and the
differences are noted in the comments where they matter.
"""

from __future__ import annotations

import csv
import json
import pathlib
import socket
import struct
import threading
import time
from collections import deque

# ---------------------------------------------------------------------------
# Protocol constants
# ---------------------------------------------------------------------------

HEAD = 0xCC
TAIL = 0x33
CENTER = 0x80              # neutral stick position
TRIM_CENTER = 0x40         # neutral trim position
OPTICAL_FLOW_DEFAULT = 122  # ~1.22, the value the spec asks for

SEND_INTERVAL = 0.1        # a frame must go out every 100 ms or the drone cuts out
PULSE_TICKS = 5            # momentary buttons are held for ~5 frames (500 ms)
TURN_LEAD_REF_POWER = 50   # the power that the learned braking distance refers to

# --- Factory settings -------------------------------------------------------
# How far and how fast this model of drone moves. These are averages measured
# from a real GD-149; they are not exact for any individual drone. See the
# "About distances" section of the README before relying on them.
# A teacher can change these once here and every student gets the new values.
MOVE_POWER = 40             # power used for forward / back / left / right
MOVE_CM_PER_SECOND = 37.0   # how far it travels per second at that power
TURN_POWER = 50             # power used for rotate_cw / rotate_ccw
TURN_DEG_PER_SECOND = 90.0  # how fast it turns at that power (timed fallback)
CLIMB_POWER = 40            # power used for up / down
CLIMB_CM_PER_SECOND = 55.0  # how fast it climbs at that power (timed fallback)

# Allowed ranges. Asking for something outside these raises an error that says
# what the limits are, rather than silently doing nothing.
DIST_MIN_CM, DIST_MAX_CM = 10, 180
ANGLE_MIN_DEG, ANGLE_MAX_DEG = 1, 360
SPEED_MIN, SPEED_MAX = 1, 3

# Safety cap: no single command may run longer than this, however large a value
# is asked for. Stops a slip like forward(dist=1000) crossing the room.
MAX_MOVE_SECONDS = 5.0

WIFI_HOST = "192.168.1.1"
WIFI_PORT = 3333

BLE_UUID_UART = "0000ae01-0000-1000-8000-00805f9b34fb"   # older models, raw bytes
BLE_UUID_USART = "0000ffe1-0000-1000-8000-00805f9b34fb"  # newer models, MSP v1
BLE_NOTIFY_USART = "0000ffe4-0000-1000-8000-00805f9b34fb"
BLE_PREFIXES = ("APEX_USART", "APEX_UART")

# The folder this library lives in. Used so that output files always land in a
# predictable place no matter which directory you run your script from.
HERE = pathlib.Path(__file__).resolve().parent


# ---------------------------------------------------------------------------
# Small helpers
# ---------------------------------------------------------------------------

def find_file(name: str) -> pathlib.Path | None:
    """Look for a file in the current directory first, then next to this library."""
    for base in (pathlib.Path.cwd(), HERE):
        candidate = base / name
        if candidate.exists():
            return candidate
    return None


def output_file(name: str) -> pathlib.Path:
    """Where results are written: always beside this library, so nothing scatters."""
    return HERE / name


def clean_name(name: str | None) -> str:
    """Tidy a Bluetooth name so it can be compared and copied reliably.

    Real drones advertise names with rubbish attached - one GD-149 calls itself
    "APEX_USART_751F02\x02\n", with a control byte and a newline on the end.
    Those characters are invisible on screen, so a name copied from a scan looks
    identical to the real one and still fails to match. Stray spaces typed by
    hand cause the same puzzle. Strip both, on both sides of every comparison.
    """
    return "".join(ch for ch in (name or "") if ch.isprintable()).strip()


def yaw_delta(new: int, old: int) -> int:
    """Difference between two headings, wrapping correctly. Result is -180..180.

    Example: going from 16 to 327 is -49 degrees, not +311.
    """
    return ((new - old + 180) % 360) - 180


def _clamp(value: float, low: int, high: int) -> int:
    return max(low, min(high, int(value)))


def _num(value: float) -> str:
    """Render 50.0 as '50' but 1.5 as '1.5', so messages read naturally."""
    return str(int(value)) if float(value).is_integer() else str(value)


def _stick(power: float) -> int:
    """Convert a power of -100..100 into a stick byte of 0x01..0xFF (0x80 = centre)."""
    return _clamp(CENTER + round(power * 1.27), 0x01, 0xFF)


# ---------------------------------------------------------------------------
# Battery
# ---------------------------------------------------------------------------

# Resting-voltage curve for a 1S LiPo cell. LiPo has a very flat middle: 3.70 to
# 3.90 V covers 10% to 70%, so treat the percentage as a rough guide only.
_BATTERY_CURVE = [
    (3.27, 0), (3.61, 5), (3.69, 10), (3.71, 15), (3.73, 20), (3.75, 25),
    (3.77, 30), (3.79, 35), (3.80, 40), (3.82, 45), (3.84, 50), (3.85, 55),
    (3.87, 60), (3.91, 65), (3.95, 70), (3.98, 75), (4.02, 80), (4.08, 85),
    (4.11, 90), (4.15, 95), (4.20, 100),
]

BATTERY_WARN_VOLT = 3.70   # start looking for a place to land
BATTERY_LAND_VOLT = 3.55   # land now
BATTERY_MIN_VOLT = 3.30    # below this a LiPo is permanently damaged


def battery_percent_from_volt(volt: float) -> int:
    """Rough state of charge for a 1S LiPo, from its resting voltage.

    While the motors are running the voltage sags by up to 0.4 V, so an in-flight
    reading understates the real charge. Trust the value measured at rest.
    """
    if volt <= _BATTERY_CURVE[0][0]:
        return 0
    if volt >= _BATTERY_CURVE[-1][0]:
        return 100
    for (v1, p1), (v2, p2) in zip(_BATTERY_CURVE, _BATTERY_CURVE[1:]):
        if v1 <= volt <= v2:
            return round(p1 + (p2 - p1) * (volt - v1) / (v2 - v1))
    return 0


# ---------------------------------------------------------------------------
# The command frame - every link sends the same information, wrapped differently
# ---------------------------------------------------------------------------

class _Frame:
    """The state of every stick and button at one moment in time."""

    def __init__(self) -> None:
        self.reset_sticks()
        self.speed = 0                            # 0 slow, 1 medium, 2 fast
        self.optical_flow = OPTICAL_FLOW_DEFAULT
        # Momentary buttons are countdowns: held for N frames, then released.
        self._pulses = {"headless": 0, "takeoff": 0, "return": 0,
                        "flip": 0, "emergency": 0, "calibrate": 0}

    def reset_sticks(self) -> None:
        self.throttle = CENTER
        self.yaw = CENTER
        self.pitch = CENTER
        self.roll = CENTER
        self.pitch_trim = TRIM_CENTER
        self.roll_trim = TRIM_CENTER

    def press(self, name: str, ticks: int = PULSE_TICKS) -> None:
        self._pulses[name] = ticks

    def tick(self) -> None:
        """Call once per frame sent, so momentary buttons release themselves."""
        for name, count in self._pulses.items():
            if count > 0:
                self._pulses[name] = count - 1

    @property
    def special1(self) -> int:
        p = self._pulses
        return ((_clamp(self.speed, 0, 2) << 6)
                | ((1 if p["headless"] else 0) << 5)
                | ((1 if p["takeoff"] else 0) << 3)
                | ((1 if p["return"] else 0) << 1)
                | (1 if p["flip"] else 0))

    @property
    def special2(self) -> int:
        p = self._pulses
        return ((1 if p["emergency"] else 0) << 1) | (1 if p["calibrate"] else 0)

    def ble_payload(self) -> bytes:
        """The 10 payload bytes used by the Bluetooth link."""
        return bytes([self.throttle, self.yaw, 0x00, self.pitch, self.roll,
                      self.pitch_trim, self.roll_trim,
                      self.special1, self.special2, self.optical_flow])

    def ble_frame(self) -> bytes:
        """Full 13-byte frame. Checksum here is the sum of the payload."""
        payload = self.ble_payload()
        return bytes([HEAD]) + payload + bytes([sum(payload) & 0xFF, TAIL])

    def wifi_bytes(self, legacy_init: bool = False) -> list[int]:
        """The 14-byte Wi-Fi frame. One byte longer, and a different checksum rule."""
        payload = [self.throttle, self.yaw, 0x30, self.pitch, self.roll,
                   self.pitch_trim, self.roll_trim,
                   self.special1 | (1 if legacy_init else 0), self.special2,
                   0x00, self.optical_flow]
        checksum = (0xFF - (sum(payload) & 0xFF)) & 0xFF
        return [HEAD] + payload + [checksum, TAIL]


def _msp_v1(code: int, data: bytes) -> bytes:
    """Wrap a payload in an MSP v1 frame:  $ M <  length  code  data...  xor"""
    frame = bytearray(b"$M<")
    frame.append(len(data))
    frame.append(code)
    checksum = len(data) ^ code
    for byte in data:
        frame.append(byte)
        checksum ^= byte
    frame.append(checksum)
    return bytes(frame)


# ---------------------------------------------------------------------------
# Telemetry (Bluetooth only)
# ---------------------------------------------------------------------------

def parse_telemetry(frame: bytes) -> dict | None:
    """Decode a `$M>` code 0x0F frame from the drone into a readable dict.

    Units here come from measurements on a real GD-149, not from the vendor
    document, which is wrong in three places:
      * yaw is whole degrees 0-359, not hundredths of a degree
      * battery voltage is in units of 10 mV, not 1 mV
      * byte 9 is not a usable percentage: it equals 150 * volts - 453, so a
        healthy LiPo always reads above 100. It is kept as `battery_raw` for
        reference and the percentage is computed from the voltage instead.
    Flags bits 0-3 (sensor-ready flags) are always zero on this firmware, so
    they are not reported - a False there would wrongly suggest a broken sensor.
    """
    if len(frame) < 18 or frame[:3] != b"$M>" or frame[4] != 0x0F:
        return None
    d = frame[5:5 + frame[3]]
    if len(d) < 12:
        return None

    alt_bat = d[6] | (d[7] << 8) | (d[8] << 16)
    flags = d[10] | (d[11] << 8)
    armed_code = (flags >> 7) & 0x3
    armed_names = {0: "locked", 1: "armed", 2: "flying"}
    volt = ((alt_bat >> 12) & 0xFFF) / 100.0

    return {
        # Not a distance. Behaves like a position error against a hold point the
        # flight controller keeps re-setting. Do not use it to measure travel.
        "optical_flow_x": struct.unpack_from("<h", d, 0)[0],
        "optical_flow_y": struct.unpack_from("<h", d, 2)[0],
        "yaw_deg": struct.unpack_from("<h", d, 4)[0],   # whole degrees, 0-359
        "altitude_cm": alt_bat & 0xFFF,                 # only updates once armed
        "battery_volt": round(volt, 2),
        "battery_percent": battery_percent_from_volt(volt),
        "battery_raw": d[9],                            # unusable, kept for study
        "flags_raw": flags,
        "armed": armed_names.get(armed_code, "?"),
        # The firmware only ever uses 0 and 1: it means "on the ground" or
        # "in the air". It never reports hovering or descending separately.
        "fly_status": ["on ground", "in air", "hovering", "descending"][(flags >> 12) & 0x3],
        "low_battery": (flags >> 5) & 0x3,
    }


# ---------------------------------------------------------------------------
# Links - each one exposes open() / send() / close()
# ---------------------------------------------------------------------------

class _SimLink:
    """Practice mode. Connects to nothing and simply reports what it would do."""

    name = "simulation"
    telemetry: dict | None = None
    raw: bytes | None = None

    def open(self) -> None:
        print("Practice mode - no real drone. Your code runs exactly as normal.")

    def send(self, frame: _Frame) -> None:
        pass

    def frame_repr(self, frame: _Frame) -> str:
        return frame.ble_frame().hex(" ")

    def close(self) -> None:
        print("Practice mode finished.")


class _WifiLink:
    """Talks to the drone's own Wi-Fi hotspot over TCP, using the CTP protocol."""

    name = "Wi-Fi"
    telemetry: dict | None = None
    raw: bytes | None = None

    def __init__(self, host: str = WIFI_HOST, port: int = WIFI_PORT,
                 legacy_init: bool = False) -> None:
        self.host = host
        self.port = port
        self.legacy_init = legacy_init
        self.sock: socket.socket | None = None

    @staticmethod
    def _packet(topic: str, content: dict) -> bytes:
        topic_bytes = topic.encode()
        body = json.dumps(content).encode()
        return (b"CTP:" + struct.pack("<h", len(topic_bytes)) + topic_bytes
                + struct.pack("<i", len(body)) + body)

    def open(self) -> None:
        self.sock = socket.create_connection((self.host, self.port), timeout=5)
        print(f"Connected to the drone at {self.host}:{self.port}")

    def send(self, frame: _Frame) -> None:
        if self.sock is None:
            return
        values = frame.wifi_bytes(self.legacy_init)
        params = {f"D{i}": str(v) for i, v in enumerate(values)}
        packet = self._packet("GENERIC_CMD", {"op": "PUT", "param": params})
        try:
            self.sock.sendall(packet)
        except OSError:
            self._reconnect()

    def frame_repr(self, frame: _Frame) -> str:
        return " ".join(str(v) for v in frame.wifi_bytes(self.legacy_init))

    def _reconnect(self) -> None:
        try:
            if self.sock:
                self.sock.close()
        except OSError:
            pass
        self.sock = None
        for attempt in range(1, 4):
            try:
                self.sock = socket.create_connection((self.host, self.port), timeout=5)
                print("Reconnected.")
                return
            except OSError:
                print(f"Reconnect attempt {attempt} failed.")
                time.sleep(1)

    def close(self) -> None:
        if self.sock:
            self.sock.close()
            self.sock = None


class _BleLink:
    """Talks over Bluetooth Low Energy. Needs `pip install bleak`.

    asyncio is hidden inside a background thread so the rest of the library, and
    your own code, stays plain and synchronous.
    """

    name = "Bluetooth"

    def __init__(self, device_name: str | None = None, retries: int = 5) -> None:
        self.device_name = device_name
        self.retries = retries
        self.telemetry: dict | None = None
        self.raw: bytes | None = None
        self._client = None
        self._loop = None
        self._thread = None
        self._write_uuid = BLE_UUID_UART
        self._use_msp = False

    def _run(self, coro):
        import asyncio
        return asyncio.run_coroutine_threadsafe(coro, self._loop).result(timeout=40)

    def open(self) -> None:
        import asyncio
        from bleak import BleakClient, BleakScanner

        self._loop = asyncio.new_event_loop()
        self._thread = threading.Thread(target=self._loop.run_forever, daemon=True)
        self._thread.start()

        target = clean_name(self.device_name)
        if target:
            print(f"Looking for drone '{target}'...")
        else:
            print("Looking for a drone (no name given)...")

        # Keep scanning for a while: the drone may still be starting up.
        device = None
        seen: set[str] = set()
        for attempt in range(1, self.retries + 1):
            devices = self._run(BleakScanner.discover(timeout=6.0))
            named = [d for d in devices if d.name]
            seen.update(clean_name(d.name) for d in named)

            if target:
                # A name was given, so match that name and nothing else.
                # Compared after cleaning, because the advertised name often
                # carries invisible characters. Never guess between drones.
                matched = [d for d in named if clean_name(d.name) == target]
                if not matched:
                    matched = [d for d in named
                               if clean_name(d.name).upper() == target.upper()]
                if len(matched) == 1:
                    device = matched[0]
                    break
            else:
                apex = [d for d in named if d.name.upper().startswith(BLE_PREFIXES)]
                if len(apex) == 1:
                    device = apex[0]
                    break
                if len(apex) > 1:
                    # Several drones nearby and no way to tell which is yours.
                    # Refusing is the only safe answer: picking one could fly
                    # somebody else's drone.
                    raise RuntimeError(
                        "Found more than one drone, and none was specified:\n"
                        + "\n".join(f"    {clean_name(d.name)}"
                                    for d in sorted(apex, key=lambda x: x.name))
                        + "\n\nSay which one is yours:\n"
                          f"    Drone('ble', device_name="
                          f"'{clean_name(sorted(apex, key=lambda x: x.name)[0].name)}')\n"
                          "\nRun  python3 scan_drones.py  if you are not sure "
                          "which name is which.")

            if attempt < self.retries:
                print(f"  Not found yet, retrying {attempt + 1} of {self.retries}...")

        if device is None:
            raise RuntimeError(
                (f"No drone named '{target}' found"
                 if target else "No drone whose name starts with APEX_ was found")
                + f" after {self.retries} attempts.\n"
                "Things to check:\n"
                "  1. Is the drone switched on and close to the computer?\n"
                "  2. Is a phone or the remote already connected to it?\n"
                "  3. Do not pair the drone in your system Bluetooth settings.\n"
                "  4. Is the name spelled correctly? Run scan_drones.py to check.\n"
                "     Spaces and invisible characters are removed automatically,\n"
                "     so only the visible letters and digits have to match.\n"
                f"Devices seen while searching: {sorted(seen) if seen else 'none'}")

        self._client = BleakClient(device.address, timeout=30.0)
        self._run(self._client.connect())

        # Choose the protocol from the characteristics the drone actually has.
        # That is more reliable than guessing from its name.
        uuids = {c.uuid.lower()
                 for service in self._client.services for c in service.characteristics}
        if BLE_UUID_USART in uuids:
            self._use_msp, self._write_uuid = True, BLE_UUID_USART
        elif BLE_UUID_UART in uuids:
            self._use_msp, self._write_uuid = False, BLE_UUID_UART
        else:
            self._use_msp = bool(device.name
                                 and device.name.upper().startswith("APEX_USART"))
            self._write_uuid = BLE_UUID_USART if self._use_msp else BLE_UUID_UART
            print("Warning: no familiar characteristic found, guessing from the name.")
            print("  Characteristics present:", sorted(uuids))

        print(f"Connected to {clean_name(device.name)} "
              f"(protocol {'USART/MSP' if self._use_msp else 'UART/RAW'})")

        if self._use_msp:
            try:
                self._run(self._client.start_notify(BLE_NOTIFY_USART, self._on_notify))
                print("Sensor data enabled.")
            except Exception:
                print("Note: this drone sends no sensor data. Control still works.")

    def _on_notify(self, _sender, data: bytearray) -> None:
        self.raw = bytes(data)
        self.telemetry = parse_telemetry(self.raw) or self.telemetry

    def send(self, frame: _Frame) -> None:
        if self._client is None:
            return
        payload = (_msp_v1(0x0F, frame.ble_payload()) if self._use_msp
                   else frame.ble_frame())
        try:
            self._run(self._client.write_gatt_char(self._write_uuid, payload))
        except Exception:
            pass

    def frame_repr(self, frame: _Frame) -> str:
        if self._use_msp:
            return _msp_v1(0x0F, frame.ble_payload()).hex(" ")
        return frame.ble_frame().hex(" ")

    def close(self) -> None:
        try:
            if self._client:
                self._run(self._client.disconnect())
        except Exception:
            pass
        if self._loop:
            self._loop.call_soon_threadsafe(self._loop.stop)


# ---------------------------------------------------------------------------
# The Drone class
# ---------------------------------------------------------------------------

class Drone:
    """One APEX GD-149 drone.

    Example:
        with Drone("ble") as d:
            d.takeoff()
            d.forward(2, power=40)
            d.turn_right_deg(90)
            d.land()

    Arguments:
        link         "sim", "ble" or "wifi".
        device_name  Which drone to connect to, e.g. "APEX_USART_C73303".
                     Run scan_drones.py to see the names. Leave it out only if
                     exactly one drone will ever be switched on.
        debug        Print the actual bytes sent whenever they change.
        verbose      Print a line for each action (on by default).
        legacy_init  Wi-Fi only. Try True if takeoff does nothing over Wi-Fi.
    """

    def __init__(self, link: str = "sim", *, device_name: str | None = None,
                 legacy_init: bool = False, verbose: bool = True,
                 debug: bool = False) -> None:
        link = link.lower()
        if link == "sim":
            self._link = _SimLink()
        elif link == "wifi":
            self._link = _WifiLink(legacy_init=legacy_init)
        elif link == "ble":
            self._link = _BleLink(device_name=device_name)
            if self._link.device_name:
                print(f"Target drone: {self._link.device_name}")
            else:
                print("No drone name given - will only connect if exactly one "
                      "is found. Run scan_drones.py to see the names.")
        else:
            raise ValueError('link must be "sim", "ble" or "wifi"')

        self.verbose = verbose
        self.flying = False
        self._frame = _Frame()
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._sender: threading.Thread | None = None
        self._connected = False

        self.debug = debug
        self.log: list[tuple[float, str]] = []
        self._t0 = time.time()
        self._last_frame = ""
        self._rec = None
        self._rec_file = None
        self._battery_alert = 0
        self._volt_window = deque(maxlen=20)   # about 2 seconds of readings
        self._turn_lead = 13.0   # braking distance in degrees, learned as you fly
        self._climb_lead = 8.0   # braking distance in cm, learned as you fly
        self._emergency_active = False

    # ---------------------------------------------------------------- plumbing

    def _say(self, message: str) -> None:
        self.log.append((round(time.time() - self._t0, 2), message))
        if self.verbose:
            print(message)

    def connect(self) -> "Drone":
        self._link.open()
        self._connected = True
        self._stop.clear()
        self._sender = threading.Thread(target=self._send_loop, daemon=True)
        self._sender.start()
        time.sleep(0.5)      # let a few neutral frames arrive before commanding
        return self

    def _send_loop(self) -> None:
        """Send a frame every 100 ms, forever. This is the heart of the protocol."""
        while not self._stop.is_set():
            with self._lock:
                if self._emergency_active:
                    # Nothing else may raise the throttle while we are cutting
                    # the motors - not even a movement command that is still
                    # finishing on another thread.
                    self._frame.throttle = 0x00
                self._link.send(self._frame)
                if self.debug:
                    text = self._link.frame_repr(self._frame)
                    if text != self._last_frame:
                        self._last_frame = text
                        print("    frame sent:", text)
                self._frame.tick()
            self._write_telemetry_row()
            self._check_battery()
            time.sleep(SEND_INTERVAL)

    def close(self) -> None:
        if self.flying:
            self._say("Warning: still flying - landing before disconnecting.")
            try:
                self.land()
            except Exception:
                pass

        # Never disconnect while the motors might still be turning.
        info = self.status()
        if info is not None and info["armed"] != "locked":
            print("\n!! Warning: the drone is not locked. The motors may keep "
                  "running after this program ends.")
            print("   Sending an emergency stop...")
            for _ in range(2):
                with self._lock:
                    self._frame.throttle = 0x00
                    self._frame.press("emergency", PULSE_TICKS * 2)
                time.sleep(1.2)
            info = self.status()
            if info is not None and info["armed"] != "locked":
                print("   Still not locked. If the motors are running, unplug "
                      "the battery,")
                print("   or run  python3 emergency_stop.py  in a new window.")

        self._stop.set()
        if self._sender:
            self._sender.join(timeout=2)
        self._link.close()
        self._connected = False

    def __enter__(self) -> "Drone":
        return self.connect()

    def __exit__(self, exc_type, exc, tb) -> bool:
        if exc_type is KeyboardInterrupt:
            self._say("\nCtrl+C pressed - landing safely.")
        self.close()
        return exc_type is KeyboardInterrupt

    # ------------------------------------------------------- internal helpers

    @staticmethod
    def _check(name: str, value: float, low: float, high: float, unit: str) -> float:
        """Reject a value that is outside the range the drone can manage."""
        if not low <= value <= high:
            span = f"between {low} and {high} {unit}".rstrip()
            raise ValueError(f"{name} must be {span}, got {value}. "
                             f"Try {name}={low if value < low else high}.")
        return value

    def _press(self, button: str, label: str) -> None:
        self._say(label)
        with self._lock:
            self._frame.press(button)
        time.sleep(PULSE_TICKS * SEND_INTERVAL + 0.1)

    def _move(self, label: str, seconds: float, **sticks: int) -> None:
        """Hold the sticks in a position for a time, then centre them again."""
        if self._emergency_active:
            self._say(f"Refusing {label}: an emergency stop is in progress.")
            return
        if seconds > MAX_MOVE_SECONDS:
            self._say(f"Refusing {label}: that needs {seconds:.1f} s and the limit "
                      f"is {MAX_MOVE_SECONDS} s. Ask for a smaller value.")
            return
        self._say(f"{label} ({round(seconds, 2)} s)")
        with self._lock:
            for name, value in sticks.items():
                setattr(self._frame, name, value)
        time.sleep(max(0.0, seconds))
        with self._lock:
            self._frame.reset_sticks()
        time.sleep(0.3)      # settle before the next command

    # -------------------------------------------------------------- taking off

    def takeoff(self) -> None:
        """Take off. The drone climbs to about 110 cm on its own."""
        if self.flying:
            self._say("Note: already flying, skipping takeoff().")
            return
        self._press("takeoff", "Taking off")
        self.flying = True
        time.sleep(3.0)

    def land(self) -> None:
        """Land.

        Takeoff and land are the same button on this drone, so the library keeps
        track of whether you are airborne and ignores whichever one is wrong.
        """
        if not self.flying:
            self._say("Note: not flying, skipping land().")
            return
        self._press("takeoff", "Landing")
        self.flying = False
        time.sleep(3.0)

    def hover(self, seconds: float = 2.0) -> None:
        """Stay in place for a number of seconds."""
        self._say(f"Hovering for {_num(seconds)} s")
        with self._lock:
            self._frame.reset_sticks()
        time.sleep(seconds)

    # ------------------------------------------------- moving, distance in cm

    def forward(self, dist: int) -> None:
        """Fly forward by dist, in centimetres. Approximate - see the README."""
        self._check("dist", dist, DIST_MIN_CM, DIST_MAX_CM, "cm")
        self._move(f"Forward {_num(dist)} cm", dist / MOVE_CM_PER_SECOND,
                   pitch=_stick(MOVE_POWER))

    def back(self, dist: int) -> None:
        """Fly backward by dist, in centimetres. Approximate."""
        self._check("dist", dist, DIST_MIN_CM, DIST_MAX_CM, "cm")
        self._move(f"Back {_num(dist)} cm", dist / MOVE_CM_PER_SECOND,
                   pitch=_stick(-MOVE_POWER))

    def left(self, dist: int) -> None:
        """Slide left by dist, in centimetres, without turning. Approximate."""
        self._check("dist", dist, DIST_MIN_CM, DIST_MAX_CM, "cm")
        self._move(f"Left {_num(dist)} cm", dist / MOVE_CM_PER_SECOND,
                   roll=_stick(-MOVE_POWER))

    def right(self, dist: int) -> None:
        """Slide right by dist, in centimetres, without turning. Approximate."""
        self._check("dist", dist, DIST_MIN_CM, DIST_MAX_CM, "cm")
        self._move(f"Right {_num(dist)} cm", dist / MOVE_CM_PER_SECOND,
                   roll=_stick(MOVE_POWER))

    def up(self, dist: int) -> None:
        """Climb by dist, in centimetres.

        Over Bluetooth this watches the height sensor and stops when it gets
        there, so it is much more accurate than the horizontal moves.
        """
        self._check("dist", dist, DIST_MIN_CM, DIST_MAX_CM, "cm")
        self._climb("up", dist)

    def down(self, dist: int) -> None:
        """Descend by dist, in centimetres. Uses the height sensor if available."""
        self._check("dist", dist, DIST_MIN_CM, DIST_MAX_CM, "cm")
        self._climb("down", dist)

    # --------------------------------------------- turning, angle in degrees

    def rotate_cw(self, angle: int) -> None:
        """Rotate clockwise (turn right) by angle, in degrees.

        Over Bluetooth this watches the gyro and stops at the right heading, so
        it is far more accurate than the horizontal moves.
        """
        self._check("angle", angle, ANGLE_MIN_DEG, ANGLE_MAX_DEG, "degrees")
        self._rotate("cw", angle)

    def rotate_ccw(self, angle: int) -> None:
        """Rotate anticlockwise (turn left) by angle, in degrees."""
        self._check("angle", angle, ANGLE_MIN_DEG, ANGLE_MAX_DEG, "degrees")
        self._rotate("ccw", angle)

    # ------------------------------------------------------------ raw control

    def move(self, direction: str, seconds: float, power: int = MOVE_POWER) -> None:
        """Hold one stick for a time, instead of asking for a distance.

        This is the raw form of a movement command - it is what forward(),
        up() and the rest are built from. Use it when you want to control the
        drone by feel rather than by numbers.

            d.move("forward", seconds=1.5, power=60)

        direction: forward, back, left, right, up, down, cw, ccw
        """
        self._check("seconds", seconds, 0.1, MAX_MOVE_SECONDS, "seconds")
        self._check("power", power, 0, 100, "percent")
        sticks = {
            "forward": {"pitch": _stick(power)},
            "back": {"pitch": _stick(-power)},
            "left": {"roll": _stick(-power)},
            "right": {"roll": _stick(power)},
            "up": {"throttle": _stick(power)},
            "down": {"throttle": _stick(-power)},
            "cw": {"yaw": _stick(power)},
            "ccw": {"yaw": _stick(-power)},
        }
        if direction not in sticks:
            raise ValueError(f"direction must be one of {sorted(sticks)}, "
                             f"got '{direction}'")
        self._move(f"Move {direction} at power {power}", seconds, **sticks[direction])

    # ----------------------------------------------------------- extra actions

    def flip(self) -> None:
        """Do a flip. Needs 3 m of clear space and more than 50% battery.

        Unlike some drones, this one has no way to choose the direction.
        """
        self._press("flip", "Flip")
        time.sleep(1.5)

    def go_home(self) -> None:
        """Ask the drone to return to where it took off from."""
        self._press("return", "Returning to start")

    def headless(self) -> None:
        """Toggle headless mode, where forward means the way you push rather
        than the way the drone is facing."""
        self._press("headless", "Toggling headless mode")

    def calibrate(self) -> None:
        """Calibrate the sensors. Put the drone on a flat surface first."""
        self._press("calibrate", "Calibrating")
        time.sleep(2)

    def set_speed(self, level: int) -> None:
        """Set the speed gear: 1 slow, 2 medium, 3 fast."""
        self._check("level", level, SPEED_MIN, SPEED_MAX, "")
        self._say(f"Speed set to level {level}")
        with self._lock:
            self._frame.speed = _clamp(level - 1, 0, 2)

    def emergency_stop(self, repeats: int = 3) -> None:
        """Cut the motors immediately. The drone will fall if it is in the air.

        Sends the emergency bit together with zero throttle, several times over,
        because a single short pulse is not always enough.
        """
        self._say("EMERGENCY STOP - cutting motors")
        self._emergency_active = True
        try:
            for _ in range(max(1, repeats)):
                with self._lock:
                    self._frame.throttle = 0x00
                    self._frame.press("emergency", PULSE_TICKS * 2)
                time.sleep(1.2)
        finally:
            self._emergency_active = False
        with self._lock:
            self._frame.reset_sticks()
        self.flying = False
        time.sleep(0.5)
        info = self.status()
        if info is not None and info["armed"] != "locked":
            print(f"!! Still not locked (armed={info['armed']}, "
                  f"status={info['fly_status']}). If the motors are running, "
                  "unplug the battery.")

    # ------------------------------------- sensor-guided turning and climbing

    def _rotate(self, direction: str, deg: float, power: float = TURN_POWER,
                tolerance: float = 4) -> None:
        """Turn to an angle, allowing for the fact that the drone cannot stop dead.

        Releasing the stick exactly on target always overshoots, because the drone
        keeps spinning for another 13 degrees or so at power 50. Two things fix it:
          1. brake early, by a distance that is learned from the previous turn
          2. after it settles, measure again and correct gently if still off
        """
        sign = 1 if direction == "cw" else -1

        if self.status() is None:
            # No sensor data on this link, so the turn has to be timed instead.
            self._move(f"Rotate {direction} {_num(deg)} deg (timed, approximate)",
                       deg / TURN_DEG_PER_SECOND, yaw=_stick(sign * TURN_POWER))
            return

        self._say(f"Rotate {direction} {_num(deg)} deg (guided by the gyro)")
        origin = self.status()["yaw_deg"]

        def turned() -> float:
            """Degrees turned so far, positive in the requested direction."""
            return -sign * yaw_delta(self.status()["yaw_deg"], origin)

        for attempt in range(3):
            error = deg - turned()
            if abs(error) <= tolerance:
                break

            use_power = power if attempt == 0 else 22
            # Braking distance scales with turn rate, so scale it with power.
            lead = self._turn_lead * use_power / TURN_LEAD_REF_POWER
            if attempt > 0:
                self._say(f"  still off by {error:+.0f} deg, correcting gently")

            way = 1 if error > 0 else -1     # negative means we overshot
            goal = turned() + abs(error) - lead
            deadline = time.time() + max(3.0, abs(error) / 15.0 + 2.0)

            with self._lock:
                self._frame.yaw = _stick(sign * way * use_power)
            try:
                while time.time() < deadline:
                    if (turned() >= goal) if way > 0 else (turned() <= goal):
                        break
                    time.sleep(0.04)
            finally:
                with self._lock:
                    self._frame.reset_sticks()

            time.sleep(0.7)      # let it stop completely before measuring

            if attempt == 0 and way > 0:
                # Store the coast distance normalised to the reference power.
                observed = (turned() - goal) * TURN_LEAD_REF_POWER / max(use_power, 1)
                if 0 <= observed <= 60:
                    self._turn_lead = round(self._turn_lead * 0.4 + observed * 0.6, 1)

        final = turned()
        self._say(f"  turned {final:.0f} deg (off by {final - deg:+.0f}), "
                  f"heading now {self.status()['yaw_deg']}")

    def _climb(self, direction: str, dist: float, tolerance: float = 5) -> None:
        """Change height by a distance, watching the height sensor if there is one.

        Same idea as _rotate: stop a little early because the drone keeps moving
        after the stick is released, then check and correct.
        """
        sign = 1 if direction == "up" else -1

        info = self.status()
        if info is None or info["altitude_cm"] == 0:
            # No height reading (Wi-Fi link, or not airborne yet), so time it.
            self._move(f"{direction.capitalize()} {_num(dist)} cm (timed, approximate)",
                       dist / CLIMB_CM_PER_SECOND, throttle=_stick(sign * CLIMB_POWER))
            return

        self._say(f"{direction.capitalize()} {_num(dist)} cm (guided by the height sensor)")
        origin = info["altitude_cm"]

        def moved() -> float:
            return sign * (self.status()["altitude_cm"] - origin)

        for attempt in range(2):
            error = dist - moved()
            if abs(error) <= tolerance:
                break
            use_power = CLIMB_POWER if attempt == 0 else 22
            lead = self._climb_lead if attempt == 0 else 1.0
            way = 1 if error > 0 else -1
            goal = moved() + abs(error) - lead
            deadline = time.time() + max(3.0, abs(error) / 20.0 + 2.0)

            with self._lock:
                self._frame.throttle = _stick(sign * way * use_power)
            try:
                while time.time() < deadline:
                    if (moved() >= goal) if way > 0 else (moved() <= goal):
                        break
                    time.sleep(0.04)
            finally:
                with self._lock:
                    self._frame.reset_sticks()

            time.sleep(0.7)

            if attempt == 0 and way > 0:
                observed = moved() - goal
                if 0 <= observed <= 40:
                    self._climb_lead = round(self._climb_lead * 0.4 + observed * 0.6, 1)

        final = moved()
        self._say(f"  moved {final:.0f} cm (off by {final - dist:+.0f}), "
                  f"height now {self.status()['altitude_cm']} cm")

    # --------------------------------------------------------- sensor readings

    def status(self) -> dict | None:
        """Latest sensor readings, or None. Bluetooth only."""
        return getattr(self._link, "telemetry", None)

    def raw_status(self) -> bytes | None:
        """The raw bytes the drone last sent, before they were decoded."""
        return getattr(self._link, "raw", None)

    def battery(self) -> int | None:
        """Battery percentage, or None. Only trustworthy while the motors are idle."""
        info = self.status()
        return info["battery_percent"] if info else None

    def _check_battery(self) -> None:
        """Warn once per level when the battery gets low.

        Uses the median of the last two seconds rather than the latest reading,
        because the voltage dips by up to 0.36 V whenever the motors spin up and
        springs back afterwards. A single reading would trigger false alarms.
        """
        info = self.status()
        if info is None:
            return
        self._volt_window.append(info["battery_volt"])
        if len(self._volt_window) < self._volt_window.maxlen:
            return
        volt = sorted(self._volt_window)[len(self._volt_window) // 2]
        if volt <= BATTERY_LAND_VOLT and self._battery_alert < 2:
            self._battery_alert = 2
            print(f"!! Battery very low: {volt} V - land now with d.land()")
        elif volt <= BATTERY_WARN_VOLT and self._battery_alert < 1:
            self._battery_alert = 1
            print(f"!  Battery getting low: {volt} V - prepare to land")

    # ------------------------------------------------------ recording to files

    def save_log(self, path: str = "flight_log.csv") -> str:
        """Write every action, with its timestamp, to a CSV file."""
        path = output_file(path)
        with open(path, "w", newline="", encoding="utf-8") as handle:
            writer = csv.writer(handle)
            writer.writerow(["time_s", "event"])
            writer.writerows(self.log)
        print(f"Log saved to {path} ({len(self.log)} lines)")
        return str(path)

    def start_recording(self, path: str = "telemetry.csv") -> None:
        """Record sensor readings every 100 ms to a CSV file. Bluetooth only."""
        path = output_file(path)
        self._rec_file = open(path, "w", newline="", encoding="utf-8")
        self._rec = csv.writer(self._rec_file)
        self._rec.writerow(["time_s", "volts", "battery_pct", "altitude_cm",
                            "yaw_deg", "flow_x", "flow_y", "status"])
        print(f"Recording sensor data to {path}")

    def stop_recording(self) -> None:
        if self._rec_file is not None:
            self._rec_file.close()
            self._rec_file = None
            self._rec = None
            print("Recording stopped.")

    def _write_telemetry_row(self) -> None:
        if self._rec is None:
            return
        info = self.status()
        if info is None:
            return
        self._rec.writerow([round(time.time() - self._t0, 2),
                            info["battery_volt"], info["battery_percent"],
                            info["altitude_cm"], info["yaw_deg"],
                            info["optical_flow_x"], info["optical_flow_y"],
                            info["fly_status"]])


if __name__ == "__main__":
    # Run this file directly to check the library works. No drone needed.
    with Drone("sim", debug=True) as d:
        d.takeoff()
        d.hover(1)
        d.forward(dist=50)
        d.rotate_cw(angle=90)
        d.land()
