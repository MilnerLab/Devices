"""
ell14/controller.py
====================

`ElliptecController` — the Elliptec (ELLx) serial-bus controller, on the shared
device framework. This module also owns the low-level protocol: command/reply
enums, the wire encode/parse helpers, and the `ElliptecError` exception.

Elliptec modules are a multi-drop serial bus: several addressed devices (hex
digits '0'..'F') share one serial line. This controller owns that line and speaks
the fixed-length ASCII packet protocol; each device is addressed by its hex bus
address, and the low-level I/O / status polling / discovery live here keyed by
that address.

**Rule**: after sending any motion command we poll GS at a fixed frequency and
only continue once we receive GS00; a hard max poll count prevents endless loops.
"""

from __future__ import annotations

import time
from enum import Enum, IntEnum
from typing import Dict, Iterable, List, Optional

from serial import EIGHTBITS, PARITY_NONE, STOPBITS_ONE, Serial

from control_readout.base.controller import Controller


# --------------------------------------------------------------------------- #
#  Protocol enums
# --------------------------------------------------------------------------- #
class HostCommand(str, Enum):
    """Commands sent from host to device (lowercase in protocol)."""
    GET_STATUS = "gs"
    GET_VELOCITY = "gv"
    SET_VELOCITY = "sv"

    HOME = "ho"
    MOVE_ABSOLUTE = "ma"
    MOVE_RELATIVE = "mr"

    GET_POSITION = "gp"
    STOP = "st"  # only applicable to some devices (e.g. continuous mode)


class ReplyCommand(str, Enum):
    """Commands sent from device to host (uppercase in protocol)."""
    STATUS = "GS"
    VELOCITY = "GV"
    POSITION = "PO"


class StatusCode(IntEnum):
    OK = 0
    COMMUNICATION_TIMEOUT = 1
    MECHANICAL_TIMEOUT = 2
    COMMAND_ERROR_OR_NOT_SUPPORTED = 3
    VALUE_OUT_OF_RANGE = 4
    MODULE_ISOLATED = 5
    MODULE_OUT_OF_ISOLATION = 6
    INITIALIZING_ERROR = 7
    THERMAL_ERROR = 8
    BUSY = 9
    SENSOR_ERROR = 10
    MOTOR_ERROR = 11
    OUT_OF_RANGE = 12
    OVER_CURRENT = 13


class HomeDirection(IntEnum):
    """Only used by some rotary devices; other devices ignore this parameter."""
    CW = 0
    CCW = 1


# --------------------------------------------------------------------------- #
#  Errors
# --------------------------------------------------------------------------- #
class ElliptecError(RuntimeError):
    def __init__(self, message: str, *, status: Optional[StatusCode] = None, reply: Optional[str] = None):
        super().__init__(message)
        self.status = status
        self.reply = reply


# --------------------------------------------------------------------------- #
#  Wire encode / parse helpers
# --------------------------------------------------------------------------- #
_HEX_DIGITS = "0123456789ABCDEF"


def _normalize_address(addr: str) -> str:
    if not isinstance(addr, str) or len(addr) != 1:
        raise ValueError(f"Address must be a single hex digit '0'..'F', got: {addr!r}")
    a = addr.upper()
    if a not in _HEX_DIGITS:
        raise ValueError(f"Address must be a hex digit '0'..'F', got: {addr!r}")
    return a


def _iter_addresses(min_addr: str, max_addr: str) -> Iterable[str]:
    mn = int(_normalize_address(min_addr), 16)
    mx = int(_normalize_address(max_addr), 16)
    if mn > mx:
        raise ValueError(f"min_address ({min_addr}) must be <= max_address ({max_addr})")
    for v in range(mn, mx + 1):
        yield f"{v:X}"


def _encode_u8_percent(percent: int) -> str:
    """Encode velocity compensation as two HEX ASCII digits, 0..100 (%) (e.g. 50 -> '32')."""
    if not isinstance(percent, int):
        raise TypeError("percent must be int")
    if not (0 <= percent <= 100):
        raise ValueError("percent must be in [0, 100]")
    return f"{percent:02X}"


def _encode_long32(value: int) -> str:
    """Encode signed 32-bit integer as 8 HEX ASCII digits (2's complement)."""
    if not isinstance(value, int):
        raise TypeError("value must be int")
    return f"{(value & 0xFFFFFFFF):08X}"


def _parse_status_reply(reply: str, address: str) -> StatusCode:
    # Example: "0GS00"
    if len(reply) < 5 or reply[0] != address or reply[1:3] != ReplyCommand.STATUS.value:
        raise ElliptecError("Unexpected status reply", reply=reply)
    code_hex = reply[3:5]
    try:
        code = int(code_hex, 16)
    except ValueError as e:
        raise ElliptecError("Malformed status code in reply", reply=reply) from e
    try:
        return StatusCode(code)
    except ValueError:
        # Reserved / unknown codes: keep it explicit
        return StatusCode.COMMAND_ERROR_OR_NOT_SUPPORTED


def _parse_velocity_reply(reply: str, address: str) -> int:
    # Example: "AGV64" => 100%
    if len(reply) < 5 or reply[0] != address or reply[1:3] != ReplyCommand.VELOCITY.value:
        raise ElliptecError("Unexpected velocity reply", reply=reply)
    try:
        return int(reply[3:5], 16)
    except ValueError as e:
        raise ElliptecError("Malformed velocity value in reply", reply=reply) from e


def _parse_position_reply(reply: str, address: str) -> int:
    # Example: "APO00003000" => 0x3000
    if len(reply) < 11 or reply[0] != address or reply[1:3] != ReplyCommand.POSITION.value:
        raise ElliptecError("Unexpected position reply", reply=reply)
    hex32 = reply[3:11]
    try:
        raw = int(hex32, 16)
    except ValueError as e:
        raise ElliptecError("Malformed position value in reply", reply=reply) from e
    # interpret as signed 32-bit
    if raw & 0x8000_0000:
        raw -= 0x1_0000_0000
    return raw


POLL_INTERVAL = float(0.2)
MAX_POLLS = int(400)


class ElliptecController(Controller):
    """Owns the Elliptec serial bus and a registry of addressed devices.

    The addressed device ``address`` for this controller is a single hex digit
    string ('0'..'F').
    """

    def __init__(self, port: str) -> None:
        super().__init__()
        self.port = port
        self._serial: Optional[Serial] = None
        #: latest known status per address (updated from GS polls)
        self._status: Dict[str, StatusCode] = {}

    # -- transport -------------------------------------------------------- #
    def _open(self) -> None:
        self._serial = Serial(
            port=self.port,
            baudrate=9600,
            bytesize=EIGHTBITS,
            parity=PARITY_NONE,
            stopbits=STOPBITS_ONE,
            timeout=0.5,
            write_timeout=0.5,
        )

    def _close(self) -> None:
        if self._serial is not None:
            try:
                self._serial.close()
            finally:
                self._serial = None

    @property
    def serial(self) -> Serial:
        if self._serial is None:
            raise ElliptecError("No open serial connection. Call connect() first.")
        return self._serial

    # -- status ----------------------------------------------------------- #
    def status(self, address: str) -> StatusCode:
        """Software-side latest known status for an address (from GS polls)."""
        return self._status.get(_normalize_address(address), StatusCode.OK)

    # -- low level -------------------------------------------------------- #
    def _readline(self) -> Optional[str]:
        raw = self.serial.read_until(b"\n")  # \r\n terminated
        if not raw:
            return None
        return raw.strip().decode("ascii", errors="replace")

    def _send_raw(self, cmd: str) -> None:
        # Elliptec uses fixed-length packets; do NOT append CRLF.
        self.serial.write(cmd.encode("ascii"))
        self.serial.flush()
        # Small pacing helps with some adapters
        time.sleep(0.05)

    def _read_one_status(self, address: str) -> Optional[StatusCode]:
        t0 = time.monotonic()
        while time.monotonic() - t0 < float(self.serial.timeout or 0.5):
            line = self._readline()
            if line is None:
                return None
            if len(line) >= 5 and line[0] == address and line[1:3] == ReplyCommand.STATUS.value:
                st = _parse_status_reply(line, address)
                self._status[address] = (
                    StatusCode.BUSY if st == StatusCode.MECHANICAL_TIMEOUT else st
                )
                return st
        return None

    def _wait_until_gs00(self, address: str) -> None:
        for _ in range(MAX_POLLS):
            self._send_raw(f"{address}{HostCommand.GET_STATUS.value}")
            st = self._read_one_status(address)
            if st is None:
                time.sleep(POLL_INTERVAL)
                continue
            if st == StatusCode.OK:
                self._status[address] = StatusCode.OK
                return
            if st in (StatusCode.BUSY, StatusCode.MECHANICAL_TIMEOUT):
                self._status[address] = StatusCode.BUSY
                time.sleep(POLL_INTERVAL)
                continue
            self._status[address] = st
            raise ElliptecError(f"Device status error while waiting: {st.name}", status=st)

        self._status[address] = StatusCode.COMMUNICATION_TIMEOUT
        raise ElliptecError(
            "Exceeded max GS polls without receiving GS00",
            status=StatusCode.COMMUNICATION_TIMEOUT,
        )

    def _send_and_wait_ok(self, address: str, cmd: str) -> None:
        with self._lock:
            self.serial.reset_input_buffer()
            self._status[address] = StatusCode.BUSY
            self._send_raw(cmd)
            self._wait_until_gs00(address)

    # -- discovery -------------------------------------------------------- #
    def find_addresses(self) -> List[str]:
        """Scan the bus and return the hex addresses that answered GS."""
        found: List[str] = []
        with self._lock:
            self.serial.write(b"\r")
            self.serial.flush()
            self.serial.reset_input_buffer()
            for a in _iter_addresses(_HEX_DIGITS[0], _HEX_DIGITS[-1]):
                try:
                    self.serial.reset_input_buffer()
                    self._send_raw(f"{a}{HostCommand.GET_STATUS.value}")
                    line = self._readline()
                    if (
                        line
                        and len(line) >= 5
                        and line[0] == a
                        and line[1:3] == ReplyCommand.STATUS.value
                    ):
                        found.append(a)
                except Exception:
                    continue
        return found

    def resolve_address(self, address: Optional[str] = None) -> str:
        """Return a single valid bus address, discovering one if not given."""
        if address is not None:
            return _normalize_address(address)
        addrs = self.find_addresses()
        if not addrs:
            raise ElliptecError(f"No Elliptec device found on {self.port} in address range.")
        if len(addrs) > 1:
            raise ElliptecError(
                f"Multiple Elliptec devices found on {self.port}: {addrs}. "
                f"Pass address=... to select one."
            )
        return addrs[0]

    # -- addressed commands ----------------------------------------------- #
    def get_status(self, address: str) -> StatusCode:
        with self._lock:
            self.serial.reset_input_buffer()
            self._send_raw(f"{address}{HostCommand.GET_STATUS.value}")
            st = self._read_one_status(address)
        if st is None:
            raise ElliptecError("No GS reply received")
        return st

    def get_speed(self, address: str) -> int:
        with self._lock:
            self.serial.reset_input_buffer()
            self._send_raw(f"{address}{HostCommand.GET_VELOCITY.value}")
            deadline = time.monotonic() + 2.0
            while time.monotonic() < deadline:
                line = self._readline()
                if line is None:
                    continue
                if len(line) >= 5 and line[0] == address and line[1:3] == ReplyCommand.VELOCITY.value:
                    return _parse_velocity_reply(line, address)
        raise ElliptecError("Timeout waiting for GV reply")

    def set_speed(self, address: str, percent: int) -> None:
        vv = _encode_u8_percent(percent)
        self._send_and_wait_ok(address, f"{address}{HostCommand.SET_VELOCITY.value}{vv}")

    def get_position_counts(self, address: str) -> int:
        with self._lock:
            self.serial.reset_input_buffer()
            self._send_raw(f"{address}{HostCommand.GET_POSITION.value}")
            deadline = time.monotonic() + 2.0
            while time.monotonic() < deadline:
                line = self._readline()
                if line is None:
                    continue
                if len(line) >= 11 and line[0] == address and line[1:3] == ReplyCommand.POSITION.value:
                    self._status[address] = StatusCode.OK
                    return _parse_position_reply(line, address)
        raise ElliptecError("Timeout waiting for PO reply")

    # -- Controller interface (opaque address == hex bus address) --------- #
    def home(self, address: str, direction: HomeDirection = HomeDirection.CW) -> None:
        self._send_and_wait_ok(address, f"{address}{HostCommand.HOME.value}{int(direction)}")

    def move_absolute(self, address: str, value: float) -> None:
        payload = _encode_long32(int(value))
        self._send_and_wait_ok(address, f"{address}{HostCommand.MOVE_ABSOLUTE.value}{payload}")

    def move_relative(self, address: str, delta: float) -> None:
        payload = _encode_long32(int(delta))
        self._send_and_wait_ok(address, f"{address}{HostCommand.MOVE_RELATIVE.value}{payload}")

    def get_position(self, address: str) -> float:
        return float(self.get_position_counts(address))

    def stop(self, address: str) -> None:
        self._send_and_wait_ok(address, f"{address}{HostCommand.STOP.value}")
