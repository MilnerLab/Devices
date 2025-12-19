from __future__ import annotations

import time
from enum import Enum, IntEnum
from typing import Iterable, List, Optional

from serial import EIGHTBITS, PARITY_NONE, STOPBITS_ONE, Serial


# ----------------------------- Enums / Types ------------------------------


class HostCommand(str, Enum):
    """Commands sent from host to device (lowercase)."""
    GET_STATUS = "gs"
    GET_VELOCITY = "gv"
    SET_VELOCITY = "sv"
    HOME = "ho"
    MOVE_ABSOLUTE = "ma"
    MOVE_RELATIVE = "mr"
    GET_POSITION = "gp"
    STOP = "st"


class ReplyCommand(str, Enum):
    """Replies sent from device to host (uppercase)."""
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
    CW = 0
    CCW = 1


class ElliptecError(RuntimeError):
    def __init__(self, message: str, *, status: Optional[StatusCode] = None, reply: Optional[str] = None):
        super().__init__(message)
        self.status = status
        self.reply = reply


# ---------------------------- Helper functions ----------------------------


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
    """Velocity compensation is encoded as two HEX ASCII digits: 0..100 -> 00..64."""
    if not isinstance(percent, int):
        raise TypeError("percent must be int")
    if not (0 <= percent <= 100):
        raise ValueError("percent must be in [0, 100]")
    return f"{percent:02X}"


def _encode_long32(value: int) -> str:
    """Encode signed 32-bit integer as exactly 8 HEX ASCII digits (2's complement)."""
    if not isinstance(value, int):
        raise TypeError("value must be int")
    return f"{(value & 0xFFFFFFFF):08X}"


def _parse_status_reply(line: str, address: str) -> StatusCode:
    # Example: "0GS00"
    if len(line) < 5 or line[0] != address or line[1:3] != ReplyCommand.STATUS.value:
        raise ElliptecError("Unexpected status reply", reply=line)
    try:
        code = int(line[3:5], 16)
    except ValueError as e:
        raise ElliptecError("Malformed status code", reply=line) from e
    try:
        return StatusCode(code)
    except ValueError:
        return StatusCode.COMMAND_ERROR_OR_NOT_SUPPORTED


def _parse_velocity_reply(line: str, address: str) -> int:
    if len(line) < 5 or line[0] != address or line[1:3] != ReplyCommand.VELOCITY.value:
        raise ElliptecError("Unexpected velocity reply", reply=line)
    try:
        return int(line[3:5], 16)
    except ValueError as e:
        raise ElliptecError("Malformed velocity reply", reply=line) from e


def _parse_position_reply(line: str, address: str) -> int:
    if len(line) < 11 or line[0] != address or line[1:3] != ReplyCommand.POSITION.value:
        raise ElliptecError("Unexpected position reply", reply=line)
    hex32 = line[3:11]
    try:
        raw = int(hex32, 16)
    except ValueError as e:
        raise ElliptecError("Malformed position reply", reply=line) from e
    if raw & 0x8000_0000:
        raw -= 0x1_0000_0000
    return raw


# ------------------------------- Main class -------------------------------


class ElliptecDevice:
    """
    Very simple Elliptec serial driver:

    **Rule**: after sending ANY command, we poll GS at a fixed frequency and only
    continue once we receive GS00. A hard-coded max poll count prevents endless loops.

    Notes:
    - Some devices do not reply to GS while moving; then we keep polling until one
      GS reply arrives (or max polls are exhausted).
    - While waiting, we ignore non-GS lines (e.g. PO, GV).
    """

    def __init__(
        self,
        port: str,
        *,
        address: Optional[str] = None,
        min_address: str = "0",
        max_address: str = "F",
        poll_interval_s: float = 0.2,
        max_polls: int = 400,
    ) -> None:
        self._serial = Serial(
            port=port,
            baudrate=9600,
            bytesize=EIGHTBITS,
            parity=PARITY_NONE,
            stopbits=STOPBITS_ONE,
            timeout=0.5,
            write_timeout=0.5,
        )

        self.poll_interval_s = float(poll_interval_s)
        self.max_polls = int(max_polls)

        if address is None:
            addrs = self._find_addresses(
                min_address=_normalize_address(min_address),
                max_address=_normalize_address(max_address),
            )
            if not addrs:
                raise ElliptecError(
                    f"No Elliptec device found on {port} in address range {min_address}..{max_address}."
                )
            if len(addrs) > 1:
                raise ElliptecError(
                    f"Multiple Elliptec devices found on {port}: {addrs}. Pass address=... to select one."
                )
            self._address = addrs[0]
        else:
            self._address = _normalize_address(address)

        self._status: StatusCode = StatusCode.OK

    # --- lifecycle ---------------------------------------------------------

    def close(self) -> None:
        if self._serial.is_open:
            self._serial.close()

    def __enter__(self) -> "ElliptecDevice":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    # --- properties --------------------------------------------------------

    @property
    def address(self) -> str:
        return self._address

    @property
    def status(self) -> StatusCode:
        """Software-side latest known status (updated from GS polls)."""
        return self._status

    # --- low level ---------------------------------------------------------

    def _readline(self) -> Optional[str]:
        raw = self._serial.read_until(b"\n")  # \r\n terminated
        if not raw:
            return None
        return raw.strip().decode("ascii", errors="replace")

    def _send_raw(self, cmd: str) -> None:
        # Elliptec uses fixed-length packets; do NOT append CRLF.
        self._serial.write(cmd.encode("ascii"))
        self._serial.flush()
        # Small pacing helps with some adapters
        time.sleep(0.05)

    def _read_one_status(self) -> Optional[StatusCode]:
        """
        Read lines until we either see a GS for our address or we hit serial timeout.
        Returns:
          - StatusCode if a GS line was seen
          - None if nothing arrived / no GS line arrived during this read window
        """
        t0 = time.monotonic()
        # Try to read a few lines within one serial timeout window
        while time.monotonic() - t0 < float(self._serial.timeout or 0.5):
            line = self._readline()
            if line is None:
                return None
            if len(line) >= 5 and line[0] == self._address and line[1:3] == ReplyCommand.STATUS.value:
                st = _parse_status_reply(line, self._address)
                # Treat mechanical timeout like busy for our control loop
                self._status = StatusCode.BUSY if st == StatusCode.MECHANICAL_TIMEOUT else st
                return st
            # ignore other lines (PO/GV/etc.)
        return None

    def _wait_until_gs00(self) -> None:
        """
        Poll GS repeatedly until we receive GS00, or until max_polls is exceeded.
        """
        for _ in range(self.max_polls):
            self._send_raw(f"{self._address}{HostCommand.GET_STATUS.value}")
            st = self._read_one_status()
            if st is None:
                time.sleep(self.poll_interval_s)
                continue

            if st == StatusCode.OK:
                self._status = StatusCode.OK
                return

            # Busy-like conditions: keep waiting
            if st in (StatusCode.BUSY, StatusCode.MECHANICAL_TIMEOUT):
                self._status = StatusCode.BUSY
                time.sleep(self.poll_interval_s)
                continue

            # Any other status is a hard failure
            self._status = st
            raise ElliptecError(f"Device status error while waiting: {st.name}", status=st)

        self._status = StatusCode.COMMUNICATION_TIMEOUT
        raise ElliptecError("Exceeded max GS polls without receiving GS00", status=self._status)

    def _send_and_wait_ok(self, cmd: str) -> None:
        """
        Send any command, then enforce the 'only proceed after GS00' policy.
        """
        # Clear stale replies, to avoid consuming an old GS00 immediately.
        self._serial.reset_input_buffer()
        self._status = StatusCode.BUSY

        self._send_raw(cmd)
        self._wait_until_gs00()

    # --- discovery ---------------------------------------------------------

    def _find_addresses(self, *, min_address: str, max_address: str) -> List[str]:
        found: List[str] = []

        # Clear receiver state machine and stale bytes
        self._serial.write(b"\r")
        self._serial.flush()
        self._serial.reset_input_buffer()

        for a in _iter_addresses(min_address, max_address):
            try:
                self._serial.reset_input_buffer()
                self._send_raw(f"{a}{HostCommand.GET_STATUS.value}")
                line = self._readline()
                if line and len(line) >= 5 and line[0] == a and line[1:3] == ReplyCommand.STATUS.value:
                    found.append(a)
            except Exception:
                continue

        return found

    # --- public commands ---------------------------------------------------

    def get_status(self) -> StatusCode:
        self._serial.reset_input_buffer()
        self._send_raw(f"{self._address}{HostCommand.GET_STATUS.value}")
        st = self._read_one_status()
        if st is None:
            raise ElliptecError("No GS reply received")
        return st

    def get_speed(self) -> int:
        self._serial.reset_input_buffer()
        self._send_raw(f"{self._address}{HostCommand.GET_VELOCITY.value}")
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            line = self._readline()
            if line is None:
                continue
            if len(line) >= 5 and line[0] == self._address and line[1:3] == ReplyCommand.VELOCITY.value:
                return _parse_velocity_reply(line, self._address)
        raise ElliptecError("Timeout waiting for GV reply")

    def set_speed(self, percent: int) -> None:
        vv = _encode_u8_percent(percent)
        self._send_and_wait_ok(f"{self._address}{HostCommand.SET_VELOCITY.value}{vv}")

    def get_position_counts(self) -> int:
        self._serial.reset_input_buffer()
        self._send_raw(f"{self._address}{HostCommand.GET_POSITION.value}")
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            line = self._readline()
            if line is None:
                continue
            if len(line) >= 11 and line[0] == self._address and line[1:3] == ReplyCommand.POSITION.value:
                # If we can read position, device is idle
                self._status = StatusCode.OK
                return _parse_position_reply(line, self._address)
        raise ElliptecError("Timeout waiting for PO reply")

    def home(self, direction: HomeDirection = HomeDirection.CW) -> None:
        self._send_and_wait_ok(f"{self._address}{HostCommand.HOME.value}{int(direction)}")

    def move_relative(self, delta_counts: int) -> None:
        payload = _encode_long32(delta_counts)
        self._send_and_wait_ok(f"{self._address}{HostCommand.MOVE_RELATIVE.value}{payload}")

    def move_absolute(self, position_counts: int) -> None:
        payload = _encode_long32(position_counts)
        self._send_and_wait_ok(f"{self._address}{HostCommand.MOVE_ABSOLUTE.value}{payload}")

    def stop(self) -> None:
        # stop is optional in firmware; if unsupported you'll get a non-OK status
        self._send_and_wait_ok(f"{self._address}{HostCommand.STOP.value}")
