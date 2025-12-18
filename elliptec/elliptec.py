from __future__ import annotations

import time
from enum import Enum, IntEnum
from typing import Iterable, List, Optional

from serial import EIGHTBITS, PARITY_NONE, STOPBITS_ONE, Serial


# ----------------------------- Enums / Types ------------------------------


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


# ------------------------------- Exceptions -------------------------------


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
    """
    Encode velocity compensation as two HEX ASCII digits, representing 0..100 (%)
    (e.g. 50 -> '32', 100 -> '64').
    """
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
        # reserved / unknown status codes: keep as int-like via StatusCode? fall back:
        return StatusCode.BUSY if code == 9 else StatusCode.COMMAND_ERROR_OR_NOT_SUPPORTED


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


# ------------------------------ Base device -------------------------------


class ElliptecDeviceBase:
    """
    Minimal Elliptec base class using *only serial* (no Thorlabs DLL).
    Implements:
      - home()
      - move_relative()
      - move_absolute()
      - set_speed()  (sv velocity compensation)

    Note:
      - Address is a single hex digit '0'..'F'.
      - Replies are CRLF terminated and can arrive asynchronously while moving.
    """

    def __init__(
        self,
        port: str,
        *,
        address: Optional[str] = None,
        min_address: str = "0",
        max_address: str = "F",
    ) -> None:
        self._serial = self._open(port, timeout=float(0.5), write_timeout=float(0.5))

        if address is None:
            addrs = self._find_addresses(min_address=_normalize_address(min_address), max_address=_normalize_address(max_address))
            if not addrs:
                raise ElliptecError(
                    f"No Elliptec device found on {port} in address range {min_address}..{max_address}."
                )
            if len(addrs) > 1:
                raise ElliptecError(
                    f"Multiple Elliptec devices found on {port}: {addrs}. "
                    f"Pass address=... to select one."
                )
            self._address = addrs[0]
        else:
            self._address = _normalize_address(address)

    # --- lifecycle ---------------------------------------------------------

    def close(self) -> None:
        if getattr(self, "_serial", None) is not None and self._serial.is_open:
            self._serial.close()

    def __enter__(self) -> "ElliptecDeviceBase":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    # --- low-level serial --------------------------------------------------

    def _open(self, port: str, *, timeout: float, write_timeout: float) -> Serial:
        return Serial(
            port=port,
            baudrate=9600,
            bytesize=EIGHTBITS,
            parity=PARITY_NONE,
            stopbits=STOPBITS_ONE,
            timeout=timeout,
            write_timeout=write_timeout,
        )

    def _readline(self) -> Optional[str]:
        raw = self._serial.read_until(b"\n")  # replies end with \r\n
        if not raw:
            return None
        return raw.strip().decode("ascii", errors="replace")

    def _send_raw(self, cmd: str) -> None:
        # Command must NOT include CRLF; protocol uses fixed-length ASCII packets.
        self._serial.write(cmd.encode("ascii"))
        self._serial.flush()
        time.sleep(0.1)

    def _send_and_read_one(self, cmd: str, *, clear_rx: bool = False) -> Optional[str]:
        """
        Send command and read a *single* reply line (if any) within serial timeout.
        This is good for 'gs', 'gv', 'gp', 'sv' etc.
        """
        if clear_rx:
            self._serial.reset_input_buffer()
        self._send_raw(cmd)
        return self._readline()

    # --- discovery ---------------------------------------------------------

    def _find_addresses(self, *, min_address: str, max_address: str) -> List[str]:
        """
        Probe addresses in [min_address, max_address] by sending 'gs' and
        checking for a well-formed 'GS' reply.
        """
        found: List[str] = []

        # CR clears the receiving state machine and cancels incomplete commands.
        self._serial.write(b"\r")
        self._serial.flush()

        for a in _iter_addresses(min_address, max_address):
            r = self._send_and_read_one(f"{a}{HostCommand.GET_STATUS.value}")
            if r and len(r) >= 5 and r[0] == a and r[1:3] == ReplyCommand.STATUS.value:
                found.append(a)

        return found

    # --- basic getters -----------------------------------------------------

    @property
    def address(self) -> str:
        return self._address

    def get_status(self) -> StatusCode:
        r = self._send_and_read_one(f"{self._address}{HostCommand.GET_STATUS.value}", clear_rx=False)
        if r is None:
            raise ElliptecError("Timeout waiting for status reply")
        return _parse_status_reply(r, self._address)

    def get_speed(self) -> int:
        """Return velocity compensation in percent (0..100)."""
        r = self._send_and_read_one(f"{self._address}{HostCommand.GET_VELOCITY.value}")
        if r is None:
            raise ElliptecError("Timeout waiting for velocity reply")
        return _parse_velocity_reply(r, self._address)

    def get_position_counts(self) -> int:
        """Return current position as signed 32-bit encoder counts."""
        r = self._send_and_read_one(f"{self._address}{HostCommand.GET_POSITION.value}")
        if r is None:
            raise ElliptecError("Timeout waiting for position reply")
        return _parse_position_reply(r, self._address)

    # --- motion / control --------------------------------------------------

    def set_speed(self, percent: int) -> None:
        """
        Set velocity compensation (% of max) via 'sv'.
        Example: 50% -> Asv32. Device typically replies with AGS00.
        """
        vv = _encode_u8_percent(percent)
        r = self._send_and_read_one(f"{self._address}{HostCommand.SET_VELOCITY.value}{vv}")
        if r is None:
            raise ElliptecError("Timeout waiting for reply to set_speed")
        if r[1:3] == ReplyCommand.STATUS.value:
            st = _parse_status_reply(r, self._address)
            if st != StatusCode.OK:
                raise ElliptecError(f"Device returned error status {st.name} after set_speed", status=st, reply=r)
            return
        # Some firmware may answer differently; fall back to checking status
        st = self.get_status()
        if st != StatusCode.OK:
            raise ElliptecError(f"Device returned error status {st.name} after set_speed", status=st, reply=r)

    def home(self, direction: HomeDirection = HomeDirection.CW, *, timeout_s: float = 30.0) -> None:
        """
        Home the stage. For rotary stages, direction is used (0=CW, 1=CCW).
        For other devices, the direction byte is ignored by firmware.
        """
        self._serial.reset_input_buffer()
        self._send_raw(f"{self._address}{HostCommand.HOME.value}{int(direction)}")
        self._wait_until_done(timeout_s=timeout_s)

    def move_absolute(self, position_counts: int, *, timeout_s: float = 30.0) -> None:
        """Move to an absolute position in encoder counts (signed 32-bit)."""
        payload = _encode_long32(position_counts)
        self._serial.reset_input_buffer()
        self._send_raw(f"{self._address}{HostCommand.MOVE_ABSOLUTE.value}{payload}")
        self._wait_until_done(timeout_s=timeout_s)

    def move_relative(self, delta_counts: int, *, timeout_s: float = 30.0) -> None:
        """Move by a relative offset in encoder counts (signed 32-bit)."""
        payload = _encode_long32(delta_counts)
        self._serial.reset_input_buffer()
        self._send_raw(f"{self._address}{HostCommand.MOVE_RELATIVE.value}{payload}")
        self._wait_until_done(timeout_s=timeout_s)

    def stop(self, *, timeout_s: float = 5.0) -> None:
        """Send 'st' (only supported on some devices / modes)."""
        r = self._send_and_read_one(f"{self._address}{HostCommand.STOP.value}")
        if r is None:
            raise ElliptecError("Timeout waiting for reply to stop()")
        if r[1:3] == ReplyCommand.STATUS.value:
            st = _parse_status_reply(r, self._address)
            if st not in (StatusCode.OK, StatusCode.BUSY):
                raise ElliptecError(f"Device returned error status {st.name} after stop()", status=st, reply=r)
            # if BUSY is returned first, poll until OK
            if st == StatusCode.BUSY:
                self._wait_until_done(timeout_s=timeout_s)
            return

    # --- internal waiting --------------------------------------------------

    def _wait_until_done(self, *, timeout_s: float) -> None:
        """
        Wait until the device is no longer BUSY after a motion command.

        The device may stream back:
          - GS09 (busy) while moving
          - PO........ (final position) when done
          - GSxx error status
        """
        deadline = time.monotonic() + float(timeout_s)

        last_reply: Optional[str] = None
        while time.monotonic() < deadline:
            line = self._readline()
            if line is None:
                # no async line; poll status
                st = self.get_status()
                if st == StatusCode.BUSY:
                    continue
                if st != StatusCode.OK:
                    raise ElliptecError(f"Motion ended with error {st.name}", status=st, reply=last_reply)
                return

            last_reply = line

            # filter out replies from other addresses (multi-drop bus)
            if not line or line[0] != self._address or len(line) < 3:
                continue

            cmd = line[1:3]
            if cmd == ReplyCommand.STATUS.value:
                st = _parse_status_reply(line, self._address)
                if st == StatusCode.BUSY:
                    continue
                if st != StatusCode.OK:
                    raise ElliptecError(f"Motion ended with error {st.name}", status=st, reply=line)
                return

            if cmd == ReplyCommand.POSITION.value:
                # done (position returned)
                return

        raise ElliptecError("Timeout waiting for motion to complete", reply=last_reply)
