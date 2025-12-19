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


# ------------------------------ Base device -------------------------------


class ElliptecDeviceBase:
    """
    Minimal Elliptec base class using *only serial* (no Thorlabs DLL).

    Implements:
      - home()
      - move_relative()
      - move_absolute()
      - set_speed()  (sv velocity compensation)

    Notes:
      - Address is a single hex digit '0'..'F'.
      - Replies are CRLF terminated and can arrive asynchronously while moving.
      - During motion, some controllers do NOT reliably answer 'gs' polls.
        Therefore _wait_until_done() primarily waits for a PO reply.
      - MECHANICAL_TIMEOUT (GS02) is treated as a soft/busy-like status.
    """

    def __init__(
        self,
        port: str,
        *,
        address: Optional[str] = None,
        min_address: str = "0",
        max_address: str = "F",
    ) -> None:
        # Keep the constructor params the same as your previous version.
        self._serial = self._open(port, timeout=float(0.5), write_timeout=float(0.5))

        self._current_status: StatusCode = StatusCode.OK

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

    # --- status tracking ---------------------------------------------------

    @property
    def status(self) -> StatusCode:
        """
        Software-side current status.

        - Set to BUSY when a motion command is issued.
        - Set to OK when a PO reply is received.
        - Updated when GS replies are seen.
        """
        return self._current_status

    def _set_status(self, st: StatusCode) -> None:
        self._current_status = st

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
        """
        Send a raw ASCII command. Do NOT append CRLF.
        (Elliptec protocol uses fixed-length packets.)
        """
        self._serial.write(cmd.encode("ascii"))
        self._serial.flush()
        # Give the device a brief moment; helps on some USB-serial adapters.
        time.sleep(0.05)

    def _query_reply(
        self,
        cmd: str,
        *,
        expect_addr: str,
        expect_reply: ReplyCommand,
        clear_rx: bool = False,
        timeout_s: Optional[float] = None,
    ) -> str:
        """
        Send a command and return the first matching reply for (expect_addr, expect_reply),
        ignoring unrelated lines (other addresses, different reply types).
        """
        if clear_rx:
            self._serial.reset_input_buffer()

        self._send_raw(cmd)

        per_line_timeout = float(self._serial.timeout or 0.5)
        deadline = time.monotonic() + float(timeout_s if timeout_s is not None else max(1.0, 2.0 * per_line_timeout))

        while time.monotonic() < deadline:
            line = self._readline()
            if line is None:
                continue
            if not line or len(line) < 3:
                continue
            if line[0] != expect_addr:
                continue
            if line[1:3] != expect_reply.value:
                continue
            return line

        raise ElliptecError(f"Timeout waiting for {expect_reply.value} reply", reply=None)

    # --- discovery ---------------------------------------------------------

    def _find_addresses(self, *, min_address: str, max_address: str) -> List[str]:
        """
        Probe addresses in [min_address, max_address] by sending 'gs' and
        checking for a well-formed 'GS' reply.
        """
        found: List[str] = []

        # CR clears the device receiving state machine and cancels incomplete commands.
        self._serial.write(b"\r")
        self._serial.flush()
        self._serial.reset_input_buffer()

        for a in _iter_addresses(min_address, max_address):
            try:
                r = self._query_reply(
                    f"{a}{HostCommand.GET_STATUS.value}",
                    expect_addr=a,
                    expect_reply=ReplyCommand.STATUS,
                    clear_rx=False,
                )
            except ElliptecError:
                continue
            if r:
                found.append(a)

        return found

    # --- basic getters -----------------------------------------------------

    @property
    def address(self) -> str:
        return self._address

    def get_status(self) -> StatusCode:
        """
        Query device status (GSxx). Note that while moving some controllers may
        not reply reliably; use the .status property for software-side state.
        """
        r = self._query_reply(
            f"{self._address}{HostCommand.GET_STATUS.value}",
            expect_addr=self._address,
            expect_reply=ReplyCommand.STATUS,
            clear_rx=False,  # do not flush away async GS/PO
        )
        st = _parse_status_reply(r, self._address)
        self._set_status(st if st != StatusCode.MECHANICAL_TIMEOUT else StatusCode.BUSY)
        return st

    def get_speed(self) -> int:
        """Return velocity compensation in percent (0..100)."""
        r = self._query_reply(
            f"{self._address}{HostCommand.GET_VELOCITY.value}",
            expect_addr=self._address,
            expect_reply=ReplyCommand.VELOCITY,
            clear_rx=False,
        )
        return _parse_velocity_reply(r, self._address)

    def get_position_counts(self) -> int:
        """Return current position as signed 32-bit encoder counts."""
        r = self._query_reply(
            f"{self._address}{HostCommand.GET_POSITION.value}",
            expect_addr=self._address,
            expect_reply=ReplyCommand.POSITION,
            clear_rx=False,
        )
        # If we got a PO, device is not busy anymore.
        self._set_status(StatusCode.OK)
        return _parse_position_reply(r, self._address)

    # --- motion / control --------------------------------------------------

    def set_speed(self, percent: int) -> None:
        """
        Set velocity compensation (% of max) via 'sv'.
        Example: 50% -> Asv32. Device typically replies with AGS00.
        """
        vv = _encode_u8_percent(percent)
        r = self._query_reply(
            f"{self._address}{HostCommand.SET_VELOCITY.value}{vv}",
            expect_addr=self._address,
            expect_reply=ReplyCommand.STATUS,
            clear_rx=False,
        )
        st = _parse_status_reply(r, self._address)
        # Some devices may return BUSY/MECH_TIMEOUT briefly; treat that as soft.
        if st in (StatusCode.BUSY, StatusCode.MECHANICAL_TIMEOUT):
            self._set_status(StatusCode.BUSY)
            return
        self._set_status(st)
        if st != StatusCode.OK:
            raise ElliptecError(f"Device returned error status {st.name} after set_speed", status=st, reply=r)

    def home(self, direction: HomeDirection = HomeDirection.CW, *, timeout_s: float = 30.0) -> None:
        """
        Home the stage. For rotary stages, direction is used (0=CW, 1=CCW).
        For other devices, the direction byte is ignored by firmware.
        """
        # Clear stale replies so we don't accidentally consume an old PO.
        self._serial.reset_input_buffer()
        self._set_status(StatusCode.BUSY)
        self._send_raw(f"{self._address}{HostCommand.HOME.value}{int(direction)}")
        self._wait_until_po(timeout_s=timeout_s)

    def move_absolute(self, position_counts: int, *, timeout_s: float = 30.0) -> None:
        """Move to an absolute position in encoder counts (signed 32-bit)."""
        payload = _encode_long32(position_counts)
        self._serial.reset_input_buffer()
        self._set_status(StatusCode.BUSY)
        self._send_raw(f"{self._address}{HostCommand.MOVE_ABSOLUTE.value}{payload}")
        self._wait_until_po(timeout_s=timeout_s)

    def move_relative(self, delta_counts: int, *, timeout_s: float = 30.0) -> None:
        """Move by a relative offset in encoder counts (signed 32-bit)."""
        payload = _encode_long32(delta_counts)
        self._serial.reset_input_buffer()
        self._set_status(StatusCode.BUSY)
        self._send_raw(f"{self._address}{HostCommand.MOVE_RELATIVE.value}{payload}")
        self._wait_until_po(timeout_s=timeout_s)

    def stop(self, *, timeout_s: float = 5.0) -> None:
        """Send 'st' (only supported on some devices / modes)."""
        r = self._query_reply(
            f"{self._address}{HostCommand.STOP.value}",
            expect_addr=self._address,
            expect_reply=ReplyCommand.STATUS,
            clear_rx=False,
            timeout_s=timeout_s,
        )
        st = _parse_status_reply(r, self._address)
        if st in (StatusCode.OK, StatusCode.BUSY, StatusCode.MECHANICAL_TIMEOUT):
            self._set_status(StatusCode.BUSY if st != StatusCode.OK else StatusCode.OK)
            return
        self._set_status(st)
        raise ElliptecError(f"Device returned error status {st.name} after stop()", status=st, reply=r)

    # --- internal waiting --------------------------------------------------

    def _wait_until_po(self, *, timeout_s: float) -> None:
        """
        Wait ONLY for a PO reply from the device (preferred).

        We still *observe* any GS replies that arrive:
          - GS09 (busy) -> keep waiting
          - GS02 (mechanical timeout) -> treated as busy, keep waiting
          - GS00 -> keep waiting until PO (some firmwares send GS00 before PO)
          - GS(other) -> raise

        No 'gs' polling is performed here.
        """
        deadline = time.monotonic() + float(timeout_s)
        last_reply: Optional[str] = None

        while time.monotonic() < deadline:
            line = self._readline()
            if line is None:
                continue

            last_reply = line

            if not line or len(line) < 3 or line[0] != self._address:
                continue

            cmd = line[1:3]

            if cmd == ReplyCommand.POSITION.value:
                self._set_status(StatusCode.OK)
                return

            if cmd == ReplyCommand.STATUS.value:
                st = _parse_status_reply(line, self._address)

                # Treat MECHANICAL_TIMEOUT as soft/busy-like.
                if st in (StatusCode.BUSY, StatusCode.MECHANICAL_TIMEOUT):
                    self._set_status(StatusCode.BUSY)
                    continue

                # Some devices might send OK while we're still waiting for PO.
                if st == StatusCode.OK:
                    # don't flip to OK yet; motion completion is PO in this mode
                    continue

                self._set_status(st)
                raise ElliptecError(f"Motion ended with error {st.name}", status=st, reply=line)

            # Ignore other reply types (GV, etc.)
            continue

        # No PO received within overall timeout
        self._set_status(StatusCode.COMMUNICATION_TIMEOUT)
        raise ElliptecError("Timeout waiting for PO (motion completion)", reply=last_reply)
