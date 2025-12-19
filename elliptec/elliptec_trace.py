from __future__ import annotations

import time
from enum import Enum, IntEnum
from typing import Iterable, List, Optional

from serial import EIGHTBITS, PARITY_NONE, STOPBITS_ONE, Serial
from elliptec.base.enums import HomeDirection, HostCommand, ReplyCommand, StatusCode
from elliptec.base.exceptions import ElliptecError
from elliptec.base.helpers import _encode_long32, _encode_u8_percent, _iter_addresses, _normalize_address, _parse_position_reply, _parse_status_reply, _parse_velocity_reply

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
