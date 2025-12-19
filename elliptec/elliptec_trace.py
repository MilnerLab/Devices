from __future__ import annotations

import time
from typing import List, Optional

from serial import EIGHTBITS, PARITY_NONE, STOPBITS_ONE, Serial

from elliptec.base.enums import HomeDirection, HostCommand, ReplyCommand, StatusCode
from elliptec.base.exceptions import ElliptecError
from elliptec.base.helpers import _encode_long32, _encode_u8_percent, _iter_addresses, _normalize_address, _parse_position_reply, _parse_status_reply, _parse_velocity_reply


class ElliptecDeviceBase:
    """
    Minimal Elliptec base class using *only serial* (no Thorlabs DLL).

    Constructor params (unchanged):
      __init__(port, *, address=None, min_address="0", max_address="F")

    Behaviour:
      - Motion commands wait for PO using _query_reply(..., expect_reply=PO, watch_status=True)
      - MECHANICAL_TIMEOUT (GS02) is treated as BUSY while waiting.
      - Some firmwares may not emit PO for HOME; we fall back to one gp query.
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
        Software-side status for your UI.

        - Set to BUSY when a motion command is issued.
        - Set to OK when a PO is received (or after successful home fallback).
        - Updated when GS replies are observed.
        """
        return self._current_status

    def _set_status(self, st: StatusCode) -> None:
        # For UI purposes treat mechanical timeout as busy-like.
        self._current_status = StatusCode.BUSY if st == StatusCode.MECHANICAL_TIMEOUT else st

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
        time.sleep(0.05)

    def _query_reply(
        self,
        cmd: str,
        *,
        expect_addr: str,
        expect_reply: ReplyCommand,
        clear_rx: bool = False,
        timeout_s: Optional[float] = None,
        watch_status: bool = False,
    ) -> str:
        """
        Send cmd and return the first matching reply (expect_addr + expect_reply).

        If watch_status=True while waiting for PO:
          - observe GS replies for the same address,
          - treat BUSY (09) + MECHANICAL_TIMEOUT (02) as keep-waiting,
          - ignore OK (00) (keep waiting for PO),
          - raise on any other GS error.
        """
        if clear_rx:
            self._serial.reset_input_buffer()

        self._send_raw(cmd)

        per_line_timeout = float(self._serial.timeout or 0.5)
        deadline = time.monotonic() + float(timeout_s if timeout_s is not None else max(1.0, 2.0 * per_line_timeout))
        last_line: Optional[str] = None

        while time.monotonic() < deadline:
            line = self._readline()
            if line is None:
                continue
            last_line = line

            if not line or len(line) < 3:
                continue
            if line[0] != expect_addr:
                continue

            reply_type = line[1:3]

            # Optional observation of GS while waiting for PO
            if watch_status and reply_type == ReplyCommand.STATUS.value:
                st = _parse_status_reply(line, expect_addr)
                self._set_status(st)
                if st in (StatusCode.BUSY, StatusCode.MECHANICAL_TIMEOUT):
                    continue
                if st == StatusCode.OK:
                    # Some firmwares emit GS00 while still moving; keep waiting for PO.
                    continue
                raise ElliptecError(f"Motion ended with error {st.name}", status=st, reply=line)

            if reply_type != expect_reply.value:
                continue

            # Matched
            if expect_reply == ReplyCommand.POSITION:
                self._set_status(StatusCode.OK)
            return line

        # Timeout
        if watch_status:
            self._set_status(StatusCode.COMMUNICATION_TIMEOUT)
        raise ElliptecError(f"Timeout waiting for {expect_reply.value} reply", reply=last_line)

    # --- discovery ---------------------------------------------------------

    def _find_addresses(self, *, min_address: str, max_address: str) -> List[str]:
        found: List[str] = []

        # Clear receiver state machine / stale bytes
        self._serial.write(b"\r")
        self._serial.flush()
        self._serial.reset_input_buffer()

        for a in _iter_addresses(min_address, max_address):
            try:
                _ = self._query_reply(
                    f"{a}{HostCommand.GET_STATUS.value}",
                    expect_addr=a,
                    expect_reply=ReplyCommand.STATUS,
                    clear_rx=False,
                    timeout_s=0.5,
                    watch_status=False,
                )
            except ElliptecError:
                continue
            found.append(a)

        return found

    # --- basic getters -----------------------------------------------------

    @property
    def address(self) -> str:
        return self._address

    def get_status(self) -> StatusCode:
        r = self._query_reply(
            f"{self._address}{HostCommand.GET_STATUS.value}",
            expect_addr=self._address,
            expect_reply=ReplyCommand.STATUS,
            clear_rx=False,
        )
        st = _parse_status_reply(r, self._address)
        self._set_status(st)
        return st

    def get_speed(self) -> int:
        r = self._query_reply(
            f"{self._address}{HostCommand.GET_VELOCITY.value}",
            expect_addr=self._address,
            expect_reply=ReplyCommand.VELOCITY,
            clear_rx=False,
        )
        return _parse_velocity_reply(r, self._address)

    def get_position_counts(self) -> int:
        r = self._query_reply(
            f"{self._address}{HostCommand.GET_POSITION.value}",
            expect_addr=self._address,
            expect_reply=ReplyCommand.POSITION,
            clear_rx=False,
        )
        return _parse_position_reply(r, self._address)

    # --- motion / control --------------------------------------------------

    def set_speed(self, percent: int) -> None:
        vv = _encode_u8_percent(percent)
        r = self._query_reply(
            f"{self._address}{HostCommand.SET_VELOCITY.value}{vv}",
            expect_addr=self._address,
            expect_reply=ReplyCommand.STATUS,
            clear_rx=False,
        )
        st = _parse_status_reply(r, self._address)
        self._set_status(st)
        if st not in (StatusCode.OK, StatusCode.BUSY, StatusCode.MECHANICAL_TIMEOUT):
            raise ElliptecError(f"Device returned error status {st.name} after set_speed", status=st, reply=r)

    def home(self, direction: HomeDirection = HomeDirection.CW, *, timeout_s: float = 30.0) -> None:
        # Clear stale replies so we don't accidentally consume an old PO.
        self._serial.reset_input_buffer()
        self._set_status(StatusCode.BUSY)

        try:
            # Prefer PO if the firmware emits it.
            _ = self._query_reply(
                f"{self._address}{HostCommand.HOME.value}{int(direction)}",
                expect_addr=self._address,
                expect_reply=ReplyCommand.POSITION,
                clear_rx=False,
                timeout_s=timeout_s,
                watch_status=True,
            )
            return
        except ElliptecError:
            # Fallback: some firmwares appear to not push PO for home; query gp once.
            _ = self._query_reply(
                f"{self._address}{HostCommand.GET_POSITION.value}",
                expect_addr=self._address,
                expect_reply=ReplyCommand.POSITION,
                clear_rx=False,
                timeout_s=min(5.0, timeout_s),
                watch_status=False,
            )
            self._set_status(StatusCode.OK)

    def move_absolute(self, position_counts: int, *, timeout_s: float = 30.0) -> None:
        payload = _encode_long32(position_counts)
        self._serial.reset_input_buffer()
        self._set_status(StatusCode.BUSY)

        _ = self._query_reply(
            f"{self._address}{HostCommand.MOVE_ABSOLUTE.value}{payload}",
            expect_addr=self._address,
            expect_reply=ReplyCommand.POSITION,
            clear_rx=False,
            timeout_s=timeout_s,
            watch_status=True,
        )

    def move_relative(self, delta_counts: int, *, timeout_s: float = 30.0) -> None:
        payload = _encode_long32(delta_counts)
        self._serial.reset_input_buffer()
        self._set_status(StatusCode.BUSY)

        _ = self._query_reply(
            f"{self._address}{HostCommand.MOVE_RELATIVE.value}{payload}",
            expect_addr=self._address,
            expect_reply=ReplyCommand.POSITION,
            clear_rx=False,
            timeout_s=timeout_s,
            watch_status=True,
        )

    def stop(self, *, timeout_s: float = 5.0) -> None:
        r = self._query_reply(
            f"{self._address}{HostCommand.STOP.value}",
            expect_addr=self._address,
            expect_reply=ReplyCommand.STATUS,
            clear_rx=False,
            timeout_s=timeout_s,
        )
        st = _parse_status_reply(r, self._address)
        self._set_status(st)
        if st not in (StatusCode.OK, StatusCode.BUSY, StatusCode.MECHANICAL_TIMEOUT):
            raise ElliptecError(f"Device returned error status {st.name} after stop()", status=st, reply=r)
