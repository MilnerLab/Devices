from __future__ import annotations

import math
import time
import logging
import threading
from dataclasses import dataclass
from datetime import datetime
from collections import deque
from enum import Enum, IntEnum
from typing import Callable, Deque, List, Optional, Tuple

from serial import EIGHTBITS, PARITY_NONE, STOPBITS_ONE, Serial


# =============================================================================
# Elliptec protocol helpers + a serial-tracing base class (threaded RX)
#
# v2 changes (important for your issue):
# - Strict packet-length validation for fixed-length commands.
#   -> prevents desync bugs like sending 9 hex digits for a 32-bit "long".
# - Optional resync helpers (send '\r' + flush) for recovery.
# =============================================================================


class HostCommand(str, Enum):
    """Lowercase host commands (sent to device)."""
    GET_STATUS = "gs"
    HOME = "ho"
    MOVE_ABSOLUTE = "ma"
    MOVE_RELATIVE = "mr"
    SET_VELOCITY = "sv"
    GET_VELOCITY = "gv"
    GET_INFO = "in"          # useful to verify device type + scaling


class ReplyCommand(str, Enum):
    """Uppercase reply tokens (sent by device)."""
    STATUS = "GS"
    POSITION = "PO"
    VELOCITY = "GV"
    INFO = "IN"


class StatusCode(IntEnum):
    OK = 0
    COMMUNICATION_ERROR = 1
    MECHANICAL_TIMEOUT = 2
    COMMAND_ERROR = 3
    VALUE_OUT_OF_RANGE = 4
    MODULE_ISOLATED = 5
    MODULE_OUT_OF_ISOLATION = 6
    INIT_ERROR = 7
    THERMAL_ERROR = 8
    BUSY = 9
    SENSOR_ERROR = 10
    MOTOR_ERROR = 11
    OUT_OF_RANGE = 12
    OVERCURRENT_ERROR = 13
    UNKNOWN = 255


class HomeDirection(IntEnum):
    CW = 0
    CCW = 1


@dataclass(frozen=True)
class SerialTraceEvent:
    ts_monotonic: float
    ts_iso: str
    direction: str  # "TX", "RX", "INFO"
    text: str
    raw: bytes = b""


class ElliptecError(RuntimeError):
    def __init__(self, message: str, *, status: Optional[StatusCode] = None, reply: Optional[str] = None):
        super().__init__(message)
        self.status = status
        self.reply = reply


_HEX_DIGITS = "0123456789ABCDEF"


def _ts_iso() -> str:
    return datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]


def _normalize_address(addr: str) -> str:
    if not isinstance(addr, str) or len(addr) != 1:
        raise ValueError(f"Address must be a single hex digit '0'..'F', got: {addr!r}")
    a = addr.upper()
    if a not in _HEX_DIGITS:
        raise ValueError(f"Address must be a single hex digit '0'..'F', got: {addr!r}")
    return a


def _address_range(min_addr: str, max_addr: str) -> List[str]:
    mn = int(_normalize_address(min_addr), 16)
    mx = int(_normalize_address(max_addr), 16)
    if mn > mx:
        raise ValueError(f"min_address must be <= max_address, got {min_addr}..{max_addr}")
    return [format(i, "X") for i in range(mn, mx + 1)]


def _encode_long32(value: int) -> str:
    """Encode signed 32-bit int as 8 hex digits (two's complement)."""
    v = int(value) & 0xFFFFFFFF
    return f"{v:08X}"


def _hexdump(raw: bytes) -> str:
    return " ".join(f"{b:02x}" for b in raw)


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
        return StatusCode.UNKNOWN


def _parse_velocity_reply(reply: str, address: str) -> int:
    # Example: "0GV32" (hex percent)
    if len(reply) < 5 or reply[0] != address or reply[1:3] != ReplyCommand.VELOCITY.value:
        raise ElliptecError("Unexpected velocity reply", reply=reply)
    try:
        return int(reply[3:5], 16)
    except ValueError as e:
        raise ElliptecError("Malformed velocity reply", reply=reply) from e


def _percent_to_hex_byte(percent: int) -> str:
    p = int(percent)
    if p < 0 or p > 100:
        raise ValueError("percent must be in [0, 100]")
    return f"{p:02X}"


class ElliptecDeviceBase:
    """
    Serial-only Elliptec base class with a background RX thread (so you never miss replies)
    and **strict packet validation** (so malformed fixed-length packets can't desync the parser).
    """

    def __init__(
        self,
        port: str,
        *,
        address: Optional[str] = None,
        min_address: str = "0",
        max_address: str = "F",
        baudrate: int = 9600,
        timeout_s: float = 0.10,
        write_timeout_s: float = 0.5,
        settle_s: float = 0.0,
        # --- tracing / debug ---
        debug: bool = False,
        debug_verbose: bool = False,
        debug_log_path: Optional[str] = None,
        ignore_mechanical_timeout: bool = True,
        trace_maxlen: int = 10_000,
    ) -> None:
        self._settle_s = float(settle_s)

        # trace
        self._debug = bool(debug)
        self._debug_verbose = bool(debug_verbose)
        self._ignore_mechanical_timeout = bool(ignore_mechanical_timeout)
        self._trace: Deque[SerialTraceEvent] = deque(maxlen=int(trace_maxlen))
        self._logger = self._make_logger(port=port, log_path=debug_log_path) if self._debug else None

        # serial
        self._serial = Serial(
            port=port,
            baudrate=baudrate,
            bytesize=EIGHTBITS,
            parity=PARITY_NONE,
            stopbits=STOPBITS_ONE,
            timeout=float(timeout_s),
            write_timeout=float(write_timeout_s),
        )

        # RX thread + buffer
        self._rx_cond = threading.Condition()
        self._rx_lines: Deque[Tuple[float, str, bytes]] = deque(maxlen=50_000)
        self._stop_evt = threading.Event()
        self._rx_thread = threading.Thread(target=self._rx_loop, name="elliptec-rx", daemon=True)
        self._rx_thread.start()

        if self._debug:
            self._trace_event("INFO", f"Opened serial port {port} (timeout={timeout_s}, write_timeout={write_timeout_s})")

        self._min_address = _normalize_address(min_address)
        self._max_address = _normalize_address(max_address)

        if address is None:
            addrs = self._find_addresses(min_address=self._min_address, max_address=self._max_address)
            if not addrs:
                raise ElliptecError(
                    f"No Elliptec device found on {port} in address range {self._min_address}..{self._max_address}."
                )
            if len(addrs) > 1:
                raise ElliptecError(
                    f"Multiple Elliptec devices found on {port}: {addrs}. Pass address=... to select one."
                )
            self._address = addrs[0]
        else:
            self._address = _normalize_address(address)

    # ------------------------- lifecycle -------------------------

    def close(self) -> None:
        try:
            self._stop_evt.set()
        except Exception:
            pass
        try:
            if self._rx_thread.is_alive():
                self._rx_thread.join(timeout=1.0)
        except Exception:
            pass
        try:
            if self._serial.is_open:
                self._serial.close()
        except Exception:
            pass

    def __enter__(self) -> "ElliptecDeviceBase":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    # ------------------------------ logging --------------------------------

    def _make_logger(self, *, port: str, log_path: Optional[str]) -> logging.Logger:
        name = f"elliptec[{port}]"
        logger = logging.getLogger(name)
        logger.setLevel(logging.INFO)
        logger.propagate = False
        if logger.handlers:
            return logger

        fmt = logging.Formatter("%(message)s")
        sh = logging.StreamHandler()
        sh.setFormatter(fmt)
        logger.addHandler(sh)

        if log_path:
            fh = logging.FileHandler(log_path, encoding="utf-8")
            fh.setFormatter(fmt)
            logger.addHandler(fh)

        return logger

    def _trace_event(self, direction: str, text: str, *, raw: bytes = b"") -> None:
        ev = SerialTraceEvent(
            ts_monotonic=time.monotonic(),
            ts_iso=_ts_iso(),
            direction=direction,
            text=text,
            raw=raw,
        )
        self._trace.append(ev)

        if not self._debug:
            return

        if direction in ("RX", "TX"):
            line = f"{ev.ts_iso} {direction:>4} {text:<12}"
            if raw:
                line += f"  [raw: {_hexdump(raw)}]"
        else:
            line = f"{ev.ts_iso} {direction:>4} {text}"

        if (not self._debug_verbose) and text == "<timeout>":
            return

        assert self._logger is not None
        self._logger.info(line)

    # ------------------------------ RX loop --------------------------------

    def _rx_loop(self) -> None:
        while not self._stop_evt.is_set():
            try:
                raw = self._serial.readline()  # replies end with CRLF
            except Exception as e:
                if self._debug:
                    self._trace_event("INFO", f"RX exception: {e!r}")
                time.sleep(0.05)
                continue

            if not raw:
                if self._debug_verbose:
                    self._trace_event("RX", "<timeout>", raw=b"")
                continue

            txt = raw.decode("ascii", errors="replace").rstrip("\r\n")
            ts = time.monotonic()

            with self._rx_cond:
                self._rx_lines.append((ts, txt, raw))
                self._rx_cond.notify_all()

            if self._debug:
                self._trace_event("RX", txt, raw=raw)

    # ------------------------------ utilities ------------------------------

    def reset_parser(self) -> None:
        """Send a single '\\r' to reset the device receive state machine."""
        raw = b"\r"
        if self._debug:
            self._trace_event("TX", r"\r", raw=raw)
        self._serial.write(raw)
        self._serial.flush()

    def flush_rx(self, *, clear_serial_buffer: bool = False) -> None:
        """Clear queued RX lines (and optionally the OS RX buffer)."""
        with self._rx_cond:
            n = len(self._rx_lines)
            self._rx_lines.clear()
        if self._debug:
            self._trace_event("INFO", f"flush_rx() cleared {n} queued line(s); clear_serial_buffer={clear_serial_buffer}")
        if clear_serial_buffer:
            try:
                self._serial.reset_input_buffer()
            except Exception:
                pass

    def _pop_rx_line(self, *, timeout_s: float) -> Optional[Tuple[float, str, bytes]]:
        deadline = time.monotonic() + float(timeout_s)
        with self._rx_cond:
            while not self._rx_lines:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return None
                self._rx_cond.wait(timeout=remaining)
            return self._rx_lines.popleft()

    # ------------------------------ TX validation ---------------------------

    def _validate_cmd(self, cmd: str) -> None:
        """
        Elliptec packets are fixed-length (no terminator on TX).
        If you send the wrong number of bytes, the device parser can desync.
        """
        if not cmd or len(cmd) < 3:
            raise ElliptecError(f"Invalid command (too short): {cmd!r}")

        addr = _normalize_address(cmd[0])
        mnemonic = cmd[1:3]

        # expected lengths from protocol manual:
        # - gs/gv/in : 3 bytes
        # - ho0/ho1  : 4 bytes
        # - svVV     : 5 bytes (VV = 2 hex digits)
        # - mrXXXXXXXX / maXXXXXXXX : 11 bytes (Position = long32 = 8 hex digits)
        if mnemonic in (HostCommand.GET_STATUS.value, HostCommand.GET_VELOCITY.value, HostCommand.GET_INFO.value):
            if len(cmd) != 3:
                raise ElliptecError(f"Packet length for '{mnemonic}' must be 3 bytes, got {len(cmd)}: {cmd!r}")
            return

        if mnemonic == HostCommand.HOME.value:
            if len(cmd) != 4 or cmd[3] not in ("0", "1"):
                raise ElliptecError(f"Packet for 'ho' must be 4 bytes like '{addr}ho0'/'{addr}ho1', got: {cmd!r}")
            return

        if mnemonic == HostCommand.SET_VELOCITY.value:
            if len(cmd) != 5:
                raise ElliptecError(f"Packet length for 'sv' must be 5 bytes, got {len(cmd)}: {cmd!r}")
            vv = cmd[3:5].upper()
            if any(c not in _HEX_DIGITS for c in vv):
                raise ElliptecError(f"'sv' payload must be 2 hex digits, got: {cmd!r}")
            return

        if mnemonic in (HostCommand.MOVE_RELATIVE.value, HostCommand.MOVE_ABSOLUTE.value):
            if len(cmd) != 11:
                raise ElliptecError(
                    f"Packet length for '{mnemonic}' must be 11 bytes ({addr}{mnemonic}<8 hex>), got {len(cmd)}: {cmd!r}"
                )
            payload = cmd[3:11].upper()
            if any(c not in _HEX_DIGITS for c in payload):
                raise ElliptecError(f"'{mnemonic}' payload must be 8 hex digits (long32), got: {cmd!r}")
            return

        # unknown mnemonic -> no strict validation (still trace)
        return

    def _send_raw(self, cmd: str) -> float:
        self._validate_cmd(cmd)
        raw = cmd.encode("ascii")
        if self._debug:
            self._trace_event("TX", cmd, raw=raw)
        self._serial.write(raw)
        self._serial.flush()
        if self._settle_s:
            time.sleep(self._settle_s)
        return time.monotonic()

    def _send_and_wait_one(
        self,
        cmd: str,
        *,
        timeout_s: float = 1.0,
        predicate: Optional[Callable[[str], bool]] = None,
        clear_rx_queue: bool = False,
        clear_rx_serial: bool = False,
    ) -> Optional[str]:
        if clear_rx_queue or clear_rx_serial:
            self.flush_rx(clear_serial_buffer=clear_rx_serial)

        tx_ts = self._send_raw(cmd)

        deadline = time.monotonic() + float(timeout_s)
        while time.monotonic() < deadline:
            item = self._pop_rx_line(timeout_s=max(0.0, deadline - time.monotonic()))
            if item is None:
                return None
            ts, txt, _raw = item

            if ts + 1e-3 < tx_ts:
                continue

            if predicate is None or predicate(txt):
                return txt

        return None

    # ------------------------------ discovery -------------------------------

    def _find_addresses(self, *, min_address: str, max_address: str) -> List[str]:
        found: List[str] = []
        self.reset_parser()
        for a in _address_range(min_address, max_address):
            reply = self._send_and_wait_one(
                f"{a}{HostCommand.GET_STATUS.value}",
                timeout_s=0.6,
                predicate=lambda s, aa=a: (len(s) >= 5 and s[0] == aa and s[1:3] == ReplyCommand.STATUS.value),
                clear_rx_queue=True,
                clear_rx_serial=True,
            )
            if reply is None:
                continue
            try:
                _ = _parse_status_reply(reply, a)
            except ElliptecError:
                continue
            found.append(a)
        return found

    # ------------------------------ public API ------------------------------

    @property
    def address(self) -> str:
        return self._address

    def get_status(self, *, timeout_s: float = 1.0) -> StatusCode:
        reply = self._send_and_wait_one(
            f"{self._address}{HostCommand.GET_STATUS.value}",
            timeout_s=timeout_s,
            predicate=lambda s: len(s) >= 5 and s[0] == self._address and s[1:3] == ReplyCommand.STATUS.value,
        )
        if reply is None:
            raise ElliptecError("Timeout waiting for status reply")
        return _parse_status_reply(reply, self._address)

    def get_speed(self, *, timeout_s: float = 1.0) -> int:
        reply = self._send_and_wait_one(
            f"{self._address}{HostCommand.GET_VELOCITY.value}",
            timeout_s=timeout_s,
            predicate=lambda s: len(s) >= 5 and s[0] == self._address and s[1:3] == ReplyCommand.VELOCITY.value,
        )
        if reply is None:
            raise ElliptecError("Timeout waiting for velocity reply")
        return _parse_velocity_reply(reply, self._address)

    def set_speed(self, percent: int, *, timeout_s: float = 1.0) -> None:
        vv = _percent_to_hex_byte(percent)
        reply = self._send_and_wait_one(
            f"{self._address}{HostCommand.SET_VELOCITY.value}{vv}",
            timeout_s=timeout_s,
            predicate=lambda s: len(s) >= 5 and s[0] == self._address and s[1:3] == ReplyCommand.STATUS.value,
        )
        if reply is None:
            raise ElliptecError("Timeout waiting for set_speed status reply")
        st = _parse_status_reply(reply, self._address)
        if st != StatusCode.OK:
            raise ElliptecError(f"set_speed failed with status {st.name}", status=st, reply=reply)

    def home(self, direction: HomeDirection = HomeDirection.CW, *, timeout_s: float = 30.0) -> None:
        self._send_raw(f"{self._address}{HostCommand.HOME.value}{int(direction)}")
        self._wait_until_done(timeout_s=timeout_s)

    def move_relative(self, delta_counts: int, *, timeout_s: float = 30.0) -> None:
        payload = _encode_long32(delta_counts)
        # => always 8 hex digits, so packet is exactly 11 bytes
        self._send_raw(f"{self._address}{HostCommand.MOVE_RELATIVE.value}{payload}")
        self._wait_until_done(timeout_s=timeout_s)

    def move_absolute(self, position_counts: int, *, timeout_s: float = 30.0) -> None:
        payload = _encode_long32(position_counts)
        self._send_raw(f"{self._address}{HostCommand.MOVE_ABSOLUTE.value}{payload}")
        self._wait_until_done(timeout_s=timeout_s)

    # ------------------------------ motion wait -----------------------------

    def _wait_until_done(self, *, timeout_s: float) -> None:
        deadline = time.monotonic() + float(timeout_s)
        poll_interval_s = 0.15
        next_poll = time.monotonic()

        last_reply: Optional[str] = None

        while time.monotonic() < deadline:
            item = self._pop_rx_line(timeout_s=0.05)
            if item is not None:
                _ts, line, _raw = item
                last_reply = line

                if not line or line[0] != self._address or len(line) < 3:
                    continue

                cmd = line[1:3]
                if cmd == ReplyCommand.STATUS.value:
                    st = _parse_status_reply(line, self._address)
                    if st == StatusCode.BUSY:
                        continue
                    if st == StatusCode.MECHANICAL_TIMEOUT and self._ignore_mechanical_timeout:
                        # Some devices report MECHANICAL_TIMEOUT when a move takes long, but still continue.
                        if self._debug:
                            self._trace_event('WARN', 'Ignoring MECHANICAL_TIMEOUT during motion wait')
                        continue
                    if st != StatusCode.OK:
                        raise ElliptecError(f"Motion ended with error {st.name}", status=st, reply=line)
                    return

                if cmd == ReplyCommand.POSITION.value:
                    return

            now = time.monotonic()
            if now >= next_poll:
                next_poll = now + poll_interval_s
                try:
                    st = self.get_status(timeout_s=0.6)
                except ElliptecError:
                    continue

                if st == StatusCode.BUSY:
                    continue
                if st == StatusCode.MECHANICAL_TIMEOUT and self._ignore_mechanical_timeout:
                    if self._debug:
                        self._trace_event('WARN', 'Ignoring MECHANICAL_TIMEOUT during motion wait (polled)')
                    continue
                if st != StatusCode.OK:
                    raise ElliptecError(f"Motion ended with error {st.name}", status=st, reply=last_reply)
                return

        raise ElliptecError("Timeout waiting for motion to complete", reply=last_reply)


def angle_to_counts(angle_rad: float, *, counts_per_rev: int = 262_144, wrap: bool = False) -> int:
    """
    Convert an angle in radians to encoder counts for a rotary axis (e.g. ELL14).
    """
    rad = float(angle_rad)
    if wrap:
        two_pi = 2.0 * math.pi
        rad = (rad + math.pi) % two_pi - math.pi
    return int(round(rad / (2.0 * math.pi) * int(counts_per_rev)))
