from __future__ import annotations

import math
import time
import logging
import threading
from dataclasses import dataclass
from datetime import datetime
from collections import deque
from enum import Enum, IntEnum
from typing import Callable, Deque, Iterable, List, Optional, Tuple

from serial import EIGHTBITS, PARITY_NONE, STOPBITS_ONE, Serial


# =============================================================================
# Elliptec protocol helpers + a serial-tracing base class (threaded RX)
#
# Purpose:
# - Make debugging reliable by NEVER "missing" replies while the device is moving.
# - Log every TX and RX line (plus raw bytes), even if it arrives asynchronously.
# =============================================================================


# ----------------------------- Enums / Types --------------------------------

class HostCommand(str, Enum):
    """Lowercase host commands (sent to device)."""
    GET_STATUS = "gs"
    HOME = "ho"
    MOVE_ABSOLUTE = "ma"
    MOVE_RELATIVE = "mr"
    SET_VELOCITY = "sv"
    GET_VELOCITY = "gv"


class ReplyCommand(str, Enum):
    """Uppercase reply tokens (sent by device)."""
    STATUS = "GS"
    POSITION = "PO"
    VELOCITY = "GV"


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


# ----------------------------- Exceptions -----------------------------------

class ElliptecError(RuntimeError):
    def __init__(self, message: str, *, status: Optional[StatusCode] = None, reply: Optional[str] = None):
        super().__init__(message)
        self.status = status
        self.reply = reply


# ----------------------------- Helper functions -----------------------------

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
    if not isinstance(value, int):
        raise TypeError("value must be int")
    v = value & 0xFFFFFFFF
    return f"{v:08X}"


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
    if not isinstance(percent, int):
        raise TypeError("percent must be int")
    if percent < 0 or percent > 100:
        raise ValueError("percent must be in [0, 100]")
    return f"{percent:02X}"


def _hexdump(raw: bytes) -> str:
    return " ".join(f"{b:02x}" for b in raw)


# =============================================================================
# Threaded serial transport with tracing
# =============================================================================

class ElliptecDeviceBase:
    """
    Minimal Elliptec base class using *only serial* (no Thorlabs DLL),
    with a background RX thread so you can reliably debug traffic.

    Implements:
      - home()
      - move_relative()
      - move_absolute()
      - set_speed() / get_speed()
      - get_status()

    Debugging:
      - Logs every TX + RX (and raw bytes) to console and optionally to a file.
      - RX is continuously collected, so you won't miss async replies.
    """

    def __init__(
        self,
        port: str,
        *,
        address: Optional[str] = None,
        min_address: str = "0",
        max_address: str = "F",
        baudrate: int = 9600,
        timeout_s: float = 0.10,          # serial read timeout (short is good for reader thread)
        write_timeout_s: float = 0.5,
        settle_s: float = 0.0,            # optional post-write sleep
        # --- tracing / debug ---
        debug: bool = False,
        debug_verbose: bool = False,
        debug_log_path: Optional[str] = None,
        trace_maxlen: int = 10_000,
    ) -> None:
        self._settle_s = float(settle_s)

        # trace
        self._debug = bool(debug)
        self._debug_verbose = bool(debug_verbose)
        self._trace: Deque[SerialTraceEvent] = deque(maxlen=int(trace_maxlen))
        self._logger = self._make_logger(port=port, log_path=debug_log_path) if self._debug else None

        # serial
        self._serial = self._open(
            port,
            baudrate=baudrate,
            timeout=timeout_s,
            write_timeout=write_timeout_s,
        )

        # RX thread + buffer
        self._rx_cond = threading.Condition()
        self._rx_lines: Deque[Tuple[float, str, bytes]] = deque(maxlen=50_000)  # (ts, text, raw)
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

    # ------------------------- context / lifecycle -------------------------

    def close(self) -> None:
        try:
            if getattr(self, "_stop_evt", None) is not None:
                self._stop_evt.set()
        except Exception:
            pass

        try:
            if getattr(self, "_rx_thread", None) is not None and self._rx_thread.is_alive():
                self._rx_thread.join(timeout=1.0)
        except Exception:
            pass

        try:
            if getattr(self, "_serial", None) is not None and self._serial.is_open:
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

        # avoid duplicate handlers if re-created
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

        if direction == "RX" or direction == "TX":
            line = f"{ev.ts_iso} {direction:>4} {text:<12}"
            if raw:
                line += f"  [raw: {_hexdump(raw)}]"
        else:
            line = f"{ev.ts_iso} {direction:>4} {text}"

        # optionally hide timeout spam
        if (not self._debug_verbose) and text == "<timeout>":
            return

        assert self._logger is not None
        self._logger.info(line)

    # ------------------------------ serial ---------------------------------

    def _open(self, port: str, *, baudrate: int, timeout: float, write_timeout: float) -> Serial:
        s = Serial(
            port=port,
            baudrate=baudrate,
            bytesize=EIGHTBITS,
            parity=PARITY_NONE,
            stopbits=STOPBITS_ONE,
            timeout=float(timeout),
            write_timeout=float(write_timeout),
        )
        return s

    def reset_parser(self) -> None:
        """
        Send a single carriage return to reset the device receive state machine.
        Useful before scans on a noisy bus.
        """
        raw = b"\r"
        if self._debug:
            self._trace_event("TX", r"\r", raw=raw)
        self._serial.write(raw)
        self._serial.flush()

    def flush_rx(self, *, clear_serial_buffer: bool = False) -> None:
        """
        Clear buffered RX lines.
        Optionally also clears the OS/driver RX buffer via reset_input_buffer().
        """
        n = 0
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

    # Background RX loop
    def _rx_loop(self) -> None:
        while not self._stop_evt.is_set():
            try:
                raw = self._serial.readline()  # expects CRLF terminated replies
            except Exception as e:
                if self._debug:
                    self._trace_event("INFO", f"RX exception: {e!r}")
                time.sleep(0.05)
                continue

            if not raw:
                if self._debug_verbose:
                    self._trace_event("RX", "<timeout>", raw=b"")
                continue

            # Decode + strip CRLF
            txt = raw.decode("ascii", errors="replace").rstrip("\r\n")
            ts = time.monotonic()

            with self._rx_cond:
                self._rx_lines.append((ts, txt, raw))
                self._rx_cond.notify_all()

            if self._debug:
                self._trace_event("RX", txt, raw=raw)

    def _send_raw(self, cmd: str) -> float:
        """
        Send a raw protocol packet (NO CRLF).
        Returns the monotonic timestamp immediately after sending.
        """
        raw = cmd.encode("ascii")
        if self._debug:
            self._trace_event("TX", cmd, raw=raw)

        self._serial.write(raw)
        self._serial.flush()
        if self._settle_s:
            time.sleep(self._settle_s)
        return time.monotonic()

    def _pop_rx_line(self, *, timeout_s: float) -> Optional[Tuple[float, str, bytes]]:
        """
        Pop one buffered RX line (ts, text, raw).
        """
        deadline = time.monotonic() + float(timeout_s)
        with self._rx_cond:
            while not self._rx_lines:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return None
                self._rx_cond.wait(timeout=remaining)
            return self._rx_lines.popleft()

    def _send_and_wait_one(
        self,
        cmd: str,
        *,
        timeout_s: float = 1.0,
        predicate: Optional[Callable[[str], bool]] = None,
        clear_rx_queue: bool = False,
        clear_rx_serial: bool = False,
    ) -> Optional[str]:
        """
        Send a command and wait for the *next matching* RX line.

        - Does NOT flush RX by default (to avoid losing async replies).
        - If predicate is None: returns the first line observed after TX timestamp.
        """
        if clear_rx_queue or clear_rx_serial:
            self.flush_rx(clear_serial_buffer=clear_rx_serial)

        tx_ts = self._send_raw(cmd)

        deadline = time.monotonic() + float(timeout_s)
        while time.monotonic() < deadline:
            item = self._pop_rx_line(timeout_s=max(0.0, deadline - time.monotonic()))
            if item is None:
                return None
            ts, txt, _raw = item

            # ignore anything that definitely happened before we sent
            if ts + 1e-3 < tx_ts:
                continue

            if predicate is None or predicate(txt):
                return txt

            # otherwise: just keep consuming; it's already traced for debugging

        return None

    # ------------------------------ discovery -------------------------------

    def _find_addresses(self, *, min_address: str, max_address: str) -> List[str]:
        """
        Probe addresses in [min_address, max_address] by sending 'gs' and
        checking for a well-formed 'GS' reply.
        """
        found: List[str] = []
        for a in _address_range(min_address, max_address):
            # For scanning we DO clear the queued RX to keep correlation clean.
            # We do NOT clear the serial driver buffer unless explicitly asked.
            reply = self._send_and_wait_one(
                f"{a}{HostCommand.GET_STATUS.value}",
                timeout_s=0.6,
                predicate=lambda s, aa=a: (len(s) >= 5 and s[0] == aa and s[1:3] == ReplyCommand.STATUS.value),
                clear_rx_queue=True,
                clear_rx_serial=True,  # during scans, this is usually what you want
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

    def listen(self, duration_s: float = 5.0) -> None:
        """
        Passive RX logging for `duration_s` seconds.
        (Useful when you want to see if the device is streaming replies.)
        """
        end = time.monotonic() + float(duration_s)
        while time.monotonic() < end:
            item = self._pop_rx_line(timeout_s=0.2)
            if item is None:
                continue
            # already traced by thread; nothing else to do

    def get_status(self, *, timeout_s: float = 1.0) -> StatusCode:
        reply = self._send_and_wait_one(
            f"{self._address}{HostCommand.GET_STATUS.value}",
            timeout_s=timeout_s,
            predicate=lambda s: len(s) >= 5 and s[0] == self._address and s[1:3] == ReplyCommand.STATUS.value,
            clear_rx_queue=False,
            clear_rx_serial=False,
        )
        if reply is None:
            raise ElliptecError("Timeout waiting for status reply")
        return _parse_status_reply(reply, self._address)

    def get_speed(self, *, timeout_s: float = 1.0) -> int:
        reply = self._send_and_wait_one(
            f"{self._address}{HostCommand.GET_VELOCITY.value}",
            timeout_s=timeout_s,
            predicate=lambda s: len(s) >= 5 and s[0] == self._address and s[1:3] == ReplyCommand.VELOCITY.value,
            clear_rx_queue=False,
            clear_rx_serial=False,
        )
        if reply is None:
            raise ElliptecError("Timeout waiting for velocity reply")
        return _parse_velocity_reply(reply, self._address)

    def set_speed(self, percent: int, *, timeout_s: float = 1.0) -> None:
        """
        Set velocity compensation in percent (0..100).
        Protocol: AsvVV with VV as 2 hex digits (00..64).
        """
        vv = _percent_to_hex_byte(percent)
        reply = self._send_and_wait_one(
            f"{self._address}{HostCommand.SET_VELOCITY.value}{vv}",
            timeout_s=timeout_s,
            predicate=lambda s: len(s) >= 5 and s[0] == self._address and s[1:3] == ReplyCommand.STATUS.value,
            clear_rx_queue=False,
            clear_rx_serial=False,
        )
        if reply is None:
            raise ElliptecError("Timeout waiting for set_speed status reply")
        st = _parse_status_reply(reply, self._address)
        if st != StatusCode.OK:
            raise ElliptecError(f"set_speed failed with status {st.name}", status=st, reply=reply)

    def home(self, direction: HomeDirection = HomeDirection.CW, *, timeout_s: float = 30.0) -> None:
        """
        Home (ho). Direction: 0 = CW, 1 = CCW.
        The device may answer:
          - GS09 (busy) while moving
          - PO........ when done
          - or GSxx error
        """
        # Don't flush: we want to see everything.
        _ = self._send_raw(f"{self._address}{HostCommand.HOME.value}{int(direction)}")
        self._wait_until_done(timeout_s=timeout_s)

    def move_relative(self, delta_counts: int, *, timeout_s: float = 30.0) -> None:
        payload = _encode_long32(int(delta_counts))
        _ = self._send_raw(f"{self._address}{HostCommand.MOVE_RELATIVE.value}{payload}")
        self._wait_until_done(timeout_s=timeout_s)

    def move_absolute(self, position_counts: int, *, timeout_s: float = 30.0) -> None:
        payload = _encode_long32(int(position_counts))
        _ = self._send_raw(f"{self._address}{HostCommand.MOVE_ABSOLUTE.value}{payload}")
        self._wait_until_done(timeout_s=timeout_s)

    # ------------------------------ motion wait -----------------------------

    def _wait_until_done(self, *, timeout_s: float) -> None:
        """
        Robust wait for motion completion.

        Strategy:
          - Prefer consuming async RX lines (GS.. / PO..).
          - If nothing arrives for a short while, poll 'gs' without flushing RX.
        """
        deadline = time.monotonic() + float(timeout_s)
        poll_interval_s = 0.15
        next_poll = time.monotonic()

        last_reply: Optional[str] = None

        while time.monotonic() < deadline:
            # 1) consume any async replies quickly
            item = self._pop_rx_line(timeout_s=0.05)
            if item is not None:
                _ts, line, _raw = item
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
                    # Some devices return GS00 when done
                    return

                if cmd == ReplyCommand.POSITION.value:
                    # done (position returned)
                    return

                # other replies are ignored (but still logged)

            # 2) poll status occasionally
            now = time.monotonic()
            if now >= next_poll:
                next_poll = now + poll_interval_s
                try:
                    st = self.get_status(timeout_s=0.6)
                except ElliptecError:
                    # if the device doesn't answer while moving, keep waiting
                    continue

                if st == StatusCode.BUSY:
                    continue
                if st != StatusCode.OK:
                    raise ElliptecError(f"Motion ended with error {st.name}", status=st, reply=last_reply)
                return

        raise ElliptecError("Timeout waiting for motion to complete", reply=last_reply)


# ------------------------------- Angle helper -------------------------------

def angle_to_counts(angle_rad: float, *, counts_per_rev: int = 262_144, wrap: bool = False) -> int:
    """
    Convert an angle in radians to encoder counts for a rotary axis (e.g. ELL14).

    wrap=False: allow multi-turn values (e.g. 4*pi -> 2 rev)
    wrap=True : wrap to (-pi, +pi]
    """
    rad = float(angle_rad)
    if wrap:
        two_pi = 2.0 * math.pi
        rad = (rad + math.pi) % two_pi - math.pi

    return int(round(rad / (2.0 * math.pi) * int(counts_per_rev)))
