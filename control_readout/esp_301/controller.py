
from __future__ import annotations

import time
from typing import Optional, TYPE_CHECKING

from control_readout.base.controller import Controller, ControllerError

try:
    import serial
except ImportError:  # keep the module importable for inspection without hardware
    serial = None

if TYPE_CHECKING:
    from serial import Serial


class ESP301Error(ControllerError):
    """Raised for connection or device errors specific to the ESP301."""


class ESP301Controller(Controller):
    """Owns the serial connection to a Newport ESP301 controller and a registry
    of devices (one per motorised axis).

    The ESP301 speaks a simple ASCII command protocol over RS-232/USB: each
    command is prefixed with a 1-based axis number and terminated with a
    carriage return (e.g. ``1PA10.0`` moves axis 1 to 10 mm). The addressed
    device ``address`` for this controller is the integer axis number.

    Example
    -------
    >>> from control_readout.base.device import Device
    >>> ctrl = ESP301Controller("COM3")
    >>> stage = ctrl.add_device(Device("x", address=1, controller=ctrl))
    >>> ctrl.connect()
    >>> stage.initialize()
    >>> stage.home()
    >>> stage.move_to(10.0)
    >>> ctrl.disconnect()
    """

    def __init__(
        self,
        port: str,
        baud: int = 19200,
        timeout: float = 5.0,
    ) -> None:
        super().__init__()
        self.port = port
        self.baud = baud
        self.timeout = timeout
        self._serial: Optional["Serial"] = None

    # -- transport -------------------------------------------------------- #
    def _open(self) -> None:
        if serial is None:
            raise ESP301Error("pyserial is not installed. Run: pip install pyserial")
        self._serial = serial.Serial(self.port, self.baud, timeout=self.timeout)

    def _close(self) -> None:
        if self._serial is not None:
            try:
                self._serial.close()
            finally:
                self._serial = None

    @property
    def serial(self) -> "Serial":
        """The underlying pyserial object (raises if not connected)."""
        if self._serial is None:
            raise ESP301Error("Controller is not connected. Call connect() first.")
        return self._serial

    # -- low-level IO ----------------------------------------------------- #
    def _write(self, command: str) -> None:
        """Send a command (carriage-return terminated) with no reply expected."""
        with self._lock:
            self.serial.write((command + "\r").encode("ascii"))

    def _query(self, command: str) -> str:
        """Send a command and return the controller's single-line reply."""
        with self._lock:
            self.serial.write((command + "\r").encode("ascii"))
            return self.serial.readline().decode("ascii").strip()

    # -- error handling --------------------------------------------------- #
    def check_errors(self) -> int:
        """Read the top of the ESP301 error buffer (``TE?``). 0 means no error."""
        reply = self._query("TE?")
        try:
            return int(float(reply))
        except ValueError:
            return 0

    def raise_on_error(self) -> None:
        code = self.check_errors()
        if code != 0:
            raise ESP301Error(f"ESP301 reported error code {code}")

    # -- addressed motion primitives (Controller interface) --------------- #
    def initialize(self, address: int) -> None:
        """Energise the axis (``MO``). Required before any move."""
        self._write(f"{address}MO")

    def motor_off(self, address: int) -> None:
        """De-energise the axis (``MF``)."""
        self._write(f"{address}MF")

    def home(self, address: int, mode: Optional[int] = None, timeout: float = 120.0) -> None:
        """Run the home / origin search (``OR``) and block until it completes.

        ``mode`` selects the home search type (see the ESP301 manual, e.g. 2 for
        find home + index); leave as None to use the axis' configured default.
        """
        self._write(f"{address}OR" if mode is None else f"{address}OR{mode}")
        self.wait_for_motion(address, timeout=timeout)

    def get_position(self, address: int) -> float:
        """Current position in the axis' configured units (``TP``)."""
        return float(self._query(f"{address}TP"))

    def move_absolute(self, address: int, value: float, timeout: float = 120.0) -> None:
        """Absolute move (``PA``); blocks until motion is done."""
        self._write(f"{address}PA{value:.4f}")
        self.wait_for_motion(address, timeout=timeout)

    def move_relative(self, address: int, delta: float, timeout: float = 120.0) -> None:
        """Relative move (``PR``); blocks until motion is done."""
        self._write(f"{address}PR{delta:.4f}")
        self.wait_for_motion(address, timeout=timeout)

    def set_velocity(self, address: int, velocity: float) -> None:
        """Set the max velocity (``VA``) for subsequent moves."""
        self._write(f"{address}VA{velocity:.4f}")

    def stop(self, address: int) -> None:
        """Stop motion on this axis with deceleration (``ST``)."""
        self._write(f"{address}ST")

    def motion_done(self, address: int) -> bool:
        """True once the axis has finished moving (``MD?`` returns 1)."""
        return self._query(f"{address}MD?").strip().startswith("1")

    def wait_for_motion(self, address: int, poll: float = 0.05, timeout: float = 120.0) -> None:
        """Block until the axis reports motion done, or raise on timeout."""
        deadline = time.monotonic() + timeout
        while not self.motion_done(address):
            if time.monotonic() > deadline:
                raise ESP301Error(f"Timed out waiting for axis {address} to stop moving.")
            time.sleep(poll)

    def __repr__(self) -> str:
        state = "connected" if self.connected else "disconnected"
        return f"ESP301Controller(port={self.port!r}, {state}, devices={list(self._devices)})"
