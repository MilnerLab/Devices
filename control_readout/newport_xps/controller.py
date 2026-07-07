
from __future__ import annotations

from typing import Optional, Tuple

from control_readout.base.controller import Controller, ControllerError

try:
    from newportxps import NewportXPS
except ImportError:  # keep the module importable for inspection without hardware
    NewportXPS = None

#: Device address on an XPS: (group, positioner). ``initialize``/``home``/``stop``
#: act on the group; ``move``/``position``/``set_velocity`` act on the positioner.
XPSAddress = Tuple[str, str]


class XPSError(ControllerError):
    """Raised for connection or device errors specific to the XPS."""


class XPSController(Controller):
    """Owns the network connection to a Newport XPS controller and a registry of
    devices. It knows nothing about whether a device is rotary, linear, etc.

    The addressed device ``address`` for this controller is a ``(group,
    positioner)`` pair (see :data:`XPSAddress`).

    Example
    -------
    >>> from control_readout.newport_xps.rgv100bl.rgv100bl_device import RGV
    >>> ctrl = XPSController("192.168.0.254", password="Administrator")
    >>> rot = ctrl.add_device(RGV("rot", group="Rot", controller=ctrl))
    >>> ctrl.connect()
    >>> rot.home()
    >>> rot.rotate(Angle(90, AngleUnit.DEG))
    >>> ctrl.disconnect()
    """

    def __init__(
        self,
        host: str,
        username: str = "Administrator",
        password: str = "Administrator",
        port: int = 5001,
        timeout: float = 10.0,
    ) -> None:
        super().__init__()
        self.host = host
        self.username = username
        self.password = password
        self.port = port
        self.timeout = timeout
        self._xps: Optional["NewportXPS"] = None

    # -- transport -------------------------------------------------------- #
    def _open(self) -> None:
        if NewportXPS is None:
            raise XPSError("newportxps is not installed. Run: pip install newportxps")
        self._xps = NewportXPS(
            self.host,
            username=self.username,
            password=self.password,
            port=self.port,
            timeout=self.timeout,
        )

    def _close(self) -> None:
        if self._xps is not None:
            try:
                self._xps.disconnect()
            finally:
                self._xps = None

    @property
    def xps(self) -> "NewportXPS":
        """The underlying NewportXPS object (raises if not connected)."""
        if self._xps is None:
            raise XPSError("Controller is not connected. Call connect() first.")
        return self._xps

    # -- addressed motion primitives (Controller interface) --------------- #
    @staticmethod
    def _split(address: XPSAddress) -> XPSAddress:
        group, positioner = address
        return group, positioner

    def kill(self, address: XPSAddress) -> None:
        """Kill (de-energise) the group, returning it to the NOTINIT state.

        The XPS refuses to initialize a group that is already initialized and
        will raise a driver fault. Killing first guarantees the clean NOTINIT
        state that ``initialize`` expects, so it's safe to call before every
        ``initialize``."""
        group, _ = self._split(address)
        self.xps.kill_group(group)

    def initialize(self, address: XPSAddress) -> None:
        """Power on / initialize the group. Required once after power-up."""
        group, _ = self._split(address)
        self.xps.initialize_group(group)

    def home(self, address: XPSAddress) -> None:
        """Run the home search. Required after initialize, before any move."""
        group, _ = self._split(address)
        self.xps.home_group(group)

    def get_position(self, address: XPSAddress) -> float:
        _, positioner = self._split(address)
        return self.xps.get_stage_position(positioner)

    def move_absolute(self, address: XPSAddress, value: float) -> None:
        _, positioner = self._split(address)
        self.xps.move_stage(positioner, value, relative=False)

    def move_relative(self, address: XPSAddress, delta: float) -> None:
        _, positioner = self._split(address)
        self.xps.move_stage(positioner, delta, relative=True)

    def set_velocity(
        self, address: XPSAddress, velocity: float, acceleration: Optional[float] = None
    ) -> None:
        """Set max velocity (and optionally acceleration) for subsequent moves."""
        _, positioner = self._split(address)
        self.xps.set_velocity(positioner, velo=velocity, accel=acceleration)

    def stop(self, address: XPSAddress) -> None:
        """Abort motion on this device's group (does not disconnect)."""
        group, _ = self._split(address)
        # abort_group exists on newportxps; fall back to a group restart if not.
        abort = getattr(self.xps, "abort_group", None)
        if callable(abort):
            abort(group)
        else:
            self.xps.initialize_group(group)

    # -- convenience ------------------------------------------------------ #
    def status(self) -> str:
        """Human-readable status of the whole controller (groups + stages)."""
        return self.xps.status_report()

    def hardware_stages(self) -> dict:
        """Raw dict of stages the XPS knows about, from its system.ini.
        Useful for discovering the exact group/positioner names to use."""
        return self.xps.stages

    def __repr__(self) -> str:
        state = "connected" if self.connected else "disconnected"
        return f"XPSController(host={self.host!r}, {state}, devices={list(self._devices)})"
