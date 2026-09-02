
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

#: Fallback spin acceleration, in units/s^2, used only when the group reports none.
#: Sized so a spin reaches a typical rotator's full speed in roughly one second.
DEFAULT_SPIN_ACCEL = 720.0


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

    # -- continuous rotation (SpindleAxis groups only) -------------------- #
    # These act on the GROUP, not the positioner, and only exist on a group declared
    # SpindleAxis in the XPS system.ini. On a SingleAxis group the controller rejects
    # them, which is the correct failure: a SingleAxis group has travel limits, and a
    # command meaning "turn forever" has no valid interpretation there.
    def spin(
        self, address: XPSAddress, velocity: float, acceleration: Optional[float] = None
    ) -> None:
        """Start (or re-parameterise) continuous rotation at ``velocity`` units/s.

        Sign sets the direction. Issuing this while already spinning changes the speed
        on the fly rather than restarting -- the XPS ramps from the current velocity to
        the new one, so a rate change mid-run is smooth and does not stop the plate.
        """
        group, _ = self._split(address)
        accel = self._default_accel(group, acceleration)
        err, _ = self.xps._xps.GroupSpinParametersSet(self.xps._sid, group, velocity, accel)
        self.xps.check_error(err, msg=f"spinning group '{group}'")

    def stop_spin(self, address: XPSAddress, acceleration: Optional[float] = None) -> None:
        """Ramp a spinning group down to a stop and leave it enabled.

        Not the same as ``stop``: ``GroupMoveAbort`` on a spinning group stops it hard,
        and on a direct-drive rotator carrying an optic that is a shock load. This
        decelerates at the configured rate and leaves the group READY, so the next
        absolute move needs no re-initialize.
        """
        group, _ = self._split(address)
        accel = self._default_accel(group, acceleration)
        err, _ = self.xps._xps.GroupSpinModeStop(self.xps._sid, group, accel)
        self.xps.check_error(err, msg=f"stopping spin on group '{group}'")

    def spin_current(self, address: XPSAddress) -> Tuple[float, float]:
        """The group's ACTUAL (velocity, acceleration) right now, read back from the XPS.

        Read-back, not the commanded setpoint: during the ramp this is still climbing.
        """
        group, _ = self._split(address)
        err, velocity, acceleration = self.xps._xps.GroupSpinCurrentGet(self.xps._sid, group)
        self.xps.check_error(err, msg=f"reading spin state of group '{group}'")
        return float(velocity), float(acceleration)

    def _default_accel(self, group: str, acceleration: Optional[float]) -> float:
        """The caller's acceleration, or whatever the group is already configured for.

        GroupSpinParametersSet has no "leave it alone" value for acceleration, so an
        omitted one has to be filled in with the current setting rather than a number
        invented here -- a guess would silently re-tune the stage.
        """
        if acceleration is not None:
            return float(acceleration)
        err, _velocity, accel = self.xps._xps.GroupSpinParametersGet(self.xps._sid, group)
        self.xps.check_error(err, msg=f"reading spin parameters of group '{group}'")
        accel = float(accel)
        # A group that has never spun reports 0, and a zero acceleration either faults or
        # means "never gets there". Fall back to a rate gentle enough for any rotator: it
        # reaches the caller's velocity in about a second.
        if accel <= 0.0:
            accel = DEFAULT_SPIN_ACCEL
        return accel

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
