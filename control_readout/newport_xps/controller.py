
from __future__ import annotations

import logging
import threading
from typing import Optional, Tuple

from control_readout.base.controller import Controller, ControllerError

log = logging.getLogger(__name__)

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

#: Minimum/maximum jerk time for PositionerSGammaParametersSet, in seconds. The XPS
#: validates the pair against the stage's own profile and rejects values outside it;
#: these are the RGV100BL's defaults, and they shape the ramp, not the top speed.
_JERK_MIN_S = 0.005
_JERK_MAX_S = 0.05

#: How far short of the travel limit a SingleAxis spin aims, in stage units. Only reached
#: after tens of hours of continuous rotation; it exists so that if it ever IS reached the
#: axis stops with room to move in both directions rather than pinned against its bound.
_LIMIT_MARGIN = 1000.0

#: Longest wait for the spin move to unwind after an abort. The deceleration itself is
#: well under a second; this only bounds a wedged socket so it cannot hang a shutdown.
_SPIN_STOP_TIMEOUT_S = 5.0


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
        # --- SingleAxis continuous rotation (see the spin section) ---
        # A second connection, used only to absorb the blocking long move, plus the thread
        # sitting inside it. Both are created on the first spin and reused after that.
        self._spin_driver: Optional[object] = None
        self._spin_sid: Optional[int] = None
        self._spin_thread: Optional[threading.Thread] = None
        self._spin_accel: float = DEFAULT_SPIN_ACCEL

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
        self._close_spin_socket()
        if self._xps is not None:
            try:
                self._xps.disconnect()
            finally:
                self._xps = None

    def _close_spin_socket(self) -> None:
        """Drop the second connection. Any move still running on it ends with the socket.

        Callers are expected to have stopped the spin first -- the device layer funnels
        every path through stop_spin() for exactly that reason. This is the backstop, so
        that a disconnect cannot leak a logged-in session on the controller.
        """
        self._join_spin_thread()
        driver, sid = self._spin_driver, self._spin_sid
        self._spin_driver = self._spin_sid = None
        if driver is not None and sid is not None:
            try:
                driver.TCP_CloseSocket(sid)
            except Exception:
                log.exception("XPS: failed to close the spin socket")

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

    # -- continuous rotation ---------------------------------------------- #
    # Two strategies, picked from how the group is declared in the XPS system.ini.
    #
    # SpindleAxis groups have real spin commands (GroupSpinParametersSet): unbounded by
    # construction, and re-rateable while turning.
    #
    # A SingleAxis group has none of them -- GroupSpinParametersGet on ours answers -18,
    # "wrong object type for this command". But it does not need them. Our RGV100BL is
    # declared with travel limits of +-165,000,000 degrees, i.e. +-458,333 revolutions,
    # which is Newport's way of saying "this axis rotates continuously" while keeping the
    # SingleAxis command set. So an ordinary move toward that limit IS a continuous spin:
    # at the 720 deg/s ceiling it runs for 63 hours before the target is reached, and at
    # the 0.5 rev/s default for ten days.
    #
    # The one wrinkle is that GroupMoveAbsolute BLOCKS its socket until the move finishes,
    # which for a move this long is forever. So it is issued on a second connection, kept
    # only for that purpose, leaving the primary socket free to read position and to abort.
    # The XPS accepts many simultaneous logins; this is Newport's own idiom for a
    # non-blocking move.
    def spin(
        self, address: XPSAddress, velocity: float, acceleration: Optional[float] = None
    ) -> None:
        """Start (or re-rate) continuous rotation at ``velocity`` units/s. Sign is direction.

        On a SpindleAxis group a re-rate is smooth -- the XPS ramps from the current
        velocity to the new one without stopping. On a SingleAxis group it is not: the
        move has to be aborted and re-issued, so the plate decelerates to a stop and ramps
        back up. Correct either way, but only one of them is seamless.
        """
        group, _ = self._split(address)
        if self.is_spindle(address):
            accel = self._default_accel(group, acceleration)
            err, _ = self.xps._xps.GroupSpinParametersSet(self.xps._sid, group, velocity, accel)
            self.xps.check_error(err, msg=f"spinning group '{group}'")
            return
        self._spin_by_long_move(address, velocity, acceleration)

    def stop_spin(self, address: XPSAddress, acceleration: Optional[float] = None) -> None:
        """Ramp a spinning group down to a stop and leave it enabled.

        Not the same as ``stop``/``kill``: this decelerates at the configured rate and
        leaves the group READY, so the next absolute move needs no re-initialize. On a
        direct-drive rotator carrying an optic, stopping hard is a shock load.

        ``GroupMoveAbort`` is the SingleAxis half of that and, despite the name, is NOT a
        hard stop -- the XPS decelerates through the positioner's configured profile.
        Measured on ours: ~41 degrees of ramp-down from 180 deg/s.
        """
        group, _ = self._split(address)
        if self.is_spindle(address):
            accel = self._default_accel(group, acceleration)
            err, _ = self.xps._xps.GroupSpinModeStop(self.xps._sid, group, accel)
            self.xps.check_error(err, msg=f"stopping spin on group '{group}'")
            return
        # Abort on the PRIMARY socket: the spin socket is blocked inside the long move and
        # cannot carry a command until that move ends -- which is what this is ending.
        err, _ = self.xps._xps.GroupMoveAbort(self.xps._sid, group)
        # -22 is "not allowed / no motion in progress". Stopping a plate that already
        # stopped is exactly what every lifecycle path does, so it is not an error.
        if err not in (0, -22):
            self.xps.check_error(err, msg=f"stopping spin on group '{group}'")
        self._join_spin_thread()

    def spin_current(self, address: XPSAddress) -> Tuple[float, float]:
        """The group's ACTUAL (velocity, acceleration) right now, read back from the XPS.

        Read-back, not the commanded setpoint: during the ramp this is still climbing.
        """
        group, _ = self._split(address)
        if self.is_spindle(address):
            err, velocity, acceleration = self.xps._xps.GroupSpinCurrentGet(self.xps._sid, group)
            self.xps.check_error(err, msg=f"reading spin state of group '{group}'")
            return float(velocity), float(acceleration)
        err, velocity = self.xps._xps.GroupVelocityCurrentGet(self.xps._sid, group, 1)
        self.xps.check_error(err, msg=f"reading velocity of group '{group}'")
        return float(velocity), float(self._spin_accel)

    # -- SingleAxis continuous rotation ------------------------------------ #
    def spin_headroom(self, address: XPSAddress, velocity: float) -> float:
        """Seconds of rotation left before the travel limit is reached, at ``velocity``.

        Only meaningful on the SingleAxis path, where a "spin" is a very long but finite
        move. Returns ``inf`` for a SpindleAxis group, which has no limit to run out of.
        """
        if self.is_spindle(address) or not velocity:
            return float("inf")
        _, positioner = self._split(address)
        low, high = self._travel_limits(positioner)
        here = self.get_position(address)
        return abs((high if velocity > 0 else low) - here) / abs(velocity)

    def _travel_limits(self, positioner: str) -> Tuple[float, float]:
        err, low, high = self.xps._xps.PositionerUserTravelLimitsGet(self.xps._sid, positioner)
        self.xps.check_error(err, msg=f"reading travel limits of '{positioner}'")
        return float(low), float(high)

    def _spin_socket(self) -> Tuple[object, int]:
        """A second logged-in connection, created on first use and then reused.

        It exists to absorb the blocking long move. Nothing else may be sent on it: it is
        unresponsive for as long as the plate is turning.
        """
        if self._spin_sid is None:
            driver = type(self.xps._xps)()
            sid = driver.TCP_ConnectToServer(self.host, self.port, self.timeout)
            if sid < 0:
                raise XPSError(f"could not open a second XPS connection to {self.host}")
            driver.Login(sid, self.username, self.password)
            self._spin_driver, self._spin_sid = driver, sid
        return self._spin_driver, self._spin_sid

    def _spin_by_long_move(
        self, address: XPSAddress, velocity: float, acceleration: Optional[float]
    ) -> None:
        group, positioner = self._split(address)
        if not velocity:
            raise XPSError("a spin velocity of zero would never start; use stop_spin()")

        # Re-rating means replacing the move in flight, so the old one has to end first.
        self.stop_spin(address)

        accel = float(acceleration) if acceleration is not None else DEFAULT_SPIN_ACCEL
        self._spin_accel = accel
        driver, sid = self._spin_socket()

        # Velocity is a POSITIONER profile setting, not a move argument -- set it before
        # the move, on the primary socket, so a failure here surfaces as an error rather
        # than as a plate silently turning at whatever rate it was left at.
        err, _ = self.xps._xps.PositionerSGammaParametersSet(
            self.xps._sid, positioner, abs(velocity), accel, _JERK_MIN_S, _JERK_MAX_S)
        self.xps.check_error(err, msg=f"setting spin velocity on '{positioner}'")

        low, high = self._travel_limits(positioner)
        # Stop just short of the limit. Landing exactly on it is a legal move, but it ends
        # with the axis pinned against its own boundary, where the next correction in that
        # direction fails; a margin keeps it recoverable without operator intervention.
        target = (high - _LIMIT_MARGIN) if velocity > 0 else (low + _LIMIT_MARGIN)

        def _run() -> None:
            # Returns only when the move ends -- normally by our abort, which answers -27.
            # Nothing here can be reported to the caller: by then the call that started the
            # spin has long returned, so a genuine fault is logged and read back off the
            # group status rather than raised into a thread nobody is joining.
            err, _msg = driver.GroupMoveAbsolute(sid, group, [target])
            if err not in (0, -27, -22):
                log.error("XPS: the spin move on '%s' ended with error %s", group, err)

        self._spin_thread = threading.Thread(
            target=_run, name=f"xps-spin-{group}", daemon=True)
        self._spin_thread.start()

    def _join_spin_thread(self) -> None:
        t = self._spin_thread
        self._spin_thread = None
        if t is not None and t.is_alive():
            # The abort has already gone out, so this is the deceleration ramp, not a wait
            # on the move itself. Bounded so a wedged socket cannot hang a lifecycle path.
            t.join(timeout=_SPIN_STOP_TIMEOUT_S)

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

    def group_categories(self) -> dict:
        """``{group_name: category}`` as declared in the XPS ``system.ini`` ``[GROUPS]``
        section -- ``SingleAxis``, ``SpindleAxis``, ``XY`` and so on.

        This is the authority on whether :meth:`spin` can work at all: continuous rotation
        is only defined on a ``SpindleAxis`` group. The table is read over FTP as part of
        connecting, so consulting it costs nothing and needs no extra round trip.
        """
        return {name: info.get("category", "?")
                for name, info in (self.xps.groups or {}).items()}

    def is_spindle(self, address: XPSAddress) -> bool:
        """Whether the group behind ``address`` is declared a SpindleAxis."""
        group, _ = self._split(address)
        return self.group_categories().get(group, "").lower() == "spindleaxis"

    def hardware_stages(self) -> dict:
        """Raw dict of stages the XPS knows about, from its system.ini.
        Useful for discovering the exact group/positioner names to use."""
        return self.xps.stages

    def __repr__(self) -> str:
        state = "connected" if self.connected else "disconnected"
        return f"XPSController(host={self.host!r}, {state}, devices={list(self._devices)})"
