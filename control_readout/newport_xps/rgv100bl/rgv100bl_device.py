from typing import Optional

from base_core.math.enums import AngleUnit
from base_core.math.models import Angle
from control_readout.base.device import Device
from control_readout.newport_xps.controller import XPSController


#: Module-level alias of :attr:`RGV.MAX_SPIN_DEG_S`, for callers sizing a rate limit.
MAX_SPIN_DEG_S = 720.0


class RGV(Device):
    """A rotary positioner such as the Newport RGV100BL (units: degrees).

    An XPS axis is addressed by a *group* and a *positioner* within it; for a
    single-axis stage the positioner is conventionally "<group>.Pos", the default
    if you don't pass `positioner`. This fixes the framework `Device` address to
    that ``(group, positioner)`` pair and adds rotary conveniences.
    """

    units = "deg"

    #: RGV100 series maximum angular velocity, deg/s (= 2 rev/s). Beyond this the
    #: controller faults, so it is checked here where the number can carry its meaning.
    MAX_SPIN_DEG_S = 720.0

    def __init__(
        self,
        name: str,
        group: str,
        controller: XPSController,
        positioner: Optional[str] = None,
    ) -> None:
        self.group = group
        # XPS stage keys are the full "Group.Positioner" name. Accept a bare
        # positioner ("POSITIONER") or the default ("Pos") and prefix the group
        # so we always address the stage the way move_stage() looks it up.
        positioner = positioner or "Pos"
        if "." not in positioner:
            positioner = f"{group}.{positioner}"
        self.positioner = positioner
        super().__init__(name, address=(self.group, self.positioner), controller=controller)

    @property
    def _xps(self) -> "NewportXPS":  # noqa: F821 - runtime type from newportxps
        """The raw NewportXPS handle, for stage-specific low-level commands."""
        return self.controller.xps  # type: ignore[attr-defined]

    def initialize(self) -> None:
        """Kill the group first, then initialize it.

        On the RGV, issuing ``GroupInitialize`` against a group that is already
        initialized raises a driver fault. Killing the group back to NOTINIT
        first makes initialization idempotent and fault-free.
        """
        with self._lock:
            self.controller.kill(self.address)  # type: ignore[attr-defined]
            self.controller.initialize(self.address)

    def set_velocity(self, velocity: float, acceleration: Optional[float] = None) -> None:
        """Set max velocity (and optionally acceleration) for subsequent moves."""
        with self._lock:
            self.controller.set_velocity(self.address, velocity, acceleration)  # type: ignore[attr-defined]

    def rotate(self, angle: Angle) -> None:
        self.move_to(angle.Deg)

    def raw_position(self) -> float:
        """The controller's own accumulated angle, unwrapped, in degrees.

        This is the number the XPS actually tracks against its travel limits, and it grows
        without bound while the plate free-runs -- 1,893 deg after a few spins, millions
        after an hour. Everything the operator sees uses :meth:`angle` instead; this is
        here for the limits arithmetic and for diagnostics.
        """
        return super().position()

    def position(self) -> float:
        """The plate's orientation in [0, 360) degrees."""
        return self.raw_position() % 360.0

    def angle(self) -> Angle:
        """Alias for position(), reads in degrees."""
        return Angle(self.position(), AngleUnit.DEG)

    def move_to(self, value: float) -> None:
        """Rotate to an orientation, by the SHORTEST path. Blocks until complete.

        A rotation stage has no absolute target, only an orientation, and the controller's
        own coordinate is a running total that a spin drives arbitrarily far from zero.
        Passing an orientation straight to ``move_absolute`` therefore commands an unwind:
        after spinning the plate to a raw 1,893 deg, asking for 93 deg means 1,800 deg of
        rewinding -- five full turns to reach an orientation the plate is ALREADY at.

        So the target is resolved as a relative move instead: the signed difference to the
        requested orientation, folded into (-180, +180]. The plate reaches the same
        orientation, never turns more than half a revolution to get there, and the raw
        coordinate is left wherever the shortest path puts it.
        """
        with self._lock:
            raw = super().position()
            # (-180, +180]: the shortest of the two ways round.
            delta = (float(value) - raw + 180.0) % 360.0 - 180.0
            self.controller.move_relative(self.address, delta)  # type: ignore[attr-defined]

    def spin(self, velocity_deg_s: float, acceleration: Optional[float] = None) -> None:
        """Start continuous rotation at ``velocity_deg_s`` deg/s. Sign sets the direction.

        Returns as soon as the XPS has accepted the command -- the stage is still ramping
        up and keeps turning until :meth:`stop_spin`. Nothing else may command a position
        while this is running.

        Works whichever way the group is declared in the XPS ``system.ini``. A SpindleAxis
        group spins natively; a SingleAxis one is spun by a very long move, which the
        RGV100BL's +-165,000,000 degree travel limits make effectively continuous -- 63
        hours at the ceiling below, ten days at the usual 0.5 rev/s. See
        ``XPSController.spin`` for the mechanism and ``spin_headroom`` for what is left.
        """
        speed = float(velocity_deg_s)
        if abs(speed) > MAX_SPIN_DEG_S:
            raise ValueError(
                f"{abs(speed):.1f} deg/s exceeds the RGV100's {MAX_SPIN_DEG_S:.0f} deg/s "
                f"maximum ({MAX_SPIN_DEG_S / 360.0:.0f} rev/s)"
            )
        with self._lock:
            self.controller.spin(self.address, speed, acceleration)  # type: ignore[attr-defined]

    def stop_spin(self, acceleration: Optional[float] = None) -> None:
        """Ramp a spin down to a stop, leaving the group ready for ordinary moves.

        Deliberately not ``abort()``/``kill()``: those de-energise the group and leave it
        needing a re-initialize before the next ordinary move. Both spin strategies stop
        through the positioner's own deceleration profile instead -- measured at ~41
        degrees of ramp-down from 180 deg/s -- so the optic is never stopped harder than
        the stage was configured to stop it.
        """
        with self._lock:
            self.controller.stop_spin(self.address, acceleration)  # type: ignore[attr-defined]

    def spin_velocity(self) -> float:
        """The stage's actual angular velocity in deg/s, read back from the controller."""
        with self._lock:
            velocity, _accel = self.controller.spin_current(self.address)  # type: ignore[attr-defined]
        return velocity

    def __repr__(self) -> str:
        return (
            f"{type(self).__name__}(name={self.name!r}, group={self.group!r}, "
            f"positioner={self.positioner!r})"
        )
