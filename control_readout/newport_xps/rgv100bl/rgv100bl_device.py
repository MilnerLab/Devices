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

    def angle(self) -> Angle:
        """Alias for position(), reads in degrees."""
        return Angle(self.position(), AngleUnit.DEG)

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
