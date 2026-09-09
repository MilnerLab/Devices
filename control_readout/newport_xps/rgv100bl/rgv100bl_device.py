from typing import Optional

from base_core.math.enums import AngleUnit
from base_core.math.models import Angle
from control_readout.base.device import Device
from control_readout.newport_xps.controller import XPSController


class RGV(Device):
    """A rotary positioner such as the Newport RGV100BL (units: degrees).

    An XPS axis is addressed by a *group* and a *positioner* within it; for a
    single-axis stage the positioner is conventionally "<group>.Pos", the default
    if you don't pass `positioner`. This fixes the framework `Device` address to
    that ``(group, positioner)`` pair and adds rotary conveniences.
    """

    units = "deg"

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

    def spin(self, velocity_deg_s: float) -> None:
        """Start continuous rotation at the given velocity.

        Delegated to the controller rather than reaching for the raw XPS handle, so a
        mocked controller can stand in for it like any other primitive. The real
        controller still refuses: continuous rotation needs the group configured as a
        SpindleAxis, and on a standard SingleAxis group the ±168° limit switches make
        endless rotation impossible.
        """
        with self._lock:
            self.controller.spin(self.address, velocity_deg_s)  # type: ignore[attr-defined]

    def stop_spin(self) -> None:
        """Ramp a free-running plate to a stop."""
        with self._lock:
            self.controller.stop_spin(self.address)  # type: ignore[attr-defined]

    def __repr__(self) -> str:
        return (
            f"{type(self).__name__}(name={self.name!r}, group={self.group!r}, "
            f"positioner={self.positioner!r})"
        )
