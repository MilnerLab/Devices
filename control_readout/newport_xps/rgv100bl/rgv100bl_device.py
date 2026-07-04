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
        self.positioner = positioner or f"{group}.Pos"
        super().__init__(name, address=(self.group, self.positioner), controller=controller)

    @property
    def _xps(self) -> "NewportXPS":  # noqa: F821 - runtime type from newportxps
        """The raw NewportXPS handle, for stage-specific low-level commands."""
        return self.controller.xps  # type: ignore[attr-defined]

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

        NOTE: this only works if the XPS group for this stage is configured as a
        SpindleAxis (continuous rotation). For a standard SingleAxis group the
        ±168° limit switches prevent endless rotation and you should use
        move_to / move_by instead. The underlying Spindle command name can vary
        by firmware generation, so verify against your controller's manual.
        """
        with self._lock:
            spin = getattr(self._xps, "set_velocity", None)
            # On a Spindle group, a velocity move = continuous rotation.
            # Many setups drive this via the raw command interface instead:
            #   self._xps._xps.GroupSpinParametersSet(self._xps._sid, self.group, velocity, accel)
            raise NotImplementedError(
                "Continuous spin requires a SpindleAxis group; wire this to your "
                "controller's GroupSpinParametersSet command. See the XPS manual."
            )

    def __repr__(self) -> str:
        return (
            f"{type(self).__name__}(name={self.name!r}, group={self.group!r}, "
            f"positioner={self.positioner!r})"
        )
