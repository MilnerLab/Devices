from typing import Optional

from base_core.math.enums import AngleUnit
from base_core.math.models import Angle
from control_readout.newport_xps.controller import XPSController
from control_readout.newport_xps.device import XPSDevice


class RGV(XPSDevice):
    """A rotary positioner such as the Newport RGV100BL (units: degrees).

    Adds rotary conveniences on top of the base Device.
    """

    units = "deg"

    def __init__(
        self,
        name: str,
        group: str,
        controller: XPSController,
        positioner: Optional[str] = None,
    ) -> None:
        """
        Parameters
        ----------
        wrap : if True, `move_to` targets are normalised into [0, 360).
               Leave False for a point-to-point group with ±168° limits;
               set True only for a continuously-rotating (Spindle) setup.
        """
        super().__init__(name, group, controller, positioner)

    def rotate(self, angle: Angle) -> None:
        super().move_to(angle.Deg)

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
