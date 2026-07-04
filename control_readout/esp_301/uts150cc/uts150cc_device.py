from control_readout.base.device import Device
from control_readout.esp_301.controller import ESP301Controller


class UTS150CC(Device):
    """A Newport UTS150CC motorised linear stage (units: mm) on an ESP301 axis.

    Thin semantic wrapper over the generic Device: fixes the address to the
    integer ESP301 axis and adds mm conveniences.
    """

    units = "mm"

    def __init__(self, name: str, axis: int, controller: ESP301Controller) -> None:
        super().__init__(name, address=axis, controller=controller)

    @property
    def axis(self) -> int:
        return self.address

    def move(self, position: float) -> None:
        """Absolute move to `position` mm. Alias for move_to()."""
        self.move_to(position)
