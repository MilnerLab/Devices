"""
control_readout/base/device.py
===============================

`Device` — the addressed half of the reusable device framework.

A device is a friendly handle onto one axis of a `Controller`. It holds a reference to
its controller and an opaque ``address`` (interpreted by that controller) and delegates
all motion through the controller's addressed primitives. Because every controller
honours the same primitive interface, this single `Device` body drives any hardware
family; hardware-specific niceties (units, angle wrapping, config) live in thin
subclasses.
"""

from __future__ import annotations

from typing import Any, TYPE_CHECKING

if TYPE_CHECKING:
    from control_readout.base.controller import Controller


class DeviceError(Exception):
    """Raised when a device is used without a bound controller."""


class Device:
    """A single addressed axis attached to a `Controller`.

    Parameters
    ----------
    name : your own friendly label used to look the device up on the controller
           (e.g. "sample_x", "hwp"); independent of the hardware address.
    address : the controller-defined address of this axis (an int axis, a
              ``(group, positioner)`` pair, a hex bus address, ...).
    controller : the controller that owns the connection this device rides on.
    """

    #: Subclasses set this for nicer messages / logging ("mm", "deg", ...).
    units: str = ""

    def __init__(self, name: str, address: Any, controller: "Controller") -> None:
        self.name = name
        self.address = address
        self._controller: "Controller" = controller

    # -- controller binding ----------------------------------------------- #
    def _bind(self, controller: "Controller") -> None:
        """Internal: called by Controller.add_device()."""
        self._controller = controller

    @property
    def controller(self) -> "Controller":
        if self._controller is None:
            raise DeviceError(
                f"Device {self.name!r} has no controller. Pass controller=... "
                f"when constructing it, or call controller.add_device(...)."
            )
        return self._controller

    @controller.setter
    def controller(self, controller: "Controller") -> None:
        self._controller = controller

    @property
    def _lock(self):
        return self.controller._lock

    @property
    def attached(self) -> bool:
        """True while this device is registered with its controller
        (i.e. between start() and stop())."""
        c = self._controller
        return c is not None and c.devices.get(self.name) is self

    # -- device lifecycle ------------------------------------------------- #
    def start(self) -> "Device":
        """Attach this device to its controller. Idempotent. Returns self."""
        self.controller.add_device(self)
        return self

    def stop(self) -> None:
        """Detach this device from its controller. Idempotent."""
        if self._controller is not None:
            self._controller.remove_device(self)

    def __enter__(self) -> "Device":
        return self.start()

    def __exit__(self, exc_type, exc, tb) -> None:
        self.stop()

    # -- motion (delegated through the controller interface) -------------- #
    def initialize(self) -> None:
        """Prepare this axis for motion (motor on / group init, if applicable)."""
        with self._lock:
            self.controller.initialize(self.address)

    def home(self) -> None:
        """Run the home / origin search. Blocks until complete."""
        with self._lock:
            self.controller.home(self.address)

    def position(self) -> float:
        """Current position in the device's native units."""
        with self._lock:
            return self.controller.get_position(self.address)

    def move_to(self, value: float) -> None:
        """Absolute move. Blocks until complete."""
        with self._lock:
            self.controller.move_absolute(self.address, value)

    def move_by(self, delta: float) -> None:
        """Relative move by `delta` units. Blocks until complete."""
        with self._lock:
            self.controller.move_relative(self.address, delta)

    def abort(self) -> None:
        """Stop motion on this axis (does not disconnect)."""
        with self._lock:
            self.controller.stop(self.address)

    def __repr__(self) -> str:
        return f"{type(self).__name__}(name={self.name!r}, address={self.address!r})"
