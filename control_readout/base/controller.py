"""
control_readout/base/controller.py
===================================

`Controller` — the transport-and-registry half of the reusable device framework.

A controller owns a single hardware connection (serial, TCP, ...) and a registry of
`Device` objects that share it. Each device is addressed by an opaque ``address`` whose
meaning is defined entirely by the concrete controller (an axis number for an ESP301, a
``(group, positioner)`` pair for an XPS, a hex bus address for an Elliptec).

Concrete controllers subclass `Controller` and:
  - implement the transport hooks `_open()` / `_close()`, and
  - implement the addressed motion primitives (`home`, `move_absolute`,
    `move_relative`, `get_position`, `stop`), optionally overriding `initialize`.

Everything a `Device` needs to talk to hardware goes through these primitives, so the
same `Device` body drives any controller that honours the interface.
"""

from __future__ import annotations

import threading
from abc import ABC, abstractmethod
from typing import Any, Dict, Optional, TYPE_CHECKING

if TYPE_CHECKING:
    from control_readout.base.device import Device


class ControllerError(Exception):
    """Raised for connection or registry errors inside a controller."""


class Controller(ABC):
    """Owns one hardware connection and a registry of addressed devices.

    Subclasses provide the transport (`_open`/`_close`) and the addressed motion
    primitives. The registry, connection lifecycle and locking are shared here.
    """

    def __init__(self) -> None:
        self._connected: bool = False
        self._devices: Dict[str, "Device"] = {}
        # Serialize hardware access; commands on one connection shouldn't interleave.
        self._lock = threading.RLock()

    # -- transport hooks (subclass) --------------------------------------- #
    @abstractmethod
    def _open(self) -> None:
        """Open the underlying hardware connection. Called once by connect()."""
        ...

    @abstractmethod
    def _close(self) -> None:
        """Close the underlying hardware connection. Called by disconnect()."""
        ...

    # -- connection ------------------------------------------------------- #
    @property
    def connected(self) -> bool:
        return self._connected

    def connect(self) -> "Controller":
        if not self._connected:
            self._open()
            self._connected = True
        return self

    def disconnect(self) -> None:
        if self._connected:
            try:
                self._close()
            finally:
                self._connected = False

    def _require_connected(self) -> None:
        if not self._connected:
            raise ControllerError(
                f"{type(self).__name__} is not connected. Call connect() first."
            )

    # -- addressed motion primitives (subclass) --------------------------- #
    def initialize(self, address: Any) -> None:
        """Prepare an axis for motion (e.g. energise the motor / init the group).

        Default is a no-op; override where the hardware needs it."""
        pass

    @abstractmethod
    def home(self, address: Any) -> None:
        """Run the home / origin search for the addressed axis. Blocks until done."""
        ...

    @abstractmethod
    def move_absolute(self, address: Any, value: float) -> None:
        """Absolute move of the addressed axis. Blocks until motion completes."""
        ...

    @abstractmethod
    def move_relative(self, address: Any, delta: float) -> None:
        """Relative move of the addressed axis. Blocks until motion completes."""
        ...

    @abstractmethod
    def get_position(self, address: Any) -> float:
        """Current position of the addressed axis in its native units."""
        ...

    @abstractmethod
    def stop(self, address: Any) -> None:
        """Stop / abort motion on the addressed axis (does not disconnect)."""
        ...

    # -- device registry -------------------------------------------------- #
    def add_device(self, device: "Device") -> "Device":
        """Register (attach) a device and bind it to this controller.

        Normally you don't call this directly — a device attaches itself via
        device.start(). Returns the device. No-op if it's already registered
        under the same name.
        """
        existing = self._devices.get(device.name)
        if existing is device:
            return device
        if existing is not None:
            raise ControllerError(
                f"A different device named {device.name!r} is already registered."
            )
        device._bind(self)
        self._devices[device.name] = device
        return device

    def remove_device(self, device) -> Optional["Device"]:
        """Unregister (detach) a device. Accepts a name or a Device instance.

        Python-side bookkeeping only; it does not stop or power down hardware.
        """
        from control_readout.base.device import Device

        name = device.name if isinstance(device, Device) else device
        return self._devices.pop(name, None)

    @property
    def devices(self) -> Dict[str, "Device"]:
        return dict(self._devices)

    def __getitem__(self, name: str) -> "Device":
        try:
            return self._devices[name]
        except KeyError:
            raise KeyError(f"No device named {name!r} registered.") from None

    def initialize_all(self) -> None:
        """Initialize + home every registered device (typical power-up sequence)."""
        for dev in self._devices.values():
            dev.initialize()
            dev.home()

    # -- context manager -------------------------------------------------- #
    def __enter__(self) -> "Controller":
        return self.connect()

    def __exit__(self, exc_type, exc, tb) -> None:
        self.disconnect()

    def __repr__(self) -> str:
        state = "connected" if self._connected else "disconnected"
        return f"{type(self).__name__}({state}, devices={list(self._devices)})"
