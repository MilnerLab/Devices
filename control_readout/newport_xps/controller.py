
from __future__ import annotations

import threading
from abc import ABC
from typing import Dict, Optional

try:
    from newportxps import NewportXPS
except ImportError:  # keep the module importable for inspection without hardware
    NewportXPS = None


class XPSError(Exception):
    """Raised for connection or device errors inside this wrapper."""

class XPSController:
    """Owns the connection to an XPS controller and a registry of devices.
 
    Example
    -------
    >>> ctrl = XPSController("192.168.0.254", password="Administrator")
    >>> rot = ctrl.add_device(RotationStage("rot", group="Rot"))
    >>> ctrl.connect()
    >>> rot.home()
    >>> rot.move_to(90.0)
    >>> ctrl.disconnect()
 
    Or as a context manager (connects on enter, disconnects on exit):
 
    >>> with XPSController("192.168.0.254") as ctrl:
    ...     rot = ctrl.add_device(RotationStage("rot", group="Rot"))
    ...     rot.home()
    ...     rot.move_to(90.0)
    """
 
    def __init__(
        self,
        host: str,
        username: str = "Administrator",
        password: str = "Administrator",
        port: int = 5001,
        timeout: float = 10.0,
    ) -> None:
        self.host = host
        self.username = username
        self.password = password
        self.port = port
        self.timeout = timeout
 
        self._xps: Optional["NewportXPS"] = None
        self._devices: Dict[str, "Device"] = {}
        # Serialize hardware access; moves on one connection shouldn't interleave.
        self._lock = threading.RLock()
 
    # -- connection ------------------------------------------------------- #
    @property
    def connected(self) -> bool:
        return self._xps is not None
 
    @property
    def xps(self) -> "NewportXPS":
        """The underlying NewportXPS object (raises if not connected)."""
        if self._xps is None:
            raise XPSError("Controller is not connected. Call connect() first.")
        return self._xps
 
    def connect(self) -> "XPSController":
        if NewportXPS is None:
            raise XPSError(
                "newportxps is not installed. Run: pip install newportxps"
            )
        if self._xps is None:
            self._xps = NewportXPS(
                self.host,
                username=self.username,
                password=self.password,
                port=self.port,
                timeout=self.timeout,
            )
        return self
 
    def disconnect(self) -> None:
        if self._xps is not None:
            try:
                self._xps.disconnect()
            finally:
                self._xps = None
 
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
            raise XPSError(f"A different device named {device.name!r} is already registered.")
        device._bind(self)
        self._devices[device.name] = device
        return device
 
    def remove_device(self, device) -> Optional["Device"]:
        """Unregister (detach) a device from this controller.
 
        Accepts either the device's name or the Device instance, and drops it
        from the registry. The device keeps its reference to this controller,
        so it can re-attach later via device.start(). Returns the removed
        device, or None if it wasn't registered.
 
        Note: this is Python-side bookkeeping only; it does not stop or power
        down hardware. Call device.abort() first if a move is in progress.
        """
        name = device.name if isinstance(device, Device) else device
        dev = self._devices.pop(name, None)
        return dev
 
    @property
    def devices(self) -> Dict[str, "Device"]:
        return dict(self._devices)
 
    def __getitem__(self, name: str) -> "Device":
        try:
            return self._devices[name]
        except KeyError:
            raise KeyError(f"No device named {name!r} registered.") from None
 
    # -- convenience ------------------------------------------------------ #
    def initialize_all(self) -> None:
        """Initialize + home every registered device (typical power-up sequence)."""
        for dev in self._devices.values():
            dev.initialize()
            dev.home()
 
    def status(self) -> str:
        """Human-readable status of the whole controller (groups + stages)."""
        return self.xps.status_report()
 
    def hardware_stages(self) -> dict:
        """Raw dict of stages the XPS knows about, from its system.ini.
        Useful for discovering the exact group/positioner names to use."""
        return self.xps.stages
 
    # -- context manager -------------------------------------------------- #
    def __enter__(self) -> "XPSController":
        return self.connect()
 
    def __exit__(self, exc_type, exc, tb) -> None:
        self.disconnect()
 
    def __repr__(self) -> str:
        state = "connected" if self.connected else "disconnected"
        return f"XPSController(host={self.host!r}, {state}, devices={list(self._devices)})"
 
