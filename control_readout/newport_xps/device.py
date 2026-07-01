"""
xps_motion.py
=============

A small, extensible framework for driving Newport XPS-controlled motion stages
(e.g. the RGV100BL rotation stage) from Python.

The design separates two concerns:

    XPSController   owns the single network connection to the XPS hardware and a
                    registry of attached devices. It knows nothing about whether a
                    device is rotary, linear, etc.

    Device (base)   common behaviour for any positioner: initialize, home, read
                    position, absolute/relative move, set velocity, stop.

    RotationStage   rotary-specific niceties (degrees, optional angle wrapping,
                    continuous-spin helper). Use this for the RGV100BL.

    LinearStage     linear-specific niceties (millimetres).

To support new hardware later, subclass `Device` (or `RotationStage`/`LinearStage`)
and register an instance with `controller.add_device(...)`. Nothing in the
controller needs to change.

Prerequisite
------------
The groups/positioners referenced here ("group" and "positioner" names) must
already exist in the XPS configuration. You create those once through the XPS
web interface (define the Group, assign the RGV100BL positioner, run tuning /
auto-scaling), not from Python. This module only *drives* what the XPS already
knows about.

Requires:  pip install newportxps
"""

# --------------------------------------------------------------------------- #
#  Device base class
# --------------------------------------------------------------------------- #
from abc import ABC
from typing import Optional

from control_readout.newport_xps.controller import XPSController, XPSError


class XPSDevice(ABC):
    
    """Base class for anything plugged into the XPS.
 
    A device is identified by an XPS *group* and a *positioner* within it.
    For a single-axis stage the positioner is conventionally "<group>.Pos",
    which is the default if you don't pass `positioner`.
 
    `name` is your own friendly label used to look the device up on the
    controller (e.g. "rot", "sample_x"); it is independent of the XPS names.
    """
 
    #: Subclasses set this for nicer messages / logging ("deg", "mm", ...).
    units: str = ""
 
    def __init__(
        self,
        name: str,
        group: str,
        controller: XPSController,
        positioner: Optional[str] = None,
    ) -> None:
        self.name = name
        self.group = group
        self.positioner = positioner or f"{group}.Pos"
        self._controller:XPSController = controller
 
    @property
    def controller(self) -> XPSController:
        if self._controller is None:
            raise XPSError(
                f"Device {self.name!r} has no controller. Pass controller=... "
                f"when constructing it, or call controller.add_device(...)."
            )
        return self._controller
 
    @controller.setter
    def controller(self, controller: Optional[XPSController]) -> None:
        self._controller = controller
 
    @property
    def _xps(self) -> "NewportXPS":
        return self.controller.xps
 
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
        """Attach this device to its controller. Idempotent.
 
        You manage the controller's connection yourself; this only registers
        the device so the controller knows about it. Returns self.
        """
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
 
    # -- lifecycle (low-level) -------------------------------------------- #
    def initialize(self) -> None:
        """Power on / initialize the group. Required once after power-up."""
        with self._lock:
            self._xps.initialize_group(self.group)
 
    def home(self) -> None:
        """Run the home search. Required after initialize, before any move.
        For the RGV100BL this seeks the once-per-revolution index pulse."""
        with self._lock:
            self._xps.home_group(self.group)
 
    # -- motion ----------------------------------------------------------- #
    def position(self) -> float:
        """Current position in the device's native units."""
        with self._lock:
            return self._xps.get_stage_position(self.positioner)
 
    def set_velocity(self, velocity: float, acceleration: Optional[float] = None) -> None:
        """Set max velocity (and optionally acceleration) for subsequent moves."""
        with self._lock:
            self._xps.set_velocity(self.positioner, velo=velocity, accel=acceleration)
 
    def move_to(self, value: float) -> None:
        """Absolute move. Blocks until the XPS reports the move complete."""
        with self._lock:
            self._xps.move_stage(self.positioner, value, relative=False)
 
    def move_by(self, delta: float) -> None:
        """Relative move by `delta` units."""
        with self._lock:
            self._xps.move_stage(self.positioner, delta, relative=True)
 
    def abort(self) -> None:
        """Abort motion on this device's group (does not disconnect)."""
        with self._lock:
            # abort_group exists on newportxps; fall back to a group restart if not.
            abort = getattr(self._xps, "abort_group", None)
            if callable(abort):
                abort(self.group)
            else:
                self._xps.initialize_group(self.group)
 
    def __repr__(self) -> str:
        return (
            f"{type(self).__name__}(name={self.name!r}, group={self.group!r}, "
            f"positioner={self.positioner!r})"
        )
 
