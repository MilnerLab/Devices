from __future__ import annotations

from dataclasses import dataclass
from mailbox import Message

from base_core.framework.subprocess.messages import Kind



@dataclass(frozen=True)
class Rotate(Message):
    """Relative rotation by the given angle (radians)."""
    NAME = "rotate"
    KIND = Kind.COMMAND
    angle_rad: float = 0.0


@dataclass(frozen=True)
class HomeRotator(Message):
    """Home the rotator to its reference position."""
    NAME = "home_rotator"
    KIND = Kind.COMMAND


@dataclass(frozen=True)
class RotatorMoved(Message):
    """Emitted after a rotation completes; carries the new absolute position."""
    NAME = "rotator_moved"
    KIND = Kind.EVENT
    position_rad: float = 0.0


@dataclass(frozen=True)
class RotatorHomed(Message):
    """Emitted after homing completes."""
    NAME = "rotator_homed"
    KIND = Kind.EVENT
