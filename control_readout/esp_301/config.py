from __future__ import annotations

from dataclasses import dataclass, field


@dataclass
class Esp301Config:
    """Configuration for the ESP301 3-axis motion controller.

    The ESP301 drives the probe, delay and truncation stages on one serial port,
    addressed per-axis (D8/Q1). ``axes`` lists the controller axis numbers to manage
    and poll; the logical name → axis mapping is kept here so callers refer to roles.
    """

    port: str = "COM3"
    baud: int = 19200
    poll_hz: float = 20.0
    axes: tuple[int, ...] = (1, 2, 3)

    # logical role -> controller axis number
    probe_axis: int = 1
    delay_axis: int = 2
    truncation_axis: int = 3

    # optional default closed-loop velocity per axis (controller units/s); None = leave as-is
    default_velocity: float | None = None
