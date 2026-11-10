from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class PressureGaugeConfig:
    """One pressure gauge on the rig.

    ``port`` is the COM port the gauge controller sits on (e.g. ``"COM5"``). While it is
    ``None`` the gauge is read from a mock source, so the readout can be laid out and
    tested before the hardware is wired up.
    """
    name: str
    port: str | None = None


# Order here is the order the rows appear in the readout.
PRESSURE_GAUGES: tuple[PressureGaugeConfig, ...] = (
    PressureGaugeConfig("Source"),
    PressureGaugeConfig("VMI"),
)
