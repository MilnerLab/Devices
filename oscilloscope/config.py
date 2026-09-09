from __future__ import annotations

from dataclasses import dataclass


@dataclass
class ScopeConfig:
    """Configuration for the oscilloscope (Tektronix TBS2012C).

    Hardware only. Whether a mock stands in for the instrument is a start-time
    decision carried by ``ConnectionMode`` on the start message, and the synthetic
    trace's shape belongs to the mock that draws it — see ``oscilloscope.mock_params``.

    ``channels``/``n_samples`` define the shared-memory trace shape and must fit inside a
    ``ScopeMemorySpec`` slot, which is sized to the instrument's largest record rather
    than to this config. A config that overruns a slot is rejected by the worker.
    """

    #: Tektronix TDS 2012C, USBTMC. Verified 2026-07-20; note this is a TDS, not the
    #: TBS2012C that ``oscilloscope/tbs_driver.py`` targets (defect G8). Lives here, with
    #: the device, so the oscilloscope module does not have to import a routine's config
    #: to know what to open.
    resource: str = "USB0::0x0699::0x03A3::C015100::INSTR"
    channels: int = 2            # CH1 = signal, CH2 reserved for analog-position sync (Q4)
    n_samples: int = 2500        # record length per trace
    sample_rate_hz: float = 1.0e9  # TBS2012C: up to 1 GS/s; only the mock's time base
