from __future__ import annotations

from dataclasses import dataclass


@dataclass
class ScopeConfig:
    """Configuration for the oscilloscope (Tektronix TBS2012C).

    Hardware only. Whether a mock stands in for the instrument is a start-time
    decision carried by ``ConnectionMode`` on the start message, and the synthetic
    trace's shape belongs to the mock that draws it — see ``oscilloscope.mock_params``.

    ``channels``/``n_samples`` define the shared-memory trace shape and must match the
    buffer spec.
    """

    resource: str = ""           # VISA resource string (e.g. "USB0::0x0699::...")
    channels: int = 2            # CH1 = signal, CH2 reserved for analog-position sync (Q4)
    n_samples: int = 2000        # record length per trace
    sample_rate_hz: float = 1.0e9  # TBS2012C: up to 1 GS/s
