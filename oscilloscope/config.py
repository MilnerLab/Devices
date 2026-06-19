from __future__ import annotations

from dataclasses import dataclass


@dataclass
class ScopeConfig:
    """Configuration for the oscilloscope (Tektronix TBS2012C).

    ``mock=True`` uses the synthetic-trace driver (no hardware); the real PyVISA/SCPI
    driver is selected when ``mock=False``. ``channels``/``n_samples`` define the
    shared-memory trace shape (must match the buffer spec).
    """

    resource: str = ""           # VISA resource string (e.g. "USB0::0x0699::...")
    channels: int = 2            # CH1 = signal, CH2 reserved for analog-position sync (Q4)
    n_samples: int = 2000        # record length per trace
    sample_rate_hz: float = 1.0e9  # TBS2012C: up to 1 GS/s
    mock: bool = True

    # --- mock-signal parameters (envelope-bounded chirped sinusoid vs time) ---
    mock_center_frac: float = 0.5
    mock_width_frac: float = 0.18
    mock_chirp: float = 9.0      # fringe count across the record
    mock_phase0: float = 0.3
    mock_noise: float = 0.01
    mock_seed: int | None = None
