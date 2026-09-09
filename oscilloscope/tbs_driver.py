"""
Real Tektronix TBS2012C driver via PyVISA/SCPI (M1.D.1b).

Not exercised in CI (needs the instrument + a VISA backend, e.g. pyvisa-py or NI-VISA).
Selected when the worker is started in ``ConnectionMode.DEVICE`` and the instrument
answers. Mirrors :class:`oscilloscope.mock_driver.MockScope` so the worker is
driver-agnostic.

SCPI flow per channel: set DATa source/encoding/range, read the WFMOutpre scaling
preamble (including ``XINcr``, the sample interval the display's time axis is drawn
from), ``CURVe?`` for the raw samples, then apply ``volts = (raw - YOFf)*YMUlt + YZEro``.
"""
from __future__ import annotations

import time

import numpy as np

from oscilloscope.config import ScopeConfig
from oscilloscope.models import ScopeTrace


class TbsScope:
    def __init__(self, config: ScopeConfig) -> None:
        self._config = config
        self._rm = None
        self._dev = None

    def open(self) -> None:
        import pyvisa  # lazy: only needed for real hardware

        self._rm = pyvisa.ResourceManager()
        self._dev = self._rm.open_resource(self._config.resource)
        self._dev.timeout = 10_000  # ms

    def apply_config(self) -> None:
        d = self._require()
        d.write("HEADer OFF")
        d.write(f"HORizontal:RECOrdlength {self._config.n_samples}")
        d.write("DATa:ENCdg RIBinary")  # signed integer, big-endian
        d.write("DATa:WIDth 2")
        d.write("DATa:STARt 1")
        d.write(f"DATa:STOP {self._config.n_samples}")
        d.write("ACQuire:STOPAfter SEQuence")  # single-sequence acquisition

    def close(self) -> None:
        try:
            if self._dev is not None:
                self._dev.close()
        finally:
            self._dev = None
            if self._rm is not None:
                self._rm.close()
                self._rm = None

    def acquire_trace(self) -> ScopeTrace:
        d = self._require()
        # Stamped before the trigger is armed: the stamp means "this capture began here",
        # which is what a freshness gate can reason about. See ScopeTrace.timestamp_ns.
        started_ns = time.time_ns()
        d.write("ACQuire:STATE RUN")
        d.query("*OPC?")  # wait for the sequence to complete
        rows = []
        dt_s = 0.0
        for ch in range(1, self._config.channels + 1):
            d.write(f"DATa:SOUrce CH{ch}")
            ymult = float(d.query("WFMOutpre:YMUlt?"))
            yoff = float(d.query("WFMOutpre:YOFf?"))
            yzero = float(d.query("WFMOutpre:YZEro?"))
            # Read per acquisition, not once at open: the horizontal scale is a physical
            # knob an operator turns while the stream is running, and a time base cached
            # at open would silently mislabel every trace after they did.
            dt_s = float(d.query("WFMOutpre:XINcr?"))
            raw = d.query_binary_values("CURVe?", datatype="h", container=np.ndarray)
            volts = (raw.astype(np.float64) - yoff) * ymult + yzero
            rows.append(volts)
        samples = np.vstack(rows).astype(np.float64)
        return ScopeTrace(samples=samples, timestamp_ns=started_ns, dt_s=dt_s)

    def _require(self):
        if self._dev is None:
            raise RuntimeError("TbsScope: not open")
        return self._dev
