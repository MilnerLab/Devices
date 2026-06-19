from __future__ import annotations

from base_core.ipc.subprocess_main import BaseSubprocessMain

from oscilloscope.buffer import ScopeBuffer
from oscilloscope.config import ScopeConfig
from oscilloscope.oscilloscope_worker import OscilloscopeWorker


class OscilloscopeProcess(BaseSubprocessMain):
    """Subprocess entry point for the oscilloscope (streams traces into ScopeBuffer)."""

    def setup(self) -> None:
        self.register_buffer_class(ScopeBuffer)
        self.register_worker(
            OscilloscopeWorker(
                bus=self.bus,
                connector=self.connector,
                config=ScopeConfig(),
                get_buffer=lambda: self.get_buffer(ScopeBuffer),
            )
        )


if __name__ == "__main__":
    OscilloscopeProcess.main()
