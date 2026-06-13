from __future__ import annotations

from base_core.framework.shm.writer_subprocess_main import WriterSubprocessMain

from oscilloscope.buffer import ScopeBuffer
from oscilloscope.config import ScopeConfig
from oscilloscope.oscilloscope_worker import OscilloscopeWorker


class OscilloscopeProcess(WriterSubprocessMain):
    """Subprocess entry point for the oscilloscope (streams traces into ScopeBuffer)."""

    def setup(self) -> None:
        self.register_buffer_class(ScopeBuffer)
        self.register_worker(
            OscilloscopeWorker(
                bus=self.bus,
                connector=self.connector,
                config=ScopeConfig(),
                get_slot=lambda: self.get_granted_slot(ScopeBuffer),
                get_buffer=lambda: self.get_buffer(ScopeBuffer),
                notify_written=lambda slot, item_id, ts: self.notify_written(
                    ScopeBuffer, slot, item_id, ts
                ),
            )
        )


if __name__ == "__main__":
    OscilloscopeProcess.main()
