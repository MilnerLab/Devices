from __future__ import annotations

from base_core.framework.shm.writer_subprocess_main import WriterSubprocessMain
from spm_002.buffer import SpectrumBuffer
from spm_002.config import SpectrometerConfig
from spm_002.spectrometer_worker import SpectrometerWorker


class SpectrometerProcess(WriterSubprocessMain):
    """
    Subprocess entry point for the spectrometer.

    Wires up SpectrumBuffer attachment and the SpectrometerWorker, then
    delegates the IPC message loop to WriterSubprocessMain.

    Launched by SpectrometerService via:
        python32 -m spm_002.spectrometer_process <pipe_fd>
    """

    def setup(self) -> None:
        self.register_buffer_class(SpectrumBuffer)
        self.register_worker(
            SpectrometerWorker(
                bus=self.bus,
                connector=self.connector,
                config=SpectrometerConfig(),
                get_slot=lambda: self.get_granted_slot(SpectrumBuffer),
                get_buffer=lambda: self.get_buffer(SpectrumBuffer),
                notify_written=lambda slot, item_id, ts: self.notify_written(
                    SpectrumBuffer, slot, item_id, ts
                ),
            )
        )


if __name__ == "__main__":
    SpectrometerProcess.main()
