
from base_core.framework.subprocess.shared_memory.shared_memory_base_messages import base_registry
from base_core.framework.subprocess.subprocess_app import SubprocessApp
from spm_002.spectrometer_worker import SpectrometerWorker


if __name__ == "__main__":
    app = SubprocessApp(base_registry(), source="spectrometer")
    app.add_worker(SpectrometerWorker())
    app.run()
