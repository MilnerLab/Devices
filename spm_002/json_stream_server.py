# acquisition/json_stream_server.py
import json
import threading
from datetime import datetime
from typing import Dict

from .spm002 import Spectrometer, SpectrometerConfig, SpectrumData
from .runtime_config import ConfigManager
from .config_gui import ConfigWindow


# ---------------------------------------------------------------------------
# Helper functions to convert data to JSON-serializable dicts
# ---------------------------------------------------------------------------

def spectrum_to_frame(spectrum: SpectrumData) -> Dict:
    return {
        "type": "frame",
        "timestamp": spectrum.timestamp.isoformat(),
        "device_index": spectrum.device_index,
        "counts": spectrum.counts,
        # wavelengths are static and sent once in the 'meta' message
    }


def config_to_message(config: SpectrometerConfig) -> Dict:
    """
    Convert the current SpectrometerConfig to a JSON-serializable dict.

    This is sent whenever a new configuration is applied to the device.
    """
    return {
        "type": "config",
        "timestamp": datetime.now().isoformat(),
        "device_index": config.device_index,
        "exposure_ms": config.exposure_ms,
        "average": config.average,
        "dark_subtraction": config.dark_subtraction,
        "mode": config.mode,
        "scan_delay": config.scan_delay,
    }


def meta_from_first_spectrum(spectrum: SpectrumData) -> Dict:
    """
    Build the static 'meta' message from the first acquired spectrum.

    Only contains properties that do not change during the run.
    """
    return {
        "type": "meta",
        "device_index": spectrum.device_index,
        "num_pixels": len(spectrum),
        "wavelengths": spectrum.wavelengths,  # may be None
    }


# ---------------------------------------------------------------------------
# Acquisition loop (runs in background thread)
# ---------------------------------------------------------------------------

def acquisition_loop(manager: ConfigManager, stop_event: threading.Event) -> None:
    """
    Background thread that:
    - waits for an initial configuration from the GUI
    - opens the spectrometer with that config
    - sends one 'meta' message
    - sends a 'config' message whenever the config changes
    - continuously acquires spectra and sends 'frame' messages
    """
    # 1) Wait for the first configuration from the GUI
    current_config = manager.wait_for_initial_config()

    with Spectrometer(config=current_config) as spectrometer:
        # 2) Acquire one spectrum to build static META info
        first = spectrometer.acquire_spectrum()

        meta = meta_from_first_spectrum(first)
        print(json.dumps(meta), flush=True)

        # 3) Send initial CONFIG message
        print(json.dumps(config_to_message(current_config)), flush=True)

        # 4) Main acquisition loop
        while not stop_event.is_set():
            # Check for updated configuration
            updated_config = manager.get_config_if_updated()
            if updated_config is not None:
                # Apply new configuration to the device
                spectrometer.configure(updated_config)
                current_config = updated_config

                # Inform the client about the new config
                print(json.dumps(config_to_message(current_config)), flush=True)

            # Acquire next spectrum
            spectrum = spectrometer.acquire_spectrum()
            frame = spectrum_to_frame(spectrum)
            print(json.dumps(frame), flush=True)


# ---------------------------------------------------------------------------
# Entry point: start acquisition thread + GUI
# ---------------------------------------------------------------------------

def main() -> None:
    """
    Entry point for the 32-bit acquisition process.

    - Creates a ConfigManager and a stop_event
    - Starts the acquisition_loop in a background thread
    - Opens the Tk configuration window in the main thread
    - When the window is closed, the stop_event is set and the
      acquisition thread is joined for a short time.
    """
    manager = ConfigManager()
    stop_event = threading.Event()

    worker = threading.Thread(
        target=acquisition_loop,
        args=(manager, stop_event),
        name="SPM002_AcquisitionThread",
        daemon=True,
    )
    worker.start()

    # Run the configuration UI in the main thread
    window = ConfigWindow(manager)
    window.run()  # blocks until the window is closed

    # When the window is closed, stop the acquisition loop
    stop_event.set()
    worker.join(timeout=2.0)


if __name__ == "__main__":
    # IMPORTANT: this module is started as:
    #   python -m acquisition.json_stream_server
    # from the 64-bit side.
    main()
