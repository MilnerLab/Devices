# acquisition/spm002/spectrometer.py
from typing import Optional, List
import ctypes as ct

from spm_002.config import SpectrometerConfig
from spm_002.exceptions import SpectrometerError
from spm_002.dll import lib, c_int, c_ushort
from spm_002.models import SpectrumData

class Spectrometer:
    """
    High-level wrapper around the PhotonSpectr.dll for a single spectrometer.

    Responsibilities:
    - open/close the device
    - read static properties (number of pixels, LUT → wavelength axis)
    - apply a SpectrometerConfig to the device
    - acquire spectra and return SpectrumData objects

    This class does NOT:
    - handle multiple devices
    - do any GUI or plotting
    """

    def __init__(self, config: SpectrometerConfig) -> None:
        self.config: SpectrometerConfig = config

        self._is_open: bool = False
        self._num_pixels: Optional[int] = None
        self._wavelengths: Optional[List[float]] = None

    # ------------------------------------------------------------------ #
    # Properties
    # ------------------------------------------------------------------ #

    @property
    def device_index(self) -> int:
        return self.config.device_index

    @property
    def is_open(self) -> bool:
        return self._is_open

    @property
    def num_pixels(self) -> int:
        if self._num_pixels is None:
            raise SpectrometerError(
                "Number of pixels is unknown. Did you call open()?"
            )
        return self._num_pixels

    @property
    def wavelengths(self) -> Optional[List[float]]:
        """
        Wavelength axis derived from LUT, or None if LUT is not available.
        """
        return self._wavelengths

    # ------------------------------------------------------------------ #
    # Context manager support
    # ------------------------------------------------------------------ #

    def __enter__(self) -> "Spectrometer":
        if not self._is_open:
            self.open()
            self.apply_config()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        try:
            self.close()
        except SpectrometerError:
            # Don't escalate errors when leaving the context
            pass

    # ------------------------------------------------------------------ #
    # Device lifecycle
    # ------------------------------------------------------------------ #

    def open(self) -> None:
        """
        Opens the spectrometer and reads the number of pixels and LUT.

        It assumes a single device environment. If no devices are found,
        or the configured device_index is invalid, an error is raised.
        """
        if self._is_open:
            return

        num_devices = lib.PHO_EnumerateDevices()
        if num_devices <= 0:
            raise SpectrometerError("No spectrometer detected.")

        if self.device_index < 0 or self.device_index >= num_devices:
            raise SpectrometerError(
                f"Configured device_index={self.device_index} is invalid. "
                f"Number of detected devices: {num_devices}"
            )

        if lib.PHO_Open(self.device_index) == 0:
            raise SpectrometerError("PHO_Open failed.")

        # Query number of pixels
        num_pixels = c_int()
        if lib.PHO_GetPn(self.device_index, ct.byref(num_pixels)) == 0:
            lib.PHO_Close(self.device_index)
            raise SpectrometerError("PHO_GetPn failed.")

        self._num_pixels = num_pixels.value

        # Read LUT (optional)
        lut = (ct.c_float * 4)()
        if lib.PHO_GetLut(self.device_index, lut, 4) == 0:
            # LUT not available → we just work with pixel indices
            self._wavelengths = None
        else:
            npix = self._num_pixels
            self._wavelengths = [
                lut[0] + lut[1] * i + lut[2] * i * i + lut[3] * i * i * i
                for i in range(npix)
            ]

        self._is_open = True

    def close(self) -> None:
        """
        Closes the spectrometer. Safe to call multiple times.
        """
        if not self._is_open:
            return

        if lib.PHO_Close(self.device_index) == 0:
            raise SpectrometerError("PHO_Close failed.")

        self._is_open = False

    # ------------------------------------------------------------------ #
    # Configuration
    # ------------------------------------------------------------------ #

    def set_config(self, config: SpectrometerConfig) -> None:
        """
        Replace the current configuration object.
        Does NOT immediately write anything to the device.
        """
        self.config = config

    def apply_config(self) -> None:
        """
        Apply the current configuration to the device.
        """
        if not self._is_open:
            self.open()

        cfg = self.config

        # Exposure time
        if lib.PHO_SetTime(self.device_index, float(cfg.exposure_ms)) == 0:
            raise SpectrometerError("PHO_SetTime failed.")

        # Averaging
        if lib.PHO_SetAverage(self.device_index, int(cfg.average)) == 0:
            raise SpectrometerError("PHO_SetAverage failed.")

        # Dark subtraction
        if lib.PHO_SetDs(self.device_index, int(cfg.dark_subtraction)) == 0:
            raise SpectrometerError("PHO_SetDs failed.")

        # Mode (0 = continuous)
        if lib.PHO_SetMode(self.device_index, int(cfg.mode), int(cfg.scan_delay)) == 0:
            raise SpectrometerError("PHO_SetMode failed.")

    def configure(self, config: Optional[SpectrometerConfig] = None) -> None:
        """
        Convenience method:
        - optionally set a new config
        - apply configuration to the device
        """
        if config is not None:
            self.set_config(config)
        self.apply_config()

    # ------------------------------------------------------------------ #
    # Acquisition
    # ------------------------------------------------------------------ #

    def acquire_spectrum(self) -> SpectrumData:
        """
        Acquire a single spectrum and return it as a SpectrumData object.

        If the device is not open yet, it will be opened and the current
        configuration will be applied automatically.
        """
        if not self._is_open:
            self.open()
            self.apply_config()

        npix = self.num_pixels

        buffer_type = c_ushort * npix
        spectrum_buffer = buffer_type()

        if lib.PHO_Acquire(self.device_index, 0, npix, spectrum_buffer) == 0:
            raise SpectrometerError("PHO_Acquire failed.")

        counts = [spectrum_buffer[i] for i in range(npix)]

        return SpectrumData.from_raw(
            counts=counts,
            wavelengths=self._wavelengths,
            config=self.config,
        )
