# acquisition/spm002/dll.py
import ctypes as ct
import os

from .exceptions import SpectrometerError


# Expose a few ctypes aliases so other modules can reuse them
c_int = ct.c_int
c_float = ct.c_float
c_ushort = ct.c_ushort
POINTER = ct.POINTER


def _find_dll_path() -> str:
    """
    Try to locate PhotonSpectr.dll.

    Priority:
    1. Environment variable PHOTON_SPECTR_DLL_PATH (if set)
    2. Same directory as this file (the spm002 package)
    3. Parent directory (project root)
    """
    env_path = os.environ.get("PHOTON_SPECTR_DLL_PATH")
    if env_path and os.path.isfile(env_path):
        return env_path

    package_dir = os.path.dirname(os.path.abspath(__file__))
    candidate_in_package = os.path.join(package_dir, "PhotonSpectr.dll")
    if os.path.isfile(candidate_in_package):
        return candidate_in_package

    project_root = os.path.dirname(os.path.dirname(package_dir))
    # project_root now points to repo root (…/spm-project)
    candidate_in_root = os.path.join(project_root, "PhotonSpectr.dll")
    if os.path.isfile(candidate_in_root):
        return candidate_in_root

    raise SpectrometerError(
        "PhotonSpectr.dll not found. "
        "Place it either next to the 'spm002' package or in the project root, "
        "or set PHOTON_SPECTR_DLL_PATH."
    )


def _load_photon_spectr() -> ct.WinDLL:
    """Load PhotonSpectr.dll (32-bit only)."""
    # Ensure we are running a 32-bit Python
    if ct.sizeof(ct.c_void_p) != 4:
        raise SpectrometerError(
            "PhotonSpectr.dll is 32-bit. "
            "Please use a 32-bit Python interpreter (x86)."
        )

    dll_path = _find_dll_path()
    return ct.WinDLL(dll_path)


# Global handle to the DLL
lib = _load_photon_spectr()


# ---------------------------------------------------------------------------
# Function prototypes (only the ones we need)
# ---------------------------------------------------------------------------

# int PHO_EnumerateDevices(void);
lib.PHO_EnumerateDevices.argtypes = []
lib.PHO_EnumerateDevices.restype = c_int

# int PHO_Open(int dev);
lib.PHO_Open.argtypes = [c_int]
lib.PHO_Open.restype = c_int

# int PHO_Close(int dev);
lib.PHO_Close.argtypes = [c_int]
lib.PHO_Close.restype = c_int

# int PHO_GetPn(int dev, int* pn);
lib.PHO_GetPn.argtypes = [c_int, POINTER(c_int)]
lib.PHO_GetPn.restype = c_int

# int PHO_GetLut(int dev, float* lut, int size);
lib.PHO_GetLut.argtypes = [c_int, POINTER(c_float), c_int]
lib.PHO_GetLut.restype = c_int

# int PHO_SetTime(int dev, float exposure_ms);
lib.PHO_SetTime.argtypes = [c_int, c_float]
lib.PHO_SetTime.restype = c_int

# int PHO_GetTime(int dev, float* exposure_ms);
lib.PHO_GetTime.argtypes = [c_int, POINTER(c_float)]
lib.PHO_GetTime.restype = c_int

# int PHO_SetAverage(int dev, int average);
lib.PHO_SetAverage.argtypes = [c_int, c_int]
lib.PHO_SetAverage.restype = c_int

# int PHO_SetDs(int dev, int dark_subtraction);
lib.PHO_SetDs.argtypes = [c_int, c_int]
lib.PHO_SetDs.restype = c_int

# int PHO_SetMode(int dev, int mode, int scan_delay);
lib.PHO_SetMode.argtypes = [c_int, c_int, c_int]
lib.PHO_SetMode.restype = c_int

# int PHO_Acquire(int dev, int start_pixel, int num_pixels, unsigned short* buffer);
lib.PHO_Acquire.argtypes = [c_int, c_int, c_int, POINTER(c_ushort)]
lib.PHO_Acquire.restype = c_int
