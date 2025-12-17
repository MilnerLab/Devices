import time
import math
import clr
from System import Decimal
from base_lib.models import Angle, AngleUnit, Range

# === Konstanten ===
ANGLE_RANGE = Range(Angle(-90, AngleUnit.DEG), Angle(90, AngleUnit.DEG))
OUT_OF_RANGE_RELATIVE_ANGLE = Angle(90, AngleUnit.DEG)
HOME_ANGLE = Angle(0, AngleUnit.DEG)

# === DLL laden ===
clr.AddReference(r"C:\Program Files\Thorlabs\Elliptec\Thorlabs.Elliptec.ELLO_DLL.dll")
from Thorlabs.Elliptec.ELLO_DLL import ELLDevicePort, ELLDevices, ELLBaseDevice


class ElliptecRotator:
    def __init__(
        self,
        port: str = "COM6",
        min_address: str = "0",
        max_address: str = "F",
    ) -> None:
        self._device = None
        self._ell_devices = None
        self._address = None  # device address (e.g. '0'..'F')

        self._current_angle: Angle = Angle(0, AngleUnit.DEG)

        self._initialize(port, min_address, max_address)


    def rotate(self, angle: Angle) -> None:
        
        if float(angle) == 0.0:
            return

        new_angle = Angle(self._current_angle + angle)
        self._validate_new_delta_angle(new_angle)
        self._move_relative(angle)
        print("Current wp angle:", self._current_angle.Deg)
        print("--------------------------")

    def home(self) -> None:
        self._device.Home(ELLBaseDevice.DeviceDirection.Linear)
        time.sleep(1.0)
        self._current_angle = Angle(0, AngleUnit.DEG)

    def set_speed(self, percent: float, *, clamp: bool = True) -> None:
        """
        Set Elliptec *velocity compensation* via the ASCII 'sv' command.

        Notes:
        - The protocol expects the value as a percentage encoded as 2 hex digits.
        - For ELL14 continuous motion (fw/bw with jog step size 0), Thorlabs recommends 50–70%.
        """
        if self._device is None:
            raise RuntimeError("Device not initialized.")

        try:
            p = float(percent)
        except Exception as e:
            raise ValueError(f"percent must be a number, got {percent!r}") from e

        if math.isnan(p) or math.isinf(p):
            raise ValueError(f"percent must be finite, got {percent!r}")

        if clamp:
            p = max(0.0, min(100.0, p))
        elif not (0.0 <= p <= 100.0):
            raise ValueError("percent must be in [0, 100]")

        p_i = int(round(p))
        data = f"{p_i:02X}"

        # --- 1) Try high-level DLL API if present ------------------------
        for name in ("SetVelocityCompensation", "SetVelocity", "SetVelocityPercentage"):
            if hasattr(self._device, name):
                fn = getattr(self._device, name)
                try:
                    fn(p_i)
                    return
                except Exception:
                    try:
                        fn(Decimal(p_i))
                        return
                    except Exception:
                        pass

        for prop in ("VelocityCompensation", "Velocity", "VelocityPercentage"):
            if hasattr(self._device, prop):
                try:
                    setattr(self._device, prop, p_i)
                    return
                except Exception:
                    try:
                        setattr(self._device, prop, Decimal(p_i))
                        return
                    except Exception:
                        pass

        # --- 2) Fallback: send raw ASCII command 'AsvNN' ------------------
        addr = self._address
        if addr is None:
            try:
                addr = str(self._device.Address)
            except Exception:
                addr = "0"

        cmd = f"{addr}sv{data}"

        # Try common ELLDevicePort send/write methods (signatures differ by DLL version)
        for meth in ("SendCommand", "SendMessage", "SendMsg", "Write"):
            if not hasattr(ELLDevicePort, meth):
                continue
            m = getattr(ELLDevicePort, meth)
            # 2a) string
            try:
                m(cmd)
                return
            except Exception:
                pass
            # 2b) byte[]
            try:
                from System import Array, Byte
                from System.Text import Encoding
                payload = Encoding.ASCII.GetBytes(cmd)
                barr = Array[Byte](payload)
                try:
                    m(barr)
                    return
                except Exception:
                    try:
                        m(barr, 0, len(payload))
                        return
                    except Exception:
                        pass
            except Exception:
                pass

        raise RuntimeError(
            "Could not set speed: no suitable method found in the ELLO_DLL to send 'sv'. "
            "(Try updating the Elliptec software / DLL, or switch to a pyserial-based driver.)"
        )

    def close(self) -> None:
        try:
            ELLDevicePort.Disconnect()
        except Exception:
            pass

    # ------------------------------------------------------------------    
    # internal helpers
    # ------------------------------------------------------------------    

    def _initialize(self, port, min_address, max_address) -> None:
        print(f"Connecting to Elliptec device on {port} ...")
        ELLDevicePort.Connect(port)

        ell_devices = ELLDevices()
        devices = ell_devices.ScanAddresses(min_address, max_address)
        if not devices:
            raise RuntimeError("No Elliptec devices found on bus.")

        addressed_device = None
        for dev in devices:
            if ell_devices.Configure(dev):
                addressed_device = ell_devices.AddressedDevice(dev[0])
                self._address = str(dev[0])
                break

        if addressed_device is None:
            raise RuntimeError("No configurable Elliptec device found.")

        self._ell_devices = ell_devices
        self._device = addressed_device

        device_info = self._device.DeviceInfo
        print("Connected to Elliptec device:")
        for line in device_info.Description():
            print("  ", line)

        print("Homing device...")
        self._device.Home(ELLBaseDevice.DeviceDirection.Linear)
        time.sleep(1.0)
        print("Device homed.")

        # Startzustand: delta = 0
        self._current_angle = Angle(0, AngleUnit.DEG)

    def _move_relative(self, angle: Angle) -> None:
        d = Decimal(angle.Deg)
        self._device.MoveRelative(d)
        self._current_angle = Angle(self._current_angle + angle)
        time.sleep(2.0)

    def _validate_new_delta_angle(self, new_angle: Angle) -> None:
        
        if ANGLE_RANGE.is_in_range(new_angle):
            return

        if new_angle > ANGLE_RANGE.max:
            correction = Angle(-OUT_OF_RANGE_RELATIVE_ANGLE)
            print("corrected max")
        else:
            correction = OUT_OF_RANGE_RELATIVE_ANGLE
            print("corrected min")

        self._move_relative(correction)

        
