import time
import clr
from decimal import Decimal

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

        
