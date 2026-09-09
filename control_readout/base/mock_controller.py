"""Mock controllers: one fake box per transport family, standing in for absent hardware.

``Controller`` is abstract over seven addressed primitives and ``Device`` routes every
move through them, so a single mock body drives all five motorized devices on the rig
(the ELL14 rotator, the three ESP301 stages and the RGV100BL). Only the dialect each
controller adds on top has to be faked separately, and those are a dozen lines each.

Two decisions worth stating, because they are what separates a useful mock from a
misleading one:

* **Moves block, in proportion to distance.** The whole ``MotorizedStageHandle.move_to``
  contract is that the reply is a genuine motion-complete signal. A mock that returns
  instantly makes every timing assumption in the UI untestable and lets races hide
  until the hardware is back on the bench. Set ``time_scale=0.0`` when a test wants
  the instant version.
* **Moves clamp to the profile's limits.** A mock that lets you drive through a hard
  stop teaches the operator a habit the real stage answers with a fault.
"""
from __future__ import annotations

import logging
import time
from typing import Any, Optional

from control_readout.base.controller import Controller
from control_readout.base.mock_params import MockMotionProfile

log = logging.getLogger(__name__)


class MockController(Controller):
    """A controller with no hardware behind it. ``connect()`` always succeeds.

    State is keyed by the same opaque address the real controller uses — an int axis,
    a ``(group, positioner)`` tuple, a hex bus address — so devices need no changes.
    """

    def __init__(
        self,
        profiles: Optional[dict[Any, MockMotionProfile]] = None,
        default_profile: MockMotionProfile = MockMotionProfile(),
    ) -> None:
        super().__init__()
        self._profiles = dict(profiles or {})
        self._default_profile = default_profile
        self._positions: dict[Any, float] = {}
        self._homed: dict[Any, bool] = {}
        self._initialized: dict[Any, bool] = {}
        self._velocity: dict[Any, float] = {}
        #: address -> (deg_per_s, position_at_start, monotonic_start). Set while spinning.
        self._spin: dict[Any, tuple[float, float, float]] = {}

    def profile(self, address: Any) -> MockMotionProfile:
        return self._profiles.get(address, self._default_profile)

    # -- transport: there is nothing to open, and it never fails ----------- #
    def _open(self) -> None:
        log.info("%s: connected (no hardware)", type(self).__name__)

    def _close(self) -> None:
        self._spin.clear()

    # -- addressed motion primitives --------------------------------------- #
    def initialize(self, address: Any) -> None:
        self._initialized[address] = True

    def home(self, address: Any) -> None:
        prof = self.profile(address)
        self._sleep(prof.home_time_s * prof.time_scale)
        self._spin.pop(address, None)
        self._positions[address] = prof.home_position
        self._homed[address] = True

    def get_position(self, address: Any) -> float:
        if address in self._spin:
            # Computed from the clock rather than advanced by a thread: a spinning axis
            # then reads correctly whenever it is asked, with nothing to start or join.
            velocity, start_pos, t0 = self._spin[address]
            return start_pos + velocity * (time.monotonic() - t0)
        return self._positions.get(address, 0.0)

    def move_absolute(self, address: Any, value: float, *args, **kwargs) -> None:
        prof = self.profile(address)
        target = prof.clamp(float(value))
        current = self.get_position(address)
        self._spin.pop(address, None)
        self._sleep(prof.travel_time_s(target - current))
        self._positions[address] = target

    def move_relative(self, address: Any, delta: float, *args, **kwargs) -> None:
        self.move_absolute(address, self.get_position(address) + float(delta))

    def stop(self, address: Any) -> None:
        # Freeze wherever the axis had got to, spin included.
        self._positions[address] = self.get_position(address)
        self._spin.pop(address, None)

    # -- shared dialect ----------------------------------------------------- #
    def set_velocity(self, address: Any, velocity: float, acceleration: Optional[float] = None) -> None:
        self._velocity[address] = float(velocity)

    def homed(self, address: Any) -> bool:
        return self._homed.get(address, False)

    def _sleep(self, seconds: float) -> None:
        if seconds > 0.0:
            time.sleep(seconds)


class MockESP301Controller(MockController):
    """Stands in for the ESP301 on COM4, which carries three stages at once."""

    def motor_off(self, address: int) -> None:
        self._initialized[address] = False

    def motion_done(self, address: int) -> bool:
        # Moves block until finished here, so by the time anyone can ask, they are.
        return True

    def wait_for_motion(self, address: int, poll: float = 0.05, timeout: float = 120.0) -> None:
        return None

    def check_errors(self) -> int:
        return 0

    def raise_on_error(self) -> None:
        return None

    def home(self, address: int, mode: Optional[int] = None, timeout: float = 120.0) -> None:
        super().home(address)


class MockXPSController(MockController):
    """Stands in for the XPS, which carries the RGV100BL half-wave plate."""

    def kill(self, address: Any) -> None:
        self._initialized[address] = False

    def spin(self, address: Any, velocity_deg_s: float) -> None:
        """Free-running rotation, tracked against the clock.

        Loud on purpose. The real XPS raises here unless its group is configured as a
        SpindleAxis, so a mock that spins silently is *more* capable than the hardware
        it stands for — which is the exact way a mock starts lying to the operator.
        """
        log.warning("MOCK spin at %.3f deg/s: the real XPS refuses this unless the "
                    "group is configured as a SpindleAxis", velocity_deg_s)
        self._spin[address] = (float(velocity_deg_s), self.get_position(address),
                               time.monotonic())

    def stop_spin(self, address: Any) -> None:
        self.stop(address)

    def status(self) -> str:
        return "MockXPSController: no hardware"

    def hardware_stages(self) -> dict:
        return {}

    @property
    def xps(self):
        raise NotImplementedError(
            "MockXPSController has no raw NewportXPS handle. Code reaching for "
            "controller.xps is bypassing the Controller interface, which is the one "
            "seam the mock can stand in for.")


class MockElliptecController(MockController):
    """Stands in for the Elliptec bus carrying the ELL14 rotator.

    Elliptec positions are encoder counts, not degrees, so the profile for a mocked
    ELL14 is expressed in counts to match what ``ELL14Rotator`` actually sends.
    """

    #: What resolve_address() reports when no bus is present.
    DEFAULT_ADDRESS = "0"

    def find_addresses(self) -> list[str]:
        return [self.DEFAULT_ADDRESS]

    def resolve_address(self, address: Optional[str] = None) -> str:
        return address or self.DEFAULT_ADDRESS

    def status(self, address: str):
        from control_readout.ell14.controller import StatusCode
        return StatusCode.OK

    def get_status(self, address: str):
        return self.status(address)

    def get_speed(self, address: str) -> int:
        return int(self._velocity.get(address, 50))

    def set_speed(self, address: str, percent: int) -> None:
        self._velocity[address] = int(percent)

    def get_position_counts(self, address: str) -> int:
        return int(round(self.get_position(address)))

    def home(self, address: str, direction: Any = None) -> None:
        super().home(address)
