"""Hands a worker the controller it asked for, real or mock, and remembers failures.

One controller serves several workers: the ESP301 on COM4 carries all three linear
stages. That makes the connection a shared resource with a shared outcome, so the
decision cannot live inside any one worker.

It also cannot live in ``ControlReadoutProcess.setup()``, which is where it used to:
a raise there aborts setup and takes every worker in the subprocess with it, which is
precisely why nothing in control-readout runs on a machine with no COM4. The provider
moves the connection to first use and turns a failure into a demotion instead.
"""
from __future__ import annotations

import logging
from typing import Callable, Optional

from base_core.ipc.connection_mode import ConnectionMode
from control_readout.base.controller import Controller

log = logging.getLogger(__name__)


class ControllerProvider:
    """Lazily connects, caches per mode, and caches the failure too.

    Caching the failure is the point of the class rather than an optimisation. Without
    it the second and third stage on a dead ESP301 each pay the connection timeout
    again, so a rig with no serial port takes three timeouts to show its first panel.
    """

    def __init__(
        self,
        name: str,
        real_factory: Callable[[], Controller],
        mock_factory: Callable[[], Controller],
    ) -> None:
        self._name = name
        self._real_factory = real_factory
        self._mock_factory = mock_factory
        self._real: Optional[Controller] = None
        self._mock: Optional[Controller] = None
        self._real_error: Optional[str] = None

    def acquire(self, mode: ConnectionMode) -> Controller:
        """Return a connected controller, or raise so the caller can demote.

        Raising on the real path is deliberate: the decision to fall back belongs to
        ``DeviceWorkerMixin``, which is the thing that reports the outcome. A provider
        that quietly returned the mock here would hide the demotion from the operator.
        """
        if mode == ConnectionMode.MOCK:
            return self._acquire_mock()

        if self._real_error is not None:
            # Already known dead. Fail immediately rather than re-paying the timeout.
            raise ConnectionError(f"{self._name}: {self._real_error}")

        if self._real is None:
            try:
                controller = self._real_factory()
                controller.connect()
            except Exception as exc:
                self._real_error = f"{type(exc).__name__}: {exc}"
                log.warning("%s: connection failed (%s)", self._name, self._real_error)
                raise
            self._real = controller
        return self._real

    def _acquire_mock(self) -> Controller:
        if self._mock is None:
            self._mock = self._mock_factory()
            self._mock.connect()
        return self._mock

    @property
    def controllers(self) -> list[Controller]:
        """Every controller actually handed out. What shutdown has to disconnect."""
        return [c for c in (self._real, self._mock) if c is not None]

    def reset(self) -> None:
        """Forget the cached failure so a reconnection can be attempted again.

        Called when hardware is released, or after the last worker stops: an ESP301
        that was unplugged at start-up may well be plugged in by the time the operator
        hits Start again, and they should not have to restart the app to find out.
        """
        self._real_error = None

    def disconnect_all(self) -> None:
        for controller in self.controllers:
            try:
                controller.disconnect()
            except Exception:
                log.exception("%s: error disconnecting %r", self._name, controller)
        self._real = None
        self._mock = None
        self._real_error = None
