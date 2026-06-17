"""
ESP301 motion worker (BaseWorker) — probe / delay / truncation on one serial port.

Follows the elliptec ``RotatorWorker`` pattern, with one addition the command-only
pattern lacks: a **position-poll thread** that emits ``PositionUpdate`` telemetry over
the IPC pipe (``_notify`` → connector, thread-safe). The shared serial port is
serialized by the driver's own lock, so the poll thread and command handlers can run
concurrently safely (D20).
"""
from __future__ import annotations

import logging
import threading
from typing import TYPE_CHECKING

from base_core.ipc.threaded_worker import ThreadedWorker, worker_thread
from control_readout.esp_301.config import Esp301Config
from control_readout.esp_301.esp_driver import EspDriver
from control_readout.esp_301.messages import (
    HomeAxis,
    MoveComplete,
    MoveRelative,
    MoveTo,
    PositionUpdate,
    SetVelocity,
    StopAxis,
)

if TYPE_CHECKING:
    from base_core.framework.events.event_bus import EventBus
    from base_core.ipc.subprocess_connector import SubprocessPipelineConnector

log = logging.getLogger(__name__)

WORKER_ID = "esp301"


class Esp301Worker(ThreadedWorker):
    def __init__(
        self,
        bus: "EventBus",
        connector: "SubprocessPipelineConnector",
        config: Esp301Config,
        port: str | None = None,
    ) -> None:
        super().__init__(WORKER_ID, bus, connector)
        self._config = config
        self._port = port or config.port
        self._driver: EspDriver | None = None
        self._poll_thread: threading.Thread | None = None
        self._poll_stop = threading.Event()
        self._moving: set[int] = set()
        self._moving_lock = threading.Lock()

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def _setup(self) -> None:
        self._unsubs.append(self._bus.subscribe(MoveTo, self._on_move_to))
        self._unsubs.append(self._bus.subscribe(MoveRelative, self._on_move_relative))
        self._unsubs.append(self._bus.subscribe(HomeAxis, self._on_home))
        self._unsubs.append(self._bus.subscribe(StopAxis, self._on_stop))
        self._unsubs.append(self._bus.subscribe(SetVelocity, self._on_set_velocity))

    def _start(self) -> None:
        self._driver = EspDriver.open(self._port, self._config.baud)
        if self._config.default_velocity is not None:
            for axis in self._config.axes:
                self._driver.set_velocity(axis, self._config.default_velocity)
        self._poll_stop.clear()
        self._poll_thread = threading.Thread(
            target=self._poll_loop, name="esp301-poll", daemon=True
        )
        self._poll_thread.start()

    def _stop(self) -> None:
        self._poll_stop.set()
        if self._poll_thread is not None:
            self._poll_thread.join(timeout=2.0)
            self._poll_thread = None
        if self._driver is not None:
            self._driver.close()
            self._driver = None
        with self._moving_lock:
            self._moving.clear()

    def _reset(self) -> None:
        self._stop()
        self._start()

    # ------------------------------------------------------------------
    # Position-poll loop (telemetry over the IPC pipe)
    # ------------------------------------------------------------------

    def _poll_loop(self) -> None:
        period = 1.0 / self._config.poll_hz if self._config.poll_hz > 0 else 0.05
        while not self._poll_stop.is_set():
            driver = self._driver
            if driver is None:
                break
            for axis in self._config.axes:
                try:
                    pos = driver.get_position(axis)
                    self._notify(PositionUpdate(axis=axis, position=pos))
                    self._check_move_complete(driver, axis, pos)
                except Exception:
                    log.exception("ESP301 poll failed on axis %s", axis)
            self._poll_stop.wait(period)

    def _check_move_complete(self, driver: EspDriver, axis: int, pos: float) -> None:
        with self._moving_lock:
            moving = axis in self._moving
        if not moving:
            return
        try:
            if driver.is_motion_done(axis):
                with self._moving_lock:
                    self._moving.discard(axis)
                self._notify(MoveComplete(axis=axis, position=pos))
        except Exception:
            log.exception("ESP301 motion-done check failed on axis %s", axis)

    def _mark_moving(self, axis: int) -> None:
        with self._moving_lock:
            self._moving.add(axis)

    # ------------------------------------------------------------------
    # Command handlers
    # ------------------------------------------------------------------

    @worker_thread
    def _on_move_to(self, msg: MoveTo) -> None:
        self._do(msg, lambda d: (d.move_to(msg.axis, msg.position), self._mark_moving(msg.axis)))

    @worker_thread
    def _on_move_relative(self, msg: MoveRelative) -> None:
        self._do(msg, lambda d: (d.move_relative(msg.axis, msg.delta), self._mark_moving(msg.axis)))

    @worker_thread
    def _on_home(self, msg: HomeAxis) -> None:
        self._do(msg, lambda d: (d.home(msg.axis), self._mark_moving(msg.axis)))

    @worker_thread
    def _on_stop(self, msg: StopAxis) -> None:
        self._do(msg, lambda d: d.stop(msg.axis))

    @worker_thread
    def _on_set_velocity(self, msg: SetVelocity) -> None:
        self._do(msg, lambda d: d.set_velocity(msg.axis, msg.velocity))

    def _do(self, msg, action) -> None:
        """Run a driver action, replying OK/error. Guards the not-started case."""
        driver = self._driver
        if driver is None:
            self._reply_error(msg, "ESP301 not started")
            return
        try:
            action(driver)
            self._reply_ok(msg)
        except Exception as exc:
            log.exception("ESP301 command failed: %s", type(msg).__name__)
            self._reply_error(msg, str(exc))
