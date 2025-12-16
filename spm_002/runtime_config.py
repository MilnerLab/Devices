# acquisition/runtime_config.py
import copy
import threading
from typing import Optional

from .spm002.config import SpectrometerConfig


class ConfigManager:
    """
    Thread-safe manager for the current SpectrometerConfig.

    - set_config(cfg): called by the GUI thread whenever the user
      clicks "Apply/Start" with new values.
    - wait_for_initial_config(): blocks until a first configuration
      has been provided.
    - get_config_if_updated(): returns a new configuration if one
      has been set since the last call, otherwise None.
    """

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._current: Optional[SpectrometerConfig] = None
        self._update_event = threading.Event()

    # ------------------------------------------------------------------ #
    # Called from GUI thread
    # ------------------------------------------------------------------ #

    def set_config(self, config: SpectrometerConfig) -> None:
        """
        Store a new configuration and signal that an update is available.
        """
        with self._lock:
            # store a copy so GUI and acquisition do not share the same object
            self._current = copy.deepcopy(config)
            self._update_event.set()

    # ------------------------------------------------------------------ #
    # Called from acquisition thread
    # ------------------------------------------------------------------ #

    def wait_for_initial_config(self) -> SpectrometerConfig:
        """
        Block until a first configuration has been provided via set_config().

        Returns a copy of the configuration.
        """
        while True:
            self._update_event.wait()
            with self._lock:
                if self._current is not None:
                    cfg = copy.deepcopy(self._current)
                    # clear the event – this update has been consumed
                    self._update_event.clear()
                    return cfg

    def get_config_if_updated(self) -> Optional[SpectrometerConfig]:
        """
        If a new configuration has been set since the last call,
        return a copy of it and clear the 'updated' flag.

        Otherwise return None.
        """
        if not self._update_event.is_set():
            return None

        with self._lock:
            cfg = copy.deepcopy(self._current)
            self._update_event.clear()
            return cfg
