"""ESP301 serial-resync fix — offline reproduction of the G19/G20 desync.

Test-first for the fix described in ``ESP301_SERIAL_FIX.md``. No hardware, no
pytest (matching the repo convention — ``test.py`` is a plain script). Run it with
the App_Apps venv, which is where pyserial and the rest live::

    App_Apps\\.venv\\Scripts\\python.exe Devices-esp301-fix\\test_esp301_resync.py

Exit 0 = every case passed; a failure prints a traceback and exits 1.

The three cases encode the *post-fix* contract, so on the unpatched driver:

* ``test_resync_recovers_from_stale_backlog`` FAILS — no ``reset_input_buffer``,
  so the stale backlog is read one line per poll and the move times out.
* ``test_no_reply_fails_fast_as_comms_fault`` FAILS — a silent controller is read
  as ``''`` → ``startswith("1")`` is ``False`` → it spins the full timeout and
  blames the axis, instead of failing fast as a comms fault (defect G20).
* ``test_happy_path_move_completes_and_sends_pa`` PASSES on both — the regression
  guard: a normal move must still send ``PA`` and complete.
"""
from __future__ import annotations

import sys
import time
import traceback
from collections import deque
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from control_readout.esp_301.controller import ESP301Controller, ESP301Error  # noqa: E402


class FakeESP301Serial:
    """A minimal stand-in for ``serial.Serial`` — line-oriented, deterministic.

    Models the input buffer as a FIFO of reply lines. ``write`` appends the reply
    the controller would send (commands like ``PA``/``MO`` get none); ``readline``
    pops one line, or returns ``b""`` when the buffer is empty (a read timeout).
    ``reset_input_buffer`` — the call the fix adds — clears the buffer, which is
    what lets the driver resynchronise after a stale/late reply.
    """

    def __init__(self, *, preload=(), md_replies=("1",), no_reply=False, te="0"):
        self.timeout = 5.0
        self._buf: deque[bytes] = deque(preload)
        self._md = list(md_replies)   # successive MD? replies; the last one repeats
        self._no_reply = no_reply     # controller stays silent to MD? (dropped link)
        self._te = te
        self.reset_count = 0
        self.written: list[bytes] = []

    def write(self, data: bytes) -> int:
        self.written.append(data)
        cmd = data.decode("ascii").strip()
        if cmd.endswith("MD?"):
            if self._no_reply:
                return len(data)
            reply = self._md.pop(0) if len(self._md) > 1 else self._md[0]
            self._buf.append((reply + "\r\n").encode("ascii"))
        elif cmd.endswith("TE?"):
            self._buf.append((self._te + "\r\n").encode("ascii"))
        elif cmd.endswith("TP"):
            self._buf.append(b"0.0\r\n")
        # PA / PR / MO / MF / VA / ST / OR: no reply, like the real controller.
        return len(data)

    def readline(self) -> bytes:
        if self._buf:
            return self._buf.popleft()
        return b""  # empty read == timeout with no data

    def reset_input_buffer(self) -> None:
        self.reset_count += 1
        self._buf.clear()

    def close(self) -> None:
        pass


def _wired(fake: FakeESP301Serial) -> ESP301Controller:
    ctrl = ESP301Controller("COM_TEST")
    ctrl._serial = fake          # inject the fake transport
    ctrl._connected = True
    return ctrl


# --- cases ----------------------------------------------------------------

def test_resync_recovers_from_stale_backlog():
    """A backlog of stale ``0`` replies must not stall a done axis.

    100 leftover lines sit in the buffer (accumulated late replies). The axis is
    already done — MD? answers ``1``. The fix flushes the backlog before each read
    and sees the true ``1`` on the first poll; the unpatched driver reads the stale
    lines one per poll and times out.
    """
    fake = FakeESP301Serial(preload=[b"0\r\n"] * 100, md_replies=["1"])
    ctrl = _wired(fake)
    ctrl.wait_for_motion(3, timeout=1.0)          # must NOT raise
    assert fake.reset_count > 0, "the fix must reset_input_buffer() to resync"


def test_no_reply_fails_fast_as_comms_fault():
    """A silent controller must surface as a fast comms fault, not a 120 s stall."""
    fake = FakeESP301Serial(no_reply=True, te="0")
    ctrl = _wired(fake)
    t0 = time.monotonic()
    try:
        ctrl.wait_for_motion(3, timeout=3.0)
    except ESP301Error as e:
        elapsed = time.monotonic() - t0
        assert elapsed < 2.0, f"should fail fast, took {elapsed:.2f}s"
        msg = str(e).lower()
        assert "answer" in msg or "respond" in msg, f"want a comms-fault message, got: {e}"
        return
    raise AssertionError("expected an ESP301Error comms fault, nothing was raised")


def test_happy_path_move_completes_and_sends_pa():
    """Regression guard: a normal move still sends PA and completes cleanly."""
    fake = FakeESP301Serial(md_replies=["0", "0", "1"])
    ctrl = _wired(fake)
    ctrl.move_absolute(3, 30.1)
    assert any(b"3PA30.1000" in w for w in fake.written), fake.written


# --- runner ---------------------------------------------------------------

def main() -> int:
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    failed = 0
    for t in tests:
        try:
            t()
        except Exception:
            failed += 1
            print(f"FAIL  {t.__name__}")
            traceback.print_exc()
        else:
            print(f"ok    {t.__name__}")
    print(f"\n{len(tests) - failed}/{len(tests)} passed")
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
