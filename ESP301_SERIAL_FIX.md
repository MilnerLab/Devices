# ESP301 serial resync fix — handoff

**Branch:** `fix/esp301-serial-resync` (worktree of the `Devices` repo, off `xcorr/devices`)
**Worktree:** `C:/git/Milner_Lab/Devices-esp301-fix`
**Do not touch** `C:/git/Milner_Lab/Devices` — the live experiment imports it as an
editable install (`-e ../Devices`).

---

## TL;DR

The ESP301 is **not dead**. The "controller CPU stopped answering, needs a power cycle"
call (defect G19 in `App_Apps/Docs/XCORR_TASKS.md`) was a **misdiagnosis of a software
framing bug**. The controller has opened COM7 cleanly ~8 times since the incident,
including in a live experiment, and the front panel + manual stage motion work. This
branch fixes the real cause so it stops recurring. **No hardware access is needed** — the
fix is testable against a mock serial.

## The evidence (gathered 2026-07-20, read-only, experiment left untouched)

- `~/.milnerlab/logs/ControlReadoutProcess.log`: exactly **one** ESP301 error all day —
  the original `Timed out waiting for axis 1` at 03:14. The other two log ERRORs are IPC
  pipe resets from app shutdowns, not serial.
- The same process relaunched ~8 times (14:39, 14:44, 15:00, 16:02, 16:34, 16:35, 17:44,
  18:36) and **opened COM7 successfully every time**. pyserial opens Windows COM ports
  *exclusively*, so a clean open proves nothing is holding the port — the 03:12 wedge is
  gone.
- Caveat, stated honestly: `Controller.connect()` (`control_readout/base/controller.py:64`)
  only **opens the port**; it does not query the controller. So the logs prove the port is
  healthy, not that the CPU answered. The operator's own test closes that gap: front panel
  works and every stage moves. Dead controllers don't do that.

## Root cause — a framing/robustness defect in the ESP301 driver

File: `control_readout/esp_301/controller.py`.

1. **`wait_for_motion` polls `MD?` at 20 Hz** (`:153`, `poll=0.05`). Aggressive for the
   ESP301 ASCII parser behind the TI-3410 USB bridge; ~10 Hz is the safe ceiling.
2. **`_query` never resyncs** (`:89`). One write paired with one `readline()`, and it never
   flushes stale input. The instant one `MD?` reply is late or dropped under that fast
   poll, the *next* `readline` returns the previous command's reply — and the write/read
   stream is off-by-one for the rest of the session. `MD?` then reads stale lines that
   don't start with `"1"`, so `motion_done` is `False` forever → spins the full 120 s and
   blames axis 1.
3. **G20 makes it indistinguishable from a stuck axis**: `readline()` timeout returns `''`,
   and `''.startswith("1")` is just `False` — "no reply" and "still moving" look identical.
4. **It survived the immediate 03:12 restarts** because the blocked move held
   `Controller._lock` (G15/G16), so the process never tore down cleanly and Windows never
   released the exclusive COM handle. The next *clean* launch got a fresh port and worked.

"Every failure on axis 1" is not an axis-1 fault — axis 1 is first and most-polled, so it
hit the desync first. Axes 2/3 completing only means the stream had not desynced yet.

**Why a power cycle "fixes" it, and why that's a trap:** it forces USB re-enumeration,
dropping the stale handle and resetting the bridge — masking the bug. The
desync-under-fast-poll defect is untouched and recurs the next time a reply lands late
during a scan.

## The fix (scope: `control_readout/esp_301/controller.py`, + a test)

1. **Resync every query.** In `_query`, `self.serial.reset_input_buffer()` before the
   write so a late reply from a prior command cannot be read as this one's. (Or read-until
   a well-formed reply; buffer reset is simpler and sufficient here.)
2. **Distinguish no-reply from not-done (closes G20 / task A13).** `_query` should signal an
   empty/garbled read distinctly (e.g. return `None` or raise) rather than `''`.
   `motion_done` / `wait_for_motion` must treat that as a **communication fault**: read
   `TE?`, retry a small bounded number of times, then fail fast in seconds with
   "controller not answering" — never 120 s of "axis N stuck".
3. **Slow the poll to ~10 Hz** (`poll=0.1`) and add a small settle (~50–100 ms) after the
   `PA`/`PR`/`OR` write before the first `MD?`, so the first poll can't collide with the
   controller still parsing the move command.
4. **Clean teardown (G15/G16 — check, may be out of scope here).** An aborted or blocked
   move must release `_lock` and the port promptly. If this needs base-class changes it may
   belong in a separate change; note it, don't force it in.

### Constraints

- **No hardware.** pyserial may be absent in some venvs — the module already guards
  `import serial` (`:9-12`). Test against a **fake serial** object (a class exposing
  `write`, `readline`, `reset_input_buffer`, `close`) that can simulate a dropped/late
  reply and prove the resync recovers where the old code desyncs. This is the test that
  reproduces 03:14 without a controller.
- **No pytest in these repos** (App_Apps convention; confirm for Devices). Prefer a
  self-contained script under the repo's existing test location, run directly with the
  repo venv, exit 0/1. Check how `Devices/` already runs its tests before choosing a form.
- **Behaviour parity on the happy path.** A normal move must still complete identically;
  the resync and slower poll must not change the success path's semantics, only its
  robustness.
- Keep it **ASCII-protocol correct**: commands are `<axis><cmd>\r`; replies are read with
  `readline()`. Don't change the wire format.

### Related defects (context, in `App_Apps/Docs/XCORR_TASKS.md` §5 and `XCORR_SPEC.md` §7)

- **G19** — the "dead controller / power cycle" conclusion this branch overturns.
- **G20** — dead link misreported as a stuck axis. Item 2 above closes it.
- **A13** — the task that was already filed for the G20 fix (Devices-side). This branch
  *is* A13, plus the upstream resync cause A13 didn't name.
- **G15/G16** — the lock/teardown hazard that let the wedge survive restarts.

## Verify before merging

- New test reproduces the desync on the old code and passes on the new.
- A simulated dropped `MD?` reply now surfaces as a fast comms-fault, not a 120 s timeout.
- Happy-path move test unchanged.
- **Do not** run against real hardware while an experiment holds COM7. A live check
  (`TP` read) is only valid once the port is free, and belongs to the operator, not CI.

## Merge path

`fix/esp301-serial-resync` → `xcorr/devices`. Ordinary merge; no squash concern here (this
is the Devices repo, not the `fringe_core` branch tangle). When done, remove the worktree:
`git -C C:/git/Milner_Lab/Devices worktree remove ../Devices-esp301-fix`.
