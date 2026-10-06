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

## Implementation log (2026-07-21, operator session — authorised for hardware/e2e)

Test-first, updated as the work lands.

**Test:** `test_esp301_resync.py` (repo root, plain script + fake serial, no pytest;
run with `App_Apps/.venv`). Three cases, encoding the post-fix contract:
`resync_recovers_from_stale_backlog`, `no_reply_fails_fast_as_comms_fault`,
`happy_path_move_completes_and_sends_pa`.

- **[RED] unpatched driver — 1/3 passed** (as designed): stale-backlog case times out
  (no `reset_input_buffer`); silent-controller case spins the full timeout and raises
  "axis stuck" (G20); happy-path passes. This is the offline reproduction of the 03:14
  desync.
- **[GREEN] mock test — 3/3.** Fix landed in `control_readout/esp_301/controller.py`:
  (1) `_query` calls `reset_input_buffer()` before every write; (2) `motion_done`
  raises on an empty read (no longer confused with "still moving"); (3)
  `wait_for_motion` settles ~80 ms after the move command, polls at 10 Hz, and on
  repeated empty reads fails fast with the `TE?` code instead of a 120 s "stuck axis".
- **[PARTIAL] hardware e2e (2026-07-21 00:11, fix merged to `xcorr/devices`).** The new
  code path ran on real hardware and behaved correctly, but the scan did not complete:
  - Fix confirmed live: the first grating move (axis 3) now fails via the *comms-fault*
    path with the fast, accurate message above at **~27 s**, not the old silent 124 s
    "stuck axis". Resync is reading true replies (real "0"s then real silence, no stale
    backlog).
  - **Underlying axis-3 fault, independent of the framing desync:** after a fresh
    connection the controller answered `MD?`="0" (moving) for ~27 s, then went fully
    silent — MD? *and* TE? both no-reply. The resync cannot cure a controller that stops
    responding mid-move; that is a link/hardware condition for the operator to inspect
    (front panel, cabling, the UTS150CC grating axis), or a USB re-enumeration to clear a
    bridge wedge left by an *earlier* unpatched desync. Do **not** conclude "dead
    controller" (that was the original G19 misdiagnosis) — it answered healthily for 27 s.
  - Open question to resolve on hardware: whether a large grating move legitimately runs
    >27 s and the fault is a transient gap at move-completion that a slightly more
    tolerant retry window (e.g. ~2 s of silence before declaring a fault, still ≪120 s)
    would ride through. Needs an operator-supervised axis-3 probe before tuning.

### Isolation post-mortem (2026-07-21 ~02:50, laser off, no XCORR stack)

Drove COM7 directly with the fixed driver (`scratchpad/esp301_diag.py`,
`esp301_raw.py`, `esp301_unwedge.py`). Findings:

- **The link is fully wedged, not just axis 3.** Every command on every axis
  (`TP`/`MD?`/`TE?`/`VE?`) returns `''` after the full serial timeout — the controller
  sends zero bytes. Front panel + manual motion still work (operator-confirmed), so the
  CPU is alive; the **USB serial channel is wedged**, exactly the failure mode this branch
  is about. It has stayed wedged since the 00:11 dropout.
- **The resync fix is NOT the cause.** Raw pyserial with *no* `reset_input_buffer` is
  equally silent, so the wedge is independent of the fix. This also settles the open
  question above: the 00:11 failure was a full link wedge, **not** a transient
  move-completion gap — so the fast-fail threshold is correct and needs no tuning.
- **No software un-wedge worked:** DTR/RTS toggle, serial break, lone-CR flush,
  close/reopen — all still silent. PnP disable/enable to force USB re-enumeration failed
  ("Generic failure" — needs admin; session is not elevated).

**DEFERRED — operator action required (30 s) to clear the wedge:**
1. Physically unplug the ESP301 USB and replug it (or power-cycle the controller), **or**
2. From an **elevated** PowerShell:
   `Disable-PnpDevice -InstanceId 'USB\VID_104D&PID_3001\0000000000000000' -Confirm:$false`
   then `Enable-PnpDevice -InstanceId 'USB\VID_104D&PID_3001\0000000000000000' -Confirm:$false`.
   Verify with `scratchpad/esp301_raw.py` — `1TP` should return a number, not `b''`.

**The real prevention question** *(RESOLVED 2026-07-21 — see "Trigger isolated" below)*:
the 00:11 run wedged *with the fix active*, and the original 03:14 incident wedged the
old 20 Hz code. The common factor was hypothesised to be **sustained fast `MD?` polling
during a multi-second move**. The test-to-failure sweep below overturned that: poll rate
and move duration are *not* the trigger — writing without draining replies is.

### Trigger isolated + `_write` prevention (2026-07-21, test-to-failure sweep)

A controlled sweep pinned the wedge to a single cause. Each hypothesis ran against the
live bridge until it either survived or wedged (full log: `Docs/XCORR_WEDGE_TESTING_20260721.md`):

| Test | Pattern | Result |
|------|---------|--------|
| H1 | idle ~79 Hz, reading replies | survived (~7,100 polls) |
| H2 | idle ~21 Hz + per-write `reset_input_buffer`, reading | survived (~1,900) |
| H3 | **write-flood, NOT draining replies** | **WEDGED — the trigger** |
| H4 | 10 Hz through 70 s moves, reading | survived (~500/move) |
| Soak | 15 min continuous, reading | survived, 6,276 polls, 0 empties |

**Conclusion.** The wedge is caused by the host **writing without draining replies** —
nothing else. Not poll rate (H1 at 79 Hz survived), not move duration (H4), not the
resync's per-write buffer purge (H2 survived — clears the earlier "reset churn" worry),
and no cumulative drift (15 min soak clean). The 00:11 wedge is explained as leftover
old-driver damage on a bridge that was never re-enumerated, not a fresh fast-poll wedge.

**The prevention, in code.** The resync discipline is *strict one-write-one-read*, and it
had one hole: `_query` drained before every write, but `_write` (the write-only path:
`MO`/`MF`/`OR`/`PA`/`PR`/`VA`/`ST`) was fire-and-forget. A burst of write-only commands
with no interleaved query — bring-up (`MO`×3), velocity setup, a stop broadcast — was an
undrained-write burst: exactly H3. Fix: **`_write` now `reset_input_buffer()`s before the
write, symmetric with `_query`**, so every command drains the bridge one-for-one and no
undrained bytes can accumulate. Proven safe at rate by H2. An `INVARIANT` comment on the
low-level IO section records that all port access must go through `_write`/`_query` — a
naked `serial.write()` elsewhere reopens the trigger.

- **[GREEN] mock test — 4/4.** New `test_write_flood_drains_on_every_write` fires 5
  consecutive write-only commands and asserts one buffer drain per write
  (`reset_count == 5`); fails on the old fire-and-forget `_write`. The three prior resync
  cases still pass. Run with `App_Apps/.venv`.
- **[DEFERRED] hardware re-validation** — a write-burst soak (`MO`/`VA`/`ST` with no
  interleaved query) against the live controller, once the probe is free. The mock test +
  H2's proven-safe per-write reset cover the design until then.

## Merge path

`fix/esp301-serial-resync` → `xcorr/devices`. Ordinary merge; no squash concern here (this
is the Devices repo, not the `fringe_core` branch tangle). When done, remove the worktree:
`git -C C:/git/Milner_Lab/Devices worktree remove ../Devices-esp301-fix`.
