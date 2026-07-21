# Prompt — paste into a fresh Claude Code session started in this worktree

Start the session with the working directory set to
`C:/git/Milner_Lab/Devices-esp301-fix` (the `fix/esp301-serial-resync` worktree of the
`Devices` repo). Then give Claude:

---

Read `ESP301_SERIAL_FIX.md` in this directory first — it is the full handoff and diagnosis.

Implement the ESP301 serial-driver fix it describes, in
`control_readout/esp_301/controller.py`:

1. Make `_query` resync before every command (`reset_input_buffer()` before the write) so
   a late/dropped reply from a prior command can't be misread as this one's.
2. Make an empty/garbled read distinguishable from a real reply, and have
   `motion_done`/`wait_for_motion` treat it as a communication fault: read `TE?`, retry a
   small bounded number of times, then fail fast (seconds) with "controller not answering"
   — never 120 s of "axis N stuck". This closes defect G20 / task A13.
3. Slow the `MD?` poll to ~10 Hz and add a ~50–100 ms settle after the move command before
   the first poll.

Write a self-contained test (no hardware, no pytest — match how `Devices/` already runs
tests; check first) using a fake serial object that simulates a dropped/late `MD?` reply.
It must **fail on the current code** (reproducing the 03:14 desync → 120 s timeout) and
**pass on the fix**, and a happy-path move must still complete identically.

Hard constraints:
- **Do NOT touch `C:/git/Milner_Lab/Devices`** or open COM7 — a live experiment holds the
  port exclusively. Everything here is offline against a mock.
- Don't change the wire protocol (`<axis><cmd>\r`, `readline()` replies).
- The clean-teardown / `_lock` hazard (G15/G16) may need base-class changes — if so, note
  it and keep it separate rather than forcing it into this change.

When green, update `ESP301_SERIAL_FIX.md`'s "Verify" section with what you ran, commit on
`fix/esp301-serial-resync`, and stop before merging or going near hardware — the operator
does the live `TP` check once the port is free.

---

## Context the session won't otherwise have

- The operator's ESP301 is **fine** — front panel and manual motion both work. This branch
  fixes a *software* misdiagnosis, not hardware. Do not suggest a power cycle.
- Related defects live in `C:/git/Milner_Lab/App_Apps/Docs/XCORR_TASKS.md` §5 and
  `XCORR_SPEC.md` §7 (G19, G20, A13, G15, G16). Those repos are separate checkouts; read
  but don't edit them from here.
- No pytest anywhere in these repos; tests are plain scripts run with the repo venv, exit
  0/1. Confirm the Devices convention before writing the test.
