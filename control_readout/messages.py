"""IPC messages addressed to the control-readout subprocess itself, not to a worker."""
from __future__ import annotations

from dataclasses import dataclass

from base_core.ipc.codec import register
from base_core.ipc.message import OKReply, Request


@register
@dataclass(frozen=True)
class ReleaseHardware(Request[OKReply]):
    """Disconnect every controller in the subprocess, cleanly, before it is killed.

    Process-scoped on purpose: a controller is shared by several workers (the three
    ESP301 stages sit on one serial port), so no single worker owns the decision to
    close it.

    The OK means the ports are closed, or that closing was given up on — never that
    a port was closed out from under a command in flight. The parent blocks on this
    reply because stop() on Windows is an uncatchable TerminateProcess, and an
    abrupt close mid-command is what wedges the ESP301's USB bridge.
    """
    pass
