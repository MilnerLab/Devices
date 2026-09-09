"""IPC messages for the oscilloscope worker.

Traces do not appear here. They go through the ``ScopeBuffer`` shared-memory segment,
and the pipe carries only the slot bookkeeping (``SlotGrant``, ``ItemAvailable``) that
``base_core.framework.shm`` defines for every writer worker. What is left is this
config request and one small report going the other way.
"""
from __future__ import annotations

from dataclasses import dataclass

from base_core.ipc.codec import register
from base_core.ipc.message import Message, OKReply, Request

from oscilloscope.config import ScopeConfig


@register
@dataclass(frozen=True)
class SetScopeConfig(Request[OKReply]):
    """main → worker: apply a new acquisition configuration.

    Rejected when the record it asks for does not fit a buffer slot. The segment is
    allocated before the subprocess attaches and cannot grow, so a config that overruns
    it has to fail here rather than at the first write, where the error would surface as
    a dead acquisition loop instead of a message.
    """

    config: ScopeConfig = None  # type: ignore[assignment]


@register
@dataclass(frozen=True)
class ScopeTimebase(Message):
    """worker → main: the sample interval the instrument is actually using.

    Sent on the first trace and again whenever it changes, so the cost is one tiny
    message per turn of the horizontal knob rather than one per frame. A shared-memory
    frame carries no metadata of its own, and the main process has no other way to know
    what the time axis should read.
    """

    dt_s: float = 0.0
