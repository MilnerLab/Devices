from typing import Optional

from control_readout.elliptec.base.enums import StatusCode



class ElliptecError(RuntimeError):
    def __init__(self, message: str, *, status: Optional[StatusCode] = None, reply: Optional[str] = None):
        super().__init__(message)
        self.status = status
        self.reply = reply
