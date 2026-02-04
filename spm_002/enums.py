from enum import StrEnum

class MsgType(StrEnum):
    CMD = "cmd"
    META = "meta"
    FRAME = "frame"
    STATUS = "status"
    ACK = "ack"

class CmdName(StrEnum):
    SET_CONFIG = "set_config"
    GET_STATUS = "get_status"
    SHUTDOWN = "shutdown"
