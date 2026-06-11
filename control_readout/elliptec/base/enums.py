from enum import Enum, IntEnum


class HostCommand(str, Enum):
    """Commands sent from host to device (lowercase in protocol)."""
    GET_STATUS = "gs"
    GET_VELOCITY = "gv"
    SET_VELOCITY = "sv"

    HOME = "ho"
    MOVE_ABSOLUTE = "ma"
    MOVE_RELATIVE = "mr"

    GET_POSITION = "gp"
    STOP = "st"  # only applicable to some devices (e.g. continuous mode)


class ReplyCommand(str, Enum):
    """Commands sent from device to host (uppercase in protocol)."""
    STATUS = "GS"
    VELOCITY = "GV"
    POSITION = "PO"


class StatusCode(IntEnum):
    OK = 0
    COMMUNICATION_TIMEOUT = 1
    MECHANICAL_TIMEOUT = 2
    COMMAND_ERROR_OR_NOT_SUPPORTED = 3
    VALUE_OUT_OF_RANGE = 4
    MODULE_ISOLATED = 5
    MODULE_OUT_OF_ISOLATION = 6
    INITIALIZING_ERROR = 7
    THERMAL_ERROR = 8
    BUSY = 9
    SENSOR_ERROR = 10
    MOTOR_ERROR = 11
    OUT_OF_RANGE = 12
    OVER_CURRENT = 13


class HomeDirection(IntEnum):
    """Only used by some rotary devices; other devices ignore this parameter."""
    CW = 0
    CCW = 1