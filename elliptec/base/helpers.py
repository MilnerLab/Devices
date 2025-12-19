from typing import Iterable
from elliptec.base.enums import ReplyCommand, StatusCode
from elliptec.base.exceptions import ElliptecError


_HEX_DIGITS = "0123456789ABCDEF"


def _normalize_address(addr: str) -> str:
    if not isinstance(addr, str) or len(addr) != 1:
        raise ValueError(f"Address must be a single hex digit '0'..'F', got: {addr!r}")
    a = addr.upper()
    if a not in _HEX_DIGITS:
        raise ValueError(f"Address must be a hex digit '0'..'F', got: {addr!r}")
    return a


def _iter_addresses(min_addr: str, max_addr: str) -> Iterable[str]:
    mn = int(_normalize_address(min_addr), 16)
    mx = int(_normalize_address(max_addr), 16)
    if mn > mx:
        raise ValueError(f"min_address ({min_addr}) must be <= max_address ({max_addr})")
    for v in range(mn, mx + 1):
        yield f"{v:X}"


def _encode_u8_percent(percent: int) -> str:
    """
    Encode velocity compensation as two HEX ASCII digits, representing 0..100 (%)
    (e.g. 50 -> '32', 100 -> '64').
    """
    if not isinstance(percent, int):
        raise TypeError("percent must be int")
    if not (0 <= percent <= 100):
        raise ValueError("percent must be in [0, 100]")
    return f"{percent:02X}"


def _encode_long32(value: int) -> str:
    """Encode signed 32-bit integer as 8 HEX ASCII digits (2's complement)."""
    if not isinstance(value, int):
        raise TypeError("value must be int")
    return f"{(value & 0xFFFFFFFF):08X}"


def _parse_status_reply(reply: str, address: str) -> StatusCode:
    # Example: "0GS00"
    if len(reply) < 5 or reply[0] != address or reply[1:3] != ReplyCommand.STATUS.value:
        raise ElliptecError("Unexpected status reply", reply=reply)
    code_hex = reply[3:5]
    try:
        code = int(code_hex, 16)
    except ValueError as e:
        raise ElliptecError("Malformed status code in reply", reply=reply) from e
    try:
        return StatusCode(code)
    except ValueError:
        # Reserved / unknown codes: keep it explicit
        return StatusCode.COMMAND_ERROR_OR_NOT_SUPPORTED


def _parse_velocity_reply(reply: str, address: str) -> int:
    # Example: "AGV64" => 100%
    if len(reply) < 5 or reply[0] != address or reply[1:3] != ReplyCommand.VELOCITY.value:
        raise ElliptecError("Unexpected velocity reply", reply=reply)
    try:
        return int(reply[3:5], 16)
    except ValueError as e:
        raise ElliptecError("Malformed velocity value in reply", reply=reply) from e


def _parse_position_reply(reply: str, address: str) -> int:
    # Example: "APO00003000" => 0x3000
    if len(reply) < 11 or reply[0] != address or reply[1:3] != ReplyCommand.POSITION.value:
        raise ElliptecError("Unexpected position reply", reply=reply)
    hex32 = reply[3:11]
    try:
        raw = int(hex32, 16)
    except ValueError as e:
        raise ElliptecError("Malformed position value in reply", reply=reply) from e
    # interpret as signed 32-bit
    if raw & 0x8000_0000:
        raw -= 0x1_0000_0000
    return raw