"""
Minimal Elliptec serial helpers for setting speed via 'sv'.

Protocol reference: Elliptec Thorlabs ELLx OEM/Bare modules protocol manual (Issue 9).
- Serial params are fixed: 9600 baud, 8 data bits, 1 stop bit, no parity, no handshake.
- Device replies are CRLF terminated.
- Velocity compensation is set with:  AsvVV  (5 bytes total)
  where VV are 2 HEX ASCII digits representing percent of max velocity:
    50% -> 0x32  => 'sv32'
    70% -> 0x46  => 'sv46'
   100% -> 0x64  => 'sv64'
"""

from __future__ import annotations
import time
from typing import Optional, List

import serial  # pip install pyserial


def _open(port: str, timeout: float = 0.5) -> serial.Serial:
    return serial.Serial(
        port=port,
        baudrate=9600,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=timeout,
        write_timeout=timeout,
    )


def _send(ser: serial.Serial, cmd: str, settle_s: float = 0.02) -> Optional[str]:
    """
    Send raw Elliptec ASCII command (WITHOUT CR/LF) and read one CRLF-terminated reply.
    Returns reply without CRLF, or None on timeout.
    """
    ser.reset_input_buffer()
    ser.write(cmd.encode("ascii"))
    ser.flush()
    if settle_s:
        time.sleep(settle_s)
    raw = ser.read_until(b"\n")  # device replies end with \r\n
    if not raw:
        return None
    return raw.strip().decode("ascii", errors="replace")


def find_addresses(port: str, addresses: str = "0123456789ABCDEF", timeout: float = 0.2) -> List[str]:
    """
    Discover devices by probing each address with 'gs' (get status).
    """
    found: List[str] = []
    with _open(port, timeout=timeout) as ser:
        ser.write(b"\r")  # clear any pending receive state machine
        ser.flush()
        for a in addresses:
            r = _send(ser, f"{a}gs")
            if r and len(r) >= 5 and r[0] == a and r[1:3] == "GS":
                found.append(a)
    return found


def get_speed(port: str, address: str = "0", timeout: float = 0.5) -> int:
    """
    Read speed via 'gv' -> 'GVVV' where VV is hex (00..64). Returns percent (0..100).
    """
    address = address.upper()
    with _open(port, timeout=timeout) as ser:
        ser.write(b"\r")
        ser.flush()
        r = _send(ser, f"{address}gv")
        if not r:
            raise TimeoutError(f"No reply to '{address}gv' on {port}.")
        if len(r) < 5 or r[0] != address or r[1:3] != "GV":
            raise RuntimeError(f"Unexpected reply to gv: {r!r}")
        return int(r[3:5], 16)


def set_speed(
    port: str,
    percent: int,
    address: Optional[str] = None,
    timeout: float = 0.5,
    verify: bool = True,
) -> int:
    """
    Set speed via 'sv'. Returns the speed read back with 'gv' if verify=True.
    """
    p = int(round(percent))
    if not (0 <= p <= 100):
        raise ValueError("percent must be in [0, 100].")

    if address is None:
        addrs = find_addresses(port, timeout=min(timeout, 0.3))
        if not addrs:
            raise RuntimeError(f"No Elliptec devices responded on {port}.")
        if len(addrs) > 1:
            raise RuntimeError(f"Multiple devices found on {port}: {addrs}. Pass address=... explicitly.")
        address = addrs[0]

    address = address.upper()
    cmd = f"{address}sv{p:02X}"

    with _open(port, timeout=timeout) as ser:
        ser.write(b"\r")
        ser.flush()

        r = _send(ser, cmd)
        if not r:
            raise TimeoutError(f"No reply to '{cmd}' on {port}.")
        # On success, device replies 'AGS00' (status OK)
        if len(r) < 5 or r[0] != address or r[1:3] != "GS":
            raise RuntimeError(f"Unexpected reply to '{cmd}': {r!r}")
        status = int(r[3:5], 16)
        if status != 0:
            raise RuntimeError(f"Device returned status={status} (reply {r!r})")

        return get_speed(port, address=address, timeout=timeout) if verify else p


if __name__ == "__main__":
    # Example:
    #   python elliptec_speed_serial.py COM6 0 60
    import sys
    if len(sys.argv) >= 4:
        _port, _addr, _p = sys.argv[1], sys.argv[2], int(sys.argv[3])
        print("Setting speed...")
        print("Speed now:", set_speed(_port, _p, address=_addr), "%")
    else:
        print("Usage: python elliptec_speed_serial.py <COMPORT> <ADDR 0-F> <PERCENT 0-100>")
