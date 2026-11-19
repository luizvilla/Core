"""Find the SPOC boards: each Spin's ThingSet shell port, by what its ThingSet tree holds.

USB product IDs do not tell the boards apart (Core-MUX from the branch and the safety test firmware both report
2fe3:0100), so each Spin shell port is asked for its ThingSet root: 'Mux' -> MUX; 'Cut' (Core-CUT) or 'Safety'
(tests/firmware/safety_l0) -> CUT. Each Spin shows two CDC ACM ports: interface 0 is the Zephyr console, interface 2
the uart shell with ThingSet.

    python3 -m spoc.boards      # from tests/common: prints the ports, exit 1 if one is missing
"""
from __future__ import annotations

import json
import sys
import time

import serial
from serial.tools import list_ports

VID_SPIN = 0x2FE3
SHELL_IFACE = ":1.2"
ROLES = {"mux": ("Mux",), "cut": ("Cut", "Safety")}
_cache: dict[str, str] = {}


def _root_keys(port: str) -> set[str]:
    """ThingSet root keys of the shell on `port` (works whether the shell is in 'select thingset' mode or not)."""
    try:
        with serial.Serial(port, 115200, timeout=0.3, write_timeout=1.0) as ser:  # a hung Spin blocks writes
            ser.reset_input_buffer()
            for cmd in (b"\r\n?\r\n", b"thingset ?\r\n"):
                ser.write(cmd)
                time.sleep(0.6)
                raw = ser.read(4096).decode(errors="replace")
                start, end = raw.find(":85 {"), raw.rfind("}")
                if start >= 0 and end > start:
                    try:
                        return set(json.loads(raw[start + 4:end + 1]))
                    except ValueError:
                        pass
    except (serial.SerialException, OSError):
        pass
    return set()


def shell_ports() -> list[str]:
    return sorted(p.device for p in list_ports.comports()
                  if p.vid == VID_SPIN and (p.location or "").endswith(SHELL_IFACE))


def find(role: str, tries: int = 3) -> str | None:
    """ThingSet shell port of the CUT or MUX Spin, or None (retried: a Spin answers ~2 s after it boots)."""
    port = _cache.get(role)
    if port and port in shell_ports():
        return port
    for attempt in range(tries):
        for port in shell_ports():
            if set(ROLES[role]) & _root_keys(port):
                _cache[role] = port
                return port
        time.sleep(1.0)
    return None


def main() -> int:
    missing = 0
    for role in ROLES:
        port = find(role)
        print(f"{role.upper()}: {port or 'MISSING'} (ThingSet root {' or '.join(ROLES[role])})")
        missing += port is None
    return 1 if missing else 0


if __name__ == "__main__":
    sys.exit(main())
