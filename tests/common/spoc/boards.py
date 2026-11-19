"""Find the SPOC boards by USB product ID, never by a fixed ttyACM number.

    python3 -m tests.common.spoc.boards      # prints the ports, exit 1 if one is missing
"""
from __future__ import annotations

import sys

from serial.tools import list_ports

VID_SPIN = 0x2FE3
# The CUT's build sets 0x01FF. Core-MUX built from SPOC_Software (branch Core-MUX, 295a3f6)
# keeps the OwnTech default 0x0100; the build first found on the board reported 0x0102.
PID = {"mux": (0x0100, 0x0102), "cut": (0x01FF,)}
VID_RIGOL = 0x1AB1
SHELL_IFACE = ":1.2"


def find(role: str) -> str | None:
    """ThingSet shell port of the CUT or MUX Spin. Each Spin shows two CDC ACM ports:
    USB interface 0 is the Zephyr console (boot log), interface 2 the uart shell with ThingSet."""
    for p in list_ports.comports():
        if p.vid == VID_SPIN and p.pid in PID[role] and (p.location or "").endswith(SHELL_IFACE):
            return p.device
    return None


def main() -> int:
    missing = 0
    for role in PID:
        port = find(role)
        print(f"{role.upper()}: {port or 'MISSING'} (2fe3:{'/'.join(f'{x:04x}' for x in PID[role])})")
        missing += port is None
    return 1 if missing else 0


if __name__ == "__main__":
    sys.exit(main())
