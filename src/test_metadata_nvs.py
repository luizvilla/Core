#!/usr/bin/env python3
"""
Automated write -> reset -> read regression test for the spin.metaData
NVS persistence feature.

Drives the serial command menu exposed by the test harness in
src/main.cpp (h/w/r/c/f) to verify that board/shield metadata survives:

  1. A same-boot round trip (sanity check).
  2. A board reset, triggered via the classic 1200bps-touch convention
     followed by an MCU manager reset from the bootloader back into the
     application (the same flow used by this project's upload scripts).
  3. Optionally (--reflash), a full firmware reflash via
     `platformio run -t upload`, proving the storage_partition survives
     an application image update.

Requires the board to be flashed with the src/main.cpp test harness, and
pyserial installed (`pip install pyserial`).

Usage:
    python3 src/test_metadata_nvs.py
        [--port PORT | --board-id BOARD_ID | --usb-vid VID]
        [--reflash]
"""

import argparse
from pathlib import Path
import re
import subprocess
import sys
import time

import serial
import serial.tools.list_ports


def find_mcumgr_path():
    """Locate OwnTech's MCU manager regardless of this script's directory."""
    script_directory = Path(__file__).resolve().parent

    for directory in (script_directory, *script_directory.parents):
        candidates = (
            directory / "owntech" / "third_party" / "mcumgr",
            directory / "third_party" / "mcumgr",
        )
        for candidate in candidates:
            if candidate.is_file():
                return candidate

    return script_directory.parent / "owntech" / "third_party" / "mcumgr"


SPIN_USB_VID = "2FE3"
DEFAULT_BOARD_ID = "423250070031003C"
BAUD_RATE = 115200
COMMAND_TIMEOUT_S = 5
RESET_WAIT_S = 10
BOOT_SETTLE_S = 1.5
MCUMGR_PATH = find_mcumgr_path()

EXPECTED_VALUES = {
    "SPIN_SERIAL":      "SPIN000000001",
    "SHIELD_SERIAL":    "SHLD000000001",
    "SPIN_VERSION":     "9.9.9",
    "SHIELD_VERSION":   "8.8.8",
    "SHIELD_PASSWORD":  "abc",
    "EXTRA_0":          "EXTRA0",
    "EXTRA_1":          "EXTRA1",
    "EXTRA_2":          "EXTRA2",
    "EXTRA_3":          "EXTRA3",
    "EXTRA_4":          "EXTRA4",
}


def parse_usb_vid(value):
    """Parse a USB VID written as hexadecimal, with or without a 0x prefix."""
    try:
        usb_vid = int(str(value), 16)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(
            f"invalid USB VID {value!r}; expected a hexadecimal value"
        ) from exc

    if not 0 <= usb_vid <= 0xFFFF:
        raise argparse.ArgumentTypeError(
            f"invalid USB VID {value!r}; expected 0000 through FFFF"
        )

    return usb_vid


def devices_match(first, second):
    """Compare direct device paths and resolved Linux device symlinks."""
    if first == second:
        return True

    return Path(first).resolve() == Path(second).resolve()


def find_spin_port(port=None, board_id=None, usb_vid=None):
    """Find a Spin serial port by path, board ID, or USB vendor ID."""
    selectors = (port is not None, board_id is not None, usb_vid is not None)
    if sum(selectors) != 1:
        raise ValueError("provide exactly one of port, board_id, or usb_vid")

    available_ports = list(serial.tools.list_ports.comports())

    if port is not None:
        ports = [
            candidate
            for candidate in available_ports
            if devices_match(candidate.device, port)
        ]
        selector = f"port {port}"
    elif board_id is not None:
        ports = [
            candidate
            for candidate in available_ports
            if candidate.serial_number == board_id
        ]
        selector = f"board ID {board_id}"
    else:
        if isinstance(usb_vid, str):
            usb_vid = parse_usb_vid(usb_vid)
        ports = [
            candidate
            for candidate in available_ports
            if candidate.vid == usb_vid
        ]
        selector = f"USB VID {usb_vid:04X}"

    if not ports:
        return None
    if len(ports) > 1:
        print(
            f"Multiple ports found for {selector}, "
            f"using the first one: {ports[0].device}"
        )
    return ports[0].device


def board_id_for_port(port):
    """Return the USB serial number associated with a device path."""
    for candidate in serial.tools.list_ports.comports():
        if devices_match(candidate.device, port):
            return candidate.serial_number

    return None


def read_lines_until(ser, end_marker, timeout=COMMAND_TIMEOUT_S):
    lines = []
    deadline = time.time() + timeout
    while time.time() < deadline:
        raw = ser.readline()
        if not raw:
            continue
        line = raw.decode("ascii", errors="replace").strip()
        if not line:
            continue
        lines.append(line)
        if line == end_marker:
            break
    return lines


def read_one_line(ser, timeout=COMMAND_TIMEOUT_S):
    deadline = time.time() + timeout
    while time.time() < deadline:
        raw = ser.readline()
        if not raw:
            continue
        line = raw.decode("ascii", errors="replace").strip()
        if line:
            return line
    return ""


def send_multiline_command(ser, command, end_marker):
    ser.reset_input_buffer()
    ser.write(command.encode("ascii"))
    return read_lines_until(ser, end_marker)


def send_singleline_command(ser, command):
    ser.reset_input_buffer()
    ser.write(command.encode("ascii"))
    return read_one_line(ser)


def parse_kv_lines(lines):
    result = {}
    for line in lines:
        match = re.match(r"^([A-Z_0-9]+)=(.*)$", line)
        if match:
            result[match.group(1)] = match.group(2)
    return result


def check_all_present(fields):
    rows = [
        (key, repr(expected_value), repr(fields.get(key)))
        for key, expected_value in EXPECTED_VALUES.items()
    ]
    field_width = max(len("FIELD"), *(len(key) for key, _, _ in rows))
    written_width = max(
        len("WRITTEN"), *(len(written) for _, written, _ in rows)
    )
    read_width = max(len("READ"), *(len(read) for _, _, read in rows))

    print(
        f"  {'FIELD':<{field_width}}  "
        f"{'WRITTEN':<{written_width}}  "
        f"{'READ':<{read_width}}  RESULT"
    )

    ok = True
    for key, written, read in rows:
        matches = fields.get(key) == EXPECTED_VALUES[key]
        print(
            f"  {key:<{field_width}}  "
            f"{written:<{written_width}}  "
            f"{read:<{read_width}}  "
            f"{'OK' if matches else 'MISMATCH'}"
        )
        ok = ok and matches

    return ok


def check_all_cleared(fields):
    ok = True
    for key in EXPECTED_VALUES:
        actual = fields.get(key, "")
        if not actual.startswith("ERR"):
            print(f"  UNEXPECTED VALUE {key}: expected ERR after clear, got {actual!r}")
            ok = False
    return ok


EXPECTED_BADSIZE_RESULTS = {
    "SPIN_SERIAL_BADSIZE":     "-2",
    "SHIELD_SERIAL_BADSIZE":   "-2",
    "SHIELD_PASSWORD_BADSIZE": "-2",
}


def check_all_rejected(results):
    rows = [
        (key, expected, results.get(key))
        for key, expected in EXPECTED_BADSIZE_RESULTS.items()
    ]
    field_width = max(len("FIELD"), *(len(key) for key, _, _ in rows))
    expected_width = max(
        len("EXPECTED"), *(len(expected) for _, expected, _ in rows)
    )
    actual_width = max(
        len("ACTUAL"), *(len(actual or "") for _, _, actual in rows)
    )

    print(
        f"  {'FIELD':<{field_width}}  "
        f"{'EXPECTED':<{expected_width}}  "
        f"{'ACTUAL':<{actual_width}}  RESULT"
    )

    ok = True
    for key, expected, actual in rows:
        matches = actual == expected
        print(
            f"  {key:<{field_width}}  "
            f"{expected:<{expected_width}}  "
            f"{(actual or ''):<{actual_width}}  "
            f"{'OK' if matches else 'REJECTED-MISMATCH'}"
        )
        ok = ok and matches

    return ok


def touch_reset(port):
    """Enter MCUboot using the 1200-baud-touch convention."""
    reset_ser = serial.Serial(port, 1200, timeout=1)
    reset_ser.close()


def reset_from_bootloader(port):
    """Ask MCUboot to reset into the application."""
    if not MCUMGR_PATH.is_file():
        raise FileNotFoundError(
            f"MCU manager executable not found at {MCUMGR_PATH}"
        )

    subprocess.run(
        [
            str(MCUMGR_PATH),
            "--conntype", "serial",
            "--connstring", f"dev={port},baud={BAUD_RATE},mtu=128",
            "reset",
        ],
        check=True,
    )


def wait_for_port_disappearance(device, timeout=RESET_WAIT_S):
    """Wait until the old USB serial device disappears."""
    deadline = time.time() + timeout

    while time.time() < deadline:
        devices = {
            port.device
            for port in serial.tools.list_ports.comports()
        }

        if device not in devices:
            return True

        time.sleep(0.1)

    return False


def wait_for_spin_port(board_id, timeout=RESET_WAIT_S):
    """Wait until a Spin USB serial port is available."""
    deadline = time.time() + timeout

    while time.time() < deadline:
        port = find_spin_port(board_id=board_id)
        if port is not None:
            return port

        time.sleep(0.25)

    return None


def open_serial_with_retry(port, timeout=RESET_WAIT_S):
    """Open a newly enumerated serial port after udev makes it accessible."""
    deadline = time.time() + timeout
    last_error = None

    while time.time() < deadline:
        try:
            return serial.Serial(port, BAUD_RATE, timeout=1)
        except (OSError, serial.SerialException) as exc:
            last_error = exc
            time.sleep(0.1)

    raise RuntimeError(
        f"Serial port {port} did not become accessible"
    ) from last_error


def reconnect_after_reset(previous_port, board_id):
    disappeared = wait_for_port_disappearance(previous_port)

    if not disappeared:
        print(
            f"Warning: {previous_port} did not visibly disappear; "
            "waiting before reconnecting..."
        )
        time.sleep(2)

    new_port = wait_for_spin_port(board_id)

    if new_port is None:
        input(
            "Board did not reappear automatically. Please power-cycle "
            "it manually, then press Enter to continue..."
        )
        new_port = find_spin_port(board_id=board_id)

        if new_port is None:
            raise RuntimeError("Spin board is still not available")

    ser = open_serial_with_retry(new_port)
    time.sleep(BOOT_SETTLE_S)

    return ser, new_port


def reset_and_reconnect(previous_port, board_id):
    touch_reset(previous_port)

    # The first USB cycle enters MCUboot.
    bootloader_ser, bootloader_port = reconnect_after_reset(
        previous_port, board_id
    )
    bootloader_ser.close()
    reset_from_bootloader(bootloader_port)

    # The second USB cycle returns to the application.
    return reconnect_after_reset(bootloader_port, board_id)


def reflash(env_name="USB"):
    print("Reflashing firmware via 'platformio run -t upload'...")
    subprocess.run(["platformio", "run", "-e", env_name, "-t", "upload"], check=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    selector_group = parser.add_mutually_exclusive_group()
    selector_group.add_argument(
        "--port",
        help="Serial device path, such as /dev/ttyACM3",
    )
    selector_group.add_argument(
        "--board-id",
        help=(
            "USB serial number of the Spin board to test; "
            f"default when no selector is provided: {DEFAULT_BOARD_ID}"
        ),
    )
    selector_group.add_argument(
        "--usb-vid",
        type=parse_usb_vid,
        metavar="VID",
        help=f"USB vendor ID in hexadecimal, such as {SPIN_USB_VID}",
    )
    parser.add_argument("--reflash", action="store_true",
                         help="Also verify persistence across a full firmware reflash")
    parser.add_argument("--env", default="USB",
                         help="PlatformIO environment to use with --reflash (default: USB)")
    args = parser.parse_args()

    requested_board_id = args.board_id
    if args.port is None and requested_board_id is None and args.usb_vid is None:
        requested_board_id = DEFAULT_BOARD_ID

    port = find_spin_port(
        port=args.port,
        board_id=requested_board_id,
        usb_vid=args.usb_vid,
    )
    if port is None:
        print("Error: no Spin board matched the requested selector.")
        return 1

    board_id = board_id_for_port(port) or requested_board_id
    if board_id is None:
        print(
            f"Error: {port} has no USB serial number, so the script "
            "cannot identify it safely after reset."
        )
        return 1

    print(f"Using board {board_id} on port {port}")
    ser = open_serial_with_retry(port)
    time.sleep(0.5)

    print("[1/8] Clearing all metadata fields...")
    print("  " + send_singleline_command(ser, "c"))

    print("[2/8] Reading back (expect all fields empty/ERR)...")
    fields = parse_kv_lines(send_multiline_command(ser, "r", "END_READ"))
    if not check_all_cleared(fields):
        print("FAIL: fields were not empty after clear")
        return 1

    print("[3/8] Attempting undersized writes (expect all rejected)...")
    badsize_results = parse_kv_lines(
        send_multiline_command(ser, "b", "END_BADSIZE")
    )
    if not check_all_rejected(badsize_results):
        print("FAIL: an undersized write was not rejected with -2")
        return 1

    print("[4/8] Reading back (expect fields still empty/ERR)...")
    fields = parse_kv_lines(send_multiline_command(ser, "r", "END_READ"))
    if not check_all_cleared(fields):
        print("FAIL: an undersized write corrupted storage")
        return 1

    print("[5/8] Checking free NVS space (baseline)...")
    print("  " + send_singleline_command(ser, "f"))

    print("[6/8] Writing canned test values...")
    write_lines = send_multiline_command(ser, "w", "END_WRITE")
    if any("ERR" in line for line in write_lines):
        print("FAIL: at least one field failed to write")
        print("\n".join(write_lines))
        return 1

    print("[7/8] Reading back immediately (same-boot sanity check)...")
    fields = parse_kv_lines(send_multiline_command(ser, "r", "END_READ"))
    if not check_all_present(fields):
        print("FAIL: values do not match right after writing")
        return 1

    print("[8/8] Resetting board and re-reading (persistence check)...")
    ser.close()
    ser, port = reset_and_reconnect(port, board_id)

    fields = parse_kv_lines(send_multiline_command(ser, "r", "END_READ"))
    if not check_all_present(fields):
        print("FAIL: values did not survive a board reset")
        return 1
    print("PASS: values survived a board reset")

    if args.reflash:
        print("[extra] Reflashing firmware and re-reading...")
        ser.close()
        reflash(args.env)
        ser, port = reconnect_after_reset(port, board_id)

        fields = parse_kv_lines(send_multiline_command(ser, "r", "END_READ"))
        if not check_all_present(fields):
            print("FAIL: values did not survive a firmware reflash")
            return 1
        print("PASS: values survived a firmware reflash")

    print("[final] Checking free NVS space (post)...")
    print("  " + send_singleline_command(ser, "f"))

    ser.close()
    print("ALL CHECKS PASSED")
    return 0


if __name__ == "__main__":
    sys.exit(main())
