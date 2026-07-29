#!/usr/bin/env python3
"""
Automated write -> reset -> read regression test for the spin.metaData
NVS persistence feature.

Drives the serial command menu exposed by the test harness in
src/main.cpp (h/w/r/c/f) to verify that board/shield metadata survives:

  1. A same-boot round trip (sanity check).
  2. A board reset, triggered via the classic 1200bps-touch convention
     (the same mechanism this project's own bootloader upload script
     uses, see owntech/scripts/pre_bootloader_serial.py).
  3. Optionally (--reflash), a full firmware reflash via
     `platformio run -t upload`, proving the storage_partition survives
     an application image update.

Requires the board to be flashed with the src/main.cpp test harness, and
pyserial installed (`pip install pyserial`).

Usage:
    python3 owntech/scripts/test_metadata_nvs.py [--port PORT] [--reflash]
"""

import argparse
import re
import subprocess
import sys
import time

import serial
import serial.tools.list_ports


SPIN_USB_VID = "2FE3"
BAUD_RATE = 115200
COMMAND_TIMEOUT_S = 5
RESET_WAIT_S = 10
BOOT_SETTLE_S = 1.5

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


def find_spin_port():
    ports = list(serial.tools.list_ports.grep(SPIN_USB_VID))
    if not ports:
        return None
    if len(ports) > 1:
        print(f"Multiple Spin boards found, using the first one: {ports[0].device}")
    return ports[0].device


def wait_for_port(exclude_device=None, timeout=RESET_WAIT_S):
    """Poll for a Spin board USB port to (re)appear after a reset."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        port = find_spin_port()
        if port is not None and port != exclude_device:
            return port
        time.sleep(0.25)
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
    ok = True
    for key, expected_value in EXPECTED_VALUES.items():
        actual = fields.get(key)
        if actual != expected_value:
            print(f"  MISMATCH {key}: expected {expected_value!r}, got {actual!r}")
            ok = False
    return ok


def check_all_cleared(fields):
    ok = True
    for key in EXPECTED_VALUES:
        actual = fields.get(key, "")
        if not actual.startswith("ERR"):
            print(f"  UNEXPECTED VALUE {key}: expected ERR after clear, got {actual!r}")
            ok = False
    return ok


def touch_reset(port):
    """Trigger a 1200bps-touch reset, same convention as
    env.TouchSerialPort() in pre_bootloader_serial.py."""
    ser = serial.Serial(port, 1200)
    ser.close()


def reconnect_after_reset(previous_port):
    new_port = wait_for_port(exclude_device=None, timeout=RESET_WAIT_S)
    if new_port is None:
        input("Board did not reappear automatically. Please power-cycle "
              "it manually, then press Enter to continue...")
        new_port = find_spin_port() or previous_port

    ser = serial.Serial(new_port, BAUD_RATE, timeout=1)
    time.sleep(BOOT_SETTLE_S)
    return ser, new_port


def reflash(env_name="USB"):
    print("Reflashing firmware via 'platformio run -t upload'...")
    subprocess.run(["platformio", "run", "-e", env_name, "-t", "upload"], check=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", help="Serial port (auto-detected if omitted)")
    parser.add_argument("--reflash", action="store_true",
                         help="Also verify persistence across a full firmware reflash")
    parser.add_argument("--env", default="USB",
                         help="PlatformIO environment to use with --reflash (default: USB)")
    args = parser.parse_args()

    port = args.port or find_spin_port()
    if port is None:
        print("Error: no Spin board found. Connect one or pass --port.")
        return 1

    print(f"Using port {port}")
    ser = serial.Serial(port, BAUD_RATE, timeout=1)
    time.sleep(0.5)

    print("[1/6] Clearing all metadata fields...")
    print("  " + send_singleline_command(ser, "c"))

    print("[2/6] Reading back (expect all fields empty/ERR)...")
    fields = parse_kv_lines(send_multiline_command(ser, "r", "END_READ"))
    if not check_all_cleared(fields):
        print("FAIL: fields were not empty after clear")
        return 1

    print("[3/6] Checking free NVS space (baseline)...")
    print("  " + send_singleline_command(ser, "f"))

    print("[4/6] Writing canned test values...")
    write_lines = send_multiline_command(ser, "w", "END_WRITE")
    if any("ERR" in line for line in write_lines):
        print("FAIL: at least one field failed to write")
        print("\n".join(write_lines))
        return 1

    print("[5/6] Reading back immediately (same-boot sanity check)...")
    fields = parse_kv_lines(send_multiline_command(ser, "r", "END_READ"))
    if not check_all_present(fields):
        print("FAIL: values do not match right after writing")
        return 1

    print("[6/6] Resetting board and re-reading (persistence check)...")
    ser.close()
    touch_reset(port)
    ser, port = reconnect_after_reset(port)

    fields = parse_kv_lines(send_multiline_command(ser, "r", "END_READ"))
    if not check_all_present(fields):
        print("FAIL: values did not survive a board reset")
        return 1
    print("PASS: values survived a board reset")

    if args.reflash:
        print("[extra] Reflashing firmware and re-reading...")
        ser.close()
        reflash(args.env)
        ser, port = reconnect_after_reset(port)

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
