#
# Copyright (c) 2021-present LAAS-CNRS
#
# SPDX-License-Identifier: GPL-2.0-or-later
#

"""Minimal SCPI driver for the Rigol MHO900 oscilloscopes (MHO984).

Ported from Core4 tests/twt_bnc/rigol_scope.py (branch test/twt-bnc). Covers: channel and timebase setup, an
edge trigger, a single acquisition, the screen waveform of a channel, the
scope's own measurements and a PNG screenshot.

Transport: the Linux ``usbtmc`` kernel driver (``/dev/usbtmc0``) with no
third-party dependency, or pyvisa when a VISA resource string is given.
``/dev/usbtmc*`` is root-only by default: install ``99-rigol-usbtmc.rules``
(see tests/common/README.md) once.
"""

import fcntl
import os
import struct
import time

# Values above this are Rigol's "no valid measurement" marker (9.9E37).
INVALID_MEASUREMENT = 9.0e37

# ioctl(USBTMC_IOCTL_SET_TIMEOUT) = _IOW('[', 10, __u32), timeout in ms.
_USBTMC_IOCTL_SET_TIMEOUT = (1 << 30) | (4 << 16) | (ord("[") << 8) | 10
# ioctl(USBTMC_IOCTL_CLEAR) = _IO('[', 2): device clear. Not used by
# default: on the MHO984 the first read after a clear (or after plugging the
# scope in) is lost, see RigolScope.sync().
_USBTMC_IOCTL_CLEAR = (ord("[") << 8) | 2


class ScopeError(RuntimeError):
    """Raised for transport errors, timeouts or malformed scope replies."""


class UsbtmcTransport:
    """SCPI over the Linux usbtmc character device."""

    def __init__(self, path="/dev/usbtmc0", timeout_ms=10000):
        self.path = path
        try:
            self.fd = os.open(path, os.O_RDWR)
        except PermissionError as exc:
            raise ScopeError(
                f"no permission on {path}: install tests/common/udev/"
                "99-rigol-usbtmc.rules (see tests/common/README.md)"
            ) from exc
        except FileNotFoundError as exc:
            raise ScopeError(f"{path} not found: is the scope plugged in?") from exc
        try:
            fcntl.ioctl(self.fd, _USBTMC_IOCTL_SET_TIMEOUT,
                        struct.pack("I", timeout_ms))
        except OSError:
            pass  # older kernels: keep the driver default (5 s)

    def clear(self):
        try:
            fcntl.ioctl(self.fd, _USBTMC_IOCTL_CLEAR)
        except OSError:
            pass

    def write(self, command):
        os.write(self.fd, (command + "\n").encode())

    def read_raw(self, chunk=1 << 20):
        try:
            return os.read(self.fd, chunk)
        except OSError as exc:
            raise ScopeError(f"read from {self.path} failed: {exc}") from exc

    def close(self):
        os.close(self.fd)


class VisaTransport:
    """SCPI through pyvisa (optional, only when a VISA resource is given)."""

    def __init__(self, resource, timeout_ms=10000):
        import pyvisa

        self.path = resource
        self.inst = pyvisa.ResourceManager("@py").open_resource(resource)
        self.inst.timeout = timeout_ms

    def write(self, command):
        self.inst.write(command)

    def read_raw(self, chunk=1 << 20):
        return self.inst.read_raw(chunk)

    def close(self):
        self.inst.close()


def open_transport(address):
    """``/dev/usbtmcN`` path -> usbtmc, anything else -> pyvisa resource."""
    if address.startswith("/dev/"):
        return UsbtmcTransport(address)
    return VisaTransport(address)


class RigolScope:
    """High-level operations used by the test on a 4-channel Rigol scope."""

    CHANNELS = (1, 2, 3, 4)

    def __init__(self, transport):
        self.t = transport
        self.sync()

    def sync(self, tries=3):
        """The MHO984 drops the first reply after it is plugged in (or after
        a device clear): send *IDN? until a reply comes back."""
        for _ in range(tries):
            try:
                if self.query("*IDN?").upper().startswith("RIGOL"):
                    return
            except ScopeError:
                continue
        raise ScopeError("scope does not answer *IDN? (replug its USB cable)")

    # ---- SCPI primitives ------------------------------------------------

    def write(self, command):
        self.t.write(command)

    def query(self, command):
        self.t.write(command)
        data = b""
        deadline = time.monotonic() + 10.0
        while not data.endswith(b"\n"):
            data += self.t.read_raw()
            if time.monotonic() > deadline:
                raise ScopeError(f"no complete reply to {command!r}")
        return data.decode(errors="replace").strip()

    def query_block(self, command):
        """Query returning an IEEE 488.2 definite-length block (#NLLLL...)."""
        self.t.write(command)
        data = self.t.read_raw()
        start = data.find(b"#")
        if start < 0 or len(data) < start + 2:
            raise ScopeError(f"no block header in reply to {command!r}")
        digits = int(data[start + 1:start + 2])
        header_end = start + 2 + digits
        while len(data) < header_end:
            data += self.t.read_raw()
        length = int(data[start + 2:header_end])
        deadline = time.monotonic() + 30.0
        while len(data) < header_end + length:
            data += self.t.read_raw()
            if time.monotonic() > deadline:
                raise ScopeError(f"block reply to {command!r} truncated")
        return data[header_end:header_end + length]

    def errors(self):
        """The MHO984 (fw 00.01.00) never answers :SYSTem:ERRor? (nor *ESR?),
        so the error queue cannot be read: setups are verified by
        read-back instead (see read_channel)."""
        return []

    # ---- setup ---------------------------------------------------------

    def idn(self):
        return self.query("*IDN?")

    def setup_channel(self, channel, *, scale, offset, probe=10,
                      coupling="DC", bandwidth="OFF"):
        c = f":CHANnel{channel}"
        self.write(f"{c}:DISPlay ON")
        self.write(f"{c}:PROBe {probe}")
        self.write(f"{c}:COUPling {coupling}")
        self.write(f"{c}:BWLimit {bandwidth}")
        self.write(f"{c}:SCALe {scale}")
        self.write(f"{c}:OFFSet {offset}")

    def read_channel(self, channel):
        c = f":CHANnel{channel}"
        return {
            "probe": self.query(f"{c}:PROBe?"),
            "coupling": self.query(f"{c}:COUPling?"),
            "scale_V": self.query(f"{c}:SCALe?"),
            "offset_V": self.query(f"{c}:OFFSet?"),
        }

    def set_acquire(self, kind):
        """NORMal, or HRESolution: each screen point is the average of the
        raw samples behind it (removes PWM ripple from DC levels)."""
        self.write(f":ACQuire:TYPE {kind}")

    def set_timebase(self, scale_s, offset_s=0.0):
        self._timebase = scale_s
        self.write(":TIMebase:MAIN:SCALe " + f"{scale_s:.3e}")
        self.write(":TIMebase:MAIN:OFFSet " + f"{offset_s:.3e}")

    def set_edge_trigger(self, channel, level_v, slope="POSitive", sweep="AUTO"):
        self.write(":TRIGger:MODE EDGE")
        self.write(f":TRIGger:EDGE:SOURce CHANnel{channel}")
        self.write(f":TRIGger:EDGE:SLOPe {slope}")
        self.write(f":TRIGger:EDGE:LEVel {level_v}")
        self.write(f":TRIGger:SWEep {sweep}")

    # ---- acquisition -----------------------------------------------------

    def run(self):
        self.write(":RUN")

    def single(self, tries=3):
        """One fresh acquisition, then the scope is stopped on it.

        On the MHO984 (fw 00.01.00) :SINGle ends with an empty record about
        half the time when the auto-trigger fires. Instead: run, wait for at
        least two full screens, stop, and check the record is complete."""
        window = 10.0 * getattr(self, "_timebase", 1e-3)
        for _ in range(tries):
            self.write(":RUN")
            time.sleep(0.15 + 1.2 * window)
            self.write(":STOP")
            time.sleep(0.05)
            if self.record_points() > 1:
                return
        raise ScopeError(f"no complete acquisition after {tries} tries")

    def record_points(self):
        try:
            return int(float(self.query(":WAVeform:PREamble?").split(",")[2]))
        except (ValueError, IndexError):
            return 0

    def waveform(self, channel):
        """Screen waveform of a channel as (times_s, volts) lists."""
        self.write(f":WAVeform:SOURce CHANnel{channel}")
        self.write(":WAVeform:MODE NORMal")
        # WORD keeps the 12-bit resolution; the preamble describes its scaling.
        self.write(":WAVeform:FORMat WORD")
        pre = [float(x) for x in self.query(":WAVeform:PREamble?").split(",")]
        # format, type, points, count, xinc, xorigin, xref, yinc, yorigin, yref
        points = int(pre[2])
        xinc, xorig, xref, yinc, yorig, yref = pre[4:10]
        raw = self.query_block(":WAVeform:DATA?")
        if int(pre[0]) == 1 and len(raw) >= 2 * points:
            codes = struct.unpack(f"<{points}H", raw[:2 * points])
        else:
            codes = raw
        volts = [(b - yorig - yref) * yinc for b in codes]
        if len(volts) < 2:
            raise ScopeError(f"empty waveform on CH{channel}")
        times = [(i - xref) * xinc + xorig for i in range(len(volts))]
        return times, volts

    def measure(self, item, channel, channel2=None):
        """Scope measurement (e.g. FREQuency, PDUTy, VTOP, VAVG, OVERshoot).
        Returns None when the scope reports no valid value."""
        source = f"CHANnel{channel}"
        if channel2 is not None:
            source += f",CHANnel{channel2}"
        try:
            value = float(self.query(f":MEASure:ITEM? {item},{source}"))
        except (ValueError, ScopeError):
            return None
        return None if abs(value) >= INVALID_MEASUREMENT else value

    def screenshot(self):
        """PNG of the screen as bytes (None if the scope refuses)."""
        for command in (":DISPlay:DATA? PNG", ":DISPlay:DATA?"):
            try:
                data = self.query_block(command)
            except ScopeError:
                continue
            if data[:8] == b"\x89PNG\r\n\x1a\n":
                return data
            if data[:2] == b"BM":
                return data  # BMP: still embeddable as image/bmp
        return None

    def close(self):
        self.t.close()


def main():
    """Identify the scope and print the scope's measurements on CH1..CH4."""
    import argparse

    ap = argparse.ArgumentParser(description=main.__doc__)
    ap.add_argument("--scope", default="/dev/usbtmc0",
                    help="/dev/usbtmcN or a VISA resource")
    args = ap.parse_args()
    scope = RigolScope(open_transport(args.scope))
    try:
        print(scope.idn())
        for ch in scope.CHANNELS:
            vals = {item: scope.measure(item, ch) for item in ("VPP", "VAVG", "FREQuency")}
            print(f"CH{ch}", vals)
    finally:
        scope.close()


if __name__ == "__main__":
    main()
