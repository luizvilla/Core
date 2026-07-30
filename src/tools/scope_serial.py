#
# Copyright (c) 2021-present LAAS-CNRS
#
# SPDX-License-Identifier: GPL-2.0-or-later
#

"""Bounded parser for the dedicated OwnTech scope-data serial interface."""

from dataclasses import dataclass
import math
import re
import struct
import time

import serial


SCOPE_BAUDRATE = 115200
SCOPE_SAMPLE_COUNT = 1024
SCOPE_CHANNEL_NAMES = (
    "V1Low_V",
    "V2Low_V",
    "VHigh_V",
    "I1Low_A",
    "I2Low_A",
    "IHigh_A",
    "Duty1",
    "Duty2",
)

_PROBE_RESPONSE = "SCOPE-DATA/1 OK"
_BEGIN_RECORD = "begin record"
_END_RECORD = "end record"
_FINAL_INDEX_RE = re.compile(r"# ([0-9]+)")
_HEX_VALUE_RE = re.compile(r"[0-9A-Fa-f]{8}")


class ScopeSerialError(RuntimeError):
    """Raised for scope transport, framing, validation, or timeout errors."""


@dataclass(frozen=True)
class ScopeCapture:
    """One decoded capture in chronological sample order."""

    decimation: int
    sample_period_us: int
    duration_ms: float
    channel_names: tuple
    samples: tuple
    final_index: int
    pretrigger_ratio: float
    time_axis_s: tuple


class ScopeSerial:
    """Read the fixed scope protocol from an explicitly selected data port."""

    def __init__(
        self,
        port,
        *,
        timeout=1.0,
        probe_timeout=3.0,
        serial_factory=None,
    ):
        if not isinstance(port, str) or not port:
            raise ValueError("scope port must be an explicit non-empty string")
        if isinstance(timeout, bool) or not isinstance(timeout, (int, float)):
            raise ValueError("timeout must be a positive finite number")
        timeout = float(timeout)
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError("timeout must be a positive finite number")
        if (
            isinstance(probe_timeout, bool)
            or not isinstance(probe_timeout, (int, float))
        ):
            raise ValueError("probe_timeout must be a positive finite number")
        probe_timeout = float(probe_timeout)
        if not math.isfinite(probe_timeout) or probe_timeout <= 0:
            raise ValueError("probe_timeout must be a positive finite number")

        factory = serial.Serial if serial_factory is None else serial_factory
        self.port = port
        self.ser = None
        try:
            self.ser = factory(
                port=port,
                baudrate=SCOPE_BAUDRATE,
                timeout=timeout,
                write_timeout=timeout,
            )
            self._probe(probe_timeout)
        except Exception as exc:
            if self.ser is not None:
                self.ser.close()
            if isinstance(exc, (ScopeSerialError, ValueError)):
                raise
            raise ScopeSerialError(
                f"failed to open scope data port {port!r}: {exc}"
            ) from exc

    def close(self):
        if self.ser is not None:
            self.ser.close()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, traceback):
        self.close()

    def __repr__(self):
        return f"<ScopeSerial {self.port}>"

    def _write_command(self, command):
        if callable(getattr(self.ser, "reset_input_buffer", None)):
            self.ser.reset_input_buffer()
        try:
            written = self.ser.write(command)
            if written is not None and written != len(command):
                raise ScopeSerialError(
                    f"short write on scope data port: {written}/{len(command)}"
                )
            if callable(getattr(self.ser, "flush", None)):
                self.ser.flush()
        except ScopeSerialError:
            raise
        except Exception as exc:
            raise ScopeSerialError(
                f"scope data-port write failed: {exc}"
            ) from exc

    def _readline(self, deadline, *, maximum_length=256):
        data = bytearray()
        while time.monotonic() < deadline:
            try:
                chunk = self.ser.read(1)
            except Exception as exc:
                raise ScopeSerialError(
                    f"scope data-port read failed: {exc}"
                ) from exc
            if not chunk:
                continue
            data.extend(chunk)
            if len(data) > maximum_length:
                raise ScopeSerialError(
                    f"scope line exceeds {maximum_length} bytes"
                )
            if chunk == b"\n":
                try:
                    return data.rstrip(b"\r\n").decode("ascii")
                except UnicodeDecodeError as exc:
                    raise ScopeSerialError(
                        "scope response is not ASCII"
                    ) from exc
        raise ScopeSerialError("scope data-port response timed out")

    def _probe(self, timeout):
        self._write_command(b"?")
        deadline = time.monotonic() + timeout
        for _ in range(32):
            line = self._readline(deadline)
            if line == _PROBE_RESPONSE:
                return
            if line.startswith("SCOPE-DATA/1 ERROR"):
                raise ScopeSerialError(f"scope probe rejected: {line}")
        raise ScopeSerialError(
            f"scope probe mismatch: {_PROBE_RESPONSE!r} was not received"
        )

    @staticmethod
    def _validate_capture_metadata(
        decimation,
        sample_period_us,
        duration_ms,
        pretrigger_ratio,
    ):
        if (
            isinstance(decimation, bool)
            or not isinstance(decimation, int)
            or not 1 <= decimation <= 100
        ):
            raise ValueError("decimation must be an integer between 1 and 100")
        if (
            isinstance(sample_period_us, bool)
            or not isinstance(sample_period_us, int)
            or sample_period_us != 100 * decimation
        ):
            raise ValueError(
                "sample_period_us must equal 100 times decimation"
            )
        if isinstance(duration_ms, bool) or not isinstance(
            duration_ms, (int, float)
        ):
            raise ValueError("duration_ms must be a finite number")
        duration_ms = float(duration_ms)
        expected_duration_ms = (
            SCOPE_SAMPLE_COUNT * sample_period_us / 1000.0
        )
        if not math.isfinite(duration_ms) or not math.isclose(
            duration_ms,
            expected_duration_ms,
            rel_tol=1e-6,
            abs_tol=0.05,
        ):
            raise ValueError(
                f"duration_ms must be {expected_duration_ms:g}"
            )
        if isinstance(pretrigger_ratio, bool) or not isinstance(
            pretrigger_ratio, (int, float)
        ):
            raise ValueError("pretrigger_ratio must be between 0.0 and 0.9")
        pretrigger_ratio = float(pretrigger_ratio)
        if (
            not math.isfinite(pretrigger_ratio)
            or pretrigger_ratio < 0.0
            or pretrigger_ratio > 0.9
        ):
            raise ValueError("pretrigger_ratio must be between 0.0 and 0.9")
        return duration_ms, pretrigger_ratio

    def download(
        self,
        *,
        decimation,
        sample_period_us,
        duration_ms,
        pretrigger_ratio,
        timeout=15.0,
    ):
        """Request, validate, decode, and chronologically rotate a capture."""

        duration_ms, pretrigger_ratio = self._validate_capture_metadata(
            decimation,
            sample_period_us,
            duration_ms,
            pretrigger_ratio,
        )
        if isinstance(timeout, bool) or not isinstance(timeout, (int, float)):
            raise ValueError("timeout must be a positive finite number")
        timeout = float(timeout)
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError("timeout must be a positive finite number")

        self._write_command(b"D")
        deadline = time.monotonic() + timeout

        for _ in range(16):
            line = self._readline(deadline)
            if line == _BEGIN_RECORD:
                break
            if line.startswith("SCOPE-DATA/1 ERROR"):
                raise ScopeSerialError(line)
        else:
            raise ScopeSerialError("scope record start marker was not received")

        channel_line = self._readline(deadline)
        if not channel_line.startswith("#"):
            raise ScopeSerialError("scope channel header is malformed")
        channel_names = tuple(
            name for name in channel_line[1:].split(",") if name
        )
        if channel_names != SCOPE_CHANNEL_NAMES:
            raise ScopeSerialError(
                "scope channel header mismatch: "
                f"expected {SCOPE_CHANNEL_NAMES!r}, got {channel_names!r}"
            )

        final_index_line = self._readline(deadline)
        match = _FINAL_INDEX_RE.fullmatch(final_index_line)
        if match is None:
            raise ScopeSerialError("scope final-index line is malformed")
        final_index = int(match.group(1))
        if not 0 <= final_index < SCOPE_SAMPLE_COUNT:
            raise ScopeSerialError(
                f"scope final index {final_index} is out of range"
            )

        values = []
        value_count = SCOPE_SAMPLE_COUNT * len(SCOPE_CHANNEL_NAMES)
        for index in range(value_count):
            line = self._readline(deadline, maximum_length=16)
            if line == _END_RECORD:
                raise ScopeSerialError(
                    f"scope payload is truncated at {index}/{value_count} values"
                )
            if _HEX_VALUE_RE.fullmatch(line) is None:
                raise ScopeSerialError(
                    f"scope payload value {index} is not 8-digit hexadecimal"
                )
            values.append(struct.unpack(">f", bytes.fromhex(line))[0])

        terminator = self._readline(deadline, maximum_length=16)
        if terminator != _END_RECORD:
            if _HEX_VALUE_RE.fullmatch(terminator):
                raise ScopeSerialError(
                    f"scope payload contains more than {value_count} values"
                )
            raise ScopeSerialError("scope record end marker is malformed")

        rows = tuple(
            tuple(values[offset : offset + len(SCOPE_CHANNEL_NAMES)])
            for offset in range(0, value_count, len(SCOPE_CHANNEL_NAMES))
        )
        first_index = (final_index + 1) % SCOPE_SAMPLE_COUNT
        chronological_samples = rows[first_index:] + rows[:first_index]

        sample_period_s = sample_period_us * 1e-6
        trigger_offset = pretrigger_ratio * SCOPE_SAMPLE_COUNT
        time_axis_s = tuple(
            (index - trigger_offset) * sample_period_s
            for index in range(SCOPE_SAMPLE_COUNT)
        )

        return ScopeCapture(
            decimation=decimation,
            sample_period_us=sample_period_us,
            duration_ms=duration_ms,
            channel_names=channel_names,
            samples=chronological_samples,
            final_index=final_index,
            pretrigger_ratio=pretrigger_ratio,
            time_axis_s=time_axis_s,
        )
