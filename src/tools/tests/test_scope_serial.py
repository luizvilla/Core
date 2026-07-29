#
# Copyright (c) 2021-present LAAS-CNRS
#
# SPDX-License-Identifier: GPL-2.0-or-later
#

import struct
import sys
import unittest
from pathlib import Path


TOOLS_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(TOOLS_DIR))

from scope_serial import (  # noqa: E402
    SCOPE_CHANNEL_NAMES,
    SCOPE_SAMPLE_COUNT,
    ScopeSerial,
    ScopeSerialError,
)


class FakeSerial:
    def __init__(self, probe_response=b"SCOPE-DATA/1 OK\n", record=b""):
        self.probe_response = probe_response
        self.record = record
        self.input = bytearray()
        self.writes = []
        self.closed = False
        self.settings = None

    def factory(self, **settings):
        self.settings = settings
        return self

    def reset_input_buffer(self):
        self.input.clear()

    def write(self, value):
        self.writes.append(value)
        if value == b"?":
            self.input.extend(self.probe_response)
        elif value == b"D":
            self.input.extend(self.record)
        return len(value)

    def flush(self):
        pass

    def read(self, size):
        if not self.input:
            return b""
        result = bytes(self.input[:size])
        del self.input[:size]
        return result

    def close(self):
        self.closed = True


def make_record(
    *,
    final_index=1022,
    channels=SCOPE_CHANNEL_NAMES,
    value_count=SCOPE_SAMPLE_COUNT * len(SCOPE_CHANNEL_NAMES),
    corrupt_value=None,
    terminator="end record",
):
    lines = [
        "begin record",
        "#" + ",".join(channels) + ",",
        f"# {final_index}",
    ]
    for index in range(value_count):
        if corrupt_value is not None and index == corrupt_value:
            lines.append("not-hex!")
        else:
            sample = index // len(SCOPE_CHANNEL_NAMES)
            channel = index % len(SCOPE_CHANNEL_NAMES)
            value = float(sample * 10 + channel)
            lines.append(struct.pack(">f", value).hex())
    lines.append(terminator)
    return ("\n".join(lines) + "\n").encode("ascii")


class ScopeSerialTests(unittest.TestCase):
    def make_scope(self, record, **kwargs):
        fake = FakeSerial(record=record)
        scope = ScopeSerial(
            "fake-scope-port",
            serial_factory=fake.factory,
            **kwargs,
        )
        return scope, fake

    def test_probe_is_explicit_fixed_baud_and_closes_on_mismatch(self):
        with self.assertRaises(ValueError):
            ScopeSerial(None)

        fake = FakeSerial(probe_response=b"WRONG PORT\n")
        with self.assertRaises(ScopeSerialError):
            ScopeSerial(
                "fake-scope-port",
                probe_timeout=0.001,
                serial_factory=fake.factory,
            )
        self.assertTrue(fake.closed)
        self.assertEqual(fake.settings["baudrate"], 115200)

    def test_valid_capture_is_decoded_and_rotated(self):
        scope, fake = self.make_scope(make_record())
        capture = scope.download(
            decimation=10,
            sample_period_us=1000,
            duration_ms=1024.0,
            pretrigger_ratio=0.2,
        )

        self.assertEqual(fake.writes, [b"?", b"D"])
        self.assertEqual(capture.channel_names, SCOPE_CHANNEL_NAMES)
        self.assertEqual(capture.final_index, 1022)
        self.assertEqual(capture.samples[0][0], 10230.0)
        self.assertEqual(capture.samples[1][0], 0.0)
        self.assertEqual(len(capture.samples), SCOPE_SAMPLE_COUNT)
        self.assertEqual(len(capture.samples[0]), len(SCOPE_CHANNEL_NAMES))
        self.assertAlmostEqual(capture.time_axis_s[0], -0.2048)
        scope.close()
        self.assertTrue(fake.closed)

    def test_truncated_nonhex_and_wrong_count_payloads_are_rejected(self):
        cases = (
            ("truncated", make_record(value_count=8191), "truncated"),
            (
                "nonhex",
                make_record(corrupt_value=123),
                "not 8-digit hexadecimal",
            ),
            (
                "extra",
                make_record(value_count=8193),
                "more than 8192 values",
            ),
        )
        for name, record, message in cases:
            with self.subTest(name=name):
                scope, _ = self.make_scope(record)
                with self.assertRaisesRegex(ScopeSerialError, message):
                    scope.download(
                        decimation=1,
                        sample_period_us=100,
                        duration_ms=102.4,
                        pretrigger_ratio=0.0,
                    )

    def test_wrong_channel_and_malformed_final_index_are_rejected(self):
        wrong_channels = ("wrong",) + SCOPE_CHANNEL_NAMES[1:]
        scope, _ = self.make_scope(make_record(channels=wrong_channels))
        with self.assertRaisesRegex(ScopeSerialError, "header mismatch"):
            scope.download(
                decimation=1,
                sample_period_us=100,
                duration_ms=102.4,
                pretrigger_ratio=0.0,
            )

        record = make_record().replace(b"# 1022\n", b"## 1022\n", 1)
        scope, _ = self.make_scope(record)
        with self.assertRaisesRegex(ScopeSerialError, "final-index"):
            scope.download(
                decimation=1,
                sample_period_us=100,
                duration_ms=102.4,
                pretrigger_ratio=0.0,
            )

    def test_not_ready_timeout_and_bounded_line_are_rejected(self):
        scope, _ = self.make_scope(
            b"SCOPE-DATA/1 ERROR NOT_READY ARMED\n"
        )
        with self.assertRaisesRegex(ScopeSerialError, "NOT_READY ARMED"):
            scope.download(
                decimation=1,
                sample_period_us=100,
                duration_ms=102.4,
                pretrigger_ratio=0.0,
            )

        scope, _ = self.make_scope(b"")
        with self.assertRaisesRegex(ScopeSerialError, "timed out"):
            scope.download(
                decimation=1,
                sample_period_us=100,
                duration_ms=102.4,
                pretrigger_ratio=0.0,
                timeout=0.001,
            )

        scope, _ = self.make_scope(b"x" * 257 + b"\n")
        with self.assertRaisesRegex(ScopeSerialError, "exceeds 256"):
            scope.download(
                decimation=1,
                sample_period_us=100,
                duration_ms=102.4,
                pretrigger_ratio=0.0,
            )

    def test_capture_metadata_validation(self):
        scope, _ = self.make_scope(make_record())
        invalid = (
            dict(
                decimation=True,
                sample_period_us=100,
                duration_ms=102.4,
                pretrigger_ratio=0.0,
            ),
            dict(
                decimation=0,
                sample_period_us=100,
                duration_ms=102.4,
                pretrigger_ratio=0.0,
            ),
            dict(
                decimation=101,
                sample_period_us=10100,
                duration_ms=10342.4,
                pretrigger_ratio=0.0,
            ),
            dict(
                decimation=10,
                sample_period_us=100,
                duration_ms=102.4,
                pretrigger_ratio=0.0,
            ),
            dict(
                decimation=1,
                sample_period_us=100,
                duration_ms=float("nan"),
                pretrigger_ratio=0.0,
            ),
            dict(
                decimation=1,
                sample_period_us=100,
                duration_ms=102.4,
                pretrigger_ratio=float("inf"),
            ),
        )
        for metadata in invalid:
            with self.subTest(metadata=metadata), self.assertRaises(ValueError):
                scope.download(**metadata)


if __name__ == "__main__":
    unittest.main()
