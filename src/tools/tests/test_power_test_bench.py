#
# Copyright (c) 2021-present LAAS-CNRS
#
# SPDX-License-Identifier: GPL-2.0-or-later
#

import copy
import sys
import unittest
from pathlib import Path


TOOLS_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(TOOLS_DIR))

from power_test_bench import (  # noqa: E402
    PowerTestBench,
    PowerTestBenchError,
    TesterMode,
)


class FakeThingSetClient:
    def __init__(self):
        leg = {
            "wEnable": False,
            "wCapa": False,
            "wDriver": False,
            "wBuck": False,
            "wBoost": False,
            "wDutyCycle": 0.1,
            "wReferenceValue": 0.0,
            "wTrackingVar": "V1",
            "wPhaseShift": 0,
            "wDeadTimeRising_ns": 100,
            "wDeadTimeFalling_ns": 100,
        }
        calibration = {"wGain": 1.0, "wOffset": 0.0, "wStore": False}
        self.state = {
            "Config": {"Mode": 0, "Frequency_Hz": 200000},
            "Config/Leg1": copy.deepcopy(leg),
            "Config/Leg2": copy.deepcopy(leg),
            "Measurements": {"rV1Low_V": 12.0, "rDuty1": 0.25},
            "Calibration/V1": copy.deepcopy(calibration),
            "Calibration/V2": copy.deepcopy(calibration),
            "Calibration/VH": copy.deepcopy(calibration),
            "Calibration/I1": copy.deepcopy(calibration),
            "Calibration/I2": copy.deepcopy(calibration),
            "Calibration/IH": copy.deepcopy(calibration),
            "Converter": {
                "wBoardName": "TWIST",
                "wBoardVersion": "v1.4.2",
                "wSerialNumber": "UNSET",
                "wFirmwareVersion": "1.0.0",
            },
        }
        self.operations = []
        self.fail_writes = {}
        self.restore_values = {}

    def read(self, path):
        self.operations.append(("read", path))
        return copy.deepcopy(self.state[path])

    def write(self, path, values):
        self.operations.append(("write", path, copy.deepcopy(values)))
        remaining = self.fail_writes.get(path, 0)
        if remaining:
            self.fail_writes[path] = remaining - 1
            raise RuntimeError(f"injected write failure at {path}")
        self.state[path].update(values)
        if values.get("wStore") is True:
            self.state[path]["wStore"] = False
        for (restore_path, name), restored in self.restore_values.items():
            if restore_path == path and name in values:
                self.state[path][name] = restored
        return True


class PowerTestBenchTests(unittest.TestCase):
    def setUp(self):
        self.client = FakeThingSetClient()
        self.bench = PowerTestBench(self.client)

    def test_mode_and_frequency_are_written_and_verified(self):
        self.assertEqual(self.bench.set_mode("power_off"), TesterMode.POWER_OFF)
        self.assertEqual(self.bench.set_frequency(100000), 100000)
        self.assertEqual(self.client.state["Config"]["Mode"], 2)
        self.assertEqual(self.client.state["Config"]["Frequency_Hz"], 100000)

    def test_configure_leg_maps_fields_and_clears_opposite_mode(self):
        result = self.bench.configure_leg(
            "leg2",
            boost=True,
            duty_cycle=0.25,
            reference_value=24,
            tracking_var="vh",
            phase_shift=-90,
            dead_time_rising_ns=120,
            dead_time_falling_ns=130,
        )
        self.assertTrue(result["wBoost"])
        self.assertFalse(result["wBuck"])
        self.assertEqual(result["wTrackingVar"], "VH")

    def test_validation_rejects_unsafe_or_ambiguous_leg_values(self):
        invalid_calls = (
            lambda: self.bench.configure_leg(1, duty_cycle=1.01),
            lambda: self.bench.configure_leg(1, buck=True, boost=True),
            lambda: self.bench.configure_leg(1, phase_shift=361),
            lambda: self.bench.configure_leg(1, dead_time_rising_ns=-1),
            lambda: self.bench.configure_leg(1, tracking_var="temperature"),
        )
        for call in invalid_calls:
            with self.subTest(call=call), self.assertRaises(ValueError):
                call()

    def test_firmware_restored_value_raises_readback_error(self):
        self.client.restore_values[("Config/Leg1", "wDutyCycle")] = 0.1
        with self.assertRaisesRegex(PowerTestBenchError, "read back"):
            self.bench.configure_leg(1, duty_cycle=0.5)

    def test_calibration_store_reset_is_expected(self):
        result = self.bench.set_calibration(
            "i1", gain=2.0, offset=-0.25, store=True
        )
        self.assertEqual(result["wGain"], 2.0)
        self.assertEqual(result["wOffset"], -0.25)
        self.assertFalse(result["wStore"])

    def test_metadata_uses_friendly_names_and_validates_limits(self):
        metadata = self.bench.set_metadata(
            board_name="OWNVERTER", serial_number="SN-123"
        )
        self.assertEqual(metadata["board_name"], "OWNVERTER")
        self.assertEqual(metadata["serial_number"], "SN-123")
        self.assertEqual(self.bench.read_metadata(), metadata)

        with self.assertRaises(ValueError):
            self.bench.set_metadata(board_version="")
        with self.assertRaises(ValueError):
            self.bench.set_metadata(firmware_version="bad\nversion")
        with self.assertRaises(ValueError):
            self.bench.set_metadata(board_name="x" * 24)

    def test_measurement_and_leg_reads(self):
        self.assertEqual(self.bench.read_measurements()["rV1Low_V"], 12.0)
        self.assertFalse(self.bench.read_leg(1)["wEnable"])

    def test_power_on_uses_safe_sequence_and_selects_one_leg(self):
        self.bench.power_on(
            2,
            connect_driver=True,
            duty_cycle=0.2,
        )
        writes = [operation for operation in self.client.operations if operation[0] == "write"]
        self.assertEqual(writes[0], ("write", "Config", {"Mode": 2}))
        self.assertEqual(writes[-1], ("write", "Config", {"Mode": 1}))
        self.assertFalse(self.client.state["Config/Leg1"]["wEnable"])
        self.assertTrue(self.client.state["Config/Leg2"]["wEnable"])
        self.assertTrue(self.client.state["Config/Leg2"]["wDriver"])

    def test_power_on_failure_attempts_shutdown(self):
        self.client.fail_writes["Config/Leg2"] = 1
        with self.assertRaises(PowerTestBenchError):
            self.bench.power_on(1, connect_driver=True)
        self.assertEqual(self.client.state["Config"]["Mode"], 2)
        self.assertFalse(self.client.state["Config/Leg1"]["wEnable"])
        self.assertFalse(self.client.state["Config/Leg2"]["wEnable"])

    def test_shutdown_attempts_every_operation(self):
        self.client.fail_writes["Config"] = 1
        self.client.fail_writes["Config/Leg1"] = 1
        with self.assertRaisesRegex(PowerTestBenchError, "shutdown completed"):
            self.bench.shutdown(disconnect_hardware=True)
        leg2 = self.client.state["Config/Leg2"]
        self.assertFalse(leg2["wEnable"])
        self.assertFalse(leg2["wDriver"])
        self.assertFalse(leg2["wCapa"])


if __name__ == "__main__":
    unittest.main()
