#
# Copyright (c) 2021-present LAAS-CNRS
#
#   This program is free software: you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation, either version 2 of the License, or
#   (at your option) any later version.
#
# SPDX-License-Identifier: GPL-2.0-or-later
#

"""High-level, safety-oriented client for the ThingSet power test bench."""

from enum import IntEnum
import math


class TesterMode(IntEnum):
    """Values exposed by ``Config/Mode`` in the test-bench firmware."""

    IDLE = 0
    POWER_ON = 1
    POWER_OFF = 2


class PowerTestBenchError(RuntimeError):
    """Raised when validation, communication, or readback verification fails."""


class PowerTestBench:
    """Power-test-bench operations layered over a ThingSet transport.

    ``client`` may be :class:`thingset_tools.ThingSetTools` or any object
    providing compatible ``read(path)`` and ``write(path, values)`` methods.
    """

    _LEGS = {"1": 1, "LEG1": 1, "2": 2, "LEG2": 2}
    _CHANNELS = ("V1", "V2", "VH", "I1", "I2", "IH")
    _METADATA_FIELDS = {
        "board_name": ("wBoardName", 23),
        "board_version": ("wBoardVersion", 15),
        "serial_number": ("wSerialNumber", 47),
        "firmware_version": ("wFirmwareVersion", 31),
    }

    def __init__(self, client):
        if not callable(getattr(client, "read", None)) or not callable(
            getattr(client, "write", None)
        ):
            raise TypeError("client must provide read(path) and write(path, values)")
        self.client = client

    @classmethod
    def _normalize_leg(cls, leg):
        if isinstance(leg, bool):
            raise ValueError("leg must be 1, 2, 'Leg1', or 'Leg2'")
        key = str(leg).upper()
        try:
            return cls._LEGS[key]
        except KeyError as exc:
            raise ValueError("leg must be 1, 2, 'Leg1', or 'Leg2'") from exc

    @classmethod
    def _normalize_channel(cls, channel):
        normalized = str(channel).upper()
        if normalized not in cls._CHANNELS:
            raise ValueError(
                "channel must be one of " + ", ".join(cls._CHANNELS)
            )
        return normalized

    @staticmethod
    def _validate_bool(name, value):
        if not isinstance(value, bool):
            raise ValueError(f"{name} must be a bool")
        return value

    @staticmethod
    def _validate_finite(name, value):
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError(f"{name} must be a finite number")
        value = float(value)
        if not math.isfinite(value):
            raise ValueError(f"{name} must be a finite number")
        return value

    @staticmethod
    def _validate_integer(name, value, minimum, maximum=None):
        if isinstance(value, bool) or not isinstance(value, int):
            raise ValueError(f"{name} must be an integer")
        if value < minimum or (maximum is not None and value > maximum):
            if maximum is None:
                raise ValueError(f"{name} must be at least {minimum}")
            raise ValueError(f"{name} must be between {minimum} and {maximum}")
        return value

    @staticmethod
    def _values_match(actual, expected, absolute_tolerance):
        if isinstance(expected, float):
            return (
                isinstance(actual, (int, float))
                and not isinstance(actual, bool)
                and math.isclose(
                    float(actual),
                    expected,
                    rel_tol=1e-6,
                    abs_tol=absolute_tolerance,
                )
            )
        return actual == expected

    def _write_and_verify(
        self, path, values, *, absolute_tolerance=0.0, expected=None
    ):
        try:
            self.client.write(path, values)
            readback = self.client.read(path)
        except Exception as exc:
            raise PowerTestBenchError(f"ThingSet operation failed at {path}: {exc}") from exc

        if not isinstance(readback, dict):
            raise PowerTestBenchError(
                f"expected object readback from {path}, got {readback!r}"
            )

        expected_values = values if expected is None else expected
        for name, requested in expected_values.items():
            if name not in readback:
                raise PowerTestBenchError(
                    f"{path}/{name} is missing from the firmware readback"
                )
            if not self._values_match(
                readback[name], requested, absolute_tolerance
            ):
                raise PowerTestBenchError(
                    f"{path}/{name} read back as {readback[name]!r}, "
                    f"requested {requested!r}"
                )
        return readback

    @staticmethod
    def _normalize_mode(mode):
        if isinstance(mode, TesterMode):
            return mode
        if isinstance(mode, str):
            try:
                return TesterMode[mode.upper()]
            except KeyError as exc:
                raise ValueError(
                    "mode must be IDLE, POWER_ON, POWER_OFF, 0, 1, or 2"
                ) from exc
        if isinstance(mode, bool):
            raise ValueError("mode must be IDLE, POWER_ON, POWER_OFF, 0, 1, or 2")
        try:
            return TesterMode(mode)
        except (TypeError, ValueError) as exc:
            raise ValueError(
                "mode must be IDLE, POWER_ON, POWER_OFF, 0, 1, or 2"
            ) from exc

    def set_mode(self, mode):
        """Set and verify the global tester mode."""

        normalized = self._normalize_mode(mode)
        self._write_and_verify("Config", {"Mode": int(normalized)})
        return normalized

    def set_frequency(self, frequency_hz):
        """Set and verify the PWM switching frequency."""

        frequency_hz = self._validate_integer(
            "frequency_hz", frequency_hz, minimum=1
        )
        self._write_and_verify("Config", {"Frequency_Hz": frequency_hz})
        return frequency_hz

    def configure_leg(
        self,
        leg,
        *,
        enable=None,
        capacitor=None,
        driver=None,
        buck=None,
        boost=None,
        duty_cycle=None,
        reference_value=None,
        tracking_var=None,
        phase_shift=None,
        dead_time_rising_ns=None,
        dead_time_falling_ns=None,
    ):
        """Validate, write, and verify one leg's supplied settings."""

        leg_number = self._normalize_leg(leg)
        values = {}

        boolean_fields = (
            ("wEnable", "enable", enable),
            ("wCapa", "capacitor", capacitor),
            ("wDriver", "driver", driver),
            ("wBuck", "buck", buck),
            ("wBoost", "boost", boost),
        )
        for field, argument, value in boolean_fields:
            if value is not None:
                values[field] = self._validate_bool(argument, value)

        if buck is True and boost is True:
            raise ValueError("buck and boost cannot both be true")
        if buck is True:
            values["wBoost"] = False
        if boost is True:
            values["wBuck"] = False

        if duty_cycle is not None:
            duty = self._validate_finite("duty_cycle", duty_cycle)
            if duty < 0.0 or duty > 1.0:
                raise ValueError("duty_cycle must be between 0 and 1")
            values["wDutyCycle"] = duty

        if reference_value is not None:
            values["wReferenceValue"] = self._validate_finite(
                "reference_value", reference_value
            )

        if tracking_var is not None:
            values["wTrackingVar"] = self._normalize_channel(tracking_var)

        if phase_shift is not None:
            values["wPhaseShift"] = self._validate_integer(
                "phase_shift", phase_shift, -360, 360
            )

        if dead_time_rising_ns is not None:
            values["wDeadTimeRising_ns"] = self._validate_integer(
                "dead_time_rising_ns", dead_time_rising_ns, 0, 65535
            )
        if dead_time_falling_ns is not None:
            values["wDeadTimeFalling_ns"] = self._validate_integer(
                "dead_time_falling_ns", dead_time_falling_ns, 0, 65535
            )

        if not values:
            raise ValueError("at least one leg setting must be supplied")

        return self._write_and_verify(
            f"Config/Leg{leg_number}", values, absolute_tolerance=5e-4
        )

    def read_leg(self, leg):
        """Read the complete configuration object for one leg."""

        leg_number = self._normalize_leg(leg)
        value = self.client.read(f"Config/Leg{leg_number}")
        if not isinstance(value, dict):
            raise PowerTestBenchError("leg readback is not a ThingSet object")
        return value

    def read_measurements(self):
        """Read all live measurements."""

        value = self.client.read("Measurements")
        if not isinstance(value, dict):
            raise PowerTestBenchError("measurement readback is not a ThingSet object")
        return value

    def read_calibration(self, channel):
        """Read gain, offset, and store state for one sensor channel."""

        normalized = self._normalize_channel(channel)
        value = self.client.read(f"Calibration/{normalized}")
        if not isinstance(value, dict):
            raise PowerTestBenchError("calibration readback is not a ThingSet object")
        return value

    def set_calibration(self, channel, *, gain=None, offset=None, store=False):
        """Set and verify one channel's calibration values.

        When ``store`` is true, firmware persists the values and resets
        ``wStore`` to false. The reset is verified; the firmware currently
        exposes no separate NVS status value.
        """

        normalized = self._normalize_channel(channel)
        values = {}
        if gain is not None:
            values["wGain"] = self._validate_finite("gain", gain)
        if offset is not None:
            values["wOffset"] = self._validate_finite("offset", offset)
        if not isinstance(store, bool):
            raise ValueError("store must be a bool")
        if store:
            values["wStore"] = True
        if not values:
            raise ValueError("gain, offset, or store=True must be supplied")

        expected = dict(values)
        if store:
            expected["wStore"] = False
        return self._write_and_verify(
            f"Calibration/{normalized}",
            values,
            absolute_tolerance=5e-7,
            expected=expected,
        )

    def read_metadata(self):
        """Return converter metadata with host-friendly field names."""

        value = self.client.read("Converter")
        if not isinstance(value, dict):
            raise PowerTestBenchError("metadata readback is not a ThingSet object")

        result = {}
        for public_name, (thingset_name, _) in self._METADATA_FIELDS.items():
            if thingset_name not in value:
                raise PowerTestBenchError(
                    f"Converter/{thingset_name} is missing from the readback"
                )
            result[public_name] = value[thingset_name]
        return result

    @staticmethod
    def _validate_metadata_value(name, value, maximum_length):
        if not isinstance(value, str):
            raise ValueError(f"{name} must be a string")
        if not value or len(value) > maximum_length:
            raise ValueError(
                f"{name} must contain 1 to {maximum_length} ASCII characters"
            )
        if any(ord(character) < 0x20 or ord(character) > 0x7E for character in value):
            raise ValueError(f"{name} must contain printable ASCII only")
        return value

    def set_metadata(
        self,
        *,
        board_name=None,
        board_version=None,
        serial_number=None,
        firmware_version=None,
    ):
        """Set one or more persistent converter identity fields."""

        supplied = {
            "board_name": board_name,
            "board_version": board_version,
            "serial_number": serial_number,
            "firmware_version": firmware_version,
        }
        values = {}
        for public_name, value in supplied.items():
            if value is None:
                continue
            thingset_name, maximum_length = self._METADATA_FIELDS[public_name]
            values[thingset_name] = self._validate_metadata_value(
                public_name, value, maximum_length
            )
        if not values:
            raise ValueError("at least one metadata field must be supplied")

        readback = self._write_and_verify("Converter", values)
        return {
            public_name: readback[thingset_name]
            for public_name, (thingset_name, _) in self._METADATA_FIELDS.items()
        }

    def power_on(
        self,
        leg,
        *,
        connect_driver=False,
        connect_capacitor=False,
        **leg_settings,
    ):
        """Safely configure and energize exactly one selected leg."""

        leg_number = self._normalize_leg(leg)
        self._validate_bool("connect_driver", connect_driver)
        self._validate_bool("connect_capacitor", connect_capacitor)
        forbidden = {"enable", "driver", "capacitor"} & set(leg_settings)
        if forbidden:
            names = ", ".join(sorted(forbidden))
            raise ValueError(f"power_on controls these settings directly: {names}")

        try:
            self.set_mode(TesterMode.POWER_OFF)
            self.configure_leg(1, enable=False)
            self.configure_leg(2, enable=False)
            self.configure_leg(
                leg_number,
                enable=True,
                driver=connect_driver,
                capacitor=connect_capacitor,
                **leg_settings,
            )
            self.set_mode(TesterMode.POWER_ON)
        except Exception as exc:
            try:
                self.shutdown()
            except Exception:
                pass
            if isinstance(exc, PowerTestBenchError):
                raise
            raise PowerTestBenchError(f"power-on sequence failed: {exc}") from exc

    def shutdown(self, *, disconnect_hardware=False):
        """Request POWER_OFF and disable both legs, attempting every step."""

        self._validate_bool("disconnect_hardware", disconnect_hardware)
        failures = []

        try:
            self.set_mode(TesterMode.POWER_OFF)
        except Exception as exc:
            failures.append(str(exc))

        for leg_number in (1, 2):
            settings = {"enable": False}
            if disconnect_hardware:
                settings.update(driver=False, capacitor=False)
            try:
                self.configure_leg(leg_number, **settings)
            except Exception as exc:
                failures.append(str(exc))

        if failures:
            raise PowerTestBenchError(
                "shutdown completed with errors: " + "; ".join(failures)
            )
