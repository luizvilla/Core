#
# Copyright (c) 2021-present LAAS-CNRS
#
# SPDX-License-Identifier: GPL-2.0-or-later
#

import sys
import unittest
from pathlib import Path


TOOLS_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(TOOLS_DIR))

from thingset_tools import ThingSetTools  # noqa: E402


class ThingSetAccessClassificationTests(unittest.TestCase):
    def test_known_unprefixed_config_items_are_writable(self):
        self.assertEqual(
            ThingSetTools._classify_leaf("Mode", "Config/Mode"), "writable"
        )
        self.assertEqual(
            ThingSetTools._classify_leaf(
                "Frequency_Hz", "/Config/Frequency_Hz"
            ),
            "writable",
        )

    def test_prefix_convention_is_preserved(self):
        self.assertEqual(
            ThingSetTools._classify_leaf(
                "wBoardName", "Converter/wBoardName"
            ),
            "writable",
        )
        self.assertEqual(
            ThingSetTools._classify_leaf(
                "rV1Low_V", "Measurements/rV1Low_V"
            ),
            "read-only",
        )
        self.assertEqual(
            ThingSetTools._classify_leaf("cType", "Device/cType"),
            "informational",
        )


if __name__ == "__main__":
    unittest.main()
