#
# Copyright (c) 2021-present LAAS-CNRS
#
#   This program is free software: you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation, either version 2 of the License, or
#   (at your option) any later version.
#
#   This program is distributed in the hope that it will be useful,
#   but WITHOUT ANY WARRANTY; without even the implied warranty of
#   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#   GNU General Public License for more details.
#
#   You should have received a copy of the GNU General Public License
#   along with this program.  If not, see <https://www.gnu.org/licenses/>.
#
# SPDX-License-Identifier: GPL-2.0-or-later
#
# @author Luiz Villa <luiz.villa@laas.fr>
#

"""
thingset_example.py

Usage example for thingset_tools.ThingSetTools: discovers the device's
ThingSet objects, auto-builds a {short_name: path} dict for the
Measurements group, reads a single measurement and the whole group, and
writes a Config value both via write() and via the attribute proxy.
"""

import re

from thingset_tools import ThingSetTools

MEAS = "Measurements"

ts = ThingSetTools()
ts.discover()

# Auto-build {short_name: full_path} for every measurement, e.g.
# "rV1Low_V" -> name "V1Low" (between the leading "r" and the "_V" unit).
measurements = {}
for name in ts.fetch_children(MEAS):
    match = re.match(r"^r(.+)_(\w+)$", name)
    if match:
        short_name, unit = match.groups()
        measurements[short_name] = f"{MEAS}/{name}"

print(measurements)

print(ts.read(measurements["V1Low"]))
print(ts.read(measurements["V2Low"]))
print(ts.read(measurements["VHigh"]))

# Flush all measurements and their current values at once.
print(ts.read(MEAS))

ts.write("Config", {"wBlinkPeriod_s": 0.5})

ts.objects.Config.wBlinkPeriod_s = 1.0
