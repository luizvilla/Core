#!/usr/bin/env python3
## SPDX-License-Identifier: LGPL-2.1
## @file app_constants.py
## @brief Read application constants from the production header, so L1 limits follow the firmware.
"""The C-Block does not report its tuning; parse `src/app/control_step.h` instead of copying values."""
from pathlib import Path
import re

HEADER = Path(__file__).resolve().parents[2] / 'src/app/control_step.h'


def app_constant(name):
    """@brief Value of `app::<name>` as declared in control_step.h.
    @param name Constant name, e.g. `startup_current_max`.
    @return The value as a float; raises if the declaration is missing."""
    m = re.search(rf'\b{name}\s*=\s*([-+0-9.eE]+)[FU]?\s*;', HEADER.read_text())
    if not m:
        raise SystemExit(f'{HEADER}: app::{name} not found')
    return float(m.group(1))


CONTROL_PERIOD = app_constant('control_task_period') * 1e-6  ##< Critical task period, s (50 µs, ADR 0006).
GRID_VOLTAGE_STALE_TICKS = round(app_constant('grid_voltage_stale_time') / CONTROL_PERIOD)  ##< VAC staleness trip, ticks.
STARTUP_CURRENT_MAX = app_constant('startup_current_max')  ##< STARTUP total current clamp, A (ADR 0003).
BOOST_TRIP_CURRENT = app_constant('boost_trip_current')    ##< Leg current that trips the boost, A.
BURST_PERIOD = app_constant('boost_burst_period')          ##< Burst period, PWM periods.
