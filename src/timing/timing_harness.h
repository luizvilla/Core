/*
 * Copyright (c) 2026-present LAAS-CNRS
 * SPDX-License-Identifier: LGPL-2.1
 */

/**
 * @file timing_harness.h
 * @brief L0-TIM-01 harness: runs controlStep() on the Spin with synthetic inputs and records the cycle cost
 * of each section in g_timing_report (ADR 0005, tests/l0/timing/README.md).
 *
 * Built only in the PlatformIO env TIMING (TIMING_HARNESS defined); main.cpp calls timingHarnessRun() after
 * controlInit() instead of starting the tasks, so no PWM is ever started.
 */
#ifndef TIMING_HARNESS_H
#define TIMING_HARNESS_H

#include "app/control_step.h"

/** @brief Called once per scenario tick after controlStep(): the critical task's scope sampling. */
using TimingScopeHook = void (*)(ControlState &state, const ControlOutputs &out, const Measurements &meas);

/**
 * @brief Runs the L0-TIM-01 scenario on an initialized control state and fills g_timing_report.
 * @param state Control state, already through controlInit().
 * @param scope_sample The critical task's scope snapshot and acquisition, timed as its own section.
 */
void timingHarnessRun(ControlState &state, TimingScopeHook scope_sample);

#endif  // TIMING_HARNESS_H
