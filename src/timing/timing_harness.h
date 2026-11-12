/*
 * Copyright (c) 2026-present LAAS-CNRS
 * SPDX-License-Identifier: LGPL-2.1
 */

/**
 * @file timing_harness.h
 * @brief L0-TIM-01 and L0-TIM-02 harness: controlStep() sections (TIM-01, main thread, interrupts locked) and the
 * whole critical tick per stage and per event (TIM-02, in the critical task, ADC and PWM running), in g_timing_report (ADR 0005, tests/l0/timing/README.md).
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

/** @brief One critical tick with synthetic inputs (main.cpp criticalTickBody). */
using TimingTickHook = ControlOutputs (*)(const Measurements &meas, const OperatorCommands &cmd);

/** @brief What the L0-TIM-02 harness needs from main.cpp. */
struct TimingTim02Hooks {
    TimingTickHook tick = nullptr;       ///< The critical tick body.
    bool (*relay_closed)() = nullptr;    ///< The critical task's relay feedback.
    void (*stage_start)() = nullptr;     ///< What a bench key does at a stage start (re-arm the scope).
};

/** @brief Prepares L0-TIM-02 after timingHarnessRun() (L0-TIM-01); then start the critical task on timingTim02Tick. */
void timingTim02Begin(ControlState &state, const TimingTim02Hooks &hooks);
/** @brief Critical-task callback: one scenario tick through the real tick body, timed and its events recorded. */
void timingTim02Tick();
/** @brief True once the L0-TIM-02 scenario has run to its end. */
bool timingTim02Done();
/** @brief Marks the report DONE (after the outputs are stopped). */
void timingFinish();

#endif  // TIMING_HARNESS_H
