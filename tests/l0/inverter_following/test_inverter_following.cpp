/*
 *
 * Copyright (c) 2026 OWNTECH TECHNOLOGIES
 *
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU Lesser General Public License as published by
 *   the Free Software Foundation, either version 2.1 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU Lesser General Public License for more details.
 *
 *   You should have received a copy of the GNU Lesser General Public License
 *   along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 * SPDX-License-Identifier: LGPL-2.1
 */

/**
 * @file test_inverter_following.cpp
 * @brief L0-GFL: open-loop tests of the grid-following inverter controller (single_phase_inverter library).
 *
 * The library is jalinei/single_phase_inverter, branch main, as fetched by
 * PlatformIO for the USB environment. Test cases are defined in
 * docs/testing/test-procedure.md §6.3; how to run them is in the README next to
 * this file.
 *
 * | Test case | Behaviour |
 * |---|---|
 * | L0-GFL-01 | Init validation |
 * | L0-GFL-02 | PLL lock on a clean 230 V / 50 Hz grid |
 * | L0-GFL-03 | PLL tracking of a 50 → 51 Hz step and re-synchronization |
 * | L0-GFL-04 | Connection lifecycle and its timeouts |
 * | L0-GFL-05 | Grid quality gating |
 * | L0-GFL-06 | Fault latching and clearing |
 * | L0-GFL-07 | Duty formula |
 * | L0-GFL-10 | Synchronization with grid harmonics (bench levels, EN 50160 limits) |
 * | L0-GFL-13 | Synchronization with a DC offset on VAC |
 * | L0-GFL-16 | PWM start-up before relay closure (PWM_WAIT, PWM_SETTLE) |
 *
 * The grid is a synthetic sine fed sample by sample; nothing the controller
 * outputs feeds back into it. Relay feedback is driven directly through
 * setGridConnected(). step() plays the application's part in the PWM start-up
 * (library 1b4fd06): it acknowledges PWM with setPwmEnabled() whenever the
 * controller requests it, as an application that applies the duty and starts
 * PWM on the same tick would; L0-GFL-16 drives the acknowledgement by hand. Settings are the application's: 400 V bus, 230 V RMS,
 * 50 Hz, 100 µs control period.
 *
 * Limits come from the library's own documentation and constants:
 * - synchronization, on one-period averages (luizvilla/Core#7): Vd within ±10 % of
 *   the nominal peak, |Vq| < 1 % of the nominal peak, |w - w_ref| < 5 % of w_ref,
 *   for 1000 consecutive calls;
 * - connection and disconnection timeouts: 1 s;
 * - duty = alpha_output / (2 V_bus) + 0.5, unclamped;
 * - PWM start-up: PWM at exactly 0.5 until acknowledged, waveform linked within
 *   ±0.01 of 0.5, then 10 grid periods of settling before the relay is requested.
 *
 * Durations the library does not state (lock time, tracking time, phase error)
 * are reported as INFO. The simulation horizons (5 s, 3 s) only bound the test;
 * they are not requirements.
 */

#include "singlePhaseInverterFollowing.h"
#include "check.h"

#include <cmath>
#include <limits>
#include <string>

namespace {

constexpr float Ts = 1e-4F;                       ///< Control period (s), as in main.cpp.
constexpr float V_bus = 400.0F;                   ///< DC bus voltage (V), as in main.cpp.
constexpr float V_pk = 230.0F * 1.41421356F;      ///< Nominal grid peak voltage (V).
constexpr double two_pi = 6.283185307179586;      ///< 2π in double precision.
constexpr float w0 = static_cast<float>(two_pi * 50.0);  ///< Nominal angular frequency (rad/s).
constexpr int sync_min_delay = 1000;              ///< Library _sync_min_delay (calls).
constexpr int ticks_per_s = 10000;                ///< Control ticks per second.
const float f_nan = std::numeric_limits<float>::quiet_NaN();  ///< Invalid-input stimulus.
const float f_inf = std::numeric_limits<float>::infinity();   ///< Invalid-input stimulus.

/** @brief Synthetic grid: v = amplitude · cos(phase), i = current · cos(phase). */
struct Grid {
    double phase = 1.0;     ///< Grid angle (rad); non-zero start so the PLL has to move.
    double hz = 50.0;       ///< Grid frequency (Hz).
    double amplitude = V_pk; ///< Voltage peak (V).
    double current = 0.0;   ///< Current peak (A), in phase with the voltage.

    /** @brief Instantaneous voltage sample (V). */
    float v() const { return static_cast<float>(amplitude * std::cos(phase)); }
    /** @brief Instantaneous current sample (A). */
    float i() const { return static_cast<float>(current * std::cos(phase)); }
    /** @brief Advances the grid by one control period. */
    void advance() { phase = std::fmod(phase + two_pi * hz * Ts, two_pi); }
};

/**
 * @brief Wraps an angle to [-π, π).
 * @param angle Angle in rad.
 * @return The equivalent angle in [-π, π).
 */
double wrap(double angle) {
    angle = std::fmod(angle + M_PI, two_pi);
    if (angle < 0) angle += two_pi;
    return angle - M_PI;
}

/**
 * @brief Runs one control period: acknowledges the PWM request, feeds the current grid sample, then advances the grid.
 * @param inv Controller under test.
 * @param grid Synthetic grid.
 * @param acknowledge Acknowledge PWM as the application would (default); false leaves the acknowledgement unchanged.
 * @return The duty returned by calculateDuty().
 */
float step(singlePhaseInverterFollowing &inv, Grid &grid, bool acknowledge = true) {
    // The application applied last period's PWM request: acknowledge it (L0-GFL-16 passes false to hold it back).
    if (acknowledge) inv.setPwmEnabled(inv.isPwmRequested());
    const float duty = inv.calculateDuty(grid.v(), grid.i());
    grid.advance();
    return duty;
}

/**
 * @brief Runs control periods until the controller reaches a mode.
 * @param inv Controller under test.
 * @param grid Synthetic grid.
 * @param mode Mode to wait for.
 * @param max_ticks Upper bound on the number of periods.
 * @return Number of periods run when the mode was reached, or -1.
 */
int runUntilMode(singlePhaseInverterFollowing &inv, Grid &grid, vsi_mode_t mode, int max_ticks) {
    for (int n = 1; n <= max_ticks; ++n) {
        step(inv, grid);
        if (inv.getMode() == mode) return n;
    }
    return -1;
}

/**
 * @brief Initializes the controller with the application's settings.
 * @param inv Controller under test.
 * @return true when init() returns 0.
 */
bool initDefault(singlePhaseInverterFollowing &inv) {
    return inv.init(V_bus, V_pk, w0, Ts) == 0;
}

/**
 * @brief Initializes a controller, requests connection and runs until GRID_CONNECT.
 * @param c Criterion that records the observations.
 * @param inv Fresh controller.
 * @param grid Synthetic grid.
 * @return true when GRID_CONNECT was reached within 5 s.
 */
bool reachConnect(tc::Criterion &c, singlePhaseInverterFollowing &inv, Grid &grid) {
    c.check("init", initDefault(inv));
    inv.requestConnect();
    return c.check("reaches GRID_CONNECT within 5 s",
                   runUntilMode(inv, grid, VSI_GRID_CONNECT, 5 * ticks_per_s) > 0);
}

/**
 * @brief Drives a fresh controller to RUN with the relay feedback closed and I_d ref = 1 A.
 * @param c Criterion that records the observations.
 * @param inv Fresh controller.
 * @param grid Synthetic grid.
 * @return true when RUN was reached.
 */
bool reachRun(tc::Criterion &c, singlePhaseInverterFollowing &inv, Grid &grid) {
    if (!reachConnect(c, inv, grid)) return false;
    inv.setGridConnected(true);
    step(inv, grid);
    inv.requestRun();
    inv.setIdqRef({1.0F, 0.0F, 0.0F});
    step(inv, grid);
    return c.check("reaches RUN", inv.getMode() == VSI_RUN);
}

}  // namespace

/**
 * @brief Runs every L0-GFL criterion.
 * @return 0 when all criteria pass, 1 otherwise (see tc::finish()).
 */
int main() {
    // ---------------------------------------------------------------- L0-GFL-01
    {
        /** @test L0-GFL-01.a: Valid init starts in OFF with no fault, no PWM and no relay request */
        tc::Criterion c("L0-GFL-01.a", "Valid init starts in OFF with no fault, no PWM and no relay request");
        singlePhaseInverterFollowing inv;
        c.check("init returns 0", initDefault(inv));
        c.check("mode OFF", inv.getMode() == VSI_OFF);
        c.check("fault NONE", inv.getFaultReason() == VSI_FAULT_NONE);
        c.check("no PWM request", !inv.isPwmRequested());
        c.check("no relay request", !inv.isGridConnectionRequested());
        c.near("estimated w = w0", inv.getw(), w0, 1e-3);
    }
    {
        /** @test L0-GFL-01.b: Invalid init parameters return -EINVAL and latch INVALID_CONFIGURATION */
        tc::Criterion c("L0-GFL-01.b", "Invalid init parameters return -EINVAL and latch INVALID_CONFIGURATION");
        struct Bad { const char *what; float vbus, vpk, w, ts; };
        const Bad bad[] = {
            {"V_bus = 0", 0.0F, V_pk, w0, Ts},       {"V_bus < 0", -1.0F, V_pk, w0, Ts},
            {"V_bus NaN", f_nan, V_pk, w0, Ts},        {"V_bus f_inf", f_inf, V_pk, w0, Ts},
            {"V_pk = 0", V_bus, 0.0F, w0, Ts},       {"w0 = 0", V_bus, V_pk, 0.0F, Ts},
            {"Ts = 0", V_bus, V_pk, w0, 0.0F},       {"w0*Ts >= pi", V_bus, V_pk, w0, 0.02F},
        };
        for (const Bad &b : bad) {
            singlePhaseInverterFollowing inv;
            c.check(b.what, inv.init(b.vbus, b.vpk, b.w, b.ts) == -EINVAL);
            c.check("mode FAULT", inv.getMode() == VSI_FAULT);
            c.check("reason INVALID_CONFIGURATION",
                    inv.getFaultReason() == VSI_FAULT_INVALID_CONFIGURATION);
            Grid grid;
            c.check("neutral duty", step(inv, grid) == 0.5F);
            c.check("no PWM request", !inv.isPwmRequested());
        }
    }
    {
        /** @test L0-GFL-01.c: A successful init after an invalid one clears the fault */
        tc::Criterion c("L0-GFL-01.c", "A successful init after an invalid one clears the fault");
        singlePhaseInverterFollowing inv;
        c.check("invalid init", inv.init(0.0F, V_pk, w0, Ts) == -EINVAL);
        c.check("valid init", initDefault(inv));
        c.check("mode OFF", inv.getMode() == VSI_OFF);
        c.check("fault NONE", inv.getFaultReason() == VSI_FAULT_NONE);
    }
    {
        /** @test L0-GFL-01.d: calculateDuty before init latches INVALID_CONFIGURATION with neutral duty */
        tc::Criterion c("L0-GFL-01.d", "calculateDuty before init latches INVALID_CONFIGURATION with neutral duty");
        singlePhaseInverterFollowing inv;
        Grid grid;
        c.check("neutral duty", step(inv, grid) == 0.5F);
        c.check("mode FAULT", inv.getMode() == VSI_FAULT);
        c.check("reason INVALID_CONFIGURATION", inv.getFaultReason() == VSI_FAULT_INVALID_CONFIGURATION);
    }

    // ---------------------------------------------------------------- L0-GFL-02
    {
        singlePhaseInverterFollowing inv;
        Grid grid;
        initDefault(inv);
        inv.requestSync();
        int lock_tick = -1;
        bool requested_hardware = false;
        bool left_sync = false;
        for (int n = 1; n <= 5 * ticks_per_s; ++n) {
            step(inv, grid);
            requested_hardware |= inv.isPwmRequested() || inv.isGridConnectionRequested();
            left_sync |= n > 1 && inv.getMode() != VSI_VOLTAGE_SYNC;
            if (lock_tick < 0 && inv.getSync()) lock_tick = n;
        }
        {
            /** @test L0-GFL-02.a: PLL synchronizes on a clean 230 V / 50 Hz grid (5 s horizon) */
            tc::Criterion c("L0-GFL-02.a", "PLL synchronizes on a clean 230 V / 50 Hz grid (5 s horizon)");
            c.check("getSync() true", lock_tick > 0);
            c.check("still synchronized at end", inv.getSync());
            c.near("frequency estimate (Hz)", inv.getw() / two_pi, 50.0, 2.5);  // ±5 % of w_ref
            c.range("Vd within ±10 % of peak (V)", inv.getVdq().d, 0.9 * V_pk, 1.1 * V_pk);
            c.range("|Vq| < 0.1 V", inv.getVdq().q, -0.1, 0.1);
        }
        {
            /** @test L0-GFL-02.b: Synchronization only never requests PWM or relay closure */
            tc::Criterion c("L0-GFL-02.b", "Synchronization only never requests PWM or relay closure");
            c.check("no PWM or relay request at any tick", !requested_hardware);
            c.check("mode stays VOLTAGE_SYNC", !left_sync);
        }
        /** @test L0-GFL-02.c: Time to getSync() true from a 1 rad phase offset (includes 1000-call dwell) */
        tc::info("L0-GFL-02.c", "Time to getSync() true from a 1 rad phase offset (includes 1000-call dwell)",
                 lock_tick > 0 ? lock_tick * Ts : -1.0, "s");
        /** @test L0-GFL-02.d: Steady-state phase error after 5 s */
        tc::info("L0-GFL-02.d", "Steady-state phase error after 5 s",
                 wrap(inv.getTheta() - (grid.phase - two_pi * grid.hz * Ts)) * 180.0 / M_PI, "deg");
        /** @test L0-GFL-02.e: Steady-state frequency error after 5 s */
        tc::info("L0-GFL-02.e", "Steady-state frequency error after 5 s", inv.getw() / two_pi - 50.0, "Hz");

        // ------------------------------------------------------------ L0-GFL-03
        // Off-nominal, the estimate carries a 100 Hz ripple, so frequency is
        // judged on a 20 ms moving mean (two ripple periods).
        grid.hz = 51.0;
        constexpr int window = 200;
        double ring[window] = {};
        double sum = 0.0;
        int last_outside = 0;
        int ticks_unsynced = 0;
        float vq_peak = 0.0F;
        for (int n = 1; n <= 3 * ticks_per_s; ++n) {
            step(inv, grid);
            const double f = inv.getw() / two_pi;
            sum += f - ring[n % window];
            ring[n % window] = f;
            if (n < window || std::fabs(sum / window - 51.0) > 0.05) last_outside = n;
            if (!inv.getSync()) ++ticks_unsynced;
            if (n > 2 * ticks_per_s) vq_peak = std::fmax(vq_peak, std::fabs(inv.getVdq().q));
        }
        {
            /** @test L0-GFL-03.a: PLL frequency estimate follows a 50 → 51 Hz step (3 s horizon) */
            tc::Criterion c("L0-GFL-03.a", "PLL frequency estimate follows a 50 → 51 Hz step (3 s horizon)");
            c.near("20 ms mean of the estimate (Hz)", sum / window, 51.0, 0.05 * 50.0);  // ±5 % of w_ref
        }
        {
            /** @test L0-GFL-03.b: Synchronized again at 51 Hz, inside the library's ±5 % frequency window (3 s horizon) */
            tc::Criterion c("L0-GFL-03.b", "Synchronized again at 51 Hz, inside the library's ±5 % frequency window (3 s horizon)");
            c.check("getSync() true at end", inv.getSync());
            c.range("|Vq| peak over the last 1 s (V)", vq_peak, 0.0, 0.1);  // _sync_voltage_tolerance
        }
        /** @test L0-GFL-03.c: Time until the 20 ms mean estimate stays within 51 ± 0.05 Hz */
        tc::info("L0-GFL-03.c", "Time until the 20 ms mean estimate stays within 51 ± 0.05 Hz",
                 last_outside * Ts, "s");
        /** @test L0-GFL-03.d: Time not synchronized after the step (of 3 s) */
        tc::info("L0-GFL-03.d", "Time not synchronized after the step (of 3 s)", ticks_unsynced * Ts, "s");
        /** @test L0-GFL-03.e: Steady-state error of the 20 ms mean estimate at 51 Hz */
        tc::info("L0-GFL-03.e", "Steady-state error of the 20 ms mean estimate at 51 Hz", sum / window - 51.0, "Hz");
        /** @test L0-GFL-03.f: Peak |Vq| ripple at 51 Hz (sync tolerance 0.1 V) */
        tc::info("L0-GFL-03.f", "Peak |Vq| ripple at 51 Hz (sync tolerance 0.1 V)", vq_peak, "V");
    }

    // ---------------------------------------------------------------- L0-GFL-04
    {
        /** @test L0-GFL-04.a: Lifecycle OFF → SYNC → PWM_WAIT → PWM_SETTLE → CONNECT → CONNECTED → RUN → CONNECTED → STOPPING → DISCONNECT → OFF */
        tc::Criterion c("L0-GFL-04.a", "Lifecycle OFF → SYNC → PWM_WAIT → PWM_SETTLE → CONNECT → CONNECTED → RUN → CONNECTED → STOPPING → DISCONNECT → OFF");
        singlePhaseInverterFollowing inv;
        Grid grid;
        c.check("init", initDefault(inv));
        inv.requestConnect();
        step(inv, grid);
        c.check("OFF → VOLTAGE_SYNC on request", inv.getMode() == VSI_VOLTAGE_SYNC);
        c.check("no PWM in VOLTAGE_SYNC", !inv.isPwmRequested());
        const int to_wait = runUntilMode(inv, grid, VSI_PWM_WAIT, 5 * ticks_per_s);
        c.check("VOLTAGE_SYNC → PWM_WAIT", to_wait > 0);
        c.check("not before the 1000-call dwell", to_wait >= sync_min_delay);
        c.check("PWM_WAIT: PWM, no relay", inv.isPwmRequested() && !inv.isGridConnectionRequested());
        c.check("PWM_WAIT → PWM_SETTLE", runUntilMode(inv, grid, VSI_PWM_SETTLE, ticks_per_s) > 0);
        c.check("PWM_SETTLE: PWM, no relay", inv.isPwmRequested() && !inv.isGridConnectionRequested());
        const int to_connect = runUntilMode(inv, grid, VSI_GRID_CONNECT, ticks_per_s);
        c.check("PWM_SETTLE → GRID_CONNECT", to_connect > 0);
        c.check("CONNECT requests relay and PWM", inv.isGridConnectionRequested() && inv.isPwmRequested());
        for (int n = 0; n < 100; ++n) step(inv, grid);
        c.check("waits in GRID_CONNECT without feedback", inv.getMode() == VSI_GRID_CONNECT);
        inv.setGridConnected(true);
        step(inv, grid);
        c.check("feedback → GRID_CONNECTED", inv.getMode() == VSI_GRID_CONNECTED);
        inv.setIdqRef({1.0F, 0.0F, 0.0F});
        inv.requestRun();
        step(inv, grid);
        c.check("requestRun → RUN", inv.getMode() == VSI_RUN);
        inv.requestStop();
        step(inv, grid);
        c.check("requestStop → GRID_CONNECTED", inv.getMode() == VSI_GRID_CONNECTED);
        grid.current = 2.0;  // current still flowing: must hold STOPPING
        for (int n = 0; n < 200; ++n) step(inv, grid);
        inv.requestDisconnect();
        step(inv, grid);
        c.check("requestDisconnect with current → STOPPING", inv.getMode() == VSI_STOPPING);
        c.check("STOPPING keeps PWM and relay", inv.isPwmRequested() && inv.isGridConnectionRequested());
        grid.current = 0.0;
        c.check("current decays → DISCONNECT", runUntilMode(inv, grid, VSI_DISCONNECT, ticks_per_s / 2) > 0);
        c.check("DISCONNECT: PWM off, relay open requested",
                !inv.isPwmRequested() && !inv.isGridConnectionRequested());
        inv.setGridConnected(false);
        step(inv, grid);
        c.check("open feedback → OFF", inv.getMode() == VSI_OFF);
        c.check("no fault on the way", inv.getFaultReason() == VSI_FAULT_NONE);
    }
    {
        /** @test L0-GFL-04.b: Relay never confirms closing: CONNECTION_TIMEOUT after 1 s */
        tc::Criterion c("L0-GFL-04.b", "Relay never confirms closing: CONNECTION_TIMEOUT after 1 s");
        singlePhaseInverterFollowing inv;
        Grid grid;
        if (reachConnect(c, inv, grid)) {
            const int ticks = runUntilMode(inv, grid, VSI_FAULT, 2 * ticks_per_s);
            c.near("time in GRID_CONNECT (s)", ticks * Ts, 1.0, 2 * Ts);
            c.check("reason CONNECTION_TIMEOUT", inv.getFaultReason() == VSI_FAULT_CONNECTION_TIMEOUT);
            c.check("PWM and relay requests dropped",
                    !inv.isPwmRequested() && !inv.isGridConnectionRequested());
        }
    }
    {
        /** @test L0-GFL-04.c: Relay never confirms opening: DISCONNECTION_TIMEOUT 1 s after the disconnect request */
        tc::Criterion c("L0-GFL-04.c", "Relay never confirms opening: DISCONNECTION_TIMEOUT 1 s after the disconnect request");
        singlePhaseInverterFollowing inv;
        Grid grid;
        if (reachConnect(c, inv, grid)) {
            inv.setGridConnected(true);
            step(inv, grid);
            inv.requestDisconnect();  // relay stays closed, no current: STOPPING then DISCONNECT
            const int ticks = runUntilMode(inv, grid, VSI_FAULT, 2 * ticks_per_s);
            c.near("time from disconnect request to fault (s)", ticks * Ts, 1.0, 2 * Ts);
            c.check("reason DISCONNECTION_TIMEOUT", inv.getFaultReason() == VSI_FAULT_DISCONNECTION_TIMEOUT);
        }
    }
    {
        /** @test L0-GFL-04.d: Relay feedback lost in RUN: GRID_CONNECTION_LOST, PWM dropped */
        tc::Criterion c("L0-GFL-04.d", "Relay feedback lost in RUN: GRID_CONNECTION_LOST, PWM dropped");
        singlePhaseInverterFollowing inv;
        Grid grid;
        if (reachRun(c, inv, grid)) {
            inv.setGridConnected(false);
            step(inv, grid);
            c.check("mode FAULT", inv.getMode() == VSI_FAULT);
            c.check("reason GRID_CONNECTION_LOST", inv.getFaultReason() == VSI_FAULT_GRID_CONNECTION_LOST);
            c.check("no PWM request", !inv.isPwmRequested());
        }
    }
    {
        /** @test L0-GFL-04.e: Grid phase jump while awaiting the relay: SYNC_LOST */
        tc::Criterion c("L0-GFL-04.e", "Grid phase jump while awaiting the relay: SYNC_LOST");
        singlePhaseInverterFollowing inv;
        Grid grid;
        if (reachConnect(c, inv, grid)) {
            grid.phase += M_PI / 2;
            const int ticks = runUntilMode(inv, grid, VSI_FAULT, ticks_per_s / 10);
            c.check("faults within 0.1 s", ticks > 0);
            c.check("reason SYNC_LOST", inv.getFaultReason() == VSI_FAULT_SYNC_LOST);
        }
    }

    // ---------------------------------------------------------------- L0-GFL-05
    {
        /** @test L0-GFL-05.a: Bad grid quality blocks synchronization; good quality restarts the 1000-call dwell */
        tc::Criterion c("L0-GFL-05.a", "Bad grid quality blocks synchronization; good quality restarts the 1000-call dwell");
        singlePhaseInverterFollowing inv;
        Grid grid;
        c.check("init", initDefault(inv));
        inv.requestConnect();
        bool synced = false, pwm = false;
        for (int n = 0; n < 2 * ticks_per_s; ++n) {
            inv.setGridQuality(false);
            step(inv, grid);
            synced |= inv.getSync();
            pwm |= inv.isPwmRequested();
        }
        c.check("never synchronized while quality is bad", !synced);
        c.check("never requests PWM", !pwm);
        c.check("held in VOLTAGE_SYNC", inv.getMode() == VSI_VOLTAGE_SYNC);
        inv.setGridQuality(true);
        const int ticks = runUntilMode(inv, grid, VSI_GRID_CONNECT, 2 * ticks_per_s);
        c.check("connects once quality is good", ticks > 0);
        c.check("dwell restarted from zero", ticks >= sync_min_delay);
    }
    {
        /** @test L0-GFL-05.b: Bad grid quality in CONNECT, CONNECTED or RUN trips BAD_GRID_QUALITY at once */
        tc::Criterion c("L0-GFL-05.b", "Bad grid quality in CONNECT, CONNECTED or RUN trips BAD_GRID_QUALITY at once");
        for (vsi_mode_t target : {VSI_GRID_CONNECT, VSI_GRID_CONNECTED, VSI_RUN}) {
            singlePhaseInverterFollowing inv;
            Grid grid;
            if (target == VSI_RUN) {
                if (!reachRun(c, inv, grid)) continue;
            } else {
                if (!reachConnect(c, inv, grid)) continue;
                if (target == VSI_GRID_CONNECTED) {
                    inv.setGridConnected(true);
                    step(inv, grid);
                }
            }
            c.check("in target mode", inv.getMode() == target);
            inv.setGridQuality(false);
            c.check("FAULT before the next sample", inv.getMode() == VSI_FAULT);
            c.check("reason BAD_GRID_QUALITY", inv.getFaultReason() == VSI_FAULT_BAD_GRID_QUALITY);
            c.check("no PWM request", !inv.isPwmRequested());
        }
    }
    {
        /** @test L0-GFL-05.c: clearFault needs good grid quality and an open relay, then returns to OFF */
        tc::Criterion c("L0-GFL-05.c", "clearFault needs good grid quality and an open relay, then returns to OFF");
        singlePhaseInverterFollowing inv;
        Grid grid;
        if (reachRun(c, inv, grid)) {
            inv.setGridQuality(false);
            c.check("refused: quality bad", !inv.clearFault());
            inv.setGridQuality(true);
            c.check("refused: relay still closed", !inv.clearFault());
            inv.setGridConnected(false);
            c.check("accepted", inv.clearFault());
            c.check("mode OFF", inv.getMode() == VSI_OFF);
            c.check("reason NONE", inv.getFaultReason() == VSI_FAULT_NONE);
        }
    }

    // ---------------------------------------------------------------- L0-GFL-06
    {
        /** @test L0-GFL-06.a: Non-finite measurements latch INVALID_MEASUREMENT with neutral duty and no PWM */
        tc::Criterion c("L0-GFL-06.a", "Non-finite measurements latch INVALID_MEASUREMENT with neutral duty and no PWM");
        for (int which = 0; which < 2; ++which) {
            singlePhaseInverterFollowing inv;
            Grid grid;
            if (!reachRun(c, inv, grid)) continue;
            const float duty = which == 0 ? inv.calculateDuty(f_nan, 0.0F) : inv.calculateDuty(0.0F, f_inf);
            c.check(which == 0 ? "NaN voltage: FAULT" : "infinite current: FAULT", inv.getMode() == VSI_FAULT);
            c.check("reason INVALID_MEASUREMENT", inv.getFaultReason() == VSI_FAULT_INVALID_MEASUREMENT);
            c.check("neutral duty", duty == 0.5F);
            c.check("no PWM request", !inv.isPwmRequested());
        }
    }
    {
        /** @test L0-GFL-06.b: Non-positive bus voltage latches INVALID_BUS_VOLTAGE */
        tc::Criterion c("L0-GFL-06.b", "Non-positive bus voltage latches INVALID_BUS_VOLTAGE");
        singlePhaseInverterFollowing inv;
        Grid grid;
        if (reachRun(c, inv, grid)) {
            inv.setVBus(0.0F);
            c.check("neutral duty", step(inv, grid) == 0.5F);
            c.check("reason INVALID_BUS_VOLTAGE", inv.getFaultReason() == VSI_FAULT_INVALID_BUS_VOLTAGE);
            inv.setGridConnected(false);
            c.check("clearFault refused while V_bus <= 0", !inv.clearFault());
            inv.setVBus(V_bus);
            c.check("clearFault accepted once V_bus is valid", inv.clearFault());
        }
    }
    {
        /** @test L0-GFL-06.c: Non-finite current reference or invalid frequency reference latches INVALID_REFERENCE */
        tc::Criterion c("L0-GFL-06.c", "Non-finite current reference or invalid frequency reference latches INVALID_REFERENCE");
        for (int which = 0; which < 4; ++which) {
            singlePhaseInverterFollowing inv;
            c.check("init", initDefault(inv));
            if (which == 0) inv.setIdqRef({f_nan, 0.0F, 0.0F});
            if (which == 1) inv.setIdqRef({0.0F, f_inf, 0.0F});
            if (which == 2) inv.setWRef(0.0F);
            if (which == 3) inv.setWRef(4.0F * M_PI / Ts);  // w * Ts >= pi
            c.check("immediate FAULT", inv.getMode() == VSI_FAULT);
            c.check("reason INVALID_REFERENCE", inv.getFaultReason() == VSI_FAULT_INVALID_REFERENCE);
        }
    }
    {
        /** @test L0-GFL-06.d: A latched fault keeps its first cause and ignores requests until cleared */
        tc::Criterion c("L0-GFL-06.d", "A latched fault keeps its first cause and ignores requests until cleared");
        singlePhaseInverterFollowing inv;
        Grid grid;
        if (reachRun(c, inv, grid)) {
            inv.tripFault();
            c.check("tripFault() without cause → EXTERNAL", inv.getFaultReason() == VSI_FAULT_EXTERNAL);
            inv.tripFault(VSI_FAULT_PWM_UPDATE_FAILED);
            c.check("first cause kept", inv.getFaultReason() == VSI_FAULT_EXTERNAL);
            inv.requestSync();
            inv.requestConnect();
            inv.requestRun();
            bool neutral = true;
            for (int n = 0; n < 2000; ++n) neutral &= step(inv, grid) == 0.5F;
            c.check("stays FAULT despite requests", inv.getMode() == VSI_FAULT);
            c.check("neutral duty every tick", neutral);
            c.check("no PWM or relay request", !inv.isPwmRequested() && !inv.isGridConnectionRequested());
        }
    }

    // ---------------------------------------------------------------- L0-GFL-07
    {
        /** @test L0-GFL-07.a: Duty is exactly 0.5 in OFF and VOLTAGE_SYNC */
        tc::Criterion c("L0-GFL-07.a", "Duty is exactly 0.5 in OFF and VOLTAGE_SYNC");
        singlePhaseInverterFollowing inv;
        Grid grid;
        c.check("init", initDefault(inv));
        for (int n = 0; n < 100; ++n) c.check("OFF: 0.5", step(inv, grid) == 0.5F);
        inv.requestSync();
        for (int n = 0; n < 5000; ++n) c.check("VOLTAGE_SYNC: 0.5", step(inv, grid) == 0.5F);
    }
    {
        /** @test L0-GFL-07.b: Duty = alpha_output / (2 V_bus) + 0.5 every sample, unclamped */
        tc::Criterion c("L0-GFL-07.b", "Duty = alpha_output / (2 V_bus) + 0.5 every sample, unclamped");
        singlePhaseInverterFollowing inv;
        Grid grid;
        if (reachConnect(c, inv, grid)) {
            float max_duty = 0.0F;
            for (float vbus : {400.0F, 300.0F, 100.0F}) {
                inv.setVBus(vbus);
                for (int n = 0; n < 200; ++n) {
                    const float duty = step(inv, grid);
                    c.near("duty vs formula", duty, inv.getVabOutput().alpha / (2.0F * vbus) + 0.5F, 1e-6);
                    if (vbus == 100.0F && duty > max_duty) max_duty = duty;
                }
            }
            c.check("not clamped: exceeds 1 when V_bus < grid peak", max_duty > 1.0F);
        }
    }

    // ---------------------------------------------------------------- L0-GFL-08
    // Cold start (fresh init, PLL at w_ref) on a grid anywhere inside both the
    // observer window (47.5–52.5 Hz) and the library's ±5 % window, from four
    // initial grid angles. 47.6 and 52.4 Hz stay strictly inside both windows.
    {
        const double freqs[] = {47.6, 49.0, 50.0, 51.0, 52.4};
        const char *letters08[] = {"a", "b", "c", "d", "e"};
        const double phases[] = {0.0, 1.0, 2.5, 4.0};
        double worst_lock_s = 0.0;
        double worst_transient_hz = 0.0;
        for (int k = 0; k < 5; ++k) {
            /** @test L0-GFL-08.a–e: cold-start synchronization at 47.6, 49, 50, 51 and 52.4 Hz from four grid angles (5 s horizon); settled frequency estimate (last 1 s) within the library's ±5 % */
            tc::Criterion c(std::string("L0-GFL-08.") + letters08[k],
                            "Cold-start synchronization at " + tc::num(freqs[k]) +
                                " Hz from four grid angles (5 s horizon); settled estimate within ±5 %");
            for (double phase0 : phases) {
                singlePhaseInverterFollowing inv;
                Grid grid;
                grid.hz = freqs[k];
                grid.phase = phase0;
                initDefault(inv);
                inv.requestSync();
                constexpr int window = 200;
                double ring[window] = {};
                double sum = 0.0;
                double settled_dev = 0.0;
                int lock_tick = -1;
                for (int n = 1; n <= 5 * ticks_per_s; ++n) {
                    step(inv, grid);
                    const double f = inv.getw() / two_pi;
                    sum += f - ring[n % window];
                    ring[n % window] = f;
                    if (n >= window) {
                        const double dev = std::fabs(sum / window - freqs[k]);
                        worst_transient_hz = std::fmax(worst_transient_hz, dev);
                        if (n > 4 * ticks_per_s) settled_dev = std::fmax(settled_dev, dev);
                    }
                    if (lock_tick < 0 && inv.getSync()) lock_tick = n;
                }
                c.check("getSync() within 5 s", lock_tick > 0);
                c.check("still synchronized at 5 s", inv.getSync());
                c.range("settled 20 ms mean estimate error over the last 1 s (Hz)", settled_dev, 0.0, 0.05 * 50.0);
                if (lock_tick > 0) worst_lock_s = std::fmax(worst_lock_s, lock_tick * Ts);
            }
        }
        /** @test L0-GFL-08.f: worst cold-start time to synchronization across L0-GFL-08.a–e (INFO) */
        tc::info("L0-GFL-08.f", "Worst cold-start time to synchronization over the cases that locked", worst_lock_s, "s");
        /** @test L0-GFL-08.g: worst transient deviation of the 20 ms mean frequency estimate during cold start (INFO) */
        tc::info("L0-GFL-08.g", "Worst transient error of the 20 ms mean frequency estimate during cold start", worst_transient_hz, "Hz");
    }

    // ---------------------------------------------------------------- L0-GFL-10, L0-GFL-13
    // Real-grid distortion (luizvilla/Core#7): synchronization judged on one-period averages.
    // Bench levels measured on 2026-11-09: 3rd 3.4 %, 5th 4.6 %, 7th 2 %, VAC offset -5.7 V.
    struct Distortion { double h3, h5, h7, h9, h11, h13, offset; };
    auto syncOn = [&](const Distortion &k, double hz, double &vq_inst, double &vq_avg, double &w_err) {
        singlePhaseInverterFollowing inv;
        if (!initDefault(inv)) return -1;
        inv.requestSync();
        double ph = 1.0;
        int synced = -1;
        vq_inst = vq_avg = w_err = 0.0;
        for (int n = 1; n <= 8 * ticks_per_s; ++n) {
            const double v = V_pk * (std::cos(ph) + k.h3 * std::cos(3 * ph) + k.h5 * std::cos(5 * ph) + k.h7 * std::cos(7 * ph) +
                                     k.h9 * std::cos(9 * ph) + k.h11 * std::cos(11 * ph) + k.h13 * std::cos(13 * ph)) + k.offset;
            inv.calculateDuty(static_cast<float>(v), 0.0F);
            ph = std::fmod(ph + two_pi * hz * Ts, two_pi);
            if (synced < 0 && inv.getSync()) synced = n;
            if (n > 6 * ticks_per_s) {
                vq_inst = std::fmax(vq_inst, std::fabs(inv.getVdq().q));
                vq_avg = std::fmax(vq_avg, std::fabs(inv.getVdqAverage().q));
                w_err = std::fmax(w_err, std::fabs(inv.getwAverage() / (two_pi * hz) - 1.0));
            }
        }
        return synced;
    };
    const double tol = 0.01 * V_pk;  // library tolerance on the averaged |Vq|
    {
        /** @test L0-GFL-10.a: Bench-measured harmonics: synchronizes within 8 s at 47.6-52.4 Hz, averaged |Vq| below 1 % of the peak */
        tc::Criterion c("L0-GFL-10.a", "Bench harmonics (3rd 3.4 %, 5th 4.6 %, 7th 2 %): synchronizes at 47.6-52.4 Hz, averaged |Vq| < 1 % of the peak");
        const Distortion bench{0.034, 0.046, 0.02, 0, 0, 0, 0};
        double worst_inst = 0, worst_avg = 0;
        for (double hz : {47.6, 49.5, 50.0, 50.5, 52.4}) {
            double vi, va, we;
            const int t = syncOn(bench, hz, vi, va, we);
            c.check(("synchronizes at " + std::to_string(hz).substr(0, 4) + " Hz").c_str(), t > 0);
            c.range("averaged |Vq| (V)", va, 0.0, tol);
            c.range("averaged w error vs the grid", we, 0.0, 0.01);
            worst_inst = std::fmax(worst_inst, vi);
            worst_avg = std::fmax(worst_avg, va);
        }
        /** @test L0-GFL-10.b: Bench harmonics: instantaneous |Vq| ripple, and its average (INFO) */
        tc::info("L0-GFL-10.b", "Bench harmonics: peak instantaneous |Vq| (the ripple the old criterion judged)", worst_inst, "V");
        tc::info("L0-GFL-10.c", "Bench harmonics: peak averaged |Vq|", worst_avg, "V");
    }
    {
        /** @test L0-GFL-10.d: EN 50160 limit harmonics (3rd 5 %, 5th 6 %, 7th 5 %, 9th 1.5 %, 11th 3.5 %, 13th 3 %) at 50 Hz: synchronizes */
        tc::Criterion c("L0-GFL-10.d", "EN 50160 limit harmonics (5 / 6 / 5 / 1.5 / 3.5 / 3 %) at 50 Hz: synchronizes, averaged |Vq| < 1 % of the peak");
        double vi, va, we;
        const int t = syncOn({0.05, 0.06, 0.05, 0.015, 0.035, 0.03, 0}, 50.0, vi, va, we);
        c.check("synchronizes", t > 0);
        c.range("averaged |Vq| (V)", va, 0.0, tol);
        /** @test L0-GFL-10.e: EN 50160 harmonics: instantaneous |Vq| ripple (INFO) */
        tc::info("L0-GFL-10.e", "EN 50160 harmonics: peak instantaneous |Vq|", vi, "V");
    }
    {
        /** @test L0-GFL-13.a: A DC offset of -5.7 V (bench) or +/-10 V on VAC: synchronizes, averaged |Vq| below 1 % of the peak */
        tc::Criterion c("L0-GFL-13.a", "VAC offset -5.7 V (bench) and +/-10 V: synchronizes, averaged |Vq| < 1 % of the peak");
        double worst = 0;
        for (double off : {-5.7, -10.0, 10.0}) {
            double vi, va, we;
            const int t = syncOn({0, 0, 0, 0, 0, 0, off}, 50.0, vi, va, we);
            c.check("synchronizes", t > 0);
            c.range("averaged |Vq| (V)", va, 0.0, tol);
            worst = std::fmax(worst, vi);
        }
        /** @test L0-GFL-13.b: Offset: instantaneous |Vq| ripple (INFO) */
        tc::info("L0-GFL-13.b", "VAC offset: peak instantaneous |Vq|", worst, "V");
    }
    {
        /** @test L0-GFL-13.c: The bench grid as measured (harmonics and -5.7 V offset): synchronizes */
        tc::Criterion c("L0-GFL-13.c", "Bench grid as measured (harmonics and -5.7 V offset) at 50 Hz: synchronizes");
        double vi, va, we;
        c.check("synchronizes", syncOn({0.034, 0.046, 0.02, 0, 0, 0, -5.7}, 50.0, vi, va, we) > 0);
        c.range("averaged |Vq| (V)", va, 0.0, tol);
    }

    // ---------------------------------------------------------------- L0-GFL-16
    // PWM start-up before relay closure (library 1b4fd06). Helper: a fresh controller
    // synchronized and asking to connect, stopped on entry to PWM_WAIT, PWM not acknowledged.
    auto toPwmWait = [&](tc::Criterion &c, singlePhaseInverterFollowing &inv, Grid &grid) {
        c.check("init", initDefault(inv));
        inv.requestConnect();
        for (int n = 0; n < 5 * ticks_per_s && inv.getMode() != VSI_PWM_WAIT; ++n) step(inv, grid, false);
        return c.check("reaches PWM_WAIT", inv.getMode() == VSI_PWM_WAIT);
    };
    {
        /** @test L0-GFL-16.a: PWM_WAIT requests PWM at exactly 0.5 with the relay open, and holds without acknowledgement */
        tc::Criterion c("L0-GFL-16.a", "PWM_WAIT requests PWM at exactly 0.5 with the relay open, and holds without acknowledgement");
        singlePhaseInverterFollowing inv;
        Grid grid;
        if (toPwmWait(c, inv, grid)) {
            bool neutral = true, relay = false;
            for (int n = 0; n < ticks_per_s; ++n) {
                neutral &= step(inv, grid, false) == 0.5F;
                relay |= inv.isGridConnectionRequested();
            }
            c.check("duty exactly 0.5 for 1 s", neutral);
            c.check("never requests the relay", !relay);
            c.check("still PWM_WAIT, PWM requested", inv.getMode() == VSI_PWM_WAIT && inv.isPwmRequested());
        }
    }
    {
        /** @test L0-GFL-16.b: Once acknowledged, the waveform is linked only within ±0.01 of neutral, near a zero crossing */
        tc::Criterion c("L0-GFL-16.b", "Once acknowledged, the waveform is linked only within ±0.01 of neutral, near a zero crossing");
        singlePhaseInverterFollowing inv;
        Grid grid;
        if (toPwmWait(c, inv, grid)) {
            c.near("default tolerance", inv.getPwmStartDutyTolerance(), 0.01, 1e-7);
            inv.setPwmEnabled(true);
            float duty = 0.5F, v = 0.0F;
            int n = 0;
            for (; n < ticks_per_s && inv.getMode() == VSI_PWM_WAIT; ++n) {
                v = grid.v();
                duty = step(inv, grid, false);
            }
            c.check("links within one grid period (20 ms)", inv.getMode() == VSI_PWM_SETTLE && n <= 201);
            c.range("first linked duty", duty, 0.49F, 0.51F);
            // The link is judged on the computed duty (±0.01 → ±8 V); the grid itself is off by the
            // PLL's steady phase error too (bounded at 2.5° in L0-GFL-02.a: 325 V · sin 2.5° ≈ 14 V).
            const float bound = 0.02F * V_bus + V_pk * std::sin(2.5F * static_cast<float>(M_PI) / 180.0F);
            c.range("grid voltage there (V): near a zero crossing", v, -bound, bound);
            float max_dev = 0.0F;
            for (int k = 0; k < 400; ++k) max_dev = std::fmax(max_dev, std::fabs(step(inv, grid, false) - 0.5F));
            c.range("then follows the grid-matching waveform (peak duty deviation)", max_dev, 0.35F, 0.45F);
        }
    }
    {
        /** @test L0-GFL-16.c: PWM_SETTLE lasts 10 grid periods with the relay open, then GRID_CONNECT requests it */
        tc::Criterion c("L0-GFL-16.c", "PWM_SETTLE lasts 10 grid periods with the relay open, then GRID_CONNECT requests it");
        singlePhaseInverterFollowing inv;
        Grid grid;
        if (toPwmWait(c, inv, grid)) {
            c.check("default 10 periods", inv.getPwmSettleCycles() == 10);
            inv.setPwmEnabled(true);
            while (inv.getMode() == VSI_PWM_WAIT) step(inv, grid, false);
            int ticks = 0;
            bool relay = false;
            while (inv.getMode() == VSI_PWM_SETTLE && ticks < ticks_per_s) {
                relay |= inv.isGridConnectionRequested();
                step(inv, grid, false);
                ++ticks;
            }
            c.check("relay never requested while settling", !relay);
            c.near("settling time (s): 10 periods of 50 Hz", ticks * Ts, 0.2, 2 * Ts);
            c.check("then GRID_CONNECT with the relay requested", inv.getMode() == VSI_GRID_CONNECT && inv.isGridConnectionRequested());
        }
    }
    {
        /** @test L0-GFL-16.d: Losing the PWM acknowledgement restarts the settling; in GRID_CONNECT it faults PWM_UPDATE_FAILED */
        tc::Criterion c("L0-GFL-16.d", "Losing the PWM acknowledgement restarts the settling; in GRID_CONNECT it faults PWM_UPDATE_FAILED");
        singlePhaseInverterFollowing inv;
        Grid grid;
        if (toPwmWait(c, inv, grid)) {
            inv.setPwmEnabled(true);
            while (inv.getMode() == VSI_PWM_WAIT) step(inv, grid, false);
            for (int n = 0; n < 1500; ++n) step(inv, grid, false);  // 150 ms of the 200 ms
            inv.setPwmEnabled(false);
            step(inv, grid, false);
            inv.setPwmEnabled(true);
            int ticks = 0;
            while (inv.getMode() == VSI_PWM_SETTLE && ticks < ticks_per_s) { step(inv, grid, false); ++ticks; }
            c.near("full 10 periods again after the loss (s)", ticks * Ts, 0.2, 2 * Ts);
            c.check("in GRID_CONNECT", inv.getMode() == VSI_GRID_CONNECT);
            inv.setPwmEnabled(false);
            step(inv, grid, false);
            c.check("loss in GRID_CONNECT → FAULT PWM_UPDATE_FAILED",
                    inv.getMode() == VSI_FAULT && inv.getFaultReason() == VSI_FAULT_PWM_UPDATE_FAILED);
            c.check("PWM and relay requests dropped", !inv.isPwmRequested() && !inv.isGridConnectionRequested());
        }
    }
    {
        /** @test L0-GFL-16.e: In PWM_WAIT and PWM_SETTLE, closed-contact feedback faults EXTERNAL, a phase jump SYNC_LOST, bad quality BAD_GRID_QUALITY */
        tc::Criterion c("L0-GFL-16.e", "In PWM_WAIT and PWM_SETTLE, closed-contact feedback faults EXTERNAL, a phase jump SYNC_LOST, bad quality BAD_GRID_QUALITY");
        for (vsi_mode_t target : {VSI_PWM_WAIT, VSI_PWM_SETTLE}) {
            for (int cause = 0; cause < 3; ++cause) {
                singlePhaseInverterFollowing inv;
                Grid grid;
                if (!toPwmWait(c, inv, grid)) continue;
                if (target == VSI_PWM_SETTLE) {
                    inv.setPwmEnabled(true);
                    while (inv.getMode() == VSI_PWM_WAIT) step(inv, grid, false);
                }
                c.check("in the target state", inv.getMode() == target);
                vsi_fault_reason_t expected = VSI_FAULT_EXTERNAL;
                if (cause == 0) {
                    inv.setGridConnected(true);
                    step(inv, grid, false);
                } else if (cause == 1) {
                    grid.phase += M_PI / 2;
                    expected = VSI_FAULT_SYNC_LOST;
                    for (int n = 0; n < ticks_per_s / 10 && inv.getMode() != VSI_FAULT; ++n) step(inv, grid, false);
                } else {
                    inv.setGridQuality(false);
                    expected = VSI_FAULT_BAD_GRID_QUALITY;
                }
                c.check("FAULT with the expected reason", inv.getMode() == VSI_FAULT && inv.getFaultReason() == expected);
                c.check("no PWM, no relay", !inv.isPwmRequested() && !inv.isGridConnectionRequested());
            }
        }
    }
    {
        /** @test L0-GFL-16.f: Invalid tolerances are rejected; zero settling periods holds PWM_SETTLE; a new count restarts the dwell */
        tc::Criterion c("L0-GFL-16.f", "Invalid tolerances are rejected; zero settling periods holds PWM_SETTLE; a new count restarts the dwell");
        singlePhaseInverterFollowing inv;
        Grid grid;
        for (float t : {0.0F, 0.5F, -0.1F, f_nan, f_inf})
            c.check("tolerance rejected", !inv.setPwmStartDutyTolerance(t));
        c.check("valid tolerance accepted", inv.setPwmStartDutyTolerance(0.02F));
        c.near("kept across init", (toPwmWait(c, inv, grid), inv.getPwmStartDutyTolerance()), 0.02, 1e-7);
        inv.setPwmSettleCycles(0);
        inv.setPwmEnabled(true);
        while (inv.getMode() == VSI_PWM_WAIT) step(inv, grid, false);
        for (int n = 0; n < ticks_per_s; ++n) step(inv, grid, false);
        c.check("zero periods: still PWM_SETTLE after 1 s, relay off", inv.getMode() == VSI_PWM_SETTLE && !inv.isGridConnectionRequested());
        inv.setPwmSettleCycles(5);
        int ticks = 0;
        while (inv.getMode() == VSI_PWM_SETTLE && ticks < ticks_per_s) { step(inv, grid, false); ++ticks; }
        c.near("5 periods counted from the change (s)", ticks * Ts, 0.1, 2 * Ts);
        inv.setPwmSettleCycles(10);
    }

    // ---------------------------------------------------------------- L0-GFL-17
    {
        /** @test L0-GFL-17.a: Current-loop settings default to the library's historical values */
        tc::Criterion c("L0-GFL-17.a", "Current-loop settings default to the library's historical values");
        singlePhaseInverterFollowing inv;
        c.check("init", initDefault(inv));
        c.near("Kp (V/A)", inv.getCurrentPiKp(), 0.001, 1e-9);
        c.near("Ti (s)", inv.getCurrentPiTi(), 0.0003, 1e-9);
        c.near("limit (V)", inv.getCurrentPiLimit(), 30.0, 1e-6);
        c.near("current SOGI gain (rad/s)", inv.getCurrentSogiGain(), 500.0, 1e-6);
        c.near("output advance (periods)", inv.getOutputPhaseAdvance(), 0.0, 0.0);
    }
    {
        /** @test L0-GFL-17.b: Invalid current-loop settings are rejected and leave the configuration unchanged */
        tc::Criterion c("L0-GFL-17.b", "Invalid current-loop settings are rejected and leave the configuration unchanged");
        singlePhaseInverterFollowing inv;
        c.check("init", initDefault(inv));
        for (float x : {0.0F, -1.0F, f_nan, f_inf}) {
            c.check("Kp rejected", !inv.setCurrentPiGains(x, 0.005F));
            c.check("Ti rejected", !inv.setCurrentPiGains(1.0F, x));
            c.check("limit rejected", !inv.setCurrentPiLimit(x));
            c.check("SOGI gain rejected", !inv.setCurrentSogiGain(x));
        }
        c.check("SOGI gain with Kr·Ts = 1 rejected", !inv.setCurrentSogiGain(1.0F / Ts));
        for (float x : {-0.1F, 2.1F, f_nan, f_inf})
            c.check("advance rejected", !inv.setOutputPhaseAdvance(x));
        c.near("Kp unchanged", inv.getCurrentPiKp(), 0.001, 1e-9);
        c.near("Ti unchanged", inv.getCurrentPiTi(), 0.0003, 1e-9);
        c.near("limit unchanged", inv.getCurrentPiLimit(), 30.0, 1e-6);
        c.near("SOGI gain unchanged", inv.getCurrentSogiGain(), 500.0, 1e-6);
        c.near("advance unchanged", inv.getOutputPhaseAdvance(), 0.0, 0.0);
    }
    {
        /** @test L0-GFL-17.c: Settings made before or after init() are kept across init() */
        tc::Criterion c("L0-GFL-17.c", "Settings made before or after init() are kept across init()");
        singlePhaseInverterFollowing inv;
        c.check("SOGI gain accepted before init", inv.setCurrentSogiGain(2000.0F));
        c.check("init", initDefault(inv));
        c.check("gains accepted", inv.setCurrentPiGains(1.0F, 0.005F));
        c.check("limit accepted", inv.setCurrentPiLimit(20.0F));
        c.check("advance accepted", inv.setOutputPhaseAdvance(0.5F));
        c.check("init again", initDefault(inv));
        c.near("Kp", inv.getCurrentPiKp(), 1.0, 1e-7);
        c.near("Ti", inv.getCurrentPiTi(), 0.005, 1e-9);
        c.near("limit", inv.getCurrentPiLimit(), 20.0, 1e-6);
        c.near("SOGI gain", inv.getCurrentSogiGain(), 2000.0, 1e-6);
        c.near("advance", inv.getOutputPhaseAdvance(), 0.5, 1e-7);
    }
    {
        /** @test L0-GFL-17.d: While the current loop runs, gains and advance change live; limit and SOGI gain are refused */
        tc::Criterion c("L0-GFL-17.d", "While the current loop runs, gains and advance change live; limit and SOGI gain are refused");
        singlePhaseInverterFollowing inv;
        Grid grid;
        if (reachRun(c, inv, grid)) {
            // I_d ref 1 A, measured 0: the PI correction is Kp · 1 A on the first step after the change.
            step(inv, grid);
            const float low = inv.getVdqOut().d - inv.getVdq().d;
            c.check("gains accepted in RUN", inv.setCurrentPiGains(5.0F, 0.005F));
            step(inv, grid);
            const float high = inv.getVdqOut().d - inv.getVdq().d;
            c.check("still RUN", inv.getMode() == VSI_RUN);
            c.check("default correction below 0.1 V", std::fabs(low) < 0.1F);
            c.check("Kp 5 V/A: correction above 4 V", high > 4.0F);
            c.check("limit refused in RUN", !inv.setCurrentPiLimit(10.0F));
            c.check("SOGI gain refused in RUN", !inv.setCurrentSogiGain(2000.0F));
            c.check("advance accepted in RUN", inv.setOutputPhaseAdvance(1.0F));
            c.near("limit unchanged", inv.getCurrentPiLimit(), 30.0, 1e-6);
            c.near("SOGI gain unchanged", inv.getCurrentSogiGain(), 500.0, 1e-6);
        }
    }
    {
        /** @test L0-GFL-17.e: The output advance rotates the output voltage by advance · w · Ts, magnitude unchanged */
        tc::Criterion c("L0-GFL-17.e", "The output advance rotates the output voltage by advance · w · Ts, magnitude unchanged");
        singlePhaseInverterFollowing a, b;
        Grid ga, gb;
        c.check("advance accepted", b.setOutputPhaseAdvance(1.0F));
        c.check("reach GRID_CONNECT (no advance)", reachConnect(c, a, ga));
        c.check("reach GRID_CONNECT (advance 1)", reachConnect(c, b, gb));
        const clarke_t va = a.getVabOutput(), vb = b.getVabOutput();
        // Each controller links its waveform at its own tick, so compare angles relative to each one's theta.
        const double angle = wrap((std::atan2(vb.beta, vb.alpha) - b.getTheta()) - (std::atan2(va.beta, va.alpha) - a.getTheta()));
        c.near("same magnitude (V)", std::hypot(vb.alpha, vb.beta), std::hypot(va.alpha, va.beta), 1e-2);
        c.near("rotated by w·Ts (rad)", angle, b.getw() * Ts, 1e-4);
    }

    return tc::finish();
}
