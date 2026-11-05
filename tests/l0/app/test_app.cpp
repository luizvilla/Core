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
 * @file test_app.cpp
 * @brief L0-APP: open-loop tests of the application control step (src/app/control_step).
 *
 * Test cases are defined in docs/testing/test-procedure.md §6.4; how to run
 * them is in the README next to this file.
 *
 * | Test case | Behaviour |
 * |---|---|
 * | L0-APP-01 | A mode change stops both legs and runs the cold loop on the same tick |
 * | L0-APP-02 | The cold loop runs every 10th tick |
 * | L0-APP-03 | VAC stale for 20 ms invalidates the measurement and trips the observer |
 * | L0-APP-04 | The inverter bridge is never enabled |
 * | L0-APP-05 | Inverter synchronization is gated by the grid observer only |
 *
 * Each case builds its own ControlState with controlInit(), i.e. with the
 * application's real tuning (app:: constants), and calls controlStep() once
 * per 100 µs period. Measurements are fixed values or a synthetic 230 V /
 * 50 Hz grid; nothing the step outputs feeds back into them.
 *
 * Grid-related checks go through the real chain (PLL → grid observer →
 * inverter grid quality), so they run in real time: the observer needs 1 s
 * of healthy grid to qualify, i.e. 10 000 steps.
 */

#include "app/control_step.h"
#include "check.h"

#include <cmath>
#include <initializer_list>

namespace {

constexpr int ticks_per_s = 10000;                       ///< Control steps per second.
constexpr double two_pi = 6.283185307179586;             ///< 2π in double precision.
constexpr float v_bus = 400.0F;                          ///< Bus voltage fed to the step (V).

/** @brief Synthetic 230 V / 50 Hz grid sampled once per control period. */
struct Grid {
    double phase = 0.0;                                  ///< Grid angle (rad).
    double rms = 230.0;                                  ///< RMS voltage (V).
    double hz = 50.0;                                    ///< Frequency (Hz).

    /** @brief Instantaneous voltage (V). */
    float v() const { return static_cast<float>(rms * std::sqrt(2.0) * std::cos(phase)); }
    /** @brief Advances the grid by one control period. */
    void advance() { phase = std::fmod(phase + two_pi * hz * app::hot_loop_period, two_pi); }
};

/**
 * @brief Runs one control step on the synthetic grid, then advances the grid.
 * @param state Control state.
 * @param grid Synthetic grid.
 * @param cmd Operator commands.
 * @param vac_fresh Whether a new VAC sample arrived this period; if false the
 * last voltage is held, as the critical task does.
 * @param held_vac Last voltage passed to the step, updated when @p vac_fresh is true.
 * @return The step's outputs.
 */
ControlOutputs gridStep(ControlState &state, Grid &grid, const OperatorCommands &cmd,
                        bool vac_fresh, float &held_vac) {
    Measurements m;
    m.v_dc_bus = v_bus;
    if (vac_fresh) held_vac = grid.v();
    m.v_ac = held_vac;
    m.v_ac_updated = vac_fresh;
    const ControlOutputs out = controlStep(state, m, cmd);
    grid.advance();
    return out;
}

/**
 * @brief Brings a fresh state to a qualified observer on a healthy grid.
 * @param c Criterion that records the observations.
 * @param state Control state, initialized here.
 * @param grid Synthetic grid.
 * @param cmd Operator commands used during the run.
 * @param held_vac Last voltage passed to the step.
 * @return true when the observer allows operation within 3 s.
 */
bool qualifyGrid(tc::Criterion &c, ControlState &state, Grid &grid, const OperatorCommands &cmd,
                 float &held_vac) {
    c.check("controlInit", controlInit(state) == 0);
    for (int n = 0; n < 3 * ticks_per_s; ++n) {
        gridStep(state, grid, cmd, true, held_vac);
        if (state.grid_observer.canOperate()) return true;
    }
    return c.check("observer qualifies within 3 s", false);
}

/**
 * @brief Measurements for boost tests: bus below its reference, no grid.
 * @return Bus at 390 V (inside the burst band, below the 400 V reference), PV side 30 V.
 */
Measurements boostMeasurements() {
    Measurements m;
    m.v_dc_bus = 390.0F;
    m.v_low = 30.0F;
    return m;
}

}  // namespace

/**
 * @brief Runs every L0-APP criterion.
 * @return 0 when all criteria pass, 1 otherwise (see tc::finish()).
 */
int main() {
    // ---------------------------------------------------------------- L0-APP-01
    {
        /** @test L0-APP-01.a: controlInit applies the application's tuning */
        tc::Criterion c("L0-APP-01.a", "controlInit applies the application's tuning");
        ControlState state;
        c.check("returns 0", controlInit(state) == 0);
        c.check("boost starts in IDLE", state.boost.getMode() == IDLE);
        c.near("leg 1 current limit (A)", state.boost.getLeg1CurrentLimit(), app::leg1_current_limit, 1e-6);
        c.near("startup ramp rate (V/s)", state.boost.getStartupRampRate(), app::startup_ramp_rate, 1e-6);
        c.near("startup current ceiling (A)", state.boost.getStartupCurrentMax(), app::startup_current_max, 1e-6);
        c.near("burst resume (V)", state.boost.getVBusBurstMin(), app::v_dc_bus_burst_resume, 1e-6);
        c.near("burst pause (V)", state.boost.getVBusBurstMax(), app::v_dc_bus_burst_pause, 1e-6);
        c.check("observer not yet operable", !state.grid_observer.canOperate());
        c.check("inverter OFF", state.inverter.getMode() == VSI_OFF);
    }
    {
        /** @test L0-APP-01.b: A mode change is flagged on its tick only and runs the cold loop on that tick */
        tc::Criterion c("L0-APP-01.b", "A mode change is flagged on its tick only and runs the cold loop on that tick");
        ControlState state;
        controlInit(state);
        const Measurements m = boostMeasurements();
        OperatorCommands cmd;
        for (int n = 0; n < 25; ++n) {
            const ControlOutputs out = controlStep(state, m, cmd);
            c.check("IDLE: no mode change flagged", !out.boost_mode_changed);
            c.check("IDLE: boost PWM not updated", !out.boost_pwm_update);
        }
        for (boost_mode mode : {STARTUP, BUSREG, MPPT, IDLE}) {
            cmd.mode = mode;
            const float ref_before = state.boost.getILowRef();
            const ControlOutputs out = controlStep(state, m, cmd);
            c.check("mode change flagged", out.boost_mode_changed);
            c.check("boost switched to the requested mode", state.boost.getMode() == mode);
            c.check("PWM updated unless IDLE", out.boost_pwm_update == (mode != IDLE));
            if (mode != IDLE) {
                c.check("cold loop ran on the change tick (current reference set)",
                        state.boost.getILowRef() > 0.0F && state.boost.getILowRef() != ref_before);
            }
            for (int n = 0; n < 15; ++n) {
                c.check("not flagged on later ticks", !controlStep(state, m, cmd).boost_mode_changed);
            }
        }
    }
    {
        /** @test L0-APP-01.c: Leg enables follow the boost controller; IDLE leaves the legs untouched */
        tc::Criterion c("L0-APP-01.c", "Leg enables follow the boost controller; IDLE leaves the legs untouched");
        ControlState state;
        controlInit(state);
        const Measurements m = boostMeasurements();
        OperatorCommands cmd;
        cmd.mode = BUSREG;
        for (int n = 0; n < 50; ++n) {
            const ControlOutputs out = controlStep(state, m, cmd);
            c.check("leg 1 enable = controller", out.leg1_enable == state.boost.isLeg1Enabled());
            c.check("leg 2 enable = controller", out.leg2_enable == state.boost.isLeg2Enabled());
            c.check("duties = controller",
                    out.boost_duty.leg1 == state.boost.getDutyLeg1() &&
                    out.boost_duty.leg2 == state.boost.getDutyLeg2());
        }
        c.check("BUSREG below reference enables leg 1", state.boost.isLeg1Enabled());
        cmd.mode = IDLE;
        controlStep(state, m, cmd);
        for (int n = 0; n < 50; ++n) {
            const ControlOutputs out = controlStep(state, m, cmd);
            c.check("IDLE: no PWM update", !out.boost_pwm_update);
            c.check("IDLE: zero duties", out.boost_duty.leg1 == 0.0F && out.boost_duty.leg2 == 0.0F);
        }
    }

    // ---------------------------------------------------------------- L0-APP-02
    {
        /** @test L0-APP-02.a: The cold loop runs on the mode-change tick, then every 10th tick */
        tc::Criterion c("L0-APP-02.a", "The cold loop runs on the mode-change tick, then every 10th tick");
        ControlState state;
        controlInit(state);
        const Measurements m = boostMeasurements();
        OperatorCommands cmd;
        cmd.mode = BUSREG;
        // In BUSREG below the reference, the voltage PID output (total current
        // reference) changes on every cold loop and never on a hot loop.
        float previous = state.boost.getILowRef();
        int cold_loops = 0;
        for (int n = 0; n < 200; ++n) {
            controlStep(state, m, cmd);
            const float ref = state.boost.getILowRef();
            const bool changed = ref != previous;
            previous = ref;
            c.check("reference changes exactly on ticks 0, 10, 20, ...", changed == (n % app::cold_loop_divider == 0));
            cold_loops += changed;
        }
        c.check("20 cold loops in 200 ticks", cold_loops == 20);
        c.check("not saturated during the test (observation valid)", previous < app::boost_voltage_params.upper_bound);
    }
    {
        /** @test L0-APP-02.b: A new mode change restarts the cold-loop phase */
        tc::Criterion c("L0-APP-02.b", "A new mode change restarts the cold-loop phase");
        ControlState state;
        controlInit(state);
        const Measurements m = boostMeasurements();
        OperatorCommands cmd;
        cmd.mode = BUSREG;
        for (int n = 0; n < 4; ++n) controlStep(state, m, cmd);  // phase 3 of 10
        cmd.mode = STARTUP;
        controlStep(state, m, cmd);
        c.check("cold-loop counter restarted", state.cold_loop_tick == 0U);
        for (int n = 1; n < app::cold_loop_divider; ++n) {
            controlStep(state, m, cmd);
            c.check("no cold loop before 10 ticks", state.cold_loop_tick == n);
        }
        controlStep(state, m, cmd);
        c.check("cold loop on the 10th tick", state.cold_loop_tick == 0U);
    }

    // ---------------------------------------------------------------- L0-APP-03
    {
        /** @test L0-APP-03.a: VAC stale for 200 periods (20 ms) trips the observer with InvalidMeasurement */
        tc::Criterion c("L0-APP-03.a", "VAC stale for 200 periods (20 ms) trips the observer with InvalidMeasurement");
        ControlState state;
        Grid grid;
        float held = 0.0F;
        OperatorCommands cmd;
        if (qualifyGrid(c, state, grid, cmd, held)) {
            int trip_tick = -1;
            for (int n = 1; n <= 400 && trip_tick < 0; ++n) {
                gridStep(state, grid, cmd, false, held);
                if (state.grid_observer.isTripped()) trip_tick = n;
            }
            c.check("trips", trip_tick > 0);
            c.check("exactly on the 200th stale period", trip_tick == static_cast<int>(app::grid_voltage_stale_ticks));
            c.check("reason InvalidMeasurement",
                    state.grid_observer.getTripReason() == GridTripReason::InvalidMeasurement);
            c.check("observer no longer operable", !state.grid_observer.canOperate());
        }
    }
    {
        /** @test L0-APP-03.b: One fresh VAC sample restarts the 20 ms staleness count */
        tc::Criterion c("L0-APP-03.b", "One fresh VAC sample restarts the 20 ms staleness count");
        ControlState state;
        Grid grid;
        float held = 0.0F;
        OperatorCommands cmd;
        if (qualifyGrid(c, state, grid, cmd, held)) {
            for (int n = 0; n < 150; ++n) gridStep(state, grid, cmd, false, held);
            gridStep(state, grid, cmd, true, held);
            for (int n = 0; n < 150; ++n) gridStep(state, grid, cmd, false, held);
            c.check("no InvalidMeasurement trip after 150 + 1 + 150 periods",
                    state.grid_observer.getTripReason() != GridTripReason::InvalidMeasurement);
            c.check("staleness counter at 150", state.grid_voltage_age_ticks == 150U);
        }
    }
    {
        // Sources: gridObserver.cpp, "An unacceptable grid at startup inhibits
        // connection without latching a trip"; control_step.h, VAC is invalid
        // when "no new VAC sample arrives for 20 ms". One missing sample on the
        // very first control step (e.g. no ADC conversion ready yet at boot) is
        // neither.
        /** @test L0-APP-03.c: A VAC sample missing on the first control step does not latch a trip; the grid still qualifies and the inverter synchronizes */
        tc::Criterion c("L0-APP-03.c", "A VAC sample missing on the first control step does not latch a trip; grid qualifies and inverter synchronizes (3 s)");
        ControlState state;
        Grid grid;
        float held = 0.0F;
        OperatorCommands cmd;
        cmd.sync_requested = true;
        c.check("controlInit", controlInit(state) == 0);
        gridStep(state, grid, cmd, false, held);  // boot: no VAC sample yet
        bool qualified = false, synced = false;
        for (int n = 0; n < 3 * ticks_per_s; ++n) {
            gridStep(state, grid, cmd, true, held);
            qualified |= state.grid_observer.canOperate();
            synced |= state.inverter.getSync();
        }
        c.check("no trip latched", !state.grid_observer.isTripped());
        c.check("observer qualifies the grid", qualified);
        c.check("inverter synchronizes", synced);
    }

    // ---------------------------------------------------------------- L0-APP-04
    {
        /** @test L0-APP-04.a: The inverter bridge is never enabled, in any mode, with or without a sync request */
        tc::Criterion c("L0-APP-04.a", "The inverter bridge is never enabled, in any mode, with or without a sync request");
        for (bool sync : {false, true}) {
            ControlState state;
            Grid grid;
            float held = 0.0F;
            OperatorCommands cmd;
            cmd.sync_requested = sync;
            c.check("controlInit", controlInit(state) == 0);
            bool bridge = false, inverter_pwm = false, relay = false, synced = false;
            for (boost_mode mode : {IDLE, STARTUP, BUSREG, MPPT, IDLE}) {
                cmd.mode = mode;
                for (int n = 0; n < ticks_per_s / 2 * 3; ++n) {  // 1.5 s per mode
                    const ControlOutputs out = gridStep(state, grid, cmd, true, held);
                    bridge |= out.bridge_enable;
                    inverter_pwm |= state.inverter.isPwmRequested();
                    relay |= state.inverter.isGridConnectionRequested();
                    synced |= state.inverter.getSync();
                }
            }
            c.check("bridge_enable always false", !bridge);
            c.check("inverter never requests PWM", !inverter_pwm);
            c.check("inverter never requests the relay", !relay);
            c.check(sync ? "with sync request: synchronization reached" : "without sync request: never synchronized",
                    synced == sync);
        }
    }

    // ---------------------------------------------------------------- L0-APP-05
    {
        ControlState state;
        Grid grid;
        float held = 0.0F;
        OperatorCommands cmd;
        cmd.sync_requested = true;
        controlInit(state);
        int qualified_tick = -1, sync_tick = -1;
        bool sync_without_quality = false;
        for (int n = 1; n <= 3 * ticks_per_s; ++n) {
            gridStep(state, grid, cmd, true, held);
            if (qualified_tick < 0 && state.grid_observer.canOperate()) qualified_tick = n;
            if (sync_tick < 0 && state.inverter.getSync()) sync_tick = n;
            sync_without_quality |= state.inverter.getSync() && !state.grid_observer.canOperate();
        }
        {
            /** @test L0-APP-05.a: Synchronization waits for the observer's qualification, not only for PLL alignment */
            tc::Criterion c("L0-APP-05.a", "Synchronization waits for the observer's qualification, not only for PLL alignment");
            c.check("synchronized within 3 s", sync_tick > 0);
            c.check("never synchronized while the observer forbids operation", !sync_without_quality);
            c.check("sync after qualification", sync_tick > qualified_tick && qualified_tick > 0);
            c.check("sync at least 1000 calls after qualification", sync_tick - qualified_tick >= 1000);
        }
        /** @test L0-APP-05.b: Time from start to observer qualification (INFO) */
        tc::info("L0-APP-05.b", "Time from start to observer qualification", qualified_tick / double(ticks_per_s), "s");
        /** @test L0-APP-05.c: Time from start to inverter synchronization (INFO) */
        tc::info("L0-APP-05.c", "Time from start to inverter synchronization", sync_tick / double(ticks_per_s), "s");
        {
            /** @test L0-APP-05.d: An observer trip (200 V under-voltage) removes synchronization on the trip tick */
            tc::Criterion c("L0-APP-05.d", "An observer trip (200 V under-voltage) removes synchronization on the trip tick");
            grid.rms = 200.0;
            int trip_tick = -1;
            bool sync_after_trip = false;
            for (int n = 1; n <= ticks_per_s / 2; ++n) {
                gridStep(state, grid, cmd, true, held);
                if (trip_tick < 0 && state.grid_observer.isTripped()) trip_tick = n;
                if (trip_tick > 0) sync_after_trip |= state.inverter.getSync();
            }
            c.check("observer trips", trip_tick > 0);
            c.check("reason UnderVoltage", state.grid_observer.getTripReason() == GridTripReason::UnderVoltage);
            c.check("no synchronization from the trip tick on", !sync_after_trip);
            c.range("trip delay (s)", trip_tick / double(ticks_per_s), 0.1, 0.1 + 0.02);
        }
    }

    return tc::finish();
}
