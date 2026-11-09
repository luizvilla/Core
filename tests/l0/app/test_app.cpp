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
 * | L0-APP-04 | Inverter PWM settles indefinitely with the relay open |
 * | L0-APP-06 | Bench step 0: open-loop 230 V RMS |
 * | L0-APP-07 | Bench steps 1-3: settle, connect through the relay, inject |
 * | L0-APP-08 | Bench current protection |
 * | L0-APP-09 | Bench-measured distorted grid: observer qualifies, step 1 reaches PWM_SETTLE |
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
#include <string>

namespace {

constexpr int ticks_per_s = 10000;                       ///< Control steps per second.
constexpr double two_pi = 6.283185307179586;             ///< 2π in double precision.
constexpr float v_bus = 400.0F;                          ///< Bus voltage fed to the step (V).

/** @brief Synthetic 230 V / 50 Hz grid sampled once per control period. */
struct Grid {
    double phase = 0.0;                                  ///< Grid angle (rad).
    double rms = 230.0;                                  ///< RMS voltage (V).
    double hz = 50.0;                                    ///< Frequency (Hz).
    double h3 = 0.0, h5 = 0.0, h7 = 0.0;                 ///< Harmonics, fraction of the fundamental.
    double offset = 0.0;                                 ///< Measurement offset (V).

    /** @brief Instantaneous voltage (V). */
    float v() const {
        return static_cast<float>(rms * std::sqrt(2.0) * (std::cos(phase) + h3 * std::cos(3 * phase) +
                                  h5 * std::cos(5 * phase) + h7 * std::cos(7 * phase)) + offset);
    }
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
    // Emulate successful hardware application, reported on the next control tick.
    state.bridge_pwm_applied = out.bridge_enable;
    grid.advance();
    return out;
}

/** @brief Hardware relay stand-in: the feedback follows the command after the contact time (as main.cpp). */
struct Relay {
    bool command = false;  ///< Last relay command.
    bool closed = false;   ///< Feedback reported to the control step.
    int ticks = 0;         ///< Ticks the command has been held.
    /** @brief Applies one period's outputs. @param out Control outputs. */
    void apply(const ControlOutputs &out) {
        if (out.relay_close != command) { command = out.relay_close; ticks = 0; }
        else if (ticks < static_cast<int>(app::relay_contact_ticks)) ++ticks;
        else closed = command;
    }
};

/**
 * @brief One bench period: measurements from the synthetic grid (or none), the relay stand-in,
 * an AC current sample and the bus; bridge PWM acknowledged as applied, as the hardware loop does.
 * @return The step's outputs.
 */
ControlOutputs benchStep(ControlState &state, Grid &grid, const OperatorCommands &cmd, Relay &relay,
                         float i_ac = 0.0F, float bus = v_bus, bool grid_present = true) {
    Measurements m;
    m.v_dc_bus = bus;
    m.v_ac = grid_present ? grid.v() : 0.0F;
    m.v_ac_updated = true;
    m.i_ac = i_ac;
    m.relay_closed = relay.closed;
    const ControlOutputs out = controlStep(state, m, cmd);
    state.bridge_pwm_applied = out.bridge_enable;
    relay.apply(out);
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
 * @return Bus at 390 V (below the 400 V reference), PV side 30 V.
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
        c.near("startup current ceiling (A)", state.boost.getStartupCurrentMax(), app::startup_current_max, 1e-6);
        c.check("burst period (PWM periods)", state.boost.isBurstEnabled() &&
                state.boost.getBurstPeriod() == app::boost_burst_period);
        state.boost.setVLow(36.0F);
        state.boost.setVBus(399.0F);
        state.boost.setMode(BUSREG);
        state.boost.coldLoop();
        c.near("voltage Kp scheduled (A/V)", state.boost.getVoltageKp(),
               app::boost_voltage_gain * app::initial_voltage_reference / 36.0F, 1e-4);
        state.boost.setMode(IDLE);
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
        Measurements m = boostMeasurements();
        // 0.2 V below the reference: with the scheduled Kp (about 5.8 A/V at 30 V,
        // ADR 0003) the voltage PID stays below its 11 A bound over the 20 cold loops.
        m.v_dc_bus = app::initial_voltage_reference - 0.2F;
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
        cmd.inverter_stage = INV_SETTLE;
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
        /** @test L0-APP-04.a: PWM_SETTLE holds with the relay open independently of boost mode */
        tc::Criterion c("L0-APP-04.a", "PWM_SETTLE holds with the relay open independently of boost mode");
        for (bool sync : {false, true}) {
            ControlState state;
            Grid grid;
            float held = 0.0F;
            OperatorCommands cmd;
            cmd.inverter_stage = sync ? INV_SETTLE : INV_OFF;
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
                    if (state.inverter.getMode() == VSI_PWM_WAIT)
                        c.near("PWM_WAIT starts at neutral", out.bridge_duty, 0.5F, 1e-6);
                }
            }
            c.check("bridge enabled only on request", bridge == sync);
            c.check("inverter PWM only on request", inverter_pwm == sync);
            c.check("inverter never requests the relay", !relay);
            c.check("ends in PWM_SETTLE or OFF", state.inverter.getMode() == (sync ? VSI_PWM_SETTLE : VSI_OFF));
            c.check(sync ? "with sync request: synchronization reached" : "without sync request: never synchronized",
                    synced == sync);
            cmd.inverter_stage = INV_OFF;
            const ControlOutputs stopped = gridStep(state, grid, cmd, true, held);
            c.check("cancel stops PWM immediately", !stopped.bridge_enable);
            c.check("cancel leaves relay open", !state.inverter.isGridConnectionRequested());
        }
    }
    {
        tc::Criterion c("L0-APP-04.b", "PWM handover waits for hardware acknowledgement");
        ControlState state;
        Grid grid;
        float held = 0.0F;
        OperatorCommands cmd;
        cmd.inverter_stage = INV_SETTLE;
        c.check("controlInit", controlInit(state) == 0);
        for (int n = 0; n < 3 * ticks_per_s; ++n) {
            state.bridge_pwm_applied = false;
            const ControlOutputs out = gridStep(state, grid, cmd, true, held);
            if (out.bridge_enable) c.near("unacknowledged PWM stays neutral", out.bridge_duty, 0.5F, 1e-6);
            c.check("relay remains open", !state.inverter.isGridConnectionRequested());
        }
        c.check("still waiting for PWM acknowledgement", state.inverter.getMode() == VSI_PWM_WAIT);
    }
    {
        tc::Criterion c("L0-APP-04.c", "Invalid live bus stops active inverter PWM");
        ControlState state;
        Grid grid;
        float held = 0.0F;
        OperatorCommands cmd;
        cmd.inverter_stage = INV_SETTLE;
        c.check("controlInit", controlInit(state) == 0);
        for (int n = 0; n < 3 * ticks_per_s; ++n) gridStep(state, grid, cmd, true, held);
        c.check("reached PWM_SETTLE", state.inverter.getMode() == VSI_PWM_SETTLE);
        Measurements m;
        m.v_ac = grid.v();
        m.v_ac_updated = true;
        m.v_dc_bus = 0.0F;
        const ControlOutputs out = controlStep(state, m, cmd);
        c.check("PWM disabled", !out.bridge_enable);
        c.check("fault latched", state.inverter.getMode() == VSI_FAULT);
        c.check("relay stays open", !state.inverter.isGridConnectionRequested());
    }

    // ---------------------------------------------------------------- L0-APP-05
    {
        ControlState state;
        Grid grid;
        float held = 0.0F;
        OperatorCommands cmd;
        cmd.inverter_stage = INV_SETTLE;
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
            // 0.1 s observer delay, + up to 0.02 s for the SOGI, + up to one period (0.02 s) because
            // the app feeds the observer one-period averages (#7).
            c.range("trip delay (s)", trip_tick / double(ticks_per_s), 0.1, 0.1 + 0.04);
        }
    }

    // ---------------------------------------------------------------- L0-APP-06 to 08
    // Inverter bench session (docs/testing/bench/inverter-session.md).
    {
        /** @test L0-APP-06.a: Step 0: open loop ramps the modulation index to 230 V RMS at 50 Hz, relay open, no grid needed */
        tc::Criterion c("L0-APP-06.a", "Step 0: open loop ramps to 230 V RMS at 50 Hz with the relay open, no grid needed");
        ControlState state;
        Grid grid;
        Relay relay;
        c.check("controlInit", controlInit(state) == 0);
        OperatorCommands cmd;
        cmd.inverter_stage = INV_OPEN_LOOP;
        const float target = app::open_loop_voltage_peak / v_bus;
        bool always_on = true, relay_ever = false;
        int ramp_ticks = -1, crossings = 0;
        float previous = 0.5F, peak = 0.0F;
        for (int n = 0; n < 2 * ticks_per_s; ++n) {
            const ControlOutputs out = benchStep(state, grid, cmd, relay, 0.0F, v_bus, false);
            always_on &= out.bridge_enable;
            relay_ever |= out.relay_close;
            if (ramp_ticks < 0 && std::fabs(state.open_loop_m - target) < 1e-6F) ramp_ticks = n + 1;
            if (n >= ticks_per_s) {  // after the ramp: one second of steady modulation
                crossings += (previous - 0.5F) * (out.bridge_duty - 0.5F) < 0.0F;
                peak = std::fmax(peak, std::fabs(out.bridge_duty - 0.5F));
            }
            previous = out.bridge_duty;
        }
        c.check("bridge enabled from the first period", always_on);
        c.check("relay never commanded", !relay_ever);
        c.near("ramp time to the 230 V RMS index (s)", ramp_ticks / static_cast<double>(ticks_per_s), target / app::open_loop_m_rate, 2e-4);
        c.near("duty peak deviation = m/2 (m = 325.3 V / 400 V)", peak, 0.5F * target, 1e-3);
        c.near("zero crossings in 1 s (50 Hz)", crossings, 100, 1);
    }
    {
        /** @test L0-APP-06.b: Step 0: a low bus saturates the index at 0.95, an invalid bus stops the bridge, leaving restarts the ramp */
        tc::Criterion c("L0-APP-06.b", "Step 0: low bus saturates the index at 0.95, invalid bus stops the bridge, leaving restarts the ramp");
        ControlState state;
        Grid grid;
        Relay relay;
        c.check("controlInit", controlInit(state) == 0);
        OperatorCommands cmd;
        cmd.inverter_stage = INV_OPEN_LOOP;
        for (int n = 0; n < 2 * ticks_per_s; ++n) benchStep(state, grid, cmd, relay, 0.0F, 300.0F, false);
        c.near("300 V bus: index saturates", state.open_loop_m, app::open_loop_m_max, 1e-6);
        c.check("0 V bus: bridge off", !benchStep(state, grid, cmd, relay, 0.0F, 0.0F, false).bridge_enable);
        cmd.inverter_stage = INV_OFF;
        c.check("OFF: bridge off at once", !benchStep(state, grid, cmd, relay, 0.0F, v_bus, false).bridge_enable);
        c.near("index reset", state.open_loop_m, 0.0, 1e-9);
        cmd.inverter_stage = INV_OPEN_LOOP;
        const ControlOutputs out = benchStep(state, grid, cmd, relay, 0.0F, v_bus, false);
        c.check("re-entry: bridge on, ramp from zero", out.bridge_enable && state.open_loop_m <= app::open_loop_m_rate * app::hot_loop_period + 1e-7F);
    }
    {
        /** @test L0-APP-07.a: Steps 1-2: CONNECT settles 10 grid periods with PWM and the relay open, then closes the relay and reaches GRID_CONNECTED */
        tc::Criterion c("L0-APP-07.a", "Steps 1-2: CONNECT settles 10 periods with PWM and the relay open, then closes the relay and reaches GRID_CONNECTED");
        ControlState state;
        Grid grid;
        Relay relay;
        c.check("controlInit", controlInit(state) == 0);
        OperatorCommands cmd;
        cmd.inverter_stage = INV_CONNECT;
        int settle_start = -1, relay_at = -1, connected_at = -1;
        bool relay_before_settle = false, pwm_off_while_settling = false;
        for (int n = 0; n < 4 * ticks_per_s && connected_at < 0; ++n) {
            const ControlOutputs out = benchStep(state, grid, cmd, relay);
            const vsi_mode_t mode = state.inverter.getMode();
            if (settle_start < 0 && mode == VSI_PWM_SETTLE) settle_start = n;
            if (mode == VSI_PWM_SETTLE && !out.bridge_enable) pwm_off_while_settling = true;
            if (out.relay_close && relay_at < 0) { relay_at = n; relay_before_settle = settle_start < 0; }
            if (mode == VSI_GRID_CONNECTED) connected_at = n;
        }
        c.check("PWM_SETTLE reached", settle_start >= 0);
        c.check("bridge PWM on while settling", !pwm_off_while_settling);
        c.check("relay not commanded before settling", relay_at >= 0 && !relay_before_settle);
        // 10 periods, then up to half a period more: the command waits for its zero-crossing instant (L0-APP-07.d).
        const double settle_s = (relay_at - settle_start) / static_cast<double>(ticks_per_s);
        c.check("settling time before the relay command: 0.2 s to 0.2 s + half a period", settle_s >= 0.2 - 5e-4 && settle_s <= 0.21 + 5e-4);
        c.near("relay feedback after the contact time (s)", (connected_at - relay_at) / static_cast<double>(ticks_per_s),
               app::relay_contact_ticks / static_cast<double>(ticks_per_s), 3e-4);
        c.check("GRID_CONNECTED with the relay closed", state.inverter.getMode() == VSI_GRID_CONNECTED && relay.closed);
    }
    {
        /** @test L0-APP-07.d: The relay command leads a grid-voltage zero crossing by app::relay_close_delay, so the contact closes at a zero crossing (several start phases, 49.5 and 50.5 Hz) */
        tc::Criterion c("L0-APP-07.d", "Relay command leads a voltage zero crossing by the relay closing time (start phases, 49.5/50.5 Hz)");
        for (double hz : {49.5, 50.0, 50.5}) {
            for (double start : {0.0, 1.0, 2.5, 4.0}) {
                ControlState state;
                Grid grid;
                grid.hz = hz;
                grid.phase = start;
                Relay relay;
                c.check("controlInit", controlInit(state) == 0);
                OperatorCommands cmd;
                cmd.inverter_stage = INV_CONNECT;
                double phase_at_cmd = -1.0;
                for (int n = 0; n < 4 * ticks_per_s && phase_at_cmd < 0.0; ++n) {
                    const double phase = grid.phase;
                    const ControlOutputs out = benchStep(state, grid, cmd, relay);
                    if (out.relay_close) phase_at_cmd = phase;
                }
                c.check("relay commanded", phase_at_cmd >= 0.0);
                // Grid::v() is a cosine: zero crossings at pi/2 + k*pi. Phase at the contact instant, folded onto them.
                const double at_contact = phase_at_cmd + two_pi * hz * app::relay_close_delay;
                double err = std::fmod(at_contact - two_pi / 4.0, two_pi / 2.0);
                if (err > two_pi / 4.0) err -= two_pi / 2.0;
                if (err < -two_pi / 4.0) err += two_pi / 2.0;
                c.near("contact instant from the nearest zero crossing (rad)", err, 0.0, 0.07);  // 4 deg, 2 ticks
            }
        }
    }
    {
        /** @test L0-APP-07.b: Step 3: INJECT runs the current loop with the reference capped at inject_current_max; back to CONNECT stops it */
        tc::Criterion c("L0-APP-07.b", "Step 3: INJECT runs with the reference capped at inject_current_max; back to CONNECT stops it");
        dqo_t out_capped{}, out_large{};
        for (int run = 0; run < 2; ++run) {
            ControlState state;
            Grid grid;
            Relay relay;
            c.check("controlInit", controlInit(state) == 0);
            OperatorCommands cmd;
            cmd.inverter_stage = INV_CONNECT;
            for (int n = 0; n < 4 * ticks_per_s && state.inverter.getMode() != VSI_GRID_CONNECTED; ++n) benchStep(state, grid, cmd, relay);
            cmd.inverter_stage = INV_INJECT;
            cmd.inject_current = run == 0 ? app::inject_current_max : 5.0F;
            benchStep(state, grid, cmd, relay);
            c.check("RUN", state.inverter.getMode() == VSI_RUN);
            (run == 0 ? out_capped : out_large) = state.inverter.getVdqOut();
            if (run == 1) {
                cmd.inverter_stage = INV_CONNECT;
                benchStep(state, grid, cmd, relay);
                c.check("back to CONNECT: GRID_CONNECTED, relay kept", state.inverter.getMode() == VSI_GRID_CONNECTED && relay.closed);
            }
        }
        c.near("5 A request gives the same output as the 0.1 A cap (d, V)", out_large.d, out_capped.d, 1e-6);
    }
    {
        /** @test L0-APP-07.c: OFF from the grid runs the disconnect sequence: relay opened, bridge off, inverter OFF; SETTLE from the grid reconnects through OFF */
        tc::Criterion c("L0-APP-07.c", "OFF from the grid opens the relay and stops the bridge; SETTLE from the grid goes through OFF with the relay open");
        for (InverterStage next : {INV_OFF, INV_SETTLE}) {
            ControlState state;
            Grid grid;
            Relay relay;
            c.check("controlInit", controlInit(state) == 0);
            OperatorCommands cmd;
            cmd.inverter_stage = INV_CONNECT;
            for (int n = 0; n < 4 * ticks_per_s && state.inverter.getMode() != VSI_GRID_CONNECTED; ++n) benchStep(state, grid, cmd, relay);
            cmd.inverter_stage = next;
            bool reached_off = false;
            for (int n = 0; n < ticks_per_s; ++n) {
                benchStep(state, grid, cmd, relay);
                reached_off |= state.inverter.getMode() == VSI_OFF;
            }
            c.check("relay opened", !relay.command && !relay.closed);
            c.check("passed through OFF", reached_off);
            if (next == INV_OFF) c.check("bridge off, inverter OFF", state.inverter.getMode() == VSI_OFF && !state.bridge_pwm_applied);
            else c.check("SETTLE again with the relay open", state.inverter.getMode() == VSI_PWM_SETTLE);
        }
    }
    {
        /** @test L0-APP-08.a: Bench current above 1.5 A trips the bridge in open loop and on the grid; OFF clears it */
        tc::Criterion c("L0-APP-08.a", "Bench current above 1.5 A trips the bridge (open loop and grid stages); OFF clears it");
        for (InverterStage stage : {INV_OPEN_LOOP, INV_SETTLE}) {
            ControlState state;
            Grid grid;
            Relay relay;
            c.check("controlInit", controlInit(state) == 0);
            OperatorCommands cmd;
            cmd.inverter_stage = stage;
            const bool grid_present = stage != INV_OPEN_LOOP;
            for (int n = 0; n < 3 * ticks_per_s && !state.bridge_pwm_applied; ++n) benchStep(state, grid, cmd, relay, 0.0F, v_bus, grid_present);
            c.check("bridge running", state.bridge_pwm_applied);
            benchStep(state, grid, cmd, relay, 1.4F, v_bus, grid_present);
            c.check("1.4 A: no trip", !state.bench_tripped);
            const ControlOutputs out = benchStep(state, grid, cmd, relay, 1.6F, v_bus, grid_present);
            c.check("1.6 A: tripped, bridge off, relay off", out.bench_trip && !out.bridge_enable && !out.relay_close);
            for (int n = 0; n < 100; ++n) benchStep(state, grid, cmd, relay, 0.0F, v_bus, grid_present);
            c.check("latched while the stage is kept", state.bench_tripped && !state.bridge_pwm_applied);
            cmd.inverter_stage = INV_OFF;
            benchStep(state, grid, cmd, relay, 0.0F, v_bus, grid_present);
            c.check("OFF clears the latch", !state.bench_tripped);
            if (grid_present) c.check("inverter fault cleared", state.inverter.getMode() != VSI_FAULT);
        }
    }

    {
        /** @test L0-APP-09.a: On the bench-measured grid (3rd 3.4 %, 5th 4.6 %, 7th 2 %, offset -5.7 V) the observer qualifies and step 1 reaches PWM_SETTLE */
        tc::Criterion c("L0-APP-09.a", "Bench grid (3rd 3.4 %, 5th 4.6 %, 7th 2 %, -5.7 V offset): observer qualifies, step 1 reaches PWM_SETTLE");
        ControlState state;
        Grid grid;
        grid.h3 = 0.034; grid.h5 = 0.046; grid.h7 = 0.02; grid.offset = -5.7;
        Relay relay;
        c.check("controlInit", controlInit(state) == 0);
        OperatorCommands cmd;
        cmd.inverter_stage = INV_SETTLE;
        int qualified = -1, settle = -1;
        for (int n = 1; n <= 5 * ticks_per_s && settle < 0; ++n) {
            benchStep(state, grid, cmd, relay);
            if (qualified < 0 && state.grid_observer.canOperate()) qualified = n;
            if (state.inverter.getMode() == VSI_PWM_SETTLE) settle = n;
        }
        c.check("observer qualifies", qualified > 0);
        c.check("PWM_SETTLE reached within 5 s", settle > 0);
        c.check("never tripped", !state.grid_observer.isTripped());
        /** @test L0-APP-09.b: Bench grid: time to observer qualification and to PWM_SETTLE (INFO) */
        tc::info("L0-APP-09.b", "Bench grid: time to observer qualification", qualified / double(ticks_per_s), "s");
        tc::info("L0-APP-09.c", "Bench grid: time to PWM_SETTLE", settle / double(ticks_per_s), "s");
    }
    {
        /** @test L0-APP-09.d: Bench grid, the bench sequence: step 1 (c) held 2 s in PWM_SETTLE, then step 2 (g): no fault, relay closed, GRID_CONNECTED */
        tc::Criterion c("L0-APP-09.d", "Bench grid: step 1 held 2 s, then step 2: no fault, relay closed, GRID_CONNECTED");
        ControlState state;
        Grid grid;
        grid.h3 = 0.034; grid.h5 = 0.046; grid.h7 = 0.02; grid.offset = -5.7;
        Relay relay;
        c.check("controlInit", controlInit(state) == 0);
        OperatorCommands cmd;
        cmd.inverter_stage = INV_SETTLE;
        int settle = -1;
        for (int n = 1; n <= 5 * ticks_per_s && settle < 0; ++n) {
            benchStep(state, grid, cmd, relay);
            if (state.inverter.getMode() == VSI_PWM_SETTLE) settle = n;
        }
        for (int n = 0; n < 2 * ticks_per_s; ++n) benchStep(state, grid, cmd, relay);
        c.check("PWM_SETTLE held in step 1", state.inverter.getMode() == VSI_PWM_SETTLE);
        cmd.inverter_stage = INV_CONNECT;
        int fault_tick = -1, connected = -1;
        for (int n = 1; n <= ticks_per_s && fault_tick < 0; ++n) {
            benchStep(state, grid, cmd, relay);
            if (state.inverter.getMode() == VSI_FAULT) fault_tick = n;
            if (connected < 0 && state.inverter.getMode() == VSI_GRID_CONNECTED) connected = n;
        }
        c.check("no fault after g", fault_tick < 0);
        if (fault_tick >= 0)
            tc::info("L0-APP-09.d", "fault after g, ticks / reason", fault_tick,
                     std::to_string(static_cast<int>(state.inverter.getFaultReason())).c_str());
        c.check("GRID_CONNECTED within 1 s", connected > 0);
    }

    return tc::finish();
}
