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
 * @file test_boost.cpp
 * @brief L0-BST: open-loop tests of the boost converter controller (src/boost_converter).
 *
 * Test cases are defined in tests/catalogue.json (procedure §6.3); how to run
 * them is in the README next to this file.
 *
 * | Test case | Behaviour |
 * |---|---|
 * | L0-BST-01 | Invalid init or measurements never enable a leg |
 * | L0-BST-02 | Leg 1 priority up to its current limit, leg 2 takes the excess |
 * | L0-BST-03 | Cold-loop saturation sets the total current reference |
 * | L0-BST-04 | MPPT perturb and observe |
 * | L0-BST-05 | STARTUP current clamp; STARTUP to BUSREG resets nothing; IDLE resets all |
 * | L0-BST-06 | Current-driven burst on leg 1, PID state kept, independent of leg 2 |
 * | L0-BST-07 | Over-voltage full burst; over-voltage and over-current trip |
 * | L0-BST-08 | Voltage PID gain scheduled with VBusRef / VLow, without reset |
 * | L0-BST-09 | Leg 2 starts only above a demand threshold; below it, off and leg 1's burst kept |
 *
 * @warning The sections share one BoostConverter and run in order: each one
 * starts from the state the previous one left. Do not reorder them, and keep
 * every call that has side effects (hotLoop, coldLoop, set*) exactly where it
 * is, including calls inside a check expression.
 */

#include "boost.h"
#include "check.h"

#include <cmath>
#include <initializer_list>
#include <limits>
#include <string>

namespace {
/** @brief Absolute tolerance for floating-point comparisons (same as the original test). */
constexpr double tol = 1e-5;
}

/**
 * @brief Runs every L0-BST criterion in order.
 * @return 0 when all criteria pass, 1 otherwise (see tc::finish()).
 */
int main() {
    PidParams voltage = {0.001F, 0.1F, 1.0F, 0, 1, 0, 12};
    PidParams current = {0.0001F, 0.1F, 1.0F, 0, 1, 0, 0.95F};
    BoostConverter boost;
    BoostDutyCycles duty;

    {
        /** @test L0-BST-01.a: Uninitialized controller outputs nothing and enables no leg */
        tc::Criterion c("L0-BST-01.a", "Uninitialized controller outputs nothing and enables no leg");
        c.check("coldLoop() == 0", boost.coldLoop() == 0);
        c.check("hotLoop() duties == 0", boost.hotLoop().leg1 == 0 && boost.hotLoop().leg2 == 0);
        c.check("no leg enabled", !boost.isLeg1Enabled() && !boost.isLeg2Enabled());
        c.check("init(BUSREG) succeeds", boost.init(BUSREG, 48, voltage, current, 5) == 0);
        c.check("leg 1 current limit stored", boost.getLeg1CurrentLimit() == 5);
    }

    {
        /** @test L0-BST-02.a: Leg 1 alone below and at its current limit */
        tc::Criterion c("L0-BST-02.a", "Leg 1 alone below and at its current limit");
        for (float total : {2.0F, 5.0F}) {
            boost.setILowRef(total);
            c.near("ILow1 ref", boost.getILow1Ref(), total, tol);
            c.check("ILow2 ref == 0", boost.getILow2Ref() == 0);
            c.check("leg 1 on, leg 2 off", boost.isLeg1Enabled() && !boost.isLeg2Enabled());
            auto d = boost.hotLoop();
            c.check("leg 1 duty > 0, leg 2 duty == 0", d.leg1 > 0 && d.leg2 == 0);
        }
    }

    {
        /** @test L0-BST-02.b: Above the limit leg 1 stays at the limit and leg 2 supplies the excess */
        tc::Criterion c("L0-BST-02.b", "Above the limit leg 1 stays at the limit and leg 2 supplies the excess");
        boost.reset();
        boost.setILowRef(8);
        c.near("ILow1 ref", boost.getILow1Ref(), 5, tol);
        c.near("ILow2 ref", boost.getILow2Ref(), 3, tol);
        c.check("both legs on", boost.isLeg1Enabled() && boost.isLeg2Enabled());
        boost.setILow1(4);
        boost.setILow2(1);
        duty = boost.hotLoop();
        c.near("leg 1 PI duty", duty.leg1, 0.1F * (1 + current.Ts), tol);
        c.near("leg 2 PI duty", duty.leg2, 0.1F * (2 + 2 * current.Ts), tol);
        c.near("getDutyLeg1", boost.getDutyLeg1(), duty.leg1, tol);
        c.near("getDutyLeg2", boost.getDutyLeg2(), duty.leg2, tol);
        boost.setVLow(24);
        c.near("total input current", boost.getILowTotal(), 5, tol);
        c.near("input power", boost.getPower(), 120, tol);
    }

    {
        /** @test L0-BST-02.c: Removing leg 2 demand clears its output and integral before reuse */
        tc::Criterion c("L0-BST-02.c", "Removing leg 2 demand clears its output and integral before reuse");
        for (int i = 0; i < 100; ++i) boost.hotLoop();
        boost.setILowRef(5);
        c.check("leg 2 off, duty 0", !boost.isLeg2Enabled() && boost.getDutyLeg2() == 0);
        boost.setILowRef(8);
        duty = boost.hotLoop();
        c.near("leg 2 duty restarts from zero integral", duty.leg2, 0.1F * (2 + 2 * current.Ts), tol);
        boost.setILowRef(0);
        // ADR 0003: leg 1 runs in every active mode (burst skips its periods at light load); leg 2 stops.
        c.check("zero demand: leg 2 off, leg 1 left to burst", boost.isLeg1Enabled() && !boost.isLeg2Enabled());
        c.check("zero demand: leg 2 duty 0, leg 1's PID not reset (its duty is kept)",
                boost.getDutyLeg2() == 0 && boost.getDutyLeg1() == duty.leg1);
    }

    {
        /** @test L0-BST-03.a: Cold-loop saturation sets the total reference, split across both legs */
        tc::Criterion c("L0-BST-03.a", "Cold-loop saturation sets the total reference, split across both legs");
        boost.setMode(BUSREG);
        boost.setVBusRef(1000);
        c.near("saturated total ref", boost.coldLoop(), 12, tol);
        c.near("ILow1 ref", boost.getILow1Ref(), 5, tol);
        c.near("ILow2 ref", boost.getILow2Ref(), 7, tol);
        boost.setILow1(0);
        boost.setILow2(0);
        for (int i = 0; i < 10000; ++i) {
            duty = boost.hotLoop();
            c.range("leg 1 duty", duty.leg1, 0, 0.95F);
            c.range("leg 2 duty", duty.leg2, 0, 0.95F);
        }
    }

    {
        /** @test L0-BST-03.b: IDLE clears references and duties */
        tc::Criterion c("L0-BST-03.b", "IDLE clears references and duties");
        boost.setMode(IDLE);
        c.check("ILow ref 0, leg 2 off", boost.getILowRef() == 0 && !boost.isLeg2Enabled());
        duty = boost.hotLoop();
        c.check("duties 0", duty.leg1 == 0 && duty.leg2 == 0);
    }

    {
        /** @test L0-BST-04.a: MPPT perturbs current demand from total input power */
        tc::Criterion c("L0-BST-04.a", "MPPT perturbs current demand from total input power");
        boost.setMode(MPPT);
        boost.setVBus(48);
        boost.setILow1(2);
        boost.setILow2(1);
        c.near("first step +0.1 A", boost.coldLoop(), 0.1F, tol);
        boost.setILow2(2);  // Rising total power: continue increasing current.
        c.near("power rose: continue up", boost.coldLoop(), 0.2F, tol);
        boost.setILow2(1);  // Falling total power: reverse current perturbation.
        c.near("power fell: reverse", boost.coldLoop(), 0.1F, tol);
        c.near("total = leg 1 + leg 2", boost.getILowRef(), boost.getILow1Ref() + boost.getILow2Ref(), tol);
    }

    {
        /** @test L0-BST-04.b: MPPT restarts on reference changes, keeps leg 1 priority, turns back at bounds */
        tc::Criterion c("L0-BST-04.b", "MPPT restarts on reference changes, keeps leg 1 priority, turns back at bounds");
        boost.setILowRef(4.95F);
        c.near("step from new reference", boost.coldLoop(), 5.05F, tol);
        c.near("ILow1 ref at limit", boost.getILow1Ref(), 5, tol);
        c.near("ILow2 ref takes excess", boost.getILow2Ref(), 0.05F, tol);
        c.check("leg 2 on", boost.isLeg2Enabled());
        boost.setILowRef(11.95F);
        c.near("clamped at upper bound", boost.coldLoop(), 12, tol);
        c.near("turns back from upper bound", boost.coldLoop(), 11.9F, tol);
        boost.setILowRef(0);
        c.near("step up from zero", boost.coldLoop(), 0.1F, tol);
        boost.setILow2(0.5F);  // Power decrease directs the search down to zero.
        c.near("search down to zero", boost.coldLoop(), 0, tol);
        c.check("at zero: leg 2 off, leg 1 left to burst (ADR 0003)", boost.isLeg1Enabled() && !boost.isLeg2Enabled());
        c.near("turns back from lower bound", boost.coldLoop(), 0.1F, tol);
        boost.setILow2(1);
    }

    BoostConverter comparison;
    PidParams otherVoltage = voltage;
    {
        /** @test L0-BST-04.c: MPPT ignores bus voltages and voltage PID gains; hot loop does not advance it */
        tc::Criterion c("L0-BST-04.c", "MPPT ignores bus voltages and voltage PID gains; hot loop does not advance it");
        otherVoltage.Kp = 10;
        otherVoltage.Ti = 0.01F;
        c.check("comparison init", comparison.init(MPPT, 100, otherVoltage, current, 5) == 0);
        boost.setMode(MPPT);
        boost.setVBus(0);
        boost.setVBusRef(1000);
        comparison.setVBusRef(1);
        comparison.setVLow(boost.getVLow());
        comparison.setILow1(boost.getILow1());
        comparison.setILow2(boost.getILow2());
        boost.setILowRef(8);
        comparison.setILowRef(8);
        c.near("same cold-loop output", boost.coldLoop(), comparison.coldLoop(), tol);
        auto comparisonDuty = comparison.hotLoop();
        duty = boost.hotLoop();
        c.near("same leg 1 duty", duty.leg1, comparisonDuty.leg1, tol);
        c.near("same leg 2 duty", duty.leg2, comparisonDuty.leg2, tol);
        c.near("reference after one step", boost.getILowRef(), 8.1F, tol);
        for (int i = 0; i < 10; ++i) boost.hotLoop();
        c.near("hot loop does not advance MPPT", boost.getILowRef(), 8.1F, tol);
    }

    {
        /** @test L0-BST-04.d: Voltage PID current bounds also constrain MPPT */
        tc::Criterion c("L0-BST-04.d", "Voltage PID current bounds also constrain MPPT");
        otherVoltage.lower_bound = 3;
        c.check("init with lower bound 3", comparison.init(MPPT, 48, otherVoltage, current, 5) == 0);
        c.near("starts above lower bound", comparison.coldLoop(), 3.1F, tol);
        otherVoltage.upper_bound = 3;
        c.check("init with bounds 3..3", comparison.init(MPPT, 48, otherVoltage, current, 5) == 0);
        c.near("pinned at 3 A", comparison.coldLoop(), 3, tol);
        c.near("still pinned at 3 A", comparison.coldLoop(), 3, tol);
        comparison.setMode(IDLE);
        comparison.Mppt();
        c.near("Mppt() in IDLE leaves zero reference", comparison.getILowRef(), 0, tol);
    }

    // ------------------------------------------------------------------
    // L0-BST-05 to L0-BST-08: control adopted from Andres Vazquez Sieber's bench
    // controller (ADR 0003). They use their own controller with the
    // application's structure: rated current 11 A, leg 1 up to 5.5 A, STARTUP
    // clamp 0.25 A, duty 0.05-0.60, burst period 200, full burst above 440 V,
    // trip above 460 V or 8 A.
    // ------------------------------------------------------------------
    const PidParams v_app = {0.001F, 0.4333F * 400.0F / 36.0F, 4.57e-3F, 0, 0, 0, 11.0F};
    const PidParams i_app = {0.0001F, 0.002712F, 3.413e-4F, 0, 0, 0.05F, 0.60F};
    const uint16_t period = 200U;
    auto fresh = [&](BoostConverter &b) {
        bool ok = b.init(IDLE, 400, v_app, i_app, 5.5F) == 0;
        b.setStartupCurrentMax(0.25F);
        ok = ok && b.setBurst(period) == 0 && b.setProtection(440, 460, 8) == 0;
        b.setVLow(36);
        b.setVBusRef(400);
        return ok;
    };

    {
        tc::Criterion c("L0-BST-06.f", "Above-target voltage keeps increasing skipping at zero current, then recovers when voltage falls");
        BoostConverter b;
        c.check("configured", fresh(b));
        b.setMode(BUSREG);
        b.setILowRef(0.1F);
        for (int n = 0; n <= period; ++n) b.hotLoop();
        b.setVBus(410);
        b.setILow1(0.1F);
        b.coldLoop();
        b.hotLoop();
        c.check("enters burst at minimum duty", b.getBurstSkip() == 1);
        b.setILow1(0);
        for (int n = 0; n < period; ++n) {
            if (n % 10 == 0) b.coldLoop();
            b.hotLoop();
        }
        c.check("zero measured current still reaches full skipping", b.getBurstSkip() == period);
        c.check("physical current references stay zero", b.getILowRef() == 0 &&
                b.getILow1Ref() == 0 && b.getILow2Ref() == 0 && !b.isLeg2Enabled());
        b.setMode(MPPT);
        b.hotLoop();
        c.check("MPPT zero reference holds skipping instead of inheriting signed demand", b.getBurstSkip() == period);
        b.setILowRef(1);
        b.hotLoop();
        c.check("MPPT positive reference resumes pulses", b.getBurstSkip() == period - 1);
        b.setMode(BUSREG);
        b.coldLoop();
        b.hotLoop();
        c.check("voltage control suppresses pulses again", b.getBurstSkip() == period);
        b.setILowRef(1);
        b.hotLoop();
        c.check("manual demand reduces skipping", b.getBurstSkip() == period - 1);
        b.setILowRef(0);
        b.hotLoop();
        c.check("direct zero command holds partial skipping", b.getBurstSkip() == period - 1);
        b.setVBus(390);
        b.coldLoop();
        b.hotLoop();
        c.check("below target starts delivering again", b.getILowRef() > 0 && b.getBurstSkip() < period);
        b.setMode(MPPT);
        b.setILowRef(0);
        b.setILow1(0);
        const auto skip = b.getBurstSkip();
        b.hotLoop();
        c.check("MPPT does not inherit signed voltage demand", b.getBurstSkip() == skip);
    }
    {
        tc::Criterion c("L0-BST-06.g", "Active burst starts at minimum pulse duty; IDLE and trip clear duty");
        BoostConverter b;
        c.check("configured", fresh(b));
        c.near("IDLE duty zero", b.getDutyLeg1(), 0, tol);
        b.setMode(STARTUP);
        c.near("startup pulse duty at lower bound", b.hotLoop().leg1, i_app.lower_bound, tol);
        b.setMode(IDLE);
        c.near("IDLE clears duty", b.getDutyLeg1(), 0, tol);
        b.setMode(BUSREG);
        c.near("reentry pulse duty at lower bound", b.hotLoop().leg1, i_app.lower_bound, tol);
        b.setVBus(461);
        c.near("trip clears duty", b.hotLoop().leg1, 0, tol);
    }
    {
        tc::Criterion c("L0-BST-06.h", "Signed voltage demand lowers duty and enters burst even with zero measured current");
        BoostConverter b;
        c.check("configured", fresh(b));
        b.setMode(BUSREG);
        b.setILowRef(3);
        b.setILow1(0);
        for (int n = 0; n < 1000; ++n) b.hotLoop();
        c.check("continuous at elevated duty", b.getBurstSkip() == 0 && b.getDutyLeg1() > i_app.lower_bound);
        b.setVBus(410);
        for (int n = 0; n < 1000; ++n) {
            if (n % 10 == 0) b.coldLoop();
            b.hotLoop();
        }
        c.near("duty reduced to minimum", b.getDutyLeg1(), i_app.lower_bound, tol);
        c.check("all pulses suppressed", b.getBurstSkip() == period);
    }
    {
        /** @test L0-BST-05.a: STARTUP regulates toward the bus reference with the current clamped, leg 1 only */
        tc::Criterion c("L0-BST-05.a", "STARTUP regulates toward the bus reference with the current clamped, leg 1 only");
        BoostConverter b;
        c.check("configured", fresh(b));
        b.setVBus(0);
        b.setMode(STARTUP);
        c.near("total reference clamped to 0.25 A", b.coldLoop(), 0.25, tol);
        c.near("leg 1 takes it", b.getILow1Ref(), 0.25, tol);
        c.check("leg 2 never", b.getILow2Ref() == 0 && !b.isLeg2Enabled());
        b.setILowRef(3);
        c.near("manual reference clamped too", b.getILowRef(), 0.25, tol);
        b.setVBus(379.9F);
        c.range("below 95% the demand stays within the startup clamp", b.coldLoop(), 0, 0.25F);
        c.check("still in STARTUP", b.getMode() == STARTUP);
    }

    {
        tc::Criterion c("L0-BST-05.d", "STARTUP automatically enters BUSREG at 95% of a positive target without resetting state");
        for (float target : {200.0F, 400.0F}) {
            BoostConverter automatic, manual;
            for (auto* b : {&automatic, &manual}) {
                c.check("configured", fresh(*b));
                b->setVBusRef(target);
                b->setVBus(0.95F * target - 0.1F);
                b->setMode(STARTUP);
                b->coldLoop();
                for (int n = 0; n < 250; ++n) b->hotLoop();
                c.check("below threshold stays in STARTUP", b->getMode() == STARTUP);
                b->setVBus(0.95F * target);
            }
            const auto skip = automatic.getBurstSkip();
            const auto duty = automatic.getDutyLeg1();
            manual.setMode(BUSREG);
            c.near("voltage PID continues like an explicit transition", automatic.coldLoop(), manual.coldLoop(), tol);
            c.check("exact threshold enters BUSREG", automatic.getMode() == BUSREG);
            c.check("startup current clamp released", automatic.getILowRef() > automatic.getStartupCurrentMax());
            c.check("burst state preserved", automatic.getBurstSkip() == skip);
            c.near("duty preserved", automatic.getDutyLeg1(), duty, tol);
            const auto expected = manual.hotLoop();
            const auto actual = automatic.hotLoop();
            c.near("leg 1 PID continues", actual.leg1, expected.leg1, tol);
            c.near("leg 2 PID continues", actual.leg2, expected.leg2, tol);
            automatic.setVBus(0);
            automatic.coldLoop();
            c.check("voltage dip does not return to STARTUP", automatic.getMode() == BUSREG);
        }
        BoostConverter b;
        c.check("configured", fresh(b));
        b.setMode(STARTUP);
        b.setVBusRef(0);
        b.coldLoop();
        c.check("zero target does not complete startup", b.getMode() == STARTUP);
        b.setVBusRef(400);
        b.setVBus(381);
        b.coldLoop();
        c.check("already above threshold also enters BUSREG", b.getMode() == BUSREG);
        b.setMode(STARTUP);
        b.setVBus(461);
        b.hotLoop();
        b.coldLoop();
        c.check("tripped startup does not advance", b.isTripped() && b.getMode() == STARTUP);
    }

    {
        /** @test L0-BST-05.b: STARTUP to BUSREG resets nothing: PID states, references and burst carry over */
        tc::Criterion c("L0-BST-05.b", "STARTUP to BUSREG resets nothing: PID states, references and burst carry over");
        BoostConverter b;
        c.check("configured", fresh(b));
        b.setVBus(300);
        b.setMode(STARTUP);
        Pid mirror;  // leg 1's current PID, fed the same inputs: proves its state is never reset
        mirror.init(i_app);
        b.setILow1(0.1F);
        b.coldLoop();
        for (int i = 0; i < 400; ++i) {
            const bool continuous = b.getBurstSkip() == 0;
            const float32_t d = b.hotLoop().leg1;
            if (continuous) c.near("leg 1 duty follows its PID", d, mirror.calculateWithReturn(b.getILow1Ref(), 0.1F), 1e-6);
        }
        c.check("burst left by then", b.getBurstSkip() == 0);
        const float32_t ref = b.getILowRef();
        const float32_t duty = b.getDutyLeg1();
        b.setMode(BUSREG);
        c.near("total reference kept", b.getILowRef(), ref, tol);
        c.near("duty kept", b.getDutyLeg1(), duty, tol);
        c.check("burst not restarted", b.getBurstSkip() == 0);
        c.near("first BUSREG duty continues the same PID", b.hotLoop().leg1,
               mirror.calculateWithReturn(b.getILow1Ref(), 0.1F), 1e-6);
        c.range("BUSREG allows more than the startup clamp", b.coldLoop(), 0.25F, 11);
    }

    {
        /** @test L0-BST-05.c: IDLE, and an active mode entered from IDLE, reset every state; burst starts fully skipped */
        tc::Criterion c("L0-BST-05.c", "IDLE, and an active mode entered from IDLE, reset every state; burst starts fully skipped");
        BoostConverter b;
        c.check("configured", fresh(b));
        b.setVBus(300);
        b.setMode(BUSREG);
        b.coldLoop();
        for (int i = 0; i < 300; ++i) b.hotLoop();
        b.setMode(IDLE);
        c.check("IDLE: references, duties 0, legs off", b.getILowRef() == 0 && b.getDutyLeg1() == 0 &&
                b.getDutyLeg2() == 0 && !b.isLeg1Enabled() && !b.isLeg2Enabled());
        b.setMode(BUSREG);
        c.check("re-entry: all periods skipped", b.getBurstSkip() == period);
        c.check("re-entry: leg 1 runs (burst gates it)", b.isLeg1Enabled());
    }

    {
        /** @test L0-BST-06.a: From full skipping, the skip count falls by one per hot loop while the current is below its reference, then leg 1's PID runs */
        tc::Criterion c("L0-BST-06.a", "From full skipping, the skip count falls by one per hot loop while the current is below its reference, then the PID runs");
        BoostConverter b;
        c.check("configured", fresh(b));
        b.setVBus(395);
        b.setMode(BUSREG);
        b.setILowRef(1);
        b.setILow1(0);
        for (uint16_t i = 1; i <= period; ++i) {
            const float32_t d = b.hotLoop().leg1;
            c.check("skip count falls by one", b.getBurstSkip() == period - i);
            c.near("minimum pulse duty held while skipping", d, i_app.lower_bound, tol);
        }
        c.check("continuous", !b.isBurstActive());
        c.range("then the PID drives leg 1", b.hotLoop().leg1, 0.05F, 0.60F);
    }

    {
        /** @test L0-BST-06.b: At minimum duty with the current above its reference, skipping starts and follows the current error within [0, period] */
        tc::Criterion c("L0-BST-06.b", "At minimum duty with the current above its reference, skipping starts and follows the current error within [0, period]");
        BoostConverter b;
        c.check("configured", fresh(b));
        b.setVBus(395);
        b.setMode(BUSREG);
        b.setILowRef(1);
        b.setILow1(0);
        for (int i = 0; i < period; ++i) b.hotLoop();
        c.check("continuous after the start", b.getBurstSkip() == 0);
        b.setILowRef(0);
        b.setILow1(0.3F);  // more current than asked for: the PID goes to its lower bound
        int ticks = 0;
        while (b.getBurstSkip() == 0 && ticks < 1000) { b.hotLoop(); ++ticks; }
        c.near("duty at its lower bound", b.getDutyLeg1(), 0.05, tol);
        c.check("skipping started", b.getBurstSkip() == 1);
        for (int i = 0; i < 500; ++i) b.hotLoop();
        c.check("skip count saturates at the period", b.getBurstSkip() == period);
        b.setILowRef(1);
        b.hotLoop();
        c.check("current below reference: one less", b.getBurstSkip() == period - 1);
        b.setILow1(1);
        b.hotLoop();
        c.check("current equal to reference: held", b.getBurstSkip() == period - 1);
    }

    {
        /** @test L0-BST-06.c: Leg 1's PID keeps its state across burst: duty held while skipping, resumes where it left */
        tc::Criterion c("L0-BST-06.c", "Leg 1's PID keeps its state across burst: duty held while skipping, resumes where it left");
        BoostConverter b;
        c.check("configured", fresh(b));
        b.setVBus(395);
        b.setMode(BUSREG);
        Pid mirror;
        mirror.init(i_app);
        float32_t i1 = 0.0F, ref = 0.5F;
        b.setILowRef(ref);
        int continuous_ticks = 0, burst_ticks = 0;
        for (int i = 0; i < 3000; ++i) {
            // the current alternates around the reference, slowly, to drive the burst in and out
            i1 = (i / 400) % 2 ? 0.0F : 2.0F;
            b.setILow1(i1);
            const bool continuous = b.getBurstSkip() == 0;
            const float32_t held = b.getDutyLeg1();
            const float32_t d = b.hotLoop().leg1;
            if (continuous) {
                ++continuous_ticks;
                c.near("continuous duty = uninterrupted PID", d, mirror.calculateWithReturn(ref, i1), 1e-6);
            } else {
                ++burst_ticks;
                c.near("duty held while skipping", d, held, 1e-7);
            }
        }
        c.check("both regimes exercised", continuous_ticks > 100 && burst_ticks > 100);
    }

    {
        tc::Criterion c("L0-BST-06.d", "Leg 1 burst follows its current error independently of leg 2 demand");
        const float limit = 5.5F;
        const float one_step = std::nextafter(limit, std::numeric_limits<float>::infinity()) - limit;
        for (boost_mode mode : {BUSREG, MPPT}) {
            for (float excess : {one_step, 0.001F, 2.5F}) {
                BoostConverter b;
                c.check("configured", fresh(b));
                b.setMode(mode);
                b.setILowRef(limit + excess);
                c.check("positive leg 2 reference exercised", b.getILow2Ref() > 0);
                b.setILow1(7);
                b.hotLoop();
                c.check("leg 2 demand does not cancel full skipping", b.getBurstSkip() == period);
                b.setILow1(0);
                for (int n = 0; n <= period; ++n) b.hotLoop();
                c.check("leg 1 below reference resumes continuous PWM", b.getBurstSkip() == 0);
                b.setILow1(7);
                for (int n = 0; n < 2000; ++n) b.hotLoop();
                c.near("leg 1 at minimum duty", b.getDutyLeg1(), i_app.lower_bound, tol);
                c.check("leg 1 above reference enters full skipping", b.getBurstSkip() == period);
            }
        }
    }

    {
        /** @test L0-BST-06.e: A zero burst period is rejected; without burst configured, leg 1 never skips */
        tc::Criterion c("L0-BST-06.e", "A zero burst period is rejected; without burst configured, leg 1 never skips");
        BoostConverter b;
        c.check("init", b.init(BUSREG, 400, v_app, i_app, 5.5F) == 0);
        c.check("period 0 rejected", b.setBurst(0) < 0 && !b.isBurstEnabled());
        b.setVBus(395);
        b.setILowRef(0);
        b.setILow1(1);
        for (int i = 0; i < 2000; ++i) b.hotLoop();
        c.check("never skips", b.getBurstSkip() == 0);
        c.near("stays at minimum duty", b.getDutyLeg1(), 0.05, tol);
    }

    {
        /** @test L0-BST-07.a: Above 440 V every period is skipped and the PID is held; below, the current error drives the burst again */
        tc::Criterion c("L0-BST-07.a", "Above 440 V every period is skipped and the PID is held; below, the current error drives the burst again");
        BoostConverter b;
        c.check("configured", fresh(b));
        b.setVBus(395);
        b.setMode(BUSREG);
        b.setILowRef(1);
        b.setILow1(0);
        for (int i = 0; i < period + 50; ++i) b.hotLoop();
        const float32_t duty = b.getDutyLeg1();
        b.setVBus(441);
        b.hotLoop();
        c.check("full skipping", b.getBurstSkip() == period);
        c.near("duty held", b.getDutyLeg1(), duty, 1e-7);
        c.check("not tripped", !b.isTripped() && b.isLeg1Enabled());
        b.setVBus(439);
        b.hotLoop();
        c.check("below 440 V the skip count falls again", b.getBurstSkip() == period - 1);
    }

    {
        /** @test L0-BST-07.b: Above 460 V the boost trips: legs off, outputs cleared, latched until IDLE */
        tc::Criterion c("L0-BST-07.b", "Above 460 V the boost trips: legs off, outputs cleared, latched until IDLE");
        BoostConverter b;
        c.check("configured", fresh(b));
        b.setVBus(395);
        b.setMode(BUSREG);
        b.coldLoop();
        b.setVBus(461);
        duty = b.hotLoop();
        c.check("tripped", b.isTripped());
        c.check("legs off, duties 0", !b.isLeg1Enabled() && !b.isLeg2Enabled() && duty.leg1 == 0 && duty.leg2 == 0);
        b.setVBus(400);
        c.near("cold loop gives no demand", b.coldLoop(), 0, tol);
        b.hotLoop();
        b.setMode(STARTUP);
        c.check("still tripped after a voltage drop and an active mode change", b.isTripped() && !b.isLeg1Enabled());
        b.setMode(IDLE);
        c.check("IDLE clears the trip", !b.isTripped());
        b.setMode(BUSREG);
        c.check("runs again", b.isLeg1Enabled());
    }

    {
        /** @test L0-BST-07.c: A leg current above 8 A trips the boost */
        tc::Criterion c("L0-BST-07.c", "A leg current above 8 A trips the boost");
        for (int leg = 1; leg <= 2; ++leg) {
            BoostConverter b;
            c.check("configured", fresh(b));
            b.setVBus(395);
            b.setMode(BUSREG);
            b.hotLoop();
            if (leg == 1) b.setILow1(8.1F); else b.setILow2(8.1F);
            b.hotLoop();
            c.check(leg == 1 ? "leg 1 over-current trips" : "leg 2 over-current trips", b.isTripped() && !b.isLeg1Enabled());
        }
        BoostConverter b;
        c.check("configured", fresh(b));
        b.setVBus(395);
        b.setMode(BUSREG);
        b.setILow1(8.0F);
        b.hotLoop();
        c.check("exactly 8 A does not trip", !b.isTripped());
    }

    {
        /** @test L0-BST-07.d: Inconsistent protection settings are rejected */
        tc::Criterion c("L0-BST-07.d", "Inconsistent protection settings are rejected");
        BoostConverter b;
        c.check("init", b.init(BUSREG, 400, v_app, i_app, 5.5F) == 0);
        c.check("trip voltage not above full burst", b.setProtection(440, 440, 8) < 0);
        c.check("zero current", b.setProtection(440, 460, 0) < 0);
        c.check("NaN", b.setProtection(std::numeric_limits<float>::quiet_NaN(), 460, 8) < 0);
        b.setVBus(1000);
        b.hotLoop();
        c.check("unconfigured protection never trips", !b.isTripped());
    }

    {
        /** @test L0-BST-08.a: The voltage PID's Kp follows gain x VBusRef / VLow, both clamped, at each STARTUP/BUSREG cold loop */
        tc::Criterion c("L0-BST-08.a", "The voltage PID's Kp follows gain x VBusRef / VLow, both clamped, at each STARTUP/BUSREG cold loop");
        BoostConverter b;
        c.check("configured", fresh(b));
        c.check("schedule accepted", b.setVoltageGainSchedule(0.4333F, 24, 48, 390, 410) == 0);
        b.setVBus(395);
        b.setMode(BUSREG);
        struct { float vin, vref, kp; } cases[] = {
            {36, 400, 0.4333F * 400 / 36}, {24, 400, 0.4333F * 400 / 24}, {48, 400, 0.4333F * 400 / 48},
            {12, 400, 0.4333F * 400 / 24}, {60, 400, 0.4333F * 400 / 48},
            {36, 380, 0.4333F * 390 / 36}, {36, 420, 0.4333F * 410 / 36}};
        for (auto k : cases) {
            b.setVLow(k.vin);
            b.setVBusRef(k.vref);
            b.coldLoop();
            c.near("Kp (A/V)", b.getVoltageKp(), k.kp, 1e-4);
        }
        b.setMode(STARTUP);
        b.setVLow(30);
        b.setVBusRef(400);
        b.coldLoop();
        c.near("STARTUP schedules too", b.getVoltageKp(), 0.4333F * 400 / 30, 1e-4);
        b.setMode(MPPT);
        b.setVLow(40);
        b.coldLoop();
        c.near("MPPT does not touch it", b.getVoltageKp(), 0.4333F * 400 / 30, 1e-4);
    }

    {
        /** @test L0-BST-08.b: Scheduling does not reset the voltage PID */
        tc::Criterion c("L0-BST-08.b", "Scheduling does not reset the voltage PID");
        BoostConverter b;
        c.check("configured", fresh(b));
        c.check("schedule accepted", b.setVoltageGainSchedule(0.4333F, 24, 48, 390, 410) == 0);
        b.setVBus(399.95F);
        b.setMode(BUSREG);
        Pid mirror;
        mirror.init(v_app);
        for (int i = 0; i < 40; ++i) {
            const float vin = i < 20 ? 36.0F : 30.0F;  // input voltage change mid-run
            b.setVLow(vin);
            b.setVBus(399.95F + 0.001F * i);
            mirror.setKp(0.4333F * 400 / vin);
            mirror.setBounds(0, 11);
            c.near("total reference = uninterrupted PID with the scheduled Kp", b.coldLoop(),
                   mirror.calculateWithReturn(400, 399.95F + 0.001F * i), 1e-5);
        }
    }

    {
        /** @test L0-BST-08.c: Inconsistent gain schedules are rejected */
        tc::Criterion c("L0-BST-08.c", "Inconsistent gain schedules are rejected");
        BoostConverter b;
        c.check("init", b.init(BUSREG, 400, v_app, i_app, 5.5F) == 0);
        c.check("zero gain", b.setVoltageGainSchedule(0, 24, 48, 390, 410) < 0);
        c.check("inverted Vin range", b.setVoltageGainSchedule(0.4F, 48, 24, 390, 410) < 0);
        c.check("zero Vin", b.setVoltageGainSchedule(0.4F, 0, 48, 390, 410) < 0);
        c.check("NaN", b.setVoltageGainSchedule(std::numeric_limits<float>::quiet_NaN(), 24, 48, 390, 410) < 0);
    }
    {
        // L0-BST-09: leg 2 starts only when its share of the demand exceeds a threshold
        // (requirement (c), Luiz 2026-11-06). Below it, leg 2 stays off and leaves leg 1's
        // burst alone. The threshold value is not chosen yet: the demands below (one float
        // step, 1 mA) are under any sensible threshold, and 1 A is above it.
        const float limit = 5.5F;
        const float one_step = std::nextafter(limit, 100.0F) - limit;  // 4.8e-7 A at 5.5 A
        for (boost_mode mode : {BUSREG, MPPT}) {
            const std::string m = mode == BUSREG ? " (BUSREG)" : " (MPPT)";
            const std::string id = mode == BUSREG ? "-busreg" : "-mppt";
            for (float excess : {one_step, 0.001F}) {
                const std::string what = excess < 1e-5F ? "one float step (4.8e-7 A)" : "1 mA";
                /** @test L0-BST-09.a-busreg/-mppt: A leg-2 demand of one float step or 1 mA keeps leg 2 off */
                tc::Criterion c(std::string("L0-BST-09.a") + id + (excess < 1e-5F ? "-step" : "-1mA"),
                                "Leg-2 demand of " + what + " keeps leg 2 off, its duty 0" + m);
                BoostConverter b;
                c.check("configured", fresh(b));
                b.setVBus(399);
                b.setMode(mode);
                b.setILowRef(limit + excess);
                b.setILow1(limit);
                b.setILow2(0.3F);  // leg 2 would conduct at minimum duty (low bus): nothing may regulate it
                c.check("leg 2 disabled", !b.isLeg2Enabled());
                bool on = false, duty = false;
                for (int n = 0; n < 1000; ++n) {
                    const BoostDutyCycles d = b.hotLoop();
                    on |= b.isLeg2Enabled();
                    duty |= d.leg2 != 0.0F;
                }
                c.check("leg 2 never enabled over 1000 ticks", !on);
                c.check("leg 2 duty always 0", !duty);
            }
            {
                /** @test L0-BST-09.b-busreg/-mppt: A negligible leg-2 demand does not cancel leg 1's burst */
                tc::Criterion c(std::string("L0-BST-09.b") + id, "A negligible leg-2 demand does not cancel leg 1's burst" + m);
                BoostConverter b;
                c.check("configured", fresh(b));
                b.setVBus(399);
                b.setMode(mode);  // entered from IDLE: fully skipped
                b.setILowRef(limit + one_step);
                b.setILow1(limit + 0.5F);  // leg 1 above its reference: burst should persist
                b.hotLoop();
                c.check("leg 1 still skipping periods", b.getBurstSkip() > 0);
            }
            {
                /** @test L0-BST-09.c-busreg/-mppt: A clear leg-2 demand (1 A) still starts leg 2 with that reference */
                tc::Criterion c(std::string("L0-BST-09.c") + id, "A clear leg-2 demand (1 A) still starts leg 2 with that reference" + m);
                BoostConverter b;
                c.check("configured", fresh(b));
                b.setVBus(399);
                b.setMode(mode);
                b.setILowRef(limit + 1.0F);
                c.check("leg 2 enabled", b.isLeg2Enabled());
                c.near("leg 2 reference (A)", b.getILow2Ref(), 1.0, 1e-5);
                c.near("leg 1 at its limit (A)", b.getILow1Ref(), limit, 1e-5);
            }
        }
        {
            // INFO: how often leg 2 starts while the total reference wanders across leg 1's
            // limit (5.4–5.6 A ramp with ±10 mA noise, BUSREG): with a plain threshold and no
            // hysteresis, some chattering around the threshold remains by design.
            BoostConverter b;
            fresh(b);
            b.setVBus(399);
            b.setMode(BUSREG);
            int starts = 0;
            bool was_on = false;
            unsigned seed = 12345U;
            for (int n = 0; n <= 20000; ++n) {
                seed = seed * 1103515245U + 12345U;
                const float noise = 0.01F * (static_cast<float>((seed >> 8) & 0xFFFF) / 32767.5F - 1.0F);
                b.setILowRef(5.4F + 0.2F * n / 20000.0F + noise);
                b.hotLoop();
                const bool on = b.isLeg2Enabled();
                starts += on && !was_on;
                was_on = on;
            }
            /** @test L0-BST-09.d: Leg-2 starts over a noisy ramp of the total reference across leg 1's limit (INFO) */
            tc::info("L0-BST-09.d", "Leg-2 starts over a noisy 5.4-5.6 A ramp of the total reference (+/-10 mA noise)", starts, "starts");
        }
    }

    {
        /** @test L0-BST-01.c: Invalid measurements and configuration cannot enable a leg */
        tc::Criterion c("L0-BST-01.c", "Invalid measurements and configuration cannot enable a leg");
        boost.setILow2(std::numeric_limits<float>::quiet_NaN());
        c.near("NaN measurement ignored", boost.getILow2(), 1, tol);
        for (float threshold : {0.0F, -1.0F, std::numeric_limits<float>::infinity()}) {
            c.check("invalid leg 1 limit rejected", boost.init(BUSREG, 48, voltage, current, threshold) < 0);
            c.check("no leg on", !boost.isLeg1Enabled() && !boost.isLeg2Enabled());
            duty = boost.hotLoop();
            c.check("duties 0", duty.leg1 == 0 && duty.leg2 == 0);
        }
        current.upper_bound = 1.1F;
        c.check("duty bound > 1 rejected", boost.init(BUSREG, 48, voltage, current, 5) < 0);
    }

    {
        tc::Criterion c("L0-BST-10.a", "Notch rejects 100 Hz, preserves DC, and never masks raw over-voltage");
        BoostConverter filtered;
        const PidParams notch_current = {0.0001F, 0.1F, 1.0F, 0, 1, 0, 0.95F};
        c.check("cannot configure before init", filtered.configureRippleControl(100, 10, 0.88F, 0.09425F) < 0);
        c.check("init", filtered.init(BUSREG, 400, voltage, notch_current, 5) == 0);
        for (float frequency : {0.0F, 5000.0F, std::numeric_limits<float>::quiet_NaN()})
            c.check("invalid frequency rejected", filtered.configureRippleControl(frequency, 10, 0.88F, 0.09425F) < 0);
        c.check("configure", filtered.configureRippleControl(100, 10, 0.88F, 0.09425F) == 0);
        filtered.enableVoltageNotch(true);
        filtered.setVBus(400);
        c.near("no startup transient", filtered.getFilteredVBus(), 400, 1e-5);
        double energy = 0;
        for (int n = 0; n < 30000; ++n) {
            filtered.setVBus(400.0F + 10.0F * std::sin(2.0F * 3.14159265358979323846F * n / 100.0F));
            if (n >= 20000) {
                const double residual = filtered.getFilteredVBus() - 400.0;
                energy += residual * residual;
            }
        }
        c.check("100 Hz residual below 0.02 V RMS for 10 V peak input", std::sqrt(energy / 10000) < 0.02);
        filtered.setProtection(440, 460, 8);
        filtered.setVBus(460.05F);
        c.check("notch hides spike from voltage loop", filtered.getFilteredVBus() < 460);
        filtered.hotLoop();
        c.check("raw spike still trips", filtered.isTripped() && !filtered.isLeg1Enabled());
        filtered.setMode(IDLE);
        filtered.setVBus(400);
        c.near("IDLE clears notch history", filtered.getFilteredVBus(), 400, 1e-5);
        filtered.enableVoltageNotch(false);
        filtered.setVBus(410);
        c.near("bypass uses raw sample", filtered.getFilteredVBus(), 410, 1e-5);
    }
    {
        tc::Criterion c("L0-BST-10.b", "Resonator selectively increases 100 Hz current-loop gain and resets cleanly");
        auto response = [](float frequency) {
            Pr resonator;
            resonator.init({0.0001F, 0.0F, 0.88F, 2.0F * PI * 100.0F, 0.09425F, -1.0F, 1.0F});
            double energy = 0;
            for (int n = 0; n < 20000; ++n) {
                const float error = 0.01F * std::sin(2.0F * 3.14159265358979323846F * frequency * n * 0.0001F);
                const double output = resonator.calculateWithReturn(error, 0.0F);
                if (n >= 10000) energy += output * output;
            }
            return std::sqrt(energy / 10000);
        };
        c.check("100 Hz gain exceeds 50 Hz gain by 100x", response(100) > 100 * response(50));
        BoostConverter regulated, pi_only;
        PidParams v = {0.001F, 0.1F, 1, 0, 0, 0, 12};
        PidParams i = {0.0001F, 0.002712F, 0.0003413F, 0, 0, 0.005F, 0.6F};
        regulated.init(BUSREG, 400, v, i, 5);
        pi_only.init(BUSREG, 400, v, i, 5);
        regulated.configureRippleControl(100, 10, 0.88F, 0.09425F);
        regulated.enableCurrentResonant(true);
        regulated.setILowRef(7); pi_only.setILowRef(7);
        const auto pr = regulated.hotLoop();
        const auto pi = pi_only.hotLoop();
        c.check("correction added on both legs", pr.leg1 > pi.leg1 && pr.leg2 > pi.leg2);
        for (int n = 0; n < 10000; ++n) {
            const auto d = regulated.hotLoop();
            c.check("combined duties bounded", d.leg1 >= 0.005F && d.leg1 <= 0.6F && d.leg2 >= 0.005F && d.leg2 <= 0.6F);
        }
        regulated.setMode(IDLE); pi_only.setMode(IDLE);
        regulated.setMode(BUSREG); pi_only.setMode(BUSREG);
        regulated.setILowRef(7); pi_only.setILowRef(7);
        regulated.enableCurrentResonant(false);
        const auto reset = regulated.hotLoop();
        const auto baseline = pi_only.hotLoop();
        c.near("disable/reset restores PI leg 1", reset.leg1, baseline.leg1, 1e-6);
        c.near("disable/reset restores PI leg 2", reset.leg2, baseline.leg2, 1e-6);
    }
    {
        tc::Criterion c("L0-BST-10.c", "Burst holds the combined PI/resonator state until continuous PWM resumes");
        BoostConverter paused, continuous;
        PidParams v = {0.001F, 0.1F, 1, 0, 0, 0, 12};
        PidParams i = {0.0001F, 0.002712F, 0.0003413F, 0, 0, 0.005F, 0.6F};
        for (auto* controller : {&paused, &continuous}) {
            controller->init(MPPT, 400, v, i, 5);
            controller->configureRippleControl(100, 10, 0.88F, 0.09425F);
            controller->enableCurrentResonant(true);
            controller->setBurst(200);
            controller->setProtection(440, 460, 8);
            controller->setMode(IDLE);
            controller->setMode(MPPT);
            controller->setILowRef(2);
            for (int n = 0; n < 250; ++n) controller->hotLoop();
        }
        const float held = paused.getDutyLeg1();
        paused.setVBus(450);
        for (int n = 0; n < 50; ++n) c.near("duty held during full burst", paused.hotLoop().leg1, held, 1e-6);
        paused.setVBus(400);
        for (int n = 0; n < 200; ++n) paused.hotLoop();
        c.check("skip count returns to zero", paused.getBurstSkip() == 0);
        c.near("resumed PI+R matches unpaused state", paused.hotLoop().leg1, continuous.hotLoop().leg1, 1e-6);
    }
    {
        tc::Criterion c("L0-BST-10.d", "Library filter buffers survive reconfiguration, reinitialization and destruction");
        BoostConverter controller;
        const PidParams v = {0.001F, 0.1F, 1, 0, 0, 0, 12};
        const PidParams i = {0.0001F, 0.002712F, 0.0003413F, 0, 0, 0.005F, 0.6F};
        for (int n = 0; n < 10; ++n) {
            c.check("reinitialize", controller.init(BUSREG, 400, v, i, 5) == 0);
            c.check("configure", controller.configureRippleControl(100, 10, 0.88F, 0.09425F) == 0);
            controller.enableVoltageNotch(true);
            controller.enableCurrentResonant(true);
            controller.setVBus(400);
            controller.setILowRef(2);
            controller.hotLoop();
            c.check("reconfigure", controller.configureRippleControl(100, 10, 0.88F, 0.09425F) == 0);
            c.check("reconfiguration disables paths", !controller.isVoltageNotchEnabled() && !controller.isCurrentResonantEnabled());
        }
    }
    return tc::finish();
}
