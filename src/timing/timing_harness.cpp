/*
 * Copyright (c) 2026-present LAAS-CNRS
 * SPDX-License-Identifier: LGPL-2.1
 */

/**
 * @file timing_harness.cpp
 * @brief L0-TIM-01: cycle cost of each controlStep() section on the Spin (ADR 0005).
 *
 * The scenario walks the bench stages with synthetic measurements (230 V / 50 Hz grid, 400 V bus, 36 V / 2 A
 * input, 50 mA AC current) and the same relay and PWM acknowledgement models as the critical task. Each control
 * step runs with interrupts locked, so a row is the code's own cost (no ISR in it); the interference of the
 * running firmware is L0-TIM-03's job. Ticks run back to back (not paced at 100 µs): the counts do not depend
 * on wall time.
 *
 * @test L0-TIM-01
 */
#ifdef TIMING_HARNESS

#include "timing/timing_harness.h"

#include <math.h>
#include <string.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include "timing/timing_probe.h"
#include "timing/timing_report.h"
#include "inverterTiming.h"

namespace timing {
volatile uint32_t probe_cycles[probe_count];
}

/** @brief Read by the host over SWD (tests/l0/timing/read_report.py); kept even though nothing reads it here. */
__attribute__((used)) volatile TimingReport g_timing_report;

namespace {

volatile uint32_t *const dwt_ctrl = reinterpret_cast<volatile uint32_t *>(0xE0001000UL);
volatile uint32_t *const dwt_cyccnt = reinterpret_cast<volatile uint32_t *>(0xE0001004UL);
volatile uint32_t *const dcb_demcr = reinterpret_cast<volatile uint32_t *>(0xE000EDFCUL);

inline uint32_t cycles() { return *dwt_cyccnt; }

/** @brief Sections timed in every bucket; the first six are the probe intervals inside controlStep(). */
enum Section : uint8_t {
    SEC_BOOST_COLD = 0,   ///< Setters, mode change and the cold loop (every 10th tick).
    SEC_BOOST_HOT,        ///< Boost hot loop.
    SEC_INV_STAGE,        ///< Stage logic before calculateDuty (requests, step 2b settings).
    SEC_INV_DUTY,         ///< single_phase_inverter calculateDuty().
    SEC_OBSERVER,         ///< Grid observer and quality gate.
    SEC_OUTPUTS,          ///< Outputs, open loop, VAC follower, protection, relay timing.
    SEC_CONTROL_STEP,     ///< controlStep() as called by the critical task.
    SEC_SCOPE,            ///< Scope snapshot and acquire (on every tick, the 10 kHz worst case).
    /* Inside calculateDuty (library probes, inverterTiming.h); together they make up SEC_INV_DUTY. */
    SEC_DUTY_PLL,         ///< PLL PI (or connection freeze), angle update and wrap.
    SEC_DUTY_SOGI,        ///< Voltage and current SOGIs.
    SEC_DUTY_PARK,        ///< Park transforms of V and I, finite checks.
    SEC_DUTY_SYNCWIN,     ///< One-period sync averaging window.
    SEC_DUTY_MODE_FF,     ///< Mode state machine, connection bookkeeping, feedforward.
    SEC_DUTY_CURRENT_PI,  ///< Current d/q PIs (or their reset).
    SEC_DUTY_OUTPUT,      ///< Output rotation, external feedforward, duty, power, checks, PWM_WAIT.
    SEC_COUNT
};
const char *const section_names[SEC_COUNT] = {
    "boost_cold", "boost_hot", "inv_stage", "inv_duty", "observer", "outputs", "controlStep", "scope",
    "duty_pll", "duty_sogi", "duty_park", "duty_syncwin", "duty_mode_ff", "duty_current_pi", "duty_output"
};
static_assert(SEC_DUTY_OUTPUT - SEC_DUTY_PLL + 2U == INV_TIMING_PROBE_COUNT, "one section per library probe interval");

/** @brief Scenario phases; repeated labels share a bucket. */
struct Phase {
    const char *bucket;
    InverterStage stage;
    boost_mode mode;
    uint32_t ticks;
};
constexpr uint32_t tps = 1000000U / app::control_task_period;  // ticks per second
const Phase phases[] = {
    {"OFF_IDLE", INV_OFF, IDLE, 2U * tps},          // grid qualification (1 s), PLL locked
    {"OFF_BUSREG", INV_OFF, BUSREG, tps},
    {"OFF_MPPT", INV_OFF, MPPT, tps},
    {"SETTLE_c", INV_SETTLE, BUSREG, 3U * tps},
    {"OFF_BUSREG", INV_OFF, BUSREG, tps / 2U},
    {"CONNECT_g", INV_CONNECT, BUSREG, 3U * tps},   // settle 200 ms, relay, GRID_CONNECTED
    {"INJECT_n", INV_INJECT, BUSREG, 2U * tps},     // RUN
    {"OFF_BUSREG", INV_OFF, BUSREG, tps},           // disconnect sequence
    {"VAC_FOLLOW_a", INV_VAC_FOLLOW, BUSREG, 2U * tps},
    {"OFF_BUSREG", INV_OFF, BUSREG, tps / 2U},
    {"FOLLOW_CONN_G", INV_FOLLOW_CONNECT, BUSREG, 3U * tps},
    {"OFF_BUSREG", INV_OFF, BUSREG, tps},
    {"OPEN_LOOP_o", INV_OPEN_LOOP, BUSREG, 2U * tps},
    {"OFF_IDLE", INV_OFF, IDLE, tps / 2U},
};

void copyName(volatile char *dst, size_t size, const char *a, const char *b)
{
    size_t n = 0U;
    for (const char *p = a; *p != '\0' && n + 1U < size; ++p) dst[n++] = *p;
    if (b != nullptr)
    {
        if (n + 1U < size) dst[n++] = '/';
        for (const char *p = b; *p != '\0' && n + 1U < size; ++p) dst[n++] = *p;
    }
    while (n < size) dst[n++] = '\0';
}

bool sameName(const volatile char *a, const char *b, size_t size)
{
    for (size_t i = 0U; i < size; ++i)
    {
        if (a[i] != b[i]) return false;
        if (b[i] == '\0') return true;
    }
    return true;
}

/** @brief Bucket index for a label, created on first use; -1 when the table is full. */
int findBucket(const char *name, boost_mode mode)
{
    volatile TimingReport &r = g_timing_report;
    for (uint32_t i = 0U; i < r.bucket_count; ++i)
    {
        if (sameName(r.buckets[i].name, name, sizeof(r.buckets[i].name))) return static_cast<int>(i);
    }
    if (r.bucket_count >= TIMING_MAX_BUCKETS || r.row_count + SEC_COUNT > TIMING_MAX_ROWS) return -1;
    const uint32_t b = r.bucket_count;
    copyName(r.buckets[b].name, sizeof(r.buckets[b].name), name, nullptr);
    r.buckets[b].boost_mode = static_cast<uint8_t>(mode);
    for (uint8_t s = 0U; s < SEC_COUNT; ++s)
    {
        volatile TimingRow &row = r.rows[b * SEC_COUNT + s];
        copyName(row.name, sizeof(row.name), name, section_names[s]);
        row.min_cycles = UINT32_MAX;
    }
    r.bucket_count = b + 1U;
    r.row_count = r.bucket_count * SEC_COUNT;
    return static_cast<int>(b);
}

inline void record(uint32_t bucket, uint8_t section, uint32_t c, uint32_t tick)
{
    volatile TimingRow &row = g_timing_report.rows[bucket * SEC_COUNT + section];
    row.calls = row.calls + 1U;
    row.sum_cycles = row.sum_cycles + c;
    if (c < row.min_cycles) row.min_cycles = c;
    if (c > row.max_cycles)
    {
        row.max_cycles = c;
        row.max_tick = tick;
    }
}

}  // namespace

void timingHarnessRun(ControlState &state, TimingScopeHook scope_sample)
{
    volatile TimingReport &r = g_timing_report;
    memset(const_cast<TimingReport *>(&r), 0, sizeof(TimingReport));
    r.layout_version = TIMING_LAYOUT_VERSION;
    r.core_hz = sys_clock_hw_cycles_per_sec();
    r.state = TIMING_RUNNING;
    r.magic = TIMING_MAGIC;

    *dcb_demcr |= (1UL << 24);  // TRCENA: enable the DWT
    *dwt_cyccnt = 0U;
    *dwt_ctrl |= 1UL;           // CYCCNTENA
    const uint32_t probe_a = cycles();
    k_busy_wait(2);
    if (cycles() == probe_a)
    {
        r.error_code = 1U;  // DWT cycle counter not running
        r.state = TIMING_ERROR;
        printk("timing: DWT cycle counter not running\n");
        return;
    }

    printk("timing: L0-TIM-01 scenario started\n");
    /* Hardware-loop models, as in loop_critical_task(). */
    bool relay_command = false;
    bool relay_closed = false;
    uint16_t relay_hold_ticks = 0U;
    bool pwm_staged = false;
    uint32_t tick = 0U;
    const float32_t w = app::grid_angular_frequency;

    for (const Phase &phase : phases)
    {
        const int b = findBucket(phase.bucket, phase.mode);
        if (b < 0)
        {
            r.error_code = 2U;  // report table full
            r.state = TIMING_ERROR;
            printk("timing: report table full\n");
            return;
        }
        const uint32_t bucket = static_cast<uint32_t>(b);
        for (uint32_t i = 0U; i < phase.ticks; ++i, ++tick)
        {
            const float32_t t = static_cast<float32_t>(tick) * app::hot_loop_period;
            const float32_t angle = fmodf(w * t, 2.0F * PI);
            Measurements meas{};
            meas.i_low_1 = 2.0F;
            meas.v_low = 36.0F;
            meas.v_dc_bus = 400.0F;
            meas.v_ac = app::grid_voltage_peak * sinf(angle);
            meas.i_ac = 0.05F * sinf(angle);
            meas.v_ac_updated = true;
            meas.relay_closed = relay_closed;
            OperatorCommands cmd;
            cmd.mode = phase.mode;
            cmd.inverter_stage = phase.stage;
            cmd.inject_current = app::inject_current_max;

            const uint32_t seq_before = inv_timing_seq;
            const unsigned int key = irq_lock();
            const uint32_t c0 = cycles();
            const ControlOutputs out = controlStep(state, meas, cmd);
            const uint32_t c1 = cycles();
            scope_sample(state, out, meas);
            const uint32_t c2 = cycles();
            irq_unlock(key);

            for (uint8_t s = 0U; s < timing::probe_count - 1U; ++s)
            {
                record(bucket, s, timing::probe_cycles[s + 1U] - timing::probe_cycles[s], tick);
            }
            if (inv_timing_seq != seq_before)  // calculateDuty ran to its end: every library probe is fresh
            {
                for (uint8_t s = 0U; s < INV_TIMING_PROBE_COUNT - 1U; ++s)
                {
                    record(bucket, static_cast<uint8_t>(SEC_DUTY_PLL + s), inv_timing_cycles[s + 1U] - inv_timing_cycles[s], tick);
                }
            }
            record(bucket, SEC_CONTROL_STEP, c1 - c0, tick);
            record(bucket, SEC_SCOPE, c2 - c1, tick);

            /* Bridge acknowledgement: the first enabled tick stages the neutral duty, the next applies it. */
            if (out.bridge_enable)
            {
                state.bridge_pwm_applied = pwm_staged;
                pwm_staged = true;
            }
            else
            {
                state.bridge_pwm_applied = false;
                pwm_staged = false;
            }
            /* Relay: feedback follows the command once held for the contact time. */
            if (out.relay_close != relay_command)
            {
                relay_command = out.relay_close;
                relay_hold_ticks = 0U;
            }
            else if (relay_hold_ticks < app::relay_contact_ticks)
            {
                ++relay_hold_ticks;
            }
            else
            {
                relay_closed = relay_command;
            }

            const uint8_t vsi = static_cast<uint8_t>(state.inverter.getMode());
            r.buckets[bucket].vsi_seen_mask = r.buckets[bucket].vsi_seen_mask | (1UL << vsi);
            r.buckets[bucket].final_vsi = vsi;
            r.buckets[bucket].ticks = r.buckets[bucket].ticks + 1U;
            r.total_ticks = tick + 1U;
            if ((tick & 0x3FFU) == 0x3FFU) k_msleep(1);  // let the lower-priority threads (USB) run
        }
    }
    r.state = TIMING_DONE;
    printk("timing: done, %u ticks, %u rows\n", static_cast<unsigned int>(r.total_ticks),
           static_cast<unsigned int>(r.row_count));
}

#endif  // TIMING_HARNESS
