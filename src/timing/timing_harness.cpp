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

/* Task API proxy probes (owntech_task_api uninterruptible_synchronous_task.cpp, TIMING builds only). */
extern volatile uint32_t timing_proxy_cycles[3];

namespace timing {
volatile uint32_t probe_cycles[probe_count];
volatile uint32_t tick_cycles[tick_probe_count];
volatile uint32_t apply_cycles[apply_probe_count];
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
    /* L0-TIM-02: the whole critical tick in the critical task (main.cpp criticalTickBody, tick probes). */
    SEC_TICK,             ///< Whole tick.
    SEC_TICK_SENSORS,     ///< Sensor reads (ADC running).
    SEC_TICK_CONTROL,     ///< controlStep().
    SEC_TICK_APPLY,       ///< PWM duties, enables, burst, relay.
    SEC_TICK_SCOPE,       ///< Scope sampling as in production (every 5th tick at 2 kHz, every tick around the relay).
    /* L0-TIM-02: the controlStep and calculateDuty sections again, measured inside the critical tick (rows "2:…"),
     * so the two tests compare section by section. Same order as SEC_BOOST_COLD…SEC_OUTPUTS, SEC_DUTY_PLL…OUTPUT. */
    SEC_T2_BOOST_COLD,
    SEC_T2_BOOST_HOT,
    SEC_T2_INV_STAGE,
    SEC_T2_INV_DUTY,
    SEC_T2_OBSERVER,
    SEC_T2_OUTPUTS,
    SEC_T2_DUTY_PLL,
    SEC_T2_DUTY_SOGI,
    SEC_T2_DUTY_PARK,
    SEC_T2_DUTY_SYNCWIN,
    SEC_T2_DUTY_MODE_FF,
    SEC_T2_DUTY_CURRENT_PI,
    SEC_T2_DUTY_OUTPUT,
    /* L0-TIM-03: the critical task's whole tick (its load bookkeeping included) with the background tasks running,
     * the same parts and sections (rows "3:…"), and the gap between two task starts. */
    SEC_T3_TICK,
    SEC_T3_TICK_SENSORS,
    SEC_T3_TICK_CONTROL,
    SEC_T3_TICK_APPLY,
    SEC_T3_TICK_SCOPE,
    SEC_T3_BOOST_COLD,
    SEC_T3_BOOST_HOT,
    SEC_T3_INV_STAGE,
    SEC_T3_INV_DUTY,
    SEC_T3_OBSERVER,
    SEC_T3_OUTPUTS,
    SEC_T3_DUTY_PLL,
    SEC_T3_DUTY_SOGI,
    SEC_T3_DUTY_PARK,
    SEC_T3_DUTY_SYNCWIN,
    SEC_T3_DUTY_MODE_FF,
    SEC_T3_DUTY_CURRENT_PI,
    SEC_T3_DUTY_OUTPUT,
    SEC_T3_GAP,           ///< Time between this task start and the previous one (period 100 µs).
    SEC_T3_HARNESS,       ///< What the harness itself adds in the interrupt (inputs, recording), outside the task tick.
    SEC_T3_SAFETY,        ///< Task API proxy: the safety check (CONFIG_OWNTECH_SAFETY_API), same interrupt, before the task.
    SEC_T3_DISPATCH,      ///< Task API proxy: spin.data.doFullDispatch() (ADC data), same interrupt, before the task.
    /* L0-TIM-03: the PWM and relay writes split by call (apply probes in main.cpp). */
    SEC_T3_AP_BOOST_DUTY,   ///< Boost mode-change stop and setDutyCycle(LEG1_LOW).
    SEC_T3_AP_BOOST_EN,     ///< Boost start/stop(LEG1_LOW).
    SEC_T3_AP_BURST,        ///< setBurstMode / startBurstMode / stopBurstMode.
    SEC_T3_AP_BRIDGE_DUTY,  ///< setInverterDutyCycle(LEG2_HIGH).
    SEC_T3_AP_BRIDGE_EN,    ///< Bridge staging and start/stop of both high legs.
    SEC_T3_AP_PWM_ACK,      ///< inverter.setPwmEnabled().
    SEC_T3_AP_RELAY,        ///< Relay command and contact model (GPIO 22 on change).
    SEC_COUNT
};
const char *const section_names[SEC_COUNT] = {
    "boost_cold", "boost_hot", "inv_stage", "inv_duty", "observer", "outputs", "controlStep", "scope",
    "duty_pll", "duty_sogi", "duty_park", "duty_syncwin", "duty_mode_ff", "duty_current_pi", "duty_output",
    "tick", "tick_sensors", "tick_control", "tick_apply", "tick_scope",
    "2:boost_cold", "2:boost_hot", "2:inv_stage", "2:inv_duty", "2:observer", "2:outputs",
    "2:duty_pll", "2:duty_sogi", "2:duty_park", "2:duty_syncwin", "2:duty_mode_ff", "2:duty_current_pi", "2:duty_output",
    "3:tick", "3:tick_sensors", "3:tick_control", "3:tick_apply", "3:tick_scope",
    "3:boost_cold", "3:boost_hot", "3:inv_stage", "3:inv_duty", "3:observer", "3:outputs",
    "3:duty_pll", "3:duty_sogi", "3:duty_park", "3:duty_syncwin", "3:duty_mode_ff", "3:duty_current_pi", "3:duty_output",
    "3:gap", "3:harness", "3:safety", "3:dispatch",
    "3:ap_boost_duty", "3:ap_boost_en", "3:ap_burst", "3:ap_bridge_duty", "3:ap_bridge_en", "3:ap_pwm_ack", "3:ap_relay"
};
static_assert(SEC_T3_AP_RELAY - SEC_T3_AP_BOOST_DUTY + 2U == timing::apply_probe_count, "one row per apply interval");
static_assert(SEC_T2_OUTPUTS - SEC_T2_BOOST_COLD + 2U == timing::probe_count, "TIM-02 controlStep sections");
static_assert(SEC_T2_DUTY_OUTPUT - SEC_T2_DUTY_PLL + 2U == INV_TIMING_PROBE_COUNT, "TIM-02 library sections");
static_assert(SEC_TICK_SCOPE - SEC_TICK + 1U == timing::tick_probe_count && SEC_T2_DUTY_PLL == SEC_T2_OUTPUTS + 1U,
              "tick rows then sections, contiguous (recordTickRows)");
static_assert(SEC_T3_TICK_SCOPE - SEC_T3_TICK + 1U == timing::tick_probe_count && SEC_T3_DUTY_PLL == SEC_T3_OUTPUTS + 1U &&
              SEC_T3_BOOST_COLD == SEC_T3_TICK_SCOPE + 1U, "TIM-03 rows mirror TIM-02's");
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

/** @brief Synthetic measurements of scenario tick @p tick (both tests use the same signals). */
/* One grid period is exactly 200 ticks (50 Hz at 10 kHz): the synthetic sine comes from a table, so the harness
 * adds no sinf/fmodf to the critical task (L0-TIM-03 measures that task's start timing). */
constexpr uint32_t grid_period_ticks = 200U;
static_assert(1000000U / app::control_task_period == 50U * grid_period_ticks, "200 ticks per 50 Hz period");
float32_t sine_table[grid_period_ticks];

void fillSineTable()
{
    for (uint32_t i = 0U; i < grid_period_ticks; ++i)
        sine_table[i] = sinf(2.0F * PI * static_cast<float32_t>(i) / static_cast<float32_t>(grid_period_ticks));
}

Measurements synthetic(uint32_t tick, bool relay_closed)
{
    const float32_t s = sine_table[tick % grid_period_ticks];
    Measurements meas{};
    meas.i_low_1 = 2.0F;
    meas.v_low = 36.0F;
    meas.v_dc_bus = 400.0F;
    meas.v_ac = app::grid_voltage_peak * s;
    meas.i_ac = 0.05F * s;
    meas.v_ac_updated = true;
    meas.relay_closed = relay_closed;
    return meas;
}

/* ---- L0-TIM-02 state (driven from the critical task) ---- */
enum EventType : uint8_t { EV_ENTRY = 1, EV_MODE, EV_VSI, EV_RELAY_CMD, EV_RELAY_FB, EV_BRIDGE };
constexpr uint8_t max_events = 96U;
struct Tim02 {
    ControlState *state = nullptr;
    TimingTim02Hooks hooks{};
    volatile bool active = false;
    volatile bool done = false;
    uint32_t tick = 0U;         // scenario tick (continues TIM-01's, so the synthetic grid has no phase jump)
    uint32_t phase = 0U;
    uint32_t in_phase = 0U;
    uint8_t prev_vsi = 0U;
    bool prev_relay_cmd = false;
    bool prev_relay_fb = false;
    bool prev_bridge = false;
    uint8_t ev_count = 0U;
    uint8_t ev_bucket[max_events] = {};
    uint16_t ev_code[max_events] = {};
    uint16_t ev_row[max_events] = {};
};
Tim02 t2;
int bucket_of_phase[sizeof(phases) / sizeof(phases[0])];

void appendText(char *dst, size_t &n, size_t size, const char *src)
{
    for (const char *p = src; *p != '\0' && n + 1U < size; ++p) dst[n++] = *p;
    dst[n] = '\0';
}

/** @brief Row of an event in a bucket, created on its first occurrence; -1 when the table is full. */
int eventRow(uint32_t bucket, uint8_t type, uint8_t arg)
{
    const uint16_t code = static_cast<uint16_t>((type << 8) | arg);
    for (uint8_t i = 0U; i < t2.ev_count; ++i)
    {
        if (t2.ev_bucket[i] == bucket && t2.ev_code[i] == code) return t2.ev_row[i];
    }
    volatile TimingReport &r = g_timing_report;
    if (t2.ev_count >= max_events || r.row_count >= TIMING_MAX_ROWS) return -1;
    char text[24];
    size_t n = 0U;
    text[0] = '\0';
    appendText(text, n, sizeof(text), "ev:");
    switch (type)
    {
        case EV_ENTRY: appendText(text, n, sizeof(text), "entry"); break;
        case EV_MODE: appendText(text, n, sizeof(text), "boost mode"); break;
        case EV_VSI:
        {
            const char v[] = {'v', 's', 'i', ' ', static_cast<char>('0' + (arg >> 4)), '>',
                              static_cast<char>('0' + (arg & 0xFU)), '\0'};
            appendText(text, n, sizeof(text), v);
            break;
        }
        case EV_RELAY_CMD: appendText(text, n, sizeof(text), arg ? "relay cmd on" : "relay cmd off"); break;
        case EV_RELAY_FB: appendText(text, n, sizeof(text), arg ? "relay fb on" : "relay fb off"); break;
        default: appendText(text, n, sizeof(text), arg ? "bridge on" : "bridge off"); break;
    }
    const uint32_t row = r.row_count;
    {
        char full[32];
        size_t m = 0U;
        full[0] = '\0';
        for (size_t i = 0U; i < sizeof(r.buckets[bucket].name) && r.buckets[bucket].name[i] != '\0' && m + 1U < sizeof(full); ++i)
            full[m++] = r.buckets[bucket].name[i];
        full[m] = '\0';
        copyName(r.rows[row].name, sizeof(r.rows[row].name), full, text);
    }
    r.rows[row].min_cycles = UINT32_MAX;
    r.row_count = row + 1U;
    t2.ev_bucket[t2.ev_count] = static_cast<uint8_t>(bucket);
    t2.ev_code[t2.ev_count] = code;
    t2.ev_row[t2.ev_count] = static_cast<uint16_t>(row);
    ++t2.ev_count;
    return static_cast<int>(row);
}

inline void recordRow(uint32_t row, uint32_t c, uint32_t tick);

/** @brief Records the tick parts and the controlStep and calculateDuty sections of the last tick from row
 * @p first (tick, 4 parts, 6 controlStep sections, 7 calculateDuty sections, as SEC_TICK… and SEC_T2_…). */
void recordTickRows(uint32_t base, uint8_t tick_row, uint8_t section_row, uint32_t tick_cycles_total,
                    bool duty_fresh, uint32_t tick)
{
    recordRow(base + tick_row, tick_cycles_total, tick);
    for (uint8_t i = 0U; i + 1U < timing::tick_probe_count; ++i)
        recordRow(base + tick_row + 1U + i, timing::tick_cycles[i + 1U] - timing::tick_cycles[i], tick);
    for (uint8_t i = 0U; i < timing::probe_count - 1U; ++i)
        recordRow(base + section_row + i, timing::probe_cycles[i + 1U] - timing::probe_cycles[i], tick);
    if (duty_fresh)
    {
        for (uint8_t i = 0U; i < INV_TIMING_PROBE_COUNT - 1U; ++i)
            recordRow(base + section_row + (timing::probe_count - 1U) + i, inv_timing_cycles[i + 1U] - inv_timing_cycles[i], tick);
    }
}

inline void recordRow(uint32_t row, uint32_t c, uint32_t tick)
{
    /* Only the critical task writes these rows; plain stores keep the harness light (the host reads RAM as is). */
    TimingRow &x = const_cast<TimingRow &>(g_timing_report.rows[row]);
    x.calls = x.calls + 1U;
    x.sum_cycles = x.sum_cycles + c;
    if (c < x.min_cycles) x.min_cycles = c;
    if (c > x.max_cycles)
    {
        x.max_cycles = c;
        x.max_tick = tick;
    }
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

    /* Every bucket first, so their fixed rows are contiguous and the L0-TIM-02 event rows follow them. */
    for (size_t p = 0U; p < sizeof(phases) / sizeof(phases[0]); ++p)
    {
        bucket_of_phase[p] = findBucket(phases[p].bucket, phases[p].mode);
        if (bucket_of_phase[p] < 0)
        {
            r.error_code = 2U;  // report table full
            r.state = TIMING_ERROR;
            printk("timing: report table full\n");
            return;
        }
    }
    fillSineTable();
    printk("timing: L0-TIM-01 scenario started\n");
    /* Hardware-loop models, as in loop_critical_task(). */
    bool relay_command = false;
    bool relay_closed = false;
    uint16_t relay_hold_ticks = 0U;
    bool pwm_staged = false;
    uint32_t tick = 0U;

    for (size_t p = 0U; p < sizeof(phases) / sizeof(phases[0]); ++p)
    {
        const Phase &phase = phases[p];
        const uint32_t bucket = static_cast<uint32_t>(bucket_of_phase[p]);
        for (uint32_t i = 0U; i < phase.ticks; ++i, ++tick)
        {
            const Measurements meas = synthetic(tick, relay_closed);
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
    r.state = TIMING_TIM01_DONE;
    printk("timing: L0-TIM-01 done, %u ticks, %u rows\n", static_cast<unsigned int>(r.total_ticks),
           static_cast<unsigned int>(r.row_count));
}

void timingTim02Begin(ControlState &state, const TimingTim02Hooks &hooks)
{
    t2.state = &state;
    t2.hooks = hooks;
    t2.tick = g_timing_report.total_ticks;
    t2.phase = 0U;
    t2.in_phase = 0U;
    t2.prev_vsi = static_cast<uint8_t>(state.inverter.getMode());
    t2.done = false;
    t2.active = g_timing_report.state == TIMING_TIM01_DONE;
    printk("timing: L0-TIM-02 scenario started (critical task)\n");
}

void timingTim02Tick()
{
    if (!t2.active) return;
    volatile TimingReport &r = g_timing_report;
    const Phase &phase = phases[t2.phase];
    const uint32_t bucket = static_cast<uint32_t>(bucket_of_phase[t2.phase]);
    const bool entry = t2.in_phase == 0U;
    if (entry) t2.hooks.stage_start();  // as the bench keys: the scope re-armed for this stage

    const Measurements meas = synthetic(t2.tick, false);  // relay feedback: the critical task's own model
    OperatorCommands cmd;
    cmd.mode = phase.mode;
    cmd.inverter_stage = phase.stage;
    cmd.inject_current = app::inject_current_max;

    const uint32_t seq_before = inv_timing_seq;
    const uint32_t c0 = cycles();
    const ControlOutputs out = t2.hooks.tick(meas, cmd);
    const uint32_t c1 = cycles();

    const uint32_t base = bucket * SEC_COUNT;
    recordTickRows(base, SEC_TICK, SEC_T2_BOOST_COLD, c1 - c0, inv_timing_seq != seq_before, t2.tick);

    /* Events of this tick: each one's row gets the whole tick. */
    const uint8_t vsi = static_cast<uint8_t>(t2.state->inverter.getMode());
    const bool relay_fb = t2.hooks.relay_closed();
    struct { bool hit; uint8_t type; uint8_t arg; } ev[] = {
        {entry, EV_ENTRY, 0U},
        {out.boost_mode_changed, EV_MODE, 0U},
        {vsi != t2.prev_vsi, EV_VSI, static_cast<uint8_t>((t2.prev_vsi << 4) | (vsi & 0xFU))},
        {out.relay_close != t2.prev_relay_cmd, EV_RELAY_CMD, static_cast<uint8_t>(out.relay_close)},
        {relay_fb != t2.prev_relay_fb, EV_RELAY_FB, static_cast<uint8_t>(relay_fb)},
        {out.bridge_enable != t2.prev_bridge, EV_BRIDGE, static_cast<uint8_t>(out.bridge_enable)},
    };
    for (const auto &e : ev)
    {
        if (!e.hit) continue;
        const int row = eventRow(bucket, e.type, e.arg);
        if (row >= 0) recordRow(static_cast<uint32_t>(row), c1 - c0, t2.tick);
    }
    t2.prev_vsi = vsi;
    t2.prev_relay_cmd = out.relay_close;
    t2.prev_relay_fb = relay_fb;
    t2.prev_bridge = out.bridge_enable;

    r.buckets[bucket].tim02_vsi_seen_mask = r.buckets[bucket].tim02_vsi_seen_mask | (1UL << vsi);
    r.buckets[bucket].tim02_ticks = r.buckets[bucket].tim02_ticks + 1U;
    ++t2.tick;
    r.total_ticks = t2.tick;
    if (++t2.in_phase >= phase.ticks)
    {
        t2.in_phase = 0U;
        if (++t2.phase >= sizeof(phases) / sizeof(phases[0]))
        {
            t2.active = false;
            t2.done = true;
        }
    }
}

bool timingTim02Done() { return t2.done; }

namespace {
struct Tim03 {
    volatile bool active = false;
    volatile bool done = false;
    uint32_t tick = 0U;
    uint32_t phase = 0U;
    uint32_t in_phase = 0U;
    bool started = false;
    uint32_t prev_start = 0U;
    int32_t lag = 0;         // accumulated (gap - period), cycles: how late the starts run against the 10 kHz grid
};
Tim03 t3;
constexpr uint32_t period_cycles = 170U * app::control_task_period;  // at 170 MHz
}

void timingTim03Begin()
{
    t3.tick = g_timing_report.total_ticks;
    t3.phase = 0U;
    t3.in_phase = 0U;
    t3.started = false;
    t3.lag = 0;
    t3.done = false;
    g_timing_report.state = TIMING_TIM03_RUNNING;
    printk("timing: L0-TIM-03 scenario started (critical task, background tasks running)\n");
    t3.active = true;
}

bool timingTim03Done() { return t3.done; }

namespace {
void tim03Tick()
{
    const uint32_t start = cycles();
    volatile TimingReport &r = g_timing_report;
    const Phase &phase = phases[t3.phase];
    const uint32_t bucket = static_cast<uint32_t>(bucket_of_phase[t3.phase]);
    const uint32_t base = bucket * SEC_COUNT;
    if (t3.started)
    {
        const uint32_t gap = start - t3.prev_start;
        recordRow(base + SEC_T3_GAP, gap, t3.tick);
        /* Lost periods from the accumulated time, so steadily late starts count even when no single gap reaches
         * 1.5 periods: each period of accumulated lateness is one period without a task start. */
        t3.lag += static_cast<int32_t>(gap) - static_cast<int32_t>(period_cycles);
        while (t3.lag >= static_cast<int32_t>(period_cycles / 2U))
        {
            r.buckets[bucket].tim03_lost = r.buckets[bucket].tim03_lost + 1U;
            t3.lag -= static_cast<int32_t>(period_cycles);
        }
    }
    t3.prev_start = start;
    t3.started = true;
    if (t3.in_phase == 0U) t2.hooks.stage_start();

    const Measurements meas = synthetic(t3.tick, false);
    OperatorCommands cmd;
    cmd.mode = phase.mode;
    cmd.inverter_stage = phase.stage;
    cmd.inject_current = app::inject_current_max;
    const uint32_t seq_before = inv_timing_seq;
    const uint32_t c0 = cycles();
    (void)t2.hooks.task_tick(meas, cmd);
    const uint32_t c1 = cycles();
    recordTickRows(base, SEC_T3_TICK, SEC_T3_BOOST_COLD, c1 - c0, inv_timing_seq != seq_before, t3.tick);

    const uint8_t vsi = static_cast<uint8_t>(t2.state->inverter.getMode());
    for (uint8_t i = 0U; i + 1U < timing::apply_probe_count; ++i)
        recordRow(base + SEC_T3_AP_BOOST_DUTY + i, timing::apply_cycles[i + 1U] - timing::apply_cycles[i], t3.tick);
    recordRow(base + SEC_T3_SAFETY, timing_proxy_cycles[1] - timing_proxy_cycles[0], t3.tick);
    recordRow(base + SEC_T3_DISPATCH, timing_proxy_cycles[2] - timing_proxy_cycles[1], t3.tick);
    recordRow(base + SEC_T3_HARNESS, (cycles() - start) - (c1 - c0), t3.tick);
    r.buckets[bucket].tim03_vsi_seen_mask = r.buckets[bucket].tim03_vsi_seen_mask | (1UL << vsi);
    r.buckets[bucket].tim03_ticks = r.buckets[bucket].tim03_ticks + 1U;
    ++t3.tick;
    r.total_ticks = t3.tick;
    if (++t3.in_phase >= phase.ticks)
    {
        t3.in_phase = 0U;
        if (++t3.phase >= sizeof(phases) / sizeof(phases[0]))
        {
            t3.active = false;
            t3.done = true;
        }
    }
}
}  // namespace

void timingCriticalTick()
{
    if (t2.active) timingTim02Tick();
    else if (t3.active) tim03Tick();
}

void timingFinish()
{
    g_timing_report.state = TIMING_DONE;
    printk("timing: done, %u ticks, %u rows\n", static_cast<unsigned int>(g_timing_report.total_ticks),
           static_cast<unsigned int>(g_timing_report.row_count));
}

#endif  // TIMING_HARNESS
