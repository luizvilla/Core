/*
 * Copyright (c) 2026-present LAAS-CNRS
 *
 * SPDX-License-Identifier: LGPL-2.1
 */

/**
 * @file main.cpp
 * @brief Safety API timing overhead firmware (issue #27, docs/safety/timing_overhead.md).
 *
 * Bare Spin, Twist 1.4.1 devicetree. One scenario per boot (the harness sets it
 * through a backup register and resets the board): the control task runs at
 * 50 or 100 us on the HRTIM with data dispatch and a fixed busy load, the PWM
 * runs, and for MEASURE_S seconds the firmware records:
 *  - the gap between two control-task starts (worst cases, histogram), lost periods;
 *  - the CPU left to the main thread, by subtraction: a loop that counts the
 *    cycles it gets (whatever it does not get went to interrupts and to the
 *    work queue, the run-time self-tests included);
 *  - the time of the first PWM output enable (arming).
 * Scenario S1 also runs the old software check on every Twist sensor.
 *
 * The same source builds at Core main (B0, before the safety work: no
 * safety_mode.h, no supervision) and on safety-hw (B1 production, B1p with
 * timing probes, B2 development with self-tests off and disarm()).
 * Driven over SWD by tests/l0/safety_timing/run.py. Never flash it on a powered board.
 */

#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/reboot.h>
#include <soc.h>

#include "SpinAPI.h"
#include "TaskAPI.h"
#include "ShieldAPI.h"
#include "SafetyAPI.h"

#if __has_include("safety_mode.h")
#include "safety_mode.h"
#include "safety_supervision.h"
#define NEW_SAFETY 1
#include "safety_timing.h"
#else
#define NEW_SAFETY 0
#define SAFETY_DEVELOPMENT_MODE 0
#endif

#define REPORT_MAGIC  0x31564F54U  /* "TOV1" */
#define MEASURE_S     4U
#define LOAD_US       10U          /* busy load of the control task */
#define HIST_BINS     64U          /* gap deviation from the period, 0.25 us per bin, centred */
#define HIST_STEP_CYC 42U          /* 0.25 us at 170 MHz (rounded) */

enum : uint32_t {
	CASE_CLEAR = 0,
	CASE_S0_50 = 1, CASE_S1_50 = 2, CASE_S0_100 = 3, CASE_S1_100 = 4,
	CASE_REACT = 5,   /* software trip under load (safety-hw builds only) */
};

struct report_t {
	uint32_t magic, layout, boot_count, ack, busy, latched_at_boot, pending;
	uint32_t build;            /* 0 B0 (Core main), 1 safety-hw production, 2 safety-hw development */
	uint32_t scenario, period_us, load_us, core_hz;
	uint32_t periods;          /* control-task starts recorded */
	uint32_t expected;         /* periods the window should hold */
	uint32_t gap_min, gap_max; /* cycles between two starts */
	uint32_t gap_late;         /* starts more than 1.5 periods after the previous one */
	uint32_t hist_under, hist_over;
	uint32_t hist[HIST_BINS];
	uint32_t bg_cycles_lo, bg_cycles_hi;       /* cycles the main thread got */
	uint32_t window_cycles_lo, window_cycles_hi;
	uint32_t bg_longest;       /* longest stretch taken from the main thread, cycles */
	uint32_t arm_cycles;       /* first PWM output enable (all units), cycles */
	uint32_t fault_after;      /* latched fault at the end (0 expected) */
	uint32_t react_cycles;     /* CASE_REACT: setChannelWatch to outputs stopped */
	uint32_t react_fault, react_reason;
	uint32_t outputs_on_end;
};

extern "C" {
__attribute__((used)) volatile report_t g_tov_report;
__attribute__((used)) volatile uint32_t g_tov_cmd;
}

#define BKP_BOOTS   (TAMP->BKP10R)
#define BKP_PENDING (TAMP->BKP29R)
#define BKP_ACK     (TAMP->BKP31R)

static const hrtim_tu_number_t units[] = {PWMA, PWMC, PWMD, PWME, PWMF};

static volatile uint32_t last_start, period_cyc;
static volatile bool recording;

static void backup_access(void)
{
	RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN | RCC_APB1ENR1_RTCAPBEN;
	(void)RCC->APB1ENR1;
	PWR->CR1 |= PWR_CR1_DBP;
}

static void quiet_pins(void)
{
	static const struct { GPIO_TypeDef *g; uint32_t pin; } p[] = {{GPIOA, 1}, {GPIOC, 1}};
	for (auto e : p) {
		e.g->BSRR = 1U << (e.pin + 16U);
		e.g->MODER = (e.g->MODER & ~(3U << (2U * e.pin))) | (1U << (2U * e.pin));
	}
	k_busy_wait(20);
	for (auto e : p) {
		e.g->MODER |= 3U << (2U * e.pin);
	}
}

static bool outputs_enabled(void)
{
	return (HRTIM1->sCommonRegs.OENR & 0xFFFU) != 0U;
}

/* The control task: records its start, then a fixed busy load. */
static void control_task(void)
{
	uint32_t now = DWT->CYCCNT;
	if (recording) {
		volatile report_t *r = &g_tov_report;
		if (last_start != 0U) {
			uint32_t gap = now - last_start;
			if (gap < r->gap_min) r->gap_min = gap;
			if (gap > r->gap_max) r->gap_max = gap;
			if (gap > period_cyc + period_cyc / 2U) r->gap_late++;
			int32_t dev = (int32_t)(gap - period_cyc);
			int32_t bin = dev / (int32_t)HIST_STEP_CYC + (int32_t)(HIST_BINS / 2U);
			if (dev < 0 && dev % (int32_t)HIST_STEP_CYC) bin--;
			if (bin < 0) r->hist_under++;
			else if (bin >= (int32_t)HIST_BINS) r->hist_over++;
			else r->hist[bin]++;
		}
		r->periods++;
	}
	last_start = now;
	uint32_t load = LOAD_US * (SystemCoreClock / 1000000U);
	while (DWT->CYCCNT - now < load) {
	}
}

static void run_scenario(uint32_t c)
{
	volatile report_t *r = &g_tov_report;
	bool s1 = (c == CASE_S1_50 || c == CASE_S1_100);
	uint32_t period_us = (c == CASE_S0_100 || c == CASE_S1_100) ? 100U : 50U;
	r->scenario = (c == CASE_REACT) ? 0U : (s1 ? 1U : 0U);
	r->period_us = period_us;
	r->load_us = LOAD_US;
	r->core_hz = SystemCoreClock;
	period_cyc = period_us * (SystemCoreClock / 1000000U);
	r->gap_min = 0xFFFFFFFFU;

	quiet_pins();
	spin.pwm.initFixedFrequency(200000);
	for (auto u : units) {
		spin.pwm.initUnit(u);
		spin.pwm.setDutyCycle(u, 0.5);
	}
	shield.sensors.enableDefaultTwistSensors();
	if (s1) {
		safety.initShield();   /* the old software check on every Twist sensor with a limit */
	}
#if SAFETY_DEVELOPMENT_MODE
	(void)safety.disarm();
#endif
	task.createCritical(control_task, period_us);   /* HRTIM source, as an application */
	task.startCritical(true);                       /* data acquisition with dispatch */
	uint32_t a0 = DWT->CYCCNT;
	for (auto u : units) {
		spin.pwm.startDualOutput(u);   /* the first enable arms the protection */
	}
	r->arm_cycles = DWT->CYCCNT - a0;
	k_msleep(200);   /* settle */

	/* Measurement window: the main thread counts the cycles it gets. */
	uint64_t window = (uint64_t)MEASURE_S * SystemCoreClock, got = 0U;
	uint32_t longest = 0U;
#ifdef CONFIG_OWNTECH_SAFETY_TIMING_PROBES
	g_safety_timing.reset = 1U;   /* the probe record covers the window only */
#endif
	recording = true;
	uint32_t start = DWT->CYCCNT, last = start;
	uint64_t elapsed = 0U;
	while (elapsed < window) {
		uint32_t now = DWT->CYCCNT;
		uint32_t d = now - last;
		last = now;
		elapsed += d;
		if (d < 64U) {
			got += d;
		} else if (d > longest) {
			longest = d;
		}
	}
	recording = false;
	r->expected = (uint32_t)(elapsed / period_cyc);
	r->bg_cycles_lo = (uint32_t)got;
	r->bg_cycles_hi = (uint32_t)(got >> 32);
	r->window_cycles_lo = (uint32_t)elapsed;
	r->window_cycles_hi = (uint32_t)(elapsed >> 32);
	r->bg_longest = longest;

#if NEW_SAFETY
	if (c == CASE_REACT) {
		/* Software trip under load: V_HIGH reads about 0 V, its min tightened to 99. */
		sensor_t vh = V_HIGH;
		safety.initShield(&vh, 0);   /* devicetree thresholds, nothing watched yet */
		float32_t x = 99.0f;
		(void)safety.setChannelThresholdMin(&vh, &x, 1);
		uint32_t t0 = DWT->CYCCNT;
		safety.setChannelWatch(&vh, 1);
		k_msleep(50);
		safety_fault_record_t rec;
		safety_supervision_get_record(&rec);
		r->react_cycles = rec.disable_cycles - t0;
		r->react_fault = (uint32_t)rec.fault;
		r->react_reason = rec.reason;
		safety.unsetChannelWatch(&vh, 1);
	}
	if (c != CASE_REACT) {
		safety_fault_record_t rec;
		safety_supervision_get_record(&rec);
		r->fault_after = (uint32_t)rec.fault;
		r->react_reason = rec.reason;   /* which limit, if a fault was latched */
		r->react_cycles = rec.pc;       /* the reading, for a software trip (float bits) */
	}
#endif
	r->outputs_on_end = outputs_enabled() ? 1U : 0U;
	for (auto u : units) {
		spin.pwm.stopDualOutput(u);
	}
	task.stopCritical();
}

int main(void)
{
	backup_access();
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
	volatile report_t *r = &g_tov_report;
	r->layout = 1U;
	BKP_BOOTS = BKP_BOOTS + 1U;
	r->boot_count = BKP_BOOTS;
#if NEW_SAFETY
	r->latched_at_boot = (uint32_t)safety_supervision_get_fault();
	r->build = SAFETY_DEVELOPMENT_MODE ? 2U : 1U;
	if (r->latched_at_boot != 0U) {
		(void)safety.clearFault();
	}
#else
	r->build = 0U;
#endif
	uint32_t pending = BKP_PENDING;
	r->pending = pending;
	r->ack = BKP_ACK;
	r->magic = REPORT_MAGIC;
	uint32_t last = BKP_ACK;
	if (pending != 0U) {
		last = pending >> 8;
		r->busy = last;
		BKP_PENDING = 0U;
		run_scenario(pending & 0xFFU);
		BKP_ACK = last;
		r->busy = 0U;
		r->ack = last;
	}
	g_tov_cmd = (last << 8) | 0xFFU;

	for (;;) {
		uint32_t cmd = g_tov_cmd, seq = cmd >> 8, c = cmd & 0xFFU;
		if (seq != last && seq != 0U) {
			last = seq;
			r->busy = seq;
			if (c == CASE_CLEAR) {
				BKP_ACK = seq;
				r->busy = 0U;
				r->ack = seq;
			} else {
				/* Every scenario starts from a fresh boot: the first PWM enable
				 * arms the protection, and Core main cannot create the control
				 * task twice. */
				BKP_PENDING = (seq << 8) | c;
				sys_reboot(SYS_REBOOT_COLD);
			}
		}
		k_msleep(10);
	}
	return 0;
}
