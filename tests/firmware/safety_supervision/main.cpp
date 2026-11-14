/*
 * Copyright (c) 2026-present LAAS-CNRS
 *
 * SPDX-License-Identifier: LGPL-2.1
 */

/**
 * @file main.cpp
 * @brief Safety test firmware for CPU supervision (ADR 0009, issue #16).
 *
 * Runs on a bare Spin. The host harness (tests/l0/safety_supervision/run.py)
 * drives it over SWD: it writes a command into g_safety_sup_cmd, the firmware
 * runs that case and acknowledges it in g_safety_sup_report. A case that ends
 * in a reset (fatal error, watchdog) leaves a pending mark in the TAMP backup
 * registers; after the reboot the firmware evaluates the latched fault, writes
 * the results and acknowledges. Results live in backup registers, so they
 * survive the resets, and are mirrored into the RAM report.
 *
 * Never flash this firmware on a power board connected to a source.
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/reboot.h>
#include <soc.h>

#include "SpinAPI.h"
#include "TaskAPI.h"
#include "safety_supervision.h"

/* ---- Protocol (mirrored in tests/l0/safety_supervision/report.py) ------- */

#define REPORT_MAGIC   0x31505553U  /* "SUP1" */
#define REPORT_LAYOUT  3U
#define N_RESULTS      18U

enum sup_case : uint32_t {
	CASE_CLEAR            = 0,  /* wipe results, pending mark and latch */
	CASE_CONFIG           = 1,  /* arming: SYSCFG, PVD, IWDG, HRTIM fault states */
	CASE_NO_FALSE_RESET   = 2,  /* 2 s fed by the kernel timer, 2 s by a 50 us control task */
	CASE_SLOW_TASK        = 3,  /* 2 ms control task: longer timeout, no reset for 1 s */
	CASE_FATAL_STACK      = 4,  /* thread stack overflow (MPU guard) */
	CASE_FATAL_HARDFAULT  = 5,  /* undefined instruction in a thread */
	CASE_CONTROL_STUCK    = 6,  /* control task never returns */
	CASE_IRQ_MASKED       = 7,  /* interrupts masked forever, no control task */
	CASE_LOCKUP           = 8,  /* Cortex-M4 lockup (fault with FAULTMASK set) */
	CASE_SOFT_RESET_LATCH = 9,  /* a software reset keeps the latched fault */
};

/* Result words (index into results[]). */
enum {
	R_CONFIG = 0,          /* b0-3 SYSCFG_CFGR2 CLL/SPL/PVDL/ECCL, b4 PVDE, b5 every fault state Inactive,
	                          b8-10 IWDG PR, b16-27 IWDG RLR */
	R_TIMEOUT_DEFAULT = 1, /* declared watchdog timeout before any control task, us */
	R_NO_FALSE_RESET = 2,  /* b0 survived 2 s on the kernel timer, b1 survived 2 s on the control task */
	R_SLOW_TASK = 3,       /* b0 survived 1 s; b8-31 declared timeout with a 2 ms task, us */
	R_STACK_FLAGS = 4,     /* per fatal case: b0-7 latched fault, b8 PWM refused while latched, */
	R_STACK_REASON = 5,    /* b9 clear accepted, b10 PWM re-enabled after clear, b11 clear refused */
	R_STACK_OUTPUTS = 6,   /*     while PWM enabled; reason; pin levels after disabling; */
	R_STACK_CYCLES = 7,    /*     cycles from the mark to the outputs disabled */
	R_HARD_FLAGS = 8,
	R_HARD_REASON = 9,
	R_HARD_OUTPUTS = 10,
	R_HARD_CYCLES = 11,
	R_STUCK_FLAGS = 12,    /* watchdog cases: b0-7 latched fault, b8-11 as above */
	R_IRQ_FLAGS = 13,
	R_LOCKUP_FLAGS = 14,
	R_SOFTRESET_FLAGS = 15,/* b0-7 latched fault after the fatal error, b8 still latched after a software reset,
	                          b9 clear accepted */
	R_OUTPUT_MASK = 16,    /* PWM outputs switching during the cases (bit n = HRTIM output n) */
	R_CORE_HZ = 17,
};

struct sup_report_t {
	uint32_t magic;
	uint32_t layout;
	uint32_t boot_count;
	uint32_t ack;          /* last command sequence completed */
	uint32_t busy;         /* sequence being run, 0 when idle */
	uint32_t latched_at_boot;
	uint32_t pending;      /* pending mark found at boot */
	uint32_t fault_reason; /* record of the fault latched at boot (safety_fault_record_t) */
	uint32_t fault_pc;
	uint32_t fault_thread;
	uint32_t fault_context;
	uint32_t fault_stack;
	uint32_t fault_lr;
	uint32_t results[N_RESULTS];
};

extern "C" {
__attribute__((used)) volatile sup_report_t g_safety_sup_report;
__attribute__((used)) volatile uint32_t g_safety_sup_cmd;  /* (sequence << 8) | case */
}

/* Backup registers: CPU supervision owns BKP0-9. */
#define BKP_BOOTS    (TAMP->BKP10R)
#define BKP_PENDING  (TAMP->BKP29R)  /* (sequence << 8) | case, plus stage in bits 4-7 of the case byte */
#define BKP_ACK      (TAMP->BKP31R)  /* last sequence acknowledged, survives resets */
static volatile uint32_t *const bkp_results = &TAMP->BKP11R; /* BKP11..BKP28 */

/* PWM units used: A, C, D, E, F (B shares PA11/PA12 with USB). */
static const hrtim_tu_number_t units[] = {PWMA, PWMC, PWMD, PWME, PWMF};
static const uint32_t OUTPUT_MASK = 0xFF3U;   /* TA1 TA2 TC1 TC2 TD1 TD2 TE1 TE2 TF1 TF2 */
#define HRTIM_ALL_OUTPUTS 0xFFFU

static void result_set(uint32_t i, uint32_t v)
{
	bkp_results[i] = v;
	g_safety_sup_report.results[i] = v;
}

static void pwm_start_all(void)
{
	static bool initialised;

	if (!initialised) {
		spin.pwm.initFixedFrequency(200000);
		for (auto u : units) {
			spin.pwm.initUnit(u);
			spin.pwm.setDutyCycle(u, 0.5);
		}
		initialised = true;
	}
	for (auto u : units) {
		spin.pwm.startDualOutput(u);
	}
}

static void pwm_stop_all(void)
{
	for (auto u : units) {
		spin.pwm.stopDualOutput(u);
	}
}

static bool outputs_enabled(void)
{
	return (HRTIM1->sCommonRegs.OENR & HRTIM_ALL_OUTPUTS) != 0U;
}

static void set_pending(uint32_t seq, uint32_t c, uint32_t stage)
{
	BKP_PENDING = (seq << 8) | (stage << 4) | c;
}

static void ack(uint32_t seq)
{
	BKP_ACK = seq;
	g_safety_sup_report.busy = 0U;
	g_safety_sup_report.ack = seq;
}

/* After a latched fault: refused while latched, clear refused while PWM on is
 * not testable (PWM cannot be on), clear accepted, PWM re-enabled. */
static uint32_t latch_checks(void)
{
	uint32_t flags = (uint32_t)safety_supervision_get_fault() & 0xFFU;

	pwm_start_all();
	if (!outputs_enabled()) {
		flags |= 1U << 8;
	}
	if (safety_supervision_clear_fault() == 0) {
		flags |= 1U << 9;
	}
	pwm_start_all();
	if (outputs_enabled()) {
		flags |= 1U << 10;
		if (safety_supervision_clear_fault() != 0) {
			flags |= 1U << 11;   /* clear refused while outputs are enabled */
		}
	}
	pwm_stop_all();
	return flags;
}

/* ---- Cases that return ----------------------------------------------------- */

static void case_config(void)
{
	pwm_start_all();   /* arms CPU supervision */
	uint32_t v = SYSCFG->CFGR2 & 0xFU;

	if ((PWR->CR2 & PWR_CR2_PVDE) != 0U) {
		v |= 1U << 4;
	}
	/* FAULT1/FAULT2 = 10 (Inactive) in OUTxR of timers A, C, D, E, F. */
	bool inactive = true;
	const uint32_t timers[] = {0, 2, 3, 4, 5};
	for (uint32_t t : timers) {
		uint32_t outr = HRTIM1->sTimerxRegs[t].OUTxR;
		if (((outr & HRTIM_OUTR_FAULT1) != HRTIM_OUTR_FAULT1_1) ||
		    ((outr & HRTIM_OUTR_FAULT2) != HRTIM_OUTR_FAULT2_1)) {
			inactive = false;
		}
	}
	if (inactive) {
		v |= 1U << 5;
	}
	v |= (IWDG->PR & 0x7U) << 8;
	v |= (IWDG->RLR & 0xFFFU) << 16;
	result_set(R_CONFIG, v);
	result_set(R_TIMEOUT_DEFAULT, safety_supervision_watchdog_timeout_us());
	result_set(R_OUTPUT_MASK, OUTPUT_MASK);
	result_set(R_CORE_HZ, SystemCoreClock);
	pwm_stop_all();
}

static volatile uint32_t control_calls;
static volatile bool control_stuck;

static void control_task(void)
{
	control_calls++;
	if (control_stuck) {
		for (;;) {
		}
	}
}

static void case_no_false_reset(void)
{
	uint32_t v = 0U;

	pwm_start_all();
	k_msleep(2000);   /* fed by the kernel timer */
	v |= 1U << 0;
	result_set(R_NO_FALSE_RESET, v);
	task.createCritical(control_task, 50, source_tim6);
	task.startCritical(false);
	k_msleep(2000);   /* fed by the control task */
	task.stopCritical();
	if (control_calls > 30000U) {
		v |= 1U << 1;
	}
	result_set(R_NO_FALSE_RESET, v);
	pwm_stop_all();
}

static void case_slow_task(void)
{
	pwm_start_all();
	task.createCritical(control_task, 2000, source_tim6);
	task.startCritical(false);
	uint32_t timeout = safety_supervision_watchdog_timeout_us();
	k_msleep(1000);
	task.stopCritical();
	result_set(R_SLOW_TASK, 1U | (timeout << 8));
	pwm_stop_all();
}

/* ---- Cases that end in a reset ---------------------------------------------- */

K_THREAD_STACK_DEFINE(victim_stack, 512);
static struct k_thread victim_thread;

/* Unbounded on purpose: it must run into the MPU stack guard. */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Winfinite-recursion"
static int __attribute__((noinline)) recurse(int depth)
{
	volatile uint8_t buf[64];

	for (unsigned i = 0; i < sizeof(buf); i++) {
		buf[i] = (uint8_t)depth;
	}
	/* Mark just before the call whose frame lands in the stack guard: the
	 * measured time is the fault reaction, not the recursion. */
	safety_supervision_mark(DWT->CYCCNT);
	return recurse(depth + 1) + buf[depth & 63];
}
#pragma GCC diagnostic pop

static void victim_stack_overflow(void *, void *, void *)
{
	(void)recurse(0);
}

static void victim_undefined(void *, void *, void *)
{
	safety_supervision_mark(DWT->CYCCNT);
	__builtin_trap();
}

static void run_victim(k_thread_entry_t entry)
{
	k_thread_create(&victim_thread, victim_stack, K_THREAD_STACK_SIZEOF(victim_stack),
			entry, NULL, NULL, NULL, K_PRIO_PREEMPT(1), 0, K_NO_WAIT);
	k_msleep(1000);   /* a reset happens long before */
}

static void provoke(uint32_t seq, uint32_t c)
{
	pwm_start_all();
	k_msleep(20);
	set_pending(seq, c, 0U);
	switch (c) {
	case CASE_FATAL_STACK:
		run_victim(victim_stack_overflow);
		break;
	case CASE_FATAL_HARDFAULT:
		run_victim(victim_undefined);
		break;
	case CASE_CONTROL_STUCK:
		control_stuck = false;
		task.createCritical(control_task, 100, source_tim6);
		task.startCritical(false);
		k_msleep(100);
		control_stuck = true;
		k_msleep(1000);
		break;
	case CASE_IRQ_MASKED:
		(void)irq_lock();
		for (;;) {
		}
		break;
	case CASE_LOCKUP:
		__set_FAULTMASK(1U);
		__builtin_trap();
		break;
	case CASE_SOFT_RESET_LATCH:
		k_panic();
		break;
	default:
		break;
	}
	/* Reaching here means no reset happened: leave the pending mark so the
	 * evaluation after the next boot reports the missing fault. */
	BKP_PENDING = 0U;
	result_set(c == CASE_FATAL_STACK ? R_STACK_FLAGS : c == CASE_FATAL_HARDFAULT ? R_HARD_FLAGS :
		   c == CASE_CONTROL_STUCK ? R_STUCK_FLAGS : c == CASE_IRQ_MASKED ? R_IRQ_FLAGS :
		   c == CASE_LOCKUP ? R_LOCKUP_FLAGS : R_SOFTRESET_FLAGS, 0xFFU);
	pwm_stop_all();
	ack(seq);
}

/* Evaluation after the reset caused by a pending case. */
static void evaluate_pending(uint32_t pending)
{
	uint32_t seq = pending >> 8;
	uint32_t c = pending & 0xFU;
	uint32_t stage = (pending >> 4) & 0xFU;
	safety_fault_record_t rec;

	safety_supervision_get_record(&rec);
	BKP_PENDING = 0U;

	switch (c) {
	case CASE_FATAL_STACK:
	case CASE_FATAL_HARDFAULT: {
		bool stack = (c == CASE_FATAL_STACK);
		uint32_t mark = safety_supervision_get_mark();
		result_set(stack ? R_STACK_REASON : R_HARD_REASON, rec.reason);
		result_set(stack ? R_STACK_OUTPUTS : R_HARD_OUTPUTS, rec.outputs_after);
		result_set(stack ? R_STACK_CYCLES : R_HARD_CYCLES, rec.disable_cycles - mark);
		result_set(stack ? R_STACK_FLAGS : R_HARD_FLAGS, latch_checks());
		break;
	}
	case CASE_CONTROL_STUCK:
		result_set(R_STUCK_FLAGS, latch_checks());
		break;
	case CASE_IRQ_MASKED:
		result_set(R_IRQ_FLAGS, latch_checks());
		break;
	case CASE_LOCKUP:
		result_set(R_LOCKUP_FLAGS, latch_checks());
		break;
	case CASE_SOFT_RESET_LATCH:
		if (stage == 0U) {
			result_set(R_SOFTRESET_FLAGS, (uint32_t)safety_supervision_get_fault() & 0xFFU);
			set_pending(seq, c, 1U);
			sys_reboot(SYS_REBOOT_COLD);   /* software reset */
		} else {
			uint32_t v = bkp_results[R_SOFTRESET_FLAGS] & 0xFFU;
			if (safety_supervision_get_fault() != SAFETY_FAULT_NONE) {
				v |= 1U << 8;
			}
			if (safety_supervision_clear_fault() == 0) {
				v |= 1U << 9;
			}
			result_set(R_SOFTRESET_FLAGS, v);
		}
		break;
	default:
		break;
	}
	ack(seq);
}

static void clear_all(void)
{
	for (uint32_t i = 0; i < N_RESULTS; i++) {
		result_set(i, 0U);
	}
	BKP_PENDING = 0U;
	(void)safety_supervision_clear_fault();
}

int main(void)
{
	g_safety_sup_report.layout = REPORT_LAYOUT;
	BKP_BOOTS = BKP_BOOTS + 1U;
	g_safety_sup_report.boot_count = BKP_BOOTS;
	g_safety_sup_report.latched_at_boot = (uint32_t)safety_supervision_get_fault();
	for (uint32_t i = 0; i < N_RESULTS; i++) {
		g_safety_sup_report.results[i] = bkp_results[i];
	}
	safety_fault_record_t rec;
	safety_supervision_get_record(&rec);
	g_safety_sup_report.fault_reason = rec.reason;
	g_safety_sup_report.fault_pc = rec.pc;
	g_safety_sup_report.fault_thread = rec.thread;
	g_safety_sup_report.fault_context = rec.context;
	g_safety_sup_report.fault_stack = rec.stack;
	g_safety_sup_report.fault_lr = rec.lr;
	uint32_t pending = BKP_PENDING;
	g_safety_sup_report.pending = pending;
	g_safety_sup_report.ack = BKP_ACK;
	g_safety_sup_report.magic = REPORT_MAGIC;

	uint32_t last = BKP_ACK;
	if (pending != 0U) {
		last = pending >> 8;
		g_safety_sup_report.busy = last;
		evaluate_pending(pending);
	}
	/* The command word survives no reset: start from the last acknowledged. */
	g_safety_sup_cmd = (last << 8) | 0xFFU;

	for (;;) {
		uint32_t cmd = g_safety_sup_cmd;
		uint32_t seq = cmd >> 8;
		uint32_t c = cmd & 0xFFU;

		if (seq != last && seq != 0U) {
			last = seq;
			g_safety_sup_report.busy = seq;
			switch (c) {
			case CASE_CLEAR: clear_all(); ack(seq); break;
			case CASE_CONFIG: case_config(); ack(seq); break;
			case CASE_NO_FALSE_RESET: case_no_false_reset(); ack(seq); break;
			case CASE_SLOW_TASK: case_slow_task(); ack(seq); break;
			default: provoke(seq, c); break;
			}
		}
		k_msleep(10);
	}
	return 0;
}
