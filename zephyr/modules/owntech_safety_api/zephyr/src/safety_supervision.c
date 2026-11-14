/*
 * Copyright (c) 2026-present LAAS-CNRS
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
 * @file safety_supervision.c
 * @brief CPU supervision (ADR 0009): independent watchdog, fatal-error handler,
 *        HRTIM system fault, and the fault latched across resets.
 *
 * References: RM0440 Rev 8 §27.3.15 (HRTIM fault protection, system fault),
 * §10.2.8 SYSCFG_CFGR2 (CLL, SPL, PVDL, ECCL), §33 (IWDG), §36.6.8 (TAMP
 * backup registers, not affected by a system reset).
 */

#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/fatal.h>
#include <zephyr/sys/printk.h>
#include <soc.h>

#include "safety_supervision.h"

/* ---- Backup-register record (TAMP_BKPxR survive a system reset) -------- */

#define BKP_MAGIC        0x5AFE0000U  /* upper half of BKP_FAULT when a record is valid */
#define BKP_FAULT        (TAMP->BKP0R)  /* magic | safety_fault_t */
#define BKP_REASON       (TAMP->BKP1R)  /* Zephyr K_ERR_* reason */
#define BKP_PC           (TAMP->BKP2R)  /* faulting PC */
#define BKP_DISABLE_CYC  (TAMP->BKP3R)  /* DWT cycles when outputs were disabled */
#define BKP_OUTPUTS      (TAMP->BKP4R)  /* PWM pin levels after disabling */
#define BKP_MARK         (TAMP->BKP5R)  /* test hook: DWT cycles just before the fault */
#define BKP_THREAD       (TAMP->BKP6R)  /* current thread (struct k_thread *) at the fault */
#define BKP_CONTEXT      (TAMP->BKP7R)  /* IPSR (active exception) | bit 31: in ISR */
#define BKP_STACK        (TAMP->BKP8R)  /* current thread stack: start (bits 31-12 kept) | size / 8 (bits 11-0) */
#define BKP_LR           (TAMP->BKP9R)  /* link register of the fault frame (caller of an assert or panic) */

/* ---- Watchdog ----------------------------------------------------------- */

/* IWDG timeout = 4 * 2^PR * (RLR + 1) / f_LSI, with f_LSI from 29.5 to 34 kHz
 * (DS12288). Default: PR = 0, RLR = 6, i.e. 0.82 ms (34 kHz) to 0.95 ms
 * (29.5 kHz), below the declared 1 ms. */
#define LSI_MIN_HZ          29500U
#define LSI_MAX_HZ          34000U
#define IWDG_PRESCALER_DIV4 0U
#define IWDG_RELOAD         6U
#define IWDG_RELOAD_MAX     4095U
#define IWDG_KEY_START      0xCCCCU
#define IWDG_KEY_UNLOCK     0x5555U
#define IWDG_KEY_FEED       0xAAAAU

/* PVD threshold: PLS = 101, about 2.8 V falling (DS12288, PVD characteristics). */
#define PVD_LEVEL_PLS       (5U << PWR_CR2_PLS_Pos)

#define HRTIM_ALL_OUTPUTS   0xFFFU      /* TA1..TF2 in ODISR / OENR */

static volatile bool armed;
static volatile uint32_t first_disable_cycles;  /* earliest fault-path disable, this boot */
static uint32_t iwdg_pr = IWDG_PRESCALER_DIV4;
static uint32_t iwdg_rlr = IWDG_RELOAD;
static volatile bool critical_feeder;
static volatile safety_fault_t latched;
static bool latched_reported;

static void feed_timer_fn(struct k_timer *timer)
{
	ARG_UNUSED(timer);
	if (!critical_feeder) {
		IWDG->KR = IWDG_KEY_FEED;
	}
}

K_TIMER_DEFINE(feed_timer, feed_timer_fn, NULL);

static void backup_access_enable(void)
{
	RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN | RCC_APB1ENR1_RTCAPBEN;
	(void)RCC->APB1ENR1;
	PWR->CR1 |= PWR_CR1_DBP;
}

static void record_clear(void)
{
	BKP_FAULT = 0U;
	BKP_REASON = 0U;
	BKP_PC = 0U;
	BKP_DISABLE_CYC = 0U;
	BKP_OUTPUTS = 0U;
	BKP_THREAD = 0U;
	BKP_CONTEXT = 0U;
	BKP_STACK = 0U;
	BKP_LR = 0U;
}

static safety_fault_t record_fault(void)
{
	uint32_t word = BKP_FAULT;

	if ((word & 0xFFFF0000U) != BKP_MAGIC) {
		return SAFETY_FAULT_NONE;
	}
	return (safety_fault_t)(word & 0xFFU);
}

/* PWM pin levels, bit n = HRTIM output n in ODISR order:
 * TA1 PA8, TA2 PA9, TB1 PA10, TB2 PA11, TC1 PB12, TC2 PB13, TD1 PB14,
 * TD2 PB15, TE1 PC8, TE2 PC9, TF1 PC6, TF2 PC7. */
static uint32_t pwm_pin_levels(void)
{
	uint32_t a = GPIOA->IDR, b = GPIOB->IDR, c = GPIOC->IDR;

	return ((a >> 8) & 0xFU)            /* PA8..PA11  -> bits 0..3  */
	     | (((b >> 12) & 0xFU) << 4)    /* PB12..PB15 -> bits 4..7  */
	     | (((c >> 8) & 0x3U) << 8)     /* PC8, PC9   -> bits 8, 9  */
	     | (((c >> 6) & 0x3U) << 10);   /* PC6, PC7   -> bits 10, 11 */
}

/* ---- Public API --------------------------------------------------------- */

void safety_outputs_disable_all(void)
{
	HRTIM1->sCommonRegs.ODISR = HRTIM_ALL_OUTPUTS;
}

void safety_supervision_feed(void)
{
	if (armed) {
		IWDG->KR = IWDG_KEY_FEED;
	}
}

/* Smallest prescaler and reload whose shortest timeout (LSI at its fastest)
 * still covers 3 control periods; never shorter than the default. */
static void iwdg_timing_for_period(uint32_t period_us)
{
	uint64_t need_us = 3ULL * period_us;

	iwdg_pr = IWDG_PRESCALER_DIV4;
	iwdg_rlr = IWDG_RELOAD;
	for (uint32_t pr = 0U; pr <= 6U; pr++) {
		uint64_t div = 4ULL << pr;
		/* ticks so that ticks * div / LSI_MAX >= need_us */
		uint64_t ticks = (need_us * LSI_MAX_HZ + div * 1000000ULL - 1U)
				 / (div * 1000000ULL);
		if (ticks <= (uint64_t)IWDG_RELOAD_MAX + 1U) {
			if (pr > iwdg_pr || ticks - 1U > iwdg_rlr) {
				iwdg_pr = pr;
				iwdg_rlr = (uint32_t)(ticks - 1U);
				if (pr == IWDG_PRESCALER_DIV4 && iwdg_rlr < IWDG_RELOAD) {
					iwdg_rlr = IWDG_RELOAD;
				}
			}
			return;
		}
	}
	iwdg_pr = 6U;
	iwdg_rlr = IWDG_RELOAD_MAX;
}

static void iwdg_apply(void)
{
	IWDG->KR = IWDG_KEY_UNLOCK;
	IWDG->PR = iwdg_pr;
	IWDG->RLR = iwdg_rlr;
	while (IWDG->SR != 0U) {
	}
	IWDG->KR = IWDG_KEY_FEED;
}

uint32_t safety_supervision_watchdog_timeout_us(void)
{
	return (uint32_t)((4ULL << iwdg_pr) * (iwdg_rlr + 1U) * 1000000ULL / LSI_MIN_HZ);
}

void safety_supervision_feeder_attach(uint32_t period_us)
{
	iwdg_timing_for_period(period_us);
	if (armed) {
		iwdg_apply();
	}
	critical_feeder = true;
}

void safety_supervision_feeder_detach(void)
{
	critical_feeder = false;
}

bool safety_supervision_armed(void)
{
	return armed;
}

safety_fault_t safety_supervision_get_fault(void)
{
	return latched;
}

void safety_supervision_get_record(safety_fault_record_t *record)
{
	if (record == NULL) {
		return;
	}
	record->fault = latched;
	record->reason = BKP_REASON;
	record->pc = BKP_PC;
	record->disable_cycles = BKP_DISABLE_CYC;
	record->outputs_after = BKP_OUTPUTS;
	record->thread = BKP_THREAD;
	record->context = BKP_CONTEXT;
	record->stack = BKP_STACK;
	record->lr = BKP_LR;
}

int safety_supervision_clear_fault(void)
{
	if ((HRTIM1->sCommonRegs.OENR & HRTIM_ALL_OUTPUTS) != 0U) {
		return -1;
	}
	record_clear();
	latched = SAFETY_FAULT_NONE;
	latched_reported = false;
	return 0;
}

void safety_supervision_mark(uint32_t cycles)
{
	BKP_MARK = cycles;
}

uint32_t safety_supervision_get_mark(void)
{
	return BKP_MARK;
}

/* ---- Arming: called before any HRTIM output is enabled ------------------- */

static void arm(void)
{
	/* HRTIM system fault sources (RM0440 SYSCFG_CFGR2): lockup, SRAM parity,
	 * flash ECC, PVD. Each bit is write-once until reset. PVD is enabled and
	 * its level set before PVDL freezes them. */
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
	(void)RCC->APB2ENR;
	PWR->CR2 = (PWR->CR2 & ~(PWR_CR2_PLS_Msk)) | PVD_LEVEL_PLS | PWR_CR2_PVDE;
	SYSCFG->CFGR2 |= SYSCFG_CFGR2_CLL | SYSCFG_CFGR2_SPL
	               | SYSCFG_CFGR2_ECCL | SYSCFG_CFGR2_PVDL;

	/* Independent watchdog: once started, only a reset stops it. */
	IWDG->KR = IWDG_KEY_START;
	iwdg_apply();

	/* Until a control task attaches, a kernel timer feeds the watchdog. */
	k_timer_start(&feed_timer, K_USEC(200), K_USEC(200));
	armed = true;
}

/**
 * Called by the HRTIM driver before it enables any output (strong override of
 * its weak default). Arms CPU supervision the first time, and refuses while a
 * fault is latched.
 */
bool hrtim_safety_output_enable_allowed(void)
{
	if (latched != SAFETY_FAULT_NONE) {
		if (!latched_reported) {
			latched_reported = true;
			printk("safety: PWM refused, CPU-supervision fault latched "
			       "(%s). Clear it with safety.clearFault() or a power cycle.\n",
			       latched == SAFETY_FAULT_WATCHDOG ? "watchdog" : "fatal error");
		}
		return false;
	}
	if (!armed) {
		arm();
	}
	return true;
}

/* ---- Earliest fault entry (linker --wrap, see CMakeLists.txt) ------------- */

/*
 * Zephyr formats and logs the exception (CONFIG_EXCEPTION_DEBUG) and the fatal
 * error before it calls k_sys_fatal_error_handler(): measured on the Spin at
 * 250 to 300 us, during which the PWM kept switching (L0-SUP-04.d, 05.d). The
 * two C entry points of the fault path are wrapped so that every HRTIM output
 * is disabled first: z_arm_fault() for CPU exceptions (hard fault, MPU stack
 * guard, usage fault...), z_fatal_error() for k_panic(), k_oops() and failed
 * asserts. Any CPU fault therefore stops the PWM, including the ones Zephyr
 * could recover from (userspace and debug-monitor faults, unused by OwnTech).
 */
void __real_z_arm_fault(uint32_t msp, uint32_t psp, uint32_t exc_return,
			_callee_saved_t *callee_regs);
void __real_z_fatal_error(unsigned int reason, const struct arch_esf *esf);

void __wrap_z_arm_fault(uint32_t msp, uint32_t psp, uint32_t exc_return,
			_callee_saved_t *callee_regs)
{
	safety_outputs_disable_all();
	if (first_disable_cycles == 0U) {
		first_disable_cycles = DWT->CYCCNT;
	}
	__real_z_arm_fault(msp, psp, exc_return, callee_regs);
}

void __wrap_z_fatal_error(unsigned int reason, const struct arch_esf *esf)
{
	safety_outputs_disable_all();
	if (first_disable_cycles == 0U) {
		first_disable_cycles = DWT->CYCCNT;
	}
	__real_z_fatal_error(reason, esf);
}

/* ---- Fatal-error handler ------------------------------------------------- */

/**
 * Overrides Zephyr's weak handler. The outputs were already disabled at the
 * fault entry; disabling again costs nothing and covers any other caller. The CPU then halts: if CPU supervision is
 * armed, the watchdog resets the MCU within its timeout, and the fault stays
 * latched across that reset.
 */
FUNC_NORETURN void k_sys_fatal_error_handler(unsigned int reason,
					     const struct arch_esf *esf)
{
	safety_outputs_disable_all();
	uint32_t cycles = (first_disable_cycles != 0U) ? first_disable_cycles : DWT->CYCCNT;
	uint32_t levels = pwm_pin_levels();

	backup_access_enable();
	BKP_FAULT = BKP_MAGIC | (uint32_t)SAFETY_FAULT_FATAL_ERROR;
	BKP_REASON = reason;
	BKP_PC = (esf != NULL) ? esf->basic.pc : 0U;
	BKP_LR = (esf != NULL) ? esf->basic.lr : 0U;
	BKP_DISABLE_CYC = cycles;
	BKP_OUTPUTS = levels;
	struct k_thread *thread = k_current_get();
	BKP_THREAD = (uint32_t)thread;
	BKP_CONTEXT = __get_IPSR() | (k_is_in_isr() ? (1U << 31) : 0U);
#if defined(CONFIG_THREAD_STACK_INFO)
	if (thread != NULL) {
		BKP_STACK = (thread->stack_info.start & ~0xFFFU)
			  | ((thread->stack_info.size / 8U) & 0xFFFU);
	}
#endif

	printk("safety: fatal error %u, every PWM output disabled\n", reason);

	(void)arch_irq_lock();
	for (;;) {
	}
	CODE_UNREACHABLE;
}

/* ---- Boot: read the record and the reset cause ---------------------------- */

static int safety_supervision_boot(void)
{
	uint32_t csr = RCC->CSR;

	/* Cycle counter for reaction-time records. */
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

	backup_access_enable();

	if ((csr & RCC_CSR_BORRSTF) != 0U) {
		/* Power-on or brown-out reset: a power cycle clears the latch. */
		record_clear();
	} else if (((csr & RCC_CSR_IWDGRSTF) != 0U) &&
		   (record_fault() == SAFETY_FAULT_NONE)) {
		BKP_FAULT = BKP_MAGIC | (uint32_t)SAFETY_FAULT_WATCHDOG;
	}
	RCC->CSR |= RCC_CSR_RMVF;

	latched = record_fault();
	if (latched != SAFETY_FAULT_NONE) {
		printk("safety: CPU-supervision fault latched from the previous run "
		       "(%s, reason %u, pc 0x%08x). PWM stays disabled until "
		       "safety.clearFault() or a power cycle.\n",
		       latched == SAFETY_FAULT_WATCHDOG ? "watchdog" : "fatal error",
		       BKP_REASON, BKP_PC);
	}
	return 0;
}

SYS_INIT(safety_supervision_boot, APPLICATION, 0);
