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
 * @file safety_selftest.c
 * @brief Class B self-tests (IEC 60730-1 Annex H, Table H.2; ADR 0009, #21).
 *
 * At start-up (always, before the kernel starts threads):
 *   1.1 CPU registers   R1-R12 with 0xAAAAAAAA / 0x55555555 (safety_cpu_test.S)
 *   4.2 variable memory March C- over all of SRAM, transparent (each block is
 *                       saved, tested, restored with interrupts off), except
 *                       the running stack window and the backup buffer
 *   4.1 invariable mem. CRC-32 of the image (hardware CRC unit) as the reference
 *                       for the run-time check; MCUboot checks the image at boot
 *   3   clock           system clock against the independent LSI, measured by
 *                       TIM5 input capture (tim_ti1_in1 = LSI, RM0440 Table 272)
 *   7.2 A/D             VREFINT conversion on ADC1 against its factory value
 * At run time (CONFIG_OWNTECH_SAFETY_RUNTIME_SELFTEST, every 10 ms in the
 * system work queue): CPU registers, one RAM block of the image's .data/.bss
 * (the heap, which holds the DMA buffers, is left out: a DMA write during the
 * test would read as a failure), 1 KiB of flash CRC, and the clock every second.
 * Program flow and interrupt rate are checked in the control task
 * (safety_supervision.c), sensor plausibility in safety_data.cpp.
 *
 * A failure latches SAFETY_FAULT_SELFTEST (cleared only by a reboot, which runs
 * the tests again) and stops the converter (safety_emergency_stop()).
 */

#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/sys/printk.h>
#include <soc.h>

#include "safety_supervision.h"
#include "safety_states.h"
#include "safety_selftest.h"

extern int safety_cpu_reg_test(void);
extern char __rom_region_start[];
extern char __rom_region_end[];
extern char _image_ram_start[];
extern char _image_ram_end[];

#define BLOCK_WORDS        16U
#define SRAM_START         ((uintptr_t)CONFIG_SRAM_BASE_ADDRESS)
#define SRAM_END           (SRAM_START + (uintptr_t)CONFIG_SRAM_SIZE * 1024U)
#define STACK_GUARD_BELOW  2048U
#define STACK_GUARD_ABOVE  512U
#define VREFINT_CAL        (*(const uint16_t *)0x1FFF75AAU)   /* at 3.0 V, 30 C (DS12288) */
#define LSI_MIN_HZ         29500U
#define LSI_MAX_HZ         34000U
#define FLASH_SLICE_BYTES  1024U

__attribute__((used)) volatile struct safety_selftest_status g_safety_selftest;

static uint32_t backup[BLOCK_WORDS];
static volatile uint32_t inject;   /* test hooks only (CONFIG_OWNTECH_SAFETY_TEST_HOOKS) */

/* ---- helpers --------------------------------------------------------------- */

static uint32_t us_since(uint32_t c0)
{
	return (DWT->CYCCNT - c0) / (SystemCoreClock / 1000000U);
}

static void fail(uint32_t id, const char *what)
{
	g_safety_selftest.last_failure = id;
	safety_supervision_latch(SAFETY_FAULT_SELFTEST, id);
	safety_emergency_stop();
	printk("safety: self-test failed: %s (ADR 0009). Reboot to test again.\n", what);
}

/* March C- on n words with interrupts fully off (zero-latency interrupts too:
 * the control task could otherwise read its own data mid-test). */
static bool march_block(volatile uint32_t *p, size_t n)
{
	uint32_t primask = __get_PRIMASK();
	bool ok = true;

	__disable_irq();
	/* The MPU guards each thread's stack bottom (CONFIG_MPU_STACK_GUARD), in
	 * .bss: off for this block only, interrupts off. */
	uint32_t mpu = MPU->CTRL;
	MPU->CTRL = 0U;
	__DSB();
	__ISB();
	for (size_t i = 0; i < n; i++) {
		backup[i] = p[i];
	}
	for (size_t i = 0; i < n; i++) {
		p[i] = 0U;
	}
	for (size_t i = 0; i < n; i++) {
		ok &= (p[i] == 0U);
		p[i] = 0xFFFFFFFFU;
	}
	for (size_t i = 0; i < n; i++) {
		ok &= (p[i] == 0xFFFFFFFFU);
		p[i] = 0U;
	}
	for (size_t i = n; i-- > 0;) {
		ok &= (p[i] == 0U);
		p[i] = 0xFFFFFFFFU;
	}
	for (size_t i = n; i-- > 0;) {
		ok &= (p[i] == 0xFFFFFFFFU);
		p[i] = 0U;
	}
	for (size_t i = n; i-- > 0;) {
		ok &= (p[i] == 0U);
	}
	for (size_t i = 0; i < n; i++) {
		p[i] = backup[i];
	}
	MPU->CTRL = mpu;
	__DSB();
	__ISB();
	__set_PRIMASK(primask);
	return ok && !(inject & SAFETY_SELFTEST_INJECT_RAM);
}

static bool overlaps(uintptr_t a, uintptr_t b, uintptr_t lo, uintptr_t hi)
{
	return a < hi && b > lo;
}

/* DMA keeps writing during a transparent block test (interrupts off do not
 * stop it): a block near an enabled DMA channel's memory address would read as
 * a failure. Data acquisition's buffers come from the kernel heap, in .bss. */
#define DMA_GUARD_BYTES 4096U

static bool near_dma(uintptr_t a, uintptr_t b)
{
	static DMA_Channel_TypeDef *const ch[] = {
		DMA1_Channel1, DMA1_Channel2, DMA1_Channel3, DMA1_Channel4, DMA1_Channel5, DMA1_Channel6,
		DMA1_Channel7, DMA1_Channel8, DMA2_Channel1, DMA2_Channel2, DMA2_Channel3, DMA2_Channel4,
		DMA2_Channel5, DMA2_Channel6, DMA2_Channel7, DMA2_Channel8,
	};
	for (size_t i = 0; i < ARRAY_SIZE(ch); i++) {
		if (ch[i]->CCR & DMA_CCR_EN) {
			uintptr_t m = ch[i]->CMAR;
			if (overlaps(a, b, m, m + DMA_GUARD_BYTES)) {
				return true;
			}
		}
	}
	return false;
}

static bool ram_test(uintptr_t start, uintptr_t end, uint32_t *bytes)
{
	uintptr_t sp = __get_MSP();
	uintptr_t psp = __get_PSP();
	uintptr_t bk = (uintptr_t)backup;

	*bytes = 0U;
	for (uintptr_t a = start; a + 4U * BLOCK_WORDS <= end; a += 4U * BLOCK_WORDS) {
		uintptr_t b = a + 4U * BLOCK_WORDS;
		if (overlaps(a, b, sp - STACK_GUARD_BELOW, sp + STACK_GUARD_ABOVE) ||
		    overlaps(a, b, psp - STACK_GUARD_BELOW, psp + STACK_GUARD_ABOVE) ||
		    overlaps(a, b, bk, bk + sizeof(backup)) || near_dma(a, b)) {
			continue;
		}
		if (!march_block((volatile uint32_t *)a, BLOCK_WORDS)) {
			return false;
		}
		*bytes += 4U * BLOCK_WORDS;
	}
	return true;
}

static uint32_t crc_feed(uint32_t state, const uint32_t *p, size_t words)
{
	RCC->AHB1ENR |= RCC_AHB1ENR_CRCEN;
	(void)RCC->AHB1ENR;
	CRC->INIT = state;
	CRC->CR = CRC_CR_RESET;
	for (size_t i = 0; i < words; i++) {
		CRC->DR = p[i];
	}
	return CRC->DR;
}

/* System clock in LSI periods: TIM5 counts its clock over 8 LSI periods. */
static uint32_t lsi_counts(void)
{
	RCC->CSR |= RCC_CSR_LSION;
	for (uint32_t i = 0; i < 100000U && !(RCC->CSR & RCC_CSR_LSIRDY); i++) {
	}
	RCC->APB1ENR1 |= RCC_APB1ENR1_TIM5EN;
	(void)RCC->APB1ENR1;
	TIM5->CR1 = 0U;
	TIM5->PSC = 0U;
	TIM5->ARR = 0xFFFFFFFFU;
	TIM5->TISEL = 1U;                                   /* TI1 <- LSI */
	TIM5->CCMR1 = (1U << TIM_CCMR1_CC1S_Pos) | (3U << TIM_CCMR1_IC1PSC_Pos);   /* IC1 on TI1, every 8 edges */
	TIM5->CCER = TIM_CCER_CC1E;
	TIM5->EGR = TIM_EGR_UG;
	TIM5->SR = 0U;
	TIM5->CR1 = TIM_CR1_CEN;
	uint32_t c[2] = {0U, 0U};
	for (int k = 0; k < 2; k++) {
		uint32_t t0 = DWT->CYCCNT;
		while (!(TIM5->SR & TIM_SR_CC1IF)) {
			if (DWT->CYCCNT - t0 > SystemCoreClock / 100U) {   /* 10 ms: no LSI edge */
				TIM5->CR1 = 0U;
				return 0U;
			}
		}
		c[k] = TIM5->CCR1;   /* reading CCR1 clears CC1IF */
	}
	TIM5->CR1 = 0U;
	return c[1] - c[0];
}

static uint32_t tim5_clock_hz(void)
{
	uint32_t ppre1 = (RCC->CFGR & RCC_CFGR_PPRE1) >> RCC_CFGR_PPRE1_Pos;
	/* APB1 prescaler 1: timers at HCLK; otherwise at twice PCLK1 = HCLK * 2 / div. */
	return (ppre1 < 4U) ? SystemCoreClock : (SystemCoreClock * 2U) >> (ppre1 - 3U);
}

static bool clock_check(uint32_t *counts)
{
	uint64_t f = tim5_clock_hz();
	uint32_t lo = (uint32_t)(8ULL * f * 100U / (LSI_MAX_HZ * 105ULL));   /* 5 % beyond the LSI range */
	uint32_t hi = (uint32_t)(8ULL * f * 100U / (LSI_MIN_HZ * 95ULL));
	*counts = lsi_counts();
	if (inject & SAFETY_SELFTEST_INJECT_CLOCK) {
		return false;
	}
	return *counts >= lo && *counts <= hi;
}

/* VREFINT on ADC1 before data acquisition configures the ADCs (the ADC is left
 * disabled, as found). VDDA = 3.0 V x VREFINT_CAL / reading. */
static bool vrefint_check(uint32_t *reading)
{
	RCC->AHB2ENR |= RCC_AHB2ENR_ADC12EN;
	(void)RCC->AHB2ENR;
	ADC12_COMMON->CCR |= ADC_CCR_CKMODE | ADC_CCR_VREFEN;   /* HCLK/4, as the ADC driver sets */
	ADC1->CR &= ~ADC_CR_DEEPPWD;
	ADC1->CR |= ADC_CR_ADVREGEN;
	k_busy_wait(25);
	ADC1->CR |= ADC_CR_ADCAL;
	for (uint32_t i = 0; i < 100000U && (ADC1->CR & ADC_CR_ADCAL); i++) {
	}
	ADC1->ISR = ADC_ISR_ADRDY;
	ADC1->CR |= ADC_CR_ADEN;
	for (uint32_t i = 0; i < 100000U && !(ADC1->ISR & ADC_ISR_ADRDY); i++) {
	}
	ADC1->SMPR2 |= 7U << ADC_SMPR2_SMP18_Pos;   /* 640.5 cycles: VREFINT needs >= 4 us */
	ADC1->SQR1 = 18U << ADC_SQR1_SQ1_Pos;
	ADC1->CFGR &= ~(ADC_CFGR_EXTEN | ADC_CFGR_DMAEN | ADC_CFGR_CONT);
	ADC1->ISR = ADC_ISR_EOC;
	ADC1->CR |= ADC_CR_ADSTART;
	for (uint32_t i = 0; i < 100000U && !(ADC1->ISR & ADC_ISR_EOC); i++) {
	}
	*reading = ADC1->DR;
	ADC1->CR |= ADC_CR_ADDIS;
	for (uint32_t i = 0; i < 100000U && (ADC1->CR & ADC_CR_ADEN); i++) {
	}
	ADC1->SMPR2 &= ~ADC_SMPR2_SMP18;
	ADC1->SQR1 = 0U;
	if (*reading == 0U) {
		return false;
	}
	/* RM0440 §21.4.33: VREF+ = 3.0 V x VREFINT_CAL / reading. The Spin feeds
	 * VREF+ from the internal VREFBUF (2.048 V, 2.5 V or 2.9 V) when it is on,
	 * else from VDDA (3.3 V). */
	uint32_t vref_mv = 3000U * VREFINT_CAL / *reading;
	g_safety_selftest.vdda_mv = vref_mv;
	if ((VREFBUF->CSR & VREFBUF_CSR_ENVR) && !(VREFBUF->CSR & VREFBUF_CSR_HIZ)) {
		static const uint32_t vrs_mv[4] = {2048U, 2500U, 2900U, 2900U};
		uint32_t want = vrs_mv[(VREFBUF->CSR & VREFBUF_CSR_VRS) >> VREFBUF_CSR_VRS_Pos];
		return vref_mv * 100U >= want * 97U && vref_mv * 100U <= want * 103U;
	}
	return vref_mv >= 3000U && vref_mv <= 3600U;
}

/* ---- start-up ---------------------------------------------------------------- */

static int safety_selftest_boot(void)
{
	uint32_t c0;

	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
	g_safety_selftest.magic = SAFETY_SELFTEST_MAGIC;

	c0 = DWT->CYCCNT;
	bool cpu = (safety_cpu_reg_test() == 0);
	g_safety_selftest.boot_us[0] = us_since(c0);

	c0 = DWT->CYCCNT;
	uint32_t bytes;
	bool ram = ram_test(SRAM_START, SRAM_END, &bytes);
	g_safety_selftest.boot_us[1] = us_since(c0);
	g_safety_selftest.boot_ram_bytes = bytes;

	c0 = DWT->CYCCNT;
	size_t words = (size_t)(__rom_region_end - __rom_region_start) / 4U;
	g_safety_selftest.flash_reference = crc_feed(0xFFFFFFFFU, (const uint32_t *)__rom_region_start, words);
	g_safety_selftest.flash_bytes = words * 4U;
	g_safety_selftest.boot_us[2] = us_since(c0);

	c0 = DWT->CYCCNT;
	uint32_t counts;
	bool clk = clock_check(&counts);
	g_safety_selftest.clock_counts = counts;
	g_safety_selftest.boot_us[3] = us_since(c0);

	uint32_t vref = 0U;
	bool vr = vrefint_check(&vref);
	g_safety_selftest.vrefint = vref;

	g_safety_selftest.boot_passed = (cpu ? 1U : 0U) | (ram ? 2U : 0U) | 4U | (clk ? 8U : 0U) | (vr ? 16U : 0U);
	if (!cpu) {
		fail(SAFETY_SELFTEST_CPU, "CPU registers");
	} else if (!ram) {
		fail(SAFETY_SELFTEST_RAM, "RAM (March C-)");
	} else if (!clk) {
		fail(SAFETY_SELFTEST_CLOCK, "system clock against the LSI");
	} else if (!vr) {
		fail(SAFETY_SELFTEST_VREF, "VREFINT (ADC or supply)");
	}
	return 0;
}

/* After CPU supervision's boot hook (APPLICATION 0), which reads the latch,
 * but before main: printk works, no thread has started yet. */
SYS_INIT(safety_selftest_boot, APPLICATION, 1);

/* ---- run time ------------------------------------------------------------------ */

#if defined(CONFIG_OWNTECH_SAFETY_RUNTIME_SELFTEST)
static void runtime_fn(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(runtime_work, runtime_fn);
static uintptr_t ram_cursor;
static size_t flash_cursor;
static uint32_t flash_state = 0xFFFFFFFFU;
static uint32_t ticks;

static void runtime_fn(struct k_work *work)
{
	ARG_UNUSED(work);
	uint32_t c0 = DWT->CYCCNT;

	if (safety_cpu_reg_test() != 0 || (inject & SAFETY_SELFTEST_INJECT_CPU)) {
		fail(SAFETY_SELFTEST_CPU, "CPU registers (run time)");
		return;
	}
	g_safety_selftest.runtime_cpu++;

	/* One block of the image's .data/.bss. */
	uintptr_t start = (uintptr_t)_image_ram_start & ~3U, end = (uintptr_t)_image_ram_end & ~3U;
	if (ram_cursor < start || ram_cursor + 4U * BLOCK_WORDS > end) {
		if (ram_cursor != 0U) {
			g_safety_selftest.runtime_ram_passes++;
		}
		ram_cursor = start;
	}
	uint32_t bytes;
	if (!ram_test(ram_cursor, ram_cursor + 4U * BLOCK_WORDS, &bytes)) {
		fail(SAFETY_SELFTEST_RAM, "RAM (run time)");
		return;
	}
	ram_cursor += 4U * BLOCK_WORDS;

	/* 1 KiB of flash CRC; a complete pass is compared with the reference. */
	size_t total = g_safety_selftest.flash_bytes;
	size_t n = MIN((size_t)FLASH_SLICE_BYTES, total - flash_cursor);
	flash_state = crc_feed(flash_state, (const uint32_t *)(__rom_region_start + flash_cursor), n / 4U);
	flash_cursor += n;
	if (flash_cursor >= total) {
		uint32_t ref = g_safety_selftest.flash_reference ^ ((inject & SAFETY_SELFTEST_INJECT_FLASH) ? 1U : 0U);
		if (flash_state != ref) {
			fail(SAFETY_SELFTEST_FLASH, "flash CRC (run time)");
			return;
		}
		g_safety_selftest.runtime_flash_passes++;
		flash_cursor = 0U;
		flash_state = 0xFFFFFFFFU;
	}

	if (++ticks % 100U == 0U) {
		uint32_t counts;
		if (!clock_check(&counts)) {
			g_safety_selftest.clock_counts = counts;
			fail(SAFETY_SELFTEST_CLOCK, "system clock against the LSI (run time)");
			return;
		}
		g_safety_selftest.runtime_clock++;
	}
	uint32_t us = us_since(c0);
	if (us > g_safety_selftest.runtime_max_us) {
		g_safety_selftest.runtime_max_us = us;
	}
	k_work_schedule(&runtime_work, K_MSEC(10));
}

static int safety_selftest_runtime_start(void)
{
	k_work_schedule(&runtime_work, K_MSEC(100));
	return 0;
}

SYS_INIT(safety_selftest_runtime_start, APPLICATION, 2);
#endif

/* ---- test hooks -------------------------------------------------------------- */

#if defined(CONFIG_OWNTECH_SAFETY_TEST_HOOKS)
void safety_selftest_inject(uint32_t what)
{
	inject = what;
}
#endif

/* Without a shield there is no sensor to check (safety_data.cpp overrides it). */
__attribute__((weak)) void safety_plausibility_period(void)
{
}

uint32_t safety_selftest_injected(void)
{
	return inject;
}
