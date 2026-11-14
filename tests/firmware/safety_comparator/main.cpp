/*
 * Copyright (c) 2026-present LAAS-CNRS
 *
 * SPDX-License-Identifier: LGPL-2.1
 */

/**
 * @file main.cpp
 * @brief Safety test firmware for the comparator path (ADR 0009, issue #18).
 *
 * Bare Spin with the Twist 1.4.1 devicetree: I1_LOW on PA1 (COMP1 -> FLT4) and
 * I2_LOW on PC1 (COMP3 -> FLT5), both high-bound comparator trips. With no
 * shield the sensor pins float near 0 V, below both limits. A trip is provoked
 * by stepping a sensor pin to 3.3 V from the firmware (GPIO output, or charged
 * then back to analog): test code only, never done by the Safety API.
 * Driven over SWD by tests/l0/safety_comparator/run.py (same protocol as
 * tests/firmware/safety_supervision). Never flash it on a powered board.
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/irq.h>
#include <soc.h>

#include "SpinAPI.h"
#include "TaskAPI.h"
#include "ShieldAPI.h"
#include "SafetyAPI.h"
#include "safety_supervision.h"
#include "safety_trips.h"

#define REPORT_MAGIC  0x31504D43U  /* "CMP1" */
#define N_RESULTS     42U

enum : uint32_t {
	CASE_CLEAR = 0, CASE_CONFIG = 1, CASE_LOCKS = 2, CASE_TRIP_I1 = 3, CASE_TRIP_I2 = 4,
	CASE_DAC_TAMPER = 5, CASE_TIGHTEN = 6, CASE_LATCH_RESET = 7, CASE_AWD_CONFIG = 8, CASE_AWD_TRIP = 9,
};

enum {
	R_ARMED = 0,        /* b0 armed, b8-13 armed fault inputs, b16-23 comparator trips allocated */
	R_CSR0 = 1,         /* COMP1 CSR (I1_LOW) */
	R_CSR1 = 2,         /* COMP3 CSR (I2_LOW) */
	R_DAC0 = 3,         /* DAC3 DOR1 (I1_LOW threshold) */
	R_DAC1 = 4,         /* DAC1 DOR1 (I2_LOW threshold) */
	R_FLTINR1 = 5,
	R_FLTINR2 = 6,
	R_FLTR_AND = 7,     /* AND of the six timers' FLTxR */
	R_IER = 8,
	R_LOCKS = 9,        /* b0 COMP1 CSR kept, b1 COMP3 CSR kept, b2 FLTINR1 kept, b3 FLTINR2 kept, b4 FLTxR kept */
	R_TRIP0 = 10,       /* per trip case: b0-1 step mode (1 output, 2 charged analog), b8 flag seen, */
	R_TRIP0_CYC = 11,   /*   b9 latched TRIP, b10 PWM refused while latched, b11 clear refused while active, */
	R_TRIP0_PINS = 12,  /*   b12 clear accepted once quiet, b13 PWM back; b16-21 tripped fault inputs */
	R_TRIP1 = 13,
	R_TRIP1_CYC = 14,
	R_TRIP1_PINS = 15,
	R_TAMPER = 16,      /* b0 latched TRIP, b1 detail 0x100 (threshold read-back), b2 outputs off, b3 cleared */
	R_TAMPER_PINS = 17,
	R_TIGHTEN = 18,     /* b0-7 rc tighten (as uint8), b8-15 rc loosen, b16-27 DAC code after */
	R_LATCH = 19,       /* b0 latched TRIP after reset, b1 PWM refused, b2 cleared, b3 PWM back */
	R_CORE_HZ = 20,
	R_MASK = 21,        /* PWM output mask of the units used */
	R_DETAIL = 22,      /* last latched detail */
	R_SPARE = 23,
	/* ADC-watchdog path (#19) */
	R_ADC1_CFGR = 24, R_ADC2_CFGR = 25, R_ADC1_TR1 = 26, R_ADC1_TR2 = 27, R_ADC1_TR3 = 28,
	R_ADC2_TR3 = 29, R_ADC1_AWD2CR = 30, R_ADC1_AWD3CR = 31, R_ADC2_AWD3CR = 32,
	R_EECR1 = 33, R_EECR2 = 34, R_AWD_DATA = 35,   /* b0 data started, b8-15 ADC1 seq length, b16-23 ADC2 seq length, b24 watchdog states all passed */
	R_AWD_TRIP = 36,   /* b0-1 step mode, b8 flag, b9 latched, b10 refused, b11 clear refused, b12 cleared, b13 back */
	R_AWD_CYC = 37, R_AWD_PINS = 38, R_AWD_FLT_MASK = 39,
	R_PATH0 = 40, R_PATH1 = 41,   /* comparator output -> HRTIM fault flag, cycles (I1_LOW, I2_LOW) */
};

struct report_t {
	uint32_t magic, layout, boot_count, ack, busy, latched_at_boot, pending;
	uint32_t results[N_RESULTS];
};

extern "C" {
__attribute__((used)) volatile report_t g_safety_cmp_report;
__attribute__((used)) volatile uint32_t g_safety_cmp_cmd;
}

/* CPU supervision owns BKP0-9; this test BKP10-31. */
#define BKP_BOOTS   (TAMP->BKP10R)
#define BKP_PENDING (TAMP->BKP29R)
#define BKP_ACK     (TAMP->BKP31R)
static volatile uint32_t *const bkp = &TAMP->BKP11R;   /* results: BKP11..BKP28 hold 18 words */
static uint32_t ram_results[N_RESULTS];

static const hrtim_tu_number_t units[] = {PWMA, PWMC, PWMD, PWME, PWMF};
static const uint32_t OUTPUT_MASK = 0xFF3U;

static void result_set(uint32_t i, uint32_t v)
{
	ram_results[i] = v;
	g_safety_cmp_report.results[i] = v;
	if (i < 18U) {
		bkp[i] = v;
	}
}

static uint32_t pwm_pin_levels(void)
{
	uint32_t a = GPIOA->IDR, b = GPIOB->IDR, c = GPIOC->IDR;
	return ((a >> 8) & 0xFU) | (((b >> 12) & 0xFU) << 4) | (((c >> 8) & 0x3U) << 8) | (((c >> 6) & 0x3U) << 10);
}

static bool outputs_enabled(void)
{
	return (HRTIM1->sCommonRegs.OENR & 0xFFFU) != 0U;
}

static void pwm_start_all(void)
{
	static bool init;
	if (!init) {
		spin.pwm.initFixedFrequency(200000);
		for (auto u : units) {
			spin.pwm.initUnit(u);
			spin.pwm.setDutyCycle(u, 0.5);
		}
		init = true;
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

static void ack(uint32_t seq)
{
	BKP_ACK = seq;
	g_safety_cmp_report.busy = 0U;
	g_safety_cmp_report.ack = seq;
}

/* Sensor pin helpers (test only). */
static void pin_analog(GPIO_TypeDef *g, uint32_t pin)
{
	g->MODER |= 3U << (2U * pin);
}

static void pin_drive(GPIO_TypeDef *g, uint32_t pin, bool high)
{
	g->BSRR = high ? (1U << pin) : (1U << (pin + 16U));
	g->MODER = (g->MODER & ~(3U << (2U * pin))) | (1U << (2U * pin));
}

static void pin_discharge(GPIO_TypeDef *g, uint32_t pin)
{
	pin_drive(g, pin, false);
	k_busy_wait(20);
	pin_analog(g, pin);
	k_busy_wait(20);
}

static bool flag(uint32_t flt)
{
	uint32_t bit = (flt == 6U) ? HRTIM_ISR_FLT6 : (HRTIM_ISR_FLT1 << (flt - 1U));   /* FLT6 is bit 6 */
	return (HRTIM1->sCommonRegs.ISR & bit) != 0U;
}

static volatile uint32_t *comp_for_flt(uint32_t flt)
{
	return (flt == 4U) ? &COMP1->CSR : (flt == 5U) ? &COMP3->CSR : NULL;
}

/* Cycles from the comparator output rising to the HRTIM fault flag: the
 * protection path itself (comparator -> 282 ns filter -> fault input), without
 * the GPIO write, the pad and the loop's own latency to see the step. */
static uint32_t path_cycles;

/* Steps the pin to 3.3 V; @return mode (1: seen as output, 2: seen once back in analog, 0: no trip). */
static uint32_t step_up(GPIO_TypeDef *g, uint32_t pin, uint32_t flt, uint32_t *cycles)
{
	volatile uint32_t *csr = comp_for_flt(flt);
	uint32_t t0 = DWT->CYCCNT, tc = 0U;
	path_cycles = 0U;
	pin_drive(g, pin, true);
	for (uint32_t i = 0; i < 20000U; i++) {
		if (tc == 0U && csr != NULL && (*csr & (1U << 30))) {
			tc = DWT->CYCCNT;
		}
		if (flag(flt)) {
			uint32_t t = DWT->CYCCNT;
			*cycles = t - t0;
			path_cycles = tc ? t - tc : 0U;
			return 1U;
		}
	}
	t0 = DWT->CYCCNT;
	pin_analog(g, pin);   /* the pad keeps its charge */
	for (uint32_t i = 0; i < 2000U; i++) {
		if (flag(flt)) {
			*cycles = DWT->CYCCNT - t0;
			return 2U;
		}
	}
	return 0U;
}

static uint32_t trip_case(GPIO_TypeDef *g, uint32_t pin, uint32_t flt, uint32_t r_flags, uint32_t r_cyc,
			  uint32_t r_pins)
{
	pin_discharge(g, pin);
	pwm_start_all();
	k_msleep(5);
	uint32_t cycles = 0U;
	/* Time the hardware alone: the Safety API's fault interrupt would clear the
	 * flag before the loop sees it. Unmasked right after, so the trip latches. */
	irq_disable(HRTIM1_FLT_IRQn);
	uint32_t mode = step_up(g, pin, flt, &cycles);
	irq_enable(HRTIM1_FLT_IRQn);
	uint32_t flags = mode;
	k_busy_wait(5);
	uint32_t pins = pwm_pin_levels();
	if (flag(flt) || (safety_trips_tripped() & (1U << (flt - 1U)))) {
		flags |= 1U << 8;
	}
	k_msleep(2);
	if (safety_supervision_get_fault() == SAFETY_FAULT_TRIP) {
		flags |= 1U << 9;
	}
	flags |= (safety_trips_tripped() & 0x3FU) << 16;
	pwm_start_all();
	if (!outputs_enabled()) {
		flags |= 1U << 10;
	}
	/* Still beyond the limit: clear must be refused. */
	if (safety.clearFault() != 0) {
		flags |= 1U << 11;
	}
	pin_discharge(g, pin);
	if (safety.clearFault() == 0) {
		flags |= 1U << 12;
	}
	pwm_start_all();
	if (outputs_enabled()) {
		flags |= 1U << 13;
	}
	pwm_stop_all();
	result_set(r_flags, flags);
	result_set(r_cyc, cycles);
	result_set(r_pins, pins);
	result_set(r_flags == R_TRIP0 ? R_PATH0 : R_PATH1, path_cycles);
	return flags;
}

static void case_config(void)
{
	pin_discharge(GPIOA, 1);
	pin_discharge(GPIOC, 1);
	pwm_start_all();   /* arms CPU supervision and the comparator trips */
	pwm_stop_all();
	uint32_t a = (safety_trips_armed() ? 1U : 0U) | (safety_trips_flt_mask() << 8)
		   | ((uint32_t)safety_trips_count() << 16);
	result_set(R_ARMED, a);
	result_set(R_CSR0, COMP1->CSR);
	result_set(R_CSR1, COMP3->CSR);
	result_set(R_DAC0, DAC3->DOR1);
	result_set(R_DAC1, DAC1->DOR1);
	result_set(R_FLTINR1, HRTIM1->sCommonRegs.FLTINR1);
	result_set(R_FLTINR2, HRTIM1->sCommonRegs.FLTINR2);
	uint32_t and_fltr = 0xFFFFFFFFU;
	for (int t = 0; t < 6; t++) {
		and_fltr &= HRTIM1->sTimerxRegs[t].FLTxR;
	}
	result_set(R_FLTR_AND, and_fltr);
	result_set(R_IER, HRTIM1->sCommonRegs.IER);
	result_set(R_CORE_HZ, SystemCoreClock);
	result_set(R_MASK, OUTPUT_MASK);
}

static void case_locks(void)
{
	uint32_t c1 = COMP1->CSR, c3 = COMP3->CSR, i1 = HRTIM1->sCommonRegs.FLTINR1, i2 = HRTIM1->sCommonRegs.FLTINR2;
	uint32_t f[6];
	for (int t = 0; t < 6; t++) {
		f[t] = HRTIM1->sTimerxRegs[t].FLTxR;
	}
	COMP1->CSR = 0U;
	COMP3->CSR = 0U;
	HRTIM1->sCommonRegs.FLTINR1 = 0U;
	HRTIM1->sCommonRegs.FLTINR2 = 0U;
	for (int t = 0; t < 6; t++) {
		HRTIM1->sTimerxRegs[t].FLTxR = 0U;
	}
	uint32_t v = (COMP1->CSR == c1 ? 1U : 0U) | (COMP3->CSR == c3 ? 2U : 0U)
		   | (HRTIM1->sCommonRegs.FLTINR1 == i1 ? 4U : 0U) | (HRTIM1->sCommonRegs.FLTINR2 == i2 ? 8U : 0U);
	bool fl = true;
	for (int t = 0; t < 6; t++) {
		fl = fl && (HRTIM1->sTimerxRegs[t].FLTxR == f[t]);
	}
	result_set(R_LOCKS, v | (fl ? 16U : 0U));
}

static void control_task(void)
{
}

static void case_dac_tamper(void)
{
	static bool task_created;
	pwm_start_all();
	if (!task_created) {
		task.createCritical(control_task, 100, source_tim6);
		task_created = true;
	}
	task.startCritical(false);
	k_msleep(5);
	uint32_t good = (uint32_t)safety_trips_dac_code(0);
	DAC3->DHR12R1 = 4000U;   /* looser threshold written behind the Safety API's back */
	k_msleep(5);
	uint32_t v = 0U;
	safety_fault_record_t rec;
	safety_supervision_get_record(&rec);
	if (rec.fault == SAFETY_FAULT_TRIP) {
		v |= 1U;
	}
	if ((rec.reason & 0x100U) != 0U) {
		v |= 2U;
	}
	if (!outputs_enabled()) {
		v |= 4U;
	}
	result_set(R_TAMPER_PINS, pwm_pin_levels());
	result_set(R_DETAIL, rec.reason);
	task.stopCritical();
	DAC3->DHR12R1 = good;
	k_busy_wait(20);
	if (safety.clearFault() == 0) {
		v |= 8U;
	}
	pwm_stop_all();
	result_set(R_TAMPER, v);
}

static void case_tighten(void)
{
	int rc1 = safety.setTripThreshold(I1_LOW, -7.0f, 5.0f);
	int rc2 = safety.setTripThreshold(I1_LOW, -7.0f, 6.0f);
	int code = safety_trips_dac_code(0);
	result_set(R_TIGHTEN, ((uint32_t)(uint8_t)rc1) | ((uint32_t)(uint8_t)rc2 << 8) | ((uint32_t)code << 16));
}

static void case_awd_config(void)
{
	pin_discharge(GPIOC, 3);
	pwm_start_all();   /* arms: starts data acquisition, the ADC hook sets the watchdogs */
	pwm_stop_all();
	result_set(R_ADC1_CFGR, ADC1->CFGR);
	result_set(R_ADC2_CFGR, ADC2->CFGR);
	result_set(R_ADC1_TR1, ADC1->TR1);
	result_set(R_ADC1_TR2, ADC1->TR2);
	result_set(R_ADC1_TR3, ADC1->TR3);
	result_set(R_ADC2_TR3, ADC2->TR3);
	result_set(R_ADC1_AWD2CR, ADC1->AWD2CR);
	result_set(R_ADC1_AWD3CR, ADC1->AWD3CR);
	result_set(R_ADC2_AWD3CR, ADC2->AWD3CR);
	result_set(R_EECR1, HRTIM1->sCommonRegs.EECR1);
	result_set(R_EECR2, HRTIM1->sCommonRegs.EECR2);
	result_set(R_FLTINR1, HRTIM1->sCommonRegs.FLTINR1);
	result_set(R_FLTINR2, HRTIM1->sCommonRegs.FLTINR2);
	bool states = true;
	for (int i = 0; i < safety_trips_watchdog_count(); i++) {
		states = states && (safety_trips_watchdog_state(i) == 1);
	}
	result_set(R_AWD_DATA, (spin.data.started() ? 1U : 0U) | (((ADC1->SQR1 & 0xFU) + 1U) << 8)
		   | (((ADC2->SQR1 & 0xFU) + 1U) << 16) | (states ? (1U << 24) : 0U));
	result_set(R_AWD_FLT_MASK, safety_trips_flt_mask());
}

static void adc_stop_wait(ADC_TypeDef *a)
{
	a->CR |= ADC_CR_ADSTP;
	while (a->CR & ADC_CR_ADSTART) {
	}
}

/* V_HIGH (ADC1 channel 9, AWD2 -> EEV2 -> FLT2). A pin cannot be held at a
 * level here (the ADC sampling capacitor drains a charged floating pin), so the
 * stimulus is the window: ADC1 is stopped, AWD2 gets a window every reading
 * falls out of, and ADC1 restarts on its HRTIM trigger with the DMA running.
 * Test code only: the Safety API never changes a window while converting. */
static uint32_t awd_step(uint32_t *cycles)
{
	adc_stop_wait(ADC1);
	ADC1->TR2 = (255U << 16) | 255U;   /* readings below 255 (8 bits) are out */
	ADC1->ISR = ADC_ISR_AWD2;
	uint32_t t0 = DWT->CYCCNT;
	ADC1->CR |= ADC_CR_ADSTART;
	for (uint32_t i = 0; i < 200000U; i++) {
		if (flag(2)) {
			*cycles = DWT->CYCCNT - t0;
			return 1U;
		}
	}
	return 0U;
}

static void awd_restore(uint32_t tr2)
{
	adc_stop_wait(ADC1);
	ADC1->TR2 = tr2;
	ADC1->ISR = ADC_ISR_AWD2;
	ADC1->CR |= ADC_CR_ADSTART;
}

static void case_awd_trip(void)
{
	pwm_start_all();
	k_msleep(5);
	uint32_t tr2 = ADC1->TR2;
	uint32_t cycles = 0U;
	irq_disable(HRTIM1_FLT_IRQn);
	uint32_t mode = awd_step(&cycles);
	irq_enable(HRTIM1_FLT_IRQn);
	uint32_t flags = mode;
	k_busy_wait(5);
	result_set(R_AWD_PINS, pwm_pin_levels());
	if (flag(2) || (safety_trips_tripped() & 0x2U)) {
		flags |= 1U << 8;
	}
	k_msleep(2);
	if (safety_supervision_get_fault() == SAFETY_FAULT_TRIP) {
		flags |= 1U << 9;
	}
	pwm_start_all();
	if (!outputs_enabled()) {
		flags |= 1U << 10;
	}
	if (safety.clearFault() != 0) {   /* the window still excludes the reading */
		flags |= 1U << 11;
	}
	awd_restore(tr2);
	if (safety.clearFault() == 0) {
		flags |= 1U << 12;
	}
	pwm_start_all();
	if (outputs_enabled()) {
		flags |= 1U << 13;
	}
	pwm_stop_all();
	result_set(R_AWD_TRIP, flags);
	result_set(R_AWD_CYC, cycles);
}

static void evaluate_pending(uint32_t pending)
{
	uint32_t seq = pending >> 8;
	BKP_PENDING = 0U;
	uint32_t v = 0U;
	if (safety_supervision_get_fault() == SAFETY_FAULT_TRIP) {
		v |= 1U;
	}
	pin_discharge(GPIOA, 1);
	pin_discharge(GPIOC, 1);
	pwm_start_all();
	if (!outputs_enabled()) {
		v |= 2U;
	}
	if (safety.clearFault() == 0) {
		v |= 4U;
	}
	pwm_start_all();
	if (outputs_enabled()) {
		v |= 8U;
	}
	pwm_stop_all();
	result_set(R_LATCH, v);
	ack(seq);
}

static void case_latch_reset(uint32_t seq)
{
	pin_discharge(GPIOA, 1);
	pwm_start_all();
	k_msleep(5);
	uint32_t cycles;
	(void)step_up(GPIOA, 1, 4, &cycles);
	k_msleep(5);
	pin_discharge(GPIOA, 1);
	BKP_PENDING = (seq << 8) | CASE_LATCH_RESET;
	sys_reboot(SYS_REBOOT_COLD);
}

int main(void)
{
	g_safety_cmp_report.layout = 1U;
	BKP_BOOTS = BKP_BOOTS + 1U;
	g_safety_cmp_report.boot_count = BKP_BOOTS;
	g_safety_cmp_report.latched_at_boot = (uint32_t)safety_supervision_get_fault();
	for (uint32_t i = 0; i < 18U; i++) {
		g_safety_cmp_report.results[i] = bkp[i];
	}
	uint32_t pending = BKP_PENDING;
	g_safety_cmp_report.pending = pending;
	g_safety_cmp_report.ack = BKP_ACK;
	g_safety_cmp_report.magic = REPORT_MAGIC;
	uint32_t last = BKP_ACK;
	if (pending != 0U) {
		last = pending >> 8;
		g_safety_cmp_report.busy = last;
		evaluate_pending(pending);
	}
	g_safety_cmp_cmd = (last << 8) | 0xFFU;

	for (;;) {
		uint32_t cmd = g_safety_cmp_cmd, seq = cmd >> 8, c = cmd & 0xFFU;
		if (seq != last && seq != 0U) {
			last = seq;
			g_safety_cmp_report.busy = seq;
			switch (c) {
			case CASE_CLEAR:
				for (uint32_t i = 0; i < N_RESULTS; i++) {
					result_set(i, 0U);
				}
				BKP_PENDING = 0U;
				pin_discharge(GPIOA, 1);
				pin_discharge(GPIOC, 1);
				(void)safety.clearFault();
				break;
			case CASE_CONFIG: case_config(); break;
			case CASE_LOCKS: case_locks(); break;
			case CASE_TRIP_I1: trip_case(GPIOA, 1, 4, R_TRIP0, R_TRIP0_CYC, R_TRIP0_PINS); break;
			case CASE_TRIP_I2: trip_case(GPIOC, 1, 5, R_TRIP1, R_TRIP1_CYC, R_TRIP1_PINS); break;
			case CASE_DAC_TAMPER: case_dac_tamper(); break;
			case CASE_TIGHTEN: case_tighten(); break;
			case CASE_LATCH_RESET: case_latch_reset(seq); break;
			case CASE_AWD_CONFIG: case_awd_config(); break;
			case CASE_AWD_TRIP: case_awd_trip(); break;
			default: break;
			}
			ack(seq);
		}
		k_msleep(10);
	}
	return 0;
}
