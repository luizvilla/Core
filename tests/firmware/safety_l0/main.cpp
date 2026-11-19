/*
 * Copyright (c) 2026-present LAAS-CNRS
 *
 * SPDX-License-Identifier: LGPL-2.1
 */

/**
 * @file main.cpp
 * @brief Generic safety test firmware for the SPOC bench (ADR 0009, issue luizvilla/Core#23).
 *
 * One app for any shield (env SAFETY_L0, shield from SAFETY_SHIELD / SAFETY_SHIELD_VERSION), on the CUT Spin
 * of the SPOC bench: a bare Spin, no power stage. Controlled over USB with ThingSet (shell port, as Core-CUT);
 * flashed and inspected over SWD with an ST-Link. Never flash it on a powered board.
 *
 * ThingSet:
 *   /Safety  rArmed rFault rTripped rFltMask rOutputs rText, xClear      state of the Safety API, clearFault()
 *   /Pwm     wFreq_Hz wDuty wUnits wRun, xApply                           legs (HRTIM units, bit 0 = A ... 5 = F)
 *   /Inj     wLevel_mV wOn, xApply; rLevel_mV rVref_mV rOn               self-injection DAC2_CH1 on PA6 (pin 34)
 *   /Cal     wOn, xApply; rOn                                             1 kHz, 0.5 on PA8 from TIM1 (L0-CAL)
 *   /Fault   wCase wIndex, xInject; rRc rDone rWritten rDetected rDetect_us rLastCase rReason rPc rTask
 *            rKernelGap_us (longest gap between kernel heartbeats since boot)
 *            software fault injection (cases below); the case that ended in a reset is kept in TAMP BKP29
 *
 * A 100 us control task (TIM6) starts with the first PWM start, after the trips are armed: it feeds the
 * watchdog and runs the threshold read-back, as in an application. It cannot start earlier: data
 * acquisition, which it starts, runs the ADC watchdog self-test, and the sensor pins must be held first.
 *
 * Fault cases (/Fault/wCase, then xInject; run 20 ms later so the ThingSet reply goes out first):
 *   1 thread stack overflow (MPU guard)     2 undefined instruction
 *   3 every interrupt masked forever (PRIMASK): the control task stops too
 *   4 control task never returns            5 Cortex-M4 lockup (FAULTMASK + fault)
 *   6 wild writes to the locked registers: COMPx CSR of each armed comparator, HRTIM FLTINR1/2, the
 *     timers' FLTxR, SYSCFG CFGR2 lock bits; rWritten bit 0-4 = a write took effect (restored at once)
 *   7 comparator trip wIndex: DAC threshold changed by one count behind the API
 *   8 watchdog trip wIndex: window changed by one count behind the API
 *   9 watchdog trip wIndex: source of the external event carrying it to its fault input changed
 *     (EECR1/EECR2, not lockable)
 *  10 timer A: fault state of TA1/TA2 set to "no action" (OUTxR, not locked)
 *  11 first watchdog trip on AWD1: watched channel changed (ADC CFGR AWD1CH)
 *  12 first watchdog trip: its ADC's conversions stopped (ADSTP), restarted after 10 ms
 *  13 kernel starved: a thread spins with irq_lock() (BASEPRI); zero-latency interrupts, the control task
 *     among them, still run, while threads and the system workqueue (run-time self-tests, heartbeat) stop
 *  14 a work item blocks forever on the system workqueue: the heartbeat and self-tests stop, threads run
 *  15 a work item spins forever on the system workqueue (cooperative): no thread runs, interrupts do
 *  16 data.stop() with the outputs on: rDetect_us holds its return code + 128 (-2 + 128 = refused)
 * Cases 7-12 report whether the change took effect (rWritten bit 0), whether a fault latched within 10 ms
 * (rDetected, rDetect_us) and restore the register. Cases 1-5 end in a reset; /Safety and rReason, rPc
 * then show the latched fault and rLastCase the case.
 *
 * DAC2_CH1 is reserved from the protection allocation (app.overlay, reserved-dacs). The calibration output
 * uses TIM1, not the HRTIM: starting the HRTIM arms the trips, and on a bare Spin the floating sensor pins
 * would trip them. PA8 is a leg output on most shields: /Cal refuses while the PWM runs and the reverse.
 */

#include <stdio.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <soc.h>
#include <stm32g4xx_ll_bus.h>
#include <stm32g4xx_ll_dac.h>
#include <stm32g4xx_ll_gpio.h>
#include <stm32g4xx_ll_system.h>
#include <stm32g4xx_ll_tim.h>

#include <thingset.h>
#include <thingset/sdk.h>

#include "SpinAPI.h"
#include "TaskAPI.h"
#include "SafetyAPI.h"
#include "safety_supervision.h"
#include "safety_trips.h"

/* ------------------------------------------------------------------ state */

static bool safety_armed, cal_on_now, inj_on_now, pwm_running;
static uint8_t fault_now;
static uint32_t tripped_now, flt_mask_now;
static uint16_t outputs_now;
static char fault_text[96] = "";
static bool clear_request;
static int8_t clear_rc;

static uint32_t pwm_freq_hz = 200000;
static float pwm_duty = 0.5f;
static uint8_t pwm_units = 0;       /* requested units, bit 0 = A */
static bool pwm_run_req;
static bool pwm_apply;
static int8_t pwm_rc;
static bool pwm_init_done;
static uint32_t pwm_init_freq;

static uint16_t inj_level_req_mV;
static bool inj_on_req, inj_apply;
static uint16_t inj_level_mV, inj_vref_mV = 2048;

static bool cal_on_req, cal_apply;
static int8_t cal_rc;

static const hrtim_tu_number_t UNITS[6] = {PWMA, PWMB, PWMC, PWMD, PWME, PWMF};

static bool task_running;
static volatile bool task_stuck;
static volatile uint32_t task_calls;

/* ------------------------------------------------------------------ safety status */

static void refresh_status(void)
{
	safety_armed = safety_trips_armed();
	fault_now = (uint8_t)safety_supervision_get_fault();
	tripped_now = safety_trips_tripped();
	flt_mask_now = safety_trips_flt_mask();
	outputs_now = (uint16_t)(HRTIM1->sCommonRegs.OENR & 0xFFFU);
	if (fault_now != SAFETY_FAULT_NONE) {
		(void)safety.getFaultDescription(fault_text, sizeof(fault_text));
	} else {
		fault_text[0] = '\0';
	}
}

/* ------------------------------------------------------------------ control task */

static void control_task(void)
{
	task_calls++;
	if (task_stuck) {
		for (;;) {
		}
	}
}

static void start_control_task(void)
{
	task.createCritical(control_task, 100, source_tim6);
	task.startCritical(false);
	task_running = true;
}

/* ------------------------------------------------------------------ PWM legs */

static int pwm_apply_now(void)
{
	if (!pwm_run_req) {
		for (int u = 0; u < 6; u++) {
			if (pwm_init_done && (pwm_units & (1U << u))) {
				spin.pwm.stopDualOutput(UNITS[u]);
			}
		}
		pwm_running = false;
		return 0;
	}
	if (cal_on_now) {
		return -1; /* PA8 is held by the calibration output */
	}
	if (pwm_units == 0U || (pwm_units & ~0x3FU)) {
		return -2;
	}
	if (!pwm_init_done) {
		spin.pwm.initFixedFrequency(pwm_freq_hz);
		pwm_init_freq = pwm_freq_hz;
		pwm_init_done = true;
	} else if (pwm_freq_hz != pwm_init_freq) {
		return -3; /* the HRTIM period is set once per boot */
	}
	for (int u = 0; u < 6; u++) {
		if (pwm_units & (1U << u)) {
			spin.pwm.initUnit(UNITS[u]);
			spin.pwm.setDutyCycle(UNITS[u], pwm_duty);
			spin.pwm.startDualOutput(UNITS[u]); /* the first start arms the trips */
		}
	}
	pwm_running = (HRTIM1->sCommonRegs.OENR & 0xFFFU) != 0U; /* false if arming refused */
	if (pwm_running && !task_running) {
		start_control_task();
	}
	return pwm_running ? 0 : -4;
}

/* ------------------------------------------------------------------ self-injection DAC2_CH1 on PA6 */

static uint16_t vref_mV(void)
{
	if ((VREFBUF->CSR & VREFBUF_CSR_ENVR) == 0U) {
		return 3300U;
	}
	switch (LL_VREFBUF_GetVoltageScaling()) {
	case LL_VREFBUF_VOLTAGE_SCALE0: return 2048U;
	case LL_VREFBUF_VOLTAGE_SCALE1: return 2500U;
	default:                        return 2900U;
	}
}

static void inj_apply_now(void)
{
	if (!inj_on_req) {
		LL_DAC_Disable(DAC2, LL_DAC_CHANNEL_1);
		LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_6, LL_GPIO_MODE_ANALOG);
		inj_on_now = false;
		inj_level_mV = 0;
		return;
	}
	inj_vref_mV = vref_mV();
	uint32_t code = inj_level_req_mV >= inj_vref_mV ? 4095U
			: ((uint32_t)inj_level_req_mV * 4095U + inj_vref_mV / 2U) / inj_vref_mV;
	LL_DAC_ConvertData12RightAligned(DAC2, LL_DAC_CHANNEL_1, code);
	inj_level_mV = (uint16_t)((code * inj_vref_mV + 2047U) / 4095U);
	if (!inj_on_now) {
		LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_6, LL_GPIO_MODE_ANALOG);
		LL_DAC_Enable(DAC2, LL_DAC_CHANNEL_1);
		k_busy_wait(10); /* tWAKEUP */
		inj_on_now = true;
	}
}

static void inj_init(void)
{
	LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
	LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_DAC2);
	LL_DAC_SetHighFrequencyMode(DAC2, LL_DAC_HIGH_FREQ_MODE_ABOVE_160MHZ);
	/* External pin, output buffer on: drives the target sensor pin through the SPOC mux and the TP1-TP3 wire. */
	LL_DAC_SetOutputMode(DAC2, LL_DAC_CHANNEL_1, LL_DAC_OUTPUT_MODE_NORMAL);
	LL_DAC_SetOutputBuffer(DAC2, LL_DAC_CHANNEL_1, LL_DAC_OUTPUT_BUFFER_ENABLE);
	LL_DAC_SetOutputConnection(DAC2, LL_DAC_CHANNEL_1, LL_DAC_OUTPUT_CONNECT_GPIO);
	LL_DAC_DisableTrigger(DAC2, LL_DAC_CHANNEL_1);
	inj_vref_mV = vref_mV();
}

/* ------------------------------------------------------------------ calibration output: TIM1_CH1 on PA8 */

static int cal_apply_now(void)
{
	if (!cal_on_req) {
		LL_TIM_DisableCounter(TIM1);
		LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH1);
		LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_8, LL_GPIO_MODE_ANALOG);
		cal_on_now = false;
		return 0;
	}
	if (pwm_running) {
		return -1; /* PA8 is a leg output while the PWM runs */
	}
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);
	uint32_t f_tim = SystemCoreClock;              /* APB2 prescaler 1 on the Spin */
	LL_TIM_SetPrescaler(TIM1, f_tim / 1000000U - 1U); /* 1 MHz */
	LL_TIM_SetAutoReload(TIM1, 999U);                /* 1 kHz */
	LL_TIM_OC_SetMode(TIM1, LL_TIM_CHANNEL_CH1, LL_TIM_OCMODE_PWM1);
	LL_TIM_OC_SetCompareCH1(TIM1, 500U);             /* duty 0.5 */
	LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH1);
	LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1);
	LL_TIM_EnableAllOutputs(TIM1);                   /* BDTR MOE */
	LL_TIM_GenerateEvent_UPDATE(TIM1);
	LL_GPIO_SetAFPin_8_15(GPIOA, LL_GPIO_PIN_8, LL_GPIO_AF_6); /* TIM1_CH1 */
	LL_GPIO_SetPinOutputType(GPIOA, LL_GPIO_PIN_8, LL_GPIO_OUTPUT_PUSHPULL);
	LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_8, LL_GPIO_SPEED_FREQ_VERY_HIGH);
	LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_8, LL_GPIO_MODE_ALTERNATE);
	LL_TIM_EnableCounter(TIM1);
	cal_on_now = true;
	return 0;
}

/* ------------------------------------------------------------------ software fault injection (C10) */

enum fault_case : uint8_t {
	F_STACK = 1, F_HARDFAULT, F_IRQ_MASKED, F_TASK_STUCK, F_LOCKUP,
	F_WILD_LOCKED, F_WILD_DAC, F_WILD_AWD, F_WILD_EEV, F_WILD_OUTR, F_WILD_AWDCH, F_ADC_STOP,
	F_KERNEL_STARVED, F_WORK_BLOCKED, F_WORK_SPIN, F_DATA_STOP, F_LAST = F_DATA_STOP
};

#define BKP_CASE        (TAMP->BKP29R)   /* 0xFA0000nn: case nn was injected; read and wiped at boot */
#define BKP_CASE_MAGIC  0xFA000000U

static uint8_t fault_case_req, fault_index_req, last_case;
static bool fault_inject, fault_done, fault_detected;
static int8_t fault_rc;
static uint8_t fault_written;
static uint32_t fault_detect_us, fault_reason, fault_pc, kernel_gap_us;

static void inject_work_fn(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(inject_work, inject_work_fn);

K_THREAD_STACK_DEFINE(victim_stack, 512);
static struct k_thread victim_thread;

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Winfinite-recursion"
static int __attribute__((noinline)) recurse(int depth)
{
	volatile uint8_t buf[64];
	for (unsigned i = 0; i < sizeof(buf); i++) {
		buf[i] = (uint8_t)depth;
	}
	return recurse(depth + 1) + buf[depth & 63];   /* unbounded on purpose: runs into the stack guard */
}
#pragma GCC diagnostic pop

static void victim_stack_overflow(void *, void *, void *) { (void)recurse(0); }
static void victim_undefined(void *, void *, void *) { __builtin_trap(); }

static void run_victim(k_thread_entry_t entry)
{
	k_thread_create(&victim_thread, victim_stack, K_THREAD_STACK_SIZEOF(victim_stack), entry,
			NULL, NULL, NULL, K_PRIO_PREEMPT(1), 0, K_NO_WAIT);
}

static uint32_t cycles_to_us(uint32_t c)
{
	return c / (SystemCoreClock / 1000000U);
}

/* After a write behind the API: wait up to 10 ms for a latched fault. */
static void await_detection(void)
{
	uint32_t c0 = DWT->CYCCNT, limit = 10000U * (SystemCoreClock / 1000000U);
	fault_detected = false;
	while (DWT->CYCCNT - c0 < limit) {
		if (safety_supervision_get_fault() != SAFETY_FAULT_NONE) {
			fault_detected = true;
			break;
		}
	}
	fault_detect_us = cycles_to_us(DWT->CYCCNT - c0);
}

/* Writes @p bad, reads back, restores. @return true if the write took effect. */
static bool wild_write(volatile uint32_t *reg, uint32_t bad)
{
	uint32_t orig = *reg;
	*reg = bad;
	bool took = (*reg != orig);
	*reg = orig;
	return took;
}

static uint8_t wild_locked(void)
{
	uint8_t written = 0U;
	struct safety_trip_regs r;
	static volatile uint32_t *const comp_csr[8] = {NULL, &COMP1->CSR, &COMP2->CSR, &COMP3->CSR,
						       &COMP4->CSR, &COMP5->CSR, &COMP6->CSR, &COMP7->CSR};
	for (int i = 0; safety_trips_test_regs(false, i, &r) == 0; i++) {
		if ((safety_trips_flt_mask() & (1U << (r.flt - 1U))) && wild_write(comp_csr[r.comp], 0U)) {
			written |= 1U << 0;
		}
	}
	if (wild_write(&HRTIM1->sCommonRegs.FLTINR1, 0U)) {
		written |= 1U << 1;
	}
	if (wild_write(&HRTIM1->sCommonRegs.FLTINR2, 0U)) {
		written |= 1U << 2;
	}
	for (int t = 0; t < 6; t++) {
		if (wild_write(&HRTIM1->sTimerxRegs[t].FLTxR, 0U)) {
			written |= 1U << 3;
		}
	}
	uint32_t cfgr2 = SYSCFG->CFGR2;
	SYSCFG->CFGR2 = cfgr2 & ~0xFU;               /* CLL SPL PVDL ECCL: set-only */
	if ((SYSCFG->CFGR2 & 0xFU) != (cfgr2 & 0xFU)) {
		written |= 1U << 4;
		SYSCFG->CFGR2 = cfgr2;
	}
	return written;
}

/* Cases 7-10: change one register behind the API, see whether it is caught, restore it. */
static void wild_unlocked(volatile uint32_t *reg, uint32_t bad)
{
	uint32_t orig = *reg;
	*reg = bad;
	fault_written = (*reg != orig) ? 1U : 0U;
	await_detection();
	*reg = orig;
}

static int first_awd1_watchdog(struct safety_trip_regs *r)
{
	for (int i = 0; safety_trips_test_regs(true, i, r) == 0; i++) {
		if (r->awd == 1U) {
			return i;
		}
	}
	return -1;
}

/* Case 12: conversions stopped; restarted (same set-up) after the detection window. */
static void adc_stop_case(volatile uint32_t *cr)
{
	*cr |= ADC_CR_ADSTP;
	for (int i = 0; i < 1000 && (*cr & ADC_CR_ADSTART); i++) {
		k_busy_wait(1);
	}
	fault_written = (*cr & ADC_CR_ADSTART) ? 0U : 1U;
	await_detection();
	*cr |= ADC_CR_ADSTART;
}

static void inject_work_fn(struct k_work *work)
{
	ARG_UNUSED(work);
	struct safety_trip_regs r;
	uint8_t c = fault_case_req;

	if (c <= F_LOCKUP || (c >= F_KERNEL_STARVED && c <= F_WORK_SPIN)) {
		BKP_CASE = BKP_CASE_MAGIC | c;
	}
	switch (c) {
	case F_STACK:      run_victim(victim_stack_overflow); return;
	case F_HARDFAULT:  run_victim(victim_undefined); return;
	case F_IRQ_MASKED: __disable_irq(); for (;;) { } break;
	case F_KERNEL_STARVED: (void)irq_lock(); for (;;) { } break;
	case F_WORK_BLOCKED: {
		static K_SEM_DEFINE(never, 0, 1);
		fault_done = true;
		(void)k_sem_take(&never, K_FOREVER);
		return;
	}
	case F_WORK_SPIN: for (;;) { } break;
	case F_DATA_STOP:
		fault_detect_us = (uint32_t)(spin.data.stop() + 128);
		fault_written = spin.data.started() ? 0U : 1U;
		break;
	case F_TASK_STUCK: task_stuck = true; return;
	case F_LOCKUP:     __set_FAULTMASK(1U); __builtin_trap(); break;
	case F_WILD_LOCKED:
		fault_written = wild_locked();
		break;
	case F_WILD_DAC:
		(void)safety_trips_test_regs(false, fault_index_req, &r);
		wild_unlocked(r.dac_dhr, *r.dac_dhr ^ 1U);
		break;
	case F_WILD_AWD:
		(void)safety_trips_test_regs(true, fault_index_req, &r);
		wild_unlocked(r.awd_tr, *r.awd_tr ^ 1U);   /* lowest bit of the low threshold */
		break;
	case F_WILD_EEV: {
		(void)safety_trips_test_regs(true, fault_index_req, &r);
		/* EEVn feeds FLTn; EExSRC 11 = the ADC watchdog, 00 = the pin */
		volatile uint32_t *eecr = (r.flt <= 5U) ? &HRTIM1->sCommonRegs.EECR1 : &HRTIM1->sCommonRegs.EECR2;
		uint32_t shift = 6U * ((r.flt <= 5U) ? r.flt - 1U : r.flt - 6U);
		wild_unlocked(eecr, *eecr & ~(3U << shift));
		break;
	}
	case F_WILD_OUTR:
		wild_unlocked(&HRTIM1->sTimerxRegs[0].OUTxR,
			      HRTIM1->sTimerxRegs[0].OUTxR & ~(HRTIM_OUTR_FAULT1 | HRTIM_OUTR_FAULT2));
		break;
	case F_WILD_AWDCH:
		(void)first_awd1_watchdog(&r);   /* AWD1CH: channel field, 5 bits; next channel up */
		wild_unlocked(r.adc_cfgr, *r.adc_cfgr ^ (1U << ADC_CFGR_AWD1CH_Pos));
		break;
	case F_ADC_STOP:
		(void)safety_trips_test_regs(true, 0, &r);
		adc_stop_case(r.adc_cr);
		break;
	default:
		break;
	}
	fault_done = true;
}

/* Checks a request; @return 0 and schedules the case, or a refusal code. */
static int fault_request(void)
{
	struct safety_trip_regs r;
	uint8_t c = fault_case_req;

	if (c < F_STACK || c > F_LAST) {
		return -3;
	}
	if (c >= F_WILD_LOCKED && !safety_trips_armed()) {
		return -1;   /* the registers are set up and locked at arming */
	}
	if ((c == F_TASK_STUCK || c >= F_WILD_DAC) && !task_running) {
		return -2;   /* the read-back and the watchdog feed run in the control task */
	}
	if ((c == F_WILD_DAC && safety_trips_test_regs(false, fault_index_req, &r) != 0) ||
	    (c == F_WILD_AWD && safety_trips_test_regs(true, fault_index_req, &r) != 0) ||
	    (c == F_WILD_EEV && safety_trips_test_regs(true, fault_index_req, &r) != 0) ||
	    (c == F_WILD_AWDCH && first_awd1_watchdog(&r) < 0) ||
	    (c == F_ADC_STOP && safety_trips_test_regs(true, 0, &r) != 0)) {
		return -4;   /* no such trip (or built without CONFIG_OWNTECH_SAFETY_TEST_HOOKS) */
	}
	fault_done = false;
	fault_detected = false;
	fault_written = 0U;
	fault_detect_us = 0U;
	k_work_schedule(&inject_work, K_MSEC(20));
	return 0;
}

static void fault_boot(void)
{
	uint32_t b = BKP_CASE;
	if ((b & 0xFFFFFF00U) == BKP_CASE_MAGIC) {
		last_case = (uint8_t)(b & 0xFFU);
	}
	BKP_CASE = 0U;
}

/* ------------------------------------------------------------------ ThingSet */

static void fault_cb(enum thingset_callback_reason reason)
{
	if (reason == THINGSET_CALLBACK_PRE_READ) {
		safety_fault_record_t rec;
		safety_supervision_get_record(&rec);
		fault_reason = rec.reason;
		fault_pc = rec.pc;
		safety_liveness_t l;
		safety_supervision_liveness(&l);
		kernel_gap_us = l.kernel_gap_max_us;
	} else if (reason == THINGSET_CALLBACK_POST_WRITE && fault_inject) {
		fault_inject = false;
		fault_rc = (int8_t)fault_request();
	}
}

static void safety_cb(enum thingset_callback_reason reason)
{
	if (reason == THINGSET_CALLBACK_PRE_READ) {
		refresh_status();
	} else if (reason == THINGSET_CALLBACK_POST_WRITE && clear_request) {
		clear_request = false;
		clear_rc = (int8_t)safety.clearFault();
		refresh_status();
	}
}

static void pwm_cb(enum thingset_callback_reason reason)
{
	if (reason == THINGSET_CALLBACK_POST_WRITE && pwm_apply) {
		pwm_apply = false;
		pwm_rc = (int8_t)pwm_apply_now();
	}
}

static void inj_cb(enum thingset_callback_reason reason)
{
	if (reason == THINGSET_CALLBACK_POST_WRITE && inj_apply) {
		inj_apply = false;
		inj_apply_now();
	}
}

static void cal_cb(enum thingset_callback_reason reason)
{
	if (reason == THINGSET_CALLBACK_POST_WRITE && cal_apply) {
		cal_apply = false;
		cal_rc = (int8_t)cal_apply_now();
	}
}

#define ID_SAFETY 0x90
#define ID_PWM    0x91
#define ID_INJ    0x92
#define ID_CAL    0x93
#define ID_FAULT  0x94

THINGSET_ADD_GROUP(TS_ID_ROOT, ID_SAFETY, "Safety", &safety_cb);
THINGSET_ADD_ITEM_BOOL(ID_SAFETY, 0x9001, "rArmed", &safety_armed, THINGSET_ANY_R, 0);
THINGSET_ADD_ITEM_UINT8(ID_SAFETY, 0x9002, "rFault", &fault_now, THINGSET_ANY_R, 0);
THINGSET_ADD_ITEM_UINT32(ID_SAFETY, 0x9003, "rTripped", &tripped_now, THINGSET_ANY_R, 0);
THINGSET_ADD_ITEM_UINT32(ID_SAFETY, 0x9004, "rFltMask", &flt_mask_now, THINGSET_ANY_R, 0);
THINGSET_ADD_ITEM_UINT16(ID_SAFETY, 0x9005, "rOutputs", &outputs_now, THINGSET_ANY_R, 0);
THINGSET_ADD_ITEM_STRING(ID_SAFETY, 0x9006, "rText", fault_text, sizeof(fault_text), THINGSET_ANY_R, 0);
THINGSET_ADD_ITEM_BOOL(ID_SAFETY, 0x9007, "xClear", &clear_request, THINGSET_ANY_RW, 0);
THINGSET_ADD_ITEM_INT8(ID_SAFETY, 0x9008, "rClearRc", &clear_rc, THINGSET_ANY_R, 0);

THINGSET_ADD_GROUP(TS_ID_ROOT, ID_PWM, "Pwm", &pwm_cb);
THINGSET_ADD_ITEM_UINT32(ID_PWM, 0x9101, "wFreq_Hz", &pwm_freq_hz, THINGSET_ANY_RW, 0);
THINGSET_ADD_ITEM_FLOAT(ID_PWM, 0x9102, "wDuty", &pwm_duty, 3, THINGSET_ANY_RW, 0);
THINGSET_ADD_ITEM_UINT8(ID_PWM, 0x9103, "wUnits", &pwm_units, THINGSET_ANY_RW, 0);
THINGSET_ADD_ITEM_BOOL(ID_PWM, 0x9104, "wRun", &pwm_run_req, THINGSET_ANY_RW, 0);
THINGSET_ADD_ITEM_BOOL(ID_PWM, 0x9105, "xApply", &pwm_apply, THINGSET_ANY_RW, 0);
THINGSET_ADD_ITEM_INT8(ID_PWM, 0x9106, "rRc", &pwm_rc, THINGSET_ANY_R, 0);
THINGSET_ADD_ITEM_BOOL(ID_PWM, 0x9107, "rRunning", &pwm_running, THINGSET_ANY_R, 0);

THINGSET_ADD_GROUP(TS_ID_ROOT, ID_INJ, "Inj", &inj_cb);
THINGSET_ADD_ITEM_UINT16(ID_INJ, 0x9201, "wLevel_mV", &inj_level_req_mV, THINGSET_ANY_RW, 0);
THINGSET_ADD_ITEM_BOOL(ID_INJ, 0x9202, "wOn", &inj_on_req, THINGSET_ANY_RW, 0);
THINGSET_ADD_ITEM_BOOL(ID_INJ, 0x9203, "xApply", &inj_apply, THINGSET_ANY_RW, 0);
THINGSET_ADD_ITEM_UINT16(ID_INJ, 0x9204, "rLevel_mV", &inj_level_mV, THINGSET_ANY_R, 0);
THINGSET_ADD_ITEM_UINT16(ID_INJ, 0x9205, "rVref_mV", &inj_vref_mV, THINGSET_ANY_R, 0);
THINGSET_ADD_ITEM_BOOL(ID_INJ, 0x9206, "rOn", &inj_on_now, THINGSET_ANY_R, 0);

THINGSET_ADD_GROUP(TS_ID_ROOT, ID_CAL, "Cal", &cal_cb);
THINGSET_ADD_ITEM_BOOL(ID_CAL, 0x9301, "wOn", &cal_on_req, THINGSET_ANY_RW, 0);
THINGSET_ADD_ITEM_BOOL(ID_CAL, 0x9302, "xApply", &cal_apply, THINGSET_ANY_RW, 0);
THINGSET_ADD_ITEM_BOOL(ID_CAL, 0x9303, "rOn", &cal_on_now, THINGSET_ANY_R, 0);
THINGSET_ADD_ITEM_INT8(ID_CAL, 0x9304, "rRc", &cal_rc, THINGSET_ANY_R, 0);

THINGSET_ADD_GROUP(TS_ID_ROOT, ID_FAULT, "Fault", &fault_cb);
THINGSET_ADD_ITEM_UINT8(ID_FAULT, 0x9401, "wCase", &fault_case_req, THINGSET_ANY_RW, 0);
THINGSET_ADD_ITEM_UINT8(ID_FAULT, 0x9402, "wIndex", &fault_index_req, THINGSET_ANY_RW, 0);
THINGSET_ADD_ITEM_BOOL(ID_FAULT, 0x9403, "xInject", &fault_inject, THINGSET_ANY_RW, 0);
THINGSET_ADD_ITEM_INT8(ID_FAULT, 0x9404, "rRc", &fault_rc, THINGSET_ANY_R, 0);
THINGSET_ADD_ITEM_BOOL(ID_FAULT, 0x9405, "rDone", &fault_done, THINGSET_ANY_R, 0);
THINGSET_ADD_ITEM_UINT8(ID_FAULT, 0x9406, "rWritten", &fault_written, THINGSET_ANY_R, 0);
THINGSET_ADD_ITEM_BOOL(ID_FAULT, 0x9407, "rDetected", &fault_detected, THINGSET_ANY_R, 0);
THINGSET_ADD_ITEM_UINT32(ID_FAULT, 0x9408, "rDetect_us", &fault_detect_us, THINGSET_ANY_R, 0);
THINGSET_ADD_ITEM_UINT8(ID_FAULT, 0x9409, "rLastCase", &last_case, THINGSET_ANY_R, 0);
THINGSET_ADD_ITEM_UINT32(ID_FAULT, 0x940A, "rReason", &fault_reason, THINGSET_ANY_R, 0);
THINGSET_ADD_ITEM_UINT32(ID_FAULT, 0x940B, "rPc", &fault_pc, THINGSET_ANY_R, 0);
THINGSET_ADD_ITEM_BOOL(ID_FAULT, 0x940C, "rTask", &task_running, THINGSET_ANY_R, 0);
THINGSET_ADD_ITEM_UINT32(ID_FAULT, 0x940D, "rKernelGap_us", &kernel_gap_us, THINGSET_ANY_R, 0);

/* ------------------------------------------------------------------ main */

static void background(void)
{
	refresh_status();
	task.suspendBackgroundMs(100);
}

int main(void)
{
	fault_boot();
	inj_init();
	refresh_status();
	uint32_t bg = task.createBackground(background);
	task.startBackground(bg);
	return 0;
}
