/*
 * Copyright (c) 2026-present LAAS-CNRS
 *
 * SPDX-License-Identifier: LGPL-2.1
 */

/**
 * @file main.cpp
 * @brief One Safety API test firmware (ADR 0009, issue #22).
 *
 * Bare Spin with the Twist 1.4.1 devicetree. Built twice from this file:
 * env SAFETY_API (production) and env SAFETY_API_DEV (the same devicetree plus
 * safety-mode = "development"). Checks the legacy functions as compatibility
 * wrappers: the reaction mapped to the leg safe states, the tighten-only rule
 * for the software thresholds and the NVS, the software check reacting like
 * every other trip, disableSafetyApi() and disarm() by mode.
 * Driven over SWD by tests/l0/safety_api/run.py (same protocol as
 * tests/firmware/safety_comparator). Never flash it on a powered board.
 */

#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/reboot.h>
#include <soc.h>

#include "SpinAPI.h"
#include "TaskAPI.h"
#include "ShieldAPI.h"
#include "SafetyAPI.h"
#include "nvs_storage.h"
#include "hrtim.h"
#include <stm32_ll_hrtim.h>
#include "safety_supervision.h"
#include "safety_trips.h"
#include "safety_states.h"
#include "safety_mode.h"

#define REPORT_MAGIC  0x31495041U  /* "API1" */
#define N_RESULTS     26U

enum : uint32_t {
	CASE_CLEAR = 0, CASE_REACTION = 1, CASE_DEV = 2, CASE_WRAPPERS = 3, CASE_NVS = 4, CASE_SW_TRIP = 5,
};

enum {
	R_REACT = 0,     /* fault codes (1 active, 2 inactive) after setChannelReaction(Short_Circuit) then PWM init:
	                  * b0-1 TA1, b2-3 TA2, b4-5 TC1, b6-7 TC2, b8-9 TD1, b10-11 TD2; b16 getChannelReaction Short;
	                  * b17 still Short after a late setChannelReaction(Open_Circuit); b18 OUTxR A unchanged by it */
	R_REACT_OPEN = 1,/* same codes after setChannelReaction(Open_Circuit) before init: checked on the leg table */
	R_DEV = 2,       /* b0 telemetry flag, b1 isDevelopmentMode, b8-15 disarm rc, b16 outputs enabled, b17 trips armed,
	                  * b18 CPU supervision armed, b24-31 disarm rc once the PWM ran */
	R_WRAP0 = 3,     /* rc (int8) a max 120 > 100, b max 80, c {V1_LOW 60, V2_LOW 75 > 70}, d min 5 < 10 */
	R_WRAP1 = 4,     /* rc e min 20, f sensor 0, g TEMP_SENSOR_1 (no board limit) max 1000, h count 100 */
	R_WRAP_V0 = 5,   /* V_HIGH max after a (float bits) */
	R_WRAP_V1 = 6,   /* V_HIGH max after b */
	R_WRAP_V2 = 7,   /* V1_LOW max after c */
	R_WRAP_V3 = 8,   /* V_HIGH min after d */
	R_WRAP_V4 = 9,   /* V_HIGH min after e */
	R_NVS = 10,      /* b0 entry existed before, b8-15 rc retrieve looser, b16-23 rc retrieve tighter, b24 entry restored */
	R_NVS_V0 = 11,   /* V_HIGH min after the looser retrieve */
	R_NVS_V1 = 12,   /* V_HIGH min after the tighter retrieve */
	R_NVS_V2 = 13,   /* V_HIGH max after the tighter retrieve */
	R_NVS_RC = 14,   /* b0-7 rc store looser, b8-15 rc store tighter, b16-23 rc restore */
	R_SW = 15,       /* b0-7 rc min 99, b8-15 fault during the disabled window, b16 outputs on in that window,
	                  * b24-31 fault after enableSafetyApi() */
	R_SW_REASON = 16,
	R_SW_VALUE = 17, /* the reading recorded with the trip (float bits) */
	R_SW_FLAGS = 18, /* b0 outputs off, b1 relay open, b2 PWM refused while latched, b3 clear 0, b4 PWM back,
	                  * b8-9 emergency path (1 external event, 2 comparator), b16 trips armed */
	R_SW_PINS = 19,
	R_SW_EXPECT = 20,/* pin levels the fault state gives */
	R_SW_CYC = 21,   /* cycles from the check (re)enabled to the outputs stopped */
	R_CORE_HZ = 22,
	R_BUILD = 23,    /* 1 development build */
	R_RACE = 24,     /* phase A, trip while the PWM starts (trips arming): b0-7 fault, b8 an output on afterwards,
	                  * b9 relay open, b16-23 rc clear */
	R_RACE_PINS = 25,
};

struct report_t {
	uint32_t magic, layout, boot_count, ack, busy, latched_at_boot, pending;
	uint32_t results[N_RESULTS];
};

extern "C" {
__attribute__((used)) volatile char g_safety_api_text[128];   /* fault description after SW_TRIP */
__attribute__((used)) volatile report_t g_safety_api_report;
__attribute__((used)) volatile uint32_t g_safety_api_cmd;
}

/* CPU supervision owns BKP0-9; this test BKP10-31. */
#define BKP_BOOTS   (TAMP->BKP10R)
#define BKP_PENDING (TAMP->BKP29R)
#define BKP_ACK     (TAMP->BKP31R)
static volatile uint32_t *const bkp = &TAMP->BKP11R;   /* results: BKP11..BKP28 hold 18 words */

static const hrtim_tu_number_t units[] = {PWMA, PWMC, PWMD, PWME, PWMF};

static void result_set(uint32_t i, uint32_t v)
{
	g_safety_api_report.results[i] = v;
	if (i < 18U) {
		bkp[i] = v;
	}
}

static uint32_t fbits(float32_t f)
{
	uint32_t b;
	memcpy(&b, &f, sizeof(b));
	return b;
}

static uint32_t u8(int v)
{
	return (uint32_t)(uint8_t)(int8_t)v;
}

static uint32_t pwm_pin_levels(void)
{
	uint32_t a = GPIOA->IDR, b = GPIOB->IDR, c = GPIOC->IDR;
	return ((a >> 8) & 0xFU) | (((b >> 12) & 0xFU) << 4) | (((c >> 8) & 0x3U) << 8) | (((c >> 6) & 0x3U) << 10);
}

static void relay_close(void)
{
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
	GPIOB->BSRR = 1U << 9;
	GPIOB->MODER = (GPIOB->MODER & ~(3U << 18)) | (1U << 18);
}

static bool relay_open(void)
{
	return (GPIOB->IDR & (1U << 9)) == 0U;
}

static uint32_t expected_fault_pins(void)
{
	static const struct { uint8_t timer, bit; } map[] = {{0, 0}, {2, 4}, {3, 6}, {4, 8}, {5, 10}};
	uint32_t m = 0U;
	for (auto e : map) {
		uint32_t outr = HRTIM1->sTimerxRegs[e.timer].OUTxR;
		if ((outr & HRTIM_OUTR_FAULT1) == HRTIM_OUTR_FAULT1_0) m |= 1U << e.bit;
		if ((outr & HRTIM_OUTR_FAULT2) == HRTIM_OUTR_FAULT2_0) m |= 1U << (e.bit + 1U);
	}
	return m;
}

/* Fault codes of a timer's two outputs: 1 active, 2 inactive, 0 no action. */
static uint32_t fault_codes(int timer)
{
	uint32_t outr = HRTIM1->sTimerxRegs[timer].OUTxR;
	return ((outr >> 4) & 3U) | (((outr >> 20) & 3U) << 2);
}

static bool outputs_enabled(void)
{
	return (HRTIM1->sCommonRegs.OENR & 0xFFFU) != 0U;
}

static bool pwm_init_done;

static void pwm_init(void)
{
	if (!pwm_init_done) {
		spin.pwm.initFixedFrequency(200000);
		for (auto u : units) {
			spin.pwm.initUnit(u);
			spin.pwm.setDutyCycle(u, 0.5);
		}
		pwm_init_done = true;
	}
}

[[maybe_unused]] static void pwm_start_all(void)
{
	pwm_init();
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
	g_safety_api_report.busy = 0U;
	g_safety_api_report.ack = seq;
}

/* Floating current-sensor pins: discharge and leave analog (below every limit). */
static void pin_discharge(GPIO_TypeDef *g, uint32_t pin)
{
	g->BSRR = 1U << (pin + 16U);
	g->MODER = (g->MODER & ~(3U << (2U * pin))) | (1U << (2U * pin));
	k_busy_wait(20);
	g->MODER |= 3U << (2U * pin);
}

static void quiet_pins(void)
{
	pin_discharge(GPIOA, 1);
	pin_discharge(GPIOC, 1);
}

/* Thresholds from the devicetree (or the NVS), no sensor watched. */
static void init_thresholds(void)
{
	sensor_t none = V_HIGH;
	safety.initShield(&none, 0);
}

/* setChannelReaction() maps to the leg safe states, applied when the PWM is
 * initialised and refused once it is. Runs first after a reset. */
static void case_reaction(void)
{
	safety.setChannelReaction(Open_Circuit);
	uint32_t a = 0U, c = 0U;
	hrtim_safety_fault_levels(PWMA, &a, &c);
	uint32_t open_a = (a == LL_HRTIM_OUT_FAULTSTATE_INACTIVE ? 2U : 1U) | ((c == LL_HRTIM_OUT_FAULTSTATE_INACTIVE ? 2U : 1U) << 2);
	hrtim_safety_fault_levels(PWMC, &a, &c);
	open_a |= ((a == LL_HRTIM_OUT_FAULTSTATE_INACTIVE ? 2U : 1U) << 4) | ((c == LL_HRTIM_OUT_FAULTSTATE_INACTIVE ? 2U : 1U) << 6);
	result_set(R_REACT_OPEN, open_a);

	safety.setChannelReaction(Short_Circuit);
	bool short_set = safety.getChannelReaction() == Short_Circuit;
	pwm_init();   /* fault levels written, counters enabled */
	uint32_t v = fault_codes(0) | (fault_codes(2) << 4) | (fault_codes(3) << 8);
	uint32_t outr_a = HRTIM1->sTimerxRegs[0].OUTxR;
	safety.setChannelReaction(Open_Circuit);   /* too late: refused with a message */
	v |= (short_set ? 1U << 16 : 0U) | (safety.getChannelReaction() == Short_Circuit ? 1U << 17 : 0U)
	   | (HRTIM1->sTimerxRegs[0].OUTxR == outr_a ? 1U << 18 : 0U);
	result_set(R_REACT, v);
}

/* Development mode: disarm() before the first PWM output leaves the hardware
 * trips unarmed; CPU supervision still arms. A production build only reports
 * its mode (disarm() does not compile there: tests/l0/safety_api build test). */
static void case_dev(void)
{
	uint32_t v = (g_safety_development_mode ? 1U : 0U) | (safety.isDevelopmentMode() ? 2U : 0U);
#if SAFETY_DEVELOPMENT_MODE
	v |= u8(safety.disarm()) << 8;
	quiet_pins();
	pwm_start_all();
	k_msleep(2);
	v |= (outputs_enabled() ? 1U << 16 : 0U) | (safety_trips_armed() ? 1U << 17 : 0U)
	   | (safety_supervision_armed() ? 1U << 18 : 0U);
	pwm_stop_all();
	v |= u8(safety.disarm()) << 24;
#endif
	result_set(R_DEV, v);
	result_set(R_BUILD, SAFETY_DEVELOPMENT_MODE);
}

/* Software thresholds: tighter than the devicetree accepted, looser refused in
 * production (all or nothing), accepted in development. Twist 1.4.1 limits:
 * V_HIGH 10..100, V1_LOW/V2_LOW -10..70, TEMP_SENSOR_1 none (node disabled). */
static void case_wrappers(void)
{
	init_thresholds();   /* devicetree values */
	sensor_t vh = V_HIGH;
	float32_t x;
	int a, b, c, d, e, f, g, h;
	x = 120.0f; a = safety.setChannelThresholdMax(&vh, &x, 1);
	result_set(R_WRAP_V0, fbits(safety.getChannelThresholdMax(V_HIGH)));
	x = 80.0f; b = safety.setChannelThresholdMax(&vh, &x, 1);
	result_set(R_WRAP_V1, fbits(safety.getChannelThresholdMax(V_HIGH)));
	sensor_t two[] = {V1_LOW, V2_LOW};
	float32_t vals[] = {60.0f, 75.0f};
	c = safety.setChannelThresholdMax(two, vals, 2);
	result_set(R_WRAP_V2, fbits(safety.getChannelThresholdMax(V1_LOW)));
	x = 5.0f; d = safety.setChannelThresholdMin(&vh, &x, 1);
	result_set(R_WRAP_V3, fbits(safety.getChannelThresholdMin(V_HIGH)));
	x = 20.0f; e = safety.setChannelThresholdMin(&vh, &x, 1);
	result_set(R_WRAP_V4, fbits(safety.getChannelThresholdMin(V_HIGH)));
	sensor_t bad = (sensor_t)0;
	f = safety.setChannelThresholdMax(&bad, &x, 1);
	sensor_t t1 = TEMP_SENSOR_1;
	x = 1000.0f; g = safety.setChannelThresholdMax(&t1, &x, 1);
	h = safety.setChannelThresholdMax(&vh, &x, 100);
	result_set(R_WRAP0, u8(a) | (u8(b) << 8) | (u8(c) << 16) | (u8(d) << 24));
	result_set(R_WRAP1, u8(e) | (u8(f) << 8) | (u8(g) << 16) | (u8(h) << 24));
	init_thresholds();   /* back to the devicetree values */
}

/* NVS: a stored threshold looser than the devicetree is ignored in production.
 * The board's own entry, if any, is put back afterwards (or the test entry
 * deleted), so the next firmware sees what it saw before. */
static void case_nvs(void)
{
	const uint16_t id = MEASURE_THRESHOLD | (V_HIGH & 0x0F);
	uint8_t orig[40];
	int n = nvs_storage_retrieve_data(id, orig, sizeof(orig));
	bool existed = n > 0;
	init_thresholds();
	uint8_t buf[10] = {0, (uint8_t)V_HIGH};
	float32_t lo = 5.0f, hi = 90.0f;   /* low looser than 10 */
	memcpy(&buf[2], &lo, 4);
	memcpy(&buf[6], &hi, 4);
	int s1 = nvs_storage_store_data(id, buf, sizeof(buf));
	int r1 = safety.retrieveThreshold(V_HIGH);
	result_set(R_NVS_V0, fbits(safety.getChannelThresholdMin(V_HIGH)));
	lo = 20.0f;
	memcpy(&buf[2], &lo, 4);
	int s2 = nvs_storage_store_data(id, buf, sizeof(buf));
	int r2 = safety.retrieveThreshold(V_HIGH);
	result_set(R_NVS_V1, fbits(safety.getChannelThresholdMin(V_HIGH)));
	result_set(R_NVS_V2, fbits(safety.getChannelThresholdMax(V_HIGH)));
	int s3 = existed ? nvs_storage_store_data(id, orig, (uint8_t)n) : nvs_storage_delete_data(id);
	uint8_t chk[40];
	int m = nvs_storage_retrieve_data(id, chk, sizeof(chk));
	bool restored = existed ? (m == n && memcmp(chk, orig, n) == 0) : (m <= 0);
	result_set(R_NVS, (existed ? 1U : 0U) | (u8(r1) << 8) | (u8(r2) << 16) | (restored ? 1U << 24 : 0U));
	result_set(R_NVS_RC, u8(s1) | (u8(s2) << 8) | (u8(s3) << 16));
	init_thresholds();
}

static void control_task(void)
{
}

/* The software check reacts like every other trip: latch, emergency stop
 * through the HRTIM, safe outputs, readable text. Data acquisition must be
 * started by the scheduler (dispatched data): the case reboots first. V_HIGH
 * reads about 0 V on a bare Spin; its software min is tightened to 99. In
 * production disableSafetyApi() is ignored, in development it holds the check
 * off until enableSafetyApi(). */
static void case_sw_trip_start(uint32_t seq)
{
	BKP_PENDING = (seq << 8) | CASE_SW_TRIP;
	sys_reboot(SYS_REBOOT_COLD);
}

static void case_sw_trip_after_reboot(void)
{
	(void)safety.clearFault();
	quiet_pins();
	pwm_init();   /* the scheduler needs the HRTIM period */
	shield.sensors.enableDefaultTwistSensors();
	init_thresholds();
	sensor_t vh = V_HIGH;
	float32_t x = 99.0f;
	int rc = safety.setChannelThresholdMin(&vh, &x, 1);
	safety.setChannelWatch(&vh, 1);
	task.createCritical(control_task, 100, source_tim6);
	task.startCritical(true);   /* data acquisition with dispatch, before the first PWM output */

	/* Phase A: the check trips while the first output enable is still arming
	 * the hardware trips (milliseconds): no output may be left on. */
	relay_close();
	for (auto u : units) {
		spin.pwm.startDualOutput(u);
	}
	k_msleep(20);
	uint32_t race = (uint32_t)safety.getFault() | (outputs_enabled() ? 1U << 8 : 0U) | (relay_open() ? 1U << 9 : 0U);
	result_set(R_RACE_PINS, pwm_pin_levels());
	safety.unsetChannelWatch(&vh, 1);
	pwm_stop_all();
	k_msleep(5);
	race |= u8(safety.clearFault()) << 16;
	result_set(R_RACE, race);
	k_msleep(150);   /* beyond the repeat-clear window (100 ms in this build) */

	/* Phase B: everything armed, then the check sees V_HIGH out of its limit. */
	for (auto u : units) {
		spin.pwm.startDualOutput(u);
	}
	k_msleep(10);
	relay_close();
	safety.disableSafetyApi();   /* ignored in production */
	safety.setChannelWatch(&vh, 1);
	uint32_t t0 = DWT->CYCCNT;
	k_msleep(50);
	uint32_t f1 = (uint32_t)safety.getFault();
	bool on1 = outputs_enabled();
	uint32_t t1 = DWT->CYCCNT;
	safety.enableSafetyApi();
	k_msleep(50);
	safety_fault_record_t rec;
	safety_supervision_get_record(&rec);
	uint32_t flags = (!outputs_enabled() ? 1U : 0U) | (relay_open() ? 2U : 0U)
		       | ((uint32_t)safety_trips_last_force_path() << 8) | (safety_trips_armed() ? 1U << 16 : 0U);
	result_set(R_SW_PINS, pwm_pin_levels());
	result_set(R_SW_EXPECT, expected_fault_pins());
	char text[128];
	safety.getFaultDescription(text, sizeof(text));
	for (size_t i = 0; i < sizeof(text); i++) {
		g_safety_api_text[i] = text[i];
	}
	for (auto u : units) {
		spin.pwm.startDualOutput(u);   /* refused while latched */
	}
	flags |= !outputs_enabled() ? 4U : 0U;
	safety.unsetChannelWatch(&vh, 1);
	pwm_stop_all();
	k_msleep(5);
	int c = safety.clearFault();
	flags |= (c == 0) ? 8U : 0U;
	for (auto u : units) {
		spin.pwm.startDualOutput(u);
	}
	k_msleep(5);
	flags |= outputs_enabled() ? 16U : 0U;
	pwm_stop_all();
	task.stopCritical();
	uint32_t ref = SAFETY_DEVELOPMENT_MODE ? t1 : t0;
	result_set(R_SW, u8(rc) | (f1 << 8) | (on1 ? 1U << 16 : 0U) | ((uint32_t)rec.fault << 24));
	result_set(R_SW_REASON, rec.reason);
	result_set(R_SW_VALUE, rec.pc);
	result_set(R_SW_FLAGS, flags);
	result_set(R_SW_CYC, rec.disable_cycles - ref);
	result_set(R_CORE_HZ, SystemCoreClock);
	result_set(R_BUILD, SAFETY_DEVELOPMENT_MODE);
}

int main(void)
{
	g_safety_api_report.layout = 1U;
	BKP_BOOTS = BKP_BOOTS + 1U;
	g_safety_api_report.boot_count = BKP_BOOTS;
	g_safety_api_report.latched_at_boot = (uint32_t)safety_supervision_get_fault();
	for (uint32_t i = 0; i < 18U; i++) {
		g_safety_api_report.results[i] = bkp[i];
	}
	uint32_t pending = BKP_PENDING;
	g_safety_api_report.pending = pending;
	g_safety_api_report.ack = BKP_ACK;
	g_safety_api_report.magic = REPORT_MAGIC;
	uint32_t last = BKP_ACK;
	if (pending != 0U) {
		last = pending >> 8;
		g_safety_api_report.busy = last;
		BKP_PENDING = 0U;
		if ((pending & 0xFFU) == CASE_SW_TRIP) {
			case_sw_trip_after_reboot();
		}
		ack(last);
	}
	g_safety_api_cmd = (last << 8) | 0xFFU;

	for (;;) {
		uint32_t cmd = g_safety_api_cmd, seq = cmd >> 8, c = cmd & 0xFFU;
		if (seq != last && seq != 0U) {
			last = seq;
			g_safety_api_report.busy = seq;
			switch (c) {
			case CASE_CLEAR:
				for (uint32_t i = 0; i < N_RESULTS; i++) {
					result_set(i, 0U);
				}
				BKP_PENDING = 0U;
				quiet_pins();
				(void)safety.clearFault();
				break;
			case CASE_REACTION: case_reaction(); break;
			case CASE_DEV: case_dev(); break;
			case CASE_WRAPPERS: case_wrappers(); break;
			case CASE_NVS: case_nvs(); break;
			case CASE_SW_TRIP: case_sw_trip_start(seq); break;
			default: break;
			}
			ack(seq);
		}
		k_msleep(10);
	}
	return 0;
}
