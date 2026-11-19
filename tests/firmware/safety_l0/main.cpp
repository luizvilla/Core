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
 *
 * DAC2_CH1 is reserved from the protection allocation (app.overlay, reserved-dacs). The calibration output
 * uses TIM1, not the HRTIM: starting the HRTIM arms the trips, and on a bare Spin the floating sensor pins
 * would trip them. PA8 is a leg output on most shields: /Cal refuses while the PWM runs and the reverse.
 * C10 adds software fault injection.
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
	pwm_running = true;
	return 0;
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

/* ------------------------------------------------------------------ ThingSet */

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

/* ------------------------------------------------------------------ main */

static void background(void)
{
	refresh_status();
	task.suspendBackgroundMs(100);
}

int main(void)
{
	inj_init();
	refresh_status();
	uint32_t bg = task.createBackground(background);
	task.startBackground(bg);
	return 0;
}
