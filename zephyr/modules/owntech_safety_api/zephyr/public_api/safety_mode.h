/*
 * Copyright (c) 2026-present LAAS-CNRS
 *
 * SPDX-License-Identifier: LGPL-2.1
 */

/**
 * @file safety_mode.h
 * @brief Production or development mode of the Safety API (ADR 0009, #22).
 *
 * The mode comes from the devicetree: safety-mode = "development" in a
 * "safety-config" node of app.overlay. Any other build is a production build:
 * there, disarming the hardware trips or loosening a limit does not compile.
 */

#ifndef SAFETY_MODE_H_
#define SAFETY_MODE_H_

#include <stdbool.h>
#include <zephyr/devicetree.h>

#define SAFETY_CONFIG_NODE DT_INST(0, safety_config)

#if DT_NODE_HAS_STATUS(SAFETY_CONFIG_NODE, okay) && \
	DT_ENUM_IDX_OR(SAFETY_CONFIG_NODE, safety_mode, 0) == 1
#define SAFETY_DEVELOPMENT_MODE 1
#else
#define SAFETY_DEVELOPMENT_MODE 0
#endif

#ifdef __cplusplus
extern "C" {
#endif

/** Telemetry flag, readable over SWD: 1 in a development build, 0 otherwise. */
extern const volatile unsigned char g_safety_development_mode;

/** @brief @return true in a development build (safety-mode = "development"). */
bool safety_development_mode(void);

/** @brief @return true once safety_disarm() left the hardware trips unarmed. */
bool safety_disarmed(void);

#if SAFETY_DEVELOPMENT_MODE
/**
 * @brief Development mode only: the comparator and ADC-watchdog trips are not
 *        armed when the PWM starts. CPU supervision stays on.
 * @return 0; -1 too late (the trips are armed and locked until the next reset).
 */
int safety_disarm(void);
#endif

#ifdef __cplusplus
}
#endif

#endif /* SAFETY_MODE_H_ */
