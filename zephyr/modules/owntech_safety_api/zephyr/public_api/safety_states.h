/*
 * Copyright (c) 2026-present LAAS-CNRS
 *
 * SPDX-License-Identifier: LGPL-2.1
 */

/**
 * @file safety_states.h
 * @brief Safe states, safe outputs and the emergency stop (ADR 0009, #20).
 */

#ifndef SAFETY_STATES_H_
#define SAFETY_STATES_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** State of a leg's switches on any trip. */
typedef enum
{
    SAFETY_SAFE_OPEN       = 0, /**< both switches off (default) */
    SAFETY_SAFE_SHORT_LOW  = 1, /**< low-side switch on, high-side off (e.g. motor brake) */
    SAFETY_SAFE_SHORT_HIGH = 2, /**< high-side switch on, low-side off */
} safety_safe_state_t;

/**
 * @brief Sets a leg's safe state (leg index in devicetree order). Only before
 *        the leg's timer is initialised and before arming: the HRTIM fixes the
 *        fault levels once its timer runs. The devicetree property safe-state is
 *        the usual way; this call is for applications that choose at run time.
 * @return 0; -1 too late (timer running or trips armed); -2 bad leg or state.
 */
int safety_set_leg_safe_state(int leg, safety_safe_state_t state);

/** @brief Drives every safe output to its safe level (register-only, any context). */
void safety_outputs_safe(void);

/**
 * @brief Emergency stop for a fault found by software: through a real hardware
 *        fault when a comparator trip is armed (every leg takes its own safe
 *        state), else by disabling the outputs (open); then the safe outputs.
 *        Register-only, any context.
 */
void safety_emergency_stop(void);

/** @brief @return number of safe outputs declared in the devicetree. */
int safety_safe_output_count(void);

/** @brief @return number of legs with a safe state. */
int safety_leg_count(void);

#ifdef __cplusplus
}
#endif

#endif /* SAFETY_STATES_H_ */
