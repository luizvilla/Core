/* Copyright (c) 2026 OWNTECH TECHNOLOGIES
 * SPDX-License-Identifier: LGPL-2.1
 */
/** @file contract.h
 * @brief Version app-v3 of the Pulsim application firmware-block buffer contract (ADR 0003 boost: burst command and trip; inverter bench stages drive the bridge and the relay, injected current input).
 * @note Matches tests/l1/contract.json; checked by the native metadata smoke test.
 */
#pragma once
/** @brief Input-buffer positions; physical units except Boolean/enumerated commands. */
enum Input {
    VLOW, ILOW1, ILOW2, VBUS, VAC, IAC, VAC_UPDATED, MODE, SYNC_REQUEST, VBUS_REF, INJECT_CURRENT,
    INPUT_COUNT ///< Number of required inputs.
};
/** @brief Output-buffer positions; telemetry is never fed back as an extra sensor. */
enum Output {
    DUTY1, DUTY2, ENABLE1, ENABLE2, MODE_CHANGED, PWM_UPDATE, BRIDGE_DUTY,
    BRIDGE_ENABLE, THETA, OMEGA, OBSERVER_RMS, OBSERVER_HZ, OBSERVER_TRIP,
    INVERTER_MODE, SYNCHRONIZED, GRID_QUALIFIED, BURST_SKIP, BURST_PERIOD,
    IREF1, IREF2, VALPHA, VBETA, VD, VQ, VAC_AGE, COLD_TICK, BOOST_MODE,
    INVOCATIONS, BLOCK_ERROR, INV_PWM_REQUEST, RELAY_REQUEST, INVERTER_FAULT, BOOST_TRIP,
    RELAY_CLOSE, RELAY_FEEDBACK, BENCH_TRIP, ID, IQ,
    OUTPUT_COUNT ///< Number of required outputs.
};
