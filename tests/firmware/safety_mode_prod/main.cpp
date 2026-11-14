/*
 * Copyright (c) 2026-present LAAS-CNRS
 *
 * SPDX-License-Identifier: LGPL-2.1
 */

/**
 * @file main.cpp
 * @brief Build test (ADR 0009, #22): safety.disarm() in a production build must
 *        not compile. tests/l0/safety_api/build_mode.py builds this folder and
 *        expects the build to fail with the development-mode message; the same
 *        source in tests/firmware/safety_mode_dev (development mode) must build.
 *        Never flashed.
 */

#include "SpinAPI.h"
#include "SafetyAPI.h"

int main(void)
{
	(void)safety.disarm();
	return 0;
}
