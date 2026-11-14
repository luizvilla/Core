/*
 * Copyright (c) 2026-present LAAS-CNRS
 *
 * SPDX-License-Identifier: LGPL-2.1
 */

/**
 * @file main.cpp
 * @brief Build test (ADR 0009, #22): CONFIG_OWNTECH_SAFETY_RUNTIME_SELFTEST=n
 *        (app.conf) in a production build must fail. Never flashed.
 */

#include "SpinAPI.h"

int main(void)
{
	return 0;
}
