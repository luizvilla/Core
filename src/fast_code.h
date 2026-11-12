/*
 * Copyright (c) 2026-present LAAS-CNRS
 * SPDX-License-Identifier: LGPL-2.1
 */

/*
 * FAST_CODE: per-period code executed from CCM SRAM on target (owntech_fast_code.h); empty in the host
 * L0 builds, which compile the same sources.
 */

#ifndef FAST_CODE_H_
#define FAST_CODE_H_

#if defined(__ZEPHYR__)
#include "owntech_fast_code.h"
#define FAST_CODE OWNTECH_FAST_CODE
#else
#define FAST_CODE
#endif

#endif /* FAST_CODE_H_ */
