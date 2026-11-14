/*
 *
 * Copyright (c) 2026 OWNTECH TECHNOLOGIES
 *
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU Lesser General Public License as published by
 *   the Free Software Foundation, either version 2.1 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU Lesser General Public License for more details.
 *
 *   You should have received a copy of the GNU Lesser General Public License
 *   along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 * SPDX-License-Identifier: LGPL-2.1
 */

/**
 * @file arm_math.h
 * @brief Host stand-in for the CMSIS-DSP symbols used by the firmware and its libraries.
 * @note Test-only: never compiled into the firmware. See tests/common/README.md.
 */
#pragma once
#include <math.h>
#include <stdint.h>
typedef float float32_t;
typedef double float64_t;
typedef int32_t q31_t;
#ifndef PI
#define PI 3.14159265358979f
#endif
static inline float32_t arm_sin_f32(float32_t x) { return sinf(x); }
static inline float32_t arm_cos_f32(float32_t x) { return cosf(x); }
/* fir.h only needs the instance type and init/apply prototypes. */
typedef struct { uint16_t numTaps; float32_t *pState; const float32_t *pCoeffs; } arm_fir_instance_f32;
static inline void arm_fir_init_f32(arm_fir_instance_f32 *S, uint16_t numTaps,
    const float32_t *pCoeffs, float32_t *pState, uint32_t blockSize)
{ S->numTaps = numTaps; S->pCoeffs = pCoeffs; S->pState = pState; (void)blockSize; }
static inline void arm_fir_f32(const arm_fir_instance_f32 *S, const float32_t *pSrc,
    float32_t *pDst, uint32_t blockSize) { (void)S; (void)pSrc; (void)pDst; (void)blockSize; }
