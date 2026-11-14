/*
 * Copyright (c) 2026-present LAAS-CNRS
 *
 * SPDX-License-Identifier: LGPL-2.1
 */

/*
 * CPU register self-test (IEC 60730-1 Table H.2, 1.1 registers; ADR 0009, #21).
 *
 * int safety_cpu_reg_test(void): writes 0xAAAAAAAA then 0x55555555 into R1-R12
 * and reads each back against R0; returns 0 when every register held both
 * patterns, 1 otherwise. R4-R11 are saved and restored (AAPCS).
 */

	.syntax unified
	.thumb
	.text
	.global safety_cpu_reg_test
	.type safety_cpu_reg_test, %function

safety_cpu_reg_test:
	push	{r4-r11, lr}
	ldr	r0, =0xAAAAAAAA
	bl	1f
	cbnz	r0, 9f
	ldr	r0, =0x55555555
	bl	1f
9:
	pop	{r4-r11, pc}

	/* r0 = pattern; returns r0 = 0 if R1-R12 all hold it, 1 otherwise */
1:
	mov	r1, r0
	mov	r2, r0
	mov	r3, r0
	mov	r4, r0
	mov	r5, r0
	mov	r6, r0
	mov	r7, r0
	mov	r8, r0
	mov	r9, r0
	mov	r10, r0
	mov	r11, r0
	mov	r12, r0
	cmp	r1, r0
	bne	8f
	cmp	r2, r0
	bne	8f
	cmp	r3, r0
	bne	8f
	cmp	r4, r0
	bne	8f
	cmp	r5, r0
	bne	8f
	cmp	r6, r0
	bne	8f
	cmp	r7, r0
	bne	8f
	cmp	r8, r0
	bne	8f
	cmp	r9, r0
	bne	8f
	cmp	r10, r0
	bne	8f
	cmp	r11, r0
	bne	8f
	cmp	r12, r0
	bne	8f
	movs	r0, #0
	bx	lr
8:
	movs	r0, #1
	bx	lr

	.ltorg
	.size safety_cpu_reg_test, . - safety_cpu_reg_test
