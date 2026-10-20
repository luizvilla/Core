/* Maps RAM at the real HRTIM1 / RCC addresses so that the LL macros
 * (which cast addresses to uint32_t) work unchanged on the host. */
#pragma once
#include <sys/mman.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "hrtim.h"

/* Mapped RAM must not hold another mapping: run on Linux x86_64 */

static void fake_map(uintptr_t addr, size_t len)
{
    void *p = mmap((void *)addr, len, PROT_READ | PROT_WRITE,
                   MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED_NOREPLACE, -1, 0);
    if (p != (void *)addr) { perror("mmap"); exit(2); }
}

static void fake_regs_init(void)
{
    fake_map(0x40016000, 0x1000);  /* HRTIM1 at 0x40016800 */
    fake_map(0x40021000, 0x1000);  /* RCC */
    HRTIM1->sCommonRegs.ISR |= HRTIM_ISR_DLLRDY;  /* DLL calibrated */
}

static int failures = 0;
#define CHECK(cond, ...) do { if (cond) { printf("  ok   "); } \
    else { printf("  FAIL "); failures++; } printf(__VA_ARGS__); printf("\n"); } while (0)

#define TIMREG(tu) (HRTIM1->sTimerxRegs[tu])
#define CKPSC(tu)  (TIMREG(tu).TIMxCR & HRTIM_TIMCR_CK_PSC)
