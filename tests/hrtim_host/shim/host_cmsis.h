/* Force-included: replaces cmsis_gcc.h (ARM asm) with host no-ops */
#pragma once
#define __CMSIS_GCC_H
#include <stdint.h>
#define __ASM __asm
#define __INLINE inline
#define __STATIC_INLINE static inline
#define __STATIC_FORCEINLINE static inline
#define __NO_RETURN __attribute__((__noreturn__))
#define __USED __attribute__((used))
#define __WEAK __attribute__((weak))
#define __PACKED __attribute__((packed, aligned(1)))
#define __PACKED_STRUCT struct __attribute__((packed, aligned(1)))
#define __PACKED_UNION union __attribute__((packed, aligned(1)))
#define __ALIGNED(x) __attribute__((aligned(x)))
#define __RESTRICT __restrict
#define __COMPILER_BARRIER() __asm volatile("" ::: "memory")
#define __UNALIGNED_UINT32_READ(addr) (*(const uint32_t *)(addr))
#define __UNALIGNED_UINT32_WRITE(addr, val) (*(uint32_t *)(addr) = (val))
static inline void __DSB(void) {}
static inline void __ISB(void) {}
static inline void __DMB(void) {}
static inline void __NOP(void) {}
static inline uint32_t __get_FPSCR(void) { return 0; }
static inline void __set_FPSCR(uint32_t v) { (void)v; }
static inline uint32_t __RBIT(uint32_t v)
{
    uint32_t r = 0;
    for (int i = 0; i < 32; i++) { r = (r << 1) | (v & 1U); v >>= 1; }
    return r;
}
static inline uint8_t __CLZ(uint32_t v) { return v ? (uint8_t)__builtin_clz(v) : 32U; }
