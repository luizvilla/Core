/* Host shim for the Zephyr kernel API used by hrtim.c */
#pragma once
#include <stdio.h>
#include <stdbool.h>
#define printk printf
static inline void k_busy_wait(unsigned us) { (void)us; }
#define IRQ_CONNECT(...)
static inline void irq_enable(unsigned irq) { (void)irq; }
static inline void irq_disable(unsigned irq) { (void)irq; }
static inline unsigned irq_lock(void) { return 0; }
static inline void irq_unlock(unsigned key) { (void)key; }
