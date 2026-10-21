/*
 * SPDX-License-Identifier: LGPL-2.1
 *
 * Measurement over SWD, without a scope: a background task executes the
 * requests written by the debugger into `meas` and writes the results
 * back. Read / written with OpenOCD (mdw / mww) while the program runs.
 */
#pragma once
#include <stdint.h>

#define MEAS_MAGIC      0x4D454153u  /* 'MEAS' */
#define MEAS_REQ_FREQ   1u           /* frequency and counter max, A to F */
#define MEAS_REQ_CAPT   2u           /* logic capture of the PWM pins */
#define MEAS_REQ_REGS   3u           /* copy of the HRTIM registers */
#define MEAS_REQ_TRACE  4u           /* 512 (cycle, counter) pairs of timer meas.key_arg */
#define MEAS_CAPT_LEN   49152u       /* samples, one byte each */

struct meas_t
{
    uint32_t magic;
    volatile uint32_t request;       /* written by the debugger */
    volatile uint32_t key;           /* menu key to execute, 0 = none */
    volatile uint32_t done;          /* incremented after each request / key */
    volatile uint32_t key_arg;       /* timer index for MEAS_REQ_TRACE */
    uint32_t cpu_hz;
    /* MEAS_REQ_FREQ */
    uint32_t window_cycles;          /* CPU cycles measured per timer */
    /* In up-down mode CNTxR reads as a rising sawtooth twice per PWM
     * period (referred to the PER event when down-counting, RM0440
     * 27.3.6): valleys / peaks count 2 per period there. */
    uint32_t valleys[6];             /* counter wraps seen, A..F */
    uint32_t peaks[6];               /* counter maxima seen, A..F */
    uint32_t cnt_max[6];             /* highest counter value read */
    /* MEAS_REQ_CAPT: bit0 PA8, bit1 PA9, bit2 PB12, bit3 PB13, bit4 PB14,
       bit5 PB15, bit6 PC8, bit7 PC9 */
    uint32_t capt_samples;
    uint32_t capt_cycles;            /* CPU cycles for the whole capture */
    uint32_t capt_addr;              /* address of the sample buffer */
    /* MEAS_REQ_REGS */
    uint32_t mcr, cr2, per[6], cmp1[6], cmp3[6], rst[6], timcr[6], timcr2[6],
             outr[6], dtr[6], set1[6], rst1[6];
};

extern struct meas_t meas;

/* Starts the measurement background task. key_handler is called for
 * meas.key (same keys as the serial menu). */
void measure_start(void (*key_handler)(char));
