/*
 * SPDX-License-Identifier: LGPL-2.1
 */
#include <zephyr/kernel.h>
#include <soc.h>
#include "TaskAPI.h"
#include "measure.h"

struct meas_t meas;
static uint8_t capt_buf[MEAS_CAPT_LEN];
static void (*key_handler)(char) = nullptr;

static HRTIM_Timerx_TypeDef *timer(int i) { return &HRTIM1->sTimerxRegs[i]; }

/* Counts valleys / peaks of one up-down counter during `ms` slices of 1 ms
 * with interrupts locked (an interrupt longer than half a period would
 * hide a direction change). */
static void measure_timer(int t, uint32_t ms)
{
    volatile uint32_t *cnt = &timer(t)->CNTxR;
    uint32_t slice = meas.cpu_hz / 1000u;
    uint32_t valleys = 0, peaks = 0, cmax = 0, total = 0;

    for (uint32_t s = 0; s < ms; s++)
    {
        unsigned int key = irq_lock();
        uint32_t prev = *cnt;
        int dir = 0;                         /* 1 up, -1 down, 0 unknown */
        uint32_t t0 = DWT->CYCCNT;
        while ((DWT->CYCCNT - t0) < slice)
        {
            uint32_t now = *cnt;
            if (now > cmax) cmax = now;
            if (now > prev)
            {
                if (dir == -1) valleys++;
                dir = 1;
            }
            else if (now < prev)
            {
                if (dir == 1) peaks++;
                dir = -1;
            }
            prev = now;
        }
        total += DWT->CYCCNT - t0;
        irq_unlock(key);
    }
    meas.valleys[t] = valleys;
    meas.peaks[t] = peaks;
    meas.cnt_max[t] = cmax;
    meas.window_cycles = total;
}

static void capture(void)
{
    volatile uint32_t *a = &GPIOA->IDR, *b = &GPIOB->IDR, *c = &GPIOC->IDR;
    unsigned int key = irq_lock();
    uint32_t t0 = DWT->CYCCNT;
    for (uint32_t i = 0; i < MEAS_CAPT_LEN; i++)
    {
        uint32_t va = *a, vb = *b, vc = *c;
        capt_buf[i] = (uint8_t)(((va >> 8) & 0x3u) | ((vb >> 10) & 0x3Cu) |
                                ((vc >> 2) & 0xC0u));
    }
    meas.capt_cycles = DWT->CYCCNT - t0;
    irq_unlock(key);
    meas.capt_samples = MEAS_CAPT_LEN;
}

static void trace(int t)
{
    volatile uint32_t *cnt = &timer(t)->CNTxR;
    uint32_t *out = (uint32_t *)capt_buf;
    unsigned int key = irq_lock();
    for (int i = 0; i < 512; i++)
    {
        out[2 * i] = DWT->CYCCNT;
        out[2 * i + 1] = *cnt;
    }
    irq_unlock(key);
}

static void copy_regs(void)
{
    meas.mcr = HRTIM1->sMasterRegs.MCR;
    meas.cr2 = HRTIM1->sCommonRegs.CR2;
    for (int i = 0; i < 6; i++)
    {
        meas.per[i] = timer(i)->PERxR;
        meas.cmp1[i] = timer(i)->CMP1xR;
        meas.cmp3[i] = timer(i)->CMP3xR;
        meas.rst[i] = timer(i)->RSTxR;
        meas.timcr[i] = timer(i)->TIMxCR;
        meas.timcr2[i] = timer(i)->TIMxCR2;
        meas.outr[i] = timer(i)->OUTxR;
        meas.dtr[i] = timer(i)->DTxR;
        meas.set1[i] = timer(i)->SETx1R;
        meas.rst1[i] = timer(i)->RSTx1R;
    }
}

static void measure_task(void)
{
    if (meas.key != 0)
    {
        if (key_handler) key_handler((char)meas.key);
        meas.key = 0;
        meas.done++;
    }
    switch (meas.request)
    {
    case MEAS_REQ_FREQ:
        for (int t = 0; t < 6; t++)
        {
            /* only running timers */
            if (HRTIM1->sMasterRegs.MCR & (HRTIM_MCR_TACEN << t))
                measure_timer(t, 100);
            else
                meas.valleys[t] = meas.peaks[t] = meas.cnt_max[t] = 0;
        }
        break;
    case MEAS_REQ_CAPT:
        capture();
        break;
    case MEAS_REQ_REGS:
        copy_regs();
        break;
    case MEAS_REQ_TRACE:
        trace((int)meas.key_arg);
        break;
    default:
        break;
    }
    if (meas.request != 0)
    {
        meas.request = 0;
        meas.done++;
    }
    task.suspendBackgroundMs(5);
}

void measure_start(void (*handler)(char))
{
    key_handler = handler;
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    meas.cpu_hz = SystemCoreClock;
    meas.capt_addr = (uint32_t)capt_buf;
    meas.magic = MEAS_MAGIC;
    uint32_t n = task.createBackground(measure_task);
    task.startBackground(n);
}
