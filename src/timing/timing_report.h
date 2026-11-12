/*
 * Copyright (c) 2026-present LAAS-CNRS
 * SPDX-License-Identifier: LGPL-2.1
 */

/**
 * @file timing_report.h
 * @brief RAM layout of the on-target timing results, read over SWD by tests/l0/timing/read_report.py.
 *
 * The host finds the symbol g_timing_report in firmware.elf and dumps it with OpenOCD. Any change here must
 * bump timing_layout_version and be mirrored in tests/l0/timing/read_report.py (the static_asserts below pin
 * the offsets the reader uses).
 */
#ifndef TIMING_REPORT_H
#define TIMING_REPORT_H

#include <stddef.h>
#include <stdint.h>

/** @brief "TIM1" in little-endian: the report is valid. */
#define TIMING_MAGIC 0x314D4954UL
/** @brief Bumped on any layout change. */
#define TIMING_LAYOUT_VERSION 4UL
#define TIMING_MAX_BUCKETS 16U
#define TIMING_MAX_ROWS 400U

/** @brief Run state of the harness. */
enum TimingState : uint32_t {
    TIMING_RUNNING = 0U,  ///< Scenario in progress.
    TIMING_DONE = 1U,     ///< Every row final.
    TIMING_ERROR = 2U,    ///< Stopped; error_code says why.
    TIMING_TIM01_DONE = 3U ///< L0-TIM-01 rows final, L0-TIM-02 running.
};

/** @brief Cost of one code section in one scenario bucket. */
struct TimingRow {
    char name[32];        ///< "<bucket>/<section>".
    uint32_t calls;       ///< Samples taken.
    uint32_t min_cycles;  ///< Shortest, in core cycles.
    uint32_t max_cycles;  ///< Longest (worst case), in core cycles.
    uint32_t max_tick;    ///< Scenario tick of the worst case.
    uint64_t sum_cycles;  ///< Sum, for the mean.
};

/** @brief What the scenario reached in one bucket (stage and boost mode). */
struct TimingBucket {
    char name[16];          ///< Bucket label, e.g. "CONNECT_g".
    uint32_t ticks;         ///< Control steps run in this bucket.
    uint32_t vsi_seen_mask; ///< Bit n set when the inverter was in vsi_mode_t n.
    uint8_t final_vsi;      ///< Inverter mode at the last tick of the bucket.
    uint8_t boost_mode;     ///< boost_mode used in this bucket.
    uint8_t pad[2];
    uint32_t tim02_ticks;          ///< L0-TIM-02 ticks in this bucket.
    uint32_t tim02_vsi_seen_mask;  ///< Inverter modes seen during L0-TIM-02.
};

/** @brief Everything the host reads. */
struct TimingReport {
    uint32_t magic;           ///< TIMING_MAGIC once initialized.
    uint32_t layout_version;  ///< TIMING_LAYOUT_VERSION.
    uint32_t state;           ///< TimingState.
    uint32_t core_hz;         ///< Core clock, cycles per second.
    uint32_t bucket_count;    ///< Buckets filled.
    uint32_t row_count;       ///< Rows filled.
    uint32_t error_code;      ///< Non-zero with TIMING_ERROR.
    uint32_t total_ticks;     ///< Control steps run so far (progress).
    TimingBucket buckets[TIMING_MAX_BUCKETS];
    TimingRow rows[TIMING_MAX_ROWS];
};

static_assert(sizeof(TimingRow) == 56, "TimingRow layout");
static_assert(sizeof(TimingBucket) == 36, "TimingBucket layout");
static_assert(offsetof(TimingReport, buckets) == 32, "TimingReport header layout");
static_assert(offsetof(TimingReport, rows) == 32 + 36 * TIMING_MAX_BUCKETS, "TimingReport rows offset");

#endif  // TIMING_REPORT_H
