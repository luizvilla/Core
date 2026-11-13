/*
 * Copyright (c) 2026 OWNTECH TECHNOLOGIES
 * SPDX-License-Identifier: LGPL-2.1
 */

/**
 * @file test_fast_average.cpp
 * @brief L0-FAV: the arithmetic of SensorsAPI::getAverageValueFast (Core2 ADR 0007).
 *
 * The code under test is data_fast_sum_new_samples() (data_fast_average.h): the sum of the samples of one rank
 * written in the circular DMA buffer between two reads. The test plays the DMA: it writes conversions one by one
 * into a buffer of size S (sequence of n channels, slot i holds rank i mod n), computes the newest slot of the rank
 * from CNDTR with the table of DataAPI::prepareFastChannel, and compares the average with the mean of the samples
 * the rank really received since the previous read.
 *
 * | Test case | Behaviour |
 * |---|---|
 * | L0-FAV-01 | Average of the samples written since the previous read (2 or 3 per 50 µs period), both ADC layouts |
 * | L0-FAV-02 | Nothing new, empty slots, the largest gap |
 */

#include "data_fast_average.h"
#include "check.h"

#include <cmath>
#include <cstdint>
#include <vector>

namespace {

constexpr uint16_t empty = 0xFFFF;  ///< DATA_FAST_EMPTY_SAMPLE

/** @brief Circular DMA buffer of one ADC, written one conversion at a time. */
struct Dma {
    uint32_t size;                 ///< S
    uint32_t n;                    ///< Channels in the sequence.
    std::vector<uint16_t> buffer;  ///< Slots.
    uint32_t next = 0;             ///< Next slot written (S - CNDTR).
    uint32_t written = 0;          ///< Conversions since start.
    std::vector<uint32_t> newest;  ///< CNDTR -> newest slot of the rank (DataAPI::prepareFastChannel).

    Dma(uint32_t s, uint32_t count, uint32_t rank_index) : size(s), n(count), buffer(s, empty), newest(s + 1) {
        for (uint32_t c = 0; c <= size; ++c) {
            const uint32_t nw = (2 * size - c - 1) % size;
            const uint32_t back = (nw + n - rank_index) % n;
            newest[c] = (nw + size - back) % size;
        }
    }
    /** @brief The DMA's CNDTR: samples left before the wrap (S when the next write is slot 0). */
    uint32_t cndtr() const { return size - next; }
    /** @brief One conversion: the value lands at the next slot. */
    void convert(uint16_t value) { buffer[next] = value; next = (next + 1) % size; ++written; }
};

/** @brief Value converted for rank r at conversion k: distinct, so a wrong slot shows. */
uint16_t sample(uint32_t k) { return static_cast<uint16_t>(1000 + (k * 37) % 3000); }

/**
 * @brief Runs a DMA with conversions every 20 µs / n (one sequence per 20 µs) and reads every 50 µs, as the Core.
 * @return Worst |average - mean of the rank's new samples| (raw counts), and the counts of 2- and 3-sample reads.
 */
double run(tc::Criterion &c, uint32_t size, uint32_t n, uint32_t rank_index, uint32_t periods, uint32_t phase_conv,
           int &reads2, int &reads3) {
    Dma dma(size, n, rank_index);
    std::vector<uint16_t> rank_samples;  // values the rank received, in order
    // Fill one lap first (the buffer starts empty in hardware; the empty case is L0-FAV-02).
    uint32_t conv = 0;
    for (; conv < size + phase_conv; ++conv) { dma.convert(sample(conv)); if (conv % n == rank_index) rank_samples.push_back(sample(conv)); }
    uint32_t previous = dma.newest[dma.cndtr()];
    size_t consumed = rank_samples.size();
    double worst = 0.0;
    // Time in units of 1/n of 20 µs: a read every 2.5·n conversions (50 µs), shifted by phase_conv.
    const uint32_t per_period_x2 = 5 * n;  // conversions per two periods
    for (uint32_t p = 1; p <= periods; ++p) {
        const uint32_t target = size + phase_conv + (p * per_period_x2) / 2;
        for (; conv < target; ++conv) {
            dma.convert(sample(conv));
            if (conv % n == rank_index) rank_samples.push_back(sample(conv));
        }
        const uint32_t nw = dma.newest[dma.cndtr()];
        uint32_t raw_sum = 0;
        const uint32_t count = data_fast_sum_new_samples(dma.buffer.data(), size, n, nw, previous, empty, &raw_sum);
        const size_t fresh = rank_samples.size() - consumed;
        double mean = 0.0;
        for (size_t k = consumed; k < rank_samples.size(); ++k) mean += rank_samples[k];
        if (fresh > 0) mean /= static_cast<double>(fresh);
        c.check("count = samples the rank received", count == fresh);
        if (count > 0) worst = std::fmax(worst, std::fabs(static_cast<double>(raw_sum) / count - mean));
        if (count == 2) ++reads2;
        if (count == 3) ++reads3;
        previous = nw;
        consumed = rank_samples.size();
    }
    return worst;
}

}  // namespace

int main() {
    // ---------------------------------------------------------------- L0-FAV-01
    struct Layout { const char *id; const char *what; uint32_t size, n, rank; };
    const Layout layouts[] = {
        {"L0-FAV-01.a", "ADC1 layout (S 42, 3 channels, VAC rank 2): average of the 2 or 3 samples since the previous 50 µs read", 42, 3, 1},
        {"L0-FAV-01.b", "ADC2 layout (S 42, 2 channels, IAC rank 2): average of the 2 or 3 samples since the previous 50 µs read", 42, 2, 1},
    };
    for (const Layout &l : layouts) {
        tc::Criterion c(l.id, l.what);
        int r2 = 0, r3 = 0;
        double worst = 0.0;
        for (uint32_t phase = 0; phase < 2 * l.n; ++phase)  // every read phase within a sequence
            worst = std::fmax(worst, run(c, l.size, l.n, l.rank, 1000, phase, r2, r3));
        c.near("average = mean of the new samples (raw counts)", worst, 0.0, 1e-9);
        c.check("both 2- and 3-sample reads occur (2.5 sequences per period)", r2 > 0 && r3 > 0);
        c.check("only 2- or 3-sample reads", r2 + r3 == static_cast<int>(1000 * 2 * l.n));
    }

    // ---------------------------------------------------------------- L0-FAV-02
    {
        /** @test L0-FAV-02.a: Nothing new: no sample summed */
        tc::Criterion c("L0-FAV-02.a", "Nothing new since the previous read: no sample summed");
        Dma dma(42, 3, 1);
        for (uint32_t k = 0; k < 50; ++k) dma.convert(sample(k));
        const uint32_t nw = dma.newest[dma.cndtr()];
        uint32_t sum = 7;
        c.check("count 0", data_fast_sum_new_samples(dma.buffer.data(), 42, 3, nw, nw, empty, &sum) == 0);
        c.check("sum 0", sum == 0);
        dma.convert(sample(50));  // a conversion of another rank: still nothing new for rank 2
        c.check("other rank converted: count 0",
                data_fast_sum_new_samples(dma.buffer.data(), 42, 3, dma.newest[dma.cndtr()], nw, empty, &sum) == 0);
    }
    {
        /** @test L0-FAV-02.b: Empty slots (never written) are skipped; all empty gives no sample */
        tc::Criterion c("L0-FAV-02.b", "Empty slots (never written) are skipped; only empty slots give no sample");
        Dma dma(42, 3, 1);
        const uint32_t start = dma.newest[dma.cndtr()];  // before any conversion: a slot never written
        for (uint32_t k = 0; k < 4; ++k) dma.convert(sample(k));  // rank 2 converted once (conversion 1)
        uint32_t sum = 0;
        const uint32_t count = data_fast_sum_new_samples(dma.buffer.data(), 42, 3, dma.newest[dma.cndtr()], start, empty, &sum);
        c.check("one written sample counted", count == 1 && sum == sample(1));
        Dma blank(42, 3, 1);
        c.check("all empty: count 0",
                data_fast_sum_new_samples(blank.buffer.data(), 42, 3, 4, 1, empty, &sum) == 0);
    }
    {
        /** @test L0-FAV-02.c: The largest gap, one sequence short of a lap, sums S/n - 1 samples */
        tc::Criterion c("L0-FAV-02.c", "Largest gap (one sequence short of a lap): S/n - 1 samples, their mean");
        Dma dma(42, 3, 1);
        for (uint32_t k = 0; k < 42; ++k) dma.convert(sample(k));
        const uint32_t before = dma.newest[dma.cndtr()];
        double mean = 0.0;
        for (uint32_t k = 42; k < 42 + 39; ++k) { dma.convert(sample(k)); if (k % 3 == 1) mean += sample(k); }
        mean /= 13.0;
        uint32_t sum = 0;
        const uint32_t count = data_fast_sum_new_samples(dma.buffer.data(), 42, 3, dma.newest[dma.cndtr()], before, empty, &sum);
        c.check("13 samples", count == 13);
        c.near("mean (raw counts)", static_cast<double>(sum) / count, mean, 1e-9);
    }
    return tc::finish();
}
