# Copied unchanged from Core4 tests/twt_bnc/analysis.py (branch test/twt-bnc, 702d34e): waveform metrics, standard library only.
#
# Copyright (c) 2021-present LAAS-CNRS
#
# SPDX-License-Identifier: GPL-2.0-or-later
#

"""Waveform metrics computed on the host from scope captures (pure Python).

Every function takes a waveform as two equal-length lists: times in seconds
and volts. Levels follow the usual oscilloscope definitions: ``top`` and
``base`` are the medians of the samples above and below the mid level,
edges are interpolated crossings of the 50 % level.
"""

import statistics


def mean(volts):
    return statistics.fmean(volts)


def levels(volts):
    """(base, top) of a two-level signal."""
    lo, hi = min(volts), max(volts)
    mid = (lo + hi) / 2.0
    upper = [v for v in volts if v >= mid]
    lower = [v for v in volts if v < mid]
    top = statistics.median(upper) if upper else hi
    base = statistics.median(lower) if lower else lo
    return base, top


def is_switching(volts, min_amplitude):
    base, top = levels(volts)
    return (top - base) >= min_amplitude


def edges(times, volts, hysteresis=0.25):
    """Edges with hysteresis: an edge counts once the signal passes 75 % of
    the amplitude (rising) or 25 % (falling) after being on the other side;
    its time is the interpolated crossing of the 50 % level just before.
    Edge ringing (seen on the bench: +/-1.2 V at ~25 MHz) therefore never
    counts as extra edges. Returns (rising, falling) lists of times."""
    base, top = levels(volts)
    amp = top - base
    if amp <= 0:
        return [], []
    mid = base + amp / 2.0
    hi, lo = base + amp * (1 - hysteresis), base + amp * hysteresis
    rising, falling = [], []
    state = None
    for i, v in enumerate(volts):
        if state != "high" and v >= hi:
            if state == "low":
                rising.append(_mid_crossing(times, volts, i, mid, up=True))
            state = "high"
        elif state != "low" and v <= lo:
            if state == "high":
                falling.append(_mid_crossing(times, volts, i, mid, up=False))
            state = "low"
    return rising, falling


def _mid_crossing(times, volts, i, mid, up):
    """Walk back from sample i to the last 50 % crossing, interpolate."""
    j = i
    while j > 0 and ((volts[j - 1] >= mid) if up else (volts[j - 1] <= mid)):
        j -= 1
    if j == 0:
        return times[i]
    a, b = volts[j - 1], volts[j]
    frac = (mid - a) / (b - a) if b != a else 0.0
    return times[j - 1] + frac * (times[j] - times[j - 1])


def period(times, volts):
    """Mean period from whichever edge direction has more crossings."""
    rising, falling = edges(times, volts)
    best = rising if len(rising) >= len(falling) else falling
    if len(best) < 2:
        return None
    p = (best[-1] - best[0]) / (len(best) - 1)
    return p if p > 0 else None


def frequency(times, volts):
    p = period(times, volts)
    return 1.0 / p if p else None


def duty(times, volts):
    """High-time fraction over the whole periods in the capture (from
    rising to rising edge, or falling to falling if that gives more)."""
    rising, falling = edges(times, volts)
    if len(rising) >= len(falling):
        starts, ends, high_first = rising, falling, True
    else:
        starts, ends, high_first = falling, rising, False
    if len(starts) < 2:
        return None
    first, last = starts[0], starts[-1]
    part = 0.0
    for s in starts[:-1]:
        nxt = [x for x in ends if x > s]
        if not nxt:
            return None
        part += min(nxt[0], last) - s
    fraction = part / (last - first)
    return fraction if high_first else 1.0 - fraction


def overshoot_pct(volts):
    """Peak above top (or below base) relative to the amplitude, in %."""
    base, top = levels(volts)
    amplitude = top - base
    if amplitude <= 0:
        return None
    return 100.0 * max(max(volts) - top, base - min(volts)) / amplitude


def dead_times(times, high, low):
    """Dead times of a complementary pair, in seconds.

    rising: low output falls -> high output rises (gap before H turns on)
    falling: high output falls -> low output rises (gap before L turns on)
    Each is the median over the edges in the capture; None if no pair."""
    h_rise, h_fall = edges(times, high)
    l_rise, l_fall = edges(times, low)

    def gaps(starts, ends):
        out = []
        for e in ends:
            before = [s for s in starts if s <= e]
            if before:
                out.append(e - before[-1])
        return out

    rising = gaps(l_fall, h_rise)
    falling = gaps(h_fall, l_rise)
    return (statistics.median(rising) if rising else None,
            statistics.median(falling) if falling else None)


def overlap_fraction(high, low):
    """Fraction of samples where both outputs of a pair are above 50 %."""
    hb, ht = levels(high)
    lb, lt = levels(low)
    hm, lm = (hb + ht) / 2.0, (lb + lt) / 2.0
    both = sum(1 for h, l in zip(high, low) if h > hm and l > lm)
    return both / len(high)


def phase_deg(times, reference, other):
    """Delay of ``other``'s rising edges after ``reference``'s, in degrees
    of the reference period (0-360)."""
    r_rise, _ = edges(times, reference)
    o_rise, _ = edges(times, other)
    p = period(times, reference)
    if not p or not r_rise or not o_rise:
        return None
    delays = []
    for r in r_rise:
        after = [o for o in o_rise if o >= r]
        if after and after[0] - r < p:
            delays.append(after[0] - r)
    if not delays:
        return None
    return 360.0 * statistics.median(delays) / p


def slow_pp(times, volts, window_s):
    """Peak-peak of the averages over consecutive windows (e.g. 1 ms):
    the slow part of a signal, such as 50 Hz hum, without switching noise."""
    if len(times) < 2:
        return None
    dt = times[1] - times[0]
    n = max(1, int(round(window_s / dt)))
    means = [statistics.fmean(volts[i:i + n])
             for i in range(0, len(volts) - n + 1, n)]
    return max(means) - min(means) if means else None


def compensation_pct(times, volts, early=(5e-6, 25e-6), late_frac=(0.6, 0.95)):
    """Compensation of a divider channel from a slow square wave: the level
    just after each edge relative to the settled level, as a % of the step.
    100 % = trimmed right; below = under-compensated (rounded corner, the
    trimmer needs more capacitance); above = over-compensated (spike that
    decays). ``early`` is a window after the edge (s), ``late_frac`` the
    settled part of the half period."""
    rising, falling = edges(times, volts)
    p = period(times, volts)
    if not p:
        return None
    base, top = levels(volts)
    step = top - base
    if step <= 0:
        return None
    half = p / 2.0

    def window(t0, a, b):
        sel = [v for t, v in zip(times, volts) if t0 + a <= t <= t0 + b]
        return statistics.fmean(sel) if sel else None

    ratios = []
    for e, up in [(r, True) for r in rising] + [(f, False) for f in falling]:
        first = window(e, *early)
        settled = window(e, late_frac[0] * half, late_frac[1] * half)
        if first is None or settled is None:
            continue
        start = base if up else top
        if abs(settled - start) > step / 2:
            ratios.append(100.0 * (first - start) / (settled - start))
    return statistics.median(ratios) if ratios else None
