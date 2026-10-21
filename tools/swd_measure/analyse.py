"""Measurement helpers: frequencies from the counter valleys, and
waveform analysis of the logic capture."""

TIMERS = "ABCDEF"
# capture bit -> (unit, output)
BITS = {"A": (0, 1), "C": (2, 3), "D": (4, 5), "E": (6, 7)}


def frequencies(r, udm):
    """Frequency of each running timer from the counter wraps counted over
    window_cycles CPU cycles. In up-down mode the CNTxR readout rises twice
    per PWM period (referred to the PER event when down-counting, RM0440
    27.3.6), so the wrap count is divided by 2."""
    t = r["window_cycles"] / r["cpu_hz"]
    out = {}
    for i, n in enumerate(TIMERS):
        v = r[f"valleys{i}"]
        if v:
            k = 2.0 if udm.get(n) else 1.0
            out[n] = {"f_valleys": v / t / k, "f_peaks": r[f"peaks{i}"] / t / k,
                      "cnt_max": r[f"cnt_max{i}"]}
    return out


def _runs(bits):
    """List of (value, start, length) runs."""
    runs, start = [], 0
    for i in range(1, len(bits) + 1):
        if i == len(bits) or bits[i] != bits[start]:
            runs.append((bits[start], start, i - start))
            start = i
    return runs


def analyse_capture(buf, cycles, cpu_hz):
    ts = cycles / len(buf) / cpu_hz          # sample period, s
    res = {"sample_ns": ts * 1e9, "duration_us": len(buf) * ts * 1e6}
    for unit, (bh, bl) in BITS.items():
        hi = [(b >> bh) & 1 for b in buf]
        lo = [(b >> bl) & 1 for b in buf]
        rises = [i for i in range(1, len(hi)) if hi[i] and not hi[i - 1]]
        u = {"high_always": all(hi), "high_never": not any(hi),
             "low_never": not any(lo)}
        if len(rises) >= 3:
            n_per = len(rises) - 1
            span = rises[-1] - rises[0]
            period = span / n_per * ts
            u["freq_hz"] = 1 / period
            u["duty_high"] = sum(hi[rises[0]:rises[-1]]) / span
            u["duty_low"] = sum(lo[rises[0]:rises[-1]]) / span
            u["overlap_samples"] = sum(1 for i in range(len(hi)) if hi[i] and lo[i])
            # dead times: both low between a high and a low pulse
            dts = []
            for i in range(rises[0], rises[-1]):
                if hi[i - 1] and not hi[i]:          # high falls
                    j = i
                    while j < len(lo) and not lo[j] and not hi[j]:
                        j += 1
                    dts.append(j - i)
                if lo[i - 1] and not lo[i]:          # low falls
                    j = i
                    while j < len(hi) and not hi[j] and not lo[j]:
                        j += 1
                    dts.append(j - i)
            u["dead_time_ns"] = (min(dts) * ts * 1e9, max(dts) * ts * 1e9) if dts else None
            # centres of the high pulses (sample index)
            centres = []
            for v, s, n in _runs(hi):
                if v and s > 0 and s + n < len(hi):
                    centres.append(s + (n - 1) / 2)
            u["centres"] = centres
        res[unit] = u
    # phase between D and E: distance from each D centre to the nearest E centre
    if "centres" in res.get("D", {}) and "centres" in res.get("E", {}):
        ec = res["E"]["centres"]
        d = [min(abs(c - e) for e in ec) for c in res["D"]["centres"] if ec]
        res["DE_centre_offset_ns"] = (min(d) * ts * 1e9, max(d) * ts * 1e9) if d else None
    for unit in BITS:
        res[unit].pop("centres", None)
    return res
