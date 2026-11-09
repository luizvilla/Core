#!/usr/bin/env python3
## SPDX-License-Identifier: LGPL-2.1
## @file current_loop.py
## @brief Analytic design of the inverter d/q current loop (single_phase_inverter, FOLLOWING).
## @par What it does
##   Averaged discrete-time model of the loop exactly as the library computes it, vectorized over
##   a grid of PI gains: the library's own SOGI recursion (sogi.cpp, float32) on the grid voltage
##   (feedforward) and on the current, the park transform with the grid angle, the PI of
##   control_library (pid.cpp: standard form, integral recomputed on saturation, ±30 V), the
##   duty limits, a one-period update delay, and the averaged output filter + grid impedance
##   integrated in 10 sub-steps per control period. Sequence as the application runs it: relay
##   open with the feedforward running (PIs reset), relay closed with I_d,ref = 0, then a step.
##   Linear margins (crossover, phase, gain) are given for the chosen gains.
## @par Usage
##   python3 tests/l1/design/current_loop.py [--json out.json]
## @par Assumptions (docs/testing/inverter-current-loop-design.md)
##   Ideal PLL angle; averaged bridge; filter 500 µH, r = 0.1 Ω (no output chokes); delay one
##   control period (optionally compensated); duty 0.05–0.95 (shield limits) at 400 V, the driver
##   default 0.1–0.9 reported separately.
"""Design aid only, no verdict: the L1 tests judge the firmware on the switched model."""
import argparse, json, math
import numpy as np

TS = 1e-4
W0 = 2 * math.pi * 50
VPK = 325.2691193458119
L_F, R_F = 500e-6, 0.1
TD_LINEAR = 1.5 * TS
SUB = 10
T_CONNECT, T_STEP = 0.05, 0.25

GRIDS = {'stiff': (0.0, 0.0), 'IEC 60725 ref.': (0.4, 0.8e-3),
         'bulb cold (40 Ω)': (40.0, 0.0), 'bulb hot (500 Ω)': (500.0, 0.0)}
F32 = np.float32


class Sogi:
    """sogi.cpp in float32, vectorized."""
    def __init__(self, kr, n):
        self.kr = F32(kr); self.d0 = np.zeros(n, F32); self.d1 = np.zeros(n, F32); self.d2 = np.zeros(n, F32); self.n1 = np.zeros(n, F32)
        wt = F32(W0) * F32(TS); self.c = F32(1) - F32(0.5) * wt * wt; self.inv = F32(1) / wt

    def calc(self, x):
        n0 = self.kr * (x.astype(F32) - self.d0)
        d0 = F32(TS) * (n0 - self.c * self.n1) + F32(2) * self.c * self.d1 - self.d2
        a = d0; b = -self.c * self.inv * d0 + self.inv * self.d1
        self.n1 = n0; self.d2 = self.d1; self.d1 = d0; self.d0 = d0
        return a.astype(float), b.astype(float)


class Pi:
    """pid.cpp calculate() with Td = 0, vectorized; reset() zeroes the integral."""
    def __init__(self, kp, ti, lim=30.0):
        self.kp, self.ti, self.lim = kp, ti, lim; self.i = np.zeros_like(kp)

    def __call__(self, e, active):
        self.i = np.where(active, self.i + TS * e, 0.0)
        u = self.kp * (e + self.i / self.ti); out = np.clip(u, -self.lim, self.lim)
        self.i = np.where(active & (out != u), self.ti * (out / self.kp - e), self.i)
        return np.where(active, out, 0.0)


def simulate(kr, kp, ti, rg, lg, iref, vbus=400.0, duty=(0.05, 0.95), t_end=0.45, comp=0.0):
    """@return dict of arrays (time, id, iq measured, i true) over the run, and clipped fraction."""
    kp = np.atleast_1d(np.asarray(kp, float)); ti = np.atleast_1d(np.asarray(ti, float)); n = kp.size
    sv, si = Sogi(500.0, n), Sogi(kr, n)
    pid, piq = Pi(kp, ti), Pi(kp, ti)
    i = np.zeros(n); vcmd = np.zeros(n); h = TS / SUB; lt = L_F + lg
    steps = int(t_end / TS); rec_id = np.zeros((steps, n)); rec_iq = np.zeros((steps, n)); rec_i = np.zeros((steps, n)); clipped = np.zeros(n)
    for k in range(steps):
        t = k * TS; th = W0 * t; vg = VPK * math.sin(th)
        closed = t >= T_CONNECT
        vpcc = vg + rg * i if closed else np.full(n, vg)
        a, b = sv.calc(vpcc); ia, ib = si.calc(i)
        thp = th - math.pi / 2; c, s = math.cos(thp), math.sin(thp)   # v = Vd·cos(θ_lib): θ_lib = θ_grid - 90°
        vd, vq = a * c + b * s, -a * s + b * c
        idm, iqm = ia * c + ib * s, -ia * s + ib * c
        ref = iref if t >= T_STEP else 0.0
        act = np.full(n, closed)
        ud = vd + pid(ref - idm, act); uq = vq + piq(-iqm, act)
        # Output rotation; comp > 0 advances it by comp control periods (delay compensation).
        tho = thp + W0 * comp * TS; co, so = math.cos(tho), math.sin(tho)
        d = (ud * co - uq * so) / (2 * vbus) + 0.5
        dc = np.clip(d, *duty); clipped += closed & (dc != d)
        vnext = vbus * (2 * dc - 1)
        if closed:  # exact for each sub-step (grid held per sub-step): stable for any L/R
            rt = R_F + rg; e = math.exp(-h * rt / lt)
            for j in range(SUB):
                vgi = VPK * math.sin(W0 * (t + (j + 0.5) * h))
                i = i * e + (1 - e) * (vcmd - vgi) / rt
        vcmd = vnext
        rec_id[k], rec_iq[k], rec_i[k] = idm, iqm, i
    nclosed = max(1, int((t_end - T_CONNECT) / TS))
    return {'t': np.arange(steps) * TS, 'id': rec_id, 'iq': rec_iq, 'i': rec_i}, clipped / nclosed


def metrics(r, iref):
    """@brief Per candidate: rise 10–90 %, overshoot, 5 % settling of the measured I_d; peak |I_q|;
    largest true current before the step (connection transient)."""
    t = r['t']; m = t >= T_STEP; tt = t[m] - T_STEP; y = r['id'][m]
    finite = np.isfinite(y).all(axis=0) & (np.abs(y).max(axis=0) < 50)
    out = []
    for j in range(y.shape[1]):
        if not finite[j]: out.append(None); continue
        yj = y[:, j]
        def first(level):
            k = np.where(yj >= level * iref)[0]; return float(tt[k[0]]) if len(k) else math.inf
        bad = np.where(np.abs(yj - iref) > 0.05 * iref)[0]
        settle = float(tt[bad[-1]]) if len(bad) else 0.0
        if len(bad) and bad[-1] >= len(yj) - 1: settle = math.inf
        pre = (t >= T_CONNECT) & (t < T_STEP); late = (t >= T_STEP - 0.05) & (t < T_STEP)
        out.append({'rise_s': first(0.9) - first(0.1), 'overshoot_pct': max(0.0, 100 * (yj.max() - iref) / iref),
                    'settle_s': settle, 'iq_peak_A': float(np.abs(r['iq'][m, j]).max()),
                    'connect_peak_A': float(np.abs(r['i'][pre, j]).max()), 'final_A': float(yj[-100:].mean()),
                    'pre_step_error_A': float(max(np.abs(r['id'][late, j]).max(), np.abs(r['iq'][late, j]).max()))})
    return out


def margins(kr, kp, ti, rg, lg):
    """@brief Linear margins of C·G·F·e^(-s·Td) (feedforward interaction with a resistive grid ignored)."""
    w = np.logspace(0, 5, 100000); s = 1j * w
    l = kp * (1 + 1 / (ti * s)) / ((L_F + lg) * s + R_F + rg) * (kr / 2) / (s + kr / 2) * np.exp(-s * TD_LINEAR)
    mag = np.abs(l); ph = np.unwrap(np.angle(l))
    i = np.where(np.diff(np.sign(mag - 1)))[0]; j = np.where(np.diff(np.sign(ph + math.pi)))[0]
    wc = w[i[-1]] if len(i) else math.nan
    return wc / 2 / math.pi, (180 + math.degrees(ph[i[-1]])) if len(i) else math.nan, (-20 * math.log10(mag[j[0]])) if len(j) else math.inf


def tune(kr, comp):
    """@brief Grid search: fastest worst-case 5 % settling of a step (2.77 A on the stiff and IEC grids,
    0.1 A through the cold bulb) with overshoot ≤ 10 % and |I_d|, |I_q| ≤ 0.05 A over the 50 ms before
    the step. @return (kp, ti, worst settling) or None."""
    kps, tis = np.meshgrid(np.logspace(-2.5, 1, 36), np.logspace(-3.5, -0.5, 25))
    kp, ti = kps.ravel(), tis.ravel(); ok = np.ones(kp.size, bool); worst = np.zeros(kp.size)
    for g, iref in (('stiff', 2.77), ('IEC 60725 ref.', 2.77), ('bulb cold (40 Ω)', 0.1)):
        r, _ = simulate(kr, kp, ti, *GRIDS[g], iref, comp=comp)
        for j, m in enumerate(metrics(r, iref)):
            if m is None or m['overshoot_pct'] > 10 or m['pre_step_error_A'] > 0.05 or not math.isfinite(m['settle_s']): ok[j] = False
            else: worst[j] = max(worst[j], m['settle_s'])
    if not ok.any(): return None
    j = int(np.argmin(np.where(ok, worst, np.inf)))
    return float(kp[j]), float(ti[j]), float(worst[j])


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('--json'); args = ap.parse_args()
    # Candidates taken to L1 (tests/l1/design/variant.py builds the same ones). The automatic
    # search (tune) found no gains meeting every constraint at once: the cold bulb, where the PCC
    # voltage feedforward cancels the bulb resistance, is slow for any gains that suit the grid.
    cands = {'A: library today': (500.0, 0.001, 0.0003, 0.0),
             'C: Kr_i 2000, Kp 1, Ti 5 ms, comp. 0.5': (2000.0, 1.0, 0.005, 0.5),
             'D: Kr_i 2000, Kp 3, Ti 20 ms, comp. 0.5': (2000.0, 3.0, 0.02, 0.5)}
    report = {}
    print('\n### Tuning (400 V bus, duty 0.05–0.95)\n')
    print('| Candidate | Kr_i (rad/s) | Kp (V/A) | Ti (ms) | Grid | Crossover (Hz) | PM (°) | GM (dB) | Step | Rise (ms) | Overshoot (%) | Settle 5 % (ms) | Peak \\|I_q\\| (A) | Connection peak (A) | Error before step (A) | Final I_d (A) |')
    print('|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|')
    for name, (kr, kp, ti, comp) in cands.items():
        report[name] = {'kr': kr, 'kp': kp, 'ti': ti, 'comp_periods': comp, 'cases': {}}
        for g, (rg, lg) in GRIDS.items():
            wc, pm, gm = margins(kr, kp, ti, rg, lg)
            for iref in ((0.1, 2.77) if 'bulb' not in g else (0.1,)):
                t_end = 0.45 if 'bulb' not in g else 2.25
                r, clip = simulate(kr, kp, ti, rg, lg, iref, t_end=t_end, comp=comp); m = metrics(r, iref)[0]
                report[name]['cases'][f'{g} {iref}'] = dict(wc_hz=wc, pm_deg=pm, gm_db=gm, **(m or {'unstable': True}))
                fmt = lambda x: '∞' if not math.isfinite(x) else f'{x * 1e3:.3g}'
                row = f'| {name} | {kr:g} | {kp:.3g} | {ti * 1e3:.3g} | {g} | {wc:.3g} | {pm:.3g} | {gm:.3g} | 0→{iref} A | '
                print(row + (f"{fmt(m['rise_s'])} | {m['overshoot_pct']:.3g} | {fmt(m['settle_s'])} | {m['iq_peak_A']:.3g} | {m['connect_peak_A']:.3g} | {m['pre_step_error_A']:.3g} | {m['final_A']:.3g} |" if m else 'unstable | | | | | | |'))
    print('\n### Same gains with the driver default duty 0.1–0.9 at 400 V, I_d,ref 0\n')
    print('| Candidate | Grid | Duty clipped (% of periods) | Peak current with I_d,ref = 0 (A) |')
    print('|---|---|---|---|')
    for name, (kr, kp, ti, comp) in cands.items():
        for g in ('stiff', 'IEC 60725 ref.', 'bulb cold (40 Ω)'):
            r, clip = simulate(kr, kp, ti, *GRIDS[g], 0.0, vbus=400.0, duty=(0.1, 0.9), t_end=0.25, comp=comp)
            pk = float(np.abs(r['i'][r['t'] >= T_CONNECT + 0.05, 0]).max())
            report[name]['cases'][f'{g} 400V'] = {'clipped': float(clip[0]), 'peak_A': pk}
            print(f'| {name} | {g} | {100 * clip[0]:.3g} | {pk:.3g} |')
    print('\n### Sensitivity to the delay compensation (the real update delay is an assumption), IEC grid, 0→2.77 A\n')
    print('| Candidate | Compensation (periods) | Overshoot (%) | Settle 5 % (ms) | Error before step (A) |')
    print('|---|---|---|---|---|')
    for name, (kr, kp, ti, comp) in list(cands.items())[1:]:
        for cc in (0.0, 0.25, 0.5, 0.75, 1.0):
            r, _ = simulate(kr, kp, ti, *GRIDS['IEC 60725 ref.'], 2.77, comp=cc); m = metrics(r, 2.77)[0]
            report[name]['cases'][f'IEC comp {cc}'] = m or {'unstable': True}
            if m is None: print(f'| {name} | {cc} | unstable | | |'); continue
            st = '∞' if not math.isfinite(m['settle_s']) else f"{m['settle_s'] * 1e3:.3g}"
            print(f"| {name} | {cc} | {m['overshoot_pct']:.3g} | {st} | {m['pre_step_error_A']:.3g} |")
    if args.json: open(args.json, 'w').write(json.dumps(report, indent=1) + '\n')


if __name__ == '__main__':
    main()
