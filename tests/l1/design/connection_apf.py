#!/usr/bin/env python3
## SPDX-License-Identifier: LGPL-2.1
## @file connection_apf.py
## @brief Averaged model of the step 2b connection with the ADR 0007 library (design test D-GFL-09, ADR 0004).
## @par What it models
##   Controller, every 50 µs, as library b9e8829 computes it (float32 SOGI, prewarped APF, park, pid.cpp, PLL PI
##   from init(): ωn 150 rad/s, ζ 0.7): VAC and IAC are the means of the previous period (the averaged ADC reads);
##   PLL on park(SOGI α, APF(SOGI α)); SOGI and APFs tuned to ω low-passed (10 ms) and clamped ±5 %; current
##   park(IAC, APF(IAC)), PI Kp 3.5 V/A, Ti 20 ms, ±30 V; output rotated by θ + 0.3·ω·Ts; duty clamped 0.05–0.95,
##   applied one period later. Step 2b from the relay command: PI on, PLL frozen 15 ms at the period-averaged ω,
##   feedforward from the source until the predicted contact (9.7 ms), held {Vd_avg, 0} to the end of the blanking.
##   Feedforward sources: 'apf' (VAC, APF(VAC)), 'sogi' (the PLL pair), 'pll' ({Vd_avg, 0} kept after the blanking,
##   option 1), 'apf_g07' (apf with gain 0.7 after the blanking).
##   Plant, averaged bridge (no ripple, no dead time): 500 µH + 0.1 Ω, C113 120 nF and C7 680 nF + R3 4.7 Ω at the
##   filter node; IAC = the line current through the relay; VAC between the relay and the bulb (the grid voltage
##   while the relay is open, the filter node once closed); the bulb a resistance R(T) (fixed, or a tungsten
##   filament heated by i²R and cooled by radiation); the grid 0.8 mH behind it with the harmonics measured on the
##   bench (CH2 with the relay open, 2026-11-13 18:32:41, fitted up to the 40th harmonic). The bridge fundamental is scaled and rotated to reproduce the step 1 mismatch
##   measured on the bench (−2.4 %, −1.4°).
## @par Usage
##   .venv/bin/python tests/l1/design/connection_apf.py --report docs/testing/design/D-GFL-09_bench
"""Design aid only (ADR 0004): INFO results that decide whether an idea is worth a switched run or a bench run."""
from __future__ import annotations

import argparse
import math
from collections import deque
from pathlib import Path

import numpy as np
from scipy.linalg import expm

F32 = np.float32
TS = 50e-6
SUB = 5
H = TS / SUB
F0 = 50.0
W0 = 2 * math.pi * F0
VPK = 325.2691193458119
VBUS = 400.0
L_F, R_F = 500e-6, 0.1
C113, C7, R3 = 120e-9, 680e-9, 4.7
LG = 0.1e-3
# Fitted to the bench (fit_grid(), 2026-11-13 18:33 dump): resistance behind the relay and the bridge's dead-time voltage.
R_FIT, VDT_FIT = 25.0, 6.0
# Grid harmonics as measured on CH2 before the contact (V peak, sine phase 0: phases unknown).
HARM = {3: 1.3, 5: 8.0, 7: 6.0, 9: 2.7}
RELAY_PREDICTED, RELAY_ACTUAL = 0.0097, 0.00835     # firmware's predicted contact; contact on the bench scope (2026-11-13, both runs)
BLANKING, CONNECT_WINDOW = 0.015, 0.020
KP_I, TI_I, LIM_I = 3.5, 0.020, 30.0
ADV = 0.3
TRIP_A, UV, OV = 3.0, 207.0, 253.0
OBS_DELAY, F_LO, F_HI = 0.1, 0.95 * F0, 1.05 * F0   # app grid_observer_config: 100 ms out of window before a trip
# Bulb: 80 W 230 V tungsten filament (the bench's). R(T) = R_COLD·(T/T0)^a with R(T_HOT) = 661 Ω; radiation k(T⁴ − T0⁴) = 80 W at T_HOT.
BULB_W = 80.0
T0, T_HOT, R_HOT = 300.0, 2800.0, 230.0 ** 2 / BULB_W


class Sogi:
    """sogi.cpp in float32."""
    def __init__(self, kr):
        self.kr = F32(kr); self.n1 = F32(0); self.d = [F32(0)] * 3

    def calc(self, x, w0):
        wt = F32(w0) * F32(TS); c = F32(1) - F32(0.5) * wt * wt
        n0 = self.kr * (F32(x) - self.d[0])
        d0 = F32(TS) * (n0 - c * self.n1) + F32(2) * c * self.d[1] - self.d[2]
        self.n1 = n0; self.d = [d0, d0, self.d[1]]
        return float(d0)


class Apf:
    """apf.cpp: first-order all-pass, prewarped Tustin, in float32."""
    def __init__(self):
        self.x1 = F32(0); self.y1 = F32(0)

    def calc(self, x, w0):
        q = F32(0.5) * F32(w0) * F32(TS); q2 = q * q
        k = q * (F32(1) + q2 * (F32(1 / 3) + q2 * F32(2 / 15)))
        a = (k - F32(1)) / (k + F32(1))
        y = a * F32(x) + self.x1 - a * self.y1
        self.x1 = F32(x); self.y1 = y
        return float(y)


class Pi:
    """pid.cpp with Td = 0: standard form, integral recomputed on saturation."""
    def __init__(self, kp, ti, lo, hi):
        self.kp, self.ti, self.lo, self.hi, self.i = kp, ti, lo, hi, 0.0

    def __call__(self, ref, meas):
        e = ref - meas; self.i += TS * e; u = self.kp * (e + self.i / self.ti)
        out = min(max(u, self.lo), self.hi)
        if out != u: self.i = self.ti * (out / self.kp - e)
        return out

    def reset(self): self.i = 0.0


class Bulb:
    """Tungsten filament: fixed resistance (thermal=False) or R(T) with i²R heating and radiative cooling."""
    def __init__(self, r_cold=R_FIT, thermal=True, c_th=0.015, t_start=None):
        self.thermal, self.c_th = thermal, c_th
        self.a = math.log(R_HOT / r_cold) / math.log(T_HOT / T0)
        self.r_cold = r_cold
        self.k = BULB_W / (T_HOT ** 4 - T0 ** 4)
        self.T = T0 if t_start is None else t_start

    @property
    def r(self):
        return self.r_cold * (self.T / T0) ** self.a if self.thermal else self.r_cold

    def heat(self, p, dt):
        if self.thermal:
            self.T = max(T0, self.T + dt * (p - self.k * (self.T ** 4 - T0 ** 4)) / self.c_th)


# Measured grid: CH2 with the relay open (mains), step 1 capture of 2026-11-13 18:32:41, 100 ms at 12.5 MS/s.
GRID_CAPTURE = 'tests/out/2026-11-13_ef4be85-dirty_174133_bench/l2_inv08/captures/183241_step1-measured/waveforms.npz'
GRID = {'kind': 'synthetic', 'f0': F0, 'table': None, 'phase0': 0.0, 'harm': {}}


def use_measured_grid(path=GRID_CAPTURE, n_harm=40, periods=4):
    """Fit the measured grid: frequency by least squares on the fundamental, then harmonics 1..n_harm (amplitude
    and phase) over whole periods; the model then plays this periodic waveform, its phase as captured."""
    repo = Path(__file__).resolve().parents[3]
    w = np.load(repo / path); t = w['CH2_t'] - w['CH2_t'][0]; v = w['CH2_v'].astype(float)
    best = None
    for f in np.arange(49.80, 50.20, 0.002):
        m = t < periods / f
        z = np.abs(np.mean(v[m] * np.exp(-2j * np.pi * f * t[m])))
        if best is None or z > best[0]: best = (z, f)
    f0 = float(best[1]); m = t < periods / f0; tt, vv = t[m], v[m]
    coef = {h: 2 * np.mean(vv * np.exp(-2j * np.pi * h * f0 * tt)) for h in range(1, n_harm + 1)}
    dc = float(np.mean(vv))
    ph = np.arange(2048) / 2048.0
    table = dc + sum((c * np.exp(2j * np.pi * h * ph)).real for h, c in coef.items())
    # Start the simulation where the fundamental rises through zero (sine phase 0), as the synthetic grid does.
    phase0 = (-np.angle(coef[1]) / (2 * np.pi) - 0.25) % 1.0
    GRID.update(kind='measured', f0=f0, table=table, phase0=phase0, dc=dc,
                harm={h: (float(abs(c)), float(np.degrees(np.angle(c / coef[1] ** h * abs(coef[1]) ** h)))) for h, c in coef.items()})
    return GRID


def use_synthetic_grid():
    GRID.update(kind='synthetic', f0=F0, table=None, phase0=0.0, harm={})


def mains(t):
    if GRID['table'] is not None:
        x = (GRID['f0'] * t + GRID['phase0']) % 1.0 * 2048.0
        i = int(x); fr = x - i; tb = GRID['table']
        return float(tb[i % 2048] * (1 - fr) + tb[(i + 1) % 2048] * fr)
    v = VPK * math.sin(W0 * t)
    for h, a in HARM.items():
        v += a * math.sin(h * W0 * t)
    return v


_cache = {}


def discrete(closed, r, lg=None):
    """Exact discretization over H of x = [iL, vn, vC7, ig] with inputs [vb, vg] (ig 0 while open)."""
    lg = LG if lg is None else lg
    key = (closed, round(r, 1), lg)
    if key in _cache:
        return _cache[key]
    A = np.zeros((4, 4)); B = np.zeros((4, 2))
    A[0, 0] = -R_F / L_F; A[0, 1] = -1 / L_F; B[0, 0] = 1 / L_F
    A[1, 0] = 1 / C113; A[1, 1] = -1 / (R3 * C113); A[1, 2] = 1 / (R3 * C113)
    A[2, 1] = 1 / (R3 * C7); A[2, 2] = -1 / (R3 * C7)
    if closed:
        A[1, 3] = -1 / C113
        A[3, 1] = 1 / lg; A[3, 3] = -r / lg; B[3, 1] = -1 / lg
    M = np.zeros((6, 6)); M[:4, :4] = A; M[:4, 4:] = B
    E = expm(M * H)
    if len(_cache) > 4000:
        _cache.clear()
    _cache[key] = (E[:4, :4], E[:4, 4:])
    return _cache[key]


def run(source='apf', bulb=None, t_settle=0.30, t_after=2.0, bridge_gain=0.976, bridge_phase_deg=-0.3, stop_on_trip=True, edge='rise',
        lg=None, contact=RELAY_ACTUAL, vdt=VDT_FIT, kp=KP_I, ti=TI_I, id_step=None, gains=None, vb_dc=0.0, iac_offset=0.0):
    """vdt (V): dead-time voltage of the bridge, opposing the inductor current (smoothed over ±50 mA): the bridge applies
    vb − vdt·tanh(iL / 0.05) — a fixed voltage loss, so an effective resistance that falls as the current grows."""
    """@return dict of traces at the control rate and events (contact, observer trip, current trip)."""
    bulb = bulb or Bulb()
    sv = Sogi(500.0); apf_v, apf_ff, apf_i = Apf(), Apf(), Apf()
    wn = 3.0 / (2 * math.pi / W0); kpp = 2 * wn * 0.7 / VPK; kip = wn * wn / VPK
    pll_pi = Pi(kpp, kpp / kip, -10 * W0, 10 * W0)
    pid, piq = Pi(kp, ti, -LIM_I, LIM_I), Pi(kp, ti, -LIM_I, LIM_I)
    # gains: [(s from the relay command, Kp, Ti)], applied live as the bench keys + - > < do (library setCurrentPiGains)
    schedule = sorted(gains or [])
    w_tune_k = TS / (TS + 0.010)
    # Relay command at the first instant after t_settle that puts the predicted contact on a rising zero crossing.
    # edge: the predicted contact on a rising or a falling zero crossing of the grid fundamental (bench: either).
    f_g = GRID['f0']; half = 0.5 if edge == 'fall' else 0.0
    t_pred = (math.ceil((t_settle + RELAY_PREDICTED) * f_g - half) + half) / f_g
    t_cmd = t_pred - RELAY_PREDICTED; t_contact = t_cmd + contact
    steps = int((t_cmd + t_after) / TS)
    x = np.zeros(4); theta = -math.pi / 2; w = W0; w_tune = W0; vq_prev = 0.0
    N = 400  # one 50 Hz period at 50 µs (the library's sync window and main.cpp's VAC RMS window)
    hist_vd = deque([VPK] * N); hist_w = deque([W0] * N); vac_hist = deque([0.0] * N)
    s_vd, s_w, s_v2 = VPK * N, W0 * N, 0.0
    freeze_w = None; hold_vd = None; pending = 0.0
    keys = ('t', 'ig', 'vac', 'vg', 'vn', 'id', 'iq', 'vd', 'vq', 'w', 'r', 'duty', 'vac_rms', 'ff_d', 'ff_q', 'pd', 'pq', 'theta', 'w_avg')
    rec = {k: np.zeros(steps) for k in keys}
    vac_meas, iac_meas = mains(0.0), 0.0
    events = {'t_cmd': t_cmd, 't_contact': t_contact, 'observer_trip': None, 'current_trip': None}
    out_since = None
    end = steps
    for k in range(steps):
        t = k * TS
        closed = t >= t_contact
        # --- controller (calculateDuty order) ---
        if freeze_w is not None and t < t_cmd + BLANKING:
            w = freeze_w
        else:
            w = W0 + pll_pi(0.0, -vq_prev)
        theta = (theta + w * TS) % (2 * math.pi)
        w_tune += (w - w_tune) * w_tune_k
        ws = min(max(w_tune, 0.95 * W0), 1.05 * W0)
        a = sv.calc(vac_meas, ws); b = apf_v.calc(a, ws)
        fa = vac_meas; fb = apf_ff.calc(vac_meas, ws)
        ia = iac_meas; ib = apf_i.calc(iac_meas, ws)
        c, s = math.cos(theta), math.sin(theta)
        vd, vq = a * c + b * s, -a * s + b * c
        if source == 'sogi':
            ffd, ffq = vd, vq
        else:
            ffd, ffq = fa * c + fb * s, -fa * s + fb * c
        idm, iqm = ia * c + ib * s, -ia * s + ib * c
        vq_prev = vq
        s_vd += vd - hist_vd.popleft(); hist_vd.append(vd); s_w += w - hist_w.popleft(); hist_w.append(w)
        vd_avg = s_vd / N
        if t >= t_cmd and freeze_w is None:
            freeze_w = s_w / N
        in_hold = t_cmd + RELAY_PREDICTED <= t < t_cmd + BLANKING
        after = t >= t_cmd + BLANKING
        gain = 1.0
        if in_hold:
            if hold_vd is None: hold_vd = vd_avg
            ffd, ffq = hold_vd, 0.0
        elif after and source == 'pll':
            ffd, ffq = vd_avg, 0.0
        elif after and source == 'apf_g07':
            gain = 0.7
        while schedule and t - t_cmd >= schedule[0][0]:
            _, kp_new, ti_new = schedule.pop(0)
            for pi_ in (pid, piq):   # bumpless, as the library since 0caa792: the integral term Kp·∫e/Ti carries over
                ui = pi_.kp * pi_.i / pi_.ti
                pi_.kp, pi_.ti = kp_new, ti_new
                pi_.i = ui * ti_new / kp_new
        if t >= t_cmd:
            # id_step = (s after the contact, A): the application's RUN reference (step 3) from that instant on.
            id_ref = id_step[1] if id_step and t >= t_contact + id_step[0] else 0.0
            pd_, pq_ = pid(id_ref, idm), piq(0.0, iqm)
        else:
            pid.reset(); piq.reset(); pd_, pq_ = 0.0, 0.0
        od, oq = gain * ffd + pd_, gain * ffq + pq_
        tho = theta + ADV * w * TS
        va = od * math.cos(tho) - oq * math.sin(tho)
        vbeta = od * math.sin(tho) + oq * math.cos(tho)
        ph = math.radians(bridge_phase_deg)
        vb_ac = bridge_gain * (va * math.cos(ph) - vbeta * math.sin(ph))
        duty = min(max(vb_ac / (2 * VBUS) + 0.5, 0.05), 0.95)
        # --- plant over one period; the duty computed now applies from the next period ---
        vb = VBUS * (2 * pending - 1) if k > 0 else 0.0
        pending = duty
        Ad, Bd = discrete(closed, bulb.r, lg)
        if closed and k == int(round(t_contact / TS)):
            x[3] = 0.0
        sv_acc = si_acc = 0.0
        for j in range(SUB):
            tj = t + (j + 0.5) * H
            vg = mains(tj)
            x = Ad @ x + Bd @ np.array([vb + vb_dc - vdt * math.tanh(x[0] / 0.05) if vdt else vb + vb_dc, vg])
            if not closed: x[3] = 0.0
            sv_acc += x[1] if closed else vg
            si_acc += x[3]
        vac_meas, iac_meas = sv_acc / SUB, si_acc / SUB + (iac_offset if closed else 0.0)
        bulb.heat(iac_meas ** 2 * bulb.r, TS)
        s_v2 += vac_meas * vac_meas - vac_hist.popleft() ** 2; vac_hist.append(vac_meas)
        vac_rms = math.sqrt(max(s_v2, 0.0) / N)
        for key, val in (('t', t), ('ig', iac_meas), ('vac', vac_meas), ('vg', mains(t + TS / 2)), ('vn', x[1]), ('id', idm), ('iq', iqm),
                         ('vd', vd), ('vq', vq), ('w', w), ('r', bulb.r), ('duty', duty), ('vac_rms', vac_rms), ('ff_d', ffd), ('ff_q', ffq),
                         ('pd', pd_), ('pq', pq_), ('theta', theta), ('w_avg', s_w / N)):
            rec[key][k] = val
        if closed and events['current_trip'] is None and abs(iac_meas) > TRIP_A:
            events['current_trip'] = t
        f_avg = s_w / N / (2 * math.pi)
        bad = not (UV <= vac_rms <= OV) or not (F_LO <= f_avg <= F_HI)
        out_since = (out_since if out_since is not None else t) if (closed and bad) else None
        if events['observer_trip'] is None and out_since is not None and t - out_since >= OBS_DELAY:
            events['observer_trip'] = t
        if stop_on_trip and (events['observer_trip'] or events['current_trip']) and t > min(x_ for x_ in (events['observer_trip'], events['current_trip']) if x_) + 0.02:
            end = k + 1
            break
    rec = {kk: v[:end] for kk, v in rec.items()}
    rec.update(events)
    return rec


def metrics(r):
    t, ig, vg = r['t'], r['ig'], r['vg']; tc = r['t_contact']
    def win(a, b): return (t >= tc + a) & (t < tc + b)
    out = {'peak20': float(np.abs(ig[win(0, 0.02)]).max()) if win(0, 0.02).any() else math.nan}
    m = win(0.012, 0.052)
    if m.sum() > 10:
        e = np.exp(-2j * np.pi * F0 * t[m]); I1 = 2 * np.mean(ig[m] * e); V1 = 2 * np.mean(vg[m] * e)
        out.update(rms_12_52=float(np.sqrt(np.mean(ig[m] ** 2))), i50=float(abs(I1)), phase=float(np.degrees(np.angle(I1 / V1))),
                   h={h: float(abs(2 * np.mean(ig[m] * np.exp(-2j * np.pi * h * F0 * t[m])))) for h in (3, 5, 7)})
    late = win(0.5, 2.0)
    if late.any():
        out['idq_late'] = float(np.hypot(r['id'][late], r['iq'][late]).max())
        out['rms_late'] = float(np.sqrt(np.mean(ig[late] ** 2)))
    after = win(0.02, 9)
    out['vac_min'] = float(r['vac_rms'][after].min()) if after.any() else math.nan
    out['r_max'] = float(r['r'].max())
    out['w_dev'] = float(np.abs(r['w'][after] - W0).max() / (2 * math.pi)) if after.any() else math.nan
    out['obs_trip'] = None if r['observer_trip'] is None else r['observer_trip'] - tc
    out['cur_trip'] = None if r['current_trip'] is None else r['current_trip'] - tc
    out['duration'] = float(t[-1] - tc)
    return out


def fmt_trip(m):
    if m['obs_trip'] is not None: return f'grid observer {m["obs_trip"]:.2f} s'
    if m['cur_trip'] is not None: return f'current {m["cur_trip"]:.3f} s'
    return 'none'


def row(label, m):
    h = m.get('h', {})
    return (f'| {label} | {m["peak20"]:.2f} | {m.get("rms_12_52", math.nan):.2f} | {m.get("i50", math.nan):.2f} at {m.get("phase", math.nan):+.0f}° | '
            f'{h.get(5, math.nan):.2f} / {h.get(7, math.nan):.2f} | {m.get("rms_late", math.nan):.2f} | {m.get("idq_late", math.nan):.2f} | '
            f'{m["vac_min"]:.0f} | {m["r_max"]:.0f} | {m["w_dev"]:.2f} | {fmt_trip(m)} |')


HEADER = ('| Case | Peak in 20 ms (A) | RMS 12–52 ms (A) | 50 Hz (A pk, vs grid) | 5th / 7th (A pk) | RMS 0.5–2 s (A) | '
          '|Idq| max 0.5–2 s (A) | VAC RMS min (V) | Bulb R max (Ω) | PLL max dev (Hz) | Trip after the contact |\n'
          '|---|---|---|---|---|---|---|---|---|---|---|')


def figure(runs, path, title):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    colors = ['#2a78d6', '#eb6834', '#1baf7a', '#eda100']
    fig, ax = plt.subplots(4, 1, figsize=(11, 10), sharex=True)
    for (label, r), col in zip(runs, colors):
        tt = r['t'] - r['t_contact']
        ax[0].plot(tt, r['ig'], color=col, lw=0.8, label=label)
        ax[1].plot(tt, np.hypot(r['id'], r['iq']), color=col, lw=1.0)
        ax[2].plot(tt, r['vac_rms'], color=col, lw=1.0)
        ax[3].plot(tt, r['r'], color=col, lw=1.0)
    ax[0].set_ylabel('line current (A)'); ax[1].set_ylabel('|Idq| (A)'); ax[2].set_ylabel('VAC RMS (V)'); ax[3].set_ylabel('bulb R (Ω)')
    ax[2].axhline(UV, color='#8a8984', ls='--', lw=0.8); ax[3].set_xlabel('time after the contact (s)')
    ax[0].legend(loc='upper right', fontsize=8); ax[0].set_title(title, fontsize=10)
    for a in ax: a.grid(color='#e4e2dc', lw=0.6); a.set_xlim(-0.05, 2.0)
    fig.tight_layout(); fig.savefig(path, dpi=110); plt.close(fig)


# ---- fit of the grid and the bridge losses to the bench -------------------------------------------------------
def fit_grid():
    """@brief Score candidate (grid resistance, bridge loss) pairs against the 18:33 ff 0 firmware dump, 8.4–115 ms after
    the relay command: normalized RMS error of IAC plus that of the period-averaged PLL frequency. @return table lines."""
    b = BENCH['ff0']; td, d = _dump(b['dump']); s = (td >= 8.4) & (td < 115); bi, bw = d['IAC_A'][s], d['omega_avg_rad_s'][s]
    def score(r_grid, vdt=0.0, rf=0.1):
        global R_F
        R_F = rf; _cache.clear()
        r = run('apf', Bulb(thermal=False, r_cold=r_grid), edge='rise', t_after=0.12, stop_on_trip=False, vdt=vdt)
        tm = (r['t'] - r['t_cmd']) * 1e3; mi = np.interp(td[s], tm, r['ig']); mw = np.interp(td[s], tm, r['w_avg'])
        ei = math.sqrt(np.mean((mi - bi) ** 2) / np.mean(bi ** 2)); ew = math.sqrt(np.mean((mw - bw) ** 2)) / np.std(bw)
        return ei + ew, ei, ew, math.sqrt(np.mean(mi ** 2)), float(np.corrcoef(mi, bi)[0, 1]), float(mw.min()), float(mw.max())
    rows = []
    for r_grid in (15, 20, 25, 30, 42):
        rows.append(('no loss', r_grid, *score(r_grid)))
    for rf in (2.0, 5.0, 8.0):
        for r_grid in (15, 22, 30):
            rows.append((f'series {rf:g} Ω', r_grid, *score(r_grid, rf=rf)))
    for vdt in (2.0, 4.0, 6.0, 8.0):
        for r_grid in (15, 20, 25, 30, 40):
            rows.append((f'dead time {vdt:g} V', r_grid, *score(r_grid, vdt=vdt)))
    globals()['R_F'] = 0.1; _cache.clear()
    rows.sort(key=lambda x: x[2])
    L = [f'Bench: IAC {math.sqrt(np.mean(bi ** 2)):.3f} A RMS, ω_avg {bw.min():.1f}–{bw.max():.1f} rad/s. Score = normalized RMS error of IAC + '
         'that of ω_avg (0 = identical). Best 12 of 34, then the worst:', '',
         '| Bridge loss | Grid R (Ω) | Score | IAC error | ω error | IAC RMS (A) | Correlation | ω_avg (rad/s) |', '|---|---|---|---|---|---|---|---|']
    for row in rows[:12] + rows[-2:]:
        loss, rg, sc, ei, ew, rms, c, wmin, wmax = row
        L.append(f'| {loss} | {rg:g} | {sc:.2f} | {ei:.2f} | {ew:.2f} | {rms:.3f} | {c:+.2f} | {wmin:.1f}–{wmax:.1f} |')
    return L


# ---- overlay with the bench (interactive figures, tests/common/interactive_plot.py) -----------------------------
REPO = Path(__file__).resolve().parents[3]
BENCH = {
    'ff0': dict(label='18:33, ff 0 (APF)', source='apf', edge='rise',
                scope='tests/out/2026-11-13_ef4be85-dirty_174133_bench/l2_inv08/captures/183336_step1-measured',
                dump='tests/out/2026-11-13_ef4be85-dirty_174133_bench/firmware_scope/183415_firmware_scope.txt',
                console='tests/out/2026-11-13_ef4be85-dirty_174133_bench/console.txt', key_time='2026-11-13T18:33:35.381'),
    'ff1': dict(label='17:24, ff 1 (SOGI)', source='sogi', edge='fall',
                scope='tests/out/2026-11-13_ef4be85-dirty_171916_bench/l2_inv08/captures/172455_step1-measured',
                dump=None, console='tests/out/2026-11-13_ef4be85-dirty_171916_bench/console.txt', key_time='2026-11-13T17:24:54.513'),
}
SETTLE_S = 0.21   # G to the relay command: 10 grid periods of PWM_SETTLE, then the zero-crossing latch (bench console)
C_BENCH, C_DUMP, C_MODEL = '#52514e', '#eda100', '#2a78d6'


def _scope(path, decim=125):
    """Scope traces as block means over decim samples (10 µs), time in ms from the trigger (the relay command)."""
    w = np.load(REPO / path / 'waveforms.npz')
    out = {}
    for ch in ('CH1', 'CH2', 'CH3'):
        t, v = w[f'{ch}_t'], w[f'{ch}_v']; n = len(v) // decim * decim
        out[ch] = (t[:n].reshape(-1, decim).mean(1) * 1e3, v[:n].reshape(-1, decim).mean(1))
        if ch == 'CH3': out['clip'] = int(np.sum(np.abs(v) > 10.2))
    return out


def _dump(path):
    import sys
    sys.path.insert(0, str(REPO / 'tests' / 'l2'))
    from console import parse_dump
    names, rows = parse_dump((REPO / path).read_text().splitlines())
    a = np.asarray(rows, float); t = (np.arange(len(a)) - 26) / 2000.0 * 1e3   # dump header: trigger at sample 26, 2000 Hz
    return t, {n: a[:, i] for i, n in enumerate(names)}


def _telemetry(path, key_time):
    import datetime as dt, re
    t_key = dt.datetime.fromisoformat(key_time).timestamp()
    rx = re.compile(r'st (\d+) inv (\d+) rly (\d\d) VAC\s*(-?[\d.]+) IAC\s*(-?[\d.]+) Vbus\s*(-?[\d.]+) Id\s*(-?[\d.]+) Iq\s*(-?[\d.]+)')
    rows = []
    for line in (REPO / path).read_text(errors='replace').splitlines():
        parts = line.split(' ', 2)
        if len(parts) < 3 or parts[1] != '<': continue
        m = rx.search(parts[2])
        if not m: continue
        t = dt.datetime.fromisoformat(parts[0]).timestamp() - t_key - SETTLE_S
        if -1.0 <= t <= 4.0: rows.append((t, float(m.group(4)), float(m.group(7)), float(m.group(8))))
    return np.asarray(rows) if rows else np.zeros((0, 4))


def overlay(out: Path, bulb_fn):
    """@brief Write one interactive figure per bench run, model against bench on one time axis. @return Markdown lines."""
    import sys
    sys.path.insert(0, str(REPO / 'tests' / 'common'))
    from interactive_plot import Figure
    lines = []
    for name, b in BENCH.items():
        r = run(b['source'], bulb_fn(), edge=b['edge'])
        tm = (r['t'] - r['t_cmd']) * 1e3
        sc = _scope(b['scope'])
        win = lambda t: (t >= -35) & (t <= 115)   # noqa: E731
        fig = Figure('ms', 'time from the relay command (ms)')
        c = fig.chart('A, line current (firmware sign)', 230)
        t3, i3 = sc['CH3']; m = win(t3)
        fig.line(c, 'bench: scope CH3 × −1 (10 µs means)', t3[m], -i3[m], C_BENCH, 1.0)
        if b['dump']:
            td, d = _dump(b['dump']); md = win(td)
            fig.line(c, 'bench: firmware IAC (2 kHz)', td[md], d['IAC_A'][md], C_DUMP, 1.6)
        mm = win(tm); fig.line(c, 'model: line current', tm[mm], r['ig'][mm], C_MODEL, 1.4)
        c['events'] = [{'t': (r['t_contact'] - r['t_cmd']) * 1e3, 'label': 'model contact'}, {'t': BLANKING * 1e3, 'label': 'end of blanking'}]
        c = fig.chart('V, grid side (VAC)', 170)
        t2, v2 = sc['CH2']; m = win(t2)
        fig.line(c, 'bench: scope CH2', t2[m], v2[m], C_BENCH, 1.0)
        if b['dump']: fig.line(c, 'bench: firmware VAC', td[md], d['VAC_V'][md], C_DUMP, 1.4)
        fig.line(c, 'model: VAC', tm[mm], r['vac'][mm], C_MODEL, 1.2)
        c = fig.chart('V, inverter side', 170)
        t1, v1 = sc['CH1']; m = win(t1)
        fig.line(c, 'bench: scope CH1', t1[m], v1[m], C_BENCH, 1.0)
        fig.line(c, 'model: filter node', tm[mm], r['vn'][mm], C_MODEL, 1.2)
        if b['dump']:
            c = fig.chart('rad/s, PLL frequency', 150)
            fig.line(c, 'bench: ω period average', td[md], d['omega_avg_rad_s'][md], C_DUMP, 1.4)
            fig.line(c, 'model: ω period average', tm[mm], r['w_avg'][mm], C_MODEL, 1.2)
            c = fig.chart('V, PLL Vq', 150)
            fig.line(c, 'bench: Vq', td[md], d['V_q_V'][md], C_DUMP, 1.2)
            fig.line(c, 'model: Vq', tm[mm], r['vq'][mm], C_MODEL, 1.2)
            c = fig.chart('bridge duty', 150)
            fig.line(c, 'bench: duty', td[md], d['bridge_duty'][md], C_DUMP, 1.2)
            fig.line(c, 'model: duty', tm[mm], r['duty'][mm], C_MODEL, 1.2)
        (out / f'iplot_{name}.html').write_text(fig.html(f'dgfl09-{name}', f'D-GFL-09 model against the bench, {b["label"]}'), encoding='utf-8')
        # seconds after: telemetry (two lines every 500 ms) against the model
        tel = _telemetry(b['console'], b['key_time'])
        fig2 = Figure('s', 'time from the relay command (s), bench aligned on the G key + 0.21 s')
        ts = r['t'] - r['t_cmd']; k = ts >= -0.5
        dec = max(1, int(round(0.005 / TS)))
        c = fig2.chart('A, Id and Iq', 200)
        if len(tel):
            fig2.line(c, 'bench: telemetry Id', tel[:, 0], tel[:, 2], C_DUMP, 1.4, points=True)
            fig2.line(c, 'bench: telemetry Iq', tel[:, 0], tel[:, 3], C_BENCH, 1.4, points=True)
        fig2.line(c, 'model: Id', ts[k][::dec], r['id'][k][::dec], C_MODEL, 1.0)
        fig2.line(c, 'model: Iq', ts[k][::dec], r['iq'][k][::dec], '#1baf7a', 1.0)
        c = fig2.chart('V, VAC RMS', 160)
        if len(tel): fig2.line(c, 'bench: telemetry VAC', tel[:, 0], tel[:, 1], C_DUMP, 1.4, points=True)
        fig2.line(c, 'model: VAC RMS (one period)', ts[k][::dec], r['vac_rms'][k][::dec], C_MODEL, 1.2)
        c['hlines'] = [{'v': UV, 'dash': [6, 4], 'color': '#8a8984'}]
        (out / f'iplot_{name}_long.html').write_text(fig2.html(f'dgfl09-{name}-long', f'D-GFL-09 seconds after the contact, {b["label"]}'), encoding='utf-8')
        mt = metrics(r)
        lines += [f'### {b["label"]}: model with the {b["source"].upper()} feedforward, edge {b["edge"]}', '',
                  f'<!-- iplot {name} -->', '',
                  f'First 115 ms from the relay command. Grey: scope (CH3 multiplied by −1 to the firmware\'s sign; 10 µs means); '
                  f'yellow: the firmware\'s own record (2 kHz dump){"" if b["dump"] else " (none for this run)"}; blue: the model. '
                  + (f'**CH3 overranged on {sc["clip"]:,} samples** (±10.24 A at 2 A/div): its peaks are cut and its values near them are not reliable. ' if sc['clip'] else '')
                  + f'Model trip: {fmt_trip(mt)}.', '',
                  f'<!-- iplot {name}_long -->', '',
                  'Seconds after: the bench telemetry (two lines every 500 ms; aligned on the G key + 0.21 s, so within about ±0.3 s) '
                  'against the model (sampled every 5 ms). The model stops 20 ms after its own trip.', '']
    return lines


def report(out: Path):
    out.mkdir(parents=True, exist_ok=True)
    g = use_measured_grid()
    harm_txt = ', '.join(f'{h}th {a:.1f} V at {p:+.0f}°' for h, (a, p) in g['harm'].items() if h > 1 and a > 1.5)
    thd = 100 * math.sqrt(sum(a * a for h, (a, _) in g['harm'].items() if h > 1)) / g['harm'][1][0]
    cold, thermal = (lambda: Bulb(thermal=False)), (lambda: Bulb())
    fit_lines = fit_grid()
    bench = [(f'APF (ff 0), {R_FIT:g} Ω', run('apf', cold())), (f'SOGI (ff 1), {R_FIT:g} Ω', run('sogi', cold())),
             ('APF (ff 0), thermal bulb', run('apf', thermal())), ('SOGI (ff 1), thermal bulb', run('sogi', thermal()))]
    options = [('APF (ff 0)', bench[0][1]), ('SOGI (ff 1)', bench[1][1]),
               ('option 1: {Vd_avg, 0} after the blanking', run('pll', cold())), ('APF, gain 0.7 after the blanking', run('apf_g07', cold()))]
    hot = [(f'{lab}, bulb hot {R_HOT:.0f} Ω', run(src, Bulb(thermal=False, r_cold=R_HOT))) for src, lab in (('apf', 'APF'), ('sogi', 'SOGI'), ('pll', 'option 1'))]
    sens = [('APF, filament 0.005 J/K', run('apf', Bulb(c_th=0.005))), ('APF, ideal bridge', run('apf', cold(), bridge_gain=1.0, bridge_phase_deg=0.0)),
            ('SOGI, ideal bridge', run('sogi', cold(), bridge_gain=1.0, bridge_phase_deg=0.0)),
            ('APF, no dead time, bulb 42 Ω (before the fit)', run('apf', Bulb(thermal=False, r_cold=42.0), vdt=0.0)),
            ('SOGI, no dead time, bulb 42 Ω (before the fit)', run('sogi', Bulb(thermal=False, r_cold=42.0), vdt=0.0))]
    use_synthetic_grid()
    sens += [('APF, synthetic grid (harmonic phases 0)', run('apf', cold())), ('SOGI, synthetic grid', run('sogi', cold()))]
    use_measured_grid()
    figure(bench, out / 'bench_check.png', f'Bench check: measured grid, fitted {R_FIT:g} Ω and {VDT_FIT:g} V dead time, fixed and thermal bulb')
    figure(options, out / 'options.png', f'Feedforward options: measured grid, fitted {R_FIT:g} Ω and {VDT_FIT:g} V dead time')
    L = ['# D-GFL-09: step 2b with each feedforward source, averaged model, measured grid, 80 W bulb', '',
         'Design test ([ADR 0004](../../../adr/0004-averaged-model-for-fast-evaluation.md)), run at Luiz Villa\'s request on 2026-11-13: '
         'averaged model, **INFO only, no verdict**. It decides which feedforward option is worth a bench or switched run. Generated by '
         '`tests/l1/design/connection_apf.py --report docs/testing/design/D-GFL-09_bench`.', '',
         '## Model', '',
         '- **Controller** as library `b9e8829` computes it every 50 µs: float32 SOGI (Kr 500), prewarped APF, park, PLL PI (ωn 150 rad/s, '
         'ζ 0.7), SOGI/APF tuning low-passed 10 ms and clamped ±5 %, current park(IAC, APF(IAC)) with PI 3.5 V/A, 20 ms, ±30 V, output '
         'advance 0.3, duty 0.05–0.95 applied one period later; VAC and IAC the means of the period (averaged ADC reads).',
         '- **Step 2b**: PI from the relay command, PLL frozen 15 ms at the period-averaged ω, feedforward held {Vd_avg, 0} from the '
         'predicted contact (9.7 ms) to 15 ms; the contact itself at 8.7 ms (bench 8.3–9 ms).',
         '- **Feedforward sources**: APF (VAC, APF(VAC)), the firmware default; SOGI (the PLL pair, key F); option 1 = {Vd_avg, 0} kept after '
         'the blanking (Vd along the PLL angle); APF with gain 0.7 after the blanking.',
         '- **Plant**, averaged bridge (no 50 kHz ripple): 500 µH + 0.1 Ω, C113 120 nF, C7 680 nF + 4.7 Ω; VAC between the relay and the '
         f'bulb; 0.1 mH of grid behind it. The bridge fundamental scaled ×0.976 and rotated −0.3° so the step 1 mismatch matches the bench '
         f'(−2.4 %, −1.4°). **Fitted to the bench (section 0):** {R_FIT:g} Ω behind the relay and a dead-time voltage of {VDT_FIT:g} V '
         'opposing the inductor current (about 150 ns at 400 V and 50 kHz); contact 8.35 ms after the command, as on the scope.',
         f'- **Grid: measured** on CH2 with the relay open (step 1 capture 18:32:41, 100 ms at 12.5 MS/s), fitted over 4 periods: '
         f'{g["f0"]:.3f} Hz, fundamental {g["harm"][1][0]:.1f} V pk, THD {thd:.1f} % ({harm_txt}; phases against the fundamental).',
         f'- **Bulb** (80 W): {R_FIT:g} Ω (fitted); or a tungsten filament from {R_FIT:g} Ω cold to {R_HOT:.0f} Ω hot, heated by i²R and '
         'cooled by radiation (heat capacity 0.015 J/K, an estimate).',
         '- **Grid observer** reduced to its windows (207–253 V on the VAC RMS of one period, ±5 % on the period-averaged PLL frequency) '
         'with the application\'s 100 ms delay; the 3 A current trip applied as in the firmware (on the period-mean IAC).', '',
         '## 0. Fit to the bench', '',
         'The resistance behind the relay and the losses of the bridge were not measured, so they are fitted to the 18:33 ff 0 run, '
         'whose firmware dump gives IAC and the PLL frequency sample by sample. A grid impedance alone cannot match both: a low '
         'resistance reproduces the PLL swing but leaves the current about 1.7× too high, and the grid inductance hardly matters below '
         '1 mH. A loss in the bridge is needed. A dead-time voltage (a fixed voltage opposing the current, so an effective resistance '
         'that falls as the current grows) fits about as well as a fixed series resistance and reproduces the late wander better '
         '(|Idq| 0.31 A against 0.68 A; bench 0.3–0.5 A), so it is kept. The 17:24 ff 1 run is not used in the fit: it is the check.', '']
    L += fit_lines
    L += ['', '## 1. Check against the bench', '',
         'Bench, 2026-11-13: **ff 0** (16:39 and 18:33, identical): 12–52 ms after the contact 0.72 A RMS (0.42 A of it at 50 kHz, absent '
         'here), 50 Hz 0.71 A pk at −124° against the grid in the firmware\'s sign (+56° on the reversed probe), then |Idq| wandering up to '
         '0.3–0.5 A, no trip in 30 s, bulb dark. **ff 1** (17:24): bulb lit, Iq 1.7 A pk and VAC 202 V about 1 s after the contact, grid '
         'observer trip.', '', HEADER]
    L += [row(lab, metrics(r)) for lab, r in bench]
    L += ['', '![Bench check](bench_check.png)', '', '### Model and bench on one time axis', '',
          'Interactive: drag to zoom in time, double-click to reset, click a legend entry to hide a trace. The model runs with the fitted '
          'grid and dead time and the zero-crossing edge of each bench run.', '']
    L += overlay(out, cold)
    L += ['',
          '**Reading.** With the fitted grid and dead time, the model reproduces the ff 0 run: the current within about 20 % and correlated '
          '0.9 with the scope, the PLL swing, no trip, the same late wander. For ff 1 (not fitted) it reproduces the waveform shape '
          '(correlation 0.95 with the scope) and 2–3× more current than ff 0, but **not the trip about 1 s after the contact**: the model '
          'stays connected and the bulb barely warms. The 17:24 scope CH3 is 6–8× the model with the same shape, consistent with that '
          'channel being unreliable in this run (overranged; the firmware read Iq 1.2–1.7 A, the model up to about 1 A).', '',
          f'## 2. Options (fitted grid, {R_FIT:g} Ω)', '', HEADER]
    L += [row(lab, metrics(r)) for lab, r in options]
    L += ['', '![Options](options.png)', '',
          '**Reading.** See the table: the late current and the trips per option. A gain of 0.7 cannot work with a ±30 V PI: it leaves about '
          '100 V of fundamental for the PI to supply. Since the model does not reproduce the SOGI trip, a option that survives here is not '
          'proven to survive on the bench; one that fails here is unlikely to do better there.', '',
          '## 3. Weak grid (bulb hot) and sensitivity', '', HEADER]
    L += [row(lab, metrics(r)) for lab, r in hot + sens]
    L += ['', '## Limits', '',
          '- Averaged bridge: no 50 kHz ripple, no dead time, no ADC aliasing. The bench\'s 0.42 A at 50 kHz is not here.',
          '- The closing transient is about twice the bench\'s; the bulb\'s thermal parameters are estimates.',
          '- No SYNC_LOST and no observer qualification beyond its windows and delay.',
          '- A result here never passes or fails the firmware (ADR 0004).']
    (out / 'report.md').write_text('\n'.join(L) + '\n', encoding='utf-8')
    return out / 'report.md'


# ---- bench gain changes (keys + - > <) replayed in the model ------------------------------------------------------
KP_STEP, KP_MIN, KP_MAX, TI_STEP, TI_MIN, TI_MAX = 0.1, 0.5, 10.0, 0.001, 0.001, 0.100   # app::current_kp_step… (control_step.h)


def step_gain(v, up, step, lo, hi):
    """app::stepGain: one step, snapped to the step grid, clamped."""
    return min(max(math.floor(v / step + (1.5 if up else -0.5)) * step, lo), hi)


def bench_gain_schedule(console, t_ref_iso):
    """@brief Current PI gains from a bench console: the + - > < presses replayed from the last boot (defaults then),
    as the firmware applies them. @return [(s from t_ref, Kp, Ti)] with the gains in force at t_ref first."""
    import datetime as dt
    t_ref = dt.datetime.fromisoformat(t_ref_iso).timestamp()
    kp, ti, out = KP_I, TI_I, []
    for line in (REPO / console).read_text(errors='replace').splitlines():
        parts = line.split(' ', 2)
        if len(parts) < 3: continue
        try: t = dt.datetime.fromisoformat(parts[0]).timestamp()
        except ValueError: continue
        if parts[1] == '<' and 'Booting Zephyr' in parts[2]:
            kp, ti = KP_I, TI_I; out = [(t - t_ref, kp, ti)]
        elif parts[1] == '>' and parts[2] in ('+', '-', '>', '<'):
            if parts[2] in '+-': kp = step_gain(kp, parts[2] == '+', KP_STEP, KP_MIN, KP_MAX)
            else: ti = step_gain(ti, parts[2] == '>', TI_STEP, TI_MIN, TI_MAX)
            out.append((t - t_ref, kp, ti))
    before = [g for g in out if g[0] <= 0]
    return ([(-1e9, *before[-1][1:])] if before else [(-1e9, KP_I, TI_I)]) + [g for g in out if g[0] > 0]


# ---- D-GFL-10: current PI gain sweep -------------------------------------------------------------------------------
STEP_AT, STEP_A = 0.6, 0.1   # Id reference step 0.6 s after the contact, 0.1 A (the bench's step 3 size)


def step_metrics(r):
    """Connection and step response of one run (time from the contact)."""
    t = r['t'] - r['t_contact']; idm, iqm = r['id'], r['iq']
    out = metrics(r)
    pre = (t >= STEP_AT - 0.2) & (t < STEP_AT)
    if not pre.any() or t[-1] < STEP_AT + 0.6:
        return out
    base = float(np.mean(idm[pre])); after = t >= STEP_AT
    # one-period moving average removes the 2ω ripple of the instantaneous dq current
    n = int(round(0.02 / TS)); k = np.ones(n) / n
    ida = np.convolve(idm, k, 'same'); iqa = np.convolve(iqm, k, 'same')
    y = (ida[after] - base) / STEP_A; ta = t[after] - STEP_AT
    def first(cond):
        i = np.where(cond)[0]; return float(ta[i[0]]) if len(i) else math.nan
    out['rise'] = first(y >= 0.9) - first(y >= 0.1)
    out['overshoot'] = float(max(0.0, (y[ta < 0.6].max() - 1.0) * 100))
    band = np.abs(y - 1.0) > 0.05; i = np.where(band & (ta < 0.6))[0]
    out['settle'] = float(ta[i[-1]]) if len(i) else 0.0
    end = ta > 0.5; out['final_err'] = float((y[end].mean() - 1.0) * 100) if end.any() else math.nan
    out['iq_dist'] = float(np.abs(iqa[after][ta < 0.6] - np.mean(iqa[pre])).max())
    ripple = (t >= 0.3) & (t < STEP_AT)
    out['idq_wander'] = float(np.hypot(ida[ripple], iqa[ripple]).max()) if ripple.any() else math.nan
    return out


def sweep(out: Path, kps=(1.0, 2.0, 3.5, 5.0, 7.0, 10.0), tis=(0.005, 0.010, 0.020, 0.040, 0.080)):
    """@brief D-GFL-10: connection (step 2b, APF feedforward, fitted grid) and an Id step for each (Kp, Ti)."""
    import sys
    sys.path.insert(0, str(REPO / 'tests' / 'common'))
    from interactive_plot import Figure
    out.mkdir(parents=True, exist_ok=True)
    use_measured_grid()
    res = {}
    for kp in kps:
        for ti in tis:
            r = run('apf', Bulb(thermal=False), t_after=STEP_AT + 0.65, kp=kp, ti=ti, id_step=(STEP_AT, STEP_A))
            res[(kp, ti)] = (r, step_metrics(r))
    def grid_table(title, key, fmt, note=''):
        L = [f'**{title}**{note}', '', '| Kp (V/A) \\ Ti (ms) | ' + ' | '.join(f'{ti * 1e3:g}' for ti in tis) + ' |',
             '|---|' + '---|' * len(tis)]
        for kp in kps:
            cells = []
            for ti in tis:
                m = res[(kp, ti)][1]
                tr = m['obs_trip'] if m['obs_trip'] is not None else m['cur_trip']
                cells.append(f'trip {tr:.2f} s' if tr is not None else fmt(m.get(key, math.nan)))
            mark = ' (today)' if (kp, abs(ti - TI_I) < 1e-9) == (KP_I, True) else ''
            L.append(f'| {kp:g}{mark} | ' + ' | '.join(cells) + ' |')
        return L + ['']
    f2 = lambda v: '—' if v != v else f'{v:.2f}'   # noqa: E731
    fms = lambda v: '—' if v != v else f'{v * 1e3:.0f}'   # noqa: E731
    fpc = lambda v: '—' if v != v else f'{v:.0f}'   # noqa: E731
    # interactive figure: Id step responses and the connection current for a few settings
    picks = [(KP_I, TI_I)] + [k for k in ((1.0, 0.005), (7.0, 0.010), (10.0, 0.040), (3.5, 0.005), (7.0, 0.040)) if k in res]
    cols = ['#2a78d6', '#eb6834', '#1baf7a', '#eda100', '#8e44ad', '#c2185b']
    fig = Figure('s', 'time from the contact (s)')
    c1 = fig.chart('A, Id (one-period mean)', 230); c2 = fig.chart('A, Iq (one-period mean)', 170); c3 = fig.chart('A, line current', 200)
    n = int(round(0.02 / TS)); k = np.ones(n) / n
    for (kp, ti), col in zip(picks, cols):
        r = res[(kp, ti)][0]; t = r['t'] - r['t_contact']; s_ = t >= -0.05; lab = f'Kp {kp:g}, Ti {ti * 1e3:g} ms'
        dec = 4
        fig.line(c1, lab, t[s_][::dec], np.convolve(r['id'], k, 'same')[s_][::dec], col, 1.4)
        fig.line(c2, lab, t[s_][::dec], np.convolve(r['iq'], k, 'same')[s_][::dec], col, 1.2)
        fig.line(c3, lab, t[s_][::dec], r['ig'][s_][::dec], col, 1.0)
    c1['events'] = [{'t': STEP_AT, 'label': f'Id reference +{STEP_A:g} A'}]
    L = ['# D-GFL-10: current PI gains (Kp, Ti) on the fitted step 2b model', '',
         'Design test ([ADR 0004](../../../adr/0004-averaged-model-for-fast-evaluation.md)), run at Luiz Villa\'s request on 2026-11-13: '
         'averaged model, **INFO only, no verdict**. It gives a ballpark of the current PI gains worth trying on the bench with the keys '
         '+ / − (Kp ±0.1 V/A, 0.5–10) and > / < (Ti ±1 ms, 1–100 ms), applied live. Generated by `tests/l1/design/connection_apf.py --sweep docs/testing/design/D-GFL-10_bench`.', '',
         f'**Model**: D-GFL-09 as fitted to the bench ([report](../D-GFL-09_bench/report.md)): APF feedforward (unchanged), measured grid, '
         f'{R_FIT:g} Ω behind the relay, {VDT_FIT:g} V of dead time, step 2b from the relay command. The gains apply from the start. '
         f'{STEP_A:g} A Id reference step {STEP_AT:g} s after the contact (the bench\'s step 3 size). Current PI output limit ±30 V. '
         'Trips: the grid observer (100 ms delay) and the 3 A current trip.', '',
         '**Reading the tables**: each cell is one (Kp, Ti); "trip" means the setting tripped before the end of the run.', '']
    L += grid_table('Connection: RMS line current 12–52 ms after the contact (A)', 'rms_12_52', f2)
    L += grid_table('Wander before the step: largest |Idq| (one-period mean) 0.3–0.6 s after the contact (A)', 'idq_wander', f2,
                    ' — bench with Kp 3.5, Ti 20 ms: telemetry |Idq| up to 0.3–0.5 A')
    L += grid_table('Id step 0.1 A: rise time 10–90 % (ms, one-period mean, so 10 ms is about the floor)', 'rise', fms)
    L += grid_table('Id step: settling to ±5 % within 0.6 s (ms; 600 = not settled)', 'settle', fms)
    L += grid_table('Id step: error 0.5–0.6 s after the step (%)', 'final_err', fpc)
    L += grid_table('Id step: overshoot (%)', 'overshoot', fpc)
    L += grid_table('Id step: largest Iq disturbance within 0.6 s (A)', 'iq_dist', f2)
    # gains changed online: today's gains for the connection, the keys pressed 0.3 s after the contact
    live = [(7.0, 0.005), (10.0, 0.010), (5.0, 0.005), (10.0, 0.005)]
    L += ['## Changing the gains online', '',
          '**Computed before the library made the change bumpless** (0caa792): here ∫e was kept, so the integral term jumped by the '
          'ratio of the new to the old Kp/Ti at the change; the Kp 5 / Ti 5 ms trip comes from that jump. The model now carries the '
          'integral term over, as the firmware does; rerun the sweep to refresh this table. '
          'On the bench the gains change while connected (keys + − > <; the telemetry reports kp and ti; every capture and firmware '
          'dump records the gains in force). The model applies the same change: today\'s gains (Kp 3.5 V/A, Ti 20 ms) through the '
          'connection, the new gains 0.3 s after the contact, the 0.1 A step at 0.6 s. `run(gains=...)` takes such a schedule, and '
          '`bench_gain_schedule()` rebuilds it from a bench console (the key presses since the last boot).', '',
          '| New gains at +0.3 s | Rise 10–90 % (ms) | Settling ±5 % (ms) | Overshoot (%) | Error 0.5–0.6 s (%) | Iq disturbance (A) | Trip |',
          '|---|---|---|---|---|---|---|']
    live_runs = []
    for kp_, ti_ in live:
        r = run('apf', Bulb(thermal=False), t_after=STEP_AT + 0.65, id_step=(STEP_AT, STEP_A), gains=[(0.3 + RELAY_ACTUAL, kp_, ti_)])
        mt = step_metrics(r); live_runs.append(((kp_, ti_), r))
        L.append(f'| Kp {kp_:g} V/A, Ti {ti_ * 1e3:g} ms | {fms(mt.get("rise", math.nan))} | {fms(mt.get("settle", math.nan))} | '
                 f'{fpc(mt.get("overshoot", math.nan))} | {fpc(mt.get("final_err", math.nan))} | {f2(mt.get("iq_dist", math.nan))} | {fmt_trip(mt)} |')
    m0 = res[(KP_I, TI_I)][1]
    L += [f'| unchanged (Kp {KP_I:g}, Ti {TI_I * 1e3:g} ms) | {fms(m0.get("rise", math.nan))} | {fms(m0.get("settle", math.nan))} | '
          f'{fpc(m0.get("overshoot", math.nan))} | {fpc(m0.get("final_err", math.nan))} | {f2(m0.get("iq_dist", math.nan))} | {fmt_trip(m0)} |', '']
    (kp_l, ti_l), r_l = live_runs[0]; t = r_l['t'] - r_l['t_contact']; s_ = t >= -0.05; lab = f'online: 3.5/20 ms → {kp_l:g}/{ti_l * 1e3:g} ms at +0.3 s'
    fig.line(c1, lab, t[s_][::4], np.convolve(r_l['id'], k, 'same')[s_][::4], '#5d6d7e', 1.4, dash=[5, 3])
    fig.line(c2, lab, t[s_][::4], np.convolve(r_l['iq'], k, 'same')[s_][::4], '#5d6d7e', 1.2, dash=[5, 3])
    fig.line(c3, lab, t[s_][::4], r_l['ig'][s_][::4], '#5d6d7e', 1.0, dash=[5, 3])
    c1['events'].append({'t': 0.3, 'label': 'online change'})
    (out / 'iplot_sweep.html').write_text(fig.html('dgfl10-sweep', 'D-GFL-10 step responses for selected current PI gains'), encoding='utf-8')
    L += ['## Responses', '', '<!-- iplot sweep -->', '',
          'Interactive: drag to zoom, double-click to reset, click a legend entry to hide a setting. Id and Iq as one-period means '
          '(the 2ω ripple of the instantaneous dq current removed).', '',
          '## Limits', '', '- Same as D-GFL-09: averaged bridge, fitted grid and dead time, no SYNC_LOST; the ff 1 trip was not reproduced '
          'there, so a setting that survives here is a candidate, not a guarantee.',
          '- The one-period mean that removes the 2ω ripple also slows the measured rise time (floor about 10 ms).',
          '- A result here never passes or fails the firmware (ADR 0004).']
    (out / 'report.md').write_text('\n'.join(L) + '\n', encoding='utf-8')
    return out / 'report.md', res


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--report', help='write report.md and figures into this folder')
    ap.add_argument('--sweep', help='D-GFL-10: current PI gain sweep, report into this folder')
    ap.add_argument('--source', default='apf', choices=['apf', 'sogi', 'pll', 'apf_g07'])
    ap.add_argument('--fixed', type=float, help='fixed bulb resistance (Ω) instead of the thermal model')
    o = ap.parse_args()
    if o.report:
        print(report(Path(o.report)))
        return
    if o.sweep:
        print(sweep(Path(o.sweep))[0])
        return
    use_measured_grid()
    r = run(o.source, Bulb(thermal=False, r_cold=o.fixed) if o.fixed else Bulb())
    print(HEADER.split('\n')[0]); print(row(o.source, metrics(r)))


if __name__ == '__main__':
    main()
