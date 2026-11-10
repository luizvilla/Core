#!/usr/bin/env python3
## SPDX-License-Identifier: LGPL-2.1
## @file ladder.py
## @brief Averaged-model fidelity ladder of the inverter connection (design tests D-GFL, ADR 0004).
## @par What it does
##   One candidate, one grid, the bench sequence from PWM_SETTLE: relay commanded so the contact falls
##   on a voltage zero crossing, contact 9.7 ms after the command, then an I_d step. The controller is
##   the library's arithmetic (sogi.cpp in float32 at the PLL frequency, park, pid.cpp, PLL PI from
##   init(), output rotation with an advance). Plant elements are switched on one at a time:
##     pll    the library PLL instead of the ideal grid angle
##     caps   the output filter capacitors (C113 120 nF; C7 680 nF + R3 4.7 Ω) with the relay contact
##            on charged capacitors; IAC measured between the capacitors and the relay
##     delay  actuation delay (s) between sampling and the duty taking effect
##   The bridge is averaged (no ripple, no dead time): the switched effects are rungs for L1.
## @par Usage
##   python3 tests/l1/design/ladder.py            # the ladder on the IEC grid, candidate C
"""Design aid only (ADR 0004): it decides whether an idea is worth a switched run, never a verdict."""
import argparse, math
import numpy as np
from scipy.linalg import expm

F32 = np.float32
TS = 1e-4; SUB = 10; H = TS / SUB
W0 = 2 * math.pi * 50; VPK = 325.2691193458119; VBUS = 400.0
L_F, R_F = 500e-6, 0.1
C113, C7, R3 = 120e-9, 680e-9, 4.7
GRIDS = {'IEC': (0.4, 0.8e-3), 'bulb cold': (40.0, 0.0), 'bulb hot': (500.0, 0.0), 'stiff': (0.01, 10e-6)}
T_CMD_NOMINAL, T_STEP, T_END = 0.40, 0.60, 0.80
RELAY_CLOSE, RELAY_FEEDBACK = 0.0097, 0.020
TRIP = 1.5


class Sogi:
    """sogi.cpp, float32, w0 given per call (the library passes the clamped PLL frequency)."""
    def __init__(self, kr):
        self.kr = F32(kr); self.n1 = F32(0); self.d = [F32(0)] * 3

    def calc(self, x, w0):
        wt = F32(w0) * F32(TS); c = F32(1) - F32(0.5) * wt * wt; inv = F32(1) / wt
        n0 = self.kr * (F32(x) - self.d[0])
        d0 = F32(TS) * (n0 - c * self.n1) + F32(2) * c * self.d[1] - self.d[2]
        a = d0; b = -c * inv * d0 + inv * self.d[1]
        self.n1 = n0; self.d = [d0, d0, self.d[1]]
        return float(a), float(b)


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


# Capacitor network on the filter node. DEFAULT_NET is the one used up to D-GFL-04 (C113 on the node, C7 + R3);
# SCHEMATIC_NET follows the board (reference/output_filter.png): C120 1 nF and C113 120 nF on the node, C7 680 nF
# + R3 4.7 Ω, and C122 120 nF on the grid side of the relay, charged by the grid while the relay is open.
DEFAULT_NET = dict(C113=C113, C120=0.0, C7=C7, R3=R3, R113=0.0, C122=0.0)
SCHEMATIC_NET = dict(C113=120e-9, C120=1e-9, C7=680e-9, R3=4.7, R113=0.0, C122=120e-9)


def capnet(rg, lg, closed, net):
    """@return (Ad, Bd, names, Cn): exact sub-step matrices of the capacitor network, the state names and the
    node capacitance. States: iL, vn, [v7], [v113 when C113 has a series resistor], [vp: C122 with the relay
    open], [ig when the grid has inductance]. Inputs [v_bridge, v_grid]."""
    names = ['iL', 'vn']
    if net['C7'] > 0: names.append('v7')
    if net['R113'] > 0 and net['C113'] > 0: names.append('v113')
    if net['C122'] > 0 and not closed: names.append('vp')
    grid_state = lg > 0 and (closed or net['C122'] > 0)
    if grid_state: names.append('ig')
    ix = {k: i for i, k in enumerate(names)}; n = len(names); A = np.zeros((n, n)); B = np.zeros((n, 2))
    cn = net['C120'] + (net['C113'] if net['R113'] == 0 else 0.0) + (net['C122'] if closed else 0.0)
    cn = max(cn, 1e-12)
    A[ix['iL'], ix['iL']] = -R_F / L_F; A[ix['iL'], ix['vn']] = -1 / L_F; B[ix['iL'], 0] = 1 / L_F
    A[ix['vn'], ix['iL']] += 1 / cn
    for st, c, r in (('v7', net['C7'], net['R3']), ('v113', net['C113'], net['R113'])):
        if st in ix:
            g = 1 / r; A[ix['vn'], ix['vn']] -= g / cn; A[ix['vn'], ix[st]] += g / cn
            A[ix[st], ix['vn']] += g / c; A[ix[st], ix[st]] -= g / c
    if closed:
        if grid_state:
            A[ix['vn'], ix['ig']] -= 1 / cn; A[ix['ig'], ix['vn']] = 1 / lg; A[ix['ig'], ix['ig']] = -rg / lg; B[ix['ig'], 1] = -1 / lg
        else:
            A[ix['vn'], ix['vn']] -= 1 / (rg * cn); B[ix['vn'], 1] += 1 / (rg * cn)
    elif 'vp' in ix:
        c = net['C122']
        if grid_state:
            A[ix['vp'], ix['ig']] = -1 / c; A[ix['ig'], ix['vp']] = 1 / lg; A[ix['ig'], ix['ig']] = -rg / lg; B[ix['ig'], 1] = -1 / lg
        else:
            A[ix['vp'], ix['vp']] = -1 / (rg * c); B[ix['vp'], 1] = 1 / (rg * c)
    M = np.zeros((n + 2, n + 2)); M[:n, :n] = A; M[:n, n:] = B
    E = expm(M * H); return E[:n, :n], E[:n, n:], names, cn


def plant(caps, rg, lg, closed):
    """@return (Ad, Bd, n_states) for x' = A x + B [v_bridge, v_grid], exact over one sub-step.
    States: i_L, then (caps) v_n (C113), v_7 (C7), then (closed, lg > 0) i_g."""
    if not caps:  # L straight to the PCC: one state, i_L = i_g
        lt, rt = L_F + lg, R_F + rg
        A = np.array([[-rt / lt]]); B = np.array([[1 / lt, -1 / lt]]) if closed else np.zeros((1, 2))
        if not closed: A = np.array([[-R_F / L_F]])
    else:
        n = 4 if (closed and lg > 0) else 3; A = np.zeros((n, n)); B = np.zeros((n, 2))
        A[0, 0] = -R_F / L_F; A[0, 1] = -1 / L_F; B[0, 0] = 1 / L_F           # L di/dt = vb - r i - vn
        A[1, 0] = 1 / C113; A[1, 1] = -1 / (R3 * C113); A[1, 2] = 1 / (R3 * C113)  # C113 dvn = iL - i7 - ig
        A[2, 1] = 1 / (R3 * C7); A[2, 2] = -1 / (R3 * C7)                      # C7 dv7 = i7
        if closed and lg > 0:
            A[1, 3] = -1 / C113; A[3, 1] = 1 / lg; A[3, 3] = -rg / lg; B[3, 1] = -1 / lg
        elif closed:  # resistive grid: ig = (vn - vg) / rg
            A[1, 1] -= 1 / (rg * C113); B[1, 1] = 1 / (rg * C113)
    n = A.shape[0]; M = np.zeros((n + 2, n + 2)); M[:n, :n] = A; M[:n, n:] = B
    E = expm(M * H); return E[:n, :n], E[:n, n:], n


def run(rg, lg, pll=False, caps=False, delay=TS, pi_from='command', kp=1.0, ti=0.005, kr_i=2000.0,
        adv=0.5, lpf_fc=None, iref=0.1, trip=TRIP, ff='measured', contact_deg=0.0,
        synth_blank=None, adv_synth=0.3, fade=0.0, pll_freeze=None, net=None, ff_gain=1.0, ff_phase_deg=0.0, bridge_gain=1.0, bridge_phase_deg=0.0,
        clamp_v=None, clamp_i=None, clamp_window=0.0, clamp_pll=False, ff_off=None, t_end=None, iref_fn=None):
    """synth_blank (s): feedforward synthesized (pre-command period average of Vd, PLL angle, advance adv_synth)
    from the relay command to contact + synth_blank, then faded to the measured one over fade (s).
    pll_freeze (s): PLL integrator held and ω latched (pre-command period average) from the contact to
    contact + pll_freeze; θ free-runs."""
    """@return dict of traces at the control rate and the trip time (None if no trip)."""
    sv, si = Sogi(500.0), Sogi(kr_i)
    pid, piq = Pi(kp, ti, -30.0, 30.0), Pi(kp, ti, -30.0, 30.0)
    wn = 3.0 / (2 * math.pi / W0); kpp = 2 * wn * 0.7 / VPK; kip = wn * wn / VPK
    pll_pi = Pi(kpp, kpp / kip, -10 * W0, 10 * W0)
    # Relay command: the application latches it so the contact (9.7 ms later) falls on a zero crossing.
    # contact_deg: grid-voltage angle at the contact, from a rising zero crossing (v = Vpk·sin, zero at k·20 ms).
    t_contact = math.ceil((T_CMD_NOMINAL + RELAY_CLOSE) / 0.02) * 0.02 + (contact_deg % 360) / 360 * 0.02
    t_cmd = t_contact - RELAY_CLOSE; t_fb = t_cmd + RELAY_FEEDBACK
    if caps:
        net = {**DEFAULT_NET, **(net or {})}
        cmats = {c: capnet(rg, lg, c, net) for c in (False, True)}
        names = cmats[False][2]; x = np.zeros(len(names))
    else:
        mats = {c: plant(caps, rg, lg, c) for c in (False, True)}
        x = np.zeros(mats[False][2])
    was_closed = False
    theta, w, vq_prev = 0.0, W0, 0.0; ilp = 0.0; a_lp = 1 - math.exp(-2 * math.pi * lpf_fc * TS) if lpf_fc else None
    pending = []  # (time it takes effect, bridge voltage)
    hist_vd, hist_w, hist_vq = [], [], []; vd_lat, w_lat = VPK, W0
    vb = 0.0; steps = int((t_end or T_END) / TS)
    rec = {k: np.zeros(steps) for k in ('t', 'ig', 'idm', 'iqm', 'vd', 'vq', 'w', 'duty', 'th_err', 'vn', 'vac', 'vg', 'ref', 'pid_d', 'pid_q')}
    trip_t = None
    for k in range(steps):
        t = k * TS; closed = t >= t_contact
        if closed and not was_closed:  # relay contact
            if caps:  # node and C122 joined: charge shared; other states carried over by name
                old = dict(zip(names, x)); new_names = cmats[True][2]; cn_open = cmats[False][3]
                if 'vp' in old: old['vn'] = (cn_open * old['vn'] + net['C122'] * old['vp']) / (cn_open + net['C122'])
                x = np.array([old.get(k, 0.0) for k in new_names]); names = new_names
            else:
                n_new = mats[True][2]; x = np.concatenate([x, np.zeros(n_new - len(x))])
            was_closed = True
        vg = VPK * math.sin(W0 * t)
        if caps:
            xs = dict(zip(names, x)); vn_now = xs['vn']
            if 'ig' in xs: ig = xs['ig']
            elif closed: ig = (xs['vn'] - vg) / rg
            elif 'vp' in xs: ig = (xs['vp'] - vg) / rg
            else: ig = 0.0
            vpcc = xs['vn'] if closed else xs.get('vp', vg)   # VAC on the grid side of the relay
        else:
            ig = x[0] if closed else 0.0; vn_now = math.nan
        if not caps and not closed: vpcc = vg
        elif not caps:  # no capacitors: VAC = vg + rg·i + lg·di/dt, di/dt from the applied bridge voltage
            vpcc = vg + rg * ig + lg * (vb - vg - (R_F + rg) * ig) / (L_F + lg)
        # --- controller (calculateDuty order) ---
        frozen = pll_freeze is not None and t_contact <= t < t_contact + pll_freeze
        if pll and frozen:
            w = w_lat; theta = (theta + w * TS) % (2 * math.pi)
        elif pll:
            w = W0 + pll_pi(0.0, -vq_prev); theta = (theta + w * TS) % (2 * math.pi)
        else:
            w = W0; theta = (W0 * t - math.pi / 2) % (2 * math.pi)
        w_s = min(max(w, 0.95 * W0), 1.05 * W0)
        a, b = sv.calc(vpcc, w_s)
        imeas = ig
        if lpf_fc: ilp += a_lp * (imeas - ilp); imeas = ilp
        ia, ib = si.calc(imeas, w_s)
        c, s = math.cos(theta), math.sin(theta)
        vd, vq = a * c + b * s, -a * s + b * c; idm, iqm = ia * c + ib * s, -ia * s + ib * c
        hist_vd.append(vd); hist_w.append(w)
        if t < t_cmd: vd_lat = float(np.mean(hist_vd[-200:])); w_lat = float(np.mean(hist_w[-200:]))
        # D-GFL-05: park outputs clamped around their pre-command values, from the command to contact + window.
        in_clamp = t_cmd <= t < t_contact + clamp_window
        if in_clamp and clamp_v is not None:
            vd = min(max(vd, vd_lat - clamp_v), vd_lat + clamp_v); vq_c = min(max(vq, -clamp_v), clamp_v)
            vq_prev = vq_c if clamp_pll else vq; vq = vq_c
        else:
            vq_prev = vq
        if in_clamp and clamp_i is not None:
            idm = min(max(idm, -clamp_i), clamp_i); iqm = min(max(iqm, -clamp_i), clamp_i)
        hist_vq.append(vq)
        if ff == 'exact': vd, vq = VPK, 0.0   # feedforward from the true grid voltage instead of the measured VAC
        elif ff in ('avg', 'avg_d'):  # one-period moving average of the park outputs (library _Vdq_avg), always on
            vd = float(np.mean(hist_vd[-200:])); vq = float(np.mean(hist_vq[-200:])) if ff == 'avg' else 0.0
        active = pi_from != 'never' and t >= (t_cmd if pi_from == 'command' else t_fb)
        if active:
            ref = iref_fn(t) if iref_fn else (iref if t >= T_STEP else 0.0)
            pd_, pq_ = pid(ref, idm), piq(0.0, iqm)
        else:
            pid.reset(); piq.reset(); pd_, pq_ = 0.0, 0.0; ref = 0.0
        ks = 0.0  # weight of the synthesized feedforward
        if synth_blank is not None and t >= t_cmd:
            t_end_s = t_contact + synth_blank
            ks = 1.0 if t < t_end_s else (max(0.0, 1.0 - (t - t_end_s) / fade) if fade > 0 else 0.0)
        tho = theta + adv * w * TS; ths = theta + adv_synth * w * TS
        thf = tho + math.radians(ff_phase_deg); g_ff = (1 - ks) * ff_gain
        if ff_off is not None:  # (start, end, fade) relative to the contact: measured feedforward off, faded back in
            a0, a1, fd = ff_off; tr = t - t_contact
            if a0 <= tr < a1: g_ff = 0.0
            elif a1 <= tr < a1 + fd: g_ff *= (tr - a1) / fd
        va = (pd_ * math.cos(tho) - pq_ * math.sin(tho) + g_ff * (vd * math.cos(thf) - vq * math.sin(thf))
              + ks * vd_lat * math.cos(ths))
        duty = min(max(va / (2 * VBUS) + 0.5, 0.05), 0.95)
        vb_cmd = VBUS * (2 * duty - 1)
        if bridge_gain != 1.0 or bridge_phase_deg:
            # Plant-side error of the bridge fundamental (emulates the switched model's unloaded bridge, D-GFL-07):
            # the voltage actually produced is the command scaled and rotated (rotation applied to the AC command).
            vb_cmd = bridge_gain * (va * math.cos(math.radians(bridge_phase_deg)) - (va_q := (pd_ * math.sin(tho) + pq_ * math.cos(tho)
                     + g_ff * (vd * math.sin(thf) + vq * math.cos(thf)) + ks * vd_lat * math.sin(ths))) * math.sin(math.radians(bridge_phase_deg)))
            vb_cmd = min(max(vb_cmd, VBUS * (2 * 0.05 - 1)), VBUS * (2 * 0.95 - 1))
        pending.append((t + delay, vb_cmd))
        # --- plant over one control period ---
        Ad, Bd = (cmats[closed][0], cmats[closed][1]) if caps else mats[closed][:2]
        for j in range(SUB):
            tj = t + j * H
            while pending and pending[0][0] <= tj + 1e-12: vb = pending.pop(0)[1]
            x = Ad @ x + Bd @ np.array([vb, VPK * math.sin(W0 * (tj + H / 2))])
        th_err = (theta - (W0 * t - math.pi / 2) + math.pi) % (2 * math.pi) - math.pi
        for key, val in (('t', t), ('ig', ig), ('idm', idm), ('iqm', iqm), ('vd', vd), ('vq', vq), ('w', w), ('duty', duty), ('th_err', th_err), ('vn', vn_now), ('vac', vpcc), ('vg', vg), ('ref', ref), ('pid_d', pd_), ('pid_q', pq_)):
            rec[key][k] = val
        if closed and abs(ig) > trip and trip_t is None: trip_t = t
        if trip_t is not None and t > trip_t + 0.02: rec = {kk: v[:k + 1] for kk, v in rec.items()}; break
    rec['t_contact'], rec['trip'] = t_contact, trip_t
    return rec


def summary(r, iref=0.1):
    t, ig = r['t'], r['ig']; tc = r['t_contact']; m = (t >= tc) & (t < tc + 0.005)
    out = {'trip_after_contact_ms': None if r['trip'] is None else 1e3 * (r['trip'] - tc),
           'peak_ig_5ms_A': float(np.abs(ig[m]).max()) if m.any() else math.nan,
           'peak_Idq_seen_5ms_A': float(np.hypot(r['idm'][m], r['iqm'][m]).max()) if m.any() else math.nan,
           'peak_ig_after_contact_A': float(np.abs(ig[t >= tc]).max())}
    if r['trip'] is None:
        late = t >= T_END - 0.04; out['final_id_A'] = float(r['idm'][late].mean())
        out['pre_step_peak_A'] = float(np.abs(ig[(t >= tc + 0.1) & (t < T_STEP)]).max())
    return out


MERMAID = """flowchart LR
  subgraph PLANT["Plant (averaged)"]
    BR["Bridge<br/>v = Vbus(2D−1)"] --> LF["500 µH<br/>(r 0.1 Ω)"]
    LF --> NODE(("filter node"))
    NODE --- C113["C113 120 nF"]
    NODE --- C7["C7 680 nF + R3 4.7 Ω"]
    NODE --> IP["IAC probe"] --> RL["Relay<br/>(contact 9.7 ms<br/>after the command)"]
    RL --> PCC(("PCC")) --> ZG["Grid impedance<br/>IEC: 0.4 Ω + 0.8 mH"] --> VG["Grid 230 V 50 Hz"]
  end
  subgraph CTRL["Controller (library arithmetic, every 100 µs)"]
    SV["SOGI v<br/>Kr 500"] --> PV["park θ"]
    PV --> PLL["PLL PI on Vq"] --> TH["θ, ω"]
    SI["SOGI i<br/>Kr_i 2000<br/>(β by difference)"] --> PI_["park θ"] --> PIC["PI d, q<br/>Kp, Ti, ±30 V"]
    PV -- "feedforward Vdq" --> SUM(("+"))
    PIC --> SUM --> IP2["inverse park<br/>θ + advance·ω·Ts"] --> DU["duty, clamp 0.05–0.95"]
  end
  PCC -- "VAC" --> SV
  IP -- "IAC" --> SI
  TH -.-> PV & PI_ & IP2
  DU -- "delay τ (80 µs)" --> BR
"""

COLORS = ['#2a78d6', '#eb6834', '#1baf7a', '#eda100']  # dataviz reference palette, slots 1-4


def time_metrics(rg, lg, **kw):
    """Trip disabled: the connection transient itself."""
    r = run(rg, lg, trip=1e9, **kw); t, ig = r['t'], r['ig']; tc = r['t_contact']
    after = t >= tc; peak = float(np.abs(ig[after & (t < tc + 0.01)]).max())
    big = np.where(after & (np.abs(ig) > 0.15) & (t < T_STEP))[0]
    dur = float(1e3 * (t[big[-1]] - tc)) if len(big) else 0.0
    m5 = after & (t < tc + 0.005)
    return {'peak10': peak, 'seen5': float(np.hypot(r['idm'][m5], r['iqm'][m5]).max()), 'real5': float(np.abs(ig[m5]).max()),
            'dur_ms': dur, 'trips': bool(np.abs(ig[after]).max() > TRIP), 'final_id': float(r['idm'][t > T_END - 0.04].mean())}


def report(grid, out):
    import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
    from pathlib import Path
    import loop_linear as ll
    out = Path(out); out.mkdir(parents=True, exist_ok=True); rg, lg = GRIDS[grid]
    base = dict(kp=1.0, ti=0.005, kr_i=2000.0, adv=0.5)
    cases = [  # (label, time-domain kwargs, linear kwargs, plotted)
        ('1 averaged L, ideal angle, delay 100 µs', {}, dict(caps=False, delay=TS), True),
        ('2 + library PLL', {'pll': True}, None, False),
        ('3 + filter capacitors, contact on charged capacitors', {'pll': True, 'caps': True}, dict(caps=True, delay=TS), True),
        ('4 + delay 80 µs (L1 assumption)', {'pll': True, 'caps': True, 'delay': 80e-6}, dict(caps=True, delay=80e-6), True),
        ('4, PI from the feedback (library today)', {'pll': True, 'caps': True, 'delay': 80e-6, 'pi_from': 'feedback'}, None, False),
        ('4, IAC low-pass 500 Hz', {'pll': True, 'caps': True, 'delay': 80e-6, 'lpf_fc': 500.0}, dict(caps=True, delay=80e-6, lpf_fc=500.0), True),
        ('4, no output advance', {'pll': True, 'caps': True, 'delay': 80e-6, 'adv': 0.0}, dict(caps=True, delay=80e-6, adv=0.0), False),
        ('4, Kp 0.5', {'pll': True, 'caps': True, 'delay': 80e-6, 'kp': 0.5}, dict(caps=True, delay=80e-6, kp=0.5), False)]
    rows, curves = [], []
    for label, tk, lk, plot in cases:
        tm = time_metrics(rg, lg, **{**base, **tk})
        lm = None
        if lk is not None:
            r = ll.loop(rg=rg, lg=lg, **{**base, **lk}); lm = ll.margins(r)
            if plot: curves.append((label, r))
        rows.append((label, tm, lm))
    # Bode plots (dq frame), one figure per loop, magnitude over phase.
    for key, name, title in (('L', 'bode_open.png', 'Open loop L (broken at the bridge voltage)'),
                             ('T', 'bode_closed.png', 'Closed loop T (current reference to line current)')):
        fig, (am, ap) = plt.subplots(2, 1, figsize=(8, 6.4), sharex=True, facecolor='#fcfcfb')
        for (label, r), col in zip(curves, COLORS):
            y = r[key]; am.semilogx(r['f'], 20 * np.log10(np.abs(y)), color=col, lw=1.5, label=label)
            ap.semilogx(r['f'], np.degrees(np.unwrap(np.angle(y))), color=col, lw=1.5)
        for ax in (am, ap):
            ax.set_facecolor('#fcfcfb'); ax.grid(True, which='both', color='#e4e3df', lw=0.6)
            for sp in ('top', 'right'): ax.spines[sp].set_visible(False)
            ax.tick_params(colors='#52514e')
        if key == 'L': am.axhline(0, color='#52514e', lw=0.8)
        else: am.axhline(-3, color='#52514e', lw=0.8, ls='--')
        am.set_ylabel('Magnitude (dB)', color='#0b0b0b'); ap.set_ylabel('Phase (°)', color='#0b0b0b')
        ap.set_xlabel('Frequency in the dq frame (Hz, offset from 50 Hz)', color='#0b0b0b')
        am.set_title(f'{title}, {grid} grid', color='#0b0b0b', loc='left', fontsize=11)
        am.legend(frameon=False, fontsize=8, labelcolor='#0b0b0b'); fig.tight_layout(); fig.savefig(out / name, dpi=130); plt.close(fig)
    f = lambda v, d=3: '—' if v is None or (isinstance(v, float) and not math.isfinite(v)) else f'{v:.{d}g}'
    lines = [f'# D-GFL-01: connection fidelity ladder, {grid} grid', '',
             f'Design test (ADR 0004): averaged model, INFO only, no verdict. Candidate C (Kp 1 V/A, Ti 5 ms, current SOGI 2000 rad/s, advance 0.5 period), current PI active from the relay command unless stated. Grid: {rg} Ω, {lg * 1e3:g} mH. Generated by `tests/l1/design/ladder.py --report`.', '',
             '## Plant and loop', '', '```mermaid', MERMAID.rstrip(), '```', '',
             '## Margins and bandwidth (linear model, relay closed, ideal angle)', '',
             'Margins are open-loop properties; the closed loop is described by its bandwidth and its resonance peak. Frequencies in the dq frame. Rung 2 (PLL) has no linear row: the linear model uses the ideal angle.', '',
             '| Case | Open loop: crossover (Hz) | Open loop: phase margin (°) | Open loop: gain margin (dB) | Closed loop: bandwidth −3 dB (Hz) | Closed loop: peak (dB) | Closed loop: peak at (Hz) |',
             '|---|---|---|---|---|---|---|']
    for label, tm, lm in rows:
        if lm: lines.append(f"| {label} | {f(lm['crossover_Hz'])} | {f(lm['pm_deg'])} | {f(lm['gm_dB'])} | {f(lm['bandwidth_Hz'])} | {f(lm['peak_dB'])} | {f(lm['peak_Hz'])} |")
    lines += ['', '![Open loop](bode_open.png)', '', '![Closed loop](bode_closed.png)', '',
              '## Connection transient (time domain, trip disabled)', '',
              'Contact on a voltage zero crossing; the 1.5 A bench trip is reported, not applied, so the whole transient is visible.', '',
              '| Case | Peak line current, first 10 ms (A) | Would trip at 1.5 A | Line current, first 5 ms (A) | \\|I_dq\\| seen by the controller, first 5 ms (A) | Transient above 0.15 A (ms) | I_d after the 0.1 A step (A) |',
              '|---|---|---|---|---|---|---|']
    for label, tm, lm in rows:
        lines.append(f"| {label} | {f(tm['peak10'])} | {'yes' if tm['trips'] else 'no'} | {f(tm['real5'])} | {f(tm['seen5'])} | {f(tm['dur_ms'])} | {f(tm['final_id'])} |")
    (out / 'report.md').write_text('\n'.join(lines) + '\n')
    print('\n'.join(lines))


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('--grid', default='IEC'); ap.add_argument('--report', default=None)
    o = ap.parse_args()
    if o.report: report(o.grid, o.report); return
    rg, lg = GRIDS[o.grid]
    rungs = [('1  averaged L, ideal angle, delay 1 period', {}),
             ('2  + library PLL', {'pll': True}),
             ('3  + filter capacitors, contact on charged capacitors', {'pll': True, 'caps': True}),
             ('4  + delay 80 µs (L1 assumption)', {'pll': True, 'caps': True, 'delay': 80e-6})]
    print(f'grid {o.grid} (rg {rg} Ω, lg {lg * 1e3} mH), candidate C (Kp 1, Ti 5 ms, Kr_i 2000, advance 0.5), PI from the relay command\n')
    print('| Rung | Trip after contact (ms) | Peak line current, 5 ms (A) | Peak \\|I_dq\\| seen, 5 ms (A) | Peak line current after contact (A) | Final I_d (A) |')
    print('|---|---|---|---|---|---|')
    for name, kw in rungs:
        s = summary(run(rg, lg, **kw))
        f = lambda v: '—' if v is None or (isinstance(v, float) and not math.isfinite(v)) else f'{v:.3g}'
        print(f"| {name} | {f(s['trip_after_contact_ms'])} | {f(s['peak_ig_5ms_A'])} | {f(s['peak_Idq_seen_5ms_A'])} | {f(s['peak_ig_after_contact_A'])} | {f(s.get('final_id_A'))} |")


if __name__ == '__main__':
    main()
