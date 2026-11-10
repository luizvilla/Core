#!/usr/bin/env python3
## SPDX-License-Identifier: LGPL-2.1
## @file loop_linear.py
## @brief Linear frequency response of the inverter current loop with the relay closed (design, ADR 0004).
## @par Model (exact in discrete time, ideal grid angle)
##   Plant: averaged bridge → 500 µH → [C113 ∥ (C7 + R3)] → IAC probe → PCC (VAC) → grid impedance,
##   sampled every Ts, the duty taking effect a fractional delay τ after the sample.
##   Controller, as the library computes it (z = e^{jωTs}):
##     SOGI (sogi.cpp): Hα = Ts·Kr(1 − c z⁻¹) / [1 − 2c z⁻¹ + z⁻² + Ts·Kr(1 − c z⁻¹) z⁻¹], Hβ = (z⁻¹ − c)/(ω0 Ts) · Hα
##     PI (pid.cpp):    C = Kp (1 + Ts / (Ti (1 − z⁻¹)))
##     dq rotation:     a dq controller seen from the stationary frame is C shifted by ±ω0; for a real input
##                      Re{e^{jφ} C(z e^{-jω0Ts})(Hα + jHβ)} gives K(ω) = ½[G(ω) + conj G(−ω)], φ the output advance
##   Loop broken at the bridge voltage: L = K_i·P_ig − F·P_vac (u = F·vac − K_i·i, negative feedback 1 + L).
##   Closed loop: T = K_i·P_ig / (1 + L), from a current reference to the line current.
##   Frequencies are reported in the dq frame (offset from 50 Hz), as the PI is designed.
"""Design aid only: the margins rank ideas; stability is confirmed by the time-domain rung and by L1."""
import math
import numpy as np
from scipy.linalg import expm

TS = 1e-4; W0 = 2 * math.pi * 50
L_F, R_F = 500e-6, 0.1
C113, C7, R3 = 120e-9, 680e-9, 4.7


def plant_matrices(caps, rg, lg):
    """@return A, B (input bridge voltage), C_ig, C_vac, D_vac for the relay closed (grid voltage = disturbance)."""
    if not caps:
        lt = L_F + lg; rt = R_F + rg
        A = np.array([[-rt / lt]]); B = np.array([[1 / lt]]); Ci = np.array([[1.0]])
        # vac = vb − R_F i − L_F di/dt with di/dt = (vb − rt i)/lt (grid voltage dropped)
        Cv = np.array([[-R_F + L_F * rt / lt]]); Dv = 1 - L_F / lt
        return A, B, Ci, Cv, Dv
    n = 4 if lg > 0 else 3; A = np.zeros((n, n)); B = np.zeros((n, 1))
    A[0, 0] = -R_F / L_F; A[0, 1] = -1 / L_F; B[0, 0] = 1 / L_F
    A[1, 0] = 1 / C113; A[1, 1] = -1 / (R3 * C113); A[1, 2] = 1 / (R3 * C113)
    A[2, 1] = 1 / (R3 * C7); A[2, 2] = -1 / (R3 * C7)
    Ci = np.zeros((1, n)); Cv = np.zeros((1, n)); Cv[0, 1] = 1.0
    if lg > 0:
        A[1, 3] = -1 / C113; A[3, 1] = 1 / lg; A[3, 3] = -rg / lg; Ci[0, 3] = 1.0
    else:
        A[1, 1] -= 1 / (rg * C113); Ci[0, 1] = 1 / rg  # ig = vn / rg (grid voltage dropped)
    return A, B, Ci, Cv, 0.0


def _phi_gamma(A, B, h):
    n = A.shape[0]; M = np.zeros((n + 1, n + 1)); M[:n, :n] = A; M[:n, n:] = B
    E = expm(M * h); return E[:n, :n], E[:n, n:]


def plant_response(w, caps, rg, lg, delay):
    """@return P_ig(ω), P_vac(ω): sampled response from the duty command u[k] (taking effect at kTs + τ)."""
    A, B, Ci, Cv, Dv = plant_matrices(caps, rg, lg); n = A.shape[0]
    Phi, _ = _phi_gamma(A, B, TS)
    eA, g_tau = _phi_gamma(A, B, delay)                       # over [kTs, kTs + τ): u[k−1]
    e_rest, g_rest = _phi_gamma(A, B, TS - delay)             # over [kTs + τ, (k+1)Ts): u[k]
    G1 = e_rest @ g_tau; G0 = g_rest                          # x[k+1] = Φx + G0 u[k] + G1 u[k−1]
    z = np.exp(1j * w * TS); Pi_ = np.zeros(len(w), complex); Pv = np.zeros(len(w), complex)
    I = np.eye(n)
    for k, zk in enumerate(z):
        X = np.linalg.solve(zk * I - Phi, (G0 + G1 / zk))
        Pi_[k] = (Ci @ X)[0, 0]; Pv[k] = (Cv @ X)[0, 0] + Dv / zk  # vac sees the applied u[k−1] at kTs
    return Pi_, Pv


def sogi(z, kr, w0=W0):
    wt = w0 * TS; c = 1 - 0.5 * wt * wt; inv = 1 / wt; zi = 1 / z
    ha = TS * kr * (1 - c * zi) / ((1 - 2 * c * zi + zi * zi) + TS * kr * (1 - c * zi) * zi)
    return ha, inv * (zi - c) * ha


def controller_response(w, kp, ti, kr_i, adv, lpf_fc=None, kr_v=500.0):
    """@return K_i(ω) (current path, sign: u = −K_i·i), F(ω) (voltage feedforward) in the stationary frame."""
    phi = adv * W0 * TS
    def g_i(ww):
        z = np.exp(1j * ww * TS); ha, hb = sogi(z, kr_i)
        zs = np.exp(1j * (ww - W0) * TS); c = kp * (1 + TS / (ti * (1 - 1 / zs)))
        lp = 1.0
        if lpf_fc:
            a = 1 - math.exp(-2 * math.pi * lpf_fc * TS); lp = a / (1 - (1 - a) / z)
        return np.exp(1j * phi) * c * (ha + 1j * hb) * lp
    def g_v(ww):
        z = np.exp(1j * ww * TS); ha, hb = sogi(z, kr_v); return np.exp(1j * phi) * (ha + 1j * hb)
    K = 0.5 * (g_i(w) + np.conj(g_i(-w))); F = 0.5 * (g_v(w) + np.conj(g_v(-w)))
    return K, F


def loop(caps, rg, lg, delay, kp, ti, kr_i, adv, lpf_fc=None, f_dq=None):
    """@return dict: dq-frame frequency (Hz), open loop L, closed loop T, at ω = ω0 + 2π f_dq."""
    if f_dq is None: f_dq = np.logspace(0, math.log10(1 / (2 * TS) - 60), 3000)
    w = W0 + 2 * math.pi * f_dq
    Pi_, Pv = plant_response(w, caps, rg, lg, delay); K, F = controller_response(w, kp, ti, kr_i, adv, lpf_fc)
    L = K * Pi_ - F * Pv; T = K * Pi_ / (1 + L)
    return {'f': f_dq, 'L': L, 'T': T}


def margins(r):
    """@return open loop: crossover (Hz, last |L| = 1), phase margin (°), gain margin (dB, first −180° above
    the crossover); closed loop: −3 dB bandwidth (Hz) and resonance peak (dB, Hz)."""
    f, L, T = r['f'], r['L'], r['T']; mag = np.abs(L); ph = np.degrees(np.unwrap(np.angle(L)))
    cross = np.where(np.diff(np.sign(mag - 1)) != 0)[0]
    out = {'crossover_Hz': math.nan, 'pm_deg': math.nan, 'gm_dB': math.inf}
    if len(cross):
        k = cross[-1]; out['crossover_Hz'] = float(f[k])
        out['pm_deg'] = float(((ph[k] + 180) + 180) % 360 - 180)
        wrapped = (ph + 180) % 360 - 180   # phase crossings of ±180° above the crossover
        j = [i for i in range(k, len(f) - 1) if abs(wrapped[i]) > 150 and np.sign(wrapped[i]) != np.sign(wrapped[i + 1])]
        if j: out['gm_dB'] = float(-20 * math.log10(mag[j[0]]))
    tm = np.abs(T); ref = tm[0]
    below = np.where(tm < ref / math.sqrt(2))[0]
    out['bandwidth_Hz'] = float(f[below[0]]) if len(below) else math.nan
    kpk = int(np.argmax(tm)); out['peak_dB'] = float(20 * math.log10(tm[kpk] / ref)); out['peak_Hz'] = float(f[kpk])
    return out
