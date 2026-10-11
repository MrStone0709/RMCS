#!/usr/bin/env python3
"""Report effective (J,B) over several candidate fit bands.

Because B is frequency/amplitude dependent, the 'right' number depends on the
band that matters for control. Print a table so the choice is explicit.
"""
import csv
import math
import sys

import numpy as np

FS, NFFT = 1000.0, 8192


def load(p):
    rows = list(csv.DictReader(open(p, encoding="utf-8-sig")))
    g = lambda k: np.array([float(r[k]) for r in rows])
    return g("/gimbal/yaw/control_torque"), g("/gimbal/yaw/velocity")


def welch(u, y, fs, nfft):
    step, win = nfft//2, np.hanning(nfft)
    wp = np.sum(win**2)
    suu = suy = syy = None
    n = 0
    for s in range(0, len(u)-nfft, step):
        us = (u[s:s+nfft]-u[s:s+nfft].mean())*win
        ys = (y[s:s+nfft]-y[s:s+nfft].mean())*win
        U, Y = np.fft.rfft(us), np.fft.rfft(ys)
        a, b, c = np.abs(U)**2, np.conj(U)*Y, np.abs(Y)**2
        if suu is None: suu, suy, syy = a, b, c
        else: suu += a; suy += b; syy += c
        n += 1
    suu /= n*wp; suy /= n*wp; syy /= n*wp
    with np.errstate(divide="ignore", invalid="ignore"):
        return np.fft.rfftfreq(nfft, 1/fs), suy/suu, np.abs(suy)**2/(suu*syy)


def fit(f, H, coh, lo, hi):
    m = (f >= lo) & (f <= hi) & (coh > 0.4) & np.isfinite(H)
    idx = np.where(m)[0]
    if len(idx) < 3: return None
    w = 2*math.pi*f[idx]
    Yv = 1.0/H[idx]
    sw = np.sqrt(coh[idx]**2)
    A = np.vstack([np.column_stack([np.ones_like(w), np.zeros_like(w)]),
                   np.column_stack([np.zeros_like(w), w])])
    b = np.concatenate([Yv.real, Yv.imag])
    sw2 = np.concatenate([sw, sw])
    sol, *_ = np.linalg.lstsq(A*sw2[:, None], b*sw2, rcond=None)
    B, J = float(sol[0]), float(sol[1])
    pred = 1.0/(B + 1j*J*w)
    res = np.median(np.abs(pred-H[idx])/np.abs(H[idx]))
    return B, J, res


bands = [(0.3, 1.0), (0.3, 1.5), (0.3, 2.0), (0.3, 3.0), (0.5, 2.0), (1.0, 3.0)]
runs = sys.argv[1:]
print(f"{'band[Hz]':>12} | " + " | ".join(f"{'B':>8}{'J':>9}" for _ in runs) + " | 残差")
print("-"*(14+20*len(runs)+8))
for lo, hi in bands:
    row = []
    res = []
    for p in runs:
        u, y = load(p)
        n0 = int(1.0*FS)
        f, H, coh = welch(u[n0:], y[n0:], FS, NFFT)
        r = fit(f, H, coh, lo, hi)
        if r is None:
            row.append(f"{'--':>8}{'--':>9}")
        else:
            row.append(f"{r[0]:8.3f}{r[1]:9.5f}")
            res.append(r[2])
    print(f"{lo:5.2f}-{hi:<6.2f} | " + " | ".join(row)
          + f" | {np.mean(res)*100:5.1f}%")
