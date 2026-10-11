#!/usr/bin/env python3
"""Sliding-band FRF fit: find the frequency band where J,B are stable.

If B drifts with the fitting band, the data does not support a single
linear 2nd-order model (nonlinearity / friction or unmodeled lag).
"""
import csv
import math
import sys

import numpy as np

FS = 1000.0
NFFT = 8192


def load(path):
    rows = list(csv.DictReader(open(path, encoding="utf-8-sig")))
    g = lambda k: np.array([float(r[k]) for r in rows])
    return g("/gimbal/yaw/control_torque"), g("/gimbal/yaw/velocity")


def welch(u, y, fs, nfft):
    step, win = nfft // 2, np.hanning(nfft)
    wp = np.sum(win**2)
    suu = suy = syy = None
    n = 0
    for s in range(0, len(u) - nfft, step):
        us = (u[s:s+nfft] - u[s:s+nfft].mean()) * win
        ys = (y[s:s+nfft] - y[s:s+nfft].mean()) * win
        U, Y = np.fft.rfft(us), np.fft.rfft(ys)
        a, b, c = np.abs(U)**2, np.conj(U)*Y, np.abs(Y)**2
        if suu is None: suu, suy, syy = a, b, c
        else: suu += a; suy += b; syy += c
        n += 1
    suu /= n*wp; suy /= n*wp; syy /= n*wp
    with np.errstate(divide="ignore", invalid="ignore"):
        return (np.fft.rfftfreq(nfft, 1/fs), suy/suu, np.abs(suy)**2/(suu*syy))


def fit_band(f, H, coh, lo, hi):
    band = (f >= lo) & (f <= hi) & (coh > 0.5) & np.isfinite(H)
    idx = np.where(band)[0]
    if len(idx) < 2:
        return None
    w = 2*math.pi*f[idx]
    Yv = 1.0/H[idx]
    A = np.vstack([np.column_stack([np.ones_like(w), np.zeros_like(w)]),
                   np.column_stack([np.zeros_like(w), w])])
    b = np.concatenate([Yv.real, Yv.imag])
    sol, *_ = np.linalg.lstsq(A, b, rcond=None)
    B, J = float(sol[0]), float(sol[1])
    pred = 1.0/(B + 1j*J*w)
    res = np.median(np.abs(pred - H[idx])/np.abs(H[idx]))
    return B, J, res, len(idx)


runs = sys.argv[1:]
print(f"分析 {len(runs)} 组数据，滑动频段扫描\n")
print(f"{'band[Hz]':>14} | " + " | ".join(f"{'B':>8}{'J':>9}" for _ in runs) + " | 残差均值")
print("-" * (16 + 20*len(runs) + 10))

bands = [(lo, lo+0.5) for lo in [0.3,0.5,0.8,1.0,1.5,2.0,3.0,4.0,6.0,8.0]]
accum = {b: [] for b in bands}
for lo, hi in bands:
    row = []
    res_list = []
    for p in runs:
        u, y = load(p)
        n0 = int(1.0*FS)
        f, H, coh = welch(u[n0:], y[n0:], FS, NFFT)
        r = fit_band(f, H, coh, lo, hi)
        if r is None:
            row.append(f"{'--':>8}{'--':>9}")
        else:
            B, J, res, n = r
            row.append(f"{B:8.3f}{J:9.4f}")
            res_list.append(res)
            accum[(lo,hi)].append((B, J))
    rstr = f"{np.mean(res_list)*100:6.1f}%" if res_list else "   n/a"
    print(f"{lo:6.2f}-{hi:<6.2f} | " + " | ".join(row) + f" | {rstr}")

print()
print("=== 各频段的 B 均值与离散度（找最稳定的频段）===")
for b in bands:
    v = accum[b]
    if len(v) < 2:
        continue
    Bs = np.array([x[0] for x in v])
    Js = np.array([x[1] for x in v])
    cvB = 100*Bs.std(ddof=1)/abs(Bs.mean())
    cvJ = 100*Js.std(ddof=1)/abs(Js.mean()) if Js.mean() else float('inf')
    print(f"  {b[0]:5.2f}-{b[1]:<5.2f} Hz: B={Bs.mean():7.3f} (CV {cvB:5.1f}%)   "
          f"J={Js.mean():8.5f} (CV {cvJ:5.1f}%)")
