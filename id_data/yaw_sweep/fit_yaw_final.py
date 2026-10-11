#!/usr/bin/env python3
"""Final yaw plant fit: coherence-weighted, low-band (where the model holds).

Reports:
  - per-run B, J from a low coherent band
  - aggregate with statistics
  - comparison against user's reference B ~= 0.38
"""
import csv
import math
import sys

import numpy as np

FS, NFFT = 1000.0, 8192


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


def fit(f, H, coh, lo, hi, wcoh=True):
    m = (f >= lo) & (f <= hi) & (coh > 0.5) & np.isfinite(H)
    idx = np.where(m)[0]
    if len(idx) < 2:
        return None
    w = 2*math.pi*f[idx]
    Yv = 1.0/H[idx]
    wt = coh[idx]**2 if wcoh else np.ones_like(w)
    sw = np.sqrt(wt)
    A = np.vstack([np.column_stack([np.ones_like(w), np.zeros_like(w)]),
                   np.column_stack([np.zeros_like(w), w])])
    b = np.concatenate([Yv.real, Yv.imag])
    sw2 = np.concatenate([sw, sw])
    sol, *_ = np.linalg.lstsq(A*sw2[:, None], b*sw2, rcond=None)
    B, J = float(sol[0]), float(sol[1])
    pred = 1.0/(B + 1j*J*w)
    res = np.median(np.abs(pred - H[idx])/np.abs(H[idx]))
    return B, J, res, len(idx)


runs = sys.argv[1:]
LO, HI = 0.3, 3.0

print(f"=== yaw 被控对象辨识（相干加权最小二乘，{LO}-{HI} Hz）===")
print(f"{'run':8}{'B':>10}{'J':>11}{'残差':>9}{'点数':>7}")
print("-"*46)
Bs, Js = [], []
for p in runs:
    u, y = load(p)
    n0 = int(1.0*FS)
    f, H, coh = welch(u[n0:], y[n0:], FS, NFFT)
    r = fit(f, H, coh, LO, HI)
    if r is None:
        print(f"{p:8}  拟合失败")
        continue
    B, J, res, n = r
    Bs.append(B); Js.append(J)
    print(f"{p.split('/')[-1]:8}{B:10.4f}{J:11.5f}{res*100:8.1f}%{n:7d}")

Bs, Js = np.array(Bs), np.array(Js)
print()
print("=== 汇总 ===")
print(f"  B = {Bs.mean():.4f} +/- {Bs.std(ddof=1):.4f} N*m*s/rad  (CV {100*Bs.std(ddof=1)/Bs.mean():.1f}%)")
print(f"  J = {Js.mean():.5f} +/- {Js.std(ddof=1):.5f} kg*m^2     (CV {100*Js.std(ddof=1)/Js.mean():.1f}%)")
print()
print(f"  模型 G(s) = 1 / ({Js.mean():.5f}*s + {Bs.mean():.4f})")
print(f"  转折频率 = {Bs.mean()/(2*math.pi*Js.mean()):.2f} Hz")
print(f"  时间常数 = {Js.mean()/Bs.mean()*1000:.2f} ms")
print()
print("=== 与你的参考值对比 ===")
print(f"  你的参考 B ≈ 0.38   本辨识 B = {Bs.mean():.3f}   差 {Bs.mean()/0.38:.2f} 倍")

# control implications
J, B = Js.mean(), Bs.mean()
for kp in [13.0, 25.0, 40.0, 60.0]:
    wc = kp/B
    fc = wc/(2*math.pi)
    pm = 90 - math.degrees(math.atan(wc*J/B))
    print(f"  kp={kp:5.1f} -> 穿越 {fc:6.2f} Hz, 相位裕度 {pm:5.1f} deg")
