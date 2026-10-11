#!/usr/bin/env python3
"""Closed-loop FRF of the yaw velocity loop from a reference-injection sweep.

Input : velocity_reference (rad/s)     Output: /gimbal/yaw/velocity_imu (rad/s)
Pure Python iterative FFT (no numpy on this host).
"""
import csv
import cmath
import math
import sys

CSV = sys.argv[1] if len(sys.argv) > 1 else "vel_sweep.csv"
FS = 1000.0
NFFT = 4096
SKIP = 1.0


def fft(a):
    """In-place iterative radix-2 FFT."""
    n = len(a)
    j = 0
    for i in range(1, n):
        bit = n >> 1
        while j & bit:
            j ^= bit
            bit >>= 1
        j |= bit
        if i < j:
            a[i], a[j] = a[j], a[i]
    length = 2
    while length <= n:
        ang = -2j * math.pi / length
        wl = cmath.exp(ang)
        for i in range(0, n, length):
            w = 1 + 0j
            for k in range(i, i + length // 2):
                u = a[k]
                v = a[k + length // 2] * w
                a[k] = u + v
                a[k + length // 2] = u - v
                w *= wl
        length <<= 1
    return a


def load(path):
    rows = list(csv.DictReader(open(path, encoding="utf-8-sig")))
    g = lambda k: [float(r[k]) for r in rows]
    return g("elapsed_s"), g("velocity_reference"), g("/gimbal/yaw/velocity_imu"), \
           g("/gimbal/yaw/torque"), g("/gimbal/yaw/temperature")


def welch(u, y, fs, nfft):
    win = [0.5 - 0.5 * math.cos(2 * math.pi * i / (nfft - 1)) for i in range(nfft)]
    wp = sum(w * w for w in win)
    nb = nfft // 2 + 1
    suu = [0 + 0j] * nb
    suy = [0 + 0j] * nb
    syy = [0 + 0j] * nb
    nseg = 0
    step = nfft // 2
    for s0 in range(0, len(u) - nfft, step):
        us, ys = u[s0:s0 + nfft], y[s0:s0 + nfft]
        mu, my = sum(us) / nfft, sum(ys) / nfft
        U = fft([complex((us[i] - mu) * win[i], 0.0) for i in range(nfft)])
        Y = fft([complex((ys[i] - my) * win[i], 0.0) for i in range(nfft)])
        for k in range(nb):
            suu[k] += (U[k].conjugate() * U[k])
            suy[k] += (U[k].conjugate() * Y[k])
            syy[k] += (Y[k].conjugate() * Y[k])
        nseg += 1
    f = [k * fs / nfft for k in range(nb)]
    H, coh = [], []
    for k in range(nb):
        a = suu[k] / (nseg * wp)
        b = suy[k] / (nseg * wp)
        c = syy[k] / (nseg * wp)
        H.append(b / a if abs(a) > 1e-300 else complex(float("nan"), float("nan")))
        coh.append(abs(b) ** 2 / (abs(a) * abs(c)) if abs(a) * abs(c) > 1e-300 else 0.0)
    return f, H, coh, nseg


t, r, v, tau, temp = load(CSV)
print(f"样本 {len(t)}  时长 {t[-1]:.2f}s   窗长 {NFFT/FS:.2f}s")
print(f"yaw 温度 {temp[0]:.0f} -> {temp[-1]:.0f} C (峰值 {max(temp):.0f})")
n0 = int(SKIP * FS)
f, H, coh, nseg = welch(r[n0:], v[n0:], FS, NFFT)
print(f"平均段数 {nseg}   频率分辨率 {FS/NFFT:.4f} Hz")
print()
print("=== 闭环频响（参考 -> 实际速度）===")
print(f"{'f[Hz]':>8}{'|T|':>9}{'|T|[dB]':>10}{'相位[deg]':>11}{'相干':>8}")
sel = []
for tgt in [0.5, 0.8, 1, 1.5, 2, 3, 4, 5, 7, 9, 11, 13, 15, 18, 21, 25, 30]:
    i = min(range(len(f)), key=lambda k: abs(f[k] - tgt))
    if coh[i] < 0.3:
        print(f"{f[i]:8.2f}{'  --':>9}{'  --':>10}{'  --':>11}{coh[i]:8.3f}")
        continue
    db = 20 * math.log10(abs(H[i])) if abs(H[i]) > 0 else -99
    print(f"{f[i]:8.2f}{abs(H[i]):9.4f}{db:10.2f}{math.degrees(cmath.phase(H[i])):11.1f}{coh[i]:8.3f}")
    sel.append((f[i], abs(H[i]), coh[i]))

print()
if sel:
    base = sel[0][1]
    tgt = base / math.sqrt(2)
    bw = None
    for i in range(len(f) - 1):
        if f[i] < 1.0 or coh[i] < 0.3:
            continue
        if abs(H[i]) >= tgt > abs(H[i + 1]):
            bw = f[i]
            break
    print(f"低频增益 ≈ {base:.4f} (≈{20*math.log10(base):.2f} dB)")
    print(f"-3 dB 带宽 ≈ {bw:.2f} Hz" if bw else "-3 dB 带宽：未在相干带内找到（高频超出相干范围）")
good = [f[i] for i in range(len(f)) if coh[i] > 0.5 and f[i] > 0.3]
if good:
    print(f"相干 >0.5 频段: {min(good):.2f} ~ {max(good):.2f} Hz")
