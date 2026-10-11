#!/usr/bin/env python3
"""Summarize graybox fit results across the 5 yaw sweep runs."""
import statistics as st

data = {
    "run1": (0.032302, 0.344620, 0.290730, 0.285056, 0.578060, 134.69),
    "run2": (0.032584, 0.317085, 0.308838, 0.274348, 0.583270, 142.59),
    "run3": (0.030014, 0.308628, 0.314620, 0.265635, 0.569428, 148.07),
    "run4": (0.088588, 0.486881, 0.217107, 0.038771, 0.739495, 128.46),
    "run5": (0.107894, 0.623533, 0.145097, 0.129045, 0.797536, 144.30),
}
names = ["J", "B", "Fc", "G", "R^2", "span_deg"]

hdr = f"{'run':6}{'J':>10}{'B':>10}{'Fc':>10}{'G':>10}{'R^2':>9}{'span':>9}"
print(hdr)
print("-" * len(hdr))
for k, v in data.items():
    print(f"{k:6}{v[0]:10.5f}{v[1]:10.5f}{v[2]:10.5f}{v[3]:10.5f}{v[4]:9.4f}{v[5]:9.1f}")

print()
print("=== 全部 5 组统计 ===")
for i, c in enumerate(names):
    vals = [v[i] for v in data.values()]
    m = st.mean(vals)
    s = st.stdev(vals) if len(vals) > 1 else 0.0
    cv = 100 * s / abs(m) if m else 0.0
    print(f"  {c:9}: mean={m:10.5f}  std={s:9.5f}  CV={cv:5.1f}%")

print()
print("=== 仅用拟合较好两组 (run4,run5, R^2>0.73) ===")
sub = [data["run4"], data["run5"]]
for i, c in enumerate(names):
    vals = [v[i] for v in sub]
    print(f"  {c:9}: mean={st.mean(vals):10.5f}")

print()
print("=== 与频域方法对比 ===")
print(f"{'method':34}{'J':>10}{'B':>10}")
print(f"{'频域复最小二乘 0.3-5Hz (我)':34}{0.124251:10.5f}{2.250094:10.5f}")
print(f"{'灰盒 linear 全部5组均值':34}{st.mean([v[0] for v in data.values()]):10.5f}"
      f"{st.mean([v[1] for v in data.values()]):10.5f}")
print(f"{'灰盒 linear run4+run5':34}{st.mean([v[0] for v in sub]):10.5f}"
      f"{st.mean([v[1] for v in sub]):10.5f}")
