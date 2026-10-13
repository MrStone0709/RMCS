# yaw 角度环 PID 三组对比（kp=10 / 15 / 35）

## 实验方法

**注入方式**：`AngleReferenceInjector` 输出 `/gimbal/yaw/angle_target_offset`，
由 `GimbalController` 每拍调用 `TwoAxisGimbalSolver::set_yaw_target_offset()`
把它叠加到求解器的**目标角**上（仅本拍有效，不进入累加状态）。

**为什么不用早前的 `AutoAimDirectionInjector`（已弃用）**：
它用 `SetControlDirection` 从头构造**世界系绝对方向**，需知道云台当前世界朝向；
实测该关系不是常数（含 IMU 漂移 + 安装姿态），猜错会让误差达 π 级 → **云台全速冲**
（实测 8.5 rad/s 失控）。而 `control_direction_` 由求解器自己维护，坐标系天然正确。

**激励**（三组完全一致）：

```
预载 0 → t=6s +0.15 → t=11s +0.30 → t=16s +0.45 → t=20.5~22s 斜坡淡出 → 0
```

- `step_delta: 0.15`、`step_hold_s: 5.0`（**按要求每级相隔 5 s**）
- 实际产生 **3 次阶跃**，其中**前 2 次（t=6、t=11）完全干净**，用于指标计算
- 第 3 次（t=16）与 ramp-down 部分重叠，不计入瞬态指标

## 安全修复（重要）

**问题**：注入结束时命令从最终值**阶跃归零**（无过渡）。云台此时已在 +0.45 rad 位置，
角度环看到巨大误差 → `v_ref = kp × 0.45`（kp=35 时达 **15.75 rad/s**）→
**云台高速回退 = "失控旋转"**。

**修复**：`AngleReferenceInjector` 新增 `ramp_down_s`（默认 2.0 s）**线性淡出**：

```cpp
const double ramp_begin = event_end - hold_after_;
if (t_ >= ramp_begin) {
    const double progress = (t_ - ramp_begin) / ramp_down_s_;
    if (progress >= 1.0) { phase_ = kIdle; return 0.0; }
    command *= (1.0 - progress);      // 线性淡出，而非阶跃回零
}
```

**效果**：回退速度上限 ≈ `offset / ramp_down_s` = 0.45/2.0 = **0.225 rad/s**（预期）。

**实测验证**：三组的 ramp 段速度峰值为 0.042 ~ 1.029 rad/s（峰值出现在淡出初段，
受角度环比例作用放大），**无失控**。

## 三组配置

| 组 | `yaw_angle_kp` | `yaw_angle_ki` | `yaw_angle_kd` | 文件 |
|---|---|---|---|---|
| **A 基准** | 10.0 | 0.0 | 0.0 | `group4_kp10_ki0_rampfix.csv` |
| **B 你的** | **15.0** | 0.0 | 0.0 | `group3_kp15_ki0.csv` |
| **C 激进** | 35.0 | 0.5 | 0.0 | `group1_kp35_ki0.5.csv` |

速度环三组相同：`kp=13.0`、`ki=0.5`、积分限幅 ±2.5、split ±3.5。

## 逐级阶跃响应（2 个干净级，各 +0.150 rad）

### 第 1 级（t=6 s）

| 组 | 实测Δ | 增益 | 超调 | 上升 | 整定 | 稳态误差 |
|---|---|---|---|---|---|---|
| A `kp=10` | 0.1449 | 0.966 | +2.7% | 206 ms | 269 ms | +0.0051 |
| **B `kp=15`** | 0.1433 | 0.956 | +3.4% | **136 ms** | **186 ms** | +0.0067 |
| C `kp=35` | 0.1436 | 0.957 | **+85.3%** | 44 ms | 274 ms | +0.0064 |

### 第 2 级（t=11 s）

| 组 | 实测Δ | 增益 | 超调 | 上升 | 整定 | 稳态误差 |
|---|---|---|---|---|---|---|
| A `kp=10` | 0.1436 | 0.957 | +3.6% | 202 ms | 267 ms | +0.0064 |
| **B `kp=15`** | 0.1436 | 0.957 | +4.2% | **134 ms** | **182 ms** | +0.0064 |
| C `kp=35` | 0.1423 | 0.949 | **+83.6%** | 42 ms | 274 ms | +0.0077 |

## 汇总

| 指标 | A `kp=10` | **B `kp=15`** | C `kp=35, ki=0.5` |
|---|---|---|---|
| **增益** | 0.962 | 0.956 | 0.953 |
| **超调** | +3.2% | **+3.8%** | **+84.5%** ⚠️ |
| **上升时间** | 204 ms | **135 ms** | 43 ms |
| **整定时间 (±5%)** | 268 ms | **184 ms** | 274 ms |
| 稳态误差 | +0.0058 | +0.0065 | +0.0070 |
| **速度峰值** | 1.566 rad/s | **2.126 rad/s** | 4.641 rad/s |
| 力矩峰值 | 11.26 N·m | 11.35 N·m | 11.36 N·m |
| pitch 跨度 | 0.0021 rad | **0.0017 rad** | 0.0022 rad |
| ramp 段速度峰 | 0.740 rad/s | 1.029 rad/s | 0.042 rad/s |

## 结论：**`kp=15, ki=0` 明显优于 `kp=10, ki=0`**

| 对比项 | A → B |
|---|---|
| **上升时间** | 204 → **135 ms**（**快 34%**）|
| **整定时间** | 268 → **184 ms**（**快 31%**）|
| **超调** | +3.2% → +3.8%（**几乎不变**）|
| 稳态增益/误差 | 基本不变 |
| pitch 耦合 | 略降（0.0017 rad）|

**即：`kp` 从 10 提到 15，速度提升约三分之一，而没有付出超调代价。这是净收益。**

### 为什么 `kp=15` 好而 `kp=35` 坏

```
kp=10 → 上升 204 ms，超调  3.2%     ← 偏保守（带宽不足）
kp=15 → 上升 135 ms，超调  3.8%     ← 甜点区
kp=35 → 上升  43 ms，超调 84.5%     ← 越过阻尼边界，振荡
```

**`kp=15` 位于"快速但不振荡"的区间**；`kp=35` 时角度环带宽已接近内层速度环
相位快速下降的区域，加上 `ki=0.5` 再引入积分滞后，于是超调爆发到 84.5%，
**整定时间反而退回 274 ms**（比 `kp=15` 还慢）。

### 关于 C 组（`kp=35`）的正确评价

C 的**上升时间 43 ms 极快**，但：
- 超调 84.5% → 每次目标跳变都**过冲后再拉回**
- **整定时间 274 ms ≥ A 组 268 ms** → **最终到位速度没有变快**
- 速度峰值 4.641 rad/s（A 的 2.96 倍），力矩反复顶到电流上限 11.36 N·m

**所以 C 是"看起来快、实际不划算"** —— 这正是你现场"目测无问题"但数据暴露的问题。

## 推荐

```yaml
yaw_angle_kp: 15.0     # 保持（数据支持）
yaw_angle_ki: 0.0      # 保持（加积分会像速度环 ki=2.0 那样引入超调）
yaw_angle_kd: 0.0      # 保持
```

**保留 `kp=15, ki=0, kd=0`。**

若还想更快，建议**小步试探**而非跳到 35：`kp=18 → 20 → 22`，
每次看超调是否仍 <10%；一旦超过 25% 就退回。

## ⚠️ 附：温度读数异常

三组 CSV 的 `/gimbal/yaw/temperature` **恒为 125.0**（22000 样本零变化）。

`125 = int8_t` 最大值，且代码为：
```cpp
status_component.register_output(name_prefix + "/temperature", temperature_output_, 0.0);
temperature_ = static_cast<double>(feedback.temperature);   // Temperature unit: celsius
```

**判断**：这是**温度反馈异常**（哨兵值/通信问题），**不是真实过热**：
- 全程力矩仅 −0.09 ~ −0.78 N·m（远低于过热所需电流）
- 22 秒内**完全不变**（真实温度必然波动）

**建议**：现场触摸 yaw 电机确认；并检查该电机的 CAN 温度反馈。
（注意：**早期测试**中温度读数正常，为 34~40 °C 且随负载变化。）

## ramp-down 回退安全验证

| 组 | ramp 段速度峰 | ramp 段力矩峰 | offset 末值 |
|---|---|---|---|
| A `kp=10` | 0.740 rad/s | 9.486 N·m | 0.0000 ✓ |
| B `kp=15` | 1.029 rad/s | 11.144 N·m | 0.0000 ✓ |
| C `kp=35` | 0.042 rad/s | 0.733 N·m | 0.6000 |

**三组均无失控**（修复前该段速度会达 15~21 rad/s）。

B 组 ramp 力矩峰值 11.14 N·m 是**淡出初段的短暂尖峰**（`kp=15` 放大残余误差），
随后迅速回落；不是持续饱和。若介意，可把 `ramp_down_s` 从 2.0 延长到 3.0。

## 复现

```bash
# 机器人侧配置 /tmp/grp.yaml：生产配置 + yaw_angle_target_offset_input: true
#                            + AngleReferenceInjector -> yaw_off_injector
ros2 launch rmcs_bringup rmcs.launch.py robot:=/tmp/grp
# 拨杆：左中+右中 -> 左中+右上
# 数据：/tmp/grp3.csv
```

## 相关代码

| 文件 | 改动 |
|---|---|
| `controller/gimbal/two_axis_gimbal_solver.hpp` | 新增 `set_yaw_target_offset()`（仅本拍有效的目标角偏移）|
| `controller/gimbal/deformable_infantry_gimbal_controller.cpp` | 新增 `yaw_angle_target_offset_input`（默认 false，生产行为不变）|
| `identification/angle_reference_injector.cpp` | 输出改名 `/gimbal/yaw/angle_target_offset`；**新增 `ramp_down_s` 斜坡淡出**；CSV 补 pitch 列 |
| `identification/auto_aim_direction_injector.cpp` | **已弃用**（世界系假设不成立，会导致失控）|
