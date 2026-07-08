# 观测器 Debug 调试指南 — Vofa 监控变量 & 推荐值

> 配套代码: FOC_ST_V2_0611 | 更新日期: 2026-07-08
> **观测器当前支持电压型 / Hybrid Active Flux 双模式** (`OBSERVER_TYPE`)

---

## 1. Vofa 上位机 9 通道说明

`HFPeriod_RUN` 上报的 9 个 float 通道 (`taskManager/taskManager.c`):

| 通道 | 变量 | 含义 | **正常时的值** | **异常表现** |
|------|------|------|---------------|-------------|
| CH1 | `motor.CurrentHz` | 当前电频率 (Hz) | 跟踪目标 ±1Hz | 大幅跳动 → 失步 |
| CH2 | `PARK_PCurr.Ds` | d 轴电流 Id (A) | **≈ 0** | 偏离 0 → 角度/参数错 |
| CH3 | `PARK_PCurr.Qs` | q 轴电流 Iq (A) | 稳定，正比于负载 | 剧烈波动 → 电流环震荡 |
| CH4 | `motor.V_d` | d 轴电压 (pu) | ≈ 0 | 异常偏大 → 角度误差 |
| CH5 | `motor.V_q` | q 轴电压 (pu) | < 0.8 | 饱和(≈1.0) → 电压不足 |
| CH6 | `BUS_Voltage` | 母线电压 (V) | ≈ 24V 稳定 | 大幅跌落 → 电源不足 |
| CH7 | `PhaseU_Curr` | U 相电流 (A) | 正弦 | 畸变 → SVPWM 问题 |
| CH8 | `PhaseV_Curr` | V 相电流 (A) | 正弦 | 同上 |
| CH9 | `PhaseW_Curr` | W 相电流 (A) | 正弦 | 同上 |

### Vofa 配置

- 协议: JustFloat
- 通道数: 9
- 帧尾: `00 00 80 7F`
- 帧率: ~1kHz

---

## 2. LCD 屏幕显示

Task_DEBUG 页面观测器诊断行 (`taskManager/taskManager.c`):

| 标签 | 含义 | **正常值** |
|------|------|-----------|
| `FHz` | 当前频率 | = TargetHz |
| `PLL` | PLL 角度误差 | **≈ 0.000** |
| `Est` | 观测器估算频率(Hz) (Mode 2/4) | ≈ CurrentHz |
| `Psi` | 转子磁链幅值 | **≈ 0.0075** |

---

## 3. Debug 调参变量 & 推荐值

### 3.1 观测器参数

| 参数 | 位置 | 电压型默认 | Hybrid 默认 | 调大效果 | 调小效果 |
|------|------|-----------|------------|---------|---------|
| `PLL_kp` | `flux.c:Flux_Observer_Init` | 20.0f | 自动计算 (2·ω_pll) | PLL 跟踪更快，但噪声敏感 | 跟踪慢，滞后大 |
| `PLL_ki` | `flux.c:Flux_Observer_Init` | 10.0f | 自动计算 (ω_pll²·Ts) | 更快消除静差，但易震荡 | 静差消除慢 |
| `Gain` | `flux.c:Flux_Observer_Init` | 5000.0f | 5000.0f | 磁链收敛更快，但过大会震荡 | 收敛慢 |
| `V_DEADTIME` | `flux.c:Deadtime_Comp_ab` | 0.3f | 0.3f | — | — |
| `OBSERVER_TYPE` | `flux.h` | 0 (电压型) | 1 (Hybrid) | — | — |
| `HYBRID_PLL_BW_HZ` | `flux.h` | — | 50Hz | 更快响应 | 更平滑 |

### 3.2 电机参数（**必须确认**）

| 参数 | 位置 | 当前值 | 获取方式 |
|------|------|--------|---------|
| `MOTOR_RS` | `flux.h` | 0.007525Ω | **万用表实测** 任意两相电阻 ÷ 2 |
| `MOTOR_LS` | `flux.h` | 0.00002H (20µH) | LCR 表测 1kHz, 任意两相电感 ÷ 2 ⚠️ 待验证 |
| `MOTOR_FLUX` | `flux.h` | 0.00752Wb | 反拖电机测线间反电动势: `Flux = Vpeak / (2π × f × √3 × 极对数)` |
| `MOTOR_POLES` | `flux.h` | 52 | 数磁钢个数 |

---

## 4. Step-by-Step 调参流程

### 阶段 A：确认 VF 模式正常

```
1. 上电, KEY1 启动 → 自动进入 Mode 3 (VF)
2. KEY3/KEY4 设 TargetHz = 20Hz, targetVolt ≥ 0.10
3. Vofa 观察 CH7/CH8/CH9 (三相电流) → 电流应该正弦
4. 电机应平稳旋转，无异响
```

**如果 VF 都跑不稳** → 先排查硬件（MOS 驱动、死区时间、电流采样）。

---

### 阶段 B：Mode 4 预热观测器

```
1. KEY2 切到 Mode 4 ([PREPOS])
2. Vofa 重点关注:
   ┌─ LCD "Psi": 应从 0 慢慢涨到 ~0.0075
   ├─ LCD "PLL": 应逐步收敛到 ~0.000
   └─ CH1 (CurrentHz): 应稳定
3. 观测器在 20~30Hz 下通常 100~500ms 收敛
```

**判断观测器是否就绪**：
- LCD "Psi" ≥ 0.005（约 67% 参考值）
- LCD "PLL" 稳定在 ±0.05 以内

---

### 阶段 C：手动切换到 Mode 2

```
1. 确认观测器就绪后 KEY2 切 Mode 2 ([SENSORLESS])
2. 切换瞬间重点观察:
   ┌─ CH3 (Iq): 是否跳变？
   ├─ CH2 (Id): 是否偏离 0？
   └─ CH5 (Vq): 是否饱和？
3. 切换后 CH1 (CurrentHz) 应稳定跟踪
```

---

### 阶段 D：如果切换失败（爆鸣/电流突变）

**按顺序排查**：

| 现象 | 最可能原因 | 调什么 |
|------|-----------|--------|
| LCD "Psi" 一直是 0 | 观测器根本没跑起来 | 确认进入了 Mode 4 |
| LCD "Psi" 涨到某个值就不动了 | 电机参数错误 | **先确认 MOTOR_FLUX**，再用 LCR 实测 MOTOR_LS |
| LCD "PLL" 大幅震荡 | PLL 增益太高 | 降低 PLL_kp → 10, PLL_ki → 5 |
| CH3(Iq) 切换瞬间跳 10A+ | 电流环响应跟不上 | 提高 pi_iq Kp → 0.05, Ki → 0.0001 |
| CH2(Id) 切换后不是 0 | 角度误差大 | 等更久预热，或提高 VF 频率到 30~40Hz |
| 切换瞬间正常，几秒后爆鸣 | PLL 积分 windup | 检查 anti-windup 是否生效 |

---

## 5. 快速诊断表（现场调参用）

在 LCD 上看这 3 个值就能快速判断：

| PLL | Psi | Est vs FHz | 诊断 |
|-----|-----|-----------|------|
| ≈ 0 | ≈ 0.0075 | 一致 | ✅ 正常 |
| 大幅震荡 | 也在震荡 | 乱跳 | PLL 震荡 → 降 PLL_kp |
| 持续偏大 | 正常 | 不一致 | PLL 滞后 → 升 PLL_kp |
| ≈ 0 | 远小于 0.0075 | 一致 | 电机参数错 → 检查 FLUX |
| ≈ 0 | 正常 | 差 2Hz 以上 | 电机参数错 → 检查 POLES |
| 逐渐变大 | 正常 | 逐渐偏离 | PLL 积分 windup → 已修复(anti-windup) |

---

## 6. 关键修改一览

| 日期 | 文件 | 修改内容 |
|------|------|---------|
| 6.25 | `flux.c` | Observer_Run 重构：磁链误差同周期计算(消延迟) + PLL 抗饱和 + 死区补偿 + PLL 增益自动 ramp |
| 6.25 | `flux.c` | Gain 15000→5000（降低震荡风险） |
| 6.25 | `flux.h` | 新增 `PLL_kp_target`, `PLL_ki_target`, `pll_ramp_active` |
| 6.27 | `motorControl.c` | VF 分离架构: Mode 4 只预热不自动切换 + Mode 2 暖启动检测 |
| 6.27 | `taskManager.c` | 按键循环重排: STOP→VF→PREPOS→SENSORLESS |
| 7.05 | `ADC_Sample.c` | 电流采样负号修复 |
| 7.05 | `PI_Cale.c` | PI 限幅修正 |
| 7.08 | `flux.c/h` | 新增 Hybrid Active Flux 观测器: 二阶系统带宽设计 + 反馈补偿 PI + SMO 备用 |
| 7.08 | `taskManager.c` | Vofa 6ch→9ch, LCD 显示 PLL/Est/Psi |
