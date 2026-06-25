# FOC_ST_V2 观测器修复变更记录

> 日期: 2025-06-25 | 版本: V2_0611 → hotfix

---

## 当前状态 (6.25)

| 现象 | 状态 |
|------|------|
| 电机爆鸣 | ✅ **大幅减少** |
| 电机转动 | ❌ **不转，手无法旋转（DC 锁死）** |

**根因**: Mode 2 冷启动检测每个 PWM 周期都复位 PLL 角度到 VF 的 OpenTheta（固定值），导致角度无法累加 → 电压矢量不动 → 电机被直流锁死。

**已修复**: 改为仅在**刚进入 Mode 2 的第一个周期**同步一次，之后让 PLL 自由运行。

---

## 修改文件清单

### 1. `Flux/inc/flux.h` — 结构体扩展

```diff
+    float PLL_kp_target;   // 正常 Kp 目标值
+    float PLL_ki_target;   // 正常 Ki 目标值
+    uint8_t pll_ramp_active; // 1=正在恢复增益
```

### 2. `Flux/src/flux.c` — 观测器核心算法重构

| 修改项 | 旧值/旧逻辑 | 新值/新逻辑 | 理由 |
|--------|------------|------------|------|
| 磁链误差计算顺序 | 先补偿(用旧Err)→后算Err | **先算转子磁链和Err→再用同周期Err补偿** | 消除一周期反馈延迟，防止震荡 |
| PLL 积分抗饱和 | 无 | **`Limit_Sat(PLL_Interg, ±2π×500Hz×Ts)`** | 防止积分 windup 导致角度飞转 |
| PLL 输出限幅 | 无 | **`Limit_Sat(PLL_Ui, ±2π×500Hz×Ts)`** | 双重保护 |
| 死区电压补偿 | 无 | **新增 `Deadtime_Comp_ab()`** | 低频时命令电压≠真实电压，补偿后磁链估算更准 |
| 角度归一化 | `if/else if` | **`while` 循环** | 防止单次超 2π 时归一化不彻底 |
| Gain (磁链补偿) | 15000 | **5000** | 降低震荡啸叫风险 |
| PLL 增益恢复 | 无 | **ramp 机制：每周期向目标靠近 2%** | 切换过渡后自动恢复到正常增益 |

### 3. `UserSoftware/src/motorControl.c` — 切换逻辑修复

| 修改项 | 旧值/旧逻辑 | 新值/新逻辑 | 理由 |
|--------|------------|------------|------|
| `VF_TO_SENSORLESS_HZ` | 5.0f | **10.0f** | 反电动势更大，观测器更容易收敛 |
| PLL 增益切换后 | 永久降为 1.5/0.08 | **降为 4.0/1.0 + 激活自动 ramp** | 过渡平滑且能自动恢复 |
| PI 预加载 | `pi_iq.i1 = V_amp` (100%) | **`pi_iq.i1 = V_amp * 0.5f`** (保守 50%) | 减少切换瞬间电流冲击 |
| Mode 2 冷启动 | 无保护 | **检测磁链未就绪 → 同步 PLL 一次** | 防止按键直接切 Mode 2 炸机 |
| Mode 切换边沿检测 | 无 | **`static prev_mode` + `mode_just_entered`** | 冷启动同步只在进入时执行一次 |

### 4. `taskManager/taskManager.c` — Debug 遥测升级

**Vofa HFPeriod_RUN 通道变更**:

| CH | 旧变量 | 新变量 | 用途 |
|----|--------|--------|------|
| 1 | `motor.OpenTheta` | **`Foc_observer.PLL_Err`** | PLL 角度误差，正常≈0 |
| 2 | `motor.IQAngle` | **`sqrt(ψrα²+ψrβ²)`** | 转子磁链幅值，正常≈0.0075 |
| 3 | `BUS_Voltage` | **`speed_hz` / `CurrentHz`** | 估算频率 vs 设定频率 |
| 4 | `PhaseU_Curr` | **`PARK_PCurr.Ds`** | d 轴电流 Id，正常≈0 |
| 5 | `PhaseV_Curr` | **`PARK_PCurr.Qs`** | q 轴电流 Iq |
| 6 | `PhaseW_Curr` | **`BUS_Voltage`** | 母线电压 |

**LCD Task_DEBUG 新增行**:

| 行 | 标签 | 内容 | 正常值 |
|----|------|------|--------|
| y=78 | `PLL` | PLL 角度误差 | ≈ 0.000 |
| y=96 | `Est` | 观测器估算频率(Mode2/4) | ≈ 设定频率 |
| y=114 | `Psi` | 转子磁链幅值 | ≈ 0.0075 |

---

## 修改前后对比

### 修复前的问题链路

```
按键 VF(3) → SENSORLESS(2)
  → 观测器从未运行 (FluxR=0, Theta=0, PLL=0)
  → 每周期: Theta复位→Angel_Get前进一步→下周期又复位  ← BUG!
  → 角度几乎不动 → 电压矢量固定 → 电机 DC 锁死
  → 手转不动, 电流持续流过同一相 → 发热/烧管风险
```

### 修复后的流程

```
按键 VF(3) → SENSORLESS(2)
  → 检测 mode_just_entered = true
  → 磁链 < 30% 参考值 → 冷启动同步 (仅一次!)
    ├─ Theta = OpenTheta (VF 当前角度)
    ├─ PLL_Ui = 2π × CurrentHz × Ts
    ├─ PLL_Interg = PLL_Ui (预加载)
    ├─ PLL_kp/ki = 4/1 (过渡), target = 20/10
    └─ pll_ramp_active = 1
  → 后续周期: Observer_Run 自由运行
    ├─ PLL 从 VF 角度起步跟踪磁链
    ├─ 磁链从 0 开始自然收敛
    ├─ PLL 增益自动 ramp 到 20/10
    └─ 角度正常累加 → 电机旋转
```

### 推荐使用路径 (Mode 4 预热)

```
STOP → VF(3) → PREPOS(4) → 自动 SENSORLESS(2)
              ↑               ↑
          VF 启动并升频    后台预热观测器, 10Hz 自动切换
                          (FluxR 已收敛, PLL 已锁定)
```

Mode 4 路径下观测器有充分预热时间，切换最平滑。Mode 2 直接进入现在也能工作（有冷启动保护），但不推荐。

---

## 已知遗留问题

1. **死区补偿 `V_DEADTIME=0.3f` 未实际标定** — 如果补偿方向反了或幅度不对，低频时电压可能有偏差。建议在 VF 模式下对比命令电压和实际电机电流来判断。
2. **电机参数 `MOTOR_RS/LS/FLUX` 需实测确认** — 当前值是代码中硬编码的，可能与实际电机不符。
3. **PI 参数 (`pi_id/pi_iq/pi_spd` Kp/Ki) 未经调优** — 当前是保守值，需要根据实际电机惯量和负载整定。

---

## Debug 速查

| Vofa 通道 | 正常值 | 当前实测？ |
|-----------|--------|-----------|
| CH1 PLL_Err | 0 ± 0.05 | |
| CH2 FluxR_mag | → 0.0075 | |
| CH3 speed_hz | = 设定频率 | |
| CH4 Id | ≈ 0 | |
| CH5 Iq | 稳定 | |
| CH6 BUS_V | ≈ 24V | |

详细调参指南见 `Observer_Debug_Guide.md`。
