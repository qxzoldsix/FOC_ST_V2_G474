# FOC_ST_V2 观测器修复变更记录

> 日期: 2025-06-27 | 版本: V2_0611 hotfix v2

---

## 当前状态 (6.27)

| 现象 | 状态 |
|------|------|
| 电机爆鸣 | ❌ **仍存在** |
| 切换瞬间大电流 | ❌ **仍存在** |
| VF→观测器切换 | ❌ **不可靠** |

**本次修改策略**: VF 分离 —— Mode 4 只预热不自动切，用户确认观测器收敛后再手动切 Mode 2。

---

## 6.27 变更：VF 分离调试架构

### 核心思路

之前 Mode 4 到了 `VF_TO_SENSORLESS_HZ`(10Hz) 就**自动切** Mode 2。但 10Hz 电频率下反电动势只有：

```
V_BEMF = 2π × 10Hz × 0.00752Wb ≈ 0.47V
```

而死区电压误差约 0.48V。**信噪比 ≈ 1:1**，观测器在物理上几乎不可能稳定收敛。

自动切换等于在观测器"半生不熟"时强行接管 → 角度有偏差 → Id/Iq 算错 → 电流失控 → 爆鸣。

**修复**: 把"预热"和"切换"解耦：

```
之前: VF → [Mode 4: VF+观测器 → 10Hz自动切] → Mode 2
现在: VF → [Mode 4: VF+观测器, 不自动切] → 用户确认 → Mode 2
```

### 修改文件

#### 1. `UserSoftware/src/motorControl.c` — Mode 4 去自动切换

```diff
- case 4: /* VF → Sensorless — VF 启动自动切磁链观测器 */
+ case 4: /* PREPOS — VF 驱动 + 观测器后台预热（不自动切换） */

- if (motor.CurrentHz >= VF_TO_SENSORLESS_HZ) {
-     VF_to_Sensorless_Sync();
-     motor.Control_Mode = 2;
- }
+ // 不自动切, 用户手动切 Mode 2
```

#### 2. `UserSoftware/src/motorControl.c` — Mode 2 增加暖启动路径

进入 Mode 2 时检测磁链幅值：

| 条件 | 路径 | 行为 |
|------|------|------|
| `|ψr| < 30% FLUX` | **冷启动** (从 STOP/VF 直接来) | 同步 PLL→VF + ramp 增益 + 预加载 PI |
| `|ψr| ≥ 30% FLUX` | **暖启动** (从 Mode 4 预热后来) | PLL 不动(已锁定) + 只设速度斜坡 + 预加载 PI |

暖启动时 PLL 已经跟踪准确，切换电压连续，理论上最平滑。

#### 3. `taskManager/taskManager.c` — 按键循环重排

```diff
- static const uint8_t mode_cycle[] = {0, 3, 2, 4};  // STOP, VF, Sensorless, Prepos
+ static const uint8_t mode_cycle[] = {0, 3, 4, 2};  // STOP, VF, PREPOS, Sensorless
```

新循环: **STOP → VF → PREPOS → SENSORLESS → STOP → ...**

---

### 推荐操作流程

```
1. 上电, KEY1 启动 → 自动进 Mode 3 (VF)
2. KEY3/KEY4 设 TargetHz = 20~30Hz, 电机平稳旋转
3. KEY2 切到 Mode 4 ([PREPOS])
   ┌─ 电机仍然由 VF 驱动, 不会停
   ├─ 后台观测器开始预热
   └─ Vofa 观察: CH2(磁链) 慢慢涨, CH1(PLL_Err) 慢慢收敛
4. 等待 500ms~1s, 确认:
   ├─ LCD "Psi" 接近 0.0075 (或 Vofa CH2 ≥ 0.005)
   ├─ LCD "PLL" 接近 0.000 (或 Vofa CH1 ≤ ±0.05)
   └─ LCD "Est" ≈ CurrentHz
5. KEY2 切到 Mode 2 ([SENSORLESS])
   └─ 暖启动: PLL 已锁定 → 直接接管, 无突变
```

---

### VF_to_Sensorless_Sync() 状态

该函数目前不再被调用（Mode 4 不再自动切换）。保留在代码中，调试通过后可考虑：
- 恢复自动切换（提高阈值到 30~40Hz）
- 或作为手动切换时的辅助同步函数

---

## 合理猜测：为什么仍然爆鸣

### 猜测 1 (最可能): 反电动势太低，观测器物理上无法收敛

在 10Hz 电频率、52 极电机下：
- 反电动势 ≈ 0.47V
- 死区误差 ≈ 0.48V
- **SNR ≈ 1**

观测器靠反电动势来估算转子位置。信号和噪声一样大 → 磁链算不准 → PLL 锁不准 → 角度错 → Id 偏离 0 → 电流失控。

**验证方法**: 在 Mode 4 下提高 TargetHz 到 40Hz（反电动势 ≈ 1.9V, SNR ≈ 4），看 Vofa 上磁链幅值能否稳定收敛到 0.0075 附近。

如果 40Hz 能收敛而 10Hz 不能，说明就是 SNR 问题。解决方案：
- 提高切换频率
- 或做准确的死区补偿标定
- 或加高频注入做低速段的补充

### 猜测 2: 电机参数 (MOTOR_RS/LS/FLUX) 与实际偏差大

`MOTOR_LS = 20µH` 对 52 极电机来说极小。如果实际电感是 50µH 或 100µH，`ψr = ψ − Ls×I` 的计算会有系统性偏差。电流越大偏差越大 → 磁链角度偏移 → Id 不为 0 → 更大电流 → 恶性循环。

**验证方法**: 用 LCR 表实测电机任意两相电感，除以 2 得到相电感。用万用表实测线电阻 ÷ 2。

### 猜测 3: PLL 在低频时对电压误差的放大效应

PLL_Err = ψrβ×cosθ − ψrα×sinθ。当磁链幅值很小时（刚切换时只有零点几 V 的反电动势信号），微小的电压误差都会被放大为大的角度误差，导致 PLL 震荡。当前的 PLL 增益 20/10 在低频下可能过高。

**验证方法**: Mode 4 下如果看到 CH1(PLL_Err) 持续震荡(>±0.2)而不是逐步收敛，尝试降低 PLL_kp 到 5。

### 猜测 4: InvProtect 被禁用导致无保护

当前 `InvProtect.c` 的检测逻辑全部被注释。观测器一旦发散，电流飙升没有硬件层面的兜底保护。

### 猜测 5: 死区补偿方向/幅度不对

`V_DEADTIME = 0.3f` 是猜测值，且补偿符号（`+` vs `−`）需要与实际硬件对照。如果方向反了，等于在观测器输入端注入一个 0.6V 的误差（命令电压本身已有 0.3V 死区误差，补偿再加 0.3V 反向误差）。

---

## 下一步调试建议

### 步骤 1: 纯 VF 验证
- Mode 3, TargetHz=20~30Hz, 看三相电流是否正弦、电机是否平稳
- 如果 VF 本身就有异响/电流畸变 → 硬件或 SVPWM 问题

### 步骤 2: Mode 4 预热观察
- 切换到 Mode 4, 在 Vofa 上看 CH1~CH6
- **关键判断**: CH2(磁链幅值) 能否涨到 >0.005? CH1(PLL_Err) 能否收敛?
- 如果不能收敛 → 提高 TargetHz 到 40Hz 重试
- 如果 40Hz 能收敛 → 确认是低频 SNR 不足

### 步骤 3: 手动切换
- 确认观测器就绪后切 Mode 2
- 切换瞬间盯紧 CH4(Id) 和 CH5(Iq)
- Id 偏离 0 → 角度有误差
- Iq 跳变 → 电压不连续

### 步骤 4: 根据现象调参
| 现象 | 调什么 |
|------|--------|
| CH2 不涨 | 提高 TargetHz, 或检查 MOTOR_FLUX |
| CH1 震荡 | 降 PLL_kp/ki |
| 切换后 Id≠0 | 角度误差 → 等更久预热, 或提高切换频率 |
| 切换后 Iq 跳 | 调 pi_iq 预加载系数 (当前 0.5) |
| 切换后几秒内 CH1 变大 | PLL 积分 windup (已有抗饱和, 但需确认) |

---

## 历史变更 (6.25)

见下方原始记录。主要修复：
- PLL 积分抗饱和
- 磁链误差同周期计算（消延迟）
- Mode 2 冷启动单次同步
- Debug 遥测升级
- Gain 15000→5000

---

## 修改文件清单 (6.25 原始)

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
