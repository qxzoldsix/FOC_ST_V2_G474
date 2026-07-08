# FOC_ST_V2 TODO 清单

> 更新日期: 2026-07-08 | 按优先级排序。状态: 🔴 阻塞 / 🟡 重要 / 🟢 优化 / ⚪ 远期 / ✅ 已完成

---

## 🔴 P0 — 阻塞性问题

### 1. Mode 2 切闭环后电机颤动 🔍

- **现象**: Mode 4 预热完成后切 Mode 2，电机高频小幅颤动（非爆鸣）
- **疑点**:
  1. `SpeedRpm_GXieLv.XieLv_X` 未同步 → 速度环给定为 0 但电机在转 → 转矩冲击
  2. 电流环 PI 积分器冷启动 → Vd/Vq 与 VF 输出电压不连续 → 瞬时失力
  3. PLL 角度在自由运行瞬间有微小跳变
- **详细分析**: 见 `Current_Tuning_Guide.md` → 问题四
- **待验证**: Vofa 观察 CH4/CH5/CH3 切换瞬间波形

---

## 🟡 P1 — 重要功能完善

### 1. 电机参数运行时可切换

- **现状**: `MOTOR_RS/LS/FLUX/POLES` 为 `#define` 硬编码，换电机需重新编译
- **方案**: 参数表 + `Motor_Switch(uint8_t id)` API
- **优先级**: 经常切换不同电机时必做

### 2. `Balance_Control()` 空任务清理

- **文件**: `taskManager.c:42-45`
- 如果不需此任务，从 `TasksPare[]` 中移除，避免空转

### 3. 过压/欠压保护启用

- **现状**: `InvProtect.c` 中过压/欠压检测代码已写好但被注释
- **操作**: 取消注释，确认阈值 (`OV_THRESHOLD_V` / `UV_THRESHOLD_V`) 合理后启用

---

## 🟢 P2 — 优化改进

### 4. 任务调度器改进

- `Task_Manage_List_Init()` 中 Task_Count 初始值硬编码（非零），建议统一从 0 开始
- 任务数量 `Task_Num=5` 为宏，建议用 `sizeof(TasksPare)/sizeof(TasksPare[0])` 自动计算
- `Timer_Task_Count` 中 `Task_Period==0` 时任务被跳过但计数仍在跑

### 5. `Pwm_Input.c` 功能澄清

- **现状**: 文件存在于 `UserSoftware/src/`，功能不明确
- **需要**: 澄清用途（可能是外部 PWM 调速输入捕获？），补充注释或移除

### 6. LCD 菜单系统

- UI 交互框架: 菜单层级、参数编辑、状态页切换
- 利用现有 4 按键实现参数浏览和修改

### 7. PI 参数在线调试

- 建议添加在线参数调试接口（通过 Vofa/串口下发 Kp/Ki）

---

## ⚪ P3 — 远期规划

### 8. 有感 FOC (Mode 1) 传感器接入

- 需要: 霍尔/正交编码器/磁编码器接口与驱动程序
- Mode 1 代码链路已通 (`UVW_Axis_DQ` + `Speed_FOC` + `Idq_FOC` + `FOC_Svpwm_dq`)，仅缺角度来源

### 9. CAN 总线通信

- 项目已有 `fdcan.c/h` 初始化，但未定义应用层协议
- 可用于多机协同、参数下发、状态上报

### 10. 参数存储

- 将电机参数、PI 参数存入 Flash/EEPROM
- 上电自动加载，避免每次重新配置

### 11. MOTOR_LS 实测验证

- 当前 `MOTOR_LS = 20µH` 为估计值，52 极电机此值偏低
- 建议用 LCR 表 1kHz 实测（两相电感 ÷ 2），确认在合理范围

---

## ✅ 已修复 (2026-06 ~ 2026-07)

| 日期 | 项目 | 说明 |
|------|------|------|
| 6.25 | `Observer_Run()` 恢复 | 去掉 IQmath，用 sinf/cosf，死区补偿 + PLL 抗饱和 + 同周期 Err |
| 6.25 | `Idq_FOC()` + `FOC_Svpwm_dq()` 恢复 | 电流环完整链路，含电压圆约束 |
| 6.25 | 变量命名统一 | Flux/flux → 统一为大写 F |
| 6.25 | PI2 宏缺失 | 改用 2.0f * PI |
| 6.25 | PLL 积分抗饱和 | Limit_Sat ±0.157 rad/周期 |
| 6.25 | Gain 15000→5000 | 降低磁链补偿震荡风险 |
| 6.27 | VF 分离架构 | Mode 4 只预热不自动切换，暖启动检测 |
| 6.27 | 按键循环重排 | STOP→VF→PREPOS→SENSORLESS |
| 6.27 | Mode 2 暖启动路径 | |ψr|≥30% FLUX → PLL 不动直接接管 |
| 7.01 | `key_bsp.h` 引脚冲突修复 | key_5 不再复用 key_1 引脚 (PC9) |
| 7.01 | InvProtect 实现 | 相过流 + 母线过流检测 + 故障锁存 + KEY1 长按清除 |
| 7.01 | 按键功能映射 | KEY1 启停, KEY2 模式切换, KEY3/4 频率± |
| 7.01 | PI 积分分离修复 | `Out==v1` 浮点比较 → 限幅检测 |
| 7.01 | SVPWM 母线电压补偿启用 | `Foc_Para_Calc` 计算 Km + `svpwm_Cale` 启用补偿 |
| 7.01 | Mode 4 预定位实现 | VF 驱动 + 观测器后台预热 + PLL 前馈 |
| 7.01 | VF 全部 Bug 修复 | 除零保护 + TargetVolt 不清零 + PWM 输出 + while 角度归一 + static |
| 7.01 | Ctrl_ts 修正 | 0.0005→5e-5 (20kHz) |
| 7.01 | 速度系数修正 | 650→1/(8×Ts×2π) 自动计算 |
| 7.05 | 电流采样负号修复 | 三相电流 + 母线电流加回负号 (ADC_Sample.c) |
| 7.05 | PI 限幅修正 | 速度环 12A→3A, 电流环 12pu→1pu |
| 7.08 | Hybrid Active Flux 观测器 | 二阶系统带宽设计 PLL + 反馈补偿 PI + SMO 备用 |
| 7.08 | Vofa 通道扩展 | 6ch→9ch，新增三相电流原始值 |
| 7.08 | Foc.h 注释整理 | 乱码清理 + 冗余字段移除 + Control_Mode 注释修正 |

---

## 📋 快速问题汇总

| # | 文件 | 问题 | 状态 |
|---|------|------|------|
| 1 | `flux.c` | Observer_Run() 已恢复完整功能 | ✅ |
| 2 | `Foc.c` | Idq_FOC / FOC_Svpwm_dq 已恢复 | ✅ |
| 3 | `motorControl.c` | Mode 3 多余 Angel_Get 已删除 | ✅ |
| 4 | `InvProtect.c` | 过流保护已实现，过/欠压待启用 | 🟡 |
| 5 | `flux.c` | 变量命名已统一为大写 F | ✅ |
| 6 | `key_bsp.h` | key_5 引脚冲突已修复 | ✅ |
| 7 | `taskManager.c` | LCD 模式标签已对齐 | ✅ |
| 8 | `flux.h` | 电机参数 #define 硬编码 | 🟡 |
| 9 | `taskManager.c` | Task_Count 非零初始值 | 🟢 |
| 10 | `PI_Cale.c` | 浮点 == 比较已修复 | ✅ |

