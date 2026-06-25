# FOC_ST_V2 TODO 清单

> 按优先级排序。状态: 🔴 阻塞 / 🟡 重要 / 🟢 优化 / ⚪ 远期

---

## 🔴 P0 — 阻塞性问题（核心功能缺失）

### 1. 恢复磁链观测器 `Observer_Run()` 并接入无感模式

- **现状**: `Flux/src/flux.c` 中 `Observer_Run()` 整函数被注释，Mode 3 (Sensorless) 的 `Angel_Get()` 也一并注释
- **影响**: 无感 FOC 完全没有角度来源，Mode 3 形同虚设
- **操作**:
  1. 取消 `Observer_Run()` 注释
  2. 将 `fluxR_in_wb` 等变量统一命名为 `FluxR_in_wb`（与 `flux.c` 实际定义一致）
  3. 确认 `PI2` / `_IQ24` / `CLARKE_PCurr` 等依赖可编译
  4. 在 `motorControl.c` 的 Mode 3 中取消 `Angel_Get()` 注释
  5. 验证 PLL 参数 (`PLL_kp=20`, `PLL_ki=10`, `Gain=150000`)

### 2. 恢复电流环 (Idq_FOC) 实现

- **现状**: `Foc.c` 中 `Idq_FOC()` 和 `FOC_Svpwm_dq()` 均被注释
- **影响**: 无感 FOC 只有速度环外环，没有电流内环，无法实际控制 Vd/Vq
- **操作**:
  1. 恢复 `Idq_FOC()` → 计算 Vd/Vq
  2. 恢复 `FOC_Svpwm_dq()` → iPark → SVPWM → PWM 更新
  3. 配置 `pi_id` / `pi_iq` 两个 PI 控制器参数
  4. 在 Mode 3 的 `Speed_FOC()` 之后串联电流环

---

## 🟡 P1 — 重要功能完善

### 3. 电机参数运行时可切换

- **现状**: `MOTOR_RS/LS/FLUX/POLES` 为 `#define` 硬编码，换电机需重新编译
- **方案**: 参数表 + `Motor_Switch(uint8_t id)` API（见已讨论方案）
- **优先级**: 经常切换不同电机时必做

### 4. 按键功能映射

- **现状**: `key_process()` 的状态机已完整实现（单/双/长按），但没有任何按键被映射到实际操作
- **建议**:
  | 按键 | 短按 | 长按 |
  |------|------|------|
  | KEY1 | 启动/停止 | — |
  | KEY2 | 模式切换 | — |
  | KEY3 | 频率+ | 连续+ |
  | KEY4 | 频率- | 连续- |

### 5. `InvProtect` 逆变器保护

- **现状**: `InvProtect.c` 为空文件
- **需实现**:
  - 过流保护 (OC) — 相电流阈值
  - 过压/欠压保护 (OV/UV) — 母线电压范围
  - 过温保护 (OT) — 可选
  - 故障锁存与清除机制

### 6. 补充 `Pwm_Input.c` 功能说明

- **现状**: 文件存在于 `UserSoftware/src/`，但无对应头文件，功能不明确
- **需要**: 澄清用途（可能是外部 PWM 调速输入捕获？），补充注释或移除

---

## 🟢 P2 — 优化改进

### 7. 代码清理

- `flux.c` 中变量命名一致性问题: `fluxR_in_wb` vs `FluxR_in_wb`（当前混合使用）
- `key_bsp.h` 中 `key_5` 与 `key_1` 定义相同引脚 (PC9)，疑似复制错误
- `Foc_Control()` 中 Mode 2/5 为空 case，补充注释说明或直接移除
- 清理 `Foc.h` 中被注释的冗余成员 (`I_d_fb`, `I_q_fb`, `IQAngle_JZ`)
- `display.c` 为空，如果没有实际用途可删除

### 8. PI 控制器性能优化

- `PID_controller()` 的积分分离逻辑可简化: 当前 `(pV->Out == pV->v1)` 判断受浮点精度影响
- 速度环 PI 参数、电流环 PI 参数需要标定文档
- 建议添加在线参数调试接口（通过 Vofa/串口下发 Kp/Ki）

### 9. 任务调度器改进

- `Task_Manage_List_Init()` 中硬编码了 Task_Count 初始值（非零），建议统一从 0 开始
- 任务数量 `Task_Num=5` 为宏，新增任务需改宏 → 建议用 `sizeof(TasksPare)/sizeof(TasksPare[0])` 自动计算
- Timer_Task_Count 的计数逻辑: 当 `Task_Period==0` 时任务会被跳过但计数仍在跑，可能非预期

### 10. VF 控制改进

- 当前 V/F 曲线为简单线性，建议加入:
  - 低频 IR 补偿（提升启动力矩）
  - 最高频率限制保护
  - VF → FOC 切换过渡（当前无）

---

## ⚪ P3 — 远期规划

### 11. 有感 FOC (Mode 2)

- 需要: 霍尔/正交编码器/磁编码器接口与驱动程序
- 实现 `UVW_Axis_DQ_HALL()` 或 `UVW_Axis_DQ_magnetic()` 角度获取
- 与无感模式对比验证

### 12. 预定位 (Mode 5)

- IPD (Initial Position Detection) 或高频注入
- 用于无感启动时的初始角度获取

### 13. CAN 总线通信

- 项目已有 `fdcan.c/h` 初始化，但未定义应用层协议
- 可用于多机协同、参数下发、状态上报

### 14. LCD 菜单系统

- UI 交互框架: 菜单层级、参数编辑、状态页切换
- 利用现有 4 按键实现参数浏览和修改

### 15. 参数存储

- 将电机参数、PI 参数存入 Flash/EEPROM
- 上电自动加载，避免每次重新配置

---

## 📋 快速问题汇总

| # | 文件 | 问题 |
|---|------|------|
| 1 | `flux.c:50-145` | `Observer_Run()` 注释，无感不工作 |
| 2 | `Foc.c:72-110` | `Idq_FOC()` / `FOC_Svpwm_dq()` 注释，电流环缺失 |
| 3 | `motorControl.c:37` | Mode 3 的 `Angel_Get()` 注释 |
| 4 | `InvProtect.c` | 保护功能空实现 |
| 5 | `flux.c:3-5` | 变量命名不一致 (Flux_in_wb vs fluxR_in_wb) |
| 6 | `key_bsp.h:25` | key_5 复用 key_1 引脚 |
| 7 | `display.c` | 文件为空 |
| 8 | `flux.h:5-8` | 电机参数 `#define` 硬编码 |
| 9 | `Foc_Control()` | Mode 0/1/2/5 case 大多为空 |
| 10 | `taskManager.c:109-125` | Task_Count 非零初始值 |
