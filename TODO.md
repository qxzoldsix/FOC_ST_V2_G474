# FOC_ST_V2 TODO 清单

> 更新日期: 2026-07-11 | 按优先级排序。状态: 🔴 阻塞 / 🟡 重要 / 🟢 优化 / ⚪ 远期 / ✅ 已完成

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

## 🔵 观测器与无感算法路线图

> 当前已实现: 电压型磁链观测器 + Hybrid Active Flux (PI/SMO 双反馈律)
> 目标: VF 分离启动 → 多观测器并行/切换 → 全速域无感

### 1. VF 分离 + 多观测器切换框架

- **现状**: VF/Mode 4 预热后手动切 Mode 2，仅支持单一观测器
- **目标**: VF 启动 → 低速观测器 → 高速观测器 自动/平滑切换
- **方案**:
  - 定义观测器切换速度区间 (如 0~5Hz HFI, 5~30Hz Hybrid, >30Hz 电压型)
  - 切换时角度/PLL 状态交接，避免突变
  - 多观测器后台并行运行，切换时直接取目标观测器角度

### 2. 高频注入 (HFI)

- **文件**: `flux.c` → `IPD_HFI_Injection()` (空壳)
- **原理**: αβ 轴注入 1kHz 高频正弦电压 → 提取高频电流响应 → 外差解调 → PLL 跟踪凸极位置
- **适用**: 凸极电机 (Ld≠Lq)，零速/极低速 (<5Hz)
- **关键**: 带通滤波器设计、注入幅值选择 (不触发磁饱和)、与基波 FOC 叠加
- **依赖**: 电流采样带宽足够 (OPA 需支持 >1kHz)

### 3. 参数辨识

- **目标**: 自动辨识 Rs / Ld / Lq / ψf，消除手动测量误差
- **方案**:
  - **Rs 辨识**: 注入直流电压，测量稳态电流 → Rs = Vdc / Idc
  - **Ld/Lq 辨识**: d/q 轴分别注入高频正弦电压 → L = Vh / (ωh × Ih)
  - **ψf 辨识**: 开环拖动到一定转速，测量反电动势 → ψf = BEMF / ωe
- **注意**: 辨识时电机不能带载，需在启动前完成

### 4. 六脉冲注入 (IPD) — 初始位置检测

- **文件**: `flux.c` → `IPD_SixPulse_Injection()` (空壳)
- **原理**: 依次注入 6 个短电压矢量 (100µs) → 比较电流响应幅值 → 判断转子 N/S 方向
- **适用**: 凸极电机静止时检测，无需高频注入硬件
- **步骤**:
  1. 依次输出 (1,0,0)(1,1,0)(0,1,0)(0,1,1)(0,0,1)(1,0,1) 六个矢量
  2. 每矢量持续 ~100µs，测量电流响应峰值
  3. 最大响应的矢量方向 = N 极方向 (±180°歧义)
  4. 二次判断用磁饱和效应区分 N/S
- **优势**: 比 HFI 简单，不依赖带通滤波器

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

### 9. CANFD 总线通信 🔥

> 底板 FDCAN1 硬件已配好 (PB8 RX / PB9 TX)，CubeMX 也生成了 `MX_FDCAN1_Init()`，
> 但现在只是个空壳，懒得搞啊 —— 经典CAN模式、没滤波器、没收发逻辑，跟没配差不多。
> 下面是从零到能用的完整 TODO，按依赖顺序排。

#### 9.1 改 FDCAN 初始化 — 切到 CANFD 模式

- **文件**: `Core/Src/fdcan.c` → `MX_FDCAN1_Init()`
- **现状**: `FrameFormat = FDCAN_FRAME_CLASSIC`，经典CAN，白瞎了G4的CANFD硬件
- **要改的**:
  - `FrameFormat` → `FDCAN_FRAME_FD_BRS` (FD帧 + 波特率切换)
  - 仲裁段波特率 1Mbps (通用)，数据段波特率 5Mbps / 8Mbps (FD加速)
  - 重新算 Nominal 和 Data 段的 Prescaler / Seg1 / Seg2 / SJW，别tm用CubeMX默认值
  - 时钟源 PCLK1 = 170MHz，仲裁段 1Mbps → Prescaler=1, Seg1=127, Seg2=42 之类
  - **别忘了** `DataPrescaler/DataTimeSeg1/DataTimeSeg2` 也要配，不然FD等于没开
- **坑**: Nominal timing 参数跟经典CAN一样算，Data段是独立的，别混了

#### 9.2 配 TX/RX Buffer 和过滤器

- **文件**: `Core/Src/fdcan.c`
- TX: 配一个 TX FIFO (深度8~16)，或者用 TX Buffer 专用槽位
- RX: 配 RX FIFO0 + RX FIFO1，或者 RX Buffer 按 ID 分组
- **过滤器**: 至少加一组标准帧过滤器，不配的话啥也收不到
  - `StdFiltersNbr` 改成 1 或 2
  - `FilterConfig` 里配 FilterID1/MaskID1，先全收 (`MaskID=0x000`) 调试通了再收紧
- 中断: IT0 已经开了但没回调，后面 9.3 补

#### 9.3 写 CANFD 驱动层 (`canfd_driver.c/h`)

- **新建文件**: `UserSoftware/inc/canfd_driver.h`, `UserSoftware/src/canfd_driver.c`
- **API 清单**:
  - `CANFD_Init()` — 初始化 + 启动
  - `CANFD_Send(uint32_t id, uint8_t *data, uint8_t len)` — 发FD帧 (len最大64)
  - `CANFD_SendClassic(uint32_t id, uint8_t *data, uint8_t len)` — 退化为经典CAN帧 (len≤8)，兼容老设备
  - `CANFD_SetFilter(uint32_t id, uint32_t mask)` — 动态改过滤器
  - `CANFD_GetRxFrame(CANFD_RxFrame_t *frame)` — 取一帧接收数据
  - `CANFD_GetRxCount()` — 看看收了多少帧没处理
- **中断回调**: `HAL_FDCAN_RxFifo0Callback()` / `HAL_FDCAN_TxFifoEmptyCallback()` 写到驱动里
  - 接收用环形缓冲存帧，别在中断里处理业务逻辑，tm会丢帧
- **错误处理**: Error callback 至少记个错，总线Off了能自动恢复

#### 9.4 定义 CANFD 通信协议

- **新建文件**: `UserSoftware/inc/canfd_protocol.h` (协议定义)
- **设计帧ID分配** :
  | ID范围 | 方向 | 内容 |
  |--------|------|------|
  | 0x100~0x11F | PC→板子 | 控制指令 (启停/模式切换/参数写入) |
  | 0x200~0x21F | 板子→PC | 状态上报 (转速/电流/温度/故障码) |
  | 0x300~0x30F | 双向 | 参数读写 (PI参数/电机参数) |
  | 0x700~0x70F | 板子→PC | 高速数据上报 (Vofa风格的实时波形) |
- **帧格式定义**:
  - 控制帧: [CMD(1B) | SubCMD(1B) | Data(0~62B)]
  - 状态帧: [Mode(1B) | Fault(2B) | Speed_f32(4B) | Iq_f32(4B) | Vbus_f32(4B) | Temp(2B) | ...]
  - 经典CAN兼容帧: 头8字节用经典格式，方便老工具监听
- **建议**: 高速波形帧用FD长帧 (64字节一口气发10个float)，别像Vofa串口那样一个float一个float发，CANFD带宽不用tm浪费了

#### 9.5 集成到任务调度

- **文件**: `taskManager/taskManager.c` + `taskManager/taskManager.h`
- 新增任务 `CANFD_Task()`，周期 1ms / 2ms:
  - 调 `CANFD_GetRxFrame()` 取指令 → 解析 → 改 motor mode / 目标转速 / PI参数
  - 调 `CANFD_Send()` 发状态帧
- 高速波形上报可以单独一个更快周期 (如 500µs / 1kHz)，或者用定时器触发
- **Task_Num** 别忘了 +1

#### 9.6 上位机/调试工具对接

- Vofa 那边可以用CAN转USB工具读，或者自己写个 Python 脚本解析
- 也可以复用现有 `CDC_Transmit_FS` 的 Vofa 数据，CAN 和 USB 并行发不冲突
- **测试步骤**:
  1. 先用回环模式 (`FDCAN_MODE_EXTERNAL_LOOPBACK`) 自发自收，确认驱动没问题
  2. 接 CAN 分析仪 (USB-CAN)，发标准帧看板子能不能收到
  3. 切 CANFD 帧 (BRS)，确认数据段高速通信正常
  4. 挂总线上跑长时间测试 (至少1小时)，看有没有丢帧/错误帧

> **总结**: 硬件已经有了，CubeMX也配了，就差人tm写代码。优先级可以放在观测器算法稳定之后搞，毕竟CANFD是锦上添花不是救命的功能。

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
| 7.11 | NTC 温度采集 | 三路 NTC (PB1/PB12/PB2) + 过温保护 + LCD 显示 |


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
| 11 | `NTC.c` | NTC 温度采集 + 过温保护已实现 | ✅ |


