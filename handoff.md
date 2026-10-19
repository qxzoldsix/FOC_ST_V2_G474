# FOC_ST_V2 工程与 CAN 控制通信 Handoff

> 核对日期：2026-10-14（CAN 历史记录保留，新增架构评审与当前改动汇总）
>
> 核对对象：当前工作区代码（包含尚未提交的本地修改）；已实施改动与待实施规划分开记录
>
> 当前 Git HEAD：`c43e794924c73d0aec71ddcb6ee14937aa39835e`（全新参数辨识可能有错）；早期 CAN 参考提交：`b7297bd203cf15b00625ec429b2e47507a3aad28`
>
> MCU：STM32G474RE；节点号：`0x01`；软件版本：`260830A`

## 1. 结论先行

当前软件使用 **经典 CAN**，不是 CAN FD：11 位标准帧、1 Mbps、单帧最多 8 字节、STM32 小端、`float` 按 IEEE-754 binary32 直接传输。

已具备：

- FDCAN1 正常模式初始化、全标准 ID 接收过滤、RX FIFO0 中断接收。
- NMT、PDO1 控制、PDO2 占位控制、SDO 参数读写、心跳和周期状态上报。
- `0x181` 固定为 8 字节状态帧，包含 `SpeedRPM/Fault_DTC/Control_Mode/PwmEnabled`。
- CAN STOP/QUICK_STOP 会立即进入 STOP 并封锁三相 PWM。
- START 仅在 NMT `OPERATIONAL` 状态下接受，并检查模式、转速参数和故障状态。

当前不能按“完整 CAN 电机控制器”交付：

- `0x281` 实际只发送 `Ia`、`Ib`；`Ic/Vbus/Id/Iq` 没有上总线。
- CAN START 帧不含 `TargetVolt`，工程内也没有其他 CAN 写入口；`motor.TargetVolt` 上电为 0。当前安全检查会直接拒绝 VF/PREPOS 启动，不再带着无效电压开启功率级。
- `0x301` 的 Id/Iq 目标帧能接收，但回调没有应用数据。
- EMCY 发送函数已写，但故障保护没有调用它；故障码目前通过周期 `0x181` 上报，不会触发即时 EMCY。
- 若 START 被拒绝，协议没有 ACK/NACK，主机只能从运行数据间接判断。

## 2. 软件与总线配置

| 项目 | 当前值 | 代码位置 |
|---|---:|---|
| 外设 | FDCAN1 | `Core/Src/fdcan.c` |
| 引脚 | PB8=RX，PB9=TX，AF9 | `Core/Src/fdcan.c` |
| 帧格式 | `FDCAN_FRAME_CLASSIC` | `Core/Src/fdcan.c` |
| 工作模式 | `FDCAN_MODE_NORMAL` | `Core/Src/fdcan.c` |
| 标称位速率 | 1 Mbps | 160 MHz / (4 × (1+31+8)) |
| 采样点 | 80% | (1+31) / 40 |
| SJW | 8 tq | `NominalSyncJumpWidth=8` |
| 自动重发 | 关闭 | `AutoRetransmission=DISABLE` |
| 接收过滤 | 标准 ID `0x000~0x7FF` 全部进 FIFO0 | range filter |
| TX FIFO | 3 个元素 | 手工写 `TXBC=(3U<<24)` |
| RX FIFO0 | 3 个元素 | 手工写 `RXF0S=(3U<<16)` |
| RX FIFO1 / TX Event FIFO | 禁用 | 手工寄存器配置 |
| 节点 ID | `0x01`，编译期固定 | `CANFD_NODE_ID` |
| 上电 NMT 状态 | `OPERATIONAL (0x05)` | `CANFD_App_Init()` |

虽然数据相位参数填写为 5 Mbps，但经典 CAN 帧不会使用数据相位。目录和函数仍沿用 `canFD`/`CANFD_*` 命名，不能据此判断线上是 FD。

## 3. 初始化与运行链路

初始化顺序与当前 `main.c` 一致：

1. `MX_FDCAN1_Init()`：配置 FDCAN、FIFO、过滤器，启动外设和 RX FIFO0 新消息中断。
2. `CANFD_App_Init()`：节点直接置为 `OPERATIONAL`，清零收发/错误计数。
3. ADC、TIM、USB、LCD、任务调度器等初始化。
4. `Foc_Pwm_Init()`：只启动 TIM1 CH4 的 ADC 触发，三相桥保持 MOE 封锁；初始化失败直接进入 `Error_Handler()`。
5. 电流零点校准、观测器和 PI 初始化，最后使能保护。
6. 主循环执行任务表；TIM3 每 1 ms 给任务计数。

接收链路：

`FDCAN1_IT0_IRQHandler()` → HAL FDCAN IRQ → `HAL_FDCAN_RxFifo0Callback()` → 一次中断排空 FIFO0 → `CANFD_ProcessRxMessage()` → NMT/PDO/SDO 回调。

发送周期：

| CAN ID | 实际周期 | 说明 |
|---|---:|---|
| `0x181` | 10 ms | 每次 `CANFD_Task()` 发送 |
| `0x281` | 50 ms | 5 次 CAN 任务发 1 次 |
| `0x701` | 50 ms | 5 次 CAN 任务发 1 次 |

NMT STOP/PRE-OP 不会停止周期 PDO 上报，只会停止电机并改变心跳状态。

## 4. 当前实际 CAN ID

| CAN ID | 方向 | DLC | 周期/触发 | 当前实际作用 | 状态 |
|---|---|---:|---|---|---|
| `0x000` | 主机→节点 | 2 | 事件 | NMT 命令 + Node ID | 已实现 |
| `0x080` | 主机→节点 | 0 | 事件 | SYNC | 仅定义，未处理 |
| `0x081` | 节点→主机 | 4 | 函数调用 | EMCY 故障码 | 函数存在，未自动调用 |
| `0x181` | 节点→主机 | 8 | 10 ms | 转速 + 故障码 + 模式 + PWM 状态 | 已实现 |
| `0x201` | 主机→节点 | 1 或 6~8 | 事件 | STOP/CLEAR/QUICK_STOP 或 START | 已实现 |
| `0x281` | 节点→主机 | 8 | 50 ms | `Ia` + `Ib` | 已实现 |
| `0x301` | 主机→节点 | 8 | 事件 | `Id_Target` + `Iq_Target` | 可接收但不生效 |
| `0x581` | 节点→主机 | 8 | SDO 应答 | 读、写、错误响应 | 已实现 |
| `0x601` | 主机→节点 | 8 推荐 | 事件 | SDO 请求 | 已实现 |
| `0x701` | 节点→主机 | 1 | 50 ms | NMT 节点状态 | 已实现 |

所有多字节字段均为 little-endian。接收端只接受标准数据帧；扩展帧和远程帧直接忽略。

## 5. 报文字节定义

### 5.1 NMT：0x000

| Byte | 类型 | 含义 |
|---:|---|---|
| 0 | `uint8` | NMT 命令 |
| 1 | `uint8` | Node ID；`0x00`=广播，`0x01`=本节点 |

| 命令 | 值 | 当前动作 |
|---|---:|---|
| START | `0x01` | 节点状态置 `OPERATIONAL`；不会启动电机 |
| STOP | `0x02` | 节点状态置 `STOPPED`，立即 `Motor_ControlStop()` |
| PRE-OP | `0x80` | 节点状态置 `PRE_OP`，立即 `Motor_ControlStop()` |
| RESET | `0x81` | 立即停机并 `NVIC_SystemReset()` |

示例：对节点 1 发 NMT START：ID `000`，DLC 2，数据 `01 01`。

### 5.2 PDO1 RX 控制：0x201

START 使用 6 字节结构：

| Byte | 类型 | 字段 | 当前规则 |
|---:|---|---|---|
| 0 | `uint8` | `Command` | START=`0x01` |
| 1 | `uint8` | `Control_Mode` | 外部 START 当前接受 2~5 |
| 2~5 | `float32 LE` | `TargetSpeedRPM` | 必须有限且 `>=0` |

模式：0=STOP，1=SENSOR，2=SENSORLESS，3=VF，4=PREPOS，5=参数辨识。模式 1 尚未接入位置传感器，当前状态机和外部 START 都会拒绝它；模式 5 要求目标转速严格为 0，并在完成或失败后自动封波停机。

目标电频率按当前代码计算：

`TargetHz = TargetSpeedRPM × (MOTOR_POLES / 2) / 60`

当前代码仍按 `MOTOR_POLES=14` 计算 `TargetHz = RPM × 7 / 60`。2026-10-14 用户确认 14 是极对数，因此这里是待修复错误，正确公式应为 `TargetHz = RPM × 14 / 60`，速度反馈应为 `RPM = Hz × 60 / 14`。当前协议行为尚未修改，主机调试时需区分现有行为与正确物理换算。

示例：START + VF + 1000 RPM：ID `201`，DLC 6，数据 `01 03 00 00 7A 44`。固件计算 `TargetHz≈116.667 Hz`。由于该帧不会设置 `TargetVolt`，上电默认值为 0 时此命令会被安全拒绝，不会开桥。

STOP、FAULT_CLEAR、QUICK_STOP 只需要 Byte0：

| 数据 | 动作 | NMT 状态限制 |
|---|---|---|
| `00` | STOP，清零目标/当前频率并封锁 PWM | 无，任何状态接受 |
| `02` | 清除可清除故障 | 无，任何状态接受 |
| `04` | QUICK_STOP；当前实现与 STOP 相同 | 无，任何状态接受 |

START 的接受条件全部满足才执行：节点为 OPERATIONAL、DLC≥6、模式 2~5、RPM 有限且非负、换算电频率不超过 400 Hz、当前无故障；VF/PREPOS 还要求 `0.01 < TargetVolt <= 0.95 pu`，参数辨识模式要求 RPM=0。拒绝时没有 CAN 应答。

### 5.3 PDO1 TX 状态：0x181，DLC 8

| Byte | 类型 | 字段 | 单位 |
|---:|---|---|---|
| 0~3 | `float32 LE` | `motor.SpeedRPM` | RPM |
| 4~5 | `uint16 LE` | `motor.Fault_DTC` | 位掩码 |
| 6 | `uint8` | `motor.Control_Mode` | 模式枚举 |
| 7 | `uint8` | `Foc_Pwm_IsEnabled()` | 0=封波，1=PWM 已使能 |

该布局从版本 `260830A` 起生效；原先 Byte4~7 的 `CurrentHz` 不再发送，主机解析需同步升级。

### 5.4 PDO2 TX 扩展状态：0x281，DLC 8

| Byte | 类型 | 字段 | 单位 |
|---:|---|---|---|
| 0~3 | `float32 LE` | `PhaseU_Curr (Ia)` | A |
| 4~7 | `float32 LE` | `PhaseV_Curr (Ib)` | A |

头文件仍定义 24 字节完整结构，但 `Ic/Vbus/Id/Iq` 当前未发送。

### 5.5 PDO2 RX：0x301，DLC 8

| Byte | 类型 | 字段 | 单位 |
|---:|---|---|---|
| 0~3 | `float32 LE` | `Id_Target` | A |
| 4~7 | `float32 LE` | `Iq_Target` | A |

仅在 OPERATIONAL 状态接收。当前 `CANFD_OnPDO2_RX()` 只做空指针检查，随后丢弃数据，不会改变电流环目标。

### 5.6 Heartbeat：0x701，DLC 1

| 值 | 状态 |
|---:|---|
| `0x04` | STOPPED |
| `0x05` | OPERATIONAL |
| `0x7F` | PRE_OP |

心跳只反映 NMT 状态，不反映电机是否运行、PWM 是否开启或是否存在故障。上电后节点直接为 `0x05`，没有 CANopen boot-up `0x00` 帧。

### 5.7 EMCY：0x081，DLC 4

| Byte | 内容 |
|---:|---|
| 0 | `fault_code` 低字节 |
| 1 | `fault_code` 高字节 |
| 2~3 | `00 00` |

`CANFD_EMCY_Send()` 可发送该帧，但当前保护代码没有调用它，正常运行中不会自动出现 EMCY。

## 6. SDO：0x601 请求 / 0x581 应答

请求/应答统一按 8 字节发送最安全：

| Byte | 含义 |
|---:|---|
| 0 | 命令字 |
| 1~2 | 16 位寄存器索引，小端 |
| 3 | 子索引；当前仅接受 0，其他值返回“索引不存在” |
| 4~7 | 32 位原始值，小端 |

命令字：读请求 `0x40`；读响应 `0x43`；写 32 位请求 `0x23`；写成功 `0x60`；错误 `0x80`。当前寄存器均为 32 位对象，`0x2B/0x2F` 会返回类型错误。

写 32 位请求必须使用 DLC 8。长度不足，或使用 16/8 位写命令，均返回 `0x06070010`；浮点值必须为有限数并通过对应范围校验。

| 索引 | 名称 | 读值/写入对象 | 当前写入效果 |
|---:|---|---|---|
| `0x0000` | SPEED_KP | `pi_spd.Kp`，float | 可写，`0~1` |
| `0x0001` | SPEED_KI | `pi_spd.Ki`，float | 可写，`0~0.1` |
| `0x0002` | SPEED_UMAX | `pi_spd.Umax`，float | 可写，`0.1~12 A`；同步设置 `Umin=-Umax` 并钳制 PI 历史量 |
| `0x0003` | ID_KP | `pi_id.Kp`，float | 可写，`0~1` |
| `0x0004` | ID_KI | `pi_id.Ki`，float | 可写，`0~0.1` |
| `0x0005` | ID_UMAX | `pi_id.Umax`，float | 只读；由 FOC 电压圆动态管理 |
| `0x0006` | IQ_KP | `pi_iq.Kp`，float | 可写，`0~1` |
| `0x0007` | IQ_KI | `pi_iq.Ki`，float | 可写，`0~0.1` |
| `0x0008` | IQ_UMAX | `pi_iq.Umax`，float | 只读；由 FOC 电压圆动态管理 |
| `0x0009` | PLL_KP | `Foc_observer.PLL_kp`，float | 可写，`0~1000`；同步更新 ramp 目标值 |
| `0x000A` | PLL_KI | `Foc_observer.PLL_ki`，float | 可写，`0~100`；同步更新 ramp 目标值 |
| `0x000B` | OBS_GAIN | `Foc_observer.Gain`，float | 可写，`0~100000` |
| `0x000C` | OBS_BW | `Foc_observer.PLL_BW_Hz`，float | 只读；避免只改带宽、不重算 PLL 增益 |
| `0x000D` | OC_THRESH | 20.0f | 只读宏 |
| `0x000E` | OV_THRESH | 30.0f | 只读宏 |
| `0x000F` | UV_THRESH | 8.0f | 只读宏 |
| `0x0010` | OC_BUS_THRESH | 15.0f | 只读宏 |
| `0x0011` | OT_THRESH | `NTC_OT_FAULT_C`，float | 只读宏 |
| `0x0012` | VF_VOLT_MAX | 0.95f (pu) | 只读宏 |
| `0x0013` | VF_FREQ_MAX | 400.0f | 只读宏 |
| `0x0014` | VF_RAMP_GRAD | `VF_HzRamp.XieLv_Grad`，float | 可写，`0.00001~0.1` |
| `0x1000` | DEVICE_ID | `0x20260718` | 只读，写返回错误 |
| `0x1001` | SW_VERSION | `0x0260830A`（版本 `260830A`） | 只读，写返回错误 |
| `0x1002` | NODE_ID | 原始整数 1 | 只读，写返回错误 |
| `0x1010` | PARAM_ID_STATE | Byte0=step，Byte1=error，Byte2=done，Byte3=valid | 只读 |
| `0x1011` | PARAM_ID_RS | `pid.Rs`，float，单位 Ω | 只读候选值 |
| `0x1012` | PARAM_ID_LD | `pid.Ld`，float，单位 H | 只读候选值 |
| `0x1013` | PARAM_ID_LQ | `pid.Lq`，float，单位 H | 只读候选值 |

PCAN-View 读取版本：发送 `601#40 01 10 00 00 00 00 00`，正常响应为
`581#43 01 10 00 0A 83 60 02`，数据区 `0A 83 60 02` 按小端解释为
`0x0260830A`，对应版本 `260830A`。

参数辨识启动帧为 `201#01 05 00 00 00 00`。完成后读取 `0x1010`；仅当
`done=1`、`valid=1`、`error=0` 时，`0x1011~0x1013` 才是通过软件范围校验的候选值。
这些值不会在辨识完成时自动覆盖观测器参数。2026-08-24 实测一次得到
`Rs=0.07710429 Ω`、`Ld=16.5559 µH`、`Lq=13.1455 µH`，已按人工试用要求写入
`Flux/inc/flux.h`；与原参数（0.875 Ω、200 µH、200 µH）相差十倍以上，实机表现为同转速
电流增大，说明该组数值尚不能视为已验证参数，继续运行前应恢复原值或重新校准辨识方法。

参数辨识内部原有的 2 A 软件门限已于 2026-08-24 暂时关闭，因此 `error=2` 枚举仍保留，
但当前代码不会由该门限主动产生。系统相过流、母线过流、过欠压和过温保护仍然生效。

辨识 step：0=IDLE，1=ALIGN，2/3=Rs低平台稳定/采样，4/5=Rs高平台稳定/采样，
6=Ld脉冲，7=Lq脉冲，8=VALIDATE，9=DONE，10=FAILED。
error：0=无错误，1=母线异常，2=辨识电流超限（枚举保留、当前检测停用），3/4=Rs响应/范围异常，
5/6=Ld/Lq响应不足，7=电感范围异常，8=非法状态。

错误码：不存在索引/子索引 `0x06020000`；只读 `0x06010002`；长度或类型不匹配 `0x06070010`；浮点非有限、负增益或越界 `0x06090030`。

## 7. 故障与停机行为

保护在 20 kHz ADC/FOC 回调里检查。任一故障都会锁存、写入 `motor.Fault_DTC`，并立即把模式置 STOP、关闭 TIM1 MOE、清零 CCR1~3；故障持续期间不会在每个控制周期重复调用 HAL 停 PWM。

| 位 | 掩码 | 当前检测/含义 | 清除方式 |
|---:|---:|---|---|
| 0 | `0x0001` | U 相绝对电流 >20 A | CAN FAULT_CLEAR，故障源需先消失 |
| 1 | `0x0002` | V 相绝对电流 >20 A | 同上 |
| 2 | `0x0004` | W 相绝对电流 >20 A | 同上 |
| 3 | `0x0008` | 母线电压 >30 V | 同上 |
| 4 | `0x0010` | 母线电压 <8 V | 同上 |
| 5 | `0x0020` | 母线绝对电流 >15 A | 同上 |
| 6 | `0x0040` | NTC 温度 >100°C | 同上 |
| 7 | `0x0080` | Hall/编码器异常 | 仅定义，当前未检测 |
| 8~12 | `0x0100~0x1000` | 堵转/缺相/启动失败/反复过流/硬故障 | 需重新上电；除反复清除外多为预留 |

2026-08-24 曾尝试用无感估算速度与目标速度的偏差实现堵转判断，但实机未可靠触发，
相关代码已按要求撤销，当前 `FAULT_LOCK` 仅保留故障码定义。经验：`motor.SpeedRPM`
来自无感观测器，并非独立机械转速；堵转时可能仍产生虚假速度。后续重新实现时应结合
观测器失锁/反电势可信度、持续电流或独立位置传感器，不能只放宽速度比例。

FAULT_CLEAR 会先复查实时故障源；过流、欠压、过压或过温仍存在时拒绝清除。可清除故障在本次上电第 6 次成功清除时会增加 `FAULT_REPEAT_OC (0x0800)` 锁存位。

`Fault_DTC` 已在 `0x181` Byte4~5 周期上报，主机可可靠读取当前锁存故障。保护仍不自动发 EMCY，因此故障通知时延取决于 `0x181` 的 10 ms 周期。

## 8. 推荐主机控制顺序（按当前实现）

1. 上电后确认收到 `0x701 = 05`，并观察 `0x181/0x281`。
2. 需要明确恢复 OPERATIONAL 时发送 `000#0101`。
3. 在电机未连接或功率级安全条件下，先验证 STOP：`201#00`，确认 PWM 硬件封锁。
4. START 使用 `0x201` 六字节帧；主机必须自行处理“无 ACK”的事实。
5. 任何异常立即发 `201#04` 或 `201#00`；两者当前效果相同。
6. 清故障前先去除欠压/过压/过流/过温源，再发 `201#02`。

在补齐 `TargetVolt` 通信入口之前，VF/PREPOS 的 CAN 启动会被拒绝。模式 1 尚未接入传感器；模式 2 没有强制的 VF→PREPOS→闭环切换门槛，主机直接切入的风险由主机承担。VF→PREPOS→SENSORLESS 等运行模式切换现在保持 PWM 连续，只在 STOP→运行时开启一次输出，并在进入闭环时清 PI 历史量、用当前速度接管斜坡。

## 9. 调试观测点

VOFA 使用 USB CDC + JustFloat，`HFPeriod_RUN()` 约每 1 ms 发送一帧。2026-08-24
已将原来的9通道调试帧精简为三相电流波形：CH1=`PhaseU_Curr`、
CH2=`PhaseV_Curr`、CH3=`PhaseW_Curr`，单位均为 A。每帧16字节：3个小端
IEEE-754 float，随后为帧尾 `00 00 80 7F`。旧的频率、Id/Iq、Vd/Vq和母线电压
不再通过 VOFA 上报。

Keil Watch 可直接查看：

| 变量/寄存器 | 含义 |
|---|---|
| `canfd_tx_cnt` | 成功入 TX FIFO 的累计帧数 |
| `canfd_rx_cnt` | 从 RX FIFO0 成功取出的累计帧数 |
| `canfd_err_cnt` | HAL 收发失败累计次数 |
| `hfdcan1.ErrorCode` | HAL 错误码 |
| `hfdcan1.Instance->TXBC` | 期望 `0x03000000` |
| `motor.Control_Mode` | 当前控制模式 |
| `motor.TargetHz` / `motor.TargetVolt` | 目标频率/目标电压 |
| `motor.Fault_DTC` | 当前故障位掩码；同时在 `0x181` Byte4~5 上报 |
| `InvProtect_GetFault()` | 保护模块内部故障位掩码 |

接收 FIFO 中断会一次排空全部待处理帧，可避免连续控制帧只处理第一条。RX/TX FIFO 仅 3 深度且自动重发关闭，主机不应突发大量 SDO/PDO 请求。

## 10. CubeMX 重新生成注意事项

`Core/Src/fdcan.c` 的以下逻辑属于手工补丁，重新生成后必须核对：

```c
hfdcan1.Instance->TXBC  = (3U << 24U);
hfdcan1.Instance->RXF0S = (3U << 16U);
hfdcan1.Instance->RXF1S = 0U;
hfdcan1.Instance->TXEFS = 0U;
```

还要核对：经典 CAN 格式、NORMAL 模式、1 Mbps 位时序、全标准 ID range filter、FDCAN 启动和 RX FIFO0 通知。

## 11. 建议按优先级补齐

1. 给 CAN START 增加 `TargetVolt`，或增加一个真正生效的运行参数寄存器；同时做范围校验。
2. 给 START 增加明确 ACK/NACK，并返回拒绝原因。
3. 重新设计扩展状态分帧，使 Vbus、Id/Iq 等诊断量真正上线。
4. 故障首次出现时调用 EMCY，并增加可读故障寄存器。
5. 让 PDO2 RX 真正接入电流目标，或删除该对外接口。
6. 为模式 1/2 增加启动前置条件与闭环切换门槛。
7. 增加 CAN RX 超时安全停机和命令序号/看门狗。

## 12. 本次核对边界

已做代码级交叉核对：FDCAN 初始化、协议头/实现、RX 中断、任务周期、控制回调、PWM 启停、保护、V/F、FOC、观测器参数及 Keil 工程文件引用。

历史记录：2026-08-30 已使用本机 Keil ARMCC V5.05 update 1 对 `MDK-ARM/FOC_ST_V2.uvprojx` 完成构建：`0 Error(s), 0 Warning(s)`，AXF/HEX 生成成功（Code=48756，RO-data=19424，RW-data=492，ZI-data=9684）。这不是 2026-10-14 当前工作区重新构建的结果。

当前仍未连接 PCAN 和电机台架，因此不宣称已完成烧录或实机回归。交付前至少执行：PCAN 1 Mbps 收发、STOP/MOE 硬件封波、START 拒绝条件、VF→PREPOS→SENSORLESS 无扰切换、故障锁存与清除、连续帧 FIFO 排空和断总线安全性测试。

## 13. 维护约定

后续每次实质代码修改都应同步更新本文件，至少记录：修改日期、实际生效位置、运行条件、
保护/通信行为变化、编译结果以及尚未完成的实机验证。台架临时关闭保护或写入候选参数时，
必须明确标为“临时/未验证”，不能在 handoff 中描述为已经完成实机验证。

## 14. 2026-10-14 当前未提交改动与脚本用途

以下是核对时工作区相对当前 HEAD 的已有差异，不代表本轮新增的固件修改。

| 文件 | 已有实际改动 |
|---|---|
| `Flux/inc/flux.h` | Rs 从 0.875 Ω 改为 0.07710429 Ω；Ls 从 200 µH 改为 14.8507 µH；Ld/Lq 从各 200 µH 改为 16.5559/13.1455 µH。为试用辨识候选值，未完成实机验证 |
| `UserSoftware/src/ParamId.c` | 删除辨识内部 2 A 门限及三相最大电流检查；全局保护仍存在。此次差异不是重新实现整套辨识算法 |
| `canFD/inc/canfd_app.h`、`canFD/src/canfd_app.c` | 版本从 260823A 改为 260830A；PDO1 固定 8 字节，发送 RPM/故障/模式/PWM 状态；SDO 接入真实 PLL/观测器参数，增加只读权限、有限值/范围/子索引/长度校验及类型错误响应；使用 memcpy 处理 float/u32 转换；速度限幅写入同步正负限幅和 PI 历史量 |
| `taskManager/taskManager.c` | VOFA 从 9 通道、40 字节改为三相电流 3 通道、16 字节 |
| `generate_can_xlsx.py` | 原为协议 Excel 生成辅助脚本；2026-10-14 按用户要求删除，不影响固件编译与运行 |
| `MDK-ARM/FOC_ST_V2.uvoptx` | 原有调试配置字段 nTsel 从 12 改为 3；本轮配置忽略规则后取消 Git 跟踪，磁盘文件保留 |
| `MDK-ARM/RTE/_FOC_ST_V2/RTE_Components.h` | 删除 CMSIS_device_header 宏及对应注释 |
| Keil 其他文件 | uvprojx 被 Git 标记修改，但当前文本 diff 无差异；另有未跟踪的 DebugConfig/FOC_ST_V2_STM32G474RETx_1.0.1.dbgconf。尚未判定调试工具配置变化是否需要保留 |
| `handoff.md` | 汇总当前 CAN 行为、辨识候选值及未验证状态；本轮补充架构、启动、母线适配、死区补偿规划 |

`generate_can_xlsx.py` 已按用户要求删除。它原先使用 openpyxl 生成 CAN 协议 Excel，内容由脚本中的列表手工维护，不自动解析源码，也不参与 STM32 固件编译或电机控制。已有 Excel 文件保留；协议维护以源码及本交接记录为准。

本次会话实际文件操作：将架构评审与规划归并到本文件，删除单独的 `Architecture_Startup_Deadtime_Plan_20261014.md`，并按后续用户要求删除 `generate_can_xlsx.py`。未修改固件或生成 Excel，未提交 Git、编译、烧录或运行电机。

2026-10-14 按用户要求完善 `.gitignore`：忽略 Keil 构建目录/产物、用户调试配置、缓存、临时文件及本地凭据；移除对 `.sct`、`.scvd` 的笼统忽略，保留工程使用的链接脚本、Event Recorder 描述、RTE 头文件、基准 dbgconf 和共享 VS Code JSON 配置。已跟踪的 Keil 用户配置及本地密钥取消 Git 跟踪，磁盘文件保留；既有 Git 历史未改写，未提交。

## 15. 2026-10-14 架构、启动与死区补偿规划（尚未实施）

### 15.1 用户确认与架构结论

电机为 **14 极对**，当前母线约 **12 V**；后续希望按实测母线电压调整调制与补偿。驱动器型号、额外硬件死区及负载/零速保持需求尚未确认。

主控制链：TIM1 中心对齐 PWM（约 20 kHz）→ CH4/CC4 触发 ADC 注入序列 → ADC1 完成回调 → Foc_Adc_Sample → Foc_Para_Calc → Foc_Control（保护、状态切换、模式运行）→ SVPWM → 统一 CCR 提交。TIM3 提供 1 ms 后台时基，主循环执行 USB/CAN、NTC、按键与 LCD 任务。

SENSORLESS 运行观测器、速度 PI 和电流 PI；VF 为开环压频；PREPOS 实际是 VF 驱动加观测器后台运行，并未实现完整初始定位；PARAM_ID 执行静止辨识。当前按键控制仍是 TODO。

**当前不适合从静止直接进入观测器角度闭环。** Hybrid 电流模型用估计角度构造磁链，磁链幅值收敛不代表已获得真实初始位置；IPD/HFI 仍为空函数。可以先闭合电流环，而角度由定位值或频率积分提供。

### 15.2 优先修复项

| 优先级 | 当前问题 | 计划 |
|---|---|---|
| P0 | flux.c 与 motorControl.c 中三处仍按 MOTOR_POLES/2 换算 | 统一极对数为 14；RPM=60*fe/14，fe=RPM*14/60 |
| P0 | 观测器将命令乘 Km*Vbus，缺少当前 SVPWM 的 1/√3 比例 | 由最终生效占空比重构物理电压，包含限幅后的输出 |
| P0 | Hybrid PLL 按角速度设计参数，却使用未归一化 Wb 误差并直接累加角度 | 误差按磁链幅值归一化；速度统一 rad/s；theta += omega*Ts；磁链过小时禁止判定估算有效 |
| P0 | 闭环先 Angel_Get，后 UVW_Axis_DQ，观测器读上一拍 Clarke/Id | 拆开 Clarke 与 Park，先更新本拍 αβ；Hybrid 使用本拍 αβ 和估计角度求 Id |
| P0 | PREPOS 固定调用电压型 Observer_Run，而闭环编译选择 Hybrid | 统一观测器 step 入口；同一模型后台预热与接管 |
| P0 | PREPOS 每拍覆盖估计速度；STOP→闭环无启动门槛；停机不复位磁链/补偿积分 | 独立保留估计输出；增加启动子状态、锁定与超时；静止重启明确 reset/seed |
| P1 | 观测器内部固定 0.3 V 死区处理，未补偿输出 PWM | 增加独立逆变器模型，共用于 PWM 补偿和观测器电压重构 |
| P1 | Km=12/Vbus 限制在 0.7～1.3，母线超出约 9.23～17.14 V 后不能维持命令电压 | 推荐内部使用伏特，在 PWM 层按实测 Vbus 归一化，保留电压/占空比限幅 |
| P1 | 速度环当前约 20 kHz，输出限幅 ±12 A；软件 ARR=3999，硬件 Period=4000 | 独立设计速度环周期并重调 Ki/限流；统一计数定义及最小脉冲/采样裕量 |

数值核算：当前 SVPWM 在线性区实际增益约为 Km*Vbus/√3。12 V、Ualpha=0.1、Ubeta=0 时，CCR 重构约 0.692 V，观测器当前按 1.2 V 积分。已做 360 角度扫描（幅值 0.4、含整型截断），与该比例的最大差约 0.000303*Vbus；这不是实机测试。

### 15.3 推荐启动方案

采用 **ALIGN → I/F_RAMP → OBS_VERIFY → BLEND → CLOSED**。外部 SENSORLESS 请求先进入启动流程；STOP/故障仍走统一封波。

- ALIGN：固定电角度，用电流环缓升定位电流，建立已知位置并初始化观测器磁链。
- I/F_RAMP：电流环控制 Id/Iq，角度由频率斜坡积分；速度环暂不接管。
- OBS_VERIFY：同一观测器独立后台估算，禁止每拍以开环频率覆盖速度；组合判断速度方向/差值、磁链幅值、归一化 PLL 误差、模型残差、饱和状态及连续满足时间。
- BLEND：沿最短角度差平滑接管；电流给定与速度 PI 积分预置，电流 PI 状态保留/重映射，避免电压突变。
- CLOSED：正常无感运行；低速退出有滞回，失锁/启动超时/堵转有明确停机或退出策略。

不能仅用磁链幅值约等于 0.00752 Wb 或人为覆盖后的速度作为锁定依据。10～20 Hz 电频率可用于首次轻载扫描，但不是保证阈值；14 极对下分别约 42.9～85.7 RPM。零速大负载或零速保持另行评估 IPD/HFI 或位置传感器。

### 15.4 死区补偿接入及母线适配

最小接入位置为 `PWM_Update_From_SVPWM()`：原始三相 CCR → 死区补偿 → 最小脉冲/采样窗口及最终限幅 → `PWM_SetDuty()`。VF、PREPOS、FOC 共用，辨识模式保留独立补偿开关并重新验证参数。

约定电流正方向为逆变器流向电机、上桥臂 duty 增大提高平均相电压时：

```text
Verr_x = Vbus * (t_eff/Tpwm) * g(ix)
duty_out_x = duty_ideal_x + (t_eff/Tpwm) * g(ix)
Vactual_ab = Clarke(Vbus*duty_final_abc) - Clarke(Verr_abc)
```

g(i) 使用滞回或平滑过零，阈值依据 ADC 噪声/纹波标定。电流方向、相序、PWM 极性和补偿符号需实测核对。输出已加补偿，观测器重构后要减去物理死区误差，不能再加一次补偿。

现有 TIM1=160 MHz、CKD=DIV1、DTG=40，基线死区约 250 ns；PWM 周期约 50 µs，因此 Δduty≈0.005、ΔCCR≈20。中心对齐不能直接将 CCR 加 40。相死区电压在 12 V 时约 0.06 V，24 V 时约 0.12 V；这是 MCU 基线，不含未知驱动器延时与器件压降。保留硬件死区，通过软件修正平均电压。

纯死区项 duty 补偿基本不随 Vbus 变化，其等效电压随实测 Vbus 成正比；器件固定压降项需除以 Vbus 换成 duty。推荐 PI/逆变器模型内部使用 A、V、Wb、rad/s，保留当前 SVPWM 内核时在 PWM 层令 `Uab=sqrt(3)*Vab_request/Vbus_measured`、`Km=1`，取消旧的 12/Vbus 前馈，避免重复归一化。迁移伏特输出时必须同步换算/重调 PI 增益、积分状态和 VF 电压给定。

区分原始、补偿后、最终限幅及实际生效 duty；电压重构必须匹配 ADC 采样对应的 PWM 作用区间。当前 PREPOS 先计算新 PWM 再估算，应改为先估算、后提交输出。PWM preload 与 ADC 完成时间需示波器验证，不能一律把当前或上一拍软件 CCR 当作已作用电压。

### 15.5 实施与验证顺序

1. 极对数/电压/PLL 量纲一致，更新采样顺序、统一观测器入口与复位。
2. 加共用逆变器模型、输出补偿、最终 duty 记录和实际电压重构；按实测 Vbus 归一化。
3. 实现定位/I-F/锁定判定/平滑接管/超时；先轻载多初始位置重复启动，再逐步带载。
4. 测 CH4 触发、ADC 完成、CCR 生效与桥臂边沿，确认补偿符号及低侧电流采样窗口；最后调电流环、PLL、速度环与低速退出。

调试通道需增加 Vbus、开环/估计角度及速度、磁链幅值、PLL 误差、原始/生效 duty、补偿电压、启动子状态与锁定计数，按同一控制快照采集。

本轮只做源码评审和数学核算，以上固件方案均未实施，未重新构建或完成实机验证。

参考：[ST AN4013 死区编码](https://www.st.com/resource/en/application_note/an4013-stm32-crossseries-timer-overview-stmicroelectronics.pdf)、[MathWorks I/F 无感启动](https://www.mathworks.com/help/mcb/gs/sensorless-foc-of-pmsm-using-if-ctrl-startup.html)、[I-f 启动与平滑切换论文](https://www.jstage.jst.go.jp/article/ieejjia/9/3/9_263/_article)。

## 16. 2026-10-19 零目标切模式 2 后自行旋转的排查

用户反馈并确认：进入 Debug 后把 Control_Mode 改为 2，再运行，电机持续旋转；SpeedRpm_GXieLv.XieLv_X 为 0，motor.SpeedRPM 显示 4000 多。没有独立机械测速结果。

当前源码核算：

- 模式 2 从 STOP 直接开启 PWM 并运行观测器、速度环与电流环，没有定位/锁定门槛。
- PLL_Ui 限幅为 ±2π×500×50µs，约 ±0.1570796 rad/周期；速度滤波正常从 0 初始化时，电频率上限趋近 500 Hz。
- 当前错误的 RPM 换算使用 14/2，因此 500 Hz 对应约 4285.7 RPM。已确认“4000 多”指 motor.SpeedRPM，与 PLL 接近正向限幅相符，优先怀疑失锁/自激，不能视为独立实测机械转速。需用 speed_hz 和 PLL_Ui 的实际值进一步确认是否顶住限幅。
- 当速度环 Ref=0、Fbk≈+4000 RPM 时，Kp=0.01 的比例项约为 -40 A，正常零积分起点下很快触及 -12 A 输出限幅；OutF 经滤波趋向负限幅。Foc.c 的 SPEED_IQ_SIGN=-1 随后将其变成趋向 +12 A 的 Iq 给定。因此目标为 0 仍可能输出较大转矩电流命令；给定值不是实际电流测量值。
- 上述负号存在速度环反馈方向错误的嫌疑，但是否构成实际正反馈还取决于电流极性、相序、估计角度及正 Iq 对真实转速的作用方向，不能未验证就简单翻转符号。

优先核对已有 Watch 数据：motor.SpeedRPM、Foc_observer.speed_hz、PLL_Ui、PLL_Interg、pi_spd.Ref/Fbk/OutF、I_q_GXieLv.XieLv_Y 和 PARK_PCurr.Qs。先将模式设为 STOP 并让程序执行封波；仅暂停 CPU 可能保留定时器 PWM，不能当作停机操作。

本次仅补充排查记录，未更改控制源码，也未访问实机调试器。后续修复顺序：隔离未可信速度对转矩给定的影响 → 核对反馈方向 → 修正极对数、PLL 量纲及电压模型 → 验证启动/锁定流程。把显示转速除以 2 本身不能解决自行旋转。
