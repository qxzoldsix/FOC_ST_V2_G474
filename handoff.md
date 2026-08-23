# FOC_ST_V2 CAN 控制通信 Handoff

> 核对日期：2026-08-23
>
> 核对对象：当前工作区代码（包含尚未提交的本地修改），不是 2026-07-23 提交时的旧协议
>
> Git 基线：`b7297bd203cf15b00625ec429b2e47507a3aad28`（普通can暂时可用）
>
> MCU：STM32G474RE；节点号：`0x01`；软件版本：`260823A`

## 1. 结论先行

当前软件使用 **经典 CAN**，不是 CAN FD：11 位标准帧、1 Mbps、单帧最多 8 字节、STM32 小端、`float` 按 IEEE-754 binary32 直接传输。

已具备：

- FDCAN1 正常模式初始化、全标准 ID 接收过滤、RX FIFO0 中断接收。
- NMT、PDO1 控制、PDO2 占位控制、SDO 参数读写、心跳和周期状态上报。
- CAN STOP/QUICK_STOP 会立即进入 STOP 并封锁三相 PWM。
- START 仅在 NMT `OPERATIONAL` 状态下接受，并检查模式、转速参数和故障状态。

当前不能按“完整 CAN 电机控制器”交付：

- `0x181` 实际只发送 `SpeedRPM`、`CurrentHz`；故障码、控制模式、电压等后续字段没有上总线。
- `0x281` 实际只发送 `Ia`、`Ib`；`Ic/Vbus/Id/Iq` 没有上总线。
- CAN START 帧不含 `TargetVolt`，工程内也没有其他 CAN 写入口；`motor.TargetVolt` 上电为 0。当前安全检查会直接拒绝 VF/PREPOS 启动，不再带着无效电压开启功率级。
- `0x301` 的 Id/Iq 目标帧能接收，但回调没有应用数据。
- EMCY 发送函数已写，但故障保护没有调用它；CAN 上目前不能自动看到故障码。
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
| `0x181` | 节点→主机 | 8 | 10 ms | `SpeedRPM` + `CurrentHz` | 已实现 |
| `0x201` | 主机→节点 | 1 或 6~8 | 事件 | STOP/CLEAR/QUICK_STOP 或 START | 已实现 |
| `0x281` | 节点→主机 | 8 | 50 ms | `Ia` + `Ib` | 已实现 |
| `0x301` | 主机→节点 | 8 | 事件 | `Id_Target` + `Iq_Target` | 可接收但不生效 |
| `0x581` | 节点→主机 | 8 | SDO 应答 | 读、写、错误响应 | 已实现但存在占位项 |
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
| 1 | `uint8` | `Control_Mode` | 外部 START 当前接受 2~4 |
| 2~5 | `float32 LE` | `TargetSpeedRPM` | 必须有限且 `>=0` |

模式：0=STOP，1=SENSOR，2=SENSORLESS，3=VF，4=PREPOS，5=参数辨识。模式 1 尚未接入位置传感器，当前状态机和外部 START 都会拒绝它；模式 5 仅供内部参数辨识流程使用。

目标电频率按当前代码计算：

`TargetHz = TargetSpeedRPM × (MOTOR_POLES / 2) / 60`

当前 `MOTOR_POLES=14`，所以 `TargetHz = RPM × 7 / 60`。代码注释把该宏称为“极对数”，但公式又除以 2；主机应以公式的实际行为为准。

示例：START + VF + 1000 RPM：ID `201`，DLC 6，数据 `01 03 00 00 7A 44`。固件计算 `TargetHz≈116.667 Hz`。由于该帧不会设置 `TargetVolt`，上电默认值为 0 时此命令会被安全拒绝，不会开桥。

STOP、FAULT_CLEAR、QUICK_STOP 只需要 Byte0：

| 数据 | 动作 | NMT 状态限制 |
|---|---|---|
| `00` | STOP，清零目标/当前频率并封锁 PWM | 无，任何状态接受 |
| `02` | 清除可清除故障 | 无，任何状态接受 |
| `04` | QUICK_STOP；当前实现与 STOP 相同 | 无，任何状态接受 |

START 的接受条件全部满足才执行：节点为 OPERATIONAL、DLC≥6、模式 2~4、RPM 有限且非负、换算电频率不超过 400 Hz、当前无故障；VF/PREPOS 还要求 `0.01 < TargetVolt <= 0.95 pu`。拒绝时没有 CAN 应答。

### 5.3 PDO1 TX 状态：0x181，DLC 8

| Byte | 类型 | 字段 | 单位 |
|---:|---|---|---|
| 0~3 | `float32 LE` | `motor.SpeedRPM` | RPM |
| 4~7 | `float32 LE` | `motor.CurrentHz` | Hz |

头文件仍定义了 33 字节完整结构，但发送函数明确只传前 8 字节。`V_amp/TargetHz/TargetVolt/Fault_DTC/Control_Mode/IQAngle/V_d/V_q` 当前均不在 CAN 报文中。

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
| 3 | 子索引，当前忽略/返回 0 |
| 4~7 | 32 位原始值，小端 |

命令字：读请求 `0x40`；读响应 `0x43`；写 32/16/8 请求分别为 `0x23/0x2B/0x2F`；写成功 `0x60`；错误 `0x80`。

当前实现的写路径始终读取 Byte4~7 并按 32 位值处理，因此即使命令字写成 16/8 位，也应发送 DLC 8；实际有效的浮点写操作应使用 `0x23`。

| 索引 | 名称 | 读值/写入对象 | 当前写入效果 |
|---:|---|---|---|
| `0x0000` | SPEED_KP | `pi_spd.Kp`，float | 生效 |
| `0x0001` | SPEED_KI | `pi_spd.Ki`，float | 生效 |
| `0x0002` | SPEED_UMAX | `pi_spd.Umax`，float | 生效；不会同步 Umin |
| `0x0003` | ID_KP | `pi_id.Kp`，float | 生效 |
| `0x0004` | ID_KI | `pi_id.Ki`，float | 生效 |
| `0x0005` | ID_UMAX | `pi_id.Umax`，float | 生效；不会同步 Umin |
| `0x0006` | IQ_KP | `pi_iq.Kp`，float | 生效 |
| `0x0007` | IQ_KI | `pi_iq.Ki`，float | 生效 |
| `0x0008` | IQ_UMAX | `pi_iq.Umax`，float | 会写入，但运行中被电压圆约束持续覆盖 |
| `0x0009` | PLL_KP | 实际错误映射到 `VF_HzRamp.XieLv_Y` | 不应使用 |
| `0x000A` | PLL_KI | 实际错误映射到 `VF_HzRamp.XieLv_X` | 不应使用 |
| `0x000B` | OBS_GAIN | 读取原始整数 5000，不是 float 位型 | 写入返回成功但不生效 |
| `0x000C` | OBS_BW | 读取原始整数 50，不是 float 位型 | 写入返回成功但不生效 |
| `0x000D` | OC_THRESH | 20.0f | 写入返回成功但宏不变 |
| `0x000E` | OV_THRESH | 30.0f | 写入返回成功但宏不变 |
| `0x000F` | UV_THRESH | 8.0f | 写入返回成功但宏不变 |
| `0x0010` | OC_BUS_THRESH | 15.0f | 写入返回成功但宏不变 |
| `0x0011` | OT_THRESH | 原始整数 0 | 写入返回成功但不生效 |
| `0x0012` | VF_VOLT_MAX | 0.95f (pu) | 写入返回成功但宏不变 |
| `0x0013` | VF_FREQ_MAX | 400.0f | 写入返回成功但宏不变 |
| `0x0014` | VF_RAMP_GRAD | 0.01f | 写入返回成功但宏/运行变量不变 |
| `0x1000` | DEVICE_ID | `0x20260718` | 只读，写返回错误 |
| `0x1001` | SW_VERSION | `0x0260823A`（版本 `260823A`） | 只读，写返回错误 |
| `0x1002` | NODE_ID | 原始整数 1 | 写返回成功但节点号不变 |

PCAN-View 读取版本：发送 `601#40 01 10 00 00 00 00 00`，正常响应为
`581#43 01 10 00 3A 82 60 02`，数据区 `3A 82 60 02` 按小端解释为
`0x0260823A`，对应版本 `260823A`。

错误码：不存在索引 `0x06020000`；只读 `0x06010002`。`0x06090030` 虽有定义，但当前没有范围校验路径会返回它。

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

FAULT_CLEAR 会先复查实时故障源；过流、欠压、过压或过温仍存在时拒绝清除。可清除故障在本次上电第 6 次成功清除时会增加 `FAULT_REPEAT_OC (0x0800)` 锁存位。

重要限制：实际 `0x181` 不含 `Fault_DTC`，保护也不自动发 EMCY，SDO 里没有故障寄存器。因此当前主机无法仅通过已上线协议可靠读取故障码。

## 8. 推荐主机控制顺序（按当前实现）

1. 上电后确认收到 `0x701 = 05`，并观察 `0x181/0x281`。
2. 需要明确恢复 OPERATIONAL 时发送 `000#0101`。
3. 在电机未连接或功率级安全条件下，先验证 STOP：`201#00`，确认 PWM 硬件封锁。
4. START 使用 `0x201` 六字节帧；主机必须自行处理“无 ACK”的事实。
5. 任何异常立即发 `201#04` 或 `201#00`；两者当前效果相同。
6. 清故障前先去除欠压/过压/过流/过温源，再发 `201#02`。

在补齐 `TargetVolt` 通信入口之前，VF/PREPOS 的 CAN 启动会被拒绝。模式 1 尚未接入传感器；模式 2 没有强制的 VF→PREPOS→闭环切换门槛，主机直接切入的风险由主机承担。VF→PREPOS→SENSORLESS 等运行模式切换现在保持 PWM 连续，只在 STOP→运行时开启一次输出，并在进入闭环时清 PI 历史量、用当前速度接管斜坡。

## 9. 调试观测点

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
| `motor.Fault_DTC` | 当前故障位掩码；目前仅调试器可可靠读取 |
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
3. 重新设计经典 CAN 状态分帧，使 Fault_DTC、模式、Vbus、Id/Iq 真正上线。
4. 故障首次出现时调用 EMCY，并增加可读故障寄存器。
5. 把 SDO 的 PLL/观测器/阈值占位映射改成真实运行变量；无效写入应返回错误，不能伪成功。
6. 让 PDO2 RX 真正接入电流目标，或删除该对外接口。
7. 为模式 1/2 增加启动前置条件与闭环切换门槛。
8. 增加 CAN RX 超时安全停机和命令序号/看门狗。

## 12. 本次核对边界

已做代码级交叉核对：FDCAN 初始化、协议头/实现、RX 中断、任务周期、控制回调、PWM 启停、保护、V/F、FOC、观测器参数及 Keil 工程文件引用。

2026-08-23 已使用本机 Keil ARMCC V5.05 update 1 对 `MDK-ARM/FOC_ST_V2.uvprojx` 完成全量构建：`0 Error(s), 0 Warning(s)`，AXF/HEX 生成成功（Code=47276，RO-data=19416，RW-data=492，ZI-data=9676）。

当前仍未连接 PCAN 和电机台架，因此不宣称已完成烧录或实机回归。交付前至少执行：PCAN 1 Mbps 收发、STOP/MOE 硬件封波、START 拒绝条件、VF→PREPOS→SENSORLESS 无扰切换、故障锁存与清除、连续帧 FIFO 排空和断总线安全性测试。
