# FDCAN 调试 Handoff — 2026-07-23

## 现状

经典 CAN 通信已调通，PCAN-View 可收到 STM32G474 发送的 CAN 帧。

## 当前配置

| 参数 | 值 |
|---|---|
| Mode | `FDCAN_MODE_NORMAL` |
| FrameFormat | `FDCAN_FRAME_CLASSIC` (经典CAN, ≤8字节/帧) |
| 时钟 | PCLK1 = 160MHz, CKDIV=0 (÷1) |
| Nominal 波特率 | **1 Mbps** (Prescaler=4, TSeg1=31, TSeg2=8) |
| Nominal tq | 1+31+8 = 40, 160M/(4×40) = 1Mbps ✓ |
| TX FIFO | 3 元素 (`TXBC = 0x03000000`) |
| RX FIFO0 | 3 元素 (`RXF0S = 0x00030000`) |
| Filter | Std ID 0x000~0x7FF → RX FIFO0 |

## 当前发送的 CAN 帧

| CAN ID | 内容 | 周期 | 字节数 |
|---|---|---|---|
| `0x181` | PDO1 前8字节 (SpeedRPM + CurrentHz) | 10ms | 8 |
| `0x281` | PDO2 前8字节 (Ia + Ib) | 50ms | 8 |
| `0x701` | Heartbeat (NodeState) | 50ms | 1 |

## 今天修复的问题

### 1. DLC 编码 bug（canfd_app.c `CANFD_SendMessage`）
- **原代码**: `txHeader.DataLength = len << 16;`
- **问题**: HAL 的 DLC 值是 0~15 的枚举码（如 `FDCAN_DLC_BYTES_8 = 8`），不是移位值。`33<<16=0x210000` 作为 `DLCtoBytes[]` 数组下标严重越界，破坏 SRAMCAN → HardFault
- **修复**: 添加 `CANFD_BytesToDLC()` 函数，将字节数映射为正确的 DLC 码

### 2. TX FIFO 无元素（fdcan.c `MX_FDCAN1_Init`）
- **问题**: CubeMX 生成的 HAL 代码未写入 `TXBC.TFQS`，寄存器保持复位值 0 → 0 个 TX 缓冲区 → FIFO 永远满
- **修复**: 手动 `hfdcan1.Instance->TXBC = (3U << 24U)`

### 3. RX FIFO 未配置
- **问题**: `RXF0S`/`RXF1S`/`TXEFS` 也未写入（读写共用地址，读=状态，写=配置，不能用 MODIFY_REG）
- **修复**: 直接赋值 `RXF0S = (3U << 16U)`, `RXF1S = 0`, `TXEFS = 0`

### 4. PCAN 硬件限制
- 用户用的是经典 PCAN-USB（非 FD），最高 1Mbps，不支持 CAN FD 帧
- 因此 `FrameFormat` 切回 `FDCAN_FRAME_CLASSIC`，PDO 暂时只发前 8 字节

## 可用的调试计数器

在 Watch 窗口直接输入变量名即可查看：

| 变量 | 含义 |
|---|---|
| `canfd_tx_cnt` | 已发送帧数 |
| `canfd_rx_cnt` | 已接收帧数 |
| `canfd_err_cnt` | 发送失败次数（FIFO满/未启动） |
| `hfdcan1.ErrorCode` | HAL 错误码（0x200=FIFO满, 0x8=未启动） |
| `hfdcan1.Instance->TXBC` | TXBC 寄存器（应为 0x03000000） |

## CubeMX 重新生成时的注意事项

`fdcan.c` 中 3 处手动修改会被 CubeMX 覆盖，重新生成后需恢复：

1. **FrameFormat** → 如果后续上 CAN FD 硬件，改为 `FDCAN_FRAME_FD_BRS`
2. **Mode** → 根据需求选 `NORMAL` / `INTERNAL_LOOPBACK`
3. **FDCAN1_Init_2** 里的 4 行 FIFO 配置必须保留：
   ```c
   hfdcan1.Instance->TXBC  = (3U << 24U);
   hfdcan1.Instance->RXF0S = (3U << 16U);
   hfdcan1.Instance->RXF1S = 0U;
   hfdcan1.Instance->TXEFS = 0U;
   ```

## TODO / 后续

- [ ] PDO1 (33字节) 需要拆成多个经典 CAN 帧（每个 ≤8 字节），设计分包协议
- [ ] PDO2 (24字节) 同理
- [ ] 如果换 PCAN-USB FD 硬件：恢复 `FDCAN_FRAME_FD_BRS` + `is_fd=1`，PDO 可直接发完整 33/24 字节
- [ ] NTC 过温保护逻辑待实现
- [ ] KEY 按键功能待实现
