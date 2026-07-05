# FOC_ST_V2 — STM32G474 无感 FOC 电机控制器

> **Author**: nono &lt;nono_1007@foxmail.com&gt;  
> **MCU**: STM32G474RE | **IDE**: Keil MDK-ARM | **生成工具**: STM32CubeMX  
> **控制模式**: 无感 FOC (Sensorless) / VF 开环 / PREPOS 预定位  
> **位置估算**: 电压模型磁链观测器 + PLL 锁相环  
> **调制方式**: SVPWM (7段式, 20kHz)  
> **状态**: ✅ 闭环可运行

---

## 1. 硬件环境

| 项目 | 参数 |
|------|------|
| MCU | STM32G474RE (Cortex-M4F, 170MHz) |
| 驱动芯片 | DRV8301 / 兼容 (6路 PWM 互补输出) |
| 电流采样 | 三电阻 shunt，12位 ADC 注入组 |
| 母线电压采样 | 电阻分压 + ADC |
| 显示 | ST7789 240×135 SPI LCD |
| 按键 | 4 路轻触按键 (PC9/PC8/PC7/PC6) |
| 通信 | USB-CDC (Vofa JustFloat 协议)、FDCAN |
| 时钟 | HSE 8MHz → PLL → 160MHz SYSCLK |

---

## 2. 目录结构

```
FOC_ST_V2_0611/
├── Core/                   # CubeMX HAL 初始化代码
│   ├── Inc/                # main.h, adc.h, tim.h, gpio.h ...
│   └── Src/                # main.c, stm32g4xx_it.c ...
├── Drivers/                # CMSIS + HAL 库 (STM32G4xx)
├── Flux/                   # 无感磁链观测器
│   ├── inc/flux.h          # 电机参数、观测器结构体、API
│   └── src/flux.c          # Observer_Run + 死区补偿 + PLL
├── Foc_Control/            # FOC 核心变换 + VF 控制
│   ├── inc/Foc.h, VF.h
│   └── src/Foc.c, VF.c
├── UserSoftware/           # 用户层软件
│   ├── inc/                # ADC_SAMPLE, PI_Cale, svpwm, Common_Math, motorControl, DrvPwmout, InvProtect
│   └── src/                # 对应 .c 实现
├── taskManager/            # 时间片任务调度器
├── Key/                    # 按键驱动 (单/双/长按)
├── LCD/                    # ST7789 LCD 驱动 + 显示
├── pmsmConfig_Inc/         # 全局头文件汇总 (headline.h)
├── vofa/                   # Vofa+ 上位机通信 (预留)
├── Middlewares/             # USB 协议栈
├── USB_Device/              # USB CDC 驱动
├── FOC_ST_V2.ioc           # CubeMX 工程文件
├── MDK-ARM/                # Keil 工程文件
├── README.md               # 本文件
├── Current_Tuning_Guide.md # 闭环调试全记录
└── Observer_Debug_Guide.md # 观测器调试指南
```

---

## 3. 控制模式

`motor.Control_Mode` 切换（按键 KEY2 循环切换）：

| Mode | 名称 | 说明 | 状态 |
|------|------|------|------|
| 0 | **STOP** | 停机，关闭 PWM 输出 | ✅ |
| 1 | **SENSOR** | 有感 FOC（霍尔/编码器） | ❌ 未实现 |
| 2 | **SENSORLESS** | 无感 FOC 闭环（观测器+PLL+速度/电流双环） | ✅ |
| 3 | **VF** | V/F 开环压频控制 | ✅ |
| 4 | **PREPOS** | VF 驱动 + 观测器后台预热（切闭环过渡模式） | ✅ |

### 模式切换流程

```
STOP(0) → VF(3) → PREPOS(4) → SENSORLESS(2) → STOP(0) → ...
```

**推荐启动流程**:
1. KEY1 启动 → 进入 Mode 3 (VF)
2. KEY3/KEY4 调节 targetHz
3. KEY2 → Mode 4 (PREPOS) 预热观测器
4. 等 LCD 显示 Psi ≈ 0.0075
5. KEY2 → Mode 2 (闭环)

---

## 4. 工作流程

### 4.1 初始化流程 (`main.c`)

```
HAL_Init() → SystemClock_Config(160MHz)
→ MX_GPIO/DMA/ADC1/ADC2/TIM1/TIM3/SPI3/USART3/USB_Init()
→ LCD_Init() → key_driver_init()
→ Task_Manage_List_Init()
→ HAL_TIM_Base_Start_IT(&htim1)
→ ADC 校准 + 注入组启动
→ PMSM_init()              ← 电流零点校准
→ Flux_Observer_Init()     ← 观测器参数初始化
→ PI_Init()                ← PI 参数初始化
→ Foc_Pwm_Start()          ← 使能 PWM
→ while(1): Execute_Task_List_RUN()
```

### 4.2 实时中断链路 (20kHz)

```
TIM1 PWM 周期中断
  └─→ ADC 注入组 (由 TIM1 触发)
       └─→ HAL_ADCEx_InjectedConvCpltCallback (RAM_FUNC)
            ├─ Foc_Adc_Sample()  ← 读 ADC → 三相电流/母线电压
            ├─ Foc_Para_Calc()   ← 母线电压 + SVPWM 前馈系数
            └─ Foc_Control()     ← FOC 状态机
```

### 4.3 后台任务 (TIM3 1ms 时基)

| 任务 | 周期 | 功能 |
|------|------|------|
| HFPeriod_RUN | 1ms | Vofa JustFloat 数据上报 |
| task_send_Rece | 8ms | LED 闪烁 |
| Balance_Control | 15ms | 空（预留） |
| KEY_RUN | 80ms | 按键扫描 + 模式/频率控制 |
| Task_DEBUG | 600ms | LCD 刷新 |

---

## 5. 核心算法

### 5.1 坐标变换链路

```
Ia, Ib, Ic  (三相电流)
  └→ Clarke (等幅值) →  Iα, Iβ
       └→ Park         →  Id, Iq  (旋转 dq 坐标系)
            └→ PI 电流环 →  Vd, Vq
                 └─→ PI 速度环 → Iq 给定
                 └→ iPark      →  Vα, Vβ
                      └→ SVPWM → TIM1 CCR
```

### 5.2 磁链观测器 (Flux Observer)

电压模型 + 死区补偿 + PLL 锁相环:

```
1. Deadtime_Comp_ab() → 命令电压 → 真实电压(V)
2. ψr = ψ_total - Ls·I          ← 转子磁链
3. Err = |ψ_ref|² - |ψr|²       ← 幅值误差
4. PLL: θ += Kp·(ψrβ·cosθ - ψrα·sinθ) + ∫Ki·Err
5. ψ_total += (V - Rs·I + ψr·Err·Gain) · Ts
```

**关键特性**:
- 死区电压补偿 (V_DEADTIME = 0.3V)
- PLL 积分抗饱和 (Limit_Sat)
- 磁链误差同周期反馈 (消除一周期延迟)
- PLL 增益自动 ramp 恢复

### 5.3 PI 控制器

- 积分分离抗饱和 (anti-windup)
- 一阶低通滤波: `OutF = 0.9·OutF_old + 0.1·Out`
- 斜坡发生器: `Grad_XieLv` 线性梯度逼近
- 电压圆约束: q 轴电压限幅由 Vd² + Vq² ≤ Vbus² 动态计算

### 5.4 V/F 控制

- 开环电压-频率比: `V = Vmin + (Vtrg-Vmin) × (F/Ftrg)`
- 角度由频率积分: `θ += 2π·F·Ts`
- 频率斜坡: `Grad_XieLv` 平滑过渡

---

## 6. 电机参数

```c
// Flux/inc/flux.h
#define MOTOR_RS    0.007525f   // 定子电阻 (Ω) — 万用表实测
#define MOTOR_LS    0.00002f    // 定子电感 (H) — ⚠️ 待 LCR 表复测
#define MOTOR_FLUX  0.00752f    // 永磁体磁链 (Wb)
#define MOTOR_POLES 52          // 极对数
```

---

## 7. PI 参数 (当前值)

| 环 | Kp | Ki | Umax |
|----|-----|------|------|
| 速度环 `pi_spd` | 0.005 | 0.00005 | **3.0 A** |
| d 轴电流 `pi_id` | 0.005 | 0.000005 | **1.0 pu** |
| q 轴电流 `pi_iq` | 0.005 | 0.000005 | **1.0 pu** |

> 文件: `UserSoftware/src/PI_Cale.c` → `PI_Init()`

---

## 8. 按键功能

| 按键 | 引脚 | 短按 | 长按 |
|------|------|------|------|
| KEY1 | PC9 | 启动/停止 (VF ↔ STOP) | 清除故障 |
| KEY2 | PC8 | 模式循环 (STOP→VF→PREPOS→Sensorless) | — |
| KEY3 | PC7 | TargetHz +5 | 连续 +5 |
| KEY4 | PC6 | TargetHz -5 | 连续 -5 |

---

## 9. Vofa+ 上位机监控

- 协议: JustFloat | 帧长: 28 字节 (6×float + 00 00 80 7F) | 接口: USB-CDC

| CH | 变量 | 含义 | 正常值 |
|----|------|------|--------|
| CH1 | `motor.CurrentHz` | 当前电频率 (Hz) | 跟踪目标 |
| CH2 | `PARK_PCurr.Ds` | d 轴电流 Id (A) | ≈ 0 |
| CH3 | `PARK_PCurr.Qs` | q 轴电流 Iq (A) | 稳定 |
| CH4 | `motor.V_d` | d 轴电压 (pu) | ≈ 0 |
| CH5 | `motor.V_q` | q 轴电压 (pu) | < 0.8 |
| CH6 | `BUS_Voltage` | 母线电压 (V) | ≈ 24V |

---

## 10. LCD 显示

```
┌──────────────────────────────┐
│  [MODE]             BUS:24V  │  顶部: 模式 + 母线电压
│  Target:xx.xHz   V:xx.xxx    │  二行: 目标频率 + 当前电压
│  [      MODE       ]         │  三行: 模式大字
│  FHz:xx.x  (当前频率)        │
│  PLL:x.xxx (PLL 误差)        │  数据行
│  Est:xx.x  (估算频率)        │
│  Psi:x.xxxx(磁链幅值)        │
└──────────────────────────────┘
```

---

## 11. 保护功能

| 保护项 | 阈值 | 动作 |
|--------|------|------|
| 相过流 | ±20A | 锁存故障 + 停机 + 关 PWM |
| 母线过流 | ±15A | 锁存故障 + 停机 + 关 PWM |

> 长按 KEY1 清除故障锁存。

---

## 12. 编译与烧录

1. 打开 `MDK-ARM/FOC_ST_V2.uvprojx` (Keil MDK 5.38+)
2. 编译 F7 → 烧录 F8
3. 连接 USB-CDC 查看 Vofa 数据
4. LCD 显示运行状态

---

## 13. 调试参数速查

| 参数 | 位置 | 说明 |
|------|------|------|
| `MOTOR_RS/LS/FLUX/POLES` | `Flux/inc/flux.h` | 电机参数 |
| `pi_spd/pi_id/pi_iq` | `UserSoftware/src/PI_Cale.c` | PI 增益 |
| `Foc_observer.Gain` | `Flux/src/flux.c` | 磁链补偿增益 (5000) |
| `Foc_observer.PLL_kp/ki` | `Flux/src/flux.c` | PLL 增益 (20/10) |
| `VF_VOLTAGE_MIN/MAX` | `Foc_Control/inc/VF.h` | VF 电压范围 |
| `OC_THRESHOLD_A` | `UserSoftware/inc/InvProtect.h` | 过流阈值 (20A) |
| `targetVolt` | 运行时设置 | VF 目标电压 (推荐 ≥0.10 for 25Hz) |
| `VF_TO_SENSORLESS_HZ` | `UserSoftware/src/motorControl.c` | 切闭环最低频率 (10Hz) |

---

## 14. 相关文档

| 文档 | 说明 |
|------|------|
| `Current_Tuning_Guide.md` | 闭环调试全记录 (爆鸣→平稳运行) |
| `Observer_Debug_Guide.md` | 观测器调试指南 |
| `VF_Analysis_Report.md` | VF 控制代码分析 |
| `Observer_Problem_Analysis.md` | 观测器问题分析 |
