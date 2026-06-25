# FOC_ST_V2 — STM32G474 双模 FOC 电机控制器

> **MCU**: STM32G474RE | **IDE**: Keil MDK-ARM | **生成工具**: STM32CubeMX  
> **控制模式**: 无感 FOC (Sensorless) / VF 开环 (V/F Open-Loop)  
> **位置估算**: 磁链观测器 + PLL 锁相环  
> **调制方式**: SVPWM (7段式, 20kHz)

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
FOC_ST_V2_0610/
├── Core/                   # CubeMX 生成的 HAL 初始化代码
│   ├── Inc/                # main.h, adc.h, tim.h, gpio.h ...
│   └── Src/                # main.c, stm32g4xx_it.c ...
├── Drivers/                # CMSIS + HAL 库 (STM32G4xx)
├── Flux/                   # 无感磁链观测器
│   ├── inc/flux.h
│   └── src/flux.c
├── Foc_Control/            # FOC 核心变换 + VF 控制
│   ├── inc/Foc.h, VF.h
│   └── src/Foc.c, VF.c
├── UserSoftware/           # 用户层软件
│   ├── inc/                # ADC_SAMPLE, PI_Cale, svpwm, Common_Math, motorControl, DrvPwmout, InvProtect
│   └── src/                # 对应 .c 实现
├── taskManager/            # 简单时间片任务调度器
│   ├── taskManager.h
│   └── taskManager.c
├── Key/                    # 按键驱动 (单/双/长按)
│   ├── key_bsp.h/c
│   └── key_driver.h/c
├── LCD/                    # ST7789 LCD 驱动 + 显示
│   ├── lcd.h/c
│   ├── lcdfont.h
│   └── display.h/c
├── pmsmConfig_Inc/         # 全局头文件汇总
│   └── headline.h
├── vofa/                   # Vofa+ 上位机通信
├── Middlewares/             # USB 协议栈
├── USB_Device/              # USB CDC 驱动
├── FOC_ST_V2.ioc           # CubeMX 工程文件
├── MDK-ARM/                # Keil 工程文件
└── VF_Analysis_Report.md   # VF 分析报告
```

---

## 3. 控制模式说明

进入 `motor.Control_Mode` 切换：

| Mode | 名称 | 说明 | 状态 |
|------|------|------|------|
| 0 | **STOP** | 停机，关闭 PWM | ✅ 可用 |
| 1 | **READY** | 输出零矢量（SVPWM 零矢量） | ✅ 可用 |
| 2 | **SENSOR** | 有感 FOC（霍尔/编码器） | ❌ 未实现 |
| 3 | **SENSORLESS** | 无感 FOC（磁链观测器 + PLL） | ⚠️ 速度环可用，电流环/角度估算待修复 |
| 4 | **VF** | V/F 开环压频控制 | ✅ 可用 |
| 5 | **PRE-POSITION** | 预定位（IPD） | ❌ 未实现 |

---

## 4. 工作流程

### 4.1 初始化流程 (`main.c`)

```
HAL_Init() → SystemClock_Config(160MHz)
→ MX_GPIO/DMA/ADC1/ADC2/TIM1/TIM3/SPI3/USART3/USB_Init()
→ LCD_Init()
→ key_driver_init()
→ Task_Manage_List_Init()     ← 初始化 5 个后台任务
→ HAL_TIM_Base_Start_IT(&htim1)  ← TIM1 触发 ADC 注入组采样
→ ADC 校准 + 注入组/ADC_DMA 启动
→ Foc_Pwm_Start()             ← 使能 6 路互补 PWM
→ PMSM_init()                 ← 电流零点校准
→ Flux_Observer_Init()        ← 磁链观测器参数初始化
→ while(1): Execute_Task_List_RUN()
```

### 4.2 实时中断链路

```
TIM1 (20kHz PWM)
  └─→ ADC 注入组 (由 TIM1 触发)
       └─→ HAL_ADCEx_InjectedConvCpltCallback (RAM_FUNC)
            ├─ Foc_Adc_Sample()     ← 读 ADC 注入寄存器 → 电流/电压
            ├─ Foc_Para_Calc()      ← 母线电压计算
            └─ Foc_Control()        ← 核心 FOC 状态机
```

### 4.3 后台任务 (TIM3 1ms 时基)

| 任务 | 周期 | 功能 |
|------|------|------|
| HFPeriod_RUN | 1ms | Vofa JustFloat 数据上报 (6 float) |
| task_send_Rece | 20ms | LED 闪烁 |
| Balance_Control | 20ms | 空（预留） |
| KEY_RUN | 100ms | 按键扫描 / 状态机 |
| Task_DEBUG | 1000ms | LCD 刷新 (总线电压/模式/频率/角度/故障码) |

---

## 5. 核心算法说明

### 5.1 坐标变换链路

```
Ia, Ib, Ic  (三相电流)
  └→ Clarke 变换 (等幅值)  →  Iα, Iβ
       └→ Park 变换         →  Id, Iq  (旋转坐标系)
            └→ PI 控制      →  Vd, Vq
                 └→ iPark 变换 →  Vα, Vβ
                      └→ SVPWM → Ta, Tb, Tc → TIM1 CCR
```

### 5.2 磁链观测器 (Flux Observer)

基于反电动势积分的电压模型：

```
ψ_total = ∫(Uαβ - Rs·Iαβ + 补偿项) dt
ψ_stator = Ls · Iαβ
ψ_rotor  = ψ_total - ψ_stator

补偿项 = ψ_rotor · (|ψ_ref|² - |ψ_rotor|²) · Gain
```

PLL 锁相环：

```
PLL_Err = ψrβ·cosθ - ψrα·sinθ  (q轴磁链 → 0)
ωe = Kp·Err + ∫Ki·Err dt
θe = ∫ωe dt
```

**⚠️ 当前状态**: `Observer_Run()` 整体被注释，无感模式角度获取无效。

### 5.3 速度/电流 PI 控制器

- 带积分分离抗饱和 (anti-windup)
- 低通滤波输出: `OutF = 0.9·OutF_old + 0.1·Out`
- 斜坡发生器: `Grad_XieLv` (线性梯度逼近)

### 5.4 V/F 控制

- 开环电压-频率比控制
- V_amp 随 CurrentHz 线性增长: `V = Vmin + (Vtrg-Vmin) × (F/Ftrg)`
- 角度由频率积分: `θ = θ + 2π·F·Ts`
- 适用于调试阶段、无传感器启动过渡

---

## 6. 电机参数 (当前硬编码)

```c
#define MOTOR_RS    0.013525f   // 定子电阻 (Ω)
#define MOTOR_LS    0.00005f    // 定子电感 (H) - 50μH
#define MOTOR_FLUX  0.01452f    // 永磁体磁链 (Wb)
#define MOTOR_POLES 52          // 极对数 (104 极)
```

> 修改参数见 `Flux/inc/flux.h`。切换电机需重新编译。

---

## 7. 按键功能

| 按键 | 引脚 | 功能 |
|------|------|------|
| KEY1 | PC9 | (待定义) |
| KEY2 | PC8 | (待定义) |
| KEY3 | PC7 | (待定义) |
| KEY4 | PC6 | (待定义) |

> 当前 `key_process()` 已实现单/双/长按检测，但未映射到具体功能。

---

## 8. Vofa+ 上位机

- 协议: JustFloat
- 通道: CH1=TargetHz, CH2=CurrentHz, CH3=BusVoltage, CH4=Iu, CH5=Iv, CH6=Iw
- 接口: USB-CDC (虚拟串口)

---

## 9. 编译与烧录

1. 打开 `MDK-ARM/FOC_ST_V2.uvprojx`（Keil MDK）
2. 编译 (Build F7) → 烧录 (F8)
3. 连接 USB-CDC 查看 Vofa 数据
4. LCD 显示运行状态

---

## 10. 调试注意事项

- PWM 频率 20kHz, 死区由 TIM1 硬件配置
- 电流零点校准 `ZeroCurrOffset_Calibration()` 在上电时执行
- 母线电压分压比: (15500+1000)/1000 = 16.5:1
- 电流采样电阻 5mΩ, 运放增益 20x → `Curr_Ratio` 自动换算
- `_RAM_FUNC` 宏将热函数映射到 SRAM (零等待执行)
