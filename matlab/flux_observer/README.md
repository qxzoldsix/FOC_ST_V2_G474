# G474 磁链观测器与源码 PI

来源：`origin/环路参数辨识`，提交 `c43e794924c73d0aec71ddcb6ee14937aa39835e`（2026-08-24）。复现默认启用的 `Observer_Run_HybirdFlux()`；观测器和 FOC PI 分别按 `flux.c`、`PI_Cale.c` 的离散实现搭建。

## 入口与文件

在本目录的 MATLAB 命令窗口执行：

```matlab
start_flux_observer            % 打开主模型，默认参数和输入已嵌入模型
start_flux_observer('run')     % 重建、仿真并显示曲线
start_flux_observer('verify')  % 观测器 + 速度/d/q PI 的源码对比检查
start_flux_observer('build')   % 仅重建模型、PI 库和结构图
start_flux_observer('run','source') % 对照原程序的全零磁链初始化
```

使用 MATLAB R2024b + Simulink。验证原始 C 时优先使用已配置的 MEX 编译器；本机使用 MATLAB 自带 LCC64 编译独立测试程序，无需更改编译器配置。FOC PI 的 C 检查也使用 LCC64。

```text
flux_observer/
├─ start_flux_observer.m          唯一日常入口
├─ README.md
├─ model/
│  ├─ G474_Hybrid_Flux_Observer.slx 主模型
│  └─ G474_Firmware_PI.slx         源码 PI 模块库
├─ scripts/                       参数、构建和 MATLAB 单步算法
│  └─ verification/               验证脚本、C 适配程序和源码快照
├─ results/                       曲线、结构图、数据和验证报告
└─ work/                          缓存、编译产物和日志，可重新生成
```

模型与正式结果不放在缓存目录。脚本将新缓存和编译产物统一放入 `work/`。

## Simulink 分层

默认启动模式现为 `aligned`：在已完成预定位、已知对齐命令角度为 0 rad 的条件下，用参数磁链和首次采样电流初始化定子磁链：

```text
psi_s(0) = Lq*i_alpha_beta(0)
         + [psi_f+(Ld-Lq)*Id(0)] * [cos(theta_align); sin(theta_align)]
```

仅改变磁链初始状态；电机参数、所有 PI 系数及离散算法不变。PLL 积分和速度估计仍从零起步，没有预装真实速度，也没有使用仿真的真实角度。此初值是对齐后的电流模型估计，不能在初始位置未知时直接使用。

`start_flux_observer('run','source')` 保留原固件全零初始化，可用于对照和源码验证。启动对比见 `results/startup_comparison.png`，数值见 `startup_comparison.txt`；默认仿真记录为 `simulation_report.txt`，原源码检查仍为 `validation_report.txt`。验证入口最后会恢复默认 aligned 模型。

主模型顶层只有三个区域：**01_Input → 02_Flux_Observer → 03_Results**。

双击进入 `02_Flux_Observer`：

| 子系统 | 内容 |
|---|---|
| 01_Voltage_Compensation | 命令电压换算、三相极性和死区补偿 |
| 02_Active_Flux_Models | 电流模型与电压模型，分别封装 |
| 03_Flux_Feedback | 独立的 α/β 补偿 PI，以及单独的 SMO 分支 |
| 04_Stator_Integrator | Rs 压降、磁链导数和前向欧拉积分 |
| 05_PLL_and_Angle | 相位检测、源码 PLL PI、角度更新与上一周期角度 |
| 06_Speed_Filter | 8 周期累积、电频率滤波和源码 RPM 换算 |

所有 PI 都由原生 Gain、Sum、Unit Delay、Switch、比较和限幅模块搭成，没有替换成通用 PID Controller。双击子系统或 PI 查看内部运算；PI 的 Kp、Ki、Ts 可在“封装参数”中查看。

## 三种 PI 分别对应各自源码

### 磁链补偿 PI：flux.c

α、β 两轴独立，先更新积分，再形成输出：

```text
I[k]     = I[k-1] + Comp_Ki × error[k]
Ucomp[k] = Comp_Kp × error[k] + I[k]
```

源码此处没有积分或输出限幅，模型保持相同处理。选择 SMO 时，补偿 PI 的积分状态保持，与 C 分支行为一致。

### PLL PI：flux.c

```text
I[k]    = sat(I[k-1] + PLL_ki × PLL_Err[k], ±limit)
step[k] = sat(PLL_kp × PLL_Err[k] + I[k], ±limit)
limit   = 2π × 500 × Ts
theta[k+1] = wrap(theta[k] + step[k])
```

积分项和输出分别限幅。PLL_Ui 是每周期角度增量 rad/周期，直接累加到角度。PLL 和补偿 PI 的 Ki 已含离散周期，不再额外乘 Ts。

### 速度、d/q 电流 PI：PI_Cale.c / PID_controller

PI 库中 `FOC_Speed_PI`、`FOC_Id_PI`、`FOC_Iq_PI` 三个独立实例均使用下面的源码算法：

```text
up = Ref - Fbk
若 v1[k-1] >= Umax 或 v1[k-1] <= Umin：ui = i1[k-1]
否则：ui = Ki × up + i1[k-1]
ui = sat(ui, Umin, Umax)
i1[k] = ui
v1[k] = Kp × up + ui
Out = sat(v1[k], Umin, Umax)
```

决定是否积分的是**上一周期未限幅的 v1**，达到边界时也停止积分。积分先限幅、再写回 i1，之后形成新的 v1，最后输出限幅。Umax/Umin 为输入端口，支持原电流环的动态电压限制。

| 实例 | Kp | Ki（每次调用） | 程序中的限制 |
|---|---:|---:|---|
| FOC_Speed_PI | 0.01 | 0.0001 | ±12 |
| FOC_Id_PI | 0.01 | 0.00001 | Idq_FOC 按母线电压设置 |
| FOC_Iq_PI | 0.01 | 0.00001 | Idq_FOC 按电压圆剩余裕量设置 |

三个实例保留零初始状态。Ts 决定模型执行周期，Ki 仍是原程序的每次调用系数。`OutF = 0.9*OutF_old + 0.1*Out` 在原程序调用 PI 的外层执行，未混进 PID_controller 内部；本库输出与该函数的 Out/ui/v1/up 一一对应。这三个 PI 作为独立模块提供，观测器主模型没有扩展成完整速度/电流闭环。

## 电机与观测器参数

| 参数 | 数值 |
|---|---:|
| Rs | 0.875 Ω |
| Ls、Ld、Lq | 各 0.0002 H（0.2 mH） |
| 永磁磁链 | 0.00752 Wb |
| MOTOR_POLES | 14，保留源码定义及换算式 |
| Ts / 控制频率 | 50 μs / 20 kHz |
| PLL 设计带宽 / Kp / Ki | 15 Hz / 188.4955592 / 0.4441321980 |
| 补偿设计带宽 / Kp / Ki | 5 Hz / 62.83185307 / 0.04934802201 |
| 默认补偿模式 | PI（Comp_Mode=0） |
| SMO 增益 / 边界 | 10 V / 0.002 Wb |
| 死区补偿 / 电流门限 | +0.3 V / ±0.01 A |
| 标称母线 | 12 V |
| PLL 双重限幅 | ±0.1570796327 rad/周期 |
| 速度输出 | 每 8 周期更新，0.95 旧值 + 0.05 新值 |

参数集中在 `scripts/flux_observer_params.m`。采用仓库的观测器配置值；该分支 ParamId.c 的辨识结果仅存入 pid 候选值，不自动覆盖观测器，仓库没有某次实机辨识完成后的数值记录。

## 数据接口与回放

`03_Results` 中有 11 个独立示波器，原输出 y01～y11 与窗口 01～11 一一对应，每个窗口仅一条曲线，名称和单位均已标注，横轴统一为秒：

| 示波器 | 曲线 |
|---|---|
| 01_估算电角度_rad | 估算电角度 |
| 02_估算电频率_Hz | 估算电频率 |
| 03_机械转速_rpm | 源码换算的机械转速 |
| 04_有效磁链alpha_Wb | 有效磁链 α |
| 05_有效磁链beta_Wb | 有效磁链 β |
| 06_定子磁链alpha_Wb | 定子磁链 α |
| 07_定子磁链beta_Wb | 定子磁链 β |
| 08_PLL误差_Wb | PLL 的 q 轴磁链误差 |
| 09_PLL角度增量_rad_per_sample | PLL 每周期角度增量 |
| 10_补偿电压alpha_V | 死区补偿后的 α 电压 |
| 11_补偿电压beta_V | 死区补偿后的 β 电压 |

运行仿真后双击对应示波器查看。观测器算法和原 11 路数据记录顺序保持不变。结构图见 `results/observer_scopes.png`。

主模型工作区 observer_input 为 N×10 timeseries，间隔 50 μs，不插值：

| 列 | 输入 | 单位 |
|---|---|---|
| 1–2 | Svpwm_dq.Ualpha、Ubeta | 工程命令 pu |
| 3–5 | PhaseU/V/W_Curr | A |
| 6–7 | CLARKE_PCurr.Alpha、Beta | A |
| 8 | PARK_PCurr.Ds | A |
| 9 | BUS_Voltage | V |
| 10 | Svpwm_Km_BackwS | 无量纲 |

```matlab
start_flux_observer;
model = 'G474_Hybrid_Flux_Observer';
ws = get_param(model,'ModelWorkspace');
assignin(ws,'observer_input',timeseries(data,t));
set_param(model,'StopTime',num2str(t(end)));
out = sim(model);
y = out.observer_y.Data;
```

输出 N×11：电角度 rad、电频率 Hz、源码 RPM、有效磁链 αβ Wb、更新后的定子磁链 αβ Wb、PLL 误差 Wb、角度增量 rad/周期、补偿后电压 αβ V。

`flux_observer_step(u,s,p)` 可脱离 Simulink 逐周期调用。单精度参数用 `flux_observer_params('single')`，状态用 `flux_observer_init(p)` 初始化。

## 验证与保留约定

results/validation_report.txt 记录观测器的默认 PI、反转/母线变化、电流死区边界、SMO 四组检查。results/pi_validation_report.txt 记录速度/d/q PI 与原始 PI_Cale.c 的对比，覆盖上下边界相等、冻结/恢复积分、限幅改变和零电压裕量。

results/ 下的 flux_observer_diagram.png、observer_subsystems.png、firmware_pi_library.png 分别为顶层、观测器内部和 PI 库结构图；flux_observer_demo.png 是曲线；MAT 文件保存完整输入和输出。

- 死区补偿符号、Km*Vbus 电压尺度和原 Clarke 系数保持不变。
- 源码注释称 MOTOR_POLES 为极对数，但实际 RPM 式为 Hz*60/(MOTOR_POLES/2)。保留该式，即按 7 极对换算。
- 原生模型使用初始化 PLL 增益；独立 MATLAB 单步算法另保留可选增益恢复 ramp。默认 pll_ramp_active=0。
- 原生模型使用 double，C 使用 float；全零磁链启动对舍入及三角函数库敏感。报告同时保留全程和 0.3 秒后的差异，不宣称跨运行时逐位相同。
- 示例为解析电机轨迹，25→40→15 电 Hz，Id=0.2A、Iq=1A。没有机械方程、开关器件和噪声，用于观测器与离散实现检查，不代表完整 FOC 或实机验证。
