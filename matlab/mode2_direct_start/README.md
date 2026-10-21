# MODE=2 从静止直接闭环启动实验

这是独立项目。原来的 `../flux_observer` 模型保留，未改动。新模型不是给观测器播放一段预先旋转的电流轨迹，而是闭合“观测器 → Clarke/Park → 速度 PI → 电流 PI → SVPWM → 电机 → 电流采样”的反馈链路。

## 打开与运行

MATLAB 当前目录切到本文件所在目录，然后：

```matlab
start_mode2('open')    % 打开已生成的模型，点 Run 即可
start_mode2('run')     % 按参数文件重建并仿真，导出结果
start_mode2('verify')  % 重建、核对原生模块连接、检查不同初始角度
```

主模型：`model/G474_Mode2_Direct_Start.slx`。模型已内嵌参数、观测器和 PI 模块，不需要 Simscape。构建脚本从保留模型读取观测器和 PI 后复制到新模型，并解除库链接，不保存或修改原模型。

## 测试工况与参数

用户要求：12 V、小负载、200 → 400 → 1000 rpm 切换。给定切换时刻为 0、0.8、1.6 s，运行 4 s；仍经过源码的 1000 rpm/s 给定斜坡。全程 mode=2，没有 mode=4 预定位、VF、磁链预置或速度前馈。

参数集中在 `scripts/mode2_params.m`：

| 参数 | 值 | 来源 |
|---|---:|---|
| Rs | 0.875 Ω | flux.h |
| Ld / Lq | 0.0002 / 0.0002 H | flux.h |
| 永磁体磁链 | 0.00752 Wb | flux.h |
| 极对数 | 7 | 保留固件 MOTOR_POLES/2 的计算 |
| 采样周期 | 50 μs | flux.c |
| PLL / 磁链补偿带宽 | 15 / 5 Hz | flux.h |
| 速度 PI Kp / Ki | 0.01 / 0.0001 | PI_Cale.c，Ki 为每次调用系数 |
| 电流 PI Kp / Ki | 0.01 / 0.00001 | PI_Cale.c，Ki 为每次调用系数 |
| SPEED_IQ_SIGN | -1 | Foc.c，默认保留 |
| 初始电流 / 转速 | 0 A / 0 rpm | 静止启动工况 |
| 初始电角度 | 30° | 假设，可修改；不传给控制器 |
| 转动惯量 J | 2e-5 kg·m² | **假设，源码没有此辨识结果** |
| 黏性摩擦 B | 1e-5 N·m·s/rad | **假设** |
| 小负载转矩 | 0.005 N·m | **假设**，固定正方向，电机方程中减去此转矩 |
| 实际逆变器死区等效电压 | 0.3 V | **物理模型假设**，与观测器内补偿独立 |
| 采样电流相对绕组正向的符号 | +1 | **硬件极性假设**；固件 ADC 原始值前的负号不足以确定物理电流方向 |

改参数后使用 `start_mode2('run')`。也可在打开的模型中修改 Model Workspace 的 `q` 后直接 Run。`start_mode2('run')` 会重新读取参数文件覆盖临时修改。

## 模块与源码一致的部分

1. `01_Mode2_Command`：速度阶跃给定，mode 恒等于 2。
2. `02_ADC_and_Previous_Cycle`：新相电流、上一周期 Clarke/Park 电流和电压指令。保留 `Angel_Get()` 在 `UVW_Axis_DQ()` 之前的执行顺序。
3. `03_Hybrid_Flux_Observer`：原生模块搭建的有效磁链电流/电压模型、补偿 PI、磁链积分、PLL 和转速滤波。初始化全零。
4. `04_Clarke_Park`：源码相序、符号、4096 角度量化、原始 1025 项正弦表及原有下标偏移。
5. `05_Speed_Ramp_and_PI`：原始给定斜坡、速度 PI、OutF 滤波及 `SPEED_IQ_SIGN=-1`。源码 `Grad_Timer` 默认为 0，因此每个控制周期执行。
6. `06_Id_Iq_PI_and_Voltage_Circle`：Id=0，d/q 原始 PI、上一周期 v1 决定的积分冻结、动态限幅和 OutF 滤波。
7. `07_InversePark_SVPWM_Inverter`：源码反 Park、SVPWM 扇区、CCR 截断/限幅，之后独立重建逆变器平均电压。
8. `08_PMSM_Electrical_Mechanical_Plant`：dq 电气方程、转矩、惯量、负载与机械速度；每个采样周期四个 RK4 子步。物理角度/转速只用于电机自身和结果对比。
9. `09_Results_and_Logging`：11 个标注示波器、完整日志。
10. `10_Firmware_Cycle_Memory`：上一周期电压指令与电流变换结果。
11. `11_Observer_Output_Decode`：观测器输出角度和转速分发。

所有 FOC PI 均为可展开的原生离散运算模块，没有使用通用 PID 替代源码。磁链 PI / PLL 保留源码计算次序和限幅，Ki 不额外乘 Ts。固件参考来自 `origin/环路参数辨识`，提交 `c43e794924c73d0aec71ddcb6ee14937aa39835e`；相关原始函数快照放在 `scripts/reference/`。

## 11 个示波器

| 编号 | 内容 |
|---|---|
| 01 | 目标、斜坡给定、实际转速、估算转速，rpm |
| 02 | 实际和估算电角度，rad |
| 03 | Iq 给定和反馈，A |
| 04 | Id 反馈，A |
| 05 | d/q 电压指令，pu |
| 06 | 定子磁链 α/β，Wb |
| 07 | 有效磁链 α/β，Wb |
| 08 | PLL 误差，Wb |
| 09 | 电磁转矩，N·m |
| 10 | 实际逆变器与观测器输入电压 α/β，V |
| 11 | 三相采样电流，A |

## 如何看试验结果

默认严格保留 `SPEED_IQ_SIGN=-1`。在本模型假定的标准绕组相序、电流正向下，这会使正转速误差给出负 Iq，默认结果不能正向跟随给定。此结果不代表已经确认实机相序/采样极性有错，必须结合实际接线和电流方向判定。没有为了得到成功启动的曲线而暗改符号或 PI。

`verify_mode2` 额外测试 0°、90°、180° 初始电角度，并且单独跑一条 `SPEED_IQ_SIGN=+1` 的诊断工况；**该诊断不是源码配置，不会替换默认模型**。它用于观察电流给定符号对结果的影响，也不能证明无感零速启动对所有初始位置均可可靠成功。

SVPWM 采用一个 PWM 周期内的平均电压，没有开关纹波、ADC 量化/噪声、驱动器保护和实际功率级非理想特性。特别保留了两个独立量：固件观测器使用 `command*Km*Vbus`，电机实际电压由 `CCR/4000` 和三相桥重建；不强行把两者设置成相等。小电机机械参数和硬件极性未测量，因此结果用于排查启动链路，不是实机启动性能承诺。

## 输出文件

- `results/mode2_startup.png`：默认源码配置启动/切速波形。
- `results/mode2_results.mat`：完整结果、参数与假设。
- `results/simulation_report.txt`：三阶段实际/估算转速、电流峰值、跟踪结果。
- `results/verification_report.txt`：原生块与独立 MATLAB 时域参考核对、零初值、11 示波器及初始角度试验。
- `results/initial_angle_sweep.csv`：初始角度对照。
- `results/iq_sign_diagnostic.png` / `.mat`：单独的电流符号诊断。
- `results/model_diagram.png` / `scopes_11.png`：模型与示波器布局。
- `work/`：缓存、运行日志，忽略版本管理。

`mode2_results.mat` 中 `result.y` 每行一个采样点，共 34 列：

```text
 1:11 原观测器的 11 路输出
12 实际 rpm       13 实际电角度 rad      14 电磁转矩 Nm
15:17 三相采样电流 A       18:19 Park Id/Iq A
20:21 Vd/Vq 指令 pu        22 Iq 给定 A      23 斜坡 rpm
24:25 alpha/beta 电压指令 pu   26:27 实际 alpha/beta 电压 V
28:30 CCR abc             31 目标 rpm      32 mode=2
33:34 当周期 Clarke alpha/beta 电流 A
```
