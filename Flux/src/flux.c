#include "flux.h"

FOC_OBSERVER_DEF Foc_observer;
float Flux_in_wb[2];    // 总磁链（气隙磁链）αβ 分量
float FluxS_in_wb[2];   // 定子磁链 αβ 分量（ψs = Ls × I）
float FluxR_in_wb[2];   // 转子磁链 αβ 分量（ψr = ψ - ψs）

/* 转速计算用静态变量 */
static float speed_acc = 0.0f;
static float speed_now = 0.0f;
static uint8_t speed_calc_cnt = 0;

/**
 * 磁链观测器初始化
 * 加载电机参数，复位 PLL 和磁链积分状态
 */
void Flux_Observer_Init(void)
{
    Foc_observer.Rs   = MOTOR_RS;
    Foc_observer.Ls   = MOTOR_LS;
    Foc_observer.Flux = MOTOR_FLUX;

    Foc_observer.PLL_kp = 20.0f;
    Foc_observer.PLL_ki = 10.0f;
    Foc_observer.PLL_Err = 0.0f;
    Foc_observer.PLL_Interg = 0.0f;
    Foc_observer.PLL_Ui = 0.0f;

    Foc_observer.Ctrl_ts = 5e-5f;       // 控制周期 20kHz (Ts = 1/20000 = 50µs)
    Foc_observer.Gain = 15000.0f;       // 磁链幅值补偿增益 (150000→1500, 避免幅值环震荡啸叫)
    Foc_observer.Err = 0.0f;
    Foc_observer.Theta = 0.0f;
    Foc_observer.speed_hz = 0.0f;

    /* 磁链积分初值清零 */
    Flux_in_wb[0] = 0.0f;
    Flux_in_wb[1] = 0.0f;

    FluxS_in_wb[0] = 0.0f;
    FluxS_in_wb[1] = 0.0f;

    FluxR_in_wb[0] = 0.0f;
    FluxR_in_wb[1] = 0.0f;

    /* 转速变量清零 */
    speed_acc = 0.0f;
    speed_now = 0.0f;
    speed_calc_cnt = 0;
}

/**
 * 磁链观测器执行（每个 PWM 周期调用一次）
 *
 * 算法链路：
 *   1. 反电动势积分 → 总磁链 ψ = ∫(Uαβ - Rs·Iαβ + 补偿项) dt
 *   2. 定子磁链 ψs = Ls × Iαβ
 *   3. 转子磁链 ψr = ψ - ψs
 *   4. 磁链幅值误差补偿（收敛 ψr 幅值到参考值）
 *   5. PLL 锁相环跟踪转子磁链角度
 *   6. 电频率计算与低通滤波
 */
void Observer_Run(void)
{
    float VoltRs[2];       // 定子电阻压降: Rs × Iαβ
    float VoltdPhi[2];     // 磁链微分 dψ/dt
    float sin_theta, cos_theta;
    float g_fluxfluxR;

    /* ---- 第一步：α 轴磁链积分 ---- */
    VoltRs[0] = Foc_observer.Rs * CLARKE_PCurr.Alpha;
    VoltdPhi[0] = Svpwm_dq.Ualpha - VoltRs[0];
    VoltdPhi[0] += FluxR_in_wb[0] * Foc_observer.Err * Foc_observer.Gain;  // 幅值误差补偿
    Flux_in_wb[0] += VoltdPhi[0] * Foc_observer.Ctrl_ts;                   // 前向欧拉积分

    /* ---- 第二步：β 轴磁链积分 ---- */
    VoltRs[1] = Foc_observer.Rs * CLARKE_PCurr.Beta;
    VoltdPhi[1] = Svpwm_dq.Ubeta - VoltRs[1];
    VoltdPhi[1] += FluxR_in_wb[1] * Foc_observer.Err * Foc_observer.Gain;
    Flux_in_wb[1] += VoltdPhi[1] * Foc_observer.Ctrl_ts;

    /* ---- 第三步：定子磁链 ψs = Ls × I ---- */
    FluxS_in_wb[0] = Foc_observer.Ls * CLARKE_PCurr.Alpha;
    FluxS_in_wb[1] = Foc_observer.Ls * CLARKE_PCurr.Beta;

    /* ---- 第四步：转子磁链 ψr = ψ - ψs ---- */
    FluxR_in_wb[0] = Flux_in_wb[0] - FluxS_in_wb[0];
    FluxR_in_wb[1] = Flux_in_wb[1] - FluxS_in_wb[1];

    /* ---- 第五步：磁链幅值误差 ---- */
    g_fluxfluxR = FluxR_in_wb[0] * FluxR_in_wb[0] + FluxR_in_wb[1] * FluxR_in_wb[1];
    Foc_observer.Err = Foc_observer.Flux * Foc_observer.Flux - g_fluxfluxR;

    /* ---- 第六步：sinθ / cosθ（硬件浮点，直接算）---- */
    sin_theta = sinf(Foc_observer.Theta);
    cos_theta = cosf(Foc_observer.Theta);

    /* ---- 第七步：PLL 锁相环 ---- */
    // PLL 误差 = q 轴转子磁链（理想情况下应为 0）
    Foc_observer.PLL_Err = FluxR_in_wb[1] * cos_theta - FluxR_in_wb[0] * sin_theta;

    // PI 控制器
    Foc_observer.PLL_Interg += Foc_observer.PLL_Err * Foc_observer.PLL_ki;
    Foc_observer.PLL_Ui = Foc_observer.PLL_Err * Foc_observer.PLL_kp + Foc_observer.PLL_Interg;

    // 角度积分
    Foc_observer.Theta += Foc_observer.PLL_Ui;

    // 角度归一化 [0, 2π)
    if (Foc_observer.Theta < 0.0f)
        Foc_observer.Theta += 2.0f * PI;
    else if (Foc_observer.Theta >= 2.0f * PI)
        Foc_observer.Theta -= 2.0f * PI;

    /* ---- 第八步：电频率计算与低通滤波 ---- */
    speed_acc += Foc_observer.PLL_Ui;
    speed_calc_cnt++;

    if (speed_calc_cnt >= 8)
    {
        // 瞬时电频率: ωe(rad/s) → Hz
        // speed_acc = Σ(PLL_Ui×8) = Σ(ωe×Ts×8), K = 1/(8×Ts×2π)
        speed_now = speed_acc / (8.0f * Foc_observer.Ctrl_ts * 2.0f * PI);
        // 一阶低通滤波: 0.95×旧值 + 0.05×新值
        Foc_observer.speed_hz = Foc_observer.speed_hz * 0.95f + speed_now * 0.05f;

        speed_acc = 0.0f;
        speed_calc_cnt = 0;
    }
}

/**
 * 获取转子电角度
 * 调用磁链观测器 → 将 Theta(rad) 转换为 IQAngle(0~4095)
 * 由 FOC 控制循环在 UVW_Axis_DQ 之前调用
 */
void Angel_Get(void)
{
    Observer_Run();

    // rad → 定点角度 (4096 = 2π)
    motor.IQAngle = (int16_t)(651.8986f * Foc_observer.Theta);

    // 角度回绕到 [0, 4095]
    if (motor.IQAngle > 4095)
    {
        motor.IQAngle -= 4096;
    }
    else if (motor.IQAngle < 0)
    {
        motor.IQAngle += 4096;
    }

    // 速度反馈: 电频率 Hz → 机械转速 RPM
    motor.SpeedRPM = Foc_observer.speed_hz * 60.0f / (MOTOR_POLES / 2.0f);
}
