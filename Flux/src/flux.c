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
    Foc_observer.Gain = 150000.0f;       // 磁链幅值补偿增益（对齐参考项目，确保低速收敛）
    Foc_observer.Err = 0.0f;
    Foc_observer.Theta = 0.0f;
    Foc_observer.speed_hz = 0.0f;

    /* PLL 增益恢复机制 */
    Foc_observer.PLL_kp_target = Foc_observer.PLL_kp;
    Foc_observer.PLL_ki_target = Foc_observer.PLL_ki;
    Foc_observer.pll_ramp_active = 0;

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
 * 死区电压补偿（αβ 坐标系）
 *
 * 逆变器死区导致实际输出电压 ≠ 命令电压。低频时此误差占比极大，
 * 观测器若用未补偿的电压积分，磁链估算会严重偏离。
 *
 * 公式: V_dead = (T_dead / T_pwm) * V_bus * sign(I_phase)
 *       Vα_dist = (2/3) * V_dead * [sign(Ia) - 0.5*sign(Ib) - 0.5*sign(Ic)]
 *       Vβ_dist = (1/√3) * V_dead * [sign(Ib) - sign(Ic)]
 */
static void Deadtime_Comp_ab(float *Ualpha_out, float *Ubeta_out)
{
    float sign_u, sign_v, sign_w;
    float v_dead, v_alpha_dist, v_beta_dist;

    /* 死区等效电压: 24V * 0.02 ≈ 0.48V（可根据实测调整） */
    #define V_DEADTIME  0.3f

    /* 电流极性 */
    sign_u = (Volt_CurrPara.PhaseU_Curr > 0.01f) ? 1.0f :
             ((Volt_CurrPara.PhaseU_Curr < -0.01f) ? -1.0f : 0.0f);
    sign_v = (Volt_CurrPara.PhaseV_Curr > 0.01f) ? 1.0f :
             ((Volt_CurrPara.PhaseV_Curr < -0.01f) ? -1.0f : 0.0f);
    sign_w = (Volt_CurrPara.PhaseW_Curr > 0.01f) ? 1.0f :
             ((Volt_CurrPara.PhaseW_Curr < -0.01f) ? -1.0f : 0.0f);

    v_dead = V_DEADTIME;

    /* Clarke 变换：相电压畸变 → αβ */
    v_alpha_dist = 0.6666667f * v_dead * (sign_u - 0.5f * sign_v - 0.5f * sign_w);
    v_beta_dist  = 0.5773503f * v_dead * (sign_v - sign_w);  // 1/√3

    /* 补偿：命令电压 + 畸变电压 = 真实电压 */
    *Ualpha_out = Svpwm_dq.Ualpha + v_alpha_dist;
    *Ubeta_out  = Svpwm_dq.Ubeta  + v_beta_dist;
}

/**
 * 磁链观测器执行（每个 PWM 周期调用一次）
 *
 * 算法链路（修正版）：
 *   1. 死区补偿 → 修正输入电压
 *   2. 先算转子磁链 ψr = ψ - Ls·I（用当前周期的 I，当前积累的 ψ）
 *   3. 再算磁链幅值误差 Err（与步骤 2 同周期，消除一周期延迟）
 *   4. PLL 锁相环跟踪 ψr 角度（带积分抗饱和）
 *   5. 用 Err 补偿更新总磁链 ψ（用于下一周期）
 *   6. 电频率计算
 *   7. PLL 增益恢复 ramp（如果被切换逻辑降低了）
 */
void Observer_Run(void)
{
    float VoltRs[2];       // 定子电阻压降: Rs × Iαβ
    float VoltdPhi[2];     // 磁链微分 dψ/dt
    float sin_theta, cos_theta;
    float g_fluxfluxR;
    float Ualpha_cmd, Ubeta_cmd;  // 死区补偿后的电压
    float pll_ui_lim;             // PLL 积分抗饱和限幅

    /* ---- 死区补偿：修正命令电压 → 逼近真实电压 ---- */
    Deadtime_Comp_ab(&Ualpha_cmd, &Ubeta_cmd);

    /* ================================================================
     * Step 1: 先算转子磁链（用当前 I，当前累积的总磁链 ψ）
     *         这一步提前，让 Err 和补偿在同一周期内
     * ================================================================ */
    FluxR_in_wb[0] = Flux_in_wb[0] - Foc_observer.Ls * CLARKE_PCurr.Alpha;
    FluxR_in_wb[1] = Flux_in_wb[1] - Foc_observer.Ls * CLARKE_PCurr.Beta;

    /* ---- Step 2: 磁链幅值误差（与 Step 1 同周期的 FluxR）---- */
    g_fluxfluxR = FluxR_in_wb[0] * FluxR_in_wb[0] + FluxR_in_wb[1] * FluxR_in_wb[1];
    Foc_observer.Err = Foc_observer.Flux * Foc_observer.Flux - g_fluxfluxR;

    /* ---- Step 3: sinθ / cosθ ---- */
    sin_theta = sinf(Foc_observer.Theta);
    cos_theta = cosf(Foc_observer.Theta);

    /* ---- Step 4: PLL 锁相环（带积分抗饱和）---- */
    // PLL 误差 = q 轴转子磁链（理想情况下应为 0）
    Foc_observer.PLL_Err = FluxR_in_wb[1] * cos_theta - FluxR_in_wb[0] * sin_theta;

    // PI 控制器（积分项有限幅）
    Foc_observer.PLL_Interg += Foc_observer.PLL_Err * Foc_observer.PLL_ki;

    // 抗饱和: 限制积分项不超过最大电角速度对应的每周期步长
    // 500Hz 电频率 → ωe_max = 2π×500 = 3141 rad/s → 每周期步长 = 3141×50µs = 0.157 rad
    pll_ui_lim = 2.0f * PI * 500.0f * Foc_observer.Ctrl_ts;
    Foc_observer.PLL_Interg = Limit_Sat(Foc_observer.PLL_Interg, pll_ui_lim, -pll_ui_lim);

    Foc_observer.PLL_Ui = Foc_observer.PLL_Err * Foc_observer.PLL_kp + Foc_observer.PLL_Interg;
    Foc_observer.PLL_Ui = Limit_Sat(Foc_observer.PLL_Ui, pll_ui_lim, -pll_ui_lim);

    // 角度积分
    Foc_observer.Theta += Foc_observer.PLL_Ui;

    // 角度归一化 [0, 2π)
    while (Foc_observer.Theta >= 2.0f * PI) Foc_observer.Theta -= 2.0f * PI;
    while (Foc_observer.Theta < 0.0f)       Foc_observer.Theta += 2.0f * PI;

    /* ---- Step 5: 更新总磁链（用当前周期的 Err 做补偿，无延迟）---- */
    VoltRs[0] = Foc_observer.Rs * CLARKE_PCurr.Alpha;
    VoltdPhi[0] = Ualpha_cmd - VoltRs[0];
    VoltdPhi[0] += FluxR_in_wb[0] * Foc_observer.Err * Foc_observer.Gain;  // ← 同周期 Err
    Flux_in_wb[0] += VoltdPhi[0] * Foc_observer.Ctrl_ts;

    VoltRs[1] = Foc_observer.Rs * CLARKE_PCurr.Beta;
    VoltdPhi[1] = Ubeta_cmd - VoltRs[1];
    VoltdPhi[1] += FluxR_in_wb[1] * Foc_observer.Err * Foc_observer.Gain;  // ← 同周期 Err
    Flux_in_wb[1] += VoltdPhi[1] * Foc_observer.Ctrl_ts;

    /* ---- Step 6: 定子磁链 ---- */
    FluxS_in_wb[0] = Foc_observer.Ls * CLARKE_PCurr.Alpha;
    FluxS_in_wb[1] = Foc_observer.Ls * CLARKE_PCurr.Beta;

    /* ---- Step 7: 电频率计算与低通滤波 ---- */
    speed_acc += Foc_observer.PLL_Ui;
    speed_calc_cnt++;

    if (speed_calc_cnt >= 8)
    {
        speed_now = speed_acc / (8.0f * Foc_observer.Ctrl_ts * 2.0f * PI);
        Foc_observer.speed_hz = Foc_observer.speed_hz * 0.95f + speed_now * 0.05f;
        speed_acc = 0.0f;
        speed_calc_cnt = 0;
    }

    /* ---- Step 8: PLL 增益恢复 ramp ---- */
    if (Foc_observer.pll_ramp_active)
    {
        // 每周期向目标增益靠近 2%（指数收敛，约 100 周期 ≈ 5ms 恢复）
        Foc_observer.PLL_kp += (Foc_observer.PLL_kp_target - Foc_observer.PLL_kp) * 0.02f;
        Foc_observer.PLL_ki += (Foc_observer.PLL_ki_target - Foc_observer.PLL_ki) * 0.02f;

        // 接近目标时直接到位
        if (fabsf(Foc_observer.PLL_kp - Foc_observer.PLL_kp_target) < 0.05f &&
            fabsf(Foc_observer.PLL_ki - Foc_observer.PLL_ki_target) < 0.05f)
        {
            Foc_observer.PLL_kp = Foc_observer.PLL_kp_target;
            Foc_observer.PLL_ki = Foc_observer.PLL_ki_target;
            Foc_observer.pll_ramp_active = 0;
        }
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
