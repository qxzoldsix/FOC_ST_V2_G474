#include "flux.h"

FOC_OBSERVER_DEF Foc_observer;

/* ---- 保留兼容变量 ---- */
float Flux_in_wb[2];     // SMO 下未使用，保留供外部引用
float FluxS_in_wb[2];    // SMO 下未使用
float FluxR_in_wb[2];    // 由 SMO 反电动势 + PLL 角度计算

/* 转速计算用静态变量 */
static float speed_acc = 0.0f;
static float speed_now = 0.0f;
static uint8_t speed_calc_cnt = 0;

/**
 * 滑模观测器初始化
 */
void Flux_Observer_Init(void)
{
    Foc_observer.Rs   = MOTOR_RS;
    Foc_observer.Ls   = MOTOR_LS;
    Foc_observer.Flux = MOTOR_FLUX;

    Foc_observer.Ctrl_ts = 5e-5f;        // 20kHz
    Foc_observer.K_slide = SMO_K_SLIDE;
    Foc_observer.Fc_hz   = SMO_FC_HZ;

    /* PLL 参数（归一化误差，小增益） */
    Foc_observer.PLL_kp = SMO_PLL_KP;
    Foc_observer.PLL_ki = SMO_PLL_KI;
    Foc_observer.PLL_Err = 0.0f;
    Foc_observer.PLL_Interg = 0.0f;
    Foc_observer.PLL_Ui = 0.0f;

    Foc_observer.Theta = 0.0f;
    Foc_observer.speed_hz = 0.0f;

    /* SMO 状态清零 */
    Foc_observer.Ialpha_hat = 0.0f;
    Foc_observer.Ibeta_hat  = 0.0f;
    Foc_observer.Ealpha = 0.0f;
    Foc_observer.Ebeta  = 0.0f;
    Foc_observer.E_mag  = 0.0f;

    /* 兼容旧字段，SMO 下不使用 */
    Foc_observer.Gain = 0.0f;
    Foc_observer.Err  = 0.0f;

    /* PLL 增益 ramp */
    Foc_observer.PLL_kp_target = Foc_observer.PLL_kp;
    Foc_observer.PLL_ki_target = Foc_observer.PLL_ki;
    Foc_observer.pll_ramp_active = 0;

    /* 兼容磁链变量 */
    Flux_in_wb[0]  = 0.0f;
    Flux_in_wb[1]  = 0.0f;
    FluxS_in_wb[0] = 0.0f;
    FluxS_in_wb[1] = 0.0f;
    FluxR_in_wb[0] = 0.0f;
    FluxR_in_wb[1] = 0.0f;

    speed_acc = 0.0f;
    speed_now = 0.0f;
    speed_calc_cnt = 0;
}

/**
 * 符号函数（含滞环，抑制高频抖振）
 * 在 [-hys, +hys] 范围内线性过渡
 */
static float sign_hys(float x, float hys)
{
    if (x >  hys) return  1.0f;
    if (x < -hys) return -1.0f;
    return x / hys;          // 边界层线性区
}

/**
 * 死区电压补偿（αβ 坐标系）
 */
static void Deadtime_Comp_ab(float *Ualpha_out, float *Ubeta_out)
{
    float sign_u, sign_v, sign_w;
    float v_dead, v_alpha_dist, v_beta_dist;

    #define V_DEADTIME  0.3f

    sign_u = (Volt_CurrPara.PhaseU_Curr > 0.01f) ? 1.0f :
             ((Volt_CurrPara.PhaseU_Curr < -0.01f) ? -1.0f : 0.0f);
    sign_v = (Volt_CurrPara.PhaseV_Curr > 0.01f) ? 1.0f :
             ((Volt_CurrPara.PhaseV_Curr < -0.01f) ? -1.0f : 0.0f);
    sign_w = (Volt_CurrPara.PhaseW_Curr > 0.01f) ? 1.0f :
             ((Volt_CurrPara.PhaseW_Curr < -0.01f) ? -1.0f : 0.0f);

    v_dead = V_DEADTIME;

    v_alpha_dist = 0.6666667f * v_dead * (sign_u - 0.5f * sign_v - 0.5f * sign_w);
    v_beta_dist  = 0.5773503f * v_dead * (sign_v - sign_w);

    *Ualpha_out = Svpwm_dq.Ualpha + v_alpha_dist;
    *Ubeta_out  = Svpwm_dq.Ubeta  + v_beta_dist;
}

/**
 * 滑模观测器执行（每个 PWM 周期调用一次）
 *
 * 算法流程:
 *   1. 死区补偿 → 修正命令电压 Vαβ*
 *   2. 电流模型: Iαβ_hat(k+1) = Iαβ_hat(k) + Ts·[-(Rs/Ls)·Iαβ_hat + (1/Ls)·(Vαβ* - Zαβ)]
 *   3. 滑模控制: Zαβ = K_slide · sign(Iαβ_hat - Iαβ_meas)
 *   4. LPF 提取反电动势: Eαβ(k+1) = Eαβ(k) + Ts·ωc·(Zαβ - Eαβ)
 *   5. PLL 锁相环从 Eαβ 提取角度/速度
 *   6. 计算 FluxR_in_wb（兼容暖启动检测）
 */
void Observer_Run(void)
{
    float Ualpha_cmd, Ubeta_cmd;   // 死区补偿后电压
    float Ialpha_err, Ibeta_err;    // 电流误差
    float Zalpha, Zbeta;            // 滑模控制量
    float inv_Ls;                   // 1/Ls
    float Rs_div_Ls;                // Rs/Ls
    float lpf_coef;                 // LPF 滤波系数 Ts * ωc
    float omega_e;                  // 电角速度 (rad/s)
    float pll_ui_lim;               // PLL 积分限幅
    float sin_theta, cos_theta;
    float Emag_clamped;             // 钳位后的反电动势幅值

    /* ---- Step 1: 死区补偿 ---- */
    Deadtime_Comp_ab(&Ualpha_cmd, &Ubeta_cmd);

    /* ---- 预计算常数 ---- */
    inv_Ls    = 1.0f / Foc_observer.Ls;
    Rs_div_Ls = Foc_observer.Rs * inv_Ls;

    /* ---- Step 2: 电流误差 ---- */
    Ialpha_err = Foc_observer.Ialpha_hat - CLARKE_PCurr.Alpha;
    Ibeta_err  = Foc_observer.Ibeta_hat  - CLARKE_PCurr.Beta;

    /* ---- Step 3: 滑模控制量（带边界层，抑制抖振）---- */
    {
        float hys = 0.2f;   // 边界层厚度 (A)，加大以减少抖振
        Zalpha = Foc_observer.K_slide * sign_hys(Ialpha_err, hys);
        Zbeta  = Foc_observer.K_slide * sign_hys(Ibeta_err,  hys);
    }

    /* ---- Step 4: 电流模型更新 Iαβ_hat ---- */
    Foc_observer.Ialpha_hat += Foc_observer.Ctrl_ts * (
        -Rs_div_Ls * Foc_observer.Ialpha_hat + inv_Ls * (Ualpha_cmd - Zalpha)
    );
    Foc_observer.Ibeta_hat  += Foc_observer.Ctrl_ts * (
        -Rs_div_Ls * Foc_observer.Ibeta_hat  + inv_Ls * (Ubeta_cmd  - Zbeta)
    );

    /* ---- Step 5: LPF 提取反电动势 ---- */
    /* 低速时降低截止频率，减少噪声放大 */
    {
        float fc = (Foc_observer.speed_hz < 5.0f) ? SMO_FC_START_HZ : SMO_FC_HZ;
        lpf_coef = Foc_observer.Ctrl_ts * 2.0f * PI * fc;
        if (lpf_coef > 0.5f) lpf_coef = 0.5f;   // 稳定性约束
    }
    Foc_observer.Ealpha += lpf_coef * (Zalpha - Foc_observer.Ealpha);
    Foc_observer.Ebeta  += lpf_coef * (Zbeta  - Foc_observer.Ebeta);

    /* 反电动势幅值 */
    Foc_observer.E_mag = sqrtf(Foc_observer.Ealpha * Foc_observer.Ealpha +
                               Foc_observer.Ebeta  * Foc_observer.Ebeta);

    /* ---- Step 6: PLL 锁相环 ---- */
    sin_theta = sinf(Foc_observer.Theta);
    cos_theta = cosf(Foc_observer.Theta);

    /* 归一化 PLL 误差: -(Eα·cosθ + Eβ·sinθ) / |E|
     * = sin(θ̂ - θe) ≈ θ̂ - θe (小误差时)
     * 锁定后 → 0，极性正确（负反馈） */
    Emag_clamped = (Foc_observer.E_mag > 0.01f) ? Foc_observer.E_mag : 0.01f;
    Foc_observer.PLL_Err = -(Foc_observer.Ealpha * cos_theta +
                              Foc_observer.Ebeta  * sin_theta) / Emag_clamped;

    /* PI 控制器 */
    Foc_observer.PLL_Interg += Foc_observer.PLL_Err * Foc_observer.PLL_ki;

    /* 积分抗饱和: 限幅 ±最大电角速度步长 (500Hz 电频率) */
    pll_ui_lim = 2.0f * PI * 500.0f * Foc_observer.Ctrl_ts;
    Foc_observer.PLL_Interg = Limit_Sat(Foc_observer.PLL_Interg, pll_ui_lim, -pll_ui_lim);

    Foc_observer.PLL_Ui = Foc_observer.PLL_Err * Foc_observer.PLL_kp +
                          Foc_observer.PLL_Interg;
    Foc_observer.PLL_Ui = Limit_Sat(Foc_observer.PLL_Ui, pll_ui_lim, -pll_ui_lim);

    /* 角度积分 */
    Foc_observer.Theta += Foc_observer.PLL_Ui;

    /* 角度归一化 [0, 2π) */
    while (Foc_observer.Theta >= 2.0f * PI) Foc_observer.Theta -= 2.0f * PI;
    while (Foc_observer.Theta < 0.0f)       Foc_observer.Theta += 2.0f * PI;

    /* ---- Step 7: 兼容 FluxR_in_wb（供 motorControl 暖启动判断）---- */
    /* 从反电动势反算转子磁链: ψrα = Eβ/ωe, ψrβ = -Eα/ωe */
    omega_e = 2.0f * PI * Foc_observer.speed_hz;
    if (omega_e > 1.0f)   /* > ~0.16Hz, 低速时磁链估算不可靠 */
    {
        float inv_omega = 1.0f / omega_e;
        FluxR_in_wb[0] =  Foc_observer.Ebeta  * inv_omega;
        FluxR_in_wb[1] = -Foc_observer.Ealpha * inv_omega;
    }
    /* 低速/零速时用 PLL 角度 + 额定磁链构造 */
    else
    {
        FluxR_in_wb[0] = Foc_observer.Flux * cos_theta;
        FluxR_in_wb[1] = Foc_observer.Flux * sin_theta;
    }

    /* ---- Step 8: 电频率计算 ---- */
    speed_acc += Foc_observer.PLL_Ui;
    speed_calc_cnt++;

    if (speed_calc_cnt >= 8)
    {
        speed_now = speed_acc / (8.0f * Foc_observer.Ctrl_ts * 2.0f * PI);
        Foc_observer.speed_hz = Foc_observer.speed_hz * 0.95f + speed_now * 0.05f;
        speed_acc = 0.0f;
        speed_calc_cnt = 0;
    }

    /* ---- Step 9: PLL 增益恢复 ramp ---- */
    if (Foc_observer.pll_ramp_active)
    {
        Foc_observer.PLL_kp += (Foc_observer.PLL_kp_target - Foc_observer.PLL_kp) * 0.02f;
        Foc_observer.PLL_ki += (Foc_observer.PLL_ki_target - Foc_observer.PLL_ki) * 0.02f;

        if (fabsf(Foc_observer.PLL_kp - Foc_observer.PLL_kp_target) < 0.02f &&
            fabsf(Foc_observer.PLL_ki - Foc_observer.PLL_ki_target) < 0.01f)
        {
            Foc_observer.PLL_kp = Foc_observer.PLL_kp_target;
            Foc_observer.PLL_ki = Foc_observer.PLL_ki_target;
            Foc_observer.pll_ramp_active = 0;
        }
    }
}

/**
 * 获取转子电角度
 * 调用滑模观测器 → 将 Theta(rad) 转换为 IQAngle(0~4095)
 */
void Angel_Get(void)
{
    Observer_Run();

    motor.IQAngle = (int16_t)(651.8986f * Foc_observer.Theta);

    if (motor.IQAngle > 4095)
    {
        motor.IQAngle -= 4096;
    }
    else if (motor.IQAngle < 0)
    {
        motor.IQAngle += 4096;
    }

    motor.SpeedRPM = Foc_observer.speed_hz * 60.0f / (MOTOR_POLES / 2.0f);
}
