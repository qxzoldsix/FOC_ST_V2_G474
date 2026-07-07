/**
 * @file    flux.c
 * @author  nono <nono_1007@foxmail.com>
 * @brief   电压模型磁链观测器: 死区补偿 + 磁链积分 + PLL 锁相环
 */
#include <string.h>
#include "flux.h"

FOC_OBSERVER_DEF Foc_observer;
float Flux_in_wb[2];    // 总磁链（气隙磁链）αβ 分量
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
    /* 全结构体清零，避免逐字段赋 0 */
    memset(&Foc_observer, 0, sizeof(Foc_observer));

    /* ---- 电机参数 ---- */
    Foc_observer.Rs   = MOTOR_RS;
    Foc_observer.Ls   = MOTOR_LS;
    Foc_observer.Ld   = MOTOR_LD;
    Foc_observer.Lq   = MOTOR_LQ;
    Foc_observer.Flux = MOTOR_FLUX;

    /* ---- 电压型观测器补偿增益 ---- */
    Foc_observer.Gain = 5000.0f;       // 磁链幅值补偿增益（过大→震荡啸叫，过小→收敛慢）

    /* 控制周期必须先设，后续 PI 参数计算依赖它 */
    Foc_observer.Ctrl_ts = 5e-5f;      // 控制周期 20kHz (Ts = 1/20000 = 50µs)

    /* ---- PLL 参数 ---- */
#if OBSERVER_TYPE == OBS_HYBRID_ACTIVE_FLUX
    /* 混合观测器: 按二阶系统带宽设计 (ζ=1, 临界阻尼) */
    {
        float w_pll = 2.0f * PI * HYBRID_PLL_BW_HZ;
        Foc_observer.PLL_BW_Hz = HYBRID_PLL_BW_HZ;
        Foc_observer.PLL_kp = 2.0f * w_pll;
        Foc_observer.PLL_ki = w_pll * w_pll * Foc_observer.Ctrl_ts;
    }
#else
    /* 电压型观测器: 保持原有调参值 */
    Foc_observer.PLL_kp = 20.0f;
    Foc_observer.PLL_ki = 10.0f;
#endif

    /* PLL 增益恢复机制: 记录目标值供 ramp 恢复 */
    Foc_observer.PLL_kp_target = Foc_observer.PLL_kp;
    Foc_observer.PLL_ki_target = Foc_observer.PLL_ki;

    /* ---- Hybrid Active Flux 反馈补偿 PI ---- */
    {
        float w0 = 2.0f * PI * HYBRID_COMP_BW_HZ;
        Foc_observer.Comp_Kp = 2.0f * w0;
        Foc_observer.Comp_Ki = w0 * w0 * Foc_observer.Ctrl_ts;
    }

    /* SMO 参数（备用） */
    Foc_observer.SMO_Kslide   = HYBRID_SMO_KSLIDE;
    Foc_observer.SMO_Boundary = HYBRID_SMO_BOUNDARY;

    /* ---- 全局磁链变量清零 ---- */
    memset(Flux_in_wb,  0, sizeof(Flux_in_wb));
    memset(FluxR_in_wb, 0, sizeof(FluxR_in_wb));

    /* ---- 转速变量清零 ---- */
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
    float v_cmd_scale;

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

    /*
     * SVPWM applies Ualpha/Ubeta through Svpwm_Km_BackwS and the real DC bus.
     * The observer integrates volts, so use the same effective voltage scale.
     */
    v_cmd_scale = Volt_CurrPara.Svpwm_Km_BackwS * Volt_CurrPara.BUS_Voltage;
    *Ualpha_out = Svpwm_dq.Ualpha * v_cmd_scale + v_alpha_dist;
    *Ubeta_out  = Svpwm_dq.Ubeta  * v_cmd_scale + v_beta_dist;
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

    /* ---- Step 6: 电频率计算与低通滤波 ---- */
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
/* ================================================================
 * 备选观测器 (空壳, 待实现)
 * ================================================================ */
void Observer_Run_EKF(void)
{
    // TODO: 扩展卡尔曼滤波观测器
}

/**
 * Hybrid Active Flux 观测器（电流-电压互补型）
 *
 * 参考: TPE2023A "Robust Encoderless Control for PMSM Drives:
 *       A Revised Hybrid Active Flux-Based Technique"
 *
 * 核心思想:
 *   电流模型 (高频/低速准确) + 电压模型 (中高速准确) → 全速域互补
 *
 *   ψ_active = ψf + (Ld - Lq)·Id          ← 有效磁链 (统一凸极/非凸极)
 *   电流模型: ψ_I = ψ_active × [cosθ̂, sinθ̂]
 *   电压模型: ψ_V = ∫(V - Rs·I + U_comp)dt - Lq·I
 *   误差反馈: U_comp = PI(ψ_I - ψ_V)     ← 矢量误差修正
 *   PLL: ε = -ψ_Vα·sinθ̂ + ψ_Vβ·cosθ̂ → θ̂, ω̂
 *
 * 算法链路:
 *   1. 死区补偿 → 修正输入电压
 *   2. 电流模型: ψ_active = ψf + (Ld-Lq)·Id → 投影到 αβ
 *   3. 电压模型提取: ψ_V = ψs - Lq·I (上一周期累积的磁链)
 *   4. 矢量误差: Δψ = ψ_I - ψ_V
 *   5. 反馈 PI (αβ 独立) → U_comp
 *   6. 磁链积分: ψs += (V* - Rs·I + U_comp)·Ts
 *   7. PLL 锁相 → θ̂, ω̂
 *   8. 电频率计算 + PLL 增益 ramp
 */
void Observer_Run_HybirdFlux(void)
{
    float Ualpha_cmd, Ubeta_cmd;   // 死区补偿后电压
    float sin_theta, cos_theta;
    float pll_ui_lim;
    float error_alpha, error_beta; // 电流-电压模型磁链误差

    /* ---- Step 1: 死区补偿 ---- */
    Deadtime_Comp_ab(&Ualpha_cmd, &Ubeta_cmd);

    /* ---- Step 2: 电流模型 — 有效磁链计算 + αβ 投影 ---- */
    /* ψ_active = ψf + (Ld - Lq) × Id */
    Foc_observer.ActiveFlux_I = Foc_observer.Flux
        + (Foc_observer.Ld - Foc_observer.Lq) * PARK_PCurr.Ds;

    /* 用上一周期 PLL 输出的 θ̂ 做投影: ψ_I_αβ = ψ_active × [cosθ̂, sinθ̂] */
    sin_theta = sinf(Foc_observer.Theta);
    cos_theta = cosf(Foc_observer.Theta);
    Foc_observer.Flux_Active_I[0] = Foc_observer.ActiveFlux_I * cos_theta;
    Foc_observer.Flux_Active_I[1] = Foc_observer.ActiveFlux_I * sin_theta;

    /* ---- Step 3: 电压模型 — 从定子磁链提取有效磁链 ---- */
    /* ψ_V = ψs - Lq × I (上一周期累积的 ψs) */
    Foc_observer.Flux_Active_V[0] = Flux_in_wb[0] - Foc_observer.Lq * CLARKE_PCurr.Alpha;
    Foc_observer.Flux_Active_V[1] = Flux_in_wb[1] - Foc_observer.Lq * CLARKE_PCurr.Beta;

    /* 同步写入全局变量，供 Vofa/LCD 监控 */
    FluxR_in_wb[0] = Foc_observer.Flux_Active_V[0];
    FluxR_in_wb[1] = Foc_observer.Flux_Active_V[1];

    /* ---- Step 4: 矢量误差 = 电流模型 - 电压模型 ---- */
    error_alpha = Foc_observer.Flux_Active_I[0] - Foc_observer.Flux_Active_V[0];
    error_beta  = Foc_observer.Flux_Active_I[1] - Foc_observer.Flux_Active_V[1];

    /* ---- Step 5: 反馈律 — PI 或 SMO ---- */
    if (Foc_observer.Comp_Mode == 0)
    {
        /* PI 反馈律: U_comp = Kp × Δψ + Ki × ∫Δψ dt */
        /* α、β 轴独立 PI，参数相同 */
        float u_comp_alpha, u_comp_beta;

        Foc_observer.Comp_Interg[0] += error_alpha * Foc_observer.Comp_Ki;
        Foc_observer.Comp_Interg[1] += error_beta  * Foc_observer.Comp_Ki;

        u_comp_alpha = error_alpha * Foc_observer.Comp_Kp + Foc_observer.Comp_Interg[0];
        u_comp_beta  = error_beta  * Foc_observer.Comp_Kp + Foc_observer.Comp_Interg[1];

        /* ---- Step 6: 定子磁链积分 ---- */
        /* dψs/dt = V* - Rs·I + U_comp */
        Flux_in_wb[0] += (Ualpha_cmd
                          - Foc_observer.Rs * CLARKE_PCurr.Alpha
                          + u_comp_alpha) * Foc_observer.Ctrl_ts;
        Flux_in_wb[1] += (Ubeta_cmd
                          - Foc_observer.Rs * CLARKE_PCurr.Beta
                          + u_comp_beta)  * Foc_observer.Ctrl_ts;
    }
    else
    {
        /* SMO 反馈律: 带边界层的饱和函数 */
        float u_comp_alpha, u_comp_beta;

        /* α 轴 SMO */
        if (fabsf(error_alpha) < Foc_observer.SMO_Boundary)
        {
            u_comp_alpha = Foc_observer.SMO_Kslide * error_alpha
                         / Foc_observer.SMO_Boundary;
        }
        else
        {
            u_comp_alpha = (error_alpha > 0.0f) ? Foc_observer.SMO_Kslide
                                                : -Foc_observer.SMO_Kslide;
        }

        /* β 轴 SMO */
        if (fabsf(error_beta) < Foc_observer.SMO_Boundary)
        {
            u_comp_beta = Foc_observer.SMO_Kslide * error_beta
                        / Foc_observer.SMO_Boundary;
        }
        else
        {
            u_comp_beta = (error_beta > 0.0f) ? Foc_observer.SMO_Kslide
                                              : -Foc_observer.SMO_Kslide;
        }

        /* 磁链积分 */
        Flux_in_wb[0] += (Ualpha_cmd
                          - Foc_observer.Rs * CLARKE_PCurr.Alpha
                          + u_comp_alpha) * Foc_observer.Ctrl_ts;
        Flux_in_wb[1] += (Ubeta_cmd
                          - Foc_observer.Rs * CLARKE_PCurr.Beta
                          + u_comp_beta)  * Foc_observer.Ctrl_ts;
    }

    /* ---- Step 7: PLL 锁相环（带积分抗饱和）---- */
    /* PLL 误差: ε = -ψ_Vα·sinθ̂ + ψ_Vβ·cosθ̂ = |ψ_V|·sin(θ - θ̂) */
    Foc_observer.PLL_Err = -Foc_observer.Flux_Active_V[0] * sin_theta
                           + Foc_observer.Flux_Active_V[1] * cos_theta;

    /* PI 控制器 */
    Foc_observer.PLL_Interg += Foc_observer.PLL_Err * Foc_observer.PLL_ki;

    /* 抗饱和: 限幅 ±(ωe_max × Ts) */
    pll_ui_lim = 2.0f * PI * 500.0f * Foc_observer.Ctrl_ts;
    Foc_observer.PLL_Interg = Limit_Sat(Foc_observer.PLL_Interg, pll_ui_lim, -pll_ui_lim);

    Foc_observer.PLL_Ui = Foc_observer.PLL_Err * Foc_observer.PLL_kp
                         + Foc_observer.PLL_Interg;
    Foc_observer.PLL_Ui = Limit_Sat(Foc_observer.PLL_Ui, pll_ui_lim, -pll_ui_lim);

    /* 角度积分 */
    Foc_observer.Theta += Foc_observer.PLL_Ui;

    /* 角度归一化 [0, 2π) */
    while (Foc_observer.Theta >= 2.0f * PI) Foc_observer.Theta -= 2.0f * PI;
    while (Foc_observer.Theta < 0.0f)       Foc_observer.Theta += 2.0f * PI;

    /* ---- Step 8: 电频率计算与低通滤波 ---- */
    speed_acc += Foc_observer.PLL_Ui;
    speed_calc_cnt++;

    if (speed_calc_cnt >= 8)
    {
        speed_now = speed_acc / (8.0f * Foc_observer.Ctrl_ts * 2.0f * PI);
        Foc_observer.speed_hz = Foc_observer.speed_hz * 0.95f + speed_now * 0.05f;
        speed_acc = 0.0f;
        speed_calc_cnt = 0;
    }

    /* ---- PLL 增益恢复 ramp ---- */
    if (Foc_observer.pll_ramp_active)
    {
        Foc_observer.PLL_kp += (Foc_observer.PLL_kp_target - Foc_observer.PLL_kp) * 0.02f;
        Foc_observer.PLL_ki += (Foc_observer.PLL_ki_target - Foc_observer.PLL_ki) * 0.02f;

        if (fabsf(Foc_observer.PLL_kp - Foc_observer.PLL_kp_target) < 0.05f &&
            fabsf(Foc_observer.PLL_ki - Foc_observer.PLL_ki_target) < 0.05f)
        {
            Foc_observer.PLL_kp = Foc_observer.PLL_kp_target;
            Foc_observer.PLL_ki = Foc_observer.PLL_ki_target;
            Foc_observer.pll_ramp_active = 0;
        }
    }
}

void Observer_Run_Luenberger(void)
{
    // TODO: 龙伯格观测器
}

void Observer_Run_SMO(void)
{
    // TODO: 滑模观测器
}

/* ================================================================
 * 初始位置检测 (IPD) — 空壳, 待实现
 * ================================================================ */

/**
 * 六脉冲注入定位
 * 依次注入 6 个方向电压矢量, 比较电流响应幅值判断转子 N/S 极方向.
 * 适用于凸极电机 (Ld ≠ Lq), 静止时检测.
 */
void IPD_SixPulse_Injection(void)
{
    // TODO: 六脉冲注入
    //   1. 依次输出 (1,0,0) (1,1,0) (0,1,0) (0,1,1) (0,0,1) (1,0,1) 六个电压矢量
    //   2. 每矢量持续 ~100us, 测量电流响应幅值
    //   3. 最大电流响应的矢量方向即为 N 极方向 (±180° 歧义)
    //   4. 二次判断用饱和效应区分 N/S
}

/**
 * 高频注入定位 (HFI)
 * 在 αβ 轴注入高频正弦电压 (如 1kHz, 10V), 提取高频电流响应包络,
 * 通过外差法解调出转子位置.
 * 适用于凸极电机, 静止及低速均可.
 */
void IPD_HFI_Injection(void)
{
    // TODO: 高频注入
    //   1. 注入 Vh·cos(ωh·t) 到 α 轴 / Vh·sin(ωh·t) 到 β 轴
    //   2. 采样高频电流 iαh, iβh
    //   3. 带通滤波器提取 ωh 分量
    //   4. 外差解调 → PLL 跟踪 → 估计角度
}

/**
 * 获取转子电角度
 * 根据 OBSERVER_TYPE 选择观测器 → 将 Theta(rad) 转换为 IQAngle(0~4095)
 * 由 FOC 控制循环在 UVW_Axis_DQ 之前调用
 */
void Angel_Get(void)
{
    /* 根据编译期观测器类型分派 */
#if OBSERVER_TYPE == OBS_FLUX_VOLTAGE
    Observer_Run();
#elif OBSERVER_TYPE == OBS_HYBRID_ACTIVE_FLUX
    Observer_Run_HybirdFlux();
#elif OBSERVER_TYPE == OBS_EKF
    Observer_Run_EKF();
#elif OBSERVER_TYPE == OBS_LUENBERGER
    Observer_Run_Luenberger();
#elif OBSERVER_TYPE == OBS_SMO
    Observer_Run_SMO();
#else
    Observer_Run();  // fallback
#endif

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
