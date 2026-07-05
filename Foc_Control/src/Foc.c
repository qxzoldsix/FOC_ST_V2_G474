/**
 * @file    Foc.c
 * @author  nono <nono_1007@foxmail.com>
 * @brief   Clarke/Park/iPark 变换、速度环、电流环、SVPWM 输出
 */
#include "headline.h"
GXieLv       SpeedRpm_GXieLv = GXieLv_DEFAULTS;
GXieLv       I_q_GXieLv = GXieLv_DEFAULTS;
GXieLv       I_d_GXieLv = GXieLv_DEFAULTS;
CLARKE       CLARKE_PCurr = CLARKE_DEFAULTS;
PARK         PARK_PCurr = PARK_DEFAULTS;
IPARK        IPARK_PVdq = IPARK_DEFAULTS;
Ang_SinCos   Park_SinCos = Ang_SinCos_DEFAULTS;

#define CURRENT_BETA_SIGN   (1.0f)
#define PARK_ANGLE_REVERSE  0
#define SPEED_IQ_SIGN       (-1.0f)

/**
 * Clarke 变换（等幅值）
 * 三相静止 → 两相静止: Iα = Ia, Iβ = (Ia + 2*Ib) / √3
 */
void  CLARKE_Cale(p_CLARKE  pV)
{
    pV->Alpha = pV->Us;
    pV->Beta = CURRENT_BETA_SIGN * (pV->Us + pV->Vs*2)*0.577350269f;  // 1/√3 ≈ 0.577350269
}

/**
 * Park 变换
 * 两相静止 → 两相旋转: 将 αβ 轴电流投影到 dq 旋转坐标系
 */
void  PARK_Cale(p_PARK pV)
{
    pV->Ds =  pV->Alpha * pV->Cosine + pV->Beta * pV->Sine;
    pV->Qs =  pV->Beta * pV->Cosine  - pV->Alpha * pV->Sine;
}

/**
 * 反 Park 变换
 * 两相旋转 → 两相静止: 将 dq 轴电压还原为 αβ 轴电压
 */
void  IPARK_Cale(p_IPARK pV)
{
    pV->Alpha = pV->Ds * pV->Cosine - pV->Qs * pV->Sine ;
    pV->Beta  = pV->Qs * pV->Cosine + pV->Ds * pV->Sine ;
}


/**
 * UVW → DQ 坐标变换完整链路
 * 三相电流 → Clarke → Park，得到 Id/Iq 反馈值
 */
void UVW_Axis_DQ(void){
    CLARKE_PCurr.Us  = Volt_CurrPara.PhaseU_Curr;
#if CURRENT_USE_W_AS_V
    CLARKE_PCurr.Vs = Volt_CurrPara.PhaseW_Curr;
#else
    CLARKE_PCurr.Vs =   Volt_CurrPara.PhaseV_Curr;
#endif
    CLARKE_Cale((p_CLARKE)&CLARKE_PCurr);
#if PARK_ANGLE_REVERSE
    Park_SinCos.table_Angle = (uint16_t)((4096 - motor.IQAngle) & 0x0FFF);
#else
    Park_SinCos.table_Angle = motor.IQAngle;
#endif
    SinCos_Table((p_Ang_SinCos)&Park_SinCos);   // 查表获取 sinθ/cosθ
    PARK_PCurr.Sine =Park_SinCos.table_Sin;
    PARK_PCurr.Cosine =Park_SinCos.table_Cos;
    PARK_PCurr.Alpha =CLARKE_PCurr.Alpha;
    PARK_PCurr.Beta=CLARKE_PCurr.Beta;
    PARK_Cale((p_PARK)&PARK_PCurr);
}

/**
 * 速度环 FOC
 * 目标转速经斜坡发生器后，PI 控制器输出 Iq 电流给定值
 */
void Speed_FOC(void){

    SpeedRpm_GXieLv.Timer_Count++;

    if(SpeedRpm_GXieLv.Timer_Count>SpeedRpm_GXieLv.Grad_Timer)
    {
        SpeedRpm_GXieLv.Timer_Count=0;
        Grad_XieLv((p_GXieLv)&SpeedRpm_GXieLv);        // 目标转速斜坡
        pi_spd.Ref = SpeedRpm_GXieLv.XieLv_Y;           // 速度环给定
        pi_spd.Fbk = motor.SpeedRPM;                    // 速度环反馈

        PID_controller((p_PI_Control)&pi_spd);          // 速度环 PI
        pi_spd.OutF=pi_spd.OutF*GM_Low_Lass_A+pi_spd.Out*GM_Low_Lass_B;  // 一阶低通滤波
    }
    I_q_GXieLv.XieLv_Y = pi_spd.OutF * SPEED_IQ_SIGN;                    // 输出给电流环 Iq 给定
}

/**
 * 电流环 Idq_FOC
 * d 轴: 励磁/弱磁电流控制
 * q 轴: 转矩电流控制（含电压限幅）
 */
void Idq_FOC(void)
{
    float Us_Limit = 0;

    /* ---------- d 轴电流环 ---------- */
    pi_id.Ref = I_d_GXieLv.XieLv_Y;                     // d 轴电流给定
    pi_id.Fbk = PARK_PCurr.Ds;                          // d 轴电流反馈
    PID_controller((p_PI_Control)&pi_id);               // d 轴 PI
    pi_id.OutF = pi_id.OutF * GM_Low_Lass_A + pi_id.Out * GM_Low_Lass_B;  // 低通滤波
    motor.V_d = pi_id.OutF;                             // d 轴电压输出

    /* ---------- q 轴电压限幅（电压圆约束）----------
     * Vd/Vq 是 per-unit, Bus_Voltage 是实际 V, 统一到 per-unit
     */
    {
        float bus_pu = Volt_CurrPara.BUS_Voltage / NOMINAL_BUS_VOLTAGE;
        float val = bus_pu * bus_pu - motor.V_d * motor.V_d;
        Us_Limit = (val > 0.0f) ? sqrtf(val) : 0.0f;
    }
    pi_iq.Umax =  Us_Limit;
    pi_iq.Umin = -Us_Limit;

    /* ---------- q 轴电流环 ---------- */
    pi_iq.Ref = I_q_GXieLv.XieLv_Y;                     // q 轴电流给定
    pi_iq.Fbk = PARK_PCurr.Qs;                          // q 轴电流反馈
    PID_controller((p_PI_Control)&pi_iq);               // q 轴 PI
    pi_iq.OutF = pi_iq.OutF * GM_Low_Lass_A + pi_iq.Out * GM_Low_Lass_B;  
    motor.V_q = pi_iq.OutF;                             // q 轴电压输出
}

/**
 * FOC_Svpwm_dq — 反 Park + SVPWM 输出
 * Vd/Vq → 反 Park → Vα/Vβ → SVPWM → PWM 占空比更新
 */
void FOC_Svpwm_dq(void)
{
    IPARK_PVdq.Ds = motor.V_d;
    IPARK_PVdq.Qs = motor.V_q;

    IPARK_PVdq.Sine   = Park_SinCos.table_Sin;
    IPARK_PVdq.Cosine = Park_SinCos.table_Cos;
    IPARK_Cale((p_IPARK)&IPARK_PVdq);                   // 反 Park → Vα/Vβ

    Svpwm_dq.Ualpha = IPARK_PVdq.Alpha;
    Svpwm_dq.Ubeta  = IPARK_PVdq.Beta;
    svpwm_Cale((p_SVPWM)&Svpwm_dq);                     // SVPWM 计算占空比
    PWM_Update_From_SVPWM();                             // 更新 TIM1 CCR
}
