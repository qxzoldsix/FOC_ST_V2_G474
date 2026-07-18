/**
 * @file    ParamId.c
 * @brief   电机参数辨识 — 顺序状态机 (每周期调用一次, 无 for)
 *
 * 辨识序列:
 *   ALIGN → RS_VOLT → RS_WAIT → RS_SAMP → LD_PULSE → LQ_PULSE → LS_CALC → DONE
 *
 * 算法参考: AxDr_L 的 id_Rs / id_Ls / id_Fs 框架
 */
#include "headline.h"
#include "ParamId.h"

ParamId_t pid;

/* 前向声明 */
static void id_align(void);
static void id_rs_volt(void);
static void id_rs_wait(void);
static void id_rs_samp(void);
static void id_ld_pulse(void);
static void id_lq_pulse(void);
static void id_ls_calc(void);

void ParamId_Start(void)
{
    pid.step = ID_STEP_ALIGN;
    pid.tick = 0;
    pid.done = 0;
    pid.Rs   = 0.0f;
    pid.Ld   = 0.0f;
    pid.Lq   = 0.0f;
    /* PWM 已运行，直接开始 */
}

void ParamId_Run(void)
{
    if (pid.done) return;

    pid.tick++;   // 每周期 50us → tick*50us = 时间
    UVW_Axis_DQ(); // Clarke+Park → 刷新 PARK_PCurr.Ds/Qs

    switch (pid.step)
    {
        case ID_STEP_ALIGN:   id_align();    break;
        case ID_STEP_RS_VOLT: id_rs_volt();  break;
        case ID_STEP_RS_WAIT: id_rs_wait();  break;
        case ID_STEP_RS_SAMP: id_rs_samp();  break;
        case ID_STEP_LD_PULSE:id_ld_pulse(); break;
        case ID_STEP_LQ_PULSE:id_lq_pulse(); break;
        case ID_STEP_LS_CALC: id_ls_calc();  break;
        default: break;
    }
}

/* ================================================================
 * Step 1: 预对齐 — 注入 d轴电压，将转子拉到零位
 * ================================================================ */
static void id_align(void)
{
    motor.V_d = ID_ALIGN_VOLT;   
    motor.V_q = 0.0f;
    motor.IQAngle = 0;
    FOC_Svpwm_dq();

    if (pid.tick > 2000)   // 100ms 对齐
    {
        pid.tick = 0;
        pid.step = ID_STEP_RS_VOLT;
    }
}

/* ================================================================
 * Step 2: Rs 辨识 — d轴注入直流电压
 * ================================================================ */
static void id_rs_volt(void)
{
    motor.V_d = ID_RS_VOLT;
    motor.V_q = 0.0f;
    motor.IQAngle = 0;
    FOC_Svpwm_dq();

    pid.step = ID_STEP_RS_WAIT;
    pid.tick = 0;
}

/* ================================================================
 * Step 3: 等待电流稳定 (~100ms)
 * ================================================================ */
static void id_rs_wait(void)
{
    motor.V_d = ID_RS_VOLT;
    motor.V_q = 0.0f;
    motor.IQAngle = 0;
    FOC_Svpwm_dq();

    if (pid.tick > 2000)   // 100ms
    {
        pid.tick = 0;
        pid.step = ID_STEP_RS_SAMP;
    }
}

/* ================================================================
 * Step 4: 采样 d轴电流, 计算 Rs = Vd_actual / Id
 * Vd_actual = Vd_pu × Vbus/2 (SVPWM 标幺基准)
 * ================================================================ */
static void id_rs_samp(void)
{
    float v_actual;

    motor.V_d = ID_RS_VOLT;
    motor.V_q = 0.0f;
    motor.IQAngle = 0;
    FOC_Svpwm_dq();

    pid.i_d = PARK_PCurr.Ds;
    v_actual = ID_RS_VOLT * Volt_CurrPara.BUS_Voltage * 0.5f;  // pu→V

    if (pid.i_d > 0.05f)
        pid.Rs = v_actual / pid.i_d;

    pid.tick = 0;
    pid.step = ID_STEP_LD_PULSE;
}

/* ================================================================
 * Step 5: Ld 辨识 — d轴脉冲注入, 测 di/dt
 * ================================================================ */
static void id_ld_pulse(void)
{
    motor.V_d = ID_LS_VOLT;
    motor.V_q = 0.0f;
    motor.IQAngle = 0;

    if (pid.tick == 1)
    {
        pid.i_d0 = PARK_PCurr.Ds;   // 采基线
        FOC_Svpwm_dq();              // PWM 预装载, 下周期生效
        return;
    }
    if (pid.tick == 2)
    {
        FOC_Svpwm_dq();              // 等待 ADC 采到新电流
        return;
    }
    /* tick >= 3: 电流已上升, 采样计算 */
    FOC_Svpwm_dq();
    pid.i_d = PARK_PCurr.Ds;
    pid.di_dt = (pid.i_d - pid.i_d0) / ((pid.tick - 1) * 5e-5f);

    if (pid.di_dt > 10.0f)
        pid.Ld = (ID_LS_VOLT * Volt_CurrPara.BUS_Voltage * 0.5f) / pid.di_dt;

    pid.tick = 0;
    pid.step = ID_STEP_LQ_PULSE;
}

/* ================================================================
 * Step 6: Lq 辨识 — q轴正反脉冲, 抵消净转矩防转子转动
 * ================================================================ */
static void id_lq_pulse(void)
{
    if (pid.tick == 1) {
        pid.i_d0 = PARK_PCurr.Qs;
        motor.V_q =  ID_LS_VOLT;
        FOC_Svpwm_dq();
        return;
    }
    if (pid.tick == 2) { FOC_Svpwm_dq(); return; }  // 等 +Vq 生效

    /* tick=3: 采 +Vq 电流, 立刻反转 */
    if (pid.tick == 3) {
        pid.i_d = PARK_PCurr.Qs;
        motor.V_q = -ID_LS_VOLT;
        FOC_Svpwm_dq();
        return;
    }
    if (pid.tick == 4) { FOC_Svpwm_dq(); return; }  // 等 -Vq 生效

    /* tick=5: 采 -Vq 电流, 取平均 di/dt 消去转矩偏置 */
    pid.i_d = (pid.i_d + PARK_PCurr.Qs) * 0.5f;     // 平均电流
    pid.di_dt = (pid.i_d - pid.i_d0) / ((pid.tick - 1) * 5e-5f);

    if (pid.di_dt > 10.0f)
        pid.Lq = (ID_LS_VOLT * Volt_CurrPara.BUS_Voltage * 0.5f) / pid.di_dt;

    pid.tick = 0;
    pid.step = ID_STEP_LS_CALC;
}

/* ================================================================
 * Step 7: 计算完成, 写入全局参数, 停机
 * ================================================================ */
static void id_ls_calc(void)
{
    motor.V_d = 0.0f;
    motor.V_q = 0.0f;
    FOC_Svpwm_dq();

    pid.done = 1;
    pid.step = ID_STEP_DONE;
    motor.Control_Mode = 0;  // 辨识完成, 回停机
}
