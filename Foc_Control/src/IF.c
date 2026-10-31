/**
 * @file IF.c
 * @brief MODE4 I/F开环：电流爬升/定位、频率斜坡、强制角度、电流环与SVPWM。
 * 强制坐标系Id=电流幅值、Iq=0，实际转子有滞后角时仍可产生转矩。
 * 不运行观测器或速度PI；CurrentHz是电频率命令，不冒充实测机械转速。
 */
#include "headline.h"
#include "IF.h"

float IfCurrentTargetA = 0.0f; /* 目标由用户手动赋值，不自动设置电流幅值。 */
IfControlError IfError = IF_ERROR_NONE;
GXieLv IF_HzRamp = {0.0f, 0.0f, IF_FREQ_RAMP_HZ_PER_SECOND * FOC_CONTROL_TS_S, 0U, 0U};
static uint32_t alignTicks;

void IfControlStart(void)
{
    alignTicks = 0U;
    IfError = IF_ERROR_NONE;
    motor.OpenTheta = 0.0f;
    motor.IQAngle = 0;
    motor.CurrentHz = 0.0f;
    motor.SpeedRPM = 0.0f;
    I_d_GXieLv.XieLv_Grad = IF_CURRENT_RAMP_A_PER_SECOND * FOC_CONTROL_TS_S;
    IF_HzRamp.XieLv_X = motor.TargetHz;
    IF_HzRamp.XieLv_Y = 0.0f;
    IF_HzRamp.Timer_Count = 0U;
    IF_HzRamp.Grad_Timer = 0U;
    /* 电流PI和给定的运行量由入模入口统一清零，保留已有调试PI参数。 */
}

/* 逐拍检查输入；开环不代表取消电流采样、限流或母线保护。 */
static IfControlError IfCheckInputs(float targetCurrent, float targetHz)
{
    if (!isfinite(targetCurrent) || targetCurrent <= 0.0f || targetCurrent > IF_CURRENT_MAX_A ||
        !isfinite(targetHz) || fabsf(targetHz) > IF_FREQ_MAX_HZ ||
        !isfinite(I_d_GXieLv.XieLv_Grad) || I_d_GXieLv.XieLv_Grad < 0.0f ||
        !isfinite(IF_HzRamp.XieLv_Grad) || IF_HzRamp.XieLv_Grad < 0.0f) {
        return IF_ERROR_COMMAND;
    }
    const float current[3] = {Volt_CurrPara.PhaseU_Curr,
                             Volt_CurrPara.PhaseV_Curr,
                             Volt_CurrPara.PhaseW_Curr};
    const uint16_t raw[3] = {ADCSampPara.PhaseU_Raw,
                            ADCSampPara.PhaseV_Raw,
                            ADCSampPara.PhaseW_Raw};
    for (unsigned i = 0U; i < 3U; i++) {
        /* 12位ADC上下限各保留16个刻度，削顶后不能相信换算电流。 */
        if (!isfinite(current[i]) || fabsf(current[i]) > IF_PHASE_CURRENT_LIMIT_A ||
            raw[i] <= 16U || raw[i] >= 4079U) return IF_ERROR_CURRENT;
    }
    return IF_ERROR_NONE;
}

void IfControlRun(void)
{
    float targetCurrent = IfCurrentTargetA;
    float targetHz = motor.TargetHz;
    IfError = IfCheckInputs(targetCurrent, targetHz);
    if (IfError != IF_ERROR_NONE) {
        MotorControlStop();
        return;
    }

    /* 电流幅值从零爬升；运行中修改幅值同样经过斜坡。 */
    I_d_GXieLv.XieLv_X = targetCurrent;
    Grad_XieLv(&I_d_GXieLv);
    I_q_GXieLv.XieLv_X = 0.0f;
    I_q_GXieLv.XieLv_Y = 0.0f;

    /* 先固定角度建立电流，随后只按频率给定运行；不切到MODE2。 */
    IF_HzRamp.XieLv_X = targetHz;
    if (alignTicks < IF_ALIGN_HOLD_MS * FOC_CONTROL_FREQ_HZ / 1000U) {
        if (I_d_GXieLv.XieLv_Y == targetCurrent) alignTicks++;
    } else {
        Grad_XieLv(&IF_HzRamp);
    }
    motor.CurrentHz = IF_HzRamp.XieLv_Y;

    motor.OpenTheta += 2.0f * PI * motor.CurrentHz * FOC_CONTROL_TS_S;
    if (motor.OpenTheta >= 2.0f * PI) motor.OpenTheta -= 2.0f * PI;
    if (motor.OpenTheta < 0.0f) motor.OpenTheta += 2.0f * PI;
    motor.IQAngle = (int16_t)(motor.OpenTheta * (4096.0f / (2.0f * PI)));
    UVW_Axis_DQ();
    Idq_FOC();
    FOC_Svpwm_dq();
}
