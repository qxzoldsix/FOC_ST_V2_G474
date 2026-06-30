#include "InvProtect.h"

static uint8_t fault_latch = FAULT_NONE;   // 故障锁存
static uint8_t protect_enabled = 0;        // 保护使能标志（校准完成后置1）

/**
 * 逆变器保护检查（每个 PWM 周期调用）
 * 检测过流/过压/欠压，首次触发时锁存故障码并强制停机
 */
void InvProtect_Check(void)
{
    /* 校准未完成 → 跳过保护检查，避免零点偏置未就绪时误触发 */
    if (!protect_enabled) return;

//    /* 已有锁存故障 → 保持，不再重复检测 */
    if (fault_latch != FAULT_NONE) {
        motor.Control_Mode = 0;         // 保持停机
        motor.Fault_DTC = fault_latch;
        return;
    }

    /* ---- 相过流检测 ---- */
    float iu = Volt_CurrPara.PhaseU_Curr;
    float iv = Volt_CurrPara.PhaseV_Curr;
    float iw = Volt_CurrPara.PhaseW_Curr;

    if      (iu > OC_THRESHOLD_A || iu < -OC_THRESHOLD_A) fault_latch = FAULT_OC_U;
    else if (iv > OC_THRESHOLD_A || iv < -OC_THRESHOLD_A) fault_latch = FAULT_OC_V;
    else if (iw > OC_THRESHOLD_A || iw < -OC_THRESHOLD_A) fault_latch = FAULT_OC_W;

    /* ---- 母线过/欠压检测 ---- */
    float vbus = Volt_CurrPara.BUS_Voltage;

    if      (fault_latch == FAULT_NONE && vbus > OV_THRESHOLD_V) fault_latch = FAULT_OV;
    else if (fault_latch == FAULT_NONE && vbus < UV_THRESHOLD_V) fault_latch = FAULT_UV;

//    /* ---- 触发保护: 锁存 + 停机 + 封锁 PWM ---- */
    if (fault_latch != FAULT_NONE) {
        motor.Control_Mode = 0;
        motor.Fault_DTC = fault_latch;
        Foc_Pwm_Stop();
    }
}

/**
 * 启用保护检测（零点校准完成后调用）
 */
void InvProtect_Enable(void)
{
    protect_enabled = 1;
}

/**
 * 清除故障锁存（外部按键或通信调用）
 */
void InvProtect_Clear(void)
{
    fault_latch = FAULT_NONE;
    motor.Fault_DTC = FAULT_NONE;
}
