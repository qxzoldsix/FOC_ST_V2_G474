/**
 * @file    InvProtect.c
 * @author  nono <nono_1007@foxmail.com>
 * @brief   逆变器保护: 位掩码故障检测 + 反复触发锁存
 */
#include "InvProtect.h"

static uint16_t fault_latch = FAULT_NONE;       // 当前故障位掩码
static uint8_t  protect_enabled = 0;             // 校准完成后置1
static uint8_t  clear_count = 0;                 // 本次上电累计清除次数

void InvProtect_Check(void)
{
    if (!protect_enabled) return;

    uint16_t new_fault = FAULT_NONE;

    /* === 逐个独立检测，可同时捕获多个故障 === */
    float iu = Volt_CurrPara.PhaseU_Curr;
    float iv = Volt_CurrPara.PhaseV_Curr;
    float iw = Volt_CurrPara.PhaseW_Curr;

    if (iu > OC_THRESHOLD_A || iu < -OC_THRESHOLD_A) new_fault |= FAULT_OC_U;
    if (iv > OC_THRESHOLD_A || iv < -OC_THRESHOLD_A) new_fault |= FAULT_OC_V;
    if (iw > OC_THRESHOLD_A || iw < -OC_THRESHOLD_A) new_fault |= FAULT_OC_W;

    float ibus = Volt_CurrPara.BUS_Curr;
    if (ibus > OC_BUS_THRESHOLD_A || ibus < -OC_BUS_THRESHOLD_A) new_fault |= FAULT_OC_BUS;

    float vbus = Volt_CurrPara.BUS_Voltage;
    if (vbus > OV_THRESHOLD_V) new_fault |= FAULT_OV;
    if (vbus < UV_THRESHOLD_V) new_fault |= FAULT_UV;

    if (ntc.ot_fault) new_fault |= FAULT_OT;

    /* === 锁存故障不会被新检测覆盖 === */
    fault_latch = (fault_latch & FAULT_LATCH_MASK) | new_fault;

    if (fault_latch != FAULT_NONE) {
        motor.Control_Mode = 0;
        motor.Fault_DTC = fault_latch;
        Foc_Pwm_Stop();
    } else {
        motor.Fault_DTC = FAULT_NONE;
    }
}

uint16_t InvProtect_GetFault(void)
{
    return fault_latch;
}

void InvProtect_Enable(void)
{
    protect_enabled = 1;
}

void InvProtect_Clear(void)
{
    uint16_t clearable = fault_latch & FAULT_CLEARABLE_MASK;

    if (clearable == FAULT_NONE) return;  // 只有锁存故障或已无故障

    clear_count++;
    if (clear_count > 5) {
        /* 本次上电已清除超过5次，升级为锁存，此后只能重新上电 */
        fault_latch |= FAULT_REPEAT_OC;
        motor.Fault_DTC = fault_latch;
        return;
    }

    /* 只清除可清除部分，保留锁存故障 */
    fault_latch &= ~FAULT_CLEARABLE_MASK;
    motor.Fault_DTC = fault_latch;
}
