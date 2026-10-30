/**
 * @file    InvProtect.c
 * @author  nono <nono_1007@foxmail.com>
 * @brief   逆变器保护: 位掩码故障检测 + 反复触发锁存
 */
#include "InvProtect.h"

static uint16_t fault_latch = FAULT_NONE;       // 当前故障位掩码
static uint8_t  protect_enabled = 0;             // 校准完成后置1
static uint8_t  clear_count = 0;                 // 本次上电累计清除次数

static uint16_t InvProtectDetectFaults(void)
{
    uint16_t new_fault = FAULT_NONE;
    float iu = Volt_CurrPara.PhaseU_Curr;
    float iv = Volt_CurrPara.PhaseV_Curr;
    float iw = Volt_CurrPara.PhaseW_Curr;
    float ibus = Volt_CurrPara.BUS_Curr;
    float vbus = Volt_CurrPara.BUS_Voltage;

    if (iu > OC_THRESHOLD_A || iu < -OC_THRESHOLD_A) new_fault |= FAULT_OC_U;
    if (iv > OC_THRESHOLD_A || iv < -OC_THRESHOLD_A) new_fault |= FAULT_OC_V;
    if (iw > OC_THRESHOLD_A || iw < -OC_THRESHOLD_A) new_fault |= FAULT_OC_W;
    if (ibus > OC_BUS_THRESHOLD_A || ibus < -OC_BUS_THRESHOLD_A) new_fault |= FAULT_OC_BUS;
    if (vbus > OV_THRESHOLD_V) new_fault |= FAULT_OV;
    if (vbus < UV_THRESHOLD_V) new_fault |= FAULT_UV;
    if (ntc.ot_fault) new_fault |= FAULT_OT;

    return new_fault;
}

void InvProtectCheck(void)
{
    if (protect_enabled == 0U) {
        return;
    }

    /* 所有已检测故障均锁存，必须显式清除；锁存位只能重新上电清除。 */
    fault_latch |= InvProtectDetectFaults();

    if (fault_latch != FAULT_NONE) {
        motor.Fault_DTC = fault_latch;
        if (motor.Control_Mode != MOTOR_MODE_STOP || PwmIsEnabled()) {
            MotorControlStop();
        }
    } else {
        motor.Fault_DTC = FAULT_NONE;
    }
}

uint16_t InvProtectGetFault(void)
{
    return fault_latch;
}

void InvProtectEnable(void)
{
    protect_enabled = 1U;
}

void InvProtectClear(void)
{
    uint16_t active_fault;
    uint16_t clearable;

    if (protect_enabled == 0U) {
        return;
    }

    active_fault = InvProtectDetectFaults();
    if (active_fault != FAULT_NONE) {
        fault_latch |= active_fault;
        motor.Fault_DTC = fault_latch;
        return;
    }

    clearable = fault_latch & FAULT_CLEARABLE_MASK;
    if (clearable == FAULT_NONE) {
        return;
    }

    clear_count++;
    if (clear_count > FAULT_CLEAR_MAX) {
        /* 本次上电已清除超过5次，升级为锁存，此后只能重新上电 */
        fault_latch |= FAULT_REPEAT_OC;
        motor.Fault_DTC = fault_latch;
        return;
    }

    /* 只清除可清除部分，保留锁存故障 */
    fault_latch &= ~FAULT_CLEARABLE_MASK;
    motor.Fault_DTC = fault_latch;
}
