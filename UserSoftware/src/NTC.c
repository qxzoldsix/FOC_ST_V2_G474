/**
 * @file    NTC.c
 * @brief   NTC 10K B=3950 温度采集 + 过温保护
 *
 * 硬件: VCC(3.3V) → 10k → ADC引脚 → NTC 10k → GND
 * PB12 需插头插入才可用, 通过 NTC_PB12_ENABLE 宏控制
 */
#include "headline.h"
#include "NTC.h"

NTC_Data ntc;

extern uint16_t adc1_buff[2];  // [0]=PB12/CH11, [1]=PB1/CH12

/* ---- 温度换算宏 (内联, 省掉函数调用) ---- */
#define ADC2TEMP(raw) do { \
    float _r, _tk; uint16_t _adc = (raw); \
    if (_adc == 0 || _adc >= 4095) { _tk = 0.0f; } \
    else { \
        _r = NTC_R_SERIES * (float)_adc / (4096.0f - (float)_adc); \
        _tk = 1.0f / (1.0f / NTC_T_REF + logf(_r / NTC_R_REF) / NTC_B_VALUE); \
    } \
    (_tk - 273.15f); \
} while(0)

/* ---- 过温判断宏 ---- */
#define CHECK_OT(t, warn, fault) do { \
    if ((t) > NTC_OT_FAULT_C) { (fault) = 1; } \
    if ((t) > NTC_OT_WARN_C)  { (warn)  = 1; } \
} while(0)

/**
 * @brief 后台任务入口: 采样 + 换算 + 保护, 一次搞定
 */
void NTC_Task(void)
{
    /* 采样 */
#if NTC_PB12_ENABLE
    ntc.ntc1_c = NTC_AdcToTemp_C(adc1_buff[0]);  // PB12
#else
    ntc.ntc1_c = 0.0f;
#endif
    ntc.ntc2_c = NTC_AdcToTemp_C(adc1_buff[1]);   // PB1
    ntc.ntc3_c = NTC_AdcToTemp_C(ADC2->JDR2);      // PB2

    /* 过温检测 */
    ntc.ot_warn  = 0;
    ntc.ot_fault = 0;
    CHECK_OT(ntc.ntc1_c, ntc.ot_warn, ntc.ot_fault);
    CHECK_OT(ntc.ntc2_c, ntc.ot_warn, ntc.ot_fault);
    CHECK_OT(ntc.ntc3_c, ntc.ot_warn, ntc.ot_fault);
}

