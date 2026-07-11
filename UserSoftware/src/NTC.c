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

/* ---- 温度换算 ---- */
float NTC_AdcToTemp_C(uint16_t adc_raw)
{
    float r, tk;
    if (adc_raw < 20 || adc_raw > 4076) return 0.0f;
    r  = NTC_R_SERIES * (float)adc_raw / (4096.0f - (float)adc_raw);
    tk = 1.0f / (1.0f / NTC_T_REF + logf(r / NTC_R_REF) / NTC_B_VALUE);
    return tk - 273.15f;
}

/* ---- 过温判断 ---- */
#define CHECK_OT(t) do { \
    if ((t) > NTC_OT_FAULT_C) { ntc.ot_fault = 1; } \
    if ((t) > NTC_OT_WARN_C)  { ntc.ot_warn  = 1; } \
} while(0)

/**
 * @brief 温度很热你忍一下
 */
void NTC_Task(void)
{
    /* 采样 & 换算 */
#if NTC_PB12_ENABLE
    ntc.ntc1_c = NTC_AdcToTemp_C(adc1_buff[0]);  // PB12 (需插头)
#else
    ntc.ntc1_c = 0.0f;
#endif
    ntc.ntc2_c = NTC_AdcToTemp_C(adc1_buff[1]);   // PB1
    ntc.ntc3_c = NTC_AdcToTemp_C(ADC2->JDR2);      // PB2

    /* 过温检测 */
    ntc.ot_warn  = 0;
    ntc.ot_fault = 0;
    CHECK_OT(ntc.ntc1_c);
    CHECK_OT(ntc.ntc2_c);
    CHECK_OT(ntc.ntc3_c);
}

