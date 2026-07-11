/**
 * @file    NTC.h
 * @author  nono <nono_1007@foxmail.com>
 * @brief   NTC 10K B=3950 温度采集 (PB1/PB12/PB2 三路)
 */
#ifndef __NTC_H_
#define __NTC_H_

#include "headline.h"

/* ========== NTC 参数 ========== */
#define NTC_R_SERIES    10000.0f   // 分压电阻 10k
#define NTC_R_REF       10000.0f   // NTC 25°C 标称值 10k
#define NTC_B_VALUE     3950.0f    // B 值
#define NTC_T_REF       298.15f    // 25°C = 298.15K

/* ========== 电机温度 NTC 插头检测 ========== */
#define NTC_PB12_ENABLE  0         // 1=启用(NTC1), 0=禁用

/* ========== 过温阈值 ========== */
#define NTC_OT_WARN_C    80.0f
#define NTC_OT_FAULT_C  100.0f

typedef struct {
    float ntc1_c;    // PB12 (需插)
    float ntc2_c;    // PB1
    float ntc3_c;    // PB2
    uint8_t ot_warn;
    uint8_t ot_fault;
} NTC_Data;

extern NTC_Data ntc;

float NTC_AdcToTemp_C(uint16_t adc_raw);
void  NTC_Task(void);     

#endif

