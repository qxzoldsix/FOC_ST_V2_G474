/**
 * @file    ADC_SAMPLE.h
 * @author  nono <nono_1007@foxmail.com>
 * @brief   ADC 采样参数: 分压比、电流比例、校准 + 数据结构
 */
#ifndef __ADC_SAMPLE_H_
#define __ADC_SAMPLE_H_

#include "headline.h"

#define V_REF        3.3f    // ADC 参考电压 3.3V
#define ADC_MAX      4096.0f // 12位 ADC 最大值 2^12 = 4096
#define R_UP         15500.0f// 上分压电阻 15.5k
#define R_DOWN       1000.0f // 下分压电阻 1k


#define SenseRes     0.005f   // 采样电阻 5mΩ

// 电压比例 = (上电阻 + 下电阻) / 下电阻
#define Curr_OP				20.0f
#define VOLTAGE_OP   ((R_UP + R_DOWN) / R_DOWN)  
// 实际电压 = ADC原始值 × VOLTAGE_RATIO
#define VOLTAGE_RATIO (V_REF / ADC_MAX * VOLTAGE_OP)
#define Curr_Ratio    (V_REF / ADC_MAX/SenseRes/Curr_OP)
#define OFFSET_CALIB_CNT 1000

#define NOMINAL_BUS_VOLTAGE  24.0f  // 标称母线电压 (V), 用于 SVPWM 前馈补偿

typedef struct {
    uint16_t PhaseU_Raw;    // U 相电流原始采样值
    uint16_t PhaseV_Raw;    // V 相电流原始采样值
    uint16_t PhaseW_Raw;    // W 相电流原始采样值
    uint16_t BUS_Curr_Raw;  // 母线电流原始采样值

    uint16_t OffsetBUS_Raw;     // 母线电流偏置（校准值）
    uint16_t OffsetPhaseU_Raw;  // U 相电流偏置（校准值）
    uint16_t OffsetPhaseV_Raw;  // V 相电流偏置（校准值）
    uint16_t OffsetPhaseW_Raw;  // W 相电流偏置（校准值）

    uint16_t BUS_Volt_Raw;  // 母线电压原始采样值
} ADCSample , *p_ADCSample ; 
extern  ADCSample   ADCSampPara;
#define  ADCSamp_DEFAULTS  {0,0,0,0,0,0,0,0,0}   // 初始化宏
typedef struct {
	float   BUS_Curr ;     // 母线电流 DC Bus Current
	float   PhaseU_Curr;   // U 相电流 Phase U Current
	float   PhaseV_Curr;   // V 相电流 Phase V Current
	float   PhaseW_Curr;   // W 相电流 Phase W Current

	float   BUS_Voltage ;  // 母线电压 DC Bus Voltage

	float   Svpwm_Km_BackwS;

}Volt_Curr;
extern  Volt_Curr   Volt_CurrPara;
#define  Volt_Curr_DEFAULTS  {0.0,0.0,0.0,0.0,0.0,0.0}   // 初始化宏



//extern s16 Calibrattion_Val;
_RAM_FUNC void Foc_Para_Calc(void);
_RAM_FUNC void Foc_Adc_Sample(p_ADCSample pADC);
void ZeroCurrOffset_Calibration(void);
//void ADC_Function_Init(void);
//void ADC_DMA_Init(u32 ppadr,u32 memadr,u16 bufsize);
//u16 Get_ConversionVal(s16 val);
//u16 Get_ADC_Average(u8 ch, u8 times);

//void Adc_Sample(void);
//void Deal_Data(void);

#endif
