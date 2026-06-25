#ifndef __ADC_SAMPLE_H_
#define __ADC_SAMPLE_H_

#include "headline.h"

#define V_REF        3.3f    // ADC�ο���ѹ 3.3V
#define ADC_MAX      4096.0f // 12λADC���ֵ 2^12 = 4096
#define R_UP         15500.0f// �Ϸ�ѹ���� 20k��
#define R_DOWN       1000.0f // �·�ѹ���� 1k��


#define SenseRes     0.005f   //��������

// ��ѹ���� = (���� + ����) / ����
#define Curr_OP				20.0f
#define VOLTAGE_OP   ((R_UP + R_DOWN) / R_DOWN)  
// ʵ�ʵ�ѹ = ADCԭʼֵ �� VOLTAGE_RATIO
#define VOLTAGE_RATIO (V_REF / ADC_MAX * VOLTAGE_OP)
#define Curr_Ratio    (V_REF / ADC_MAX/SenseRes/Curr_OP)
#define OFFSET_CALIB_CNT 1000

#define NOMINAL_BUS_VOLTAGE  24.0f  // 标称母线电压 (V), 用于 SVPWM 前馈补偿

typedef struct {
    uint16_t PhaseU_Raw;    // U�����ԭʼ����ֵ
    uint16_t PhaseV_Raw;    // V�����ԭʼ����ֵ
    uint16_t PhaseW_Raw;    // W�����ԭʼ����ֵ
    uint16_t BUS_Curr_Raw;  // ĸ�ߵ���ԭʼ����ֵ

    uint16_t OffsetBUS_Raw;     // ĸ�ߵ���ƫ�ã�У׼ֵ��
    uint16_t OffsetPhaseU_Raw;  // U�����ƫ�ã�У׼ֵ��
    uint16_t OffsetPhaseV_Raw;  // V�����ƫ�ã�У׼ֵ��
    uint16_t OffsetPhaseW_Raw;  // W�����ƫ�ã�У׼ֵ��

    uint16_t BUS_Volt_Raw;  // ĸ�ߵ�ѹԭʼ����ֵ
} ADCSample , *p_ADCSample ; 
extern  ADCSample   ADCSampPara;
#define  ADCSamp_DEFAULTS  {0,0,0,0,0,0,0,0,0}   // ��ʼ������
typedef struct {
	float   BUS_Curr ;     // ĸ�ߵ��� DC Bus  Current
	float   PhaseU_Curr;   // U����� Phase U Current
	float   PhaseV_Curr;   // V�����Phase V Current
	float   PhaseW_Curr;   // W�����Phase W Current

	float   BUS_Voltage ;  //ĸ�ߵ�ѹDC Bus  Voltage

	float   Svpwm_Km_BackwS;

}Volt_Curr;
extern  Volt_Curr   Volt_CurrPara;
#define  Volt_Curr_DEFAULTS  {0.0,0.0,0.0,0.0,0.0,0.0}   // ��ʼ������



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
