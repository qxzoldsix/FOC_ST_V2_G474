#include "headline.h"

Volt_Curr Volt_CurrPara = Volt_Curr_DEFAULTS;

extern uint16_t adc1_buff[2];
extern uint16_t adc2_buff[4];
_RAM_FUNC void Foc_Adc_Sample(p_ADCSample pADC)
{
	pADC->PhaseU_Raw = ADC1->JDR3;
	pADC->PhaseV_Raw = ADC1->JDR2;
	pADC->PhaseW_Raw = ADC1->JDR1;

//	pADC->adc.va = adc2_buff[1];
//	pADC->adc.vb = adc2_buff[2];
//	pADC->adc.vc = adc2_buff[3];
	pADC->BUS_Volt_Raw  = ADC2->JDR1;
	pADC->BUS_Curr_Raw  = ADC1->JDR4;   // PA3/ADC1_IN4 母线电流

//    //  Convert ADC values to actual currents with offset compensation and scaling
	Volt_CurrPara.PhaseU_Curr=((float)pADC->PhaseU_Raw-ADCSampPara.OffsetPhaseU_Raw)*Curr_Ratio;
	Volt_CurrPara.PhaseV_Curr=((float)pADC->PhaseV_Raw-ADCSampPara.OffsetPhaseV_Raw)*Curr_Ratio;
	Volt_CurrPara.PhaseW_Curr=((float)pADC->PhaseW_Raw-ADCSampPara.OffsetPhaseW_Raw)*Curr_Ratio;
	Volt_CurrPara.BUS_Curr    =((float)pADC->BUS_Curr_Raw-ADCSampPara.OffsetBUS_Raw)  *Curr_Ratio;
//    pADC->foc.i_a = ((float) pADC->adc.ia - pADC->adc.ia_off) * pADC->board.i_ratio;
//    pADC->foc.i_b = ((float) pADC->adc.ib - pADC->adc.ib_off) * pADC->board.i_ratio;
//    pADC->foc.i_c = ((float) pADC->adc.ic - pADC->adc.ic_off) * pADC->board.i_ratio;
}
_RAM_FUNC void Foc_Para_Calc(void){
    Volt_CurrPara.BUS_Voltage = ADCSampPara.BUS_Volt_Raw * VOLTAGE_RATIO;

    /* 母线电流 = 占空比加权平均（准确的直流侧平均电流）*/
    Volt_CurrPara.BUS_Curr = (Volt_CurrPara.PhaseU_Curr * Svpwm_dq.SVPTa
                            + Volt_CurrPara.PhaseV_Curr * Svpwm_dq.SVPTb
                            + Volt_CurrPara.PhaseW_Curr * Svpwm_dq.SVPTc)
                            / (float)(TIM1_ARR + 1);

    /* SVPWM bus voltage feedforward: Km = Vnom / Vbus */
    if (Volt_CurrPara.BUS_Voltage > 1.0f) {
        Volt_CurrPara.Svpwm_Km_BackwS = NOMINAL_BUS_VOLTAGE / Volt_CurrPara.BUS_Voltage;
        if (Volt_CurrPara.Svpwm_Km_BackwS > 1.3f) Volt_CurrPara.Svpwm_Km_BackwS = 1.3f;
        if (Volt_CurrPara.Svpwm_Km_BackwS < 0.7f) Volt_CurrPara.Svpwm_Km_BackwS = 0.7f;
    } else {
        Volt_CurrPara.Svpwm_Km_BackwS = 1.0f;
    }
}
void ZeroCurrOffset_Calibration(void)
{
    uint32_t sumU = 0, sumV = 0, sumW = 0, sumBus = 0;

    for (uint16_t i = 0; i < OFFSET_CALIB_CNT; i++)
    {
        sumU   += (float)(ADC1->JDR3);
        sumV   += (float)(ADC1->JDR2);
        sumW   += (float)(ADC1->JDR1);
        sumBus += (float)(ADC1->JDR4);

        /* 等待 >1个PWM周期（50µs），确保每次读到不同采样值 */
        for (volatile uint16_t d = 0; d < 500; d++);
    }

    ADCSampPara.OffsetPhaseU_Raw = (uint16_t)(sumU   / OFFSET_CALIB_CNT);
    ADCSampPara.OffsetPhaseV_Raw = (uint16_t)(sumV   / OFFSET_CALIB_CNT);
    ADCSampPara.OffsetPhaseW_Raw = (uint16_t)(sumW   / OFFSET_CALIB_CNT);
    ADCSampPara.OffsetBUS_Raw    = (uint16_t)(sumBus / OFFSET_CALIB_CNT);
}

//void Adc_Sample(void)
//{
//    ADCSampPara.PhaseU_Curr = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
//    ADCSampPara.PhaseV_Curr = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);
//    ADCSampPara.PhaseW_Curr = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_3);
//    ADCSampPara.BUS_Voltage = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_4);
//}

//void Deal_Data(void)
//{
//    Volt_CurrPara.PhaseU_Curr = ((int16_t)ADCSampPara.PhaseU_Curr - (int16_t)ADCSampPara.OffsetPhaseU) * CURR_SCALE;
//    Volt_CurrPara.PhaseV_Curr = ((int16_t)ADCSampPara.PhaseV_Curr - (int16_t)ADCSampPara.OffsetPhaseV) * CURR_SCALE;
//    Volt_CurrPara.PhaseW_Curr = ((int16_t)ADCSampPara.PhaseW_Curr - (int16_t)ADCSampPara.OffsetPhaseW) * CURR_SCALE;

//    Volt_CurrPara.BUS_Voltage = (float)ADCSampPara.BUS_Voltage * VBUS_SCALE;
//    Volt_CurrPara.Vdc = Volt_CurrPara.BUS_Voltage;

//    if (Volt_CurrPara.Vdc > 1.0f)
//        Volt_CurrPara.Svpwm_Km_BackwS = 1.0f / Volt_CurrPara.Vdc;
//    else
//        Volt_CurrPara.Svpwm_Km_BackwS = 1.0f;
//}
