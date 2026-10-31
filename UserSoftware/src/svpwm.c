/**
 * @file    svpwm.c
 * @author  nono <nono_1007@foxmail.com>
 * @brief   SVPWM 七段式调制: 扇区判断 + 矢量时间计算 + CCR 输出
 */
#include "headline.h"

static uint16_t svpwm_to_ccr(float t)
{
    int32_t ccr = (int32_t)(t * Volt_CurrPara.Svpwm_Km_BackwS * PWM_HalfPerMax) + PWM_HalfPerMax;

    if (ccr < 0) {
        ccr = 0;
    } else if (ccr > TIM1_ARR) {
        ccr = TIM1_ARR;
    }

    return (uint16_t)ccr;
}
float PWM_GetVoltageLimitPu(void)
{
    float vbus = Volt_CurrPara.BUS_Voltage;
    return (isfinite(vbus) && vbus > 1.0f) ?
           CONTROL_LINEAR_MODULATION_MARGIN * vbus / CONTROL_NOMINAL_BUS_VOLTAGE_V : 0.0f;
}

void PWM_ReconstructVoltage(float *u_alpha_v, float *u_beta_v)
{
    float vbus = Volt_CurrPara.BUS_Voltage;
    float duty_u = (float)Svpwm_dq.SVPTa / (float)(TIM1_ARR + 1U);
    float duty_v = (float)Svpwm_dq.SVPTb / (float)(TIM1_ARR + 1U);
    float duty_w = (float)Svpwm_dq.SVPTc / (float)(TIM1_ARR + 1U);

    if (!isfinite(vbus) || vbus <= 1.0f) vbus = 0.0f;
    *u_alpha_v = TWO_OVER_3 * vbus * (duty_u - 0.5f * duty_v - 0.5f * duty_w);
    *u_beta_v = INV_SQRT3 * vbus * (duty_v - duty_w);
}

void svpwm_Cale(p_SVPWM pV){
    float voltage_limit = PWM_GetVoltageLimitPu();

    if (voltage_limit <= 0.0f ||
        !isfinite(pV->Ualpha) || !isfinite(pV->Ubeta)) {
        pV->Ualpha = 0.0f;
        pV->Ubeta = 0.0f;
    } else {
        float voltage_mag = hypotf(pV->Ualpha, pV->Ubeta);
        if (voltage_mag > voltage_limit) {
            float scale = voltage_limit / voltage_mag;
            pV->Ualpha *= scale;
            pV->Ubeta *= scale;
        }
    }
    pV->tmp1= pV->Ubeta;   //Vref1
    pV->tmp2= pV->Ubeta*0.5f + pV->Ualpha*SQRT3_OVER_2 ;//vref2
    pV->tmp3= pV->tmp2 - pV->tmp1;   // Vref3 = (√3/2)Ualpha - 0.5*Ubeta
    pV->VecSector=3;                  // 基准值
    pV->VecSector=(pV->tmp2>0)?(pV->VecSector-1):(pV->VecSector);
    pV->VecSector=(pV->tmp3>0)?(pV->VecSector-1):(pV->VecSector);
    pV->VecSector=(pV->tmp1<0)?(7-pV->VecSector):(pV->VecSector);
    if(pV->VecSector==1 || pV->VecSector==4)   // 基本矢量扇区，计算矢量占空比 Tabc
    {
        pV->Ta= pV->tmp2;
        pV->Tb= pV->tmp1-pV->tmp3;
        pV->Tc=-pV->tmp2;
    }
    else if(pV->VecSector==2 || pV->VecSector==5)
    {
        pV->Ta= pV->tmp3+pV->tmp2;
        pV->Tb= pV->tmp1;
        pV->Tc=-pV->tmp1;
    }
    else if(pV->VecSector==3 || pV->VecSector==6)
    {
        pV->Ta= pV->tmp3;
        pV->Tb=-pV->tmp3;
        pV->Tc= -(pV->tmp1+pV->tmp2);
    }
    else     // 异常状态，输出零矢量 SVPWM(000)
    {
        pV->Ta=0;
        pV->Tb=0;
        pV->Tc=0;
    }
    pV->SVPTa = svpwm_to_ccr(pV->Ta);
    pV->SVPTb = svpwm_to_ccr(pV->Tb);
    pV->SVPTc = svpwm_to_ccr(pV->Tc);
}
