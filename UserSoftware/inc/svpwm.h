/**
 * @file    svpwm.h
 * @author  nono <nono_1007@foxmail.com>
 * @brief   SVPWM 结构体定义 + 七段式调制
 */
#ifndef __SVPWM_H_
#define __SVPWM_H_

#include "main.h"
#include "ADC_SAMPLE.h"

#define TIM1_ARR          3999
#define PWM_HalfPerMax    ((TIM1_ARR + 1) / 2)  // 2000

typedef struct 	{ 
        float  Ualpha; 	// alpha 电压 float pu，基准 CONTROL_VOLTAGE_BASE_V
        float  Ubeta;	// beta 电压 float pu，与 VF/电流环同基准
        float  Ta;	
        float  Tb;		
        float  Tc;		
        uint16_t  SVPTa;		
        uint16_t  SVPTb;		
        uint16_t  SVPTc;		
        float  tmp1;	
        float  tmp2;	
        float  tmp3;	
        uint16_t VecSector;	 
} SVPWM , *p_SVPWM ;


#define SVPWM_DEFAULTS  {0.0,0.0,0.0,0.0,0.0,0,0,0,0.0,0.0,0.0,0}  // 默认初始化值

extern SVPWM  Svpwm_dq;
void svpwm_Cale(p_SVPWM pV);
float PWM_GetVoltageLimitPu(void); // 实测母线允许的电压矢量幅值 (pu)
void PWM_ReconstructVoltage(float *u_alpha_v, float *u_beta_v); // 最终 CCR 的理想桥电压 (V)
#endif
