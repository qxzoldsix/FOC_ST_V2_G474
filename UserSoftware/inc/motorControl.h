#ifndef __MOTORCONTROL_H_
#define __MOTORCONTROL_H_
#include "headline.h"

/* ========== VF → 无感 FOC 自动过渡参数 ========== */
#define VF_TO_SENSORLESS_HZ  10.0f  // 切无感阈值(Hz): 太低反电动势不够, 观测器无法收敛
void 	PMSM_init(void);
void Foc_Control(void);
void PWM_Update_From_SVPWM(void);
void PWM_SetDuty(uint16_t ccr1, uint16_t ccr2, uint16_t ccr3);
#endif
