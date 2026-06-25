#ifndef __MOTORCONTROL_H_
#define __MOTORCONTROL_H_
#include "headline.h"
void 	PMSM_init(void);
void Foc_Control(void);
void PWM_Update_From_SVPWM(void);
void PWM_SetDuty(uint16_t ccr1, uint16_t ccr2, uint16_t ccr3);
#endif
