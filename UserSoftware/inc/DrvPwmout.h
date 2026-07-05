/**
 * @file    DrvPwmout.h
 * @author  nono <nono_1007@foxmail.com>
 * @brief   PWM 输出启停控制 API
 */
#ifndef __DRVPWMOUT_H_
#define __DRVPWMOUT_H_

#include "headline.h"

#ifdef __cplusplus
extern "C" {
#endif

void Foc_Pwm_Start(void);
void Foc_Pwm_Stop(void);

#ifdef __cplusplus
}
#endif

#endif
