/**
 * @file    DrvPwmout.h
 * @author  nono <nono_1007@foxmail.com>
 * @brief   PWM 输出启停控制 API
 */
#ifndef __DRVPWMOUT_H_
#define __DRVPWMOUT_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

uint8_t Foc_Pwm_Init(void);
uint8_t Foc_Pwm_Start(void);
void Foc_Pwm_Stop(void);
uint8_t Foc_Pwm_IsEnabled(void);

#ifdef __cplusplus
}
#endif

#endif
