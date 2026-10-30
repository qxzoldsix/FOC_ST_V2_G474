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

uint8_t PwmInit(void);
uint8_t PwmStart(void);
void PwmStop(void);
uint8_t PwmIsEnabled(void);
void PwmSetDuty(uint16_t ccr1, uint16_t ccr2, uint16_t ccr3);

#ifdef __cplusplus
}
#endif

#endif
