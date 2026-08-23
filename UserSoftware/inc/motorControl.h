/**
 * @file    motorControl.h
 * @author  nono <nono_1007@foxmail.com>
 * @brief   FOC 控制主状态机 API
 */
#ifndef __MOTORCONTROL_H_
#define __MOTORCONTROL_H_
#include <stdint.h>

typedef enum {
    MOTOR_MODE_STOP       = 0U,
    MOTOR_MODE_SENSOR     = 1U,
    MOTOR_MODE_SENSORLESS = 2U,
    MOTOR_MODE_VF         = 3U,
    MOTOR_MODE_PREPOS     = 4U,
    MOTOR_MODE_PARAM_ID   = 5U
} MotorControlMode;

void PMSM_init(void);
void Foc_Control(void);
uint8_t Motor_ControlStart(uint8_t mode, float target_speed_rpm);
void Motor_ControlStop(void);
void PWM_Update_From_SVPWM(void);
void PWM_SetDuty(uint16_t ccr1, uint16_t ccr2, uint16_t ccr3);

#endif
