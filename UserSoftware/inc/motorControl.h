/**
 * @file    motorControl.h
 * @author  nono <nono_1007@foxmail.com>
 * @brief   FOC 控制主状态机 API
 */
#ifndef __MOTORCONTROL_H_
#define __MOTORCONTROL_H_
#include <stdint.h>

typedef enum {
    MOTOR_MODE_STOP          = 0U, /* 停机：关闭 PWM，清除运行控制量 */
    MOTOR_MODE_SENSOR        = 1U, /* 有感闭环：预留编码器/霍尔位置反馈 */
    MOTOR_MODE_SENSORLESS    = 2U, /* 无感直接闭环：禁止自动 VF/I-F/HFI 辅助启动 */
    MOTOR_MODE_VF            = 3U, /* V/F 开环：仅按频率和电压驱动，不闭合观测器速度环 */
    MOTOR_MODE_PREPOS        = 4U, /* 预定位：定位后进入后续闭环流程，当前实现仍为预定位/VF试验链 */
    MOTOR_MODE_PARAM_ID      = 5U, /* 参数辨识：静止执行 Rs/Ld/Lq 辨识状态机 */
    MOTOR_MODE_IF_STARTUP    = 6U, /* I/F 启动：电流/频率斜坡启动，锁定后切无感闭环 */
    MOTOR_MODE_HFI_STARTUP   = 7U  /* HFI 启动：高频注入获取初始位置，随后切无感闭环 */
} MotorControlMode;

void PMSM_init(void);
void Foc_Control(void);
uint8_t Motor_ControlStart(uint8_t mode, float target_speed_rpm);
void Motor_ControlStop(void);
void PWM_Update_From_SVPWM(void);
void PWM_SetDuty(uint16_t ccr1, uint16_t ccr2, uint16_t ccr3);

#endif
