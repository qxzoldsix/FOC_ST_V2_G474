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
    MOTOR_MODE_IF            = 4U, /* I/F 开环：电流闭环、频率/角度开环，持续运行，不自动切闭环 */
    MOTOR_MODE_PREPOS        = 5U, /* 预定位：保留入口，当前运行函数尚未实现 */
    MOTOR_MODE_PARAM_ID      = 6U, /* 参数辨识：静止测 R/L，随后空载拖转/封波测磁链 */
    MOTOR_MODE_IF_STARTUP    = 7U, /* I/F 启动：电流/频率斜坡启动，锁定后切无感闭环 */
    MOTOR_MODE_HFI_STARTUP   = 8U  /* HFI 启动：高频注入获取初始位置，随后切无感闭环 */
} MotorControlMode;

void MotorControlInit(void);
void FocControl(void);
uint8_t MotorControlStart(uint8_t mode, float target_speed_rpm);
void MotorControlStop(void);

#endif
