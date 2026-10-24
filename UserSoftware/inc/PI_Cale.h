/**
 * @file    PI_Cale.h
 * @author  nono <nono_1007@foxmail.com>
 * @brief   PI 控制器结构体 + 参数宏定义
 *
 * Kp: 比例系数（响应速度）
 * Ki: 积分系数（消除静差）
 * Umax/Umin: 输出限幅
 * up: 当前误差
 * ui: 积分项
 * v1: 预输出值
 * Out: 实际输出值
 */
#ifndef __PI_CALE_H_
#define __PI_CALE_H_

#include "headline.h"
typedef struct{
    float Ref;
    float Fbk;
    float Out;
    float OutF;       // 滤波后的输出
    float Kp;
    float Ki;
    float Umax;       // 输出上限
    float Umin;
    float  up;        // 当前误差 Ref - Fbk
    float  ui;        // 本拍积分输出
    float  v1;        // 限幅前的 PI 输出
    float  i1;        // 上一拍积分输出
}PI_controller, *p_PI_Control;
/* 
Kp：比例系数（响应速度）
Ki：积分系数（消除静差）
Umax/Umin：输出上下限
up：当前误差
ui：本拍积分输出
v1：限幅前的 PI 输出
Out：限幅后的输出
*/
#define PI_Control_DEFAULTS {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0}
void PI_Init(void);
extern  PI_controller   pi_spd ;
extern  PI_controller   pi_id ;
extern  PI_controller   pi_iq ;
void PID_controller(p_PI_Control  pV);
void PI_ResetRuntime(p_PI_Control pV);
#endif
