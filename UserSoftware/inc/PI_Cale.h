#ifndef __PI_CALE_H_
#define __PI_CALE_H_

#include "headline.h"
typedef struct{
    float Ref;
    float Fbk;
    float Out;
    float OutF;//滤波后输出
    float Kp;
    float Ki;
    float Umax;//积分饱和上限
    float Umin;
    float  up;        // PI控制的比例项输出参数
    float  ui;        // PI控制的积分项输出参数
    float  v1;        // PI控制的历史输出项参数
    float  i1;        // PI控制的历史积分项输出参数
}PI_controller, *p_PI_Control;
/* 
Kp：比例系数（响应速度）
Ki：积分系数（消除静差）
Umax/Umin：输出限幅
up：当前误差
ui：积分项
v1：理论输出值
Out：实际输出值
*/
#define PI_Control_DEFAULTS {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0}
void PI_Init(void);
extern  PI_controller   pi_spd ;
extern  PI_controller   pi_id ;
extern  PI_controller   pi_iq ;
void PID_controller(p_PI_Control  pV);
#endif
