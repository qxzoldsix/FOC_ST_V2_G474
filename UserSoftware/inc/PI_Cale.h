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
    float OutF;//�˲������
    float Kp;
    float Ki;
    float Umax;//���ֱ�������
    float Umin;
    float  up;        // PI���Ƶı������������
    float  ui;        // PI���ƵĻ������������
    float  v1;        // PI���Ƶ���ʷ��������
    float  i1;        // PI���Ƶ���ʷ�������������
}PI_controller, *p_PI_Control;
/* 
Kp������ϵ������Ӧ�ٶȣ�
Ki������ϵ�����������
Umax/Umin������޷�
up����ǰ���
ui��������
v1���������ֵ
Out��ʵ�����ֵ
*/
#define PI_Control_DEFAULTS {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0}
void PI_Init(void);
extern  PI_controller   pi_spd ;
extern  PI_controller   pi_id ;
extern  PI_controller   pi_iq ;
void PID_controller(p_PI_Control  pV);
#endif
