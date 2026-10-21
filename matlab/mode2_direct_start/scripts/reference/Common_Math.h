/**
 * @file    Common_Math.h
 * @author  nono <nono_1007@foxmail.com>
 * @brief   通用数学: 正弦查表、斜坡发生器、限幅函数
 */
#ifndef __COMMON_MATH_H_
#define __COMMON_MATH_H_
#include "headline.h"
#define U_0       0x0c00     //  0~4095 电角度控制分段查询 90°
#define U0_90     0x0000
#define U90_180   0x0400
#define U180_270  0x0800
#define U270_360  0x0c00
#define PI  3.14159265358979323846f
#define GM_Low_Lass_A  0.9f
#define GM_Low_Lass_B  0.1f
typedef struct
{
	uint16_t  table_Angle;
	float table_Sin;
	float table_Cos;
}Ang_SinCos, *p_Ang_SinCos;
#define  Ang_SinCos_DEFAULTS    {0,0.0,0.0}

typedef struct
{  //ָ�������б�ʴ���
  	float  XieLv_X;   // ָ�����б���������x
	float  XieLv_Y;
	float  XieLv_Grad;
	uint32_t    Timer_Count;
	uint32_t    Grad_Timer;
}GXieLv, *p_GXieLv;
 #define  GXieLv_DEFAULTS    {0.0,0.0,0.0,0,0}

void SinCos_Table(p_Ang_SinCos PV);
void Grad_XieLv(p_GXieLv pV);
float  Limit_Sat( float Uint,float U_max, float U_min);

#endif
