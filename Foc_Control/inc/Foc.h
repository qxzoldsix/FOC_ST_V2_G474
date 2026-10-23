/**
 * @file    Foc.h
 * @author  nono <nono_1007@foxmail.com>
 * @brief   FOC 数据结构定义: Control_FB, CLARKE, PARK, IPARK + 变换函数声明
 */
#ifndef __FOC_H_
#define __FOC_H_
#include "headline.h"
#define  Control_FB_DEFAULTS  {0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0,0,0}
#define  CLARKE_DEFAULTS {0,0,0,0,0} 
#define  PARK_DEFAULTS {0,0,0,0,0,0,0}
#define  IPARK_DEFAULTS {0,0,0,0,0,0,0}  

#define CURRENT_USE_W_AS_V  0
#define FOC_VOLTAGE_PU_MAX  0.95f

typedef struct 
{	  
	float V_d;                 // d 轴电压输出 (pu)
	float V_q;                 // q 轴电压输出 (pu)
	float V_amp;               // VF 电压幅值 (pu)
	float  SpeedRPM;			// 机械转速 (RPM)
	float CurrentHz;			// 当前电频率 (Hz)
	float OpenTheta;			// VF 开环角度 (rad)
	float TargetHz;				// 目标电频率 (Hz)
	float TargetVolt;			// VF 目标电压 (pu)
	uint16_t   Fault_DTC;      // 故障码（位掩码）
	int16_t IQAngle;            // 电角度 (0~4095)
	uint8_t Control_Mode;		// 0停机 1有感 2无感直闭环 3VF 4预定位 5辨识 6I/F启动 7HFI启动
}Control_FB;

typedef struct {
            float  Us;        //  相电流 A
            float  Vs;          //  相电流 B
            float  Ws;          //  相电流 C
            float  Alpha;       //  两相静止坐标系 Alpha 轴
            float  Beta;        //  两相静止坐标系 Beta 轴
} CLARKE ,*p_CLARKE ;

typedef struct {
            float  Alpha;     //  两相静止坐标系 Alpha 轴
            float  Beta;        //  两相静止坐标系 Beta 轴
            uint16_t   Angle;       //  转子磁极位置角度 0~4095 (0~360°)
            float  Ds;          //  两相旋转坐标系 d 轴分量
            float  Qs;          //  两相旋转坐标系 q 轴分量
            float  Sine;    //  正弦值 -1~1
            float  Cosine;  //  余弦值 -1~1
} PARK , *p_PARK ;

typedef struct {
            float  Alpha;         // 两相静止坐标系 Alpha 轴
            float  Beta;          // 两相静止坐标系 Beta 轴
            float  Angle;         // 转子磁极位置角度 0~4095
            float  Ds;            // 两相旋转坐标系 d 轴分量
            float  Qs;            // 两相旋转坐标系 q 轴分量
            float  Sine;          // 正弦值 -1~1
            float  Cosine;        // 余弦值 -1~1
}IPARK , *p_IPARK;


extern  Control_FB motor;
extern  CLARKE CLARKE_PCurr;
extern 	PARK PARK_PCurr;
extern  GXieLv I_q_GXieLv;
extern  GXieLv I_d_GXieLv;
extern  GXieLv SpeedRpm_GXieLv;
extern IPARK IPARK_PVdq;
extern Ang_SinCos   Park_SinCos;
void  CLARKE_Cale(p_CLARKE  pV); // 三相到两相变换 Clarke 变换
void  PARK_Cale(p_PARK  pV) ;   // 两相静止到旋转 Park 变换
void  IPARK_Cale(p_IPARK  pV) ;   // 两相旋转到静止 反 Park 变换
void UVW_Axis_DQ(void);
//void UVW_Axis_DQ_HALL(void);
//void UVW_Axis_DQ_magnetic(void);
void Speed_FOC(void);
void Idq_FOC(void);
 void FOC_Svpwm_dq(void);
//void Init_Pos_Detect(void);
#endif
