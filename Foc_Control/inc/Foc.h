#ifndef __FOC_H_
#define __FOC_H_
#include "headline.h"
#define  Control_FB_DEFAULTS  {0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0,0,0,0}
#define  CLARKE_DEFAULTS {0,0,0,0,0} 
#define  PARK_DEFAULTS {0,0,0,0,0,0,0}
#define  IPARK_DEFAULTS {0,0,0,0,0,0,0}  

#define CURRENT_USE_W_AS_V  0

typedef struct 
{	  
	float  V_d;                //Vd电流环输入
	float  V_q;                //Vq电流环输入
	float V_amp;
//	float  I_d_fb;             //Idq控制
//	float  I_q_fb;             //Idq控制
	float  I_d;                //d轴电流环输出
	float  I_q;                //q轴电流环输出
	float  CANV_q;
	float  Posref;					//位置参考
	float  Torque_Fbk;			
	float  Speed_Start;     
	float  UI_Watt_Temp;
	float  Step_Count;
	float  Motor_Speed;
	float  SpeedRPM;			
  float CurrentHz;					//电频率
	float OpenTheta;					//VF
	float TargetHz;
	float TargetVolt;
	uint8_t    Fault_DTC;      //oc1、2过流、过压、欠压
	int16_t IQAngle;
//	int16_t IQAngle_JZ;
	int8_t  Move_State;        //电机旋转状态
	uint8_t Control_Mode;			//0：停机 1：有感 2：svc 3:VF
}Control_FB;

typedef struct {
            float  Us;        //  三相电流A
            float  Vs;          //  三相电流B
            float  Ws;          //  三相电流C
            float  Alpha;       //  二相静止坐标系 Alpha 轴
            float  Beta;        //  二相静止坐标系 Beta 轴
} CLARKE ,*p_CLARKE ;

typedef struct {
            float  Alpha;     //  二相静止坐标系 Alpha 轴
            float  Beta;        //  二相静止坐标系 Beta 轴
            uint16_t   Angle;       //  电机磁极位置角度0---65536即是0---360度
            float  Ds;          //  电机二相旋转坐标系下的d轴电流
            float  Qs;          //  电机二相旋转坐标系下的q轴电流
            float  Sine;    //  正弦参数，-32768---32767  -1到1
            float  Cosine;  //  余弦参数，-32768---32767  -1到1
} PARK , *p_PARK ;

typedef struct {
            float  Alpha;         // 二相静止坐标系 Alpha 轴
            float  Beta;          // 二相静止坐标系 Beta 轴
            float  Angle;         // 电机磁极位置角度0---65536即是0---360度
            float  Ds;            //  电机二相旋转坐标系下的d轴电流
            float  Qs;            //  电机二相旋转坐标系下的q轴电流
            float  Sine;          //  正弦参数，-32768---32767  -1到1
            float  Cosine;        //  余弦参数，-32768---32767  -1到1
}IPARK , *p_IPARK;


extern  Control_FB motor;
extern  CLARKE CLARKE_PCurr;
extern 	PARK PARK_PCurr;
extern  GXieLv I_q_GXieLv;
extern  GXieLv I_d_GXieLv;
extern  GXieLv SpeedRpm_GXieLv;
extern IPARK IPARK_PVdq;
extern Ang_SinCos   Park_SinCos;
void  CLARKE_Cale(p_CLARKE  pV); // 三相到二相变换 克拉克变换
void  PARK_Cale(p_PARK  pV) ;   // 二相到二相变换 park变换
void  IPARK_Cale(p_IPARK  pV) ;   // 二相到二相变换 IPARK变换
void UVW_Axis_DQ(void);
//void UVW_Axis_DQ_HALL(void);
//void UVW_Axis_DQ_magnetic(void);
void Speed_FOC(void);
void Idq_FOC(void);
 void FOC_Svpwm_dq(void);
//void Init_Pos_Detect(void);
#endif