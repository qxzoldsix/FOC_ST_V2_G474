#ifndef _TASKMANAGER_H_
#define _TASKMANAGER_H_

#include "headline.h"

typedef  void ( *noTimer_Ptr) ( void );

typedef struct {
	    uint16_t	 Task_Period;  			
	    uint16_t 	 Task_Count;
	    noTimer_Ptr Task_Function;		
}TaskTime;

#define Task_Num   6

/* 1：电流环逐拍采样、每毫秒批量发送；0：每毫秒发送一组。 */
#define HyperSpeedDataProcessEn     1
/* 高速每批保存 1ms 的电流环样本；低速每次发送 1 组。 */
#define CURRENT_BATCH_SIZE (HyperSpeedDataProcessEn ? FOC_CONTROL_FREQ_HZ / 1000U : 1U)
#define LOW_SPEED_CHANNEL_CAPACITY 16U /* 低速上报最大数据通道数，不包含帧尾 */

void HyperSpeedData_Capture(float iu, float iv, float iw);

#define HFPeriod_COUNT      1    
#define FaulPeriod_COUNT    20
#define Balance_COUNT    20
#define KEY_COUNT    100    
#define DebugPeriod_COUNT    1000
#define CAN_APP_TASK_PERIOD_MS         10    // 10ms, PDO1@100Hz


void HFPeriod_RUN(void);
void Task_send_Rece(void);
void Balance_Control(void);
void KEY_RUN(void);
void Task_DEBUG(void);
void CAN_App_Task(void);
void Timer_Task_Count(void);
void Task_Manage_List_Init(void);
void Execute_Task_List_RUN(void);

#endif
