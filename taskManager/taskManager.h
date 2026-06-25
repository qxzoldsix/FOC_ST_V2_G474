#ifndef _TASKMANAGER_H_
#define _TASKMANAGER_H_

#include "headline.h"

typedef  void ( *noTimer_Ptr) ( void );

typedef struct {
	    uint16_t	 Task_Period;  			
	    uint16_t 	 Task_Count;
	    noTimer_Ptr Task_Function;		
}TaskTime;

#define Task_Num   5

#define HFPeriod_COUNT      1    
#define FaulPeriod_COUNT    20
#define Balance_COUNT    20
#define KEY_COUNT    100    
#define DebugPeriod_COUNT    1000


void HFPeriod_RUN(void);
void Task_send_Rece(void);
void Balance_Control(void);
void KEY_RUN(void);
void Task_DEBUG(void);
void Timer_Task_Count(void);
void Task_Manage_List_Init(void);
void Execute_Task_List_RUN(void);

#endif
