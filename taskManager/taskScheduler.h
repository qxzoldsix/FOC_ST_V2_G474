/** @file taskScheduler.h @brief 裸机时间片调度接口。 */
#ifndef TASK_SCHEDULER_H
#define TASK_SCHEDULER_H
void TaskSchedulerInit(void);
void TaskSchedulerTick(void);
void TaskSchedulerRun(void);
#endif
