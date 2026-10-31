/**
 * @file taskScheduler.c
 * @brief 调度层：1ms 中断只计时，任务函数在主循环执行。
 */
#include "headline.h"
#include "taskScheduler.h"
#include "debugSerial.h"

typedef struct {
    uint16_t period;
    volatile uint16_t count; /* ISR 写入、主循环读取。 */
    void (*run)(void);
} TaskTime;

/* 初始计数保留原任务相位；空的 Balance 任务不再占用调度项。 */
static TaskTime taskList[] = {
    {TELEMETRY_TASK_PERIOD_MS,     1U, TelemetryTaskRun},
    {BOARD_MONITOR_TASK_PERIOD_MS,8U, BoardMonitorTaskRun},
    {KEY_TASK_PERIOD_MS,         80U, KeyTaskRun},
    {DISPLAY_TASK_PERIOD_MS,    600U, DisplayTaskRun},
    {CAN_APP_TASK_PERIOD_MS,      5U, CanTaskRun},
    {DEBUG_STATUS_TASK_PERIOD_MS, 0U, DebugStatusTaskRun}
};

void TaskSchedulerInit(void)
{
    if (HAL_TIM_Base_Start_IT(&htim3) != HAL_OK) Error_Handler();
}

void TaskSchedulerTick(void)
{
    for (uint32_t index = 0U; index < sizeof(taskList) / sizeof(taskList[0]); index++) {
        if (taskList[index].period > 0U && taskList[index].count < taskList[index].period) {
            taskList[index].count++;
        }
    }
}

void TaskSchedulerRun(void)
{
    for (uint32_t index = 0U; index < sizeof(taskList) / sizeof(taskList[0]); index++) {
        if (taskList[index].period > 0U && taskList[index].count >= taskList[index].period) {
            taskList[index].run();
            taskList[index].count = 0U;
        }
    }
}
