/** @file taskManager.h @brief 应用任务与数据上报配置。 */
#ifndef TASK_MANAGER_H
#define TASK_MANAGER_H
#include "headline.h"

/* 1：电流环逐拍采样、每毫秒批量发送；0：每毫秒发送一组。 */
#define HyperSpeedDataProcessEn 1
#define CURRENT_BATCH_SIZE (HyperSpeedDataProcessEn ? FOC_CONTROL_FREQ_HZ / 1000U : 1U)
#define LOW_SPEED_CHANNEL_CAPACITY 16U

#define TELEMETRY_TASK_PERIOD_MS       1U
#define BOARD_MONITOR_TASK_PERIOD_MS  20U
#define KEY_TASK_PERIOD_MS           100U
#define DISPLAY_TASK_PERIOD_MS      1000U
#define CAN_APP_TASK_PERIOD_MS        10U
#define DEBUG_STATUS_TASK_PERIOD_MS  500U /* USART3 可读状态日志 */

void HyperSpeedDataCapture(float iu, float iv, float iw);
void TelemetryTaskRun(void);
void BoardMonitorTaskRun(void);
void KeyTaskRun(void);
void DisplayTaskRun(void);
void CanTaskRun(void);
#endif
