/** @file IF.h @brief I/F 开环：给定电流幅值和电频率，转子位置不参与控制。 */
#ifndef IF_CONTROL_H
#define IF_CONTROL_H
#include "headline.h"

#define IF_CURRENT_MAX_A              1.5f   /* 电流给定上限，A */
#define IF_PHASE_CURRENT_LIMIT_A      2.0f   /* 相电流保护值，A */
#define IF_CURRENT_RAMP_A_PER_SECOND   5.0f   /* 电流变化速率，A/s */
#define IF_FREQ_RAMP_HZ_PER_SECOND    50.0f   /* 电频率变化速率，Hz/s */
#define IF_FREQ_MAX_HZ               100.0f  /* 电频率幅值上限，Hz */
#define IF_ALIGN_HOLD_MS             150U   /* 电流到位后定位时间，ms */

typedef enum {
    IF_ERROR_NONE = 0U,
    IF_ERROR_COMMAND,     /* 电流/频率给定不合法 */
    IF_ERROR_CURRENT      /* 实测相电流异常或ADC削顶 */
} IfControlError;

extern float IfCurrentTargetA;  /* 上电为0，由用户手动给定；有效范围(0,IF_CURRENT_MAX_A] */
extern IfControlError IfError; /* 停机后保留，下次进入I/F时清零 */
extern GXieLv IF_HzRamp;       /* X由motor.TargetHz写入；Y为输出电Hz；Grad为每拍步长 */

void IfControlStart(void);
void IfControlRun(void);
#endif
