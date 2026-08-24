/**
 * @file    ParamId.h
 * @brief   静止电机参数辨识：Rs / Ld / Lq 候选值
 */
#ifndef __PARAMID_H_
#define __PARAMID_H_

#include "headline.h"

typedef enum {
    ID_STEP_IDLE = 0U,        /* 未启动 */
    ID_STEP_ALIGN,            /* 固定电角度，建立d轴电流并锁定转子 */
    ID_STEP_RS_LOW_SETTLE,    /* Rs低电压平台等待RL暂态衰减 */
    ID_STEP_RS_LOW_SAMPLE,    /* Rs低平台多点平均 */
    ID_STEP_RS_HIGH_SETTLE,   /* Rs高电压平台等待RL暂态衰减 */
    ID_STEP_RS_HIGH_SAMPLE,   /* Rs高平台多点平均并计算ΔV/ΔI */
    ID_STEP_LD_PULSE,         /* 带正d轴偏置的Ld交替脉冲 */
    ID_STEP_LQ_PULSE,         /* 保持d轴定位的Lq正负脉冲 */
    ID_STEP_VALIDATE,         /* 检查数值范围和Ld/Lq比例 */
    ID_STEP_DONE,             /* 候选值有效，已经自动封波 */
    ID_STEP_FAILED            /* 辨识失败，已经自动封波 */
} ParamIdStep;

typedef enum {
    ID_ERROR_NONE = 0U,
    ID_ERROR_BUS_VOLTAGE,
    ID_ERROR_OVERCURRENT,
    ID_ERROR_RS_RESPONSE,
    ID_ERROR_RS_RANGE,
    ID_ERROR_LD_RESPONSE,
    ID_ERROR_LQ_RESPONSE,
    ID_ERROR_L_RANGE,
    ID_ERROR_INVALID_STATE
} ParamIdError;

typedef struct {
    uint8_t  step;          /* ParamIdStep，可通过SDO 0x1010读取 */
    uint8_t  error;         /* ParamIdError，0表示没有错误 */
    uint8_t  done;          /* 流程结束标志，成功和失败都会置1 */
    uint8_t  valid;         /* 只有该位为1时Rs/Ld/Lq才是有效候选值 */
    uint32_t tick;          /* 当前步骤的20kHz周期计数 */
    uint16_t valid_samples; /* 最近一个电感轴通过校验的脉冲数量 */
    float    Rs;            /* 候选定子电阻，单位Ω */
    float    Ld;            /* 候选d轴电感，单位H，不是mH */
    float    Lq;            /* 候选q轴电感，单位H，不是mH */
    float    current_a;     /* 调试观测：当前d轴电流 */
    float    voltage_v;     /* 调试观测：由CCR反算的d轴平均电压 */
} ParamId_t;

extern ParamId_t pid;

void ParamId_Start(void);
void ParamId_Run(void);

#endif
