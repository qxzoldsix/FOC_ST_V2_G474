/**
 * @file    ParamId.h
 * @brief   电机参数辨识: Rs / Ld / Lq / Flux (顺序状态机, 无 for 循环)
 */
#ifndef __PARAMID_H_
#define __PARAMID_H_

#include "headline.h"

/* ---- 辨识流程 ---- */
#define ID_STEP_NONE      0   // 空闲
#define ID_STEP_ALIGN     1   // 预对齐到零位
#define ID_STEP_RS_VOLT   2   // 注入直流电压
#define ID_STEP_RS_WAIT   3   // 等待电流稳定
#define ID_STEP_RS_SAMP   4   // 采样并计算 Rs
#define ID_STEP_LD_PULSE  5   // d轴脉冲注入
#define ID_STEP_LQ_PULSE  6   // q轴脉冲注入
#define ID_STEP_LS_CALC   7   // 计算 Ld/Lq
#define ID_STEP_DONE       99  // 辨识完成

/* ---- 辨识电压 (标幺值, 1.0 = 满载母线) ---- */
#define ID_ALIGN_VOLT   0.015f   // 对齐 ≈ 0.36V @24V
#define ID_RS_VOLT      0.035f   // Rs 辨识 ≈ 0.84V @24V
#define ID_LS_VOLT      0.085f   // 电感脉冲 ≈ 2.0V @24V

typedef struct {
    uint8_t  step;          // 当前辨识步骤
    uint32_t tick;          // 步骤内计时 (ms)
    float    vd_inject;     // 注入电压
    float    i_d;           // d轴电流采样
    float    i_d0;          // 注入前 d轴电流
    float    di_dt;         // 电流变化率
    float    Rs;            // 辨识结果: 定子电阻
    float    Ld;            // 辨识结果: d轴电感
    float    Lq;            // 辨识结果: q轴电感
    uint8_t  done;          // 辨识完成标志
} ParamId_t;

extern ParamId_t pid;

void ParamId_Start(void);
void ParamId_Run(void);

#endif
