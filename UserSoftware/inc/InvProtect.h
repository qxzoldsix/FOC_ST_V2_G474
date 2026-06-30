#ifndef _INVPROTECT_H_
#define _INVPROTECT_H_

#include "headline.h"

/* ========== 保护阈值 ========== */
#define OC_THRESHOLD_A      10.0f   // 相过流阈值 (A)
#define OV_THRESHOLD_V      30.0f   // 母线过压阈值 (V)
#define UV_THRESHOLD_V      8.0f    // 母线欠压阈值 (V)

/* ========== 故障码 ========== */
#define FAULT_NONE          0       // 无故障
#define FAULT_OC_U          1       // U 相过流
#define FAULT_OC_V          2       // V 相过流
#define FAULT_OC_W          3       // W 相过流
#define FAULT_OV            4       // 母线过压
#define FAULT_UV            5       // 母线欠压

void InvProtect_Check(void);
void InvProtect_Clear(void);
void InvProtect_Enable(void);

#endif
