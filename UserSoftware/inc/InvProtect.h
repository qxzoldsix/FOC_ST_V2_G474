#ifndef _INVPROTECT_H_
#define _INVPROTECT_H_

#include "headline.h"

/* ========== 保护阈值 ========== */
#define OC_THRESHOLD_A      20.0f   // 相过流阈值 (A)
#define OC_BUS_THRESHOLD_A  15.0f    // 母线电流过流阈值 (A)
#define OV_THRESHOLD_V      30.0f   // 母线过压阈值 (V)
#define UV_THRESHOLD_V      8.0f    // 母线欠压阈值 (V)

/* ========== 故障码（位掩码，可同时记录多个故障）========== */
#define FAULT_NONE          0x0000

/* --- 可清除故障 (bit 0~7) --- */
#define FAULT_OC_U          0x0001  // U 相过流
#define FAULT_OC_V          0x0002  // V 相过流
#define FAULT_OC_W          0x0004  // W 相过流
#define FAULT_OV            0x0008  // 母线过压
#define FAULT_UV            0x0010  // 母线欠压
#define FAULT_OC_BUS        0x0020  // 母线电流过流
#define FAULT_OT            0x0040  // 过温 (NTC)
#define FAULT_HALL          0x0080  // 霍尔/编码器异常

/* --- 锁存故障 (bit 8~15, 需重新上电才能清除) --- */
#define FAULT_LOCK          0x0100  // 堵转
#define FAULT_PHASE_LOSS    0x0200  // 缺相
#define FAULT_START_FAIL    0x0400  // 启动失败
#define FAULT_REPEAT_OC     0x0800  // 反复过流锁存
#define FAULT_HARD_FAULT    0x1000  // 硬件级故障

/* --- 掩码 --- */
#define FAULT_CLEARABLE_MASK 0x00FF // 可软件清除
#define FAULT_LATCH_MASK     0xFF00 // 需重新上电

/* 本次上电累计清除超过此次数→锁存，需重新上电 */
#define FAULT_CLEAR_MAX         5

void InvProtect_Check(void);
void InvProtect_Clear(void);
void InvProtect_Enable(void);
uint16_t InvProtect_GetFault(void);

#endif
