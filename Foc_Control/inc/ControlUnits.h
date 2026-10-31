#ifndef CONTROL_UNITS_H
#define CONTROL_UNITS_H
#include "Common_Math.h"

/*
 * 电压标幺统一以固定的相电压峰值为基准：
 * 1 pu = 12 V / sqrt(3) = 6.928203 V。
 * 基准不随实测母线变化；母线补偿只在 PWM 层执行一次。
 * 当前控制运算为 float。未来电压定点格式统一采用 int32_t Q7.24，
 * raw = round(pu * 2^24)，范围 [-128, 128)，禁止把 float 当作 Q24 原码。
 */
#define CONTROL_NOMINAL_BUS_VOLTAGE_V  12.0f
#define CONTROL_VOLTAGE_BASE_V (CONTROL_NOMINAL_BUS_VOLTAGE_V * INV_SQRT3)
#define CONTROL_LINEAR_MODULATION_MARGIN  0.95f

#endif
