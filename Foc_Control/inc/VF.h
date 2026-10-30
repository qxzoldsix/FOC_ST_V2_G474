/**
 * @file    VF.h
 * @author  nono <nono_1007@foxmail.com>
 * @brief   V/F 开环压频控制: 参数宏定义 + 斜坡发生器 + 控制函数声明
 */
#ifndef __VF_H
#define __VF_H

#ifdef __cplusplus
extern "C" {
#endif
#include "headline.h"



#define VF_VOLTAGE_MIN      0.01f    // float pu，固定基准见 ControlUnits.h；定点规范 Q7.24
#define VF_FREQ_MAX         400.0f   // VF 最高输出频率
#define VF_TS               FOC_CONTROL_TS_S  // 与电流环使用同一控制周期

#define VF_RAMP_GRAD_DEF    0.01f

#define VF_HzRamp_DEFAULTS  {0.0f, 0.0f, VF_RAMP_GRAD_DEF, 0, 0}


extern GXieLv VF_HzRamp;

void VF_Control_Run(Control_FB *pCtrl);
	
#ifdef __cplusplus
}
#endif

#endif
