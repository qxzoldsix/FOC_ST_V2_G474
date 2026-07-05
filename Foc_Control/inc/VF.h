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



#define VF_VOLTAGE_MIN      0.01f    // VF 最低输出电压
#define VF_VOLTAGE_MAX      5.0f     // VF 最高输出电压（不超过额定电压）
#define VF_FREQ_MAX         400.0f   // VF 最高输出频率
#define VF_TS               5e-5     // 控制周期 50µs (20kHz)

#define VF_RAMP_GRAD_DEF    0.01f

#define VF_HzRamp_DEFAULTS  {0.0f, 0.0f, VF_RAMP_GRAD_DEF, 0, 0}


extern GXieLv VF_HzRamp;

void VF_Control_Run(Control_FB *pCtrl);
	
#ifdef __cplusplus
}
#endif

#endif
