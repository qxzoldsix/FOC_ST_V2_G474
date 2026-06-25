#ifndef __VF_H
#define __VF_H

#ifdef __cplusplus
extern "C" {
#endif
#include "headline.h"



#define VF_VOLTAGE_MIN      0.01f    
#define VF_VOLTAGE_MAX      5.0f  // VF 控制最大输出电压（额定电压）	
#define VF_FREQ_MAX         400.0f  // VF 控制最大运行频率
#define VF_TS               5e-5    // 控制周期时间（单位：秒）

#define VF_RAMP_GRAD_DEF    0.01f

#define VF_HzRamp_DEFAULTS  {0.0f, 0.0f, VF_RAMP_GRAD_DEF, 0, 0}


extern GXieLv VF_HzRamp;

void VF_Control_Run(Control_FB *pCtrl);
	
#ifdef __cplusplus
}
#endif

#endif
