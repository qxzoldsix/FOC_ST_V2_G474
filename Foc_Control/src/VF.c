/**
 * @file    VF.c
 * @author  nono <nono_1007@foxmail.com>
 * @brief   V/F 开环压频控制: 频率斜坡 + 电压斜坡 + 角度积分 + SVPWM 输出
 */
#include "headline.h"


GXieLv VF_HzRamp = VF_HzRamp_DEFAULTS;
static Ang_SinCos ParkSinCos = Ang_SinCos_DEFAULTS;
SVPWM Svpwm_dq = SVPWM_DEFAULTS;

void VF_Control_Run(Control_FB *pCtrl)
{
		IPARK VF_IPARK;
    float target_pu = pCtrl->TargetVolt;
    float voltage_limit = PWM_GetVoltageLimitPu();
    if (!isfinite(target_pu) || target_pu < 0.0f) target_pu = 0.0f;
    if (target_pu > voltage_limit) target_pu = voltage_limit;
		VF_HzRamp.XieLv_X = pCtrl->TargetHz;
		Grad_XieLv(&VF_HzRamp);
		pCtrl->CurrentHz = VF_HzRamp.XieLv_Y;	

    if (pCtrl->CurrentHz <= 0.0f || pCtrl->TargetHz <= 0.0f || target_pu <= VF_VOLTAGE_MIN) {
        pCtrl->V_amp = 0.0f;
    }
    else {
        pCtrl->V_amp = VF_VOLTAGE_MIN + (target_pu - VF_VOLTAGE_MIN) * (pCtrl->CurrentHz / pCtrl->TargetHz);
        if (pCtrl->V_amp > target_pu) {
            pCtrl->V_amp = target_pu;
        }
    }
		pCtrl->OpenTheta += TWO_PI * pCtrl->CurrentHz * VF_TS;
		while (pCtrl->OpenTheta >= TWO_PI) pCtrl->OpenTheta -= TWO_PI;
		while (pCtrl->OpenTheta < 0.0f)        pCtrl->OpenTheta += TWO_PI;
		ParkSinCos.table_Angle = (uint16_t)(pCtrl->OpenTheta * RAD_TO_ANGLE);
    SinCos_Table(&ParkSinCos);
		VF_IPARK.Ds    = 0.0f;
    VF_IPARK.Qs    = pCtrl->V_amp;// F:motor.TargetHz
		VF_IPARK.Angle =ParkSinCos.table_Angle;
    VF_IPARK.Sine   = ParkSinCos.table_Sin;
    VF_IPARK.Cosine = ParkSinCos.table_Cos;
		IPARK_Cale(&VF_IPARK);
		Svpwm_dq.Ualpha=VF_IPARK.Alpha;
	
		Svpwm_dq.Ubeta=VF_IPARK.Beta;
		svpwm_Cale(&Svpwm_dq);
		PwmSetDuty(Svpwm_dq.SVPTa, Svpwm_dq.SVPTb, Svpwm_dq.SVPTc);
}
