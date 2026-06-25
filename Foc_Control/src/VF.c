#include "headline.h"


GXieLv VF_HzRamp = VF_HzRamp_DEFAULTS;
static Ang_SinCos ParkSinCos = Ang_SinCos_DEFAULTS;
SVPWM Svpwm_dq = SVPWM_DEFAULTS;

void VF_Control_Run(Control_FB *pCtrl)
{
		IPARK VF_IPARK;
		VF_HzRamp.XieLv_X = pCtrl->TargetHz;
		Grad_XieLv(&VF_HzRamp);
		pCtrl->CurrentHz = VF_HzRamp.XieLv_Y;	

    if (pCtrl->CurrentHz <= 0.0f || pCtrl->TargetHz <= 0.0f || pCtrl->TargetVolt <= VF_VOLTAGE_MIN) {
        pCtrl->V_amp = 0.0f;
    }
    else {
        pCtrl->V_amp = VF_VOLTAGE_MIN + (pCtrl->TargetVolt - VF_VOLTAGE_MIN) * (pCtrl->CurrentHz / pCtrl->TargetHz);
        if (pCtrl->V_amp > pCtrl->TargetVolt) {
            pCtrl->V_amp = pCtrl->TargetVolt;
        }
    }
		pCtrl->OpenTheta += 2.0f * PI * pCtrl->CurrentHz * VF_TS;
		while (pCtrl->OpenTheta >= 2.0f * PI) pCtrl->OpenTheta -= 2.0f * PI;
		while (pCtrl->OpenTheta < 0.0f)        pCtrl->OpenTheta += 2.0f * PI;
		ParkSinCos.table_Angle = (uint16_t)(pCtrl->OpenTheta * (4096.0f / (2.0f * PI)));
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
		PWM_Update_From_SVPWM();
}