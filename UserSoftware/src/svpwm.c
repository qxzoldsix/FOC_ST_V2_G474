#include "headline.h"

static uint16_t svpwm_to_ccr(float t)
{
    int32_t ccr = (int32_t)(t * Volt_CurrPara.Svpwm_Km_BackwS * PWM_HalfPerMax) + PWM_HalfPerMax;

    if (ccr < 0) {
        ccr = 0;
    } else if (ccr > TIM1_ARR) {
        ccr = TIM1_ARR;
    }

    return (uint16_t)ccr;
}
void svpwm_Cale(p_SVPWM pV){
    pV->tmp1= pV->Ubeta;   //Vref1
    pV->tmp2= pV->Ubeta*0.5f + pV->Ualpha*0.8660254f ;//vref2
    pV->tmp3= pV->tmp2 - pV->tmp1;   // 三相逆变换和网上公式极性不同，(√3/2)Ualpha - 0.5*Ubeta反转
    pV->VecSector=3;//基准值,反转的话理论要转为4
    pV->VecSector=(pV->tmp2>0)?(pV->VecSector-1):(pV->VecSector);
    pV->VecSector=(pV->tmp3>0)?(pV->VecSector-1):(pV->VecSector);
    pV->VecSector=(pV->tmp1<0)?(7-pV->VecSector):(pV->VecSector);
    if(pV->VecSector==1 || pV->VecSector==4)   // 根据矢量扇区计算矢量占空比Tabc
    {
        pV->Ta= pV->tmp2;
        pV->Tb= pV->tmp1-pV->tmp3;
        pV->Tc=-pV->tmp2;
    }
    else if(pV->VecSector==2 || pV->VecSector==5)
    {
        pV->Ta= pV->tmp3+pV->tmp2;
        pV->Tb= pV->tmp1;
        pV->Tc=-pV->tmp1;
    }
    else if(pV->VecSector==3 || pV->VecSector==6)
    {
        pV->Ta= pV->tmp3;
        pV->Tb=-pV->tmp3;
        pV->Tc= -(pV->tmp1+pV->tmp2);
    }
    else     //异常状态下的判断出的扇区 0---7或者其他就执行0电压矢量
    {
        pV->Ta=0;
        pV->Tb=0;
        pV->Tc=0;
    }
    pV->SVPTa = svpwm_to_ccr(pV->Ta);
    pV->SVPTb = svpwm_to_ccr(pV->Tb);
    pV->SVPTc = svpwm_to_ccr(pV->Tc);
}