#include "headline.h"
PI_controller   pi_spd=PI_Control_DEFAULTS ;
PI_controller   pi_id =PI_Control_DEFAULTS ;
PI_controller   pi_iq =PI_Control_DEFAULTS ;
void PID_controller(p_PI_Control  pV){
    pV->up = pV->Ref - pV->Fbk;
    // ���ַ��뿹����: ���Ԥ����ֵ v1 �Ƿ񳬳��޷�
    // v1 �� [Umin, Umax] ������������; ������ֹͣ����
    pV->ui = (pV->v1 >= pV->Umax || pV->v1 <= pV->Umin) ? pV->i1 : (pV->Ki * pV->up + pV->i1);
    pV->ui = Limit_Sat(pV->ui, pV->Umax, pV->Umin); 
    pV->i1 = pV->ui; 

    /* control output*/
    pV->v1 = pV->Kp*pV->up + pV->ui;  
    pV->Out= Limit_Sat(pV->v1, pV->Umax, pV->Umin); 
}
/**
 * PI ������������ʼ��
 * ������ PI ����������ֵ������Ĭ��ȫ 0 ʱ�ٶȻ�/�����������Ϊ 0
 * ����ʱ��: ����ʼ��ĩβ, Flux_Observer_Init() ֮��
 */
void PI_Init(void)
{
    /* ---- �ٶȻ� PI ---- */
    pi_spd.Kp   = 0.1f;
    pi_spd.Ki   = 0.005f;
    pi_spd.Umax =  12.0f;
    pi_spd.Umin = -12.0f;

    /* ---- d axis PI ---- */
    pi_id.Kp   = 0.3f;
    pi_id.Ki   = 0.01f;
    pi_id.Umax =  12.0f;
    pi_id.Umin = -12.0f;

    /* ---- q axis PI ---- */
    pi_iq.Kp   = 0.3f;
    pi_iq.Ki   = 0.01f;
    pi_iq.Umax =  12.0f;
    pi_iq.Umin = -12.0f;
}
