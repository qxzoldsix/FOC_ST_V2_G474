#include "headline.h"
PI_controller   pi_spd=PI_Control_DEFAULTS ;
PI_controller   pi_id =PI_Control_DEFAULTS ;
PI_controller   pi_iq =PI_Control_DEFAULTS ;
void PID_controller(p_PI_Control  pV){
    pV->up = pV->Ref - pV->Fbk;
    // 积分分离抗饱和: 检查预饱和值 v1 是否超出限幅
    // v1 在 [Umin, Umax] 内则正常积分; 超出则停止积分
    pV->ui = (pV->v1 >= pV->Umax || pV->v1 <= pV->Umin) ? pV->i1 : (pV->Ki * pV->up + pV->i1);
    pV->ui = Limit_Sat(pV->ui, pV->Umax, pV->Umin); 
    pV->i1 = pV->ui; 

    /* control output*/
    pV->v1 = pV->Kp*pV->up + pV->ui;  
    pV->Out= Limit_Sat(pV->v1, pV->Umax, pV->Umin); 
}
/**
 * PI 控制器参数初始化
 * 给三个 PI 控制器赋初值，否则默认全 0 时速度环/电流环输出恒为 0
 * 调用时机: 主初始化末尾, Flux_Observer_Init() 之后
 */
void PI_Init(void)
{
    /* ---- 速度环 PI ---- */
    pi_spd.Kp   = 0.01f;      // 比例系数
    pi_spd.Ki   = 0.0001f;     // 积分系数
    pi_spd.Umax =  12.0f;     // 上限: Iq 最大电流 5A
    pi_spd.Umin = -12.0f;     // 下限

    /* ---- d 轴电流环 PI ---- */
    pi_id.Kp   = 0.01f;       // 比例系数
    pi_id.Ki   = 0.00001f;       // 积分系数
    pi_id.Umax =  12.0f;     // 上限 (V)
    pi_id.Umin = -12.0f;     // 下限 (V)

    /* ---- q 轴电流环 PI ---- */
    pi_iq.Kp   =  0.01f;       // 比例系数
    pi_iq.Ki   =0.00001f;        // 积分系数
    // Umax/Umin 由 Idq_FOC 根据电压圆约束动态设置
    pi_iq.Umax =  12.0f;
    pi_iq.Umin = -12.0f;
}
