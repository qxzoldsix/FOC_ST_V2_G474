/**
 * @file    PI_Cale.c
 * @author  nono <nono_1007@foxmail.com>
 * @brief   PI 控制器: 积分分离抗饱和 + 低通滤波输出
 */
#include "headline.h"
PI_controller pi_spd = PI_Control_DEFAULTS;
PI_controller pi_id = PI_Control_DEFAULTS;
PI_controller pi_iq = PI_Control_DEFAULTS;

void PI_ResetRuntime(p_PI_Control pV)
{
    if (pV == NULL) {
        return;
    }

    pV->Ref = 0.0f;
    pV->Fbk = 0.0f;
    pV->Out = 0.0f;
    pV->OutF = 0.0f;
    pV->up = 0.0f;
    pV->ui = 0.0f;
    pV->v1 = 0.0f;
    pV->i1 = 0.0f;
}

void PID_controller(p_PI_Control pV)
{
    pV->up = pV->Ref - pV->Fbk;
    // 积分分离抗饱和: 若预输出值 v1 超出限幅
    // v1 在 [Umin, Umax] 内时正常积分; 否则停止积分
    pV->ui = (pV->v1 >= pV->Umax || pV->v1 <= pV->Umin) ? pV->i1 : (pV->Ki * pV->up + pV->i1);
    pV->ui = Limit_Sat(pV->ui, pV->Umax, pV->Umin);
    pV->i1 = pV->ui;

    pV->v1 = pV->Kp * pV->up + pV->ui;
    pV->Out = Limit_Sat(pV->v1, pV->Umax, pV->Umin);
}
/**
 * PI 控制器参数初始化
 * 注意: PI 默认全 0 时速度环/电流环输出均为 0
 * 调用时机: 主初始化末尾, FluxObserverInit() 之后
 */
void PiInit(void)
{
    /* ---- 速度环 PI ---- */
    pi_spd.Kp   = 0.01f;
    pi_spd.Ki   = 0.0001f;
    pi_spd.Umax = 12.0f;
    pi_spd.Umin = -12.0f;

    /* ---- d 轴电流 PI：输出为 SVPWM 标幺电压 ---- */
    pi_id.Kp   = 0.01f;
    pi_id.Ki   = 0.00001f;
    pi_id.Umax = PWM_GetVoltageLimitPu();
    pi_id.Umin = -pi_id.Umax;

    /* ---- q 轴电流 PI ---- */
    pi_iq.Kp   = 0.01f;
    pi_iq.Ki   = 0.00001f;
    // Umax/Umin 在 Idq_FOC 中根据电压圆约束动态设置
    pi_iq.Umax = PWM_GetVoltageLimitPu();
    pi_iq.Umin = -pi_iq.Umax;

    PI_ResetRuntime(&pi_spd);
    PI_ResetRuntime(&pi_id);
    PI_ResetRuntime(&pi_iq);
}
