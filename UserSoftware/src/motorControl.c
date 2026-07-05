/**
 * @file    motorControl.c
 * @author  nono <nono_1007@foxmail.com>
 * @brief   FOC 控制状态机: 模式切换、无感切换、VF 驱动
 */
#include "headline.h"

Control_FB motor = Control_FB_DEFAULTS;
ADCSample ADCSampPara = ADCSamp_DEFAULTS;

/**
 * PMSM 初始化
 * 上电时执行电流零点校准
 */
void PMSM_init(void){
    ZeroCurrOffset_Calibration();
}

/**
 * ADC 注入组转换完成回调（20kHz，RAM 中执行）
 * 采样 → 参数计算 → FOC 控制，构成实时中断链路
 */
_RAM_FUNC void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    Foc_Adc_Sample(&ADCSampPara);   // 读 ADC 注入寄存器 → 相电流 / 母线电压
    Foc_Para_Calc();                
    Foc_Control();                  
}

/**
 * FOC 控制状态机
 * 根据 motor.Control_Mode 切换：
 *   0: 停机
 *   1: 有感 FOC（角度来自霍尔/编码器）
 *   2: 无感 FOC（角度来自磁链观测器 + PLL）
 *   3: VF 开环压频控制
 *   4: PREPOS VF 驱动 + 观测器后台预热
 */
void Foc_Control(void)
{
    static uint8_t prev_mode = 0xFF;  // 上一周期模式（用于检测模式切换边沿）
    static uint16_t prepos_ticks = 0;
    uint8_t mode_just_entered;        // 本周期是否刚进入当前模式

    InvProtect_Check();

    mode_just_entered = (motor.Control_Mode != prev_mode);
    prev_mode = motor.Control_Mode;

    switch (motor.Control_Mode)
    {
        case 0: /* STOP — 停机, PWM 输出零矢量 */
            prepos_ticks = 0;
            break;

        case 1: /* SENSOR — 有感 FOC, 角度来自霍尔/编码器 */
            // TODO: 接入霍尔/编码器角度获取
            UVW_Axis_DQ();
            Speed_FOC();
            Idq_FOC();
            FOC_Svpwm_dq();
            break;

        case 2: /* SENSORLESS — 无感 FOC, 角度来自磁链观测器 + PLL */
            Angel_Get();
            UVW_Axis_DQ();
            Speed_FOC();
            Idq_FOC();
            FOC_Svpwm_dq();
            break;

        case 3: /* VF — 纯开环 V/f 控制 (不自动切换) */
            prepos_ticks = 0;
            VF_Control_Run(&motor);
            break;

        case 4: /* PREPOS — VF 驱动 + 观测器后台预热（不自动切换） */
            {
                float pll_step;

                if (mode_just_entered) {
                    prepos_ticks = 0;
                }

                VF_Control_Run(&motor);
                pll_step = 2.0f * PI * motor.CurrentHz * Foc_observer.Ctrl_ts;

                if (mode_just_entered) {
                    Foc_observer.Theta = motor.OpenTheta;
                    Foc_observer.PLL_Ui = pll_step;
                    Foc_observer.PLL_Err = 0.0f;
                }

                /* Feed forward VF speed, but let the observer correct the angle. */
                Foc_observer.PLL_Interg = pll_step;

                /* 后台跑观测器: Clarke 获取 Iαβ + 观测器更新磁链/PLL */
                CLARKE_PCurr.Us = Volt_CurrPara.PhaseU_Curr;
#if CURRENT_USE_W_AS_V
                CLARKE_PCurr.Vs = Volt_CurrPara.PhaseW_Curr;
#else
                CLARKE_PCurr.Vs = Volt_CurrPara.PhaseV_Curr;
#endif
                CLARKE_Cale((p_CLARKE)&CLARKE_PCurr);
                Observer_Run();

                /*
                 * Observer_Run updates flux and angle. Keep only the speed estimate anchored
                 * to VF so the handoff gate sees the actual running speed.
                 */
                Foc_observer.PLL_Interg = pll_step;
                Foc_observer.speed_hz = motor.CurrentHz;
                motor.IQAngle = (int16_t)(651.8986f * Foc_observer.Theta);
                if (motor.IQAngle > 4095) {
                    motor.IQAngle -= 4096;
                } else if (motor.IQAngle < 0) {
                    motor.IQAngle += 4096;
                }
                motor.SpeedRPM = motor.CurrentHz * 60.0f / (MOTOR_POLES / 2.0f);

                UVW_Axis_DQ(); // Update Id/Iq for Vofa diagnostics while VF still drives PWM
                if (prepos_ticks < 60000u) {
                    prepos_ticks++;
                }
            }
            break;


        default:
            break;
    }
}

/**
 * 从 SVPWM 结构体更新 PWM 占空比到 TIM1
 */
void PWM_Update_From_SVPWM(void)
{
    PWM_SetDuty(Svpwm_dq.SVPTa, Svpwm_dq.SVPTb, Svpwm_dq.SVPTc);
}

/**
 * 设置三路 PWM 占空比（带硬件限幅）
 */
void PWM_SetDuty(uint16_t ccr1, uint16_t ccr2, uint16_t ccr3)
{
    if (ccr1 > TIM1_ARR) ccr1 = TIM1_ARR;
    if (ccr2 > TIM1_ARR) ccr2 = TIM1_ARR;
    if (ccr3 > TIM1_ARR) ccr3 = TIM1_ARR;

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ccr1);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, ccr2);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, ccr3);
}
