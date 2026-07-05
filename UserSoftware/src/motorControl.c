/**
 * @file    motorControl.c
 * @author  nono <nono_1007@foxmail.com>
 * @brief   FOC 控制状态机: 模式切换、无感切换、VF 驱动
 */
#include "headline.h"

Control_FB motor = Control_FB_DEFAULTS;
SVPWM Svpwm_zero = SVPWM_DEFAULTS;
ADCSample ADCSampPara = ADCSamp_DEFAULTS;

#define PREPOS_MIN_READY_TICKS      4000u  /* 4000 * 50us = 200ms */
#define SENSORLESS_READY_FLUX_MIN   0.70f
#define SENSORLESS_READY_FLUX_MAX   1.40f
#define SENSORLESS_READY_SPEED_ERR  2.0f
#define SENSORLESS_READY_PLL_ERR    0.004f
#define SENSORLESS_READY_ID_ABS     3.5f
#define SENSORLESS_READY_IQ_ABS     12.0f
#define SENSORLESS_HANDOFF_IQ_ABS   1.5f
#define SENSORLESS_SOFTSTART_TICKS  4000u  /* 4000 * 50us = 200ms */
#define SENSORLESS_SPEED_IQ_SIGN   (-1.0f)

static void Sensorless_Handoff_Preload(float spd_start_rpm, float spd_target_rpm)
{
    float vq_preload;
    float bus_pu;
    bus_pu = Volt_CurrPara.BUS_Voltage / NOMINAL_BUS_VOLTAGE;
    if (bus_pu <= 0.0f) {
        bus_pu = pi_iq.Umax;
    }
    vq_preload = Limit_Sat(motor.V_amp, bus_pu, -bus_pu);

    pi_spd.i1   = 0.0f;
    pi_spd.v1   = 0.0f;
    pi_spd.Out  = 0.0f;
    pi_spd.OutF = 0.0f;
    pi_spd.Ref  = spd_start_rpm;
    pi_spd.Fbk  = spd_start_rpm;

    I_q_GXieLv.XieLv_X = 0.0f;
    I_q_GXieLv.XieLv_Y = 0.0f;
    I_d_GXieLv.XieLv_X = 0.0f;
    I_d_GXieLv.XieLv_Y = 0.0f;

    pi_iq.Ref  = 0.0f;
    pi_iq.Fbk  = PARK_PCurr.Qs;
    pi_iq.i1   = vq_preload;
    pi_iq.v1   = vq_preload;
    pi_iq.Out  = vq_preload;
    pi_iq.OutF = vq_preload;

    pi_id.Ref  = 0.0f;
    pi_id.Fbk  = PARK_PCurr.Ds;
    pi_id.i1   = 0.0f;
    pi_id.v1   = 0.0f;
    pi_id.Out  = 0.0f;
    pi_id.OutF = 0.0f;

    SpeedRpm_GXieLv.XieLv_X = spd_target_rpm;
    SpeedRpm_GXieLv.XieLv_Y = spd_start_rpm;
    SpeedRpm_GXieLv.XieLv_Grad = 0.5f;
    SpeedRpm_GXieLv.Grad_Timer = 100;
    SpeedRpm_GXieLv.Timer_Count = 0;
}

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
    static uint16_t sensorless_ticks = 0;
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
            /*
             * 冷启动检测: 仅在刚进入 Mode 2 时执行一次.
             * 如果转子磁链幅值 < 参考值的 30%, 说明观测器未预热,
             * 同步 PLL 到当前 VF 状态作为起点, 之后让观测器自行收敛.
             * 重要: 不能每个周期都复位 PLL, 否则角度无法累加 → 电机锁死.
             */
            if (mode_just_entered) {
                float psi_r_mag;
                float speed_err_hz;
                sensorless_ticks = 0;

                if (fabsf(motor.CurrentHz) < VF_TO_SENSORLESS_HZ ||
                    prepos_ticks < PREPOS_MIN_READY_TICKS) {
                    motor.Control_Mode = 4;
                    break;
                }

                psi_r_mag = sqrtf(FluxR_in_wb[0] * FluxR_in_wb[0] +
                                  FluxR_in_wb[1] * FluxR_in_wb[1]);
                speed_err_hz = fabsf(Foc_observer.speed_hz - motor.CurrentHz);

                if (psi_r_mag < Foc_observer.Flux * SENSORLESS_READY_FLUX_MIN ||
                    psi_r_mag > Foc_observer.Flux * SENSORLESS_READY_FLUX_MAX ||
                    speed_err_hz > SENSORLESS_READY_SPEED_ERR ||
                    fabsf(Foc_observer.PLL_Err) > SENSORLESS_READY_PLL_ERR ||
                    fabsf(PARK_PCurr.Ds) > SENSORLESS_READY_ID_ABS ||
                    fabsf(PARK_PCurr.Qs) > SENSORLESS_READY_IQ_ABS) {
                    motor.Control_Mode = 4;
                    break;
                }

                UVW_Axis_DQ();

                if (psi_r_mag < Foc_observer.Flux * 0.3f) {
                    /*
                     * 冷启动: 观测器从未运行（从 STOP/VF 直接切来）。
                     * 同步 PLL 到 VF 状态一次 → 激活 ramp → 预加载电流环。
                     * 之后让观测器自行收敛。风险较高，推荐走 Mode 4 预热后再切。
                     */
                    Foc_observer.Theta      = motor.OpenTheta;
                    Foc_observer.PLL_Ui     = 2.0f * PI * motor.CurrentHz * Foc_observer.Ctrl_ts;
                    Foc_observer.PLL_Ui_Old = Foc_observer.PLL_Ui;
                    Foc_observer.PLL_Interg = Foc_observer.PLL_Ui;
                    Foc_observer.PLL_Err    = 0.0f;
                    Foc_observer.speed_hz   = motor.CurrentHz;
                    /* 激��?PLL 增益 ramp */
                    Foc_observer.PLL_kp_target = 20.0f;
                    Foc_observer.PLL_ki_target = 10.0f;
                    Foc_observer.PLL_kp = 4.0f;
                    Foc_observer.PLL_ki = 1.0f;
                    Foc_observer.pll_ramp_active = 1;
                    Sensorless_Handoff_Preload(
                        motor.CurrentHz * 60.0f / (MOTOR_POLES / 2.0f),
                        motor.TargetHz * 60.0f / (MOTOR_POLES / 2.0f));
                } else {
                    /*
                     * 暖启动: 观测器已在 Mode 4 预热就绪（磁链≥30% 参考值）。
                     * PLL 已锁定不动它 → 只设速度斜坡 + 保守预加载 Iq 电流环。
                     * 电压连续性好，切换最平滑。
                     */
                    float spd_est_rpm  = Foc_observer.speed_hz * 60.0f / (MOTOR_POLES / 2.0f);
                    float spd_targ_rpm = motor.TargetHz * 60.0f / (MOTOR_POLES / 2.0f);
                    Sensorless_Handoff_Preload(spd_est_rpm, spd_targ_rpm);
                }
            }
            Angel_Get();            // 磁链观测器 → PLL → motor.IQAngle
            UVW_Axis_DQ();          // Clarke → Park → Id/Iq
            Speed_FOC();            // 速度环 PI → Iq 给定
            if (sensorless_ticks < SENSORLESS_SOFTSTART_TICKS) {
                float iq_lim = SENSORLESS_HANDOFF_IQ_ABS +
                               (pi_spd.Umax - SENSORLESS_HANDOFF_IQ_ABS) *
                               ((float)sensorless_ticks / (float)SENSORLESS_SOFTSTART_TICKS);
                I_q_GXieLv.XieLv_Y = Limit_Sat(I_q_GXieLv.XieLv_Y, iq_lim, -iq_lim);
                sensorless_ticks++;
            }
            Idq_FOC();              // 电流环 PI → Vd/Vq
            FOC_Svpwm_dq();         // 反 Park → SVPWM → PWM 更新
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
                    Foc_observer.PLL_Ui_Old = pll_step;
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
