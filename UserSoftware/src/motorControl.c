#include "headline.h"

Control_FB motor = Control_FB_DEFAULTS;
SVPWM Svpwm_zero = SVPWM_DEFAULTS;
ADCSample ADCSampPara = ADCSamp_DEFAULTS;



/**
 * VF → 无感 FOC 平滑过渡
 * 将 VF 的开环角度/频率状态注入观测器，清零磁链积分器避免 PLL 震荡。
 *
 * 设计要点:
 *   - 磁链积分器清零 → PLL_Err 从 0 起步，PLL 在 VF 频率下开环运行
 *   - 磁链从反电动势自然建立 (~30ms @5Hz)，PLL 逐步收敛
 *   - 不清 Flux_Observer_Init() 以保留电机参数和 PLL 增益
 */
static void VF_to_Sensorless_Sync(void)
{
    float spd_current_rpm;
    float spd_target_rpm;
    float omega_e;   // 电角速度 rad/s

    spd_current_rpm = motor.CurrentHz * 60.0f / (MOTOR_POLES / 2.0f);
    spd_target_rpm  = motor.TargetHz  * 60.0f / (MOTOR_POLES / 2.0f);
    omega_e = 2.0f * PI * motor.CurrentHz;

    /*
     * PLL 角度/频率同步到 VF 当前值.
     * SMO 电流估计同步到实测值，反电动势按当前频率初始化.
     */
    Foc_observer.Theta      = motor.OpenTheta;
    Foc_observer.PLL_Ui     = omega_e * Foc_observer.Ctrl_ts;
    Foc_observer.PLL_Ui_Old = Foc_observer.PLL_Ui;
    Foc_observer.PLL_Interg = Foc_observer.PLL_Ui;
    Foc_observer.PLL_Err    = 0.0f;
    Foc_observer.speed_hz   = motor.CurrentHz;
    motor.SpeedRPM          = spd_current_rpm;

    /* SMO 内部状态同步 */
    Foc_observer.Ialpha_hat = CLARKE_PCurr.Alpha;
    Foc_observer.Ibeta_hat  = CLARKE_PCurr.Beta;
    /* 按 VF 频率预初始化反电动势: Eα = -ωψ·sinθ, Eβ = ωψ·cosθ */
    {
        float sin_th = sinf(motor.OpenTheta);
        float cos_th = cosf(motor.OpenTheta);
        float emag   = omega_e * Foc_observer.Flux;
        Foc_observer.Ealpha = -emag * sin_th;
        Foc_observer.Ebeta  =  emag * cos_th;
        Foc_observer.E_mag  = emag;
    }

    /* PLL 增益: 过渡期提高增益快速锁定，ramp 恢复至正常值 */
    Foc_observer.PLL_kp_target = SMO_PLL_KP;        // 正常 Kp
    Foc_observer.PLL_ki_target = SMO_PLL_KI;        // 正常 Ki
    Foc_observer.PLL_kp = SMO_PLL_KP * 3.0f;        // 过渡 Kp（提高 3 倍加速锁定）
    Foc_observer.PLL_ki = SMO_PLL_KI * 2.0f;        // 过渡 Ki（提高 2 倍）
    Foc_observer.pll_ramp_active = 1;

    /* 电流环: 从 VF 当前输出继承，转标幺值 */
    {
        float v_amp_pu = motor.V_amp / NOMINAL_BUS_VOLTAGE;
        pi_iq.i1   = v_amp_pu;
        pi_iq.v1   = v_amp_pu;
        pi_iq.OutF = v_amp_pu;
    }
    pi_id.i1   = 0.0f;
    pi_id.v1   = 0.0f;
    pi_id.OutF = 0.0f;

    /* 速度环: 从 CurrentHz RPM 斜坡到 TargetHz RPM */
    SpeedRpm_GXieLv.XieLv_X    = spd_target_rpm;
    SpeedRpm_GXieLv.XieLv_Y    = spd_current_rpm;
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
 */
void Foc_Control(void)
{
    static uint8_t prev_mode = 0xFF;  // 上一周期模式（用于检测模式切换边沿）
    uint8_t mode_just_entered;        // 本周期是否刚进入当前模式

    InvProtect_Check();

    mode_just_entered = (motor.Control_Mode != prev_mode);
    prev_mode = motor.Control_Mode;

    switch (motor.Control_Mode)
    {
        case 0: /* STOP — 停机, PWM 输出零矢量 */
            break;

        case 1: /* SENSOR — 有感 FOC, 角度来自霍尔/编码器 */
            // TODO: 接入霍尔/编码器角度获取（UVW_Axis_DQ_HALL 或 magnetic）
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
                float psi_r_mag = sqrtf(FluxR_in_wb[0] * FluxR_in_wb[0] +
                                        FluxR_in_wb[1] * FluxR_in_wb[1]);
                if (psi_r_mag < Foc_observer.Flux * 0.3f) {
                    /*
                     * 冷启动: 观测器从未运行（从 STOP/VF 直接切来）。
                     * 调用 VF→Sensorless 同步函数完成全部状态迁移。
                     */
                    VF_to_Sensorless_Sync();
                } else {
                    /*
                     * 暖启动: 观测器已在 Mode 4 预热就绪（磁链≥30% 参考值）。
                     * 同步 PLL 角度到 VF 角度，消除切换瞬间电压矢量跳变。
                     * SMO 状态同步 + PI 预加载标幺值，确保电压连续。
                     */
                    VF_to_Sensorless_Sync();
                }
            }
            Angel_Get();            // 磁链观测器 → PLL → motor.IQAngle
            UVW_Axis_DQ();          // Clarke → Park → Id/Iq
            Speed_FOC();            // 速度环 PI → Iq 给定
            Idq_FOC();              // 电流环 PI → Vd/Vq
            FOC_Svpwm_dq();         // 反 Park → SVPWM → PWM 更新
            break;

        case 3: /* VF — 纯开环 V/f 控制 (不自动切换) */
            VF_Control_Run(&motor);
            break;

        case 4: /* PREPOS — VF 驱动 + 观测器后台预热（不自动切换） */
            /*
             * 流程:
             *   1. VF 开环驱动 PWM (用 VF 角度)
             *   2. 后台跑 Clarke → 观测器, 磁链积分器用真实 Vᵅᵝ/Iᵅᵝ 预热
             *   3. 用户在 Vofa/LCD 上确认磁链幅值 → MOTOR_FLUX、PLL_Err → 0 后,
             *      手动按键切 Mode 2 (SENSORLESS)
             *
             * 关键: 不自动切换! 让观测器充分预热 >100ms, 确认收敛后再手动切.
             *       磁链积分器不重置, 让它从真实电压电流自然收敛.
             */
            VF_Control_Run(&motor);

            /* 后台跑观测器: Clarke 获取 Iαβ + 观测器更新磁链/PLL */
            CLARKE_PCurr.Us = Volt_CurrPara.PhaseU_Curr;
            CLARKE_PCurr.Vs = Volt_CurrPara.PhaseV_Curr;
            CLARKE_Cale((p_CLARKE)&CLARKE_PCurr);
            Angel_Get();   // Observer_Run → 磁链积分预热 + PLL 跟踪
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
