#include "headline.h"

Control_FB motor = Control_FB_DEFAULTS;
SVPWM Svpwm_zero = SVPWM_DEFAULTS;
ADCSample ADCSampPara = ADCSamp_DEFAULTS;

/* ========== VF → 无感 FOC 自动过渡参数 ========== */
#define VF_TO_SENSORLESS_HZ  10.0f  // 切无感阈值(Hz): 太低反电动势不够, 观测器无法收敛

/* ========== Mode 4 预定位参数 ========== */
#define I_PREPOS_A      0.5f        // 预定位直流电流 (A)
#define PREPOS_TIME_MS  800         // 预定位持续时间 (ms)
static uint32_t prepos_tick = 0;    // 预定位计时器 (PWM 周期数)

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

    spd_current_rpm = motor.CurrentHz * 60.0f / (MOTOR_POLES / 2.0f);
    spd_target_rpm  = motor.TargetHz  * 60.0f / (MOTOR_POLES / 2.0f);

    /*
     * PLL 角度/频率同步到 VF 当前值.
     * 磁链积分器已在 mode4 后台预热, 此处不重置.
     */
    Foc_observer.Theta      = motor.OpenTheta;
    Foc_observer.PLL_Ui     = 2.0f * PI * motor.CurrentHz * Foc_observer.Ctrl_ts;
    Foc_observer.PLL_Ui_Old = Foc_observer.PLL_Ui;
    Foc_observer.PLL_Interg = Foc_observer.PLL_Ui;
    Foc_observer.PLL_Err    = 0.0f;
    Foc_observer.speed_hz   = motor.CurrentHz;
    motor.SpeedRPM          = spd_current_rpm;

    /*
     * 保存正常增益目标 → 降低增益过渡 → 激活自动 ramp 恢复
     * 切换瞬间用低增益避免震荡, Observer_Run 每周期自动向目标恢复.
     */
    Foc_observer.PLL_kp_target = 20.0f;   // 正常 Kp（与 Flux_Observer_Init 一致）
    Foc_observer.PLL_ki_target = 10.0f;   // 正常 Ki
    Foc_observer.PLL_kp = 4.0f;           // 过渡 Kp（不要太低, 否则跟踪不上）
    Foc_observer.PLL_ki = 1.0f;           // 过渡 Ki
    Foc_observer.pll_ramp_active = 1;     // 激活自动 ramp 恢复

    /*
     * 电流环: 从 VF 当前输出继承.
     * 注: V_amp 是 VF 的输出电压幅值, 近似于 V_q (Id=0 控制时).
     *     这里将 V_q 预加载到 pi_iq 的输出/积分, 让切换瞬间电压连续.
     *     同时将 Iq 给定斜坡起点设为 0, 让速度环从当前状态平滑接管.
     */
    pi_iq.i1   = motor.V_amp * 0.5f;  // 保守预加载, 留一半余量让 PI 自己调
    pi_iq.v1   = motor.V_amp * 0.5f;
    pi_iq.OutF = motor.V_amp * 0.5f;
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
                    /* 观测器未就绪 → 同步 PLL 到 VF 状态（仅此一次） */
                    Foc_observer.Theta      = motor.OpenTheta;
                    Foc_observer.PLL_Ui     = 2.0f * PI * motor.CurrentHz * Foc_observer.Ctrl_ts;
                    Foc_observer.PLL_Ui_Old = Foc_observer.PLL_Ui;
                    Foc_observer.PLL_Interg = Foc_observer.PLL_Ui;
                    Foc_observer.PLL_Err    = 0.0f;
                    Foc_observer.speed_hz   = motor.CurrentHz;
                    /* 激活 PLL 增益 ramp */
                    Foc_observer.PLL_kp_target = 20.0f;
                    Foc_observer.PLL_ki_target = 10.0f;
                    Foc_observer.PLL_kp = 4.0f;
                    Foc_observer.PLL_ki = 1.0f;
                    Foc_observer.pll_ramp_active = 1;
                    /* 电流环预加载（保守值） */
                    pi_iq.i1   = motor.V_amp * 0.5f;
                    pi_iq.v1   = motor.V_amp * 0.5f;
                    pi_iq.OutF = motor.V_amp * 0.5f;
                    pi_id.i1   = 0.0f;
                    pi_id.v1   = 0.0f;
                    pi_id.OutF = 0.0f;
                    /* 速度环从当前状态起步 */
                    SpeedRpm_GXieLv.XieLv_X    = motor.TargetHz * 60.0f / (MOTOR_POLES / 2.0f);
                    SpeedRpm_GXieLv.XieLv_Y    = motor.CurrentHz * 60.0f / (MOTOR_POLES / 2.0f);
                    SpeedRpm_GXieLv.XieLv_Grad = 0.5f;
                    SpeedRpm_GXieLv.Grad_Timer = 100;
                    SpeedRpm_GXieLv.Timer_Count = 0;
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

        case 4: /* VF → Sensorless — VF 启动自动切磁链观测器 */
            /*
             * 流程:
             *   1. VF 开环驱动 PWM (用 VF 角度)
             *   2. 后台跑 Clarke → 观测器, 磁链积分器用真实 Vᵅᵝ/Iᵅᵝ 预热
             *   3. CurrentHz 达阈值时: 同步 PLL 角度/频率到 VF, 降 PLL 增益,
             *      电流环预加载 V_amp, 速度环从 CurrentHz 斜坡起步.
             *   4. 切 mode2 — 观测器无缝衔接.
             *
             * 关键: 磁链积分器不重置! 让它在 VF 期间从真实电压电流自然收敛,
             *       切换时 PLL 从 VF 角度起步跟踪已预热好的磁链, 避免突变.
             */
            VF_Control_Run(&motor);

            /* 后台跑观测器: Clarke 获取 Iαβ + 观测器更新磁链/PLL */
            CLARKE_PCurr.Us = Volt_CurrPara.PhaseU_Curr;
            CLARKE_PCurr.Vs = Volt_CurrPara.PhaseV_Curr;
            CLARKE_Cale((p_CLARKE)&CLARKE_PCurr);
            Angel_Get();   // Observer_Run → 磁链积分预热 + PLL 跟踪

            if (motor.CurrentHz >= VF_TO_SENSORLESS_HZ) {
                VF_to_Sensorless_Sync();   /* PLL 同步 + 降增益 + 电流环预加载 */
                /* 注意: 磁链积分器已在上面预热, 此处不清零! */
                motor.Control_Mode = 2;
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
