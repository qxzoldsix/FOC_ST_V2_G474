#include "headline.h"

Control_FB motor = Control_FB_DEFAULTS;
SVPWM Svpwm_zero = SVPWM_DEFAULTS;
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

        case 2: /* SENSORLESS — 纯闭环：观测器角度 + Id=0 + Vq开环启动 */
        {
            static float vq_cmd = 0.0f;

            if (mode_just_entered) vq_cmd = 0.0f;

            Angel_Get();            // 观测器 → motor.IQAngle
            UVW_Axis_DQ();          // Clarke/Park → Id/Iq（用观测器角度）

            /* Vq 电压斜坡: 0 → TargetVolt，推动电机起步 */
            float vq_tgt = motor.TargetVolt;
            if (vq_tgt < 0.5f) vq_tgt = 3.0f;
            if (vq_cmd < vq_tgt) vq_cmd += 0.01f;
            if (vq_cmd > vq_tgt) vq_cmd = vq_tgt;

            /* Id = 0 闭环 */
            pi_id.Ref = 0.0f;
            pi_id.Fbk = PARK_PCurr.Ds;
            PID_controller((p_PI_Control)&pi_id);
            pi_id.OutF = pi_id.OutF * GM_Low_Lass_A + pi_id.Out * GM_Low_Lass_B;
            motor.V_d = pi_id.OutF;

            /* Vq 开环 + 电压圆限幅 */
            {
                float bus_pu = Volt_CurrPara.BUS_Voltage / NOMINAL_BUS_VOLTAGE;
                float val = bus_pu * bus_pu - motor.V_d * motor.V_d;
                float us_lim = (val > 0.0f) ? sqrtf(val) : 0.0f;
                motor.V_q = (vq_cmd >  us_lim) ?  us_lim : vq_cmd;
                motor.V_q = (motor.V_q < -us_lim) ? -us_lim : motor.V_q;
            }

            FOC_Svpwm_dq();         // 反 Park → SVPWM → PWM
            break;
        }

        case 3: /* VF — 纯开环 V/f 控制 (不自动切换) */
            VF_Control_Run(&motor);
            break;

        case 4: /* PREPOS — VF 驱动 + 观测器后台预热，为切无感做准备 */
            VF_Control_Run(&motor);         // VF 继续推电机保持转速
            Angel_Get();                    // 观测器后台收敛磁链 & 锁定角度
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
