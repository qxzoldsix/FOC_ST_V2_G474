/**
 * @file    motorControl.c
 * @author  nono <nono_1007@foxmail.com>
 * @brief   FOC 控制状态机: 模式切换、无感切换、VF 驱动
 */
#include "headline.h"

Control_FB motor = Control_FB_DEFAULTS;
ADCSample ADCSampPara = ADCSamp_DEFAULTS;

#define SPEED_RAMP_RPM_PER_SECOND    1000.0f
#define SPEED_RAMP_STEP_RPM          (SPEED_RAMP_RPM_PER_SECOND * VF_TS)

static uint8_t Motor_ModeIsRunning(uint8_t mode)
{
    return (uint8_t)(mode == MOTOR_MODE_SENSORLESS ||
                     mode == MOTOR_MODE_VF ||
                     mode == MOTOR_MODE_PREPOS ||
                     mode == MOTOR_MODE_PARAM_ID);
}

static uint8_t Motor_ModeIsClosedLoop(uint8_t mode)
{
    return (uint8_t)(mode == MOTOR_MODE_SENSORLESS);
}

static uint8_t Motor_ModeIsVfFamily(uint8_t mode)
{
    return (uint8_t)(mode == MOTOR_MODE_VF || mode == MOTOR_MODE_PREPOS);
}

static void Motor_ResetClosedLoopRuntime(void)
{
    PI_ResetRuntime(&pi_spd);
    PI_ResetRuntime(&pi_id);
    PI_ResetRuntime(&pi_iq);

    I_d_GXieLv.XieLv_X = 0.0f;
    I_d_GXieLv.XieLv_Y = 0.0f;
    I_d_GXieLv.Timer_Count = 0U;
    I_q_GXieLv.XieLv_X = 0.0f;
    I_q_GXieLv.XieLv_Y = 0.0f;
    I_q_GXieLv.Timer_Count = 0U;

    motor.V_d = 0.0f;
    motor.V_q = 0.0f;
}

static void Motor_ResetStoppedRuntime(void)
{
    Motor_ResetClosedLoopRuntime();

    SpeedRpm_GXieLv.XieLv_X = 0.0f;
    SpeedRpm_GXieLv.XieLv_Y = 0.0f;
    SpeedRpm_GXieLv.Timer_Count = 0U;

    VF_HzRamp.XieLv_X = 0.0f;
    VF_HzRamp.XieLv_Y = 0.0f;
    VF_HzRamp.Timer_Count = 0U;

    motor.SpeedRPM = 0.0f;
    motor.CurrentHz = 0.0f;
    motor.OpenTheta = 0.0f;
    motor.TargetHz = 0.0f;
    motor.V_amp = 0.0f;
    motor.IQAngle = 0;

    Svpwm_dq.Ualpha = 0.0f;
    Svpwm_dq.Ubeta = 0.0f;
    Svpwm_dq.SVPTa = 0U;
    Svpwm_dq.SVPTb = 0U;
    Svpwm_dq.SVPTc = 0U;
}

static uint8_t Motor_HandleModeTransition(uint8_t previous_mode, uint8_t next_mode)
{
    if (next_mode == MOTOR_MODE_STOP) {
        Foc_Pwm_Stop();
        return 1U;
    }

    if (!Motor_ModeIsRunning(next_mode)) {
        Motor_ControlStop();
        return 0U;
    }

    if (previous_mode == MOTOR_MODE_STOP) {
        if (Foc_Pwm_Start() == 0U) {
            Motor_ControlStop();
            return 0U;
        }
    } else if (Foc_Pwm_IsEnabled() == 0U) {
        /* 运行中若 MOE 意外丢失，不自动重开桥，必须重新下发启动命令。 */
        Motor_ControlStop();
        return 0U;
    }

    if (Motor_ModeIsClosedLoop(next_mode) &&
        !Motor_ModeIsClosedLoop(previous_mode)) {
        Motor_ResetClosedLoopRuntime();
        SpeedRpm_GXieLv.XieLv_Y = motor.SpeedRPM;
    }

    if (Motor_ModeIsVfFamily(next_mode) &&
        !Motor_ModeIsVfFamily(previous_mode)) {
        VF_HzRamp.XieLv_Y = motor.CurrentHz;
        VF_HzRamp.XieLv_X = motor.TargetHz;
        if (Motor_ModeIsClosedLoop(previous_mode)) {
            motor.OpenTheta = Foc_observer.Theta;
        }
    }

    return 1U;
}

/**
 * PMSM 初始化
 * 上电时执行电流零点校准
 */
void PMSM_init(void)
{
    ZeroCurrOffset_Calibration();
    Motor_ResetStoppedRuntime();
    SpeedRpm_GXieLv.XieLv_Grad = SPEED_RAMP_STEP_RPM;
}

/**
 * ADC 注入组转换完成回调（20kHz，RAM 中执行）
 * 采样 → 参数计算 → FOC 控制，构成实时中断链路
 */
_RAM_FUNC void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc == NULL || hadc->Instance != ADC1) {
        return;
    }

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
    uint8_t current_mode;
    uint8_t mode_just_entered;

    InvProtect_Check();

    current_mode = motor.Control_Mode;
    mode_just_entered = (uint8_t)(current_mode != prev_mode);
    if (mode_just_entered) {
        if (Motor_HandleModeTransition(prev_mode, current_mode) == 0U) {
            prev_mode = motor.Control_Mode;
            return;
        }
        current_mode = motor.Control_Mode;
        prev_mode = current_mode;
    }

    if (current_mode != MOTOR_MODE_STOP && Foc_Pwm_IsEnabled() == 0U) {
        Motor_ControlStop();
        prev_mode = MOTOR_MODE_STOP;
        return;
    }

    switch (current_mode)
    {
        case MOTOR_MODE_STOP:
            break;

        case MOTOR_MODE_SENSOR:
            /* 当前未接入传感器角度源；模式切换检查会在到达这里前停机。 */
            Motor_ControlStop();
            prev_mode = MOTOR_MODE_STOP;
            break;

        case MOTOR_MODE_SENSORLESS:
            Angel_Get();
            UVW_Axis_DQ();
            Speed_FOC();
            Idq_FOC();
            FOC_Svpwm_dq();
            break;

        case MOTOR_MODE_VF:
            VF_Control_Run(&motor);
            break;

        case MOTOR_MODE_PREPOS:
            {
                float pll_step;

                VF_Control_Run(&motor);
                pll_step = 2.0f * PI * motor.CurrentHz * Foc_observer.Ctrl_ts;

                if (mode_just_entered) {
                    Foc_observer.Theta = motor.OpenTheta;
                    Foc_observer.PLL_Ui = pll_step;
                    Foc_observer.PLL_Err = 0.0f;
                }

                /* VF 提供速度前馈，观测器只负责修正角度。 */
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

                /* 保持速度估计跟随 VF，供切闭环时无扰接管。 */
                Foc_observer.PLL_Interg = pll_step;
                Foc_observer.speed_hz = motor.CurrentHz;
                motor.IQAngle = (int16_t)((uint16_t)(651.8986f *
                                      Foc_observer.Theta) & 0x0FFFU);
                motor.SpeedRPM = motor.CurrentHz * 60.0f / (MOTOR_POLES / 2.0f);

                /* VF 仍驱动 PWM，但同步更新 Id/Iq 供诊断和闭环接管。 */
                UVW_Axis_DQ();
            }
            break;

        case MOTOR_MODE_PARAM_ID:
            if (mode_just_entered) ParamId_Start();
            ParamId_Run();
            break;

        default:
            Motor_ControlStop();
            prev_mode = MOTOR_MODE_STOP;
            break;
    }
}

/**
 * @brief  接受一条运行命令并安全开启三相 PWM
 * @retval 1=已接受，0=参数非法或仍有故障
 */
uint8_t Motor_ControlStart(uint8_t mode, float target_speed_rpm)
{
    float target_hz;

    /* 使用最新采样值复查保护，避免故障清除与再次启动之间的窗口。 */
    InvProtect_Check();

    /* SENSOR 尚未接入角度源，禁止通过外部命令使能功率级。 */
    if (mode < MOTOR_MODE_SENSORLESS || mode > MOTOR_MODE_PREPOS ||
        !isfinite(target_speed_rpm) || target_speed_rpm < 0.0f ||
        InvProtect_GetFault() != FAULT_NONE) {
        return 0U;
    }

    target_hz = target_speed_rpm * (MOTOR_POLES / 2.0f) / 60.0f;
    if (!isfinite(target_hz) || target_hz > VF_FREQ_MAX) {
        return 0U;
    }

    if ((mode == MOTOR_MODE_VF || mode == MOTOR_MODE_PREPOS) &&
        (motor.TargetVolt <= VF_VOLTAGE_MIN ||
         motor.TargetVolt > VF_VOLTAGE_MAX)) {
        return 0U;
    }

    if (motor.Control_Mode == MOTOR_MODE_STOP) {
        Motor_ResetStoppedRuntime();
    }

    SpeedRpm_GXieLv.XieLv_X = target_speed_rpm;
    motor.TargetHz = target_hz;
    /* 最后写模式，确保 20 kHz 控制中断只看到完整的新命令。 */
    motor.Control_Mode = mode;
    return 1U;
}

/**
 * @brief  停止控制并立即封锁三相 PWM
 */
void Motor_ControlStop(void)
{
    /* 先发布 STOP 状态，再立即封波；随后清控制器历史量。 */
    motor.Control_Mode = MOTOR_MODE_STOP;
    Foc_Pwm_Stop();
    Motor_ResetStoppedRuntime();
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
    if (Foc_Pwm_IsEnabled() == 0U) {
        return;
    }

    if (ccr1 > TIM1_ARR) ccr1 = TIM1_ARR;
    if (ccr2 > TIM1_ARR) ccr2 = TIM1_ARR;
    if (ccr3 > TIM1_ARR) ccr3 = TIM1_ARR;

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ccr1);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, ccr2);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, ccr3);
}

/* ========== CAN 控制回调 ========== */

void CANFD_OnNMT(uint8_t cmd)
{
    switch (cmd) {
        case NMT_START:
            /* NMT START 只允许 PDO 通信，不等同于电机启动。 */
            break;
        case NMT_STOP:
        case NMT_PRE_OP:
            Motor_ControlStop();
            break;
        case NMT_RESET:
            Motor_ControlStop();
            NVIC_SystemReset();
            break;
        default: break;
    }
}

void CANFD_OnPDO1_RX(PDO1_RX_Data *data)
{
    if (data == NULL) return;

    switch (data->Command) {
        case CTRL_START:
            (void)Motor_ControlStart(data->Control_Mode, data->TargetSpeedRPM);
            break;

        case CTRL_STOP:
        case CTRL_QUICK_STOP:
            Motor_ControlStop();
            break;

        case CTRL_FAULT_CLEAR:
            InvProtect_Clear();
            break;

        default: break;
    }
}

void CANFD_OnPDO2_RX(PDO2_RX_Data *data)
{
    if (data == NULL) return;
    // Id/Iq 目标值，后续接入电流环
    (void)data;
}
