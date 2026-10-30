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

typedef struct {
    MotorControlMode active_mode;
} MotorStateMachine;

static MotorStateMachine motor_sm = {
    .active_mode = MOTOR_MODE_STOP
};

static uint8_t Motor_ModeIsSupported(MotorControlMode mode)
{
    /*
     * 这里只声明状态机认识的模式。I/F/HFI 当前先接入状态机骨架，
     * 具体启动算法尚未实现；运行函数会安全停机，禁止误把空壳当成闭环。
     */
    return (uint8_t)(mode == MOTOR_MODE_SENSORLESS ||
                     mode == MOTOR_MODE_VF ||
                     mode == MOTOR_MODE_PREPOS ||
                     mode == MOTOR_MODE_PARAM_ID ||
                     mode == MOTOR_MODE_IF_STARTUP ||
                     mode == MOTOR_MODE_HFI_STARTUP);
}

static uint8_t Motor_ModeCanStartExternally(MotorControlMode mode)
{
    /* SENSOR 无位置源，仍禁止外部直接启动；I/F/HFI 允许命令进入骨架。 */
    return (uint8_t)(mode == MOTOR_MODE_SENSORLESS ||
                     mode == MOTOR_MODE_VF ||
                     mode == MOTOR_MODE_PREPOS ||
                     mode == MOTOR_MODE_PARAM_ID ||
                     mode == MOTOR_MODE_IF_STARTUP ||
                     mode == MOTOR_MODE_HFI_STARTUP);
}

static uint8_t Motor_ModeIsClosedLoop(MotorControlMode mode)
{
    /* MODE=2 的语义是“直接无感闭环”，不隐式拼接 VF/I/F/HFI 启动。 */
    return (uint8_t)(mode == MOTOR_MODE_SENSORLESS);
}

static uint8_t Motor_ModeIsVfFamily(MotorControlMode mode)
{
    return (uint8_t)(mode == MOTOR_MODE_VF || mode == MOTOR_MODE_PREPOS);
}

static uint8_t Motor_ModeIsStartup(MotorControlMode mode)
{
    return (uint8_t)(mode == MOTOR_MODE_IF_STARTUP ||
                     mode == MOTOR_MODE_HFI_STARTUP);
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

static void Motor_PrepareClosedLoop(MotorControlMode previous_mode)
{
    if (!Motor_ModeIsClosedLoop(previous_mode)) {
        Motor_ResetClosedLoopRuntime();
        SpeedRpm_GXieLv.XieLv_Y = motor.SpeedRPM;
    }
}

static void Motor_PrepareVfFamily(MotorControlMode previous_mode)
{
    if (!Motor_ModeIsVfFamily(previous_mode)) {
        VF_HzRamp.XieLv_Y = motor.CurrentHz;
        VF_HzRamp.XieLv_X = motor.TargetHz;

        if (Motor_ModeIsClosedLoop(previous_mode)) {
            motor.OpenTheta = Foc_observer.Theta;
        }
    }
}

static void Motor_PreparePreposition(void)
{
    float pll_step = 2.0f * PI * motor.CurrentHz * Foc_observer.Ctrl_ts;

    Foc_observer.Theta = motor.OpenTheta;
    Foc_observer.PLL_Ui = pll_step;
    Foc_observer.PLL_Interg = pll_step;
    Foc_observer.PLL_Err = 0.0f;
}

static uint8_t Motor_EnterMode(MotorControlMode next_mode)
{
    MotorControlMode previous_mode = motor_sm.active_mode;

    if (next_mode == previous_mode) {
        return 1U;
    }

    if (next_mode == MOTOR_MODE_STOP) {
        Motor_ControlStop();
        return 1U;
    }

    if (!Motor_ModeIsSupported(next_mode)) {
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

    if (Motor_ModeIsClosedLoop(next_mode)) {
        Motor_PrepareClosedLoop(previous_mode);
    }

    if (Motor_ModeIsStartup(next_mode)) {
        /* 启动模式从干净的 PI/给定状态开始；具体启动子状态尚未实现。 */
        Motor_ResetClosedLoopRuntime();
        SpeedRpm_GXieLv.XieLv_Y = 0.0f;
    }

    if (Motor_ModeIsVfFamily(next_mode)) {
        Motor_PrepareVfFamily(previous_mode);
    }

    if (next_mode == MOTOR_MODE_PREPOS) {
        Motor_PreparePreposition();
    } else if (next_mode == MOTOR_MODE_PARAM_ID) {
        /* PWM已以零占空比安全开启，辨识状态机从定位阶段开始接管输出。 */
        ParamId_Start();
    }

    motor_sm.active_mode = next_mode;
    return 1U;
}

static void SpeedLimitPi(void)
{
    /* 仅零目标禁止速度 PI 产生 Iq；正、负非零目标均进入速度环。 */
    if (fabsf(SpeedRpm_GXieLv.XieLv_X) <= 0.01f) {
        PI_ResetRuntime(&pi_spd);
        SpeedRpm_GXieLv.XieLv_Y = 0.0f;
        SpeedRpm_GXieLv.Timer_Count = 0U;
        I_q_GXieLv.XieLv_X = 0.0f;
        I_q_GXieLv.XieLv_Y = 0.0f;
        I_q_GXieLv.Timer_Count = 0U;
    } else {
        Speed_FOC();
    }
}

static void Motor_RunSensorless(void)
{
    /* MODE=2：只执行观测器→Park→速度/电流环→SVPWM，不调用 VF 或启动辅助。 */
    Angel_Get();
    UVW_Axis_DQ();
    SpeedLimitPi();
    Idq_FOC();
    FOC_Svpwm_dq();
}

static void Motor_RunIfStartup(void)
{
    /*
     * I/F 启动状态机入口（算法待实现）。
     * 预期流程：ALIGN/I-F 频率斜坡→观测器锁定检查→平滑切入 MODE=2。
     * 当前不能输出试验波形，先封波，避免把未实现模式误当作安全启动。
     */
    Motor_ControlStop();
}

static void Motor_RunHfiStartup(void)
{
    /*
     * HFI 启动状态机入口（算法待实现）。
     * 预期流程：高频注入→解调凸极位置→定位/闭环接管→退出注入。
     * 当前 IPD_HFI_Injection() 仍为空壳，因此必须安全停机。
     */
    Motor_ControlStop();
}

static void Motor_RunPreposition(void)
{
    /* MODE=4：当前为预定位/VF 试验。 */

}

static void Motor_RunActiveMode(void)
{
    switch (motor_sm.active_mode) {
        case MOTOR_MODE_STOP:
            /* 停机态：不执行控制算法；PWM 已由 Motor_ControlStop() 封锁。 */
            break;

        case MOTOR_MODE_SENSORLESS:
            /* 无感直接闭环。 */
            Motor_RunSensorless();
            break;

        case MOTOR_MODE_VF:
            /* V/F 开环。 */
            VF_Control_Run(&motor);
            break;

        case MOTOR_MODE_PREPOS:
            /* 预定位模式。打12个方向脉冲确定dq位置 要点凸极性 */
            Motor_RunPreposition();
            break;

        case MOTOR_MODE_PARAM_ID:
            /* 参数辨识：完成/失败后可以置位。 */
            ParamId_Run();
            break;

        case MOTOR_MODE_IF_STARTUP:
            /* I/F 启动：当前仅保留入口，算法未实现，运行函数会安全停机。 */
            Motor_RunIfStartup();
            break;

        case MOTOR_MODE_HFI_STARTUP:
            /* HFI 启动：当前仅保留入口，注入/解调算法未实现，安全停机。 */
            Motor_RunHfiStartup();
            break;

        case MOTOR_MODE_SENSOR:
            /* 有感模式：角度源尚未接入，不能运行功率级。 */
            Motor_ControlStop();
            break;

        default:
            /* 未知模式：按故障封波。 */
            Motor_ControlStop();
            break;
    }
}

/**
 * PMSM 初始化
 * 上电时执行电流零点校准
 */
void PMSM_init(void)
{
    ZeroCurrOffset_Calibration();
    motor.Control_Mode = MOTOR_MODE_STOP;
    motor_sm.active_mode = MOTOR_MODE_STOP;
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
#if HyperSpeedDataProcessEn
    /* 每个电流环周期保存一组三相电流；此处只写缓冲区，不发送 USB。 */
    HyperSpeedData_Capture(Volt_CurrPara.PhaseU_Curr,
                           Volt_CurrPara.PhaseV_Curr,
                           Volt_CurrPara.PhaseW_Curr);
#endif
    Foc_Para_Calc();
    Foc_Control();
}

/**
 * FOC 控制状态机：保护检查 -> 模式切换 -> 输出状态确认 -> 当前模式运行
 */
void Foc_Control(void)
{
    MotorControlMode requested_mode;

    InvProtect_Check();
    requested_mode = (MotorControlMode)motor.Control_Mode;

    if (requested_mode != motor_sm.active_mode &&
        Motor_EnterMode(requested_mode) == 0U) {
        return;
    }

    if (motor_sm.active_mode != MOTOR_MODE_STOP &&
        Foc_Pwm_IsEnabled() == 0U) {
        /* 运行中丢失 MOE 时不自动恢复，必须重新下发启动命令。 */
        Motor_ControlStop();
        return;
    }

    Motor_RunActiveMode();
}

/**
 * @brief  接受一条运行命令并安全开启三相 PWM
 * @retval 1=已接受，0=参数非法或仍有故障
 */
uint8_t Motor_ControlStart(uint8_t mode, float target_speed_rpm)
{
    MotorControlMode requested_mode = (MotorControlMode)mode;
    float target_hz;

    /* 使用最新采样值复查保护，避免故障清除与再次启动之间的窗口。 */
    InvProtect_Check();

    /* SENSOR 尚未接入角度源，禁止通过外部命令使能功率级。 */
    if (!Motor_ModeCanStartExternally(requested_mode) ||
        !isfinite(target_speed_rpm) || target_speed_rpm < 0.0f ||
        InvProtect_GetFault() != FAULT_NONE) {
        return 0U;
    }

    /* 参数辨识必须静止执行，启动帧的目标转速必须为 0。 */
    if (requested_mode == MOTOR_MODE_PARAM_ID && target_speed_rpm != 0.0f) {
        return 0U;
    }

    target_hz = target_speed_rpm * (float)MOTOR_POLE_PAIRS / 60.0f;
    if (!isfinite(target_hz) || target_hz > VF_FREQ_MAX) {
        return 0U;
    }

    if (Motor_ModeIsVfFamily(requested_mode) &&
        (!isfinite(motor.TargetVolt) || motor.TargetVolt <= VF_VOLTAGE_MIN ||
         motor.TargetVolt > PWM_GetVoltageLimitPu())) {
        return 0U;
    }

    if (motor.Control_Mode == MOTOR_MODE_STOP) {
        Motor_ResetStoppedRuntime();
    }

    SpeedRpm_GXieLv.XieLv_X = target_speed_rpm;
    motor.TargetHz = target_hz;
    /* 最后写模式，确保 20 kHz 控制中断只看到完整的新命令。 */
    motor.Control_Mode = (uint8_t)requested_mode;
    return 1U;
}

/**
 * @brief  停止控制并立即封锁三相 PWM
 */
void Motor_ControlStop(void)
{
    /* 先发布 STOP 状态，再立即封波；随后清控制器历史量。 */
    motor.Control_Mode = MOTOR_MODE_STOP;
    motor_sm.active_mode = MOTOR_MODE_STOP;
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

void CAN_App_OnNMT(uint8_t cmd)
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

void CAN_App_OnPDO1_RX(PDO1_RX_Data *data)
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

void CAN_App_OnPDO2_RX(PDO2_RX_Data *data)
{
    if (data == NULL) return;
    // Id/Iq 目标值，后续接入电流环
    (void)data;
}
