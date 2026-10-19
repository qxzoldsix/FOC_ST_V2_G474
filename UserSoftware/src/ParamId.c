/**
 * @file    ParamId.c
 * @brief   Rs/Ld/Lq 静止辨识状态机
 *
 * 流程：定位 -> Rs双平台差分 -> Ld多脉冲 -> Lq多脉冲 -> 校验。
 * 结果只写入 pid 候选值，不自动覆盖观测器参数。
 */
#include "headline.h"
#include "ParamId.h"

/* 所有时间参数都基于ADC/FOC的20kHz调用周期，禁止在低频任务中调用。 */
#define ID_CONTROL_TS_S              5.0e-5f
#define ID_ALIGN_TICKS               3000U
#define ID_RS_SETTLE_TICKS           1000U
#define ID_RS_AVERAGE_SAMPLES        512U
#define ID_L_SAMPLE_TICKS            6U
#define ID_L_PULSE_COUNT             32U
#define ID_L_MIN_VALID_SAMPLES       16U

/* 注入幅值刻意保持较低；首次换电机时仍必须使用限流电源。 */
#define ID_ALIGN_VOLT_PU             0.060f
#define ID_RS_LOW_VOLT_PU            0.040f
#define ID_RS_HIGH_VOLT_PU           0.080f
#define ID_LD_BIAS_VOLT_PU           0.060f
#define ID_LD_STEP_VOLT_PU           0.040f
#define ID_LQ_STEP_VOLT_PU           0.040f

#define ID_BUS_MIN_V                 8.0f
#define ID_BUS_MAX_V                 30.0f
#define ID_RS_MIN_OHM                0.02f
#define ID_RS_MAX_OHM                20.0f
#define ID_L_MIN_H                   5.0e-6f
#define ID_L_MAX_H                   5.0e-2f
#define ID_L_RATIO_MAX               10.0f

typedef enum {
    ID_AXIS_D = 0U,
    ID_AXIS_Q = 1U
} ParamIdAxis;

typedef struct {
    float current_sum;
    float voltage_sum;
    float rs_low_current;
    float rs_low_voltage;
    float inductance_sum;
    float pulse_i0;
    float pulse_voltage;
    float vd_command;
    float vq_command;
    uint16_t sample_count;
    uint16_t pulse_count;
    uint16_t valid_count;
    int8_t pulse_sign;
} ParamIdRuntime;

ParamId_t pid;
static ParamIdRuntime id_rt;

static void ParamId_SetOutput(float vd, float vq)
{
    /* 辨识期间不使用观测器角度，统一把测试坐标系固定在电角度0。 */
    motor.IQAngle = 0;
    motor.V_d = vd;
    motor.V_q = vq;
    FOC_Svpwm_dq();
}

static float ParamId_AppliedAxisVoltage(ParamIdAxis axis)
{
    float duty_u = (float)Svpwm_dq.SVPTa / (float)(TIM1_ARR + 1U);
    float duty_v = (float)Svpwm_dq.SVPTb / (float)(TIM1_ARR + 1U);
    float duty_w = (float)Svpwm_dq.SVPTc / (float)(TIM1_ARR + 1U);
    float voltage_alpha;
    float voltage_beta;

    /*
     * 不能用 command_pu * Vbus/2 估算电压：当前SVPWM在alpha、beta轴的
     * 命令增益不同。这里直接由实际CCR占空比反算平均相电压，再做Clarke
     * 变换；公共模分量会在变换中自动抵消。
     */
    voltage_alpha = 0.666666667f * Volt_CurrPara.BUS_Voltage *
                    (duty_u - 0.5f * duty_v - 0.5f * duty_w);
    voltage_beta = 0.577350269f * Volt_CurrPara.BUS_Voltage *
                   (duty_v - duty_w);

    return (axis == ID_AXIS_D) ? voltage_alpha : voltage_beta;
}

static void ParamId_EnterStep(ParamIdStep step)
{
    pid.step = (uint8_t)step;
    pid.tick = 0U;
}

static void ParamId_ResetAccumulator(void)
{
    id_rt.current_sum = 0.0f;
    id_rt.voltage_sum = 0.0f;
    id_rt.sample_count = 0U;
}

static void ParamId_StopOutput(void)
{
    motor.V_d = 0.0f;
    motor.V_q = 0.0f;
    Motor_ControlStop();
}

static void ParamId_Fail(ParamIdError error)
{
    /* 任一步骤异常都锁定结果为无效，并立即走统一停机/封波路径。 */
    pid.error = (uint8_t)error;
    pid.valid = 0U;
    pid.done = 1U;
    pid.step = ID_STEP_FAILED;
    ParamId_StopOutput();
}

static void ParamId_Complete(void)
{
    pid.error = ID_ERROR_NONE;
    pid.valid = 1U;
    pid.done = 1U;
    pid.step = ID_STEP_DONE;
    ParamId_StopOutput();
}

static uint8_t ParamId_CheckOperatingPoint(void)
{
    /* 母线异常时电压换算无意义，禁止继续累积错误样本。 */
    if (!isfinite(Volt_CurrPara.BUS_Voltage) ||
        Volt_CurrPara.BUS_Voltage < ID_BUS_MIN_V ||
        Volt_CurrPara.BUS_Voltage > ID_BUS_MAX_V) {
        ParamId_Fail(ID_ERROR_BUS_VOLTAGE);
        return 0U;
    }

    /* 参数辨识内部的 2A 门限暂时停用；系统原有硬件/软件保护仍独立生效。 */
    return 1U;
}

static float ParamId_AxisCurrent(ParamIdAxis axis)
{
    return (axis == ID_AXIS_D) ? PARK_PCurr.Ds : PARK_PCurr.Qs;
}

static void ParamId_StartPulse(ParamIdAxis axis)
{
    float command;

    id_rt.pulse_sign = (int8_t)-id_rt.pulse_sign;
    command = (float)id_rt.pulse_sign;

    if (axis == ID_AXIS_D) {
        /* d轴在正偏置上下跳变，保持转子定位，避免负d轴退磁。 */
        id_rt.vd_command = ID_LD_BIAS_VOLT_PU + command * ID_LD_STEP_VOLT_PU;
        id_rt.vq_command = 0.0f;
    } else {
        /* q轴脉冲正负交替，d轴持续定位，尽量抵消净转矩和机械位移。 */
        id_rt.vd_command = ID_ALIGN_VOLT_PU;
        id_rt.vq_command = command * ID_LQ_STEP_VOLT_PU;
    }

    id_rt.pulse_i0 = ParamId_AxisCurrent(axis);
    pid.tick = 0U;
    ParamId_SetOutput(id_rt.vd_command, id_rt.vq_command);
    id_rt.pulse_voltage = ParamId_AppliedAxisVoltage(axis);
}

static uint8_t ParamId_EstimateInductance(ParamIdAxis axis, float *inductance)
{
    float i0 = id_rt.pulse_i0;
    float i1 = ParamId_AxisCurrent(axis);
    float i_inf;
    float ratio;
    float estimate;
    float elapsed = (float)ID_L_SAMPLE_TICKS * ID_CONTROL_TS_S;

    if (inductance == NULL || pid.Rs <= 0.0f) {
        return 0U;
    }

    /*
     * 静止RL响应：i(t)=i_inf+(i0-i_inf)*exp(-Rs*t/L)，其中
     * i_inf=V/R。整理得到 L=-Rs*t/ln((i1-i_inf)/(i0-i_inf))。
     * 相比单点V/(di/dt)，该公式包含Rs压降且对采样噪声更稳健。
     */
    i_inf = id_rt.pulse_voltage / pid.Rs;
    if (fabsf(i0 - i_inf) < 0.02f) {
        return 0U;
    }

    ratio = (i1 - i_inf) / (i0 - i_inf);
    if (!isfinite(ratio) || ratio <= 0.02f || ratio >= 0.98f) {
        return 0U;
    }

    estimate = -pid.Rs * elapsed / logf(ratio);
    if (!isfinite(estimate) || estimate < ID_L_MIN_H || estimate > ID_L_MAX_H) {
        return 0U;
    }

    *inductance = estimate;
    return 1U;
}

static void ParamId_RunAlign(void)
{
    ParamId_SetOutput(ID_ALIGN_VOLT_PU, 0.0f);
    if (pid.tick >= ID_ALIGN_TICKS) {
        ParamId_ResetAccumulator();
        ParamId_EnterStep(ID_STEP_RS_LOW_SETTLE);
    }
}

static void ParamId_RunRsSettle(float command_pu, ParamIdStep next_step)
{
    ParamId_SetOutput(command_pu, 0.0f);
    if (pid.tick >= ID_RS_SETTLE_TICKS) {
        ParamId_ResetAccumulator();
        ParamId_EnterStep(next_step);
    }
}

static uint8_t ParamId_CollectRsSamples(float command_pu)
{
    ParamId_SetOutput(command_pu, 0.0f);
    id_rt.current_sum += PARK_PCurr.Ds;
    id_rt.voltage_sum += ParamId_AppliedAxisVoltage(ID_AXIS_D);
    id_rt.sample_count++;
    return (uint8_t)(id_rt.sample_count >= ID_RS_AVERAGE_SAMPLES);
}

static void ParamId_RunRsLowSample(void)
{
    if (!ParamId_CollectRsSamples(ID_RS_LOW_VOLT_PU)) {
        return;
    }

    id_rt.rs_low_current = id_rt.current_sum / (float)id_rt.sample_count;
    id_rt.rs_low_voltage = id_rt.voltage_sum / (float)id_rt.sample_count;
    ParamId_EnterStep(ID_STEP_RS_HIGH_SETTLE);
}

static void ParamId_RunRsHighSample(void)
{
    float high_current;
    float high_voltage;
    float delta_current;

    if (!ParamId_CollectRsSamples(ID_RS_HIGH_VOLT_PU)) {
        return;
    }

    high_current = id_rt.current_sum / (float)id_rt.sample_count;
    high_voltage = id_rt.voltage_sum / (float)id_rt.sample_count;
    delta_current = high_current - id_rt.rs_low_current;

    if (!isfinite(delta_current) || fabsf(delta_current) < 0.05f) {
        ParamId_Fail(ID_ERROR_RS_RESPONSE);
        return;
    }

    /* Rs=ΔV/ΔI；两个平台电流同向，可抵消大部分零偏和固定死区压降。 */
    pid.Rs = fabsf((high_voltage - id_rt.rs_low_voltage) / delta_current);
    if (!isfinite(pid.Rs) || pid.Rs < ID_RS_MIN_OHM || pid.Rs > ID_RS_MAX_OHM) {
        ParamId_Fail(ID_ERROR_RS_RANGE);
        return;
    }

    id_rt.inductance_sum = 0.0f;
    id_rt.pulse_count = 0U;
    id_rt.valid_count = 0U;
    id_rt.pulse_sign = -1;
    ParamId_EnterStep(ID_STEP_LD_PULSE);
    ParamId_StartPulse(ID_AXIS_D);
}

static void ParamId_RunInductance(ParamIdAxis axis)
{
    float estimate;

    ParamId_SetOutput(id_rt.vd_command, id_rt.vq_command);
    if (pid.tick < ID_L_SAMPLE_TICKS) {
        return;
    }

    /* 单个脉冲不可信：只累计通过响应/范围检查的样本，最后取平均。 */
    if (ParamId_EstimateInductance(axis, &estimate)) {
        id_rt.inductance_sum += estimate;
        id_rt.valid_count++;
    }
    id_rt.pulse_count++;

    if (id_rt.pulse_count < ID_L_PULSE_COUNT) {
        ParamId_StartPulse(axis);
        return;
    }

    if (id_rt.valid_count < ID_L_MIN_VALID_SAMPLES) {
        ParamId_Fail((axis == ID_AXIS_D) ? ID_ERROR_LD_RESPONSE :
                                           ID_ERROR_LQ_RESPONSE);
        return;
    }

    pid.valid_samples = id_rt.valid_count;
    if (axis == ID_AXIS_D) {
        pid.Ld = id_rt.inductance_sum / (float)id_rt.valid_count;
        id_rt.inductance_sum = 0.0f;
        id_rt.pulse_count = 0U;
        id_rt.valid_count = 0U;
        id_rt.pulse_sign = -1;
        ParamId_EnterStep(ID_STEP_LQ_PULSE);
        ParamId_StartPulse(ID_AXIS_Q);
    } else {
        pid.Lq = id_rt.inductance_sum / (float)id_rt.valid_count;
        ParamId_EnterStep(ID_STEP_VALIDATE);
    }
}

static void ParamId_Validate(void)
{
    float ratio;

    ParamId_SetOutput(0.0f, 0.0f);
    if (!isfinite(pid.Ld) || !isfinite(pid.Lq) ||
        pid.Ld < ID_L_MIN_H || pid.Ld > ID_L_MAX_H ||
        pid.Lq < ID_L_MIN_H || pid.Lq > ID_L_MAX_H) {
        ParamId_Fail(ID_ERROR_L_RANGE);
        return;
    }

    /* 防止某一轴虽落在绝对范围内，但相对另一轴已明显失真。 */
    ratio = pid.Ld / pid.Lq;
    if (!isfinite(ratio) || ratio < (1.0f / ID_L_RATIO_MAX) ||
        ratio > ID_L_RATIO_MAX) {
        ParamId_Fail(ID_ERROR_L_RANGE);
        return;
    }

    ParamId_Complete();
}

void ParamId_Start(void)
{
    /* 每次启动彻底清除上次候选值和内部累加器，禁止混用旧样本。 */
    memset(&pid, 0, sizeof(pid));
    memset(&id_rt, 0, sizeof(id_rt));
    id_rt.pulse_sign = -1;
    ParamId_EnterStep(ID_STEP_ALIGN);
}

void ParamId_Run(void)
{
    if (pid.done != 0U) {
        return;
    }

    UVW_Axis_DQ();
    pid.current_a = PARK_PCurr.Ds;
    pid.voltage_v = ParamId_AppliedAxisVoltage(ID_AXIS_D);

    if (!ParamId_CheckOperatingPoint()) {
        return;
    }

    pid.tick++;
    /* 状态函数只处理本步骤；所有跳转统一通过ParamId_EnterStep复位计时。 */
    switch ((ParamIdStep)pid.step) {
        case ID_STEP_ALIGN:
            ParamId_RunAlign();
            break;

        case ID_STEP_RS_LOW_SETTLE:
            ParamId_RunRsSettle(ID_RS_LOW_VOLT_PU, ID_STEP_RS_LOW_SAMPLE);
            break;

        case ID_STEP_RS_LOW_SAMPLE:
            ParamId_RunRsLowSample();
            break;

        case ID_STEP_RS_HIGH_SETTLE:
            ParamId_RunRsSettle(ID_RS_HIGH_VOLT_PU, ID_STEP_RS_HIGH_SAMPLE);
            break;

        case ID_STEP_RS_HIGH_SAMPLE:
            ParamId_RunRsHighSample();
            break;

        case ID_STEP_LD_PULSE:
            ParamId_RunInductance(ID_AXIS_D);
            break;

        case ID_STEP_LQ_PULSE:
            ParamId_RunInductance(ID_AXIS_Q);
            break;

        case ID_STEP_VALIDATE:
            ParamId_Validate();
            break;

        case ID_STEP_IDLE:
        case ID_STEP_DONE:
        case ID_STEP_FAILED:
        default:
            ParamId_Fail(ID_ERROR_INVALID_STATE);
            break;
    }
}
