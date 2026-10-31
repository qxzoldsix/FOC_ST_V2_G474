/**
 * @file paramIdFlux.c
 * @brief 辨识专用：限流空载拖转，再封波测量反电动势与磁链。
 *
 * 拖转只用强制角度和已测 R/L，不用磁链观测器，也不把指令频率当实测转速。
 * 封波后，ADC2 常规组 [1]/[2]/[3] 按原工程映射读取 VA/VB/VC。
 * 每个完整电周期对 max(VA,VB,VC)-min(VA,VB,VC) 积分取平均，
 * 乘 pi/3 转为等效正弦线间峰值，再与同周期平均电角速度配对。
 * 拟合 V_LL_equivalent_peak = slope*omega_e + offset，Flux=slope/sqrt(3)。
 * 截距吸收近似固定的二极管/采样偏差；强整流、谐波或接线错误仍会影响结果。
 */
#include "headline.h"
#include "bsp.h"
#include "paramIdFlux.h"
#include <string.h>

/* 拟合失败标志，保持原日志位值。 */
enum {
    FLUX_FIT_FREQ_SPAN_LOW    = 0x01U, /* 频率跨度不足 */
    FLUX_FIT_VOLTAGE_SPAN_LOW = 0x02U, /* 电压跨度不足 */
    FLUX_FIT_NO_VARIATION     = 0x04U, /* 频率数据退化 */
    FLUX_FIT_INVALID_SLOPE    = 0x08U, /* 斜率无效 */
    FLUX_FIT_RESIDUAL_HIGH    = 0x10U, /* 拟合残差过大 */
    FLUX_FIT_SLOPE_ERROR_HIGH = 0x20U  /* 斜率误差过大 */
};

typedef struct {
    float angle;                  /* 强制电角度，rad */
    float integralD, integralQ;    /* 独立试验 PI 积分，V，不改正常运行 PI */
    float kpD, kpQ, ki;
    uint8_t crossingArmed;         /* beta 电压低于负迟滞后，等待上穿正迟滞 */
    uint8_t cycleInvalid;          /* 当前周期的拒绝原因位，与 pid.bemf_reject_flags 一致 */
    uint8_t coastFinished;         /* 幅值跌出强信号窗口后结束，避免尾段噪声重新进入 */
    uint32_t previousCrossing;     /* 滑行阶段拍数；0 表示尚未建立周期起点 */
    uint32_t settleTicks;          /* 退磁等待：至少1ms，且不少于已测电气时间常数的5倍 */
    float cyclePeak;              /* 当前电周期的线间峰值，V */
    float cycleLineIntegral;      /* 梯形积分，单位 V*采样拍；除周期拍数得到周期均值 */
    float previousLine;           /* 上一拍线间包络，V */
    float maxCyclePeak;           /* 已接受周期中的最大原始峰值，仅用于强信号窗口 */
    float betaMin, betaMax;        /* 当前周期差分电压范围，避免通道直流偏差冒充有效幅值 */
    float meanOmega, meanVoltage;  /* 以下为逐周期直线拟合的均值/中心积 */
    float omegaSquare, omegaVoltage, voltageSquare;
    float minOmega, maxOmega, minVoltage, maxVoltage;
} FluxTestRuntime;

static FluxTestRuntime fluxTest;

void ParamIdFluxStart(float biasVoltagePu)
{
    memset(&fluxTest, 0, sizeof(fluxTest));
    for (unsigned i = 0U; i < 3U; i++) pid.bemf_raw_min[i] = 4095U;
    /* 200Hz 电流调节带宽；积分与比例使用同一 V/A、A 单位。
     * 候选 R/L 已通过静止检查；所有相电流仍由父流程逐拍检查 2A 上限。
     */
    float bandwidth = TWO_PI * 200.0f;
    fluxTest.kpD = pid.Ld * bandwidth;
    fluxTest.kpQ = pid.Lq * bandwidth;
    fluxTest.ki = pid.Rs * bandwidth * FOC_CONTROL_TS_S;
    fluxTest.integralD = biasVoltagePu * CONTROL_VOLTAGE_BASE_V;
    fluxTest.settleTicks = (uint32_t)fmaxf((float)ID_MS_TICKS(1U),
        ceilf(5.0f * fmaxf(pid.Ld, pid.Lq) / pid.Rs * (float)FOC_CONTROL_FREQ_HZ));
    pid.step = ID_STEP_FLUX_SPIN;
    pid.tick = 0U;
}

/* 2s 匀速升频、保持1s。强制旋转坐标系 Id=1A、Iq=0 是 I/F 拖转，
 * 不是实际转子坐标系；转子是否跟随，最终只能由封波后电压波形确认。
 */
static void FluxSpinRun(void)
{
    float fraction = fminf((float)pid.tick / (float)ID_MS_TICKS(2000U), 1.0f);
    float frequency = ID_FLUX_TEST_HZ * fraction;
    fluxTest.angle += TWO_PI * frequency * FOC_CONTROL_TS_S;
    if (fluxTest.angle >= TWO_PI) fluxTest.angle -= TWO_PI;
    motor.IQAngle = (int16_t)(fluxTest.angle * RAD_TO_ANGLE);
    UVW_Axis_DQ();

    float errorD = ID_RS_MID_CURRENT_A - PARK_PCurr.Ds;
    float errorQ = -PARK_PCurr.Qs;
    fluxTest.integralD += fluxTest.ki * errorD;
    fluxTest.integralQ += fluxTest.ki * errorQ;
    float vd = fluxTest.kpD * errorD + fluxTest.integralD;
    float vq = fluxTest.kpQ * errorQ + fluxTest.integralQ;
    float magnitude = sqrtf(vd * vd + vq * vq);
    float limit = PWM_GetVoltageLimitPu() * CONTROL_VOLTAGE_BASE_V;
    if (magnitude > limit) {
        float scale = limit / magnitude;
        vd *= scale;
        vq *= scale;
        /* 电压圆饱和后回算积分，避免拖转失步时继续累积电压。 */
        fluxTest.integralD = vd - fluxTest.kpD * errorD;
        fluxTest.integralQ = vq - fluxTest.kpQ * errorQ;
    }
    motor.V_d = vd / CONTROL_VOLTAGE_BASE_V;
    motor.V_q = vq / CONTROL_VOLTAGE_BASE_V;
    pid.current_a = PARK_PCurr.Ds;
    pid.voltage_v = vd;
    FOC_Svpwm_dq();

    if (pid.tick >= ID_MS_TICKS(3000U)) {
        PwmStop();                /* 全桥关闭，CH4 与 ADC 继续运行 */
        motor.V_d = 0.0f;
        motor.V_q = 0.0f;
        pid.step = ID_STEP_FLUX_COAST;
        pid.tick = 0U;
    }
}

/* 每个完整有效电周期加入一次直线拟合，中心积避免大数相减损失精度。 */
static void FluxRecordCycle(float frequency, float voltage)
{
    float omega = TWO_PI * frequency;
    unsigned count = ++pid.flux_samples;
    if (count <= ID_FLUX_TRACE_CAPACITY) {
        pid.bemf_trace_hz[count - 1U] = frequency;
        pid.bemf_trace_v[count - 1U] = voltage;
        pid.bemf_trace_peak_v[count - 1U] = fluxTest.cyclePeak;
    }
    if (count == 1U) {
        fluxTest.minOmega = fluxTest.maxOmega = omega;
        fluxTest.minVoltage = fluxTest.maxVoltage = voltage;
    }
    float deltaOmega = omega - fluxTest.meanOmega;
    float deltaVoltage = voltage - fluxTest.meanVoltage;
    fluxTest.meanOmega += deltaOmega / (float)count;
    fluxTest.meanVoltage += deltaVoltage / (float)count;
    fluxTest.omegaSquare += deltaOmega * (omega - fluxTest.meanOmega);
    fluxTest.omegaVoltage += deltaOmega * (voltage - fluxTest.meanVoltage);
    fluxTest.voltageSquare += deltaVoltage * (voltage - fluxTest.meanVoltage);
    fluxTest.minOmega = fminf(fluxTest.minOmega, omega);
    fluxTest.maxOmega = fmaxf(fluxTest.maxOmega, omega);
    fluxTest.minVoltage = fminf(fluxTest.minVoltage, voltage);
    fluxTest.maxVoltage = fmaxf(fluxTest.maxVoltage, voltage);
    pid.bemf_frequency_min_hz = fluxTest.minOmega / TWO_PI;
    pid.bemf_frequency_max_hz = fluxTest.maxOmega / TWO_PI;
    pid.bemf_voltage_span_v = fluxTest.maxVoltage - fluxTest.minVoltage;
    pid.bemf_frequency_hz = frequency;
    pid.bemf_speed_rpm = frequency * 60.0f / (float)MOTOR_POLE_PAIRS;
    pid.bemf_voltage_v = voltage;
}

/* 拟合电压与电角速度，保存候选参数并检查质量。 */
static ParamIdError FluxFitResults(void)
{
    if (pid.flux_samples < 5U) return ID_ERROR_BEMF_RESPONSE;

    /* 数据检查：频率跨度、电压跨度和拟合分母。 */
    float sampleCount = (float)pid.flux_samples;
    float voltageSpan = fluxTest.maxVoltage - fluxTest.minVoltage;
    if (fluxTest.maxOmega - fluxTest.minOmega < 0.2f * fluxTest.maxOmega) {
        pid.flux_fit_flags |= FLUX_FIT_FREQ_SPAN_LOW;
    }
    if (voltageSpan < 8.0f * VOLTAGE_RATIO) {
        pid.flux_fit_flags |= FLUX_FIT_VOLTAGE_SPAN_LOW;
    }
    if (!isfinite(fluxTest.omegaSquare) || fluxTest.omegaSquare <= 0.0f) {
        pid.flux_fit_flags |= FLUX_FIT_NO_VARIATION;
        return ID_ERROR_FLUX_FIT;
    }

    /* 直线拟合：电压 = slope × 电角速度 + offset。 */
    float slope = fluxTest.omegaVoltage / fluxTest.omegaSquare;
    float residualSum = fmaxf(fluxTest.voltageSquare - slope * fluxTest.omegaVoltage, 0.0f);
    pid.bemf_offset_v = fluxTest.meanVoltage - slope * fluxTest.meanOmega;
    if (voltageSpan > 0.0f) {
        pid.flux_fit_error = sqrtf(residualSum / sampleCount) / voltageSpan;
    }
    if (!isfinite(pid.flux_fit_error) || pid.flux_fit_error > ID_RS_MAX_FIT_ERROR) {
        pid.flux_fit_flags |= FLUX_FIT_RESIDUAL_HIGH;
    }
    if (!isfinite(slope) || slope <= 0.0f) {
        pid.flux_fit_flags |= FLUX_FIT_INVALID_SLOPE;
        return ID_ERROR_FLUX_FIT;
    }

    /* 保存候选；是否合格由残差和斜率标准误差决定。 */
    pid.Flux = slope * INV_SQRT3;
    pid.Ke = slope * (float)MOTOR_POLE_PAIRS * TWO_PI * (1000.0f / 60.0f);
    float slopeStdError = sqrtf(residualSum / (sampleCount - 2.0f) / fluxTest.omegaSquare);
    pid.flux_slope_relative_error = slopeStdError / slope;
    if (!isfinite(slopeStdError) || slopeStdError > 0.05f * slope) {
        pid.flux_fit_flags |= FLUX_FIT_SLOPE_ERROR_HIGH;
    }
    if (pid.flux_fit_flags) return ID_ERROR_FLUX_FIT;

    pid.flux_valid = 1U;
    return ID_ERROR_NONE;
}

/* 完整周期的输入质量，与最终直线拟合质量分别验收。
 * 端电压已很弱时，固定电压偏差/钳位/过零误差的相对影响增大。
 * 只使用不低于已测最大有效峰值一半的连续前段；首次跌出即结束，
 * 不在事后挑选能拟合成功的点，也不允许尾段偶发过零重新加入。
 */
static uint8_t FluxCycleRejectReasons(uint32_t period)
{
    uint8_t reasons = fluxTest.cycleInvalid;
    if (period < FOC_CONTROL_FREQ_HZ / 1000U) reasons |= 8U;
    if (fluxTest.cyclePeak < 16.0f * VOLTAGE_RATIO ||
        fluxTest.betaMax - fluxTest.betaMin < 8.0f * VOLTAGE_RATIO) {
        reasons |= 4U;
    }
    if (pid.flux_samples != 0U && fluxTest.cyclePeak < 0.5f * fluxTest.maxCyclePeak) {
        reasons |= 16U;
        fluxTest.coastFinished = 1U;
    }
    return reasons;
}

/* 端电压经单极性分压和钳位，允许接近0V；上限削顶或桥二极管整流时拒绝。
 * beta=(VB-VC)/sqrt(3) 的迟滞上穿每电周期一次，与旋转方向无关。
 * 按已测 L/R 等待退磁，再建立第一个周期起点，首个不完整周期不用于拟合。
 */
static void FluxCoastCapture(void)
{
    if (pid.tick <= fluxTest.settleTicks) return;
    uint16_t raw[3] = {adc2_buff[1], adc2_buff[2], adc2_buff[3]};
    float va = (float)raw[0] * VOLTAGE_RATIO;
    float vb = (float)raw[1] * VOLTAGE_RATIO;
    float vc = (float)raw[2] * VOLTAGE_RATIO;
    float line = fmaxf(va, fmaxf(vb, vc)) - fminf(va, fminf(vb, vc));
    float beta = (vb - vc) * INV_SQRT3;
    float hysteresis = 2.0f * VOLTAGE_RATIO;
    for (unsigned i = 0U; i < 3U; i++) {
        if (raw[i] < pid.bemf_raw_min[i]) pid.bemf_raw_min[i] = raw[i];
        if (raw[i] > pid.bemf_raw_max[i]) pid.bemf_raw_max[i] = raw[i];
    }
    pid.bemf_peak_v = fmaxf(pid.bemf_peak_v, line);
    fluxTest.cycleLineIntegral += 0.5f * (fluxTest.previousLine + line);
    fluxTest.previousLine = line;
    fluxTest.cyclePeak = fmaxf(fluxTest.cyclePeak, line);
    fluxTest.betaMin = fminf(fluxTest.betaMin, beta);
    fluxTest.betaMax = fmaxf(fluxTest.betaMax, beta);
    if (raw[0] >= 4095U - ID_ADC_RAIL_MARGIN ||
        raw[1] >= 4095U - ID_ADC_RAIL_MARGIN ||
        raw[2] >= 4095U - ID_ADC_RAIL_MARGIN) {
        fluxTest.cycleInvalid |= 1U;
    }
    if (line >= 0.9f * Volt_CurrPara.BUS_Voltage) {
        fluxTest.cycleInvalid |= 2U;
    }
    /* 即使没有过零也发布削顶原因，避免“零周期”掩盖通道异常。 */
    pid.bemf_reject_flags |= fluxTest.cycleInvalid;
    if (beta < -hysteresis) fluxTest.crossingArmed = 1U;
    if (beta > hysteresis && fluxTest.crossingArmed) {
        uint32_t period = pid.tick - fluxTest.previousCrossing;
        pid.bemf_crossings++;
        /* 最高接受1kHz。线间峰值至少16个ADC刻度（约0.206V），
         * 同时要求周期内beta摆幅至少8个刻度；不能只凭通道间直流差放行。
         * 周期数、速度跨度及拟合残差继续独立验收。
         */
        if (fluxTest.previousCrossing != 0U) {
            uint8_t reasons = FluxCycleRejectReasons(period);
            if (reasons) {
                pid.bemf_rejected_cycles++;
                pid.bemf_reject_flags |= reasons;
            } else {
                float equivalentPeak = fluxTest.cycleLineIntegral / (float)period * (TWO_PI / 6.0f);
                fluxTest.maxCyclePeak = fmaxf(fluxTest.maxCyclePeak, fluxTest.cyclePeak);
                FluxRecordCycle((float)FOC_CONTROL_FREQ_HZ / (float)period, equivalentPeak);
            }
        }
        fluxTest.previousCrossing = pid.tick;
        fluxTest.crossingArmed = 0U;
        fluxTest.cyclePeak = 0.0f;
        fluxTest.cycleLineIntegral = 0.0f;
        fluxTest.betaMin = fluxTest.betaMax = beta;
        fluxTest.cycleInvalid = 0U;
    }
}

uint8_t ParamIdFluxRun(void)
{
    if (pid.step == ID_STEP_FLUX_SPIN) {
        FluxSpinRun();
        return 0U;
    }
    FluxCoastCapture();
    if (!fluxTest.coastFinished && pid.tick < ID_MS_TICKS(2000U)) return 0U;
    pid.error = (uint8_t)FluxFitResults();
    return 1U;
}
