/**
 * @file    ParamId.c
 * @brief   静止电机的 Rs / Ld / Lq 候选参数辨识。
 *
 * 运行顺序：受限升压定位 → 三个稳态电流平台测 Rs → d 轴脉冲 → q 轴脉冲。
 * 本文件按“公共输出/保护、Rs、Ld/Lq、状态调度”排列；配置统一在 ParamId.h。
 *
 * 调用条件：ADC1 整组转换完成后，每个电流环周期调用一次 ParamIdRun()。
 * MODE=5 直接控制测试电压，不经过速度 PI 或电流 PI。
 * 电压命令为 pu，电压重构/拟合用 V，电流用 A，电阻用 Ω，电感用 H。
 *
 * 固定电角度与定位电流不能证明转子一定对齐；转子转动、ADC 饱和、采样错时
 * 或逆变器损耗变化都会影响结果。pid.valid 表示通过软件检查，结果仍需实测复核。
 */
#include "ParamId.h"

/* 以下编号仅用于本模块的数组索引，不是对外的 ParamIdStep 状态编号。 */
typedef enum {
    ID_RS_LOW = 0,
    ID_RS_MID,
    ID_RS_HIGH,
    ID_RS_PLATFORM_COUNT
} ParamIdRsPlatform;

typedef enum {
    ID_POSITIVE = 0,
    ID_NEGATIVE,
    ID_POLARITY_COUNT
} ParamIdPolarity;

typedef enum {
    ID_PULSE_REST = 0,   /* 回到中平台，等待电流恢复 */
    ID_PULSE_WAIT,       /* 已提交脉冲，等待 PWM 更新后再建立 i0 */
    ID_PULSE_SAMPLE      /* 在相同 PWM 相位记录后续电流点 */
} ParamIdPulsePhase;

typedef enum {
    ID_AXIS_D = 0,
    ID_AXIS_Q
} ParamIdAxis;

/* 一个 Rs 平台的临时累加值；平台采满后清零，不影响已保存的三平台结果。 */
typedef struct {
    float current_sum;          /* 电流之和，A */
    float q_abs_sum;            /* |Iq| 之和，避免正负波动抵消，也不只查最后一拍 */
    float current_square_sum;   /* 电流平方之和，用于检查平台波动 */
    float voltage_sum;          /* 重构电压之和，V */
    uint16_t count;
} ParamIdRsSamples;

typedef struct {
    float voltage_pu;                        /* 正在调节/保持的 d 轴电压命令 */
    uint32_t stable_ticks;                    /* 连续合格均值窗口折算的拍数 */
    float target_d_sum;                      /* 当前 1ms 调节窗口的 Id 累加值 */
    float target_q_abs_sum;                  /* 当前 1ms 窗口的 |Iq| 累加值 */
    ParamIdRsSamples samples;
    float current[ID_RS_PLATFORM_COUNT];      /* 低/中/高平台平均电流，A */
    float voltage[ID_RS_PLATFORM_COUNT];      /* 低/中/高平台平均电压，V */
    float command[ID_RS_PLATFORM_COUNT];      /* 对应电压命令，供后续脉冲复用 */
} ParamIdRsRuntime;

/* 正、负极性分别平均轨迹、分别统计，防止一侧异常被另一侧掩盖。 */
typedef struct {
    float wave_sum[ID_L_SAMPLE_TICKS + 1U]; /* 尚未凑满一组的电流轨迹之和 */
    uint16_t wave_count;                   /* 上面累加了几条轨迹 */
    uint16_t group_count;                  /* 通过拟合的平均组数 */
    float inductance_sum;                 /* 每个有效平均组的 L 之和，H */
    float inductance_square_sum;
} ParamIdPolarityStats;

typedef struct {
    ParamIdPulsePhase phase;
    ParamIdPolarity polarity;              /* 当前脉冲是正阶跃还是负阶跃 */
    uint16_t pulse_count;                  /* 本轴已完成的脉冲数，含无效脉冲 */
    int8_t phase_sign;                     /* i0 时三相电流的符号掩码；-1=接近零 */
    uint8_t invalid;                       /* 当前轨迹出现过零/母线变化等异常 */
    float current[ID_L_SAMPLE_TICKS + 1U];  /* i0..i8，共 9 点、8 个采样间隔 */
    float start_bus_v;                     /* 建立 i0 时的母线电压，V */
    ParamIdPolarityStats stats[ID_POLARITY_COUNT];
    float fit_error_sum;                   /* 有效平均组的归一化残差之和 */
} ParamIdInductanceRuntime;

typedef struct {
    uint32_t previous_cycle; /* DWT 上次读数，无符号相减允许计数器回绕 */
    uint8_t started;         /* 第一拍只建立时间基准，不计算周期 */
} ParamIdTiming;

typedef struct {
    ParamIdRsRuntime rs;
    ParamIdInductanceRuntime l;
    ParamIdTiming timing;
} ParamIdRuntime;

ParamId_t pid;
static ParamIdRuntime id_rt;

/* 公共输出、状态与采样检查 */

static void ParamId_SetOutput(float vd_pu, float vq_pu)
{
    motor.V_d = vd_pu;
    motor.V_q = vq_pu;
    FOC_Svpwm_dq();
}

static float ParamId_AxisCurrent(ParamIdAxis axis)
{
    return axis == ID_AXIS_D ? PARK_PCurr.Ds : PARK_PCurr.Qs;
}

/* 偏置选择集中在此处，保持恢复目标与两轴的输出平台一致。 */
static ParamIdRsPlatform ParamId_BiasPlatform(ParamIdAxis axis)
{
    return axis == ID_AXIS_D ? ID_RS_MID : ID_RS_HIGH;
}

/* 主步骤切换只管理状态及计时；采样累加器由使用它的 Rs/L 流程复位。 */
static void ParamId_EnterStep(ParamIdStep step)
{
    pid.step = (uint8_t)step;
    pid.tick = 0U;
}

/* 成功和失败统一封波，保留候选值及质量数据供 Watch/CAN 检查。 */
static void ParamId_End(ParamIdError error)
{
    pid.failed_step = error == ID_ERROR_NONE ? ID_STEP_IDLE : pid.step;
    pid.error = (uint8_t)error;
    pid.done = 1U;
    pid.valid = error == ID_ERROR_NONE;
    pid.step = pid.valid ? ID_STEP_DONE : ID_STEP_FAILED;
    MotorControlStop();
}

/**
 * 核对回调间隔，不能把软件的 20kHz 宏当作真实时序的证明。
 * 使用 ADC 整组回调入口的时间戳，首拍开桥/初始化不会改变取时位置。
 * DWT 测的是执行点间隔，存在中断调度抖动；它不是 ADC 的采样孔径时间。
 * 原值与极值保留在 pid 中；37.5..62.5us 门限由头文件按 Ts 的比例配置。
 */
static ParamIdError ParamId_CheckSamplePeriod(uint32_t cycle)
{
    ParamIdTiming *timing = &id_rt.timing;

    if (timing->started) {
        float period_us = (float)(uint32_t)(cycle - timing->previous_cycle) *
                          (1000000.0f / (float)SystemCoreClock);
        float expected_us = FOC_CONTROL_TS_S * 1000000.0f;

        pid.sample_period_us = period_us;
        if (pid.sample_period_min_us == 0.0f || period_us < pid.sample_period_min_us) {
            pid.sample_period_min_us = period_us;
        }
        if (period_us > pid.sample_period_max_us) {
            pid.sample_period_max_us = period_us;
        }
        if (period_us < expected_us * ID_TIMING_MIN_RATIO ||
            period_us > expected_us * ID_TIMING_MAX_RATIO) {
            return ID_ERROR_SAMPLE_TIMING;
        }
    }

    timing->previous_cycle = cycle;
    timing->started = 1U;
    return ID_ERROR_NONE;
}

/* 必须在施加下一拍电压前检查。ADC 削顶后，换算电流不能反映真实峰值。 */
static ParamIdError ParamId_CheckOperatingPoint(void)
{
    const float phase_current[3] = {
        Volt_CurrPara.PhaseU_Curr,
        Volt_CurrPara.PhaseV_Curr,
        Volt_CurrPara.PhaseW_Curr
    };
    const uint16_t raw_current[3] = {
        ADCSampPara.PhaseU_Raw,
        ADCSampPara.PhaseV_Raw,
        ADCSampPara.PhaseW_Raw
    };
    float bus_v = Volt_CurrPara.BUS_Voltage;

    if (!isfinite(bus_v) || bus_v < ID_BUS_MIN_V || bus_v > ID_BUS_MAX_V) {
        return ID_ERROR_BUS_VOLTAGE;
    }
    for (unsigned i = 0U; i < 3U; i++) {
        /* 当前 ADC 为 12 位，原始范围 0..4095；留出上下限裕量。 */
        if (raw_current[i] <= ID_ADC_RAIL_MARGIN ||
            raw_current[i] >= 4095U - ID_ADC_RAIL_MARGIN) {
            return ID_ERROR_ADC_SATURATION;
        }
        if (!isfinite(phase_current[i]) || fabsf(phase_current[i]) > ID_CURRENT_LIMIT_A) {
            return ID_ERROR_OVERCURRENT;
        }
    }
    return ID_ERROR_NONE;
}

/* 定位与 Rs：建立电流、采集稳态平台、拟合电阻 */

/**
 * 定位与三个 Rs 平台共用同一套受限升压逻辑。
 * 每 1ms 对 Id 与 |Iq| 取均值，再小步调压并检查稳定状态。
 * 调节死区为 ±0.02A，稳定验收为 Id 均值 ±0.08A、|Iq| 均值≤0.15A。
 * 较小的调节死区使电流靠近目标中心，避免贴边纹波反复打断判稳。
 * 连续合格的窗口累计到 hold_ticks 才进入下一步；保护仍逐拍检查原始电流。
 * 这是辨识用的缓慢电压调节，不使用正常运行的电流 PI。
 */
static void ParamId_RunCurrentTarget(float target_a, uint32_t hold_ticks, ParamIdStep next_step)
{
    ParamIdRsRuntime *rs = &id_rt.rs;
    if (pid.tick > ID_TARGET_TIMEOUT_TICKS) {
        ParamId_End(ID_ERROR_TARGET_CURRENT);
        return;
    }

    rs->target_d_sum += PARK_PCurr.Ds;
    rs->target_q_abs_sum += fabsf(PARK_PCurr.Qs);
    if (pid.tick % ID_ADJUST_TICKS == 0U) {
        float error_a = target_a - rs->target_d_sum / (float)ID_ADJUST_TICKS;
        float mean_abs_q_a = rs->target_q_abs_sum / (float)ID_ADJUST_TICKS;
        float voltage_limit_pu = fminf(ID_COMMAND_MAX_PU, PWM_GetVoltageLimitPu());

        /* 目标步骤只在完整窗口末切换，下一窗口从零累加，不需要额外计数。 */
        rs->target_d_sum = 0.0f;
        rs->target_q_abs_sum = 0.0f;
        if (fabsf(error_a) > ID_ADJUST_DEADBAND_A) {
            float increment_pu = Limit_Sat(error_a * ID_ADJUST_GAIN_PU_PER_A,
                                           ID_VOLT_STEP_MAX_PU, -ID_VOLT_STEP_MAX_PU);
            rs->voltage_pu = Limit_Sat(rs->voltage_pu + increment_pu, voltage_limit_pu, 0.0f);
        }
        if (fabsf(error_a) <= ID_TARGET_TOLERANCE_A && mean_abs_q_a <= ID_HOLD_Q_MAX_A) {
            rs->stable_ticks += ID_ADJUST_TICKS;
        } else {
            rs->stable_ticks = 0U;
        }
        if (rs->stable_ticks >= hold_ticks) {
            rs->stable_ticks = 0U;
            ParamId_EnterStep(next_step);
        }
    }
    ParamId_SetOutput(rs->voltage_pu, 0.0f);
}

/* 保持平台电压，积累均值与方差；成功保存后清空本平台的临时累加器。 */
static uint8_t ParamId_CollectRs(ParamIdRsPlatform platform)
{
    ParamIdRsSamples *samples = &id_rt.rs.samples;
    float current_a = PARK_PCurr.Ds;

    samples->current_sum += current_a;
    samples->q_abs_sum += fabsf(PARK_PCurr.Qs);
    samples->current_square_sum += current_a * current_a;
    /* voltage_v 已在本拍写 PWM 前保存，与稳态电流对应。 */
    samples->voltage_sum += pid.voltage_v;
    ParamId_SetOutput(id_rt.rs.voltage_pu, 0.0f);
    if (++samples->count < ID_RS_AVERAGE_SAMPLES) {
        return 0U;
    }

    float mean_a = samples->current_sum / (float)samples->count;
    float variance_a2 = samples->current_square_sum / (float)samples->count - mean_a * mean_a;
    if (sqrtf(fmaxf(variance_a2, 0.0f)) > ID_RS_MAX_RIPPLE_A ||
        samples->q_abs_sum / (float)samples->count > ID_HOLD_Q_MAX_A) {
        ParamId_End(ID_ERROR_RS_FIT);
        return 0U;
    }

    id_rt.rs.current[platform] = mean_a;
    id_rt.rs.voltage[platform] = samples->voltage_sum / (float)samples->count;
    id_rt.rs.command[platform] = id_rt.rs.voltage_pu;
    memset(samples, 0, sizeof(*samples));
    return 1U;
}

/**
 * 三个平台拟合 V = Rs*I + V0。
 * Rs = Σ[(I-I均值)(V-V均值)] / Σ[(I-I均值)^2]，V0 为拟合截距。
 * 截距可吸收近似固定的桥压降，但斜率仍可能包含导通电阻、线阻。
 * 电流跨度不足、负斜率或残差过大均拒绝；不能对负斜率取绝对值。
 */
static uint8_t ParamId_FitResistance(void)
{
    ParamIdRsRuntime *rs = &id_rt.rs;
    float mean_a = 0.0f;
    float mean_v = 0.0f;
    float current_variance = 0.0f;
    float current_voltage_covariance = 0.0f;
    float residual_sum = 0.0f;
    float low_current_span_a = rs->current[ID_RS_MID] - rs->current[ID_RS_LOW];
    float high_current_span_a = rs->current[ID_RS_HIGH] - rs->current[ID_RS_MID];

    if (rs->current[ID_RS_HIGH] - rs->current[ID_RS_LOW] < ID_RS_MIN_CURRENT_SPAN_A ||
        low_current_span_a <= 0.0f || high_current_span_a <= 0.0f) {
        ParamId_End(ID_ERROR_RS_RESPONSE);
        return 0U;
    }
    for (unsigned i = 0U; i < ID_RS_PLATFORM_COUNT; i++) {
        mean_a += rs->current[i];
        mean_v += rs->voltage[i];
    }
    mean_a /= (float)ID_RS_PLATFORM_COUNT;
    mean_v /= (float)ID_RS_PLATFORM_COUNT;

    for (unsigned i = 0U; i < ID_RS_PLATFORM_COUNT; i++) {
        float delta_a = rs->current[i] - mean_a;
        current_variance += delta_a * delta_a;
        current_voltage_covariance += delta_a * (rs->voltage[i] - mean_v);
    }
    pid.Rs = current_voltage_covariance / current_variance;
    if (!isfinite(pid.Rs) || pid.Rs < ID_RS_MIN_OHM || pid.Rs > ID_RS_MAX_OHM) {
        ParamId_End(ID_ERROR_RS_RANGE);
        return 0U;
    }
    pid.voltage_offset_v = mean_v - pid.Rs * mean_a;

    for (unsigned i = 0U; i < ID_RS_PLATFORM_COUNT; i++) {
        float error_v = rs->voltage[i] - (pid.Rs * rs->current[i] + pid.voltage_offset_v);
        residual_sum += error_v * error_v;
    }
    float voltage_span_v = fabsf(rs->voltage[ID_RS_HIGH] - rs->voltage[ID_RS_LOW]);
    pid.rs_fit_error = sqrtf(residual_sum / (float)ID_RS_PLATFORM_COUNT) /
                       fmaxf(voltage_span_v, 0.001f);
    if (pid.rs_fit_error > ID_RS_MAX_FIT_ERROR) {
        ParamId_End(ID_ERROR_RS_FIT);
        return 0U;
    }

    /* 用分段 V-I 斜率近似该区间的有效阻尼，而不是纯绕组电阻。
     * Ld 上升段处于中→高平台区间，下降段处于低→中平台区间。
     * 这里只修正换算所用的增量电阻；pid.Rs 仍保留原三点拟合结果。
     * 固定 PWM 相位的电流不等于周期均值，候选仍需端电压/波形实测复核。
     */
    pid.Rs_low = (rs->voltage[ID_RS_MID] - rs->voltage[ID_RS_LOW]) / low_current_span_a;
    pid.Rs_high = (rs->voltage[ID_RS_HIGH] - rs->voltage[ID_RS_MID]) / high_current_span_a;
    if (!isfinite(pid.Rs_low) || !isfinite(pid.Rs_high) ||
        pid.Rs_low <= 0.0f || pid.Rs_high <= 0.0f) {
        ParamId_End(ID_ERROR_RS_RANGE);
        return 0U;
    }
    return 1U;
}

/* 电感脉冲准备与采样 */

/* 每个脉冲之前恢复偏置：d 轴回中平台，q 轴回高平台。 */
static void ParamId_RestPulse(ParamIdAxis axis)
{
    id_rt.l.phase = ID_PULSE_REST;
    pid.tick = 0U;
    ParamId_SetOutput(id_rt.rs.command[ParamId_BiasPlatform(axis)], 0.0f);
}

static void ParamId_BeginAxis(ParamIdAxis axis)
{
    /* 只清本轴数据，保留三个 Rs 平台以及连续采样计时。 */
    memset(&id_rt.l, 0, sizeof(id_rt.l));
    memset(pid.l_positive_current, 0, sizeof(pid.l_positive_current));
    memset(pid.l_negative_current, 0, sizeof(pid.l_negative_current));
    pid.l_positive_h = 0.0f;
    pid.l_negative_h = 0.0f;
    pid.valid_samples = 0U;
    pid.l_bias_current_a = id_rt.rs.current[ParamId_BiasPlatform(axis)];
    id_rt.l.polarity = ID_NEGATIVE; /* StartPulse 切换极性，第一次为正脉冲。 */
    ParamId_EnterStep(axis == ID_AXIS_D ? ID_STEP_LD_PULSE : ID_STEP_LQ_PULSE);
    ParamId_RestPulse(axis);
}

static void ParamId_StartPulse(ParamIdAxis axis)
{
    ParamIdInductanceRuntime *pulse = &id_rt.l;
    pulse->polarity = pulse->polarity == ID_POSITIVE ? ID_NEGATIVE : ID_POSITIVE;

    if (axis == ID_AXIS_D) {
        /* d 轴在已测低/高电流平台之间切换，避免另设未经验证的大注入电压。 */
        ParamIdRsPlatform platform = pulse->polarity == ID_POSITIVE ? ID_RS_HIGH : ID_RS_LOW;
        ParamId_SetOutput(id_rt.rs.command[platform], 0.0f);
    } else {
        float step_pu = ID_L_Q_STEP_FRACTION *
                        (id_rt.rs.command[ID_RS_HIGH] - id_rt.rs.command[ID_RS_LOW]);
        float direction = pulse->polarity == ID_POSITIVE ? 1.0f : -1.0f;
        /* 增强 q 激励时同时提高 Id 偏置，避免一相在负阶跃时接近零或换向。 */
        ParamId_SetOutput(id_rt.rs.command[ParamId_BiasPlatform(axis)], direction * step_pu);
    }
    pulse->phase = ID_PULSE_WAIT;
    pulse->invalid = 0U;
    pid.tick = 0U;
}

/* 纯读取：bit0/1/2 分别表示 U/V/W 为正；任一相接近零则返回 -1。
 * 相电流换向时桥压降可能改变，不能继续按恒定截距模型拟合。
 */
static int8_t ParamId_ReadPhaseSigns(void)
{
    const float phase_current[3] = {
        Volt_CurrPara.PhaseU_Curr,
        Volt_CurrPara.PhaseV_Curr,
        Volt_CurrPara.PhaseW_Curr
    };
    int8_t signs = 0;

    for (unsigned i = 0U; i < 3U; i++) {
        if (fabsf(phase_current[i]) < ID_PHASE_SIGN_MIN_A) {
            return -1;
        }
        if (phase_current[i] > 0.0f) {
            signs |= (int8_t)(1U << i);
        }
    }
    return signs;
}

/* 等待两完整拍之后才取 i0；排除写入 CCR 到首次生效之间的混合区间。
 * 采样阶段用 pid.tick=1..8 作为数组索引，不再另设重复的采样计数器。
 */
static void ParamId_BeginPulseSampling(ParamIdAxis axis)
{
    ParamIdInductanceRuntime *pulse = &id_rt.l;
    pulse->current[0] = ParamId_AxisCurrent(axis);
    pulse->start_bus_v = Volt_CurrPara.BUS_Voltage;
    pulse->phase_sign = ParamId_ReadPhaseSigns();
    pulse->invalid = pulse->phase_sign < 0;
    pulse->phase = ID_PULSE_SAMPLE;
    pid.tick = 0U;
}

/* 每次记录一个同相位电流点；任一拍异常会使整条轨迹无效。
 * 只判无效，不在这里修改输出；脉冲结束后统一决定继续或结束该轴。
 */
static uint8_t ParamId_CollectPulse(ParamIdAxis axis)
{
    ParamIdInductanceRuntime *pulse = &id_rt.l;
    int8_t signs = ParamId_ReadPhaseSigns();
    float bus_change_v = fabsf(Volt_CurrPara.BUS_Voltage - pulse->start_bus_v);

    if (signs < 0 || signs != pulse->phase_sign ||
        bus_change_v > pulse->start_bus_v * ID_L_MAX_BUS_CHANGE) {
        pulse->invalid = 1U;
    }
    pulse->current[pid.tick] = ParamId_AxisCurrent(axis);
    return pid.tick == ID_L_SAMPLE_TICKS;
}

/* 电感估计与平均组统计 */

/**
 * 对 N+1 个电流点拟合 i[k+1] = a*i[k] + b，输入不依赖硬件或运行状态。
 * 对静止、恒压、固定采样间隔的局部 RL 响应，有 a=exp(-R*Ts/L)。
 * R 为对应电流区间的增量电阻。先求 a 和自由截距 b，再算 L=-R*Ts/ln(a)。
 *
 * 自由截距吸收近似恒定的电压偏差，不强制使用理想的 V/R 渐近电流。
 * 残差为一步预测 RMS / 整条电流响应幅度，避免小信号的绝对误差看似很小。
 * 调用方负责排除过零/母线变化轨迹；本函数只检查数学可辨识性和拟合质量。
 */
static uint8_t ParamId_FitInductance(const float *current, float resistance_ohm,
                                    float *inductance_h, float *fit_error)
{
    float mean_current = 0.0f;
    float mean_next_current = 0.0f;
    float current_variance = 0.0f;
    float adjacent_covariance = 0.0f;
    float residual_sum = 0.0f;
    float minimum_a = current[0];
    float maximum_a = current[0];

    for (unsigned k = 0U; k < ID_L_SAMPLE_TICKS; k++) {
        mean_current += current[k];
        mean_next_current += current[k + 1U];
        minimum_a = fminf(minimum_a, current[k + 1U]);
        maximum_a = fmaxf(maximum_a, current[k + 1U]);
    }
    float response_a = maximum_a - minimum_a;
    if (response_a < ID_L_MIN_RESPONSE_A) {
        return 0U;
    }
    mean_current /= (float)ID_L_SAMPLE_TICKS;
    mean_next_current /= (float)ID_L_SAMPLE_TICKS;

    for (unsigned k = 0U; k < ID_L_SAMPLE_TICKS; k++) {
        float delta_a = current[k] - mean_current;
        current_variance += delta_a * delta_a;
        adjacent_covariance += delta_a * (current[k + 1U] - mean_next_current);
    }
    if (current_variance < 1.0e-8f) {
        return 0U;
    }
    float decay = adjacent_covariance / current_variance;
    if (!(decay > 0.0f && decay < 1.0f)) {
        return 0U; /* 偏离稳态值的分量应衰减；a 不在 (0,1) 时不满足稳定 RL 模型。 */
    }
    float offset_a = mean_next_current - decay * mean_current;
    float estimate_h = -resistance_ohm * FOC_CONTROL_TS_S / logf(decay);
    if (!isfinite(estimate_h) || estimate_h < ID_L_MIN_H || estimate_h > ID_L_MAX_H) {
        return 0U;
    }

    for (unsigned k = 0U; k < ID_L_SAMPLE_TICKS; k++) {
        float error_a = current[k + 1U] - (decay * current[k] + offset_a);
        residual_sum += error_a * error_a;
    }
    *fit_error = sqrtf(residual_sum / (float)ID_L_SAMPLE_TICKS) / response_a;
    if (*fit_error > ID_L_MAX_FIT_ERROR) {
        return 0U;
    }
    *inductance_h = estimate_h;
    return 1U;
}

/**
 * 只接收完整有效的轨迹。正负各凑满四条后逐点平均，再拟合一次。
 * 统计以“有效平均组”为单位，不再反复乘四、除四；对外的有效轨迹数
 * 在保存结果时由组数换算。轴结束时不足四条的尾组不参加拟合。
 */
static void ParamId_AccumulatePulse(ParamIdAxis axis)
{
    ParamIdInductanceRuntime *pulse = &id_rt.l;
    ParamIdPolarityStats *stats = &pulse->stats[pulse->polarity];
    float inductance_h;
    float fit_error;
    float resistance_ohm = pid.Rs;

    /* q 轴未单独标定 Vq-Iq 斜率，沿用整体 Rs，不套用 d 轴分段值。 */
    if (axis == ID_AXIS_D) {
        resistance_ohm = pulse->polarity == ID_POSITIVE ? pid.Rs_high : pid.Rs_low;
    }

    if (pulse->invalid) {
        return;
    }
    for (unsigned k = 0U; k <= ID_L_SAMPLE_TICKS; k++) {
        stats->wave_sum[k] += pulse->current[k];
    }
    if (++stats->wave_count < ID_L_AVERAGE_PULSES) {
        return;
    }
    for (unsigned k = 0U; k <= ID_L_SAMPLE_TICKS; k++) {
        pulse->current[k] = stats->wave_sum[k] / (float)ID_L_AVERAGE_PULSES;
        stats->wave_sum[k] = 0.0f;
    }
    stats->wave_count = 0U;

    if (ParamId_FitInductance(pulse->current, resistance_ohm, &inductance_h, &fit_error)) {
        float *saved_current = pulse->polarity == ID_POSITIVE ?
                               pid.l_positive_current : pid.l_negative_current;
        /* 每个有效平均组只保存一次，便于直接从 pid 对比两侧曲线。 */
        memcpy(saved_current, pulse->current, sizeof(pulse->current));
        stats->group_count++;
        stats->inductance_sum += inductance_h;
        stats->inductance_square_sum += inductance_h * inductance_h;
        pulse->fit_error_sum += fit_error;
    }
}

/* 正负均值用于检查两种阶跃是否一致；CV 是平均组之间的离散度，
 * 不是四条原始轨迹各自的离散度。即使 CV 或极性检查失败，也保留已计算的诊断。
 */
static uint8_t ParamId_FinishAxis(ParamIdAxis axis)
{
    ParamIdInductanceRuntime *pulse = &id_rt.l;
    const ParamIdPolarityStats *positive = &pulse->stats[ID_POSITIVE];
    const ParamIdPolarityStats *negative = &pulse->stats[ID_NEGATIVE];
    unsigned groups = positive->group_count + negative->group_count;
    unsigned valid_samples = groups * ID_L_AVERAGE_PULSES;

    /* 数量不足也发布本轴实际统计，避免失败时仍显示上一轴的样本数。 */
    pid.valid_samples = (uint16_t)valid_samples;
    if (axis == ID_AXIS_D) {
        pid.ld_valid_samples = (uint16_t)valid_samples;
        pid.ld_fit_groups = (uint16_t)groups;
    } else {
        pid.lq_valid_samples = (uint16_t)valid_samples;
        pid.lq_fit_groups = (uint16_t)groups;
    }
    if (positive->group_count * ID_L_AVERAGE_PULSES < ID_L_MIN_VALID_PER_SIGN ||
        negative->group_count * ID_L_AVERAGE_PULSES < ID_L_MIN_VALID_PER_SIGN) {
        ParamId_End(axis == ID_AXIS_D ? ID_ERROR_LD_RESPONSE : ID_ERROR_LQ_RESPONSE);
        return 0U;
    }
    float mean_h = (positive->inductance_sum + negative->inductance_sum) / (float)groups;
    float variance_h2 = (positive->inductance_square_sum + negative->inductance_square_sum) /
                        (float)groups - mean_h * mean_h;
    float cv = sqrtf(fmaxf(variance_h2, 0.0f)) / mean_h;
    float positive_mean_h = positive->inductance_sum / positive->group_count;
    float negative_mean_h = negative->inductance_sum / negative->group_count;
    float polarity_error = fabsf(positive_mean_h - negative_mean_h) / mean_h;
    float fit_error = pulse->fit_error_sum / (float)groups;

    pid.l_positive_h = positive_mean_h;
    pid.l_negative_h = negative_mean_h;
    if (axis == ID_AXIS_D) {
        pid.Ld = mean_h;
        pid.ld_cv = cv;
        pid.ld_polarity_error = polarity_error;
        pid.ld_fit_error = fit_error;
    } else {
        pid.Lq = mean_h;
        pid.lq_cv = cv;
        pid.lq_polarity_error = polarity_error;
        pid.lq_fit_error = fit_error;
    }
    if (cv > ID_L_MAX_CV || polarity_error > ID_L_MAX_POLARITY_ERROR) {
        ParamId_End(ID_ERROR_L_QUALITY);
        return 0U;
    }
    return 1U;
}

/* 这里只调度脉冲的三个阶段，采样、平均和质量判定分别由专用函数处理。 */
static void ParamId_RunInductance(ParamIdAxis axis)
{
    ParamIdRsPlatform bias_platform = ParamId_BiasPlatform(axis);

    switch (id_rt.l.phase) {
        case ID_PULSE_REST:
            if (pid.tick > ID_L_REST_TIMEOUT_TICKS) {
                ParamId_End(ID_ERROR_TARGET_CURRENT);
            } else if (pid.tick >= ID_L_REST_TICKS &&
                       fabsf(PARK_PCurr.Ds - id_rt.rs.current[bias_platform]) < ID_TARGET_TOLERANCE_A &&
                       fabsf(PARK_PCurr.Qs) < ID_HOLD_Q_MAX_A) {
                ParamId_StartPulse(axis);
            }
            break;

        case ID_PULSE_WAIT:
            if (pid.tick >= ID_PWM_SETTLE_TICKS) {
                ParamId_BeginPulseSampling(axis);
            }
            break;

        case ID_PULSE_SAMPLE:
            if (!ParamId_CollectPulse(axis)) {
                break;
            }
            ParamId_AccumulatePulse(axis);
            if (++id_rt.l.pulse_count < ID_L_PULSE_COUNT) {
                ParamId_RestPulse(axis);
            } else if (ParamId_FinishAxis(axis)) {
                if (axis == ID_AXIS_D) {
                    ParamId_BeginAxis(ID_AXIS_Q);
                } else {
                    ParamId_EnterStep(ID_STEP_VALIDATE);
                }
            }
            break;

        default:
            ParamId_End(ID_ERROR_INVALID_STATE);
            break;
    }
}

/* Rs 平台采满后的处理：低/中平台继续下一档，高平台完成 Rs 拟合后进入 Ld。 */
static void ParamId_RunRsSample(ParamIdRsPlatform platform)
{
    if (!ParamId_CollectRs(platform)) {
        return;
    }
    if (platform == ID_RS_HIGH) {
        if (ParamId_FitResistance()) {
            ParamId_BeginAxis(ID_AXIS_D);
        }
    } else {
        ParamId_EnterStep(platform == ID_RS_LOW ? ID_STEP_RS_MID_SETTLE : ID_STEP_RS_HIGH_SETTLE);
    }
}

static void ParamId_ValidateResults(void)
{
    float ratio = pid.Ld / pid.Lq;
    if (ratio < 1.0f / ID_L_RATIO_MAX || ratio > ID_L_RATIO_MAX) {
        ParamId_End(ID_ERROR_L_RANGE);
    } else {
        ParamId_End(ID_ERROR_NONE);
    }
}

/* 对外入口与主状态调度 */

/**
 * 每次启动清除全部旧候选和统计，输出零矢量，再进入受限电流定位。
 * DWT 已在 PWM/ADC 时基初始化时开启；此处不清零共享周期计数。
 */
void ParamIdStart(void)
{
    memset(&pid, 0, sizeof(pid));
    memset(&id_rt, 0, sizeof(id_rt));
    motor.IQAngle = 0;
    ParamId_SetOutput(0.0f, 0.0f);
    ParamId_EnterStep(ID_STEP_ALIGN);
}

/**
 * 一次调用处理一拍：固定坐标系 → 保存反馈 → 检查 → 推进当前步骤。
 * 三平台均值、电感拟合、停止与错误处理均在各自函数内完成。
 * pid.tick 在主步骤切换时清零；电感阶段在 REST/WAIT/SAMPLE 切换时清零。
 */
void ParamIdRun(uint32_t sample_cycle)
{
    ParamIdError error;
    float alpha_v;
    float beta_v;

    if (pid.done) {
        return;
    }
    motor.IQAngle = 0;
    UVW_Axis_DQ();
    PWM_ReconstructVoltage(&alpha_v, &beta_v);
    pid.current_a = PARK_PCurr.Ds;
    pid.voltage_v = alpha_v;

    error = ParamId_CheckSamplePeriod(sample_cycle);
    if (error == ID_ERROR_NONE) {
        error = ParamId_CheckOperatingPoint();
    }
    if (error != ID_ERROR_NONE) {
        ParamId_End(error);
        return;
    }

    pid.tick++;
    switch ((ParamIdStep)pid.step) {
        case ID_STEP_ALIGN:
            ParamId_RunCurrentTarget(ID_ALIGN_CURRENT_A, ID_ALIGN_HOLD_TICKS, ID_STEP_RS_LOW_SETTLE);
            break;
        case ID_STEP_RS_LOW_SETTLE:
            ParamId_RunCurrentTarget(ID_RS_LOW_CURRENT_A, ID_RS_SETTLE_TICKS, ID_STEP_RS_LOW_SAMPLE);
            break;
        case ID_STEP_RS_LOW_SAMPLE:
            ParamId_RunRsSample(ID_RS_LOW);
            break;
        case ID_STEP_RS_MID_SETTLE:
            ParamId_RunCurrentTarget(ID_RS_MID_CURRENT_A, ID_RS_SETTLE_TICKS, ID_STEP_RS_MID_SAMPLE);
            break;
        case ID_STEP_RS_MID_SAMPLE:
            ParamId_RunRsSample(ID_RS_MID);
            break;
        case ID_STEP_RS_HIGH_SETTLE:
            ParamId_RunCurrentTarget(ID_RS_HIGH_CURRENT_A, ID_RS_SETTLE_TICKS, ID_STEP_RS_HIGH_SAMPLE);
            break;
        case ID_STEP_RS_HIGH_SAMPLE:
            ParamId_RunRsSample(ID_RS_HIGH);
            break;
        case ID_STEP_LD_PULSE:
            ParamId_RunInductance(ID_AXIS_D);
            break;
        case ID_STEP_LQ_PULSE:
            ParamId_RunInductance(ID_AXIS_Q);
            break;
        case ID_STEP_VALIDATE:
            ParamId_ValidateResults();
            break;
        default:
            ParamId_End(ID_ERROR_INVALID_STATE);
            break;
    }
}
