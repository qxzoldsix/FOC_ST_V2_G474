/**
 * @file    ParamId.h
 * @brief   静止电机 Rs/Ld/Lq 辨识配置、状态和候选结果
 */
#ifndef __PARAMID_H_
#define __PARAMID_H_
#include "headline.h"

/* 时间基准：以下毫秒配置转换成回调拍数，前提是完整 ADC 组频率正确。
 * 20kHz 时 1 拍=50us；模式5需连续运行，调试暂停可能触发周期检查。
 */
#define ID_MS_TICKS(ms)              ((FOC_CONTROL_FREQ_HZ * (ms)) / 1000U)
#define ID_ALIGN_HOLD_TICKS          ID_MS_TICKS(150U)  /* 定位电流连续稳定时间 */
#define ID_TARGET_TIMEOUT_TICKS      ID_MS_TICKS(2000U) /* 达不到目标电流的最大等待时间 */
#define ID_ADJUST_TICKS              ID_MS_TICKS(1U)    /* 定位/Rs 均值窗口及调压间隔 */
#define ID_RS_SETTLE_TICKS           ID_MS_TICKS(50U)   /* 每个 Rs 平台连续稳定时间 */
#define ID_RS_AVERAGE_SAMPLES        512U              /* 每个平台的平均点数 */

/* 定位与三个 Rs 电流平台。电压从零小步调节，稳定后冻结命令采样。
 * 同向平台用于拟合斜率与截距；并不证明转子在机械负载下已可靠定位。
 */
#define ID_ALIGN_CURRENT_A          1.0f
#define ID_RS_LOW_CURRENT_A         0.5f
#define ID_RS_MID_CURRENT_A         1.0f
#define ID_RS_HIGH_CURRENT_A        1.5f
#define ID_TARGET_TOLERANCE_A        0.08f    /* 定位/Rs 时 1ms 平均 Id 的验收范围 */
#define ID_HOLD_Q_MAX_A             0.15f    /* 定位/Rs 的 |Iq| 均值上限；L恢复仍查瞬时值 */
#define ID_ADJUST_DEADBAND_A          0.02f    /* 调节死区小于验收范围，避免电流停在边缘 */
#define ID_ADJUST_GAIN_PU_PER_A      0.002f   /* 电流误差换算成小步电压修正 */
#define ID_VOLT_STEP_MAX_PU          0.0005f  /* 每次电压修正的最大幅值 */
#define ID_COMMAND_MAX_PU            0.45f    /* 同时受母线可用电压限制 */

/* Ld/Lq 脉冲：先恢复偏置，再提交阶跃、等更新、取 i0、采后续点。
 * Ld 用中平台偏置；Lq 用高平台偏置，给较大的 q 阶跃留出相电流符号裕量。
 * 正负各16次；同极性4次逐点平均后拟合，默认每轴最多8个拟合组。
 */
#define ID_L_REST_TICKS              ID_MS_TICKS(10U)  /* 脉冲前偏置平台最短恢复时间 */
#define ID_L_REST_TIMEOUT_TICKS      ID_MS_TICKS(100U) /* 偏置平台无法恢复时终止 */
#define ID_PWM_SETTLE_TICKS          2U               /* 排除命令首次生效的混合区间 */
#define ID_L_SAMPLE_TICKS            8U               /* 8个间隔，共9点；20kHz时400us */
#define ID_L_PULSE_COUNT             32U
#define ID_L_AVERAGE_PULSES          4U
#define ID_L_MIN_VALID_PER_SIGN      12U              /* 每极性至少3个平均组 */
#define ID_L_Q_STEP_FRACTION          0.6f             /* 配合高平台Id偏置，提高q阶跃的有效响应 */

/* 辨识独立软件保护；削顶后换算电流已不可信，不能只检查安培值。 */
#define ID_CURRENT_LIMIT_A          2.0f     /* 任一相的采样电流上限 */
#define ID_ADC_RAIL_MARGIN           16U      /* 12位ADC上下限保留的原始count裕量 */
#define ID_BUS_MIN_V                 8.0f
#define ID_BUS_MAX_V                 30.0f
#define ID_TIMING_MIN_RATIO          0.75f    /* 回调间隔下限/Ts，非ADC孔径时间 */
#define ID_TIMING_MAX_RATIO          1.25f    /* 回调间隔上限/Ts */

/* 初始质量门限，用于拒绝不可信数据，不代表绝对测量精度。 */
#define ID_RS_MIN_OHM                0.02f
#define ID_RS_MAX_OHM                20.0f
#define ID_RS_MIN_CURRENT_SPAN_A     0.75f    /* 高低平台至少相差的电流 */
#define ID_RS_MAX_RIPPLE_A           0.05f    /* Rs平台电流标准差上限 */
#define ID_RS_MAX_FIT_ERROR          0.05f    /* 电压拟合RMS残差/高低平台电压差 */
#define ID_L_MIN_H                   5.0e-6f
#define ID_L_MAX_H                   5.0e-2f
#define ID_L_RATIO_MAX               10.0f    /* Ld/Lq允许范围为1/此值..此值 */
#define ID_L_MIN_RESPONSE_A          0.15f    /* 平均轨迹的最大电流减最小电流 */
#define ID_L_MAX_FIT_ERROR           0.05f    /* 一步预测RMS残差/轨迹电流幅度 */
#define ID_L_MAX_CV                  0.12f    /* 平均组之间的标准差/均值 */
#define ID_L_MAX_POLARITY_ERROR      0.15f    /* 正负均值差/总体均值 */
#define ID_L_MAX_BUS_CHANGE          0.02f    /* 单条轨迹内母线相对变化上限 */
#define ID_PHASE_SIGN_MIN_A          0.02f    /* 任一相接近零时拒绝该轨迹 */

/* 防止零除数、不完整平均组和不足的拟合点数；这些只在编译期检查。 */
#if ID_ADJUST_TICKS == 0 || ID_L_SAMPLE_TICKS < 4 || ID_L_AVERAGE_PULSES == 0 || \
    ID_L_PULSE_COUNT % (2U * ID_L_AVERAGE_PULSES) != 0 || \
    ID_L_MIN_VALID_PER_SIGN % ID_L_AVERAGE_PULSES != 0
#error Invalid_parameter_identification_configuration
#endif

typedef enum {
    ID_STEP_IDLE = 0U,           /* 未启动 */
    ID_STEP_ALIGN = 1U,          /* 受限升压定位 */
    ID_STEP_RS_LOW_SETTLE = 2U,  /* 低平台调节及稳定 */
    ID_STEP_RS_LOW_SAMPLE = 3U,  /* 低平台平均 */
    ID_STEP_RS_HIGH_SETTLE = 4U, /* 高平台调节及稳定 */
    ID_STEP_RS_HIGH_SAMPLE = 5U, /* 高平台平均及三点拟合 */
    ID_STEP_LD_PULSE = 6U,       /* d 轴偏置/等待生效/多点采样 */
    ID_STEP_LQ_PULSE = 7U,       /* q 轴偏置/等待生效/多点采样 */
    ID_STEP_VALIDATE = 8U,       /* 最终校验 */
    ID_STEP_DONE = 9U,           /* 已成功并封波 */
    ID_STEP_FAILED = 10U,        /* 已失败并封波 */
    ID_STEP_RS_MID_SETTLE = 11U, /* 保留原状态编号，新增中平台 */
    ID_STEP_RS_MID_SAMPLE = 12U
} ParamIdStep;

typedef enum {
    ID_ERROR_NONE = 0U,
    ID_ERROR_BUS_VOLTAGE,
    ID_ERROR_OVERCURRENT,
    ID_ERROR_RS_RESPONSE,
    ID_ERROR_RS_RANGE,
    ID_ERROR_LD_RESPONSE,
    ID_ERROR_LQ_RESPONSE,
    ID_ERROR_L_RANGE,
    ID_ERROR_INVALID_STATE,
    ID_ERROR_ADC_SATURATION,    /* ADC 原始电流值接近上下限 */
    ID_ERROR_SAMPLE_TIMING,     /* 回调周期异常或计时器未工作 */
    ID_ERROR_TARGET_CURRENT,    /* 规定时间内无法建立定位/测试电流 */
    ID_ERROR_RS_FIT,            /* 平台波动或三点拟合残差过大 */
    ID_ERROR_L_QUALITY          /* 电感离散度或正负脉冲差异过大 */
} ParamIdError;

typedef struct {
    uint8_t step;               /* 状态，SDO 0x1010 可读 */
    uint8_t error;
    uint8_t done;               /* 成功或失败结束时置 1 */
    uint8_t valid;              /* 通过全部检查才置 1，仍是候选参数 */
    uint32_t tick;              /* 当前步骤拍数；L 的 REST/WAIT/SAMPLE 各自从 0 计数 */
    uint16_t valid_samples;     /* 最近一个轴有效脉冲数 */
    float Rs;                  /* Ω，可能仍包含桥臂/线阻的影响 */
    float Ld;                  /* H */
    float Lq;                  /* H */
    float current_a;            /* 当前 Id，A */
    float voltage_v;            /* 前一拍 CCR 重构的 d 轴电压，V */
    float voltage_offset_v;     /* 三平台 V-I 拟合截距，V */
    float rs_fit_error;         /* 归一化拟合残差 */
    float ld_fit_error;         /* 有效平均组的平均归一化残差 */
    float lq_fit_error;
    float ld_cv;               /* 有效平均组之间的标准差/均值 */
    float lq_cv;
    float ld_polarity_error;    /* 正负脉冲均值差/总体均值 */
    float lq_polarity_error;
    uint16_t ld_valid_samples;
    uint16_t lq_valid_samples;
    uint16_t ld_fit_groups;       /* 每组为 4 次同极性轨迹平均，不是独立单脉冲拟合 */
    uint16_t lq_fit_groups;
    float sample_period_us;     /* DWT 测得回调间隔，不等于 ADC 孔径时间 */
    float sample_period_min_us;
    float sample_period_max_us;
    uint8_t failed_step;        /* 失败前的原步骤；0 表示本次尚未失败 */
    /* 最近辨识轴的正/负阶跃诊断；换轴时清零，停机后保留。
     * 每条曲线是最近一个通过拟合的平均组，共 i0..i8，单位 A。
     * 两侧电感是各自全部有效平均组的均值，单位 H。
     */
    float l_positive_current[ID_L_SAMPLE_TICKS + 1U];
    float l_negative_current[ID_L_SAMPLE_TICKS + 1U];
    float l_positive_h;
    float l_negative_h;
    float Rs_low;              /* 低→中平台的增量电阻，Ω，含逆变器/线阻影响 */
    float Rs_high;             /* 中→高平台的增量电阻，Ω，分别用于 Ld 两侧动态换算 */
    float l_bias_current_a;    /* 最近辨识轴的实际偏置平台 Id，A；Lq 在高平台辨识 */
} ParamId_t;

extern ParamId_t pid;
void ParamIdStart(void);
void ParamIdRun(uint32_t sample_cycle); /* 本拍 ADC 整组回调入口的 DWT 周期计数 */
#endif
