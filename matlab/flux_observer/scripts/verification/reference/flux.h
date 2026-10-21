#ifndef __FLUX_H_
#define __FLUX_H_
#include "headline.h"

/* ========== 观测器类型选择 ========== */
#define OBS_FLUX_VOLTAGE         0   // 电压型磁链观测器
#define OBS_EKF                  1   // 扩展卡尔曼滤波
#define OBS_HYBRID_ACTIVE_FLUX   2   // 混合有效磁链观测器 (电流+电压互补)
#define OBS_LUENBERGER           3   // 龙伯格观测器
#define OBS_SMO                  4   // 滑模观测器


#define OBSERVER_TYPE  OBS_HYBRID_ACTIVE_FLUX   // <-- 在此切换观测器

/* ========== 电机参数（运行时可切换方案待实现）========== */
#define MOTOR_RS    0.875f      // 定子电阻 (Ω)
#define MOTOR_LS    0.0002f     // 定子电感 (H) — 表贴式电机 Ld≈Lq≈Ls
#define MOTOR_LD    0.0002f     // d 轴电感 (H)
#define MOTOR_LQ    0.0002f     // q 轴电感 (H)
#define MOTOR_FLUX  0.00752f    // 永磁体磁链 (Wb)
#define MOTOR_POLES 14          // 极对数

/* ========== Hybrid Active Flux 观测器参数 ========== */
#define HYBRID_PLL_BW_HZ      15.0f   // PLL 带宽 (Hz)，按二阶系统 ζ=1 设计
#define HYBRID_COMP_BW_HZ     5.0f    // 电流-电压模型反馈补偿环带宽 (Hz)
#define HYBRID_SMO_KSLIDE     10.0f   // SMO 滑模增益 (V)，决定最大修正电压幅值
#define HYBRID_SMO_BOUNDARY   0.002f  // SMO 边界层厚度 (Wb)，≈27% of ψf

typedef struct {
    /* ---- 电机参数 ---- */
    float Rs;              // 定子电阻 (Ω)
    float Ls;              // 定子电感 (H) — 电压型观测器用
    float Ld;              // d 轴电感 (H) — 混合有效磁链观测器用
    float Lq;              // q 轴电感 (H) — 混合有效磁链观测器用
    float Flux;            // 永磁体磁链幅值 (Wb)
    float Gain;            // 磁链幅值补偿增益（电压型观测器用）
    /* ---- PLL 锁相环 ---- */
    float PLL_ki;          // PLL 积分系数
    float PLL_kp;          // PLL 比例系数
    float PLL_Err;         // PLL 角度误差
    float PLL_Interg;      // PLL 积分累加值
    float PLL_Ui;          // PLL PI 输出（电角速度 rad/s）
    float PLL_BW_Hz;       // PLL 带宽 Hz（混合观测器按带宽设计 Kp/Ki）
    float Ctrl_ts;         // FOC 控制周期 (s)
    float Err;             // 磁链幅值误差（电压型用）/ 未使用（混合型用）
    float Theta;           // 转子电角度 (rad)
    float speed_hz;        // 转子电频率 (Hz)
    /* ---- PLL 增益恢复 ramp ---- */
    float PLL_kp_target;   // 正常 Kp 目标值
    float PLL_ki_target;   // 正常 Ki 目标值
    uint8_t pll_ramp_active; // 1=正在恢复增益
    /* ---- Hybrid Active Flux 专用 ---- */
    float ActiveFlux_I;               // 电流模型有效磁链标量: ψf + (Ld-Lq)·Id
    float Flux_Active_I[2];           // 电流模型有效磁链 αβ (Wb)
    float Flux_Active_V[2];           // 电压模型有效磁链 αβ (Wb) = ψs - Lq·I
    float Comp_Kp;                    // 反馈补偿 PI 比例系数
    float Comp_Ki;                    // 反馈补偿 PI 积分系数 (已含 Ts)
    float Comp_Interg[2];             // 反馈补偿积分累加 αβ
    uint8_t Comp_Mode;                // 0=PI 反馈律, 1=SMO 反馈律
    float SMO_Kslide;                 // SMO 滑模增益
    float SMO_Boundary;               // SMO 边界层厚度
} FOC_OBSERVER_DEF;

extern FOC_OBSERVER_DEF Foc_observer;

/* 磁链中间变量（αβ 坐标系，单位 Wb） */
extern float Flux_in_wb[2];    // 总磁链
extern float FluxR_in_wb[2];   // 转子磁链

void Flux_Observer_Init(void);
void Observer_Run(void);
void Angel_Get(void);

/* ========== 初始位置检测 (IPD) ========== */
void IPD_SixPulse_Injection(void);   // 六脉冲注入定位
void IPD_HFI_Injection(void);        // 高频注入定位 (HFI)

/* ========== 备选观测器 (空壳, 待实现) ========== */
void Observer_Run_EKF(void);
void Observer_Run_HybirdFlux(void);
void Observer_Run_Luenberger(void);
void Observer_Run_SMO(void);
#endif
