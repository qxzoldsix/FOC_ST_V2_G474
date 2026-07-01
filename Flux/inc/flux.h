#ifndef __FLUX_H_
#define __FLUX_H_
#include "headline.h"

/* ========== 电机参数 ========== */
#define MOTOR_RS    0.875f      // 定子电阻 (Ω), 实测线电阻 1.75Ω / 2 (Y接)
#define MOTOR_LS    0.0005f     // 定子电感 (H), 0.5mH 保守估计（实测后修正）
#define MOTOR_FLUX  0.00752f    // 永磁体磁链 (Wb)
#define MOTOR_POLES 52          // 极对数

/* ========== 滑模观测器参数 ========== */
#define SMO_K_SLIDE     5.0f    // 滑模增益 (V), 须 > 最大反电动势 (20Hz→~0.95V)
#define SMO_FC_HZ       100.0f  // 反电动势 LPF 截止频率 (Hz)
#define SMO_FC_START_HZ 40.0f   // 低速时降低的截止频率 (Hz)
#define SMO_PLL_KP      0.01f   // PLL 比例增益（误差量纲: rad）
#define SMO_PLL_KI      0.5f    // PLL 积分增益（误差量纲: rad）

typedef struct {
    /* ---- 电机参数 ---- */
    float Rs;              // 定子电阻 (Ω)
    float Ls;              // 定子电感 (H)
    float Flux;            // 永磁体磁链 (Wb)
    float Ctrl_ts;         // FOC 控制周期 (s)

    /* ---- SMO 状态 ---- */
    float Ialpha_hat;      // 估算 α 轴电流 (A)
    float Ibeta_hat;       // 估算 β 轴电流 (A)
    float Ealpha;          // 估算 α 轴反电动势 (V)
    float Ebeta;           // 估算 β 轴反电动势 (V)
    float K_slide;         // 滑模增益 (V)
    float Fc_hz;           // 反电动势 LPF 截止频率 (Hz)
    float E_mag;           // 反电动势幅值 (V)

    /* ---- PLL 锁相环 ---- */
    float PLL_kp;          // PLL 比例系数
    float PLL_ki;          // PLL 积分系数
    float PLL_Err;         // PLL 角度误差
    float PLL_Interg;      // PLL 积分累加值
    float PLL_Ui;          // PLL PI 输出（电角速度 rad/s）
    float PLL_Ui_Old;      // 上一周期 PLL 输出
    float Theta;           // 转子电角度 (rad)

    /* ---- 速度估算 ---- */
    float speed_hz;        // 转子电频率 (Hz)
    float speed_hz_Old;    // 上一周期电频率
    float speed_hz_Acc;    // 频率加速度累加
    float speed_hz_Acc_Temp;

    /* ---- PLL 增益 ramp ---- */
    float PLL_kp_target;
    float PLL_ki_target;
    uint8_t pll_ramp_active;

    /* ---- 兼容旧接口（磁链幅值误差，供 motorControl 暖启动判断）---- */
    float Err;             // 保留（SMO 下不使用）
    float Gain;            // 保留（SMO 下不使用）
} FOC_OBSERVER_DEF;

extern FOC_OBSERVER_DEF Foc_observer;

/* 转子磁链 αβ 分量 (Wb) — 由 SMO 反电动势计算，供 motorControl 暖启动检测 */
extern float FluxR_in_wb[2];

/* ---- 保留旧名称兼容（SMO 不使用，但其他文件可能引用）---- */
extern float Flux_in_wb[2];
extern float FluxS_in_wb[2];

void Flux_Observer_Init(void);
void Observer_Run(void);
void Angel_Get(void);
#endif
