#ifndef __FLUX_H_
#define __FLUX_H_
#include "headline.h"

/* ========== 观测器类型选择 ========== */
#define OBS_FLUX_VOLTAGE     0   // 电压型磁链观测器 (当前)
#define OBS_EKF              1   // 扩展卡尔曼滤波
#define OBS_FLUX_NONLINEAR   2   // 非线性磁链观测器
#define OBS_LUENBERGER       3   // 龙伯格观测器
#define OBS_SMO              4   // 滑模观测器

#define OBSERVER_TYPE  OBS_FLUX_VOLTAGE   // <-- 在此切换观测器

/* ========== 电机参数（运行时可切换方案待实现）========== */
#define MOTOR_RS    0.875f   // 定子电阻 (Ω)
#define MOTOR_LS    0.0002f    // 定子电感 (H)，关系启动顺滑性
#define MOTOR_FLUX  0.00752f    // 永磁体磁链 (Wb)
#define MOTOR_POLES 14          // 极对数

typedef struct {
    float Rs;              // 定子电阻 (Ω)
    float Ls;              // 定子电感 (H)
    float Flux;            // 永磁体磁链 (Wb)
    float Gain;            // 磁链幅值补偿增益
    float PLL_ki;          // PLL 积分系数
    float PLL_kp;          // PLL 比例系数
    float PLL_Err;         // PLL 角度误差
    float PLL_Interg;      // PLL 积分累加值
    float PLL_Ui;          // PLL PI 输出（电角速度）
    float Ctrl_ts;         // FOC 控制周期 (s)
    float Err;             // 磁链幅值误差
    float Theta;           // 转子电角度 (rad)
    float speed_hz;        // 转子电频率 (Hz)
    /* PLL gain ramp recovery */
    float PLL_kp_target;   // 正常 Kp 目标值
    float PLL_ki_target;   // 正常 Ki 目标值
    uint8_t pll_ramp_active; // 1=正在恢复增益
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
void Observer_Run_NonlinearFlux(void);
void Observer_Run_Luenberger(void);
void Observer_Run_SMO(void);
#endif
