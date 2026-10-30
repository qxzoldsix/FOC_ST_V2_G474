/**
 * @file    Pwm_output.c
 * @brief   旧 PWM 输出模块占位
 *
 * PWM 的唯一实现已收敛到：
 *   - UserSoftware/src/DrvPwmout.c：TIM1 六路互补输出启停与硬件封波
 *   - UserSoftware/src/motorControl.c：SVPWM 占空比提交
 *
 * 保留该空翻译单元是为了兼容旧工程目录；不要在这里恢复 PWM_Start、
 * PWM_Stop、PwmSetDuty 或 PWM_Update_From_SVPWM，避免出现第二套输出状态。
 */
