/**
 * @file    DrvPwmout.c
 * @author  nono <nono_1007@foxmail.com>
 * @brief   PWM 输出控制: 6 路互补 PWM 启停
 */
#include "headline.h"

#define FOC_ADC_TRIGGER_CCR    3950U

static volatile uint8_t pwm_bridge_enabled = 0U;

static void Foc_Pwm_ClearDuty(void)
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0U);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0U);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0U);
}

/**
 * @brief  初始化 PWM 时基和 ADC 采样触发，三相桥保持封波
 * @note   TIM1 CH4 只用于触发 ADC 注入组，停机时也必须继续运行保护采样
 */
uint8_t Foc_Pwm_Init(void)
{
    pwm_bridge_enabled = 0U;
    __HAL_TIM_MOE_DISABLE(&htim1);
    Foc_Pwm_ClearDuty();
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, FOC_ADC_TRIGGER_CCR);

    if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4) != HAL_OK) {
        return 0U;
    }

    /* CH4 仅提供内部 ADC 触发；MOE 必须保持关闭。 */
    __HAL_TIM_MOE_DISABLE(&htim1);
    return 1U;
}

/**
 * @brief  启动 PWM 输出 (6 路互补)
 * @note   使用 TIM1 互补输出
 */
uint8_t Foc_Pwm_Start(void)
{
    if (Foc_Pwm_IsEnabled()) {
        return 1U;
    }

    /* 输出恢复前先封总输出并清零占空比，避免带旧 CCR 开桥。 */
    pwm_bridge_enabled = 0U;
    __HAL_TIM_MOE_DISABLE(&htim1);
    Foc_Pwm_ClearDuty();

    if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK ||
        HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2) != HAL_OK ||
        HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3) != HAL_OK ||
        HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3) != HAL_OK) {
        Foc_Pwm_Stop();
        return 0U;
    }

    pwm_bridge_enabled = 1U;
    __HAL_TIM_MOE_ENABLE(&htim1);
    return 1U;
}

/**
 * @brief  停止 PWM 输出
 * @note   封锁 6 路互补 PWM
 */
void Foc_Pwm_Stop(void)
{
    /* 先改变软件状态，再关 MOE；后续占空比写入会立即被拒绝。 */
    pwm_bridge_enabled = 0U;
    __HAL_TIM_MOE_DISABLE(&htim1);
    Foc_Pwm_ClearDuty();

    (void)HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    (void)HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
    (void)HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
    (void)HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
    (void)HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
    (void)HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);

    /* HAL Stop 会处理通道状态；再次确保桥输出关闭。CH4 保持运行。 */
    __HAL_TIM_MOE_DISABLE(&htim1);
}

uint8_t Foc_Pwm_IsEnabled(void)
{
    return (uint8_t)((pwm_bridge_enabled != 0U) &&
                     ((htim1.Instance->BDTR & TIM_BDTR_MOE) != 0U));
}
