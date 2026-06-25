#include "Pwm_output.h"

extern TIM_HandleTypeDef htim1;
extern SVPWM Svpwm_dq;

void PWM_Start(void)
{
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
    // CH4 用于 TRGO 触发 ADC
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
}

void PWM_Stop(void)
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);

    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);
}

void PWM_SetDuty(uint16_t ccr1, uint16_t ccr2, uint16_t ccr3)
{
    if (ccr1 > TIM1_ARR) ccr1 = TIM1_ARR;
    if (ccr2 > TIM1_ARR) ccr2 = TIM1_ARR;
    if (ccr3 > TIM1_ARR) ccr3 = TIM1_ARR;

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ccr1);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, ccr2);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, ccr3);
}

void PWM_Update_From_SVPWM(void)
{
    PWM_SetDuty(Svpwm_dq.SVPTa, Svpwm_dq.SVPTb, Svpwm_dq.SVPTc);
}
