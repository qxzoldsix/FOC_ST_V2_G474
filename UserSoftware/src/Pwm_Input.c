#include "headline.h"//做外部输入捕获
//volatile pwm_Input_Struct motor_Pwm_Input_Value;
//uint16_t current_throttle;
//void Pwm_Input_Trigger_Callback(void){
//		motor_Pwm_Input_Value.period = __HAL_TIM_GET_COMPARE(&htim4, TIM_CHANNEL_3) + 1;
//		motor_Pwm_Input_Value.high   = __HAL_TIM_GET_COMPARE(&htim4, TIM_CHANNEL_4) + 1;
//    motor_Pwm_Input_Value.low    = motor_Pwm_Input_Value.period - motor_Pwm_Input_Value.high;
//    motor_Pwm_Input_Value.freq   = 1000000 / motor_Pwm_Input_Value.period;
//    // 高电平时长需要在1ms-2ms之间，这里设置的最大值为3ms
//    if( (20 < motor_Pwm_Input_Value.freq) &&
//            (400 > motor_Pwm_Input_Value.freq) &&
//            (3000 > motor_Pwm_Input_Value.high) &&
//            (1000 < motor_Pwm_Input_Value.high) )
//    {

//        if(2000 <= motor_Pwm_Input_Value.high)
//        {
//            motor_Pwm_Input_Value.throttle = 1000;
//        }
//        else
//        {
//            motor_Pwm_Input_Value.throttle = (motor_Pwm_Input_Value.high - 1000);
//        }

//        if(BLDC_MIN_DUTY > (motor_Pwm_Input_Value.throttle * BLDC_MAX_DUTY / 1000))
//        {
//            motor_Pwm_Input_Value.throttle = 0;
//        }
//    }
//    else
//    {
//        motor_Pwm_Input_Value.throttle = 0;
//    }
//    motor.CurrentHz = Foc_observer.speed_hz*(MOTOR_POLES/2);
//    motor.Motor_Speed = (uint32_t)motor_Pwm_Input_Value.throttle * BLDC_MAX_DUTY / 1000;
//    motor.SpeedRPM = (Foc_observer.speed_hz * 60.0f) / (MOTOR_POLES/2);
//    current_throttle= motor_Pwm_Input_Value.throttle;
//    current_throttle =  SpeedRpm_GXieLv.XieLv_Y;
//    // usb_printf("Motor Speed: %d\n", motor_Pwm_Input_Value.throttle);
//}
//void Pwm_Input_Timeout_Callback(void){
//    motor_Pwm_Input_Value.high   = 0;
//    motor_Pwm_Input_Value.low    = 0;
//    motor_Pwm_Input_Value.period = 3000;       // 获取周期值
//    motor_Pwm_Input_Value.freq   = 0;
//    motor_Pwm_Input_Value.throttle = 0;
//    motor.Motor_Speed = 0;
//    motor.CurrentHz=0;
//    motor.SpeedRPM=0;
//}