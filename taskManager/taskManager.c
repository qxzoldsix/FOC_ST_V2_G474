#include "headline.h"


TaskTime  TasksPare[Task_Num];


/**
 * Vofa JustFloat 数据上报 (1ms / ~1kHz)
 *
 * CH1: PLL_Err        — PLL 角度误差, 正常 ≈ 0
 * CH2: FluxR_mag      — 转子磁链幅值, 正常 ≈ MOTOR_FLUX
 * CH3: speed_hz / Hz  — 观测器估算电频率 (无感) 或 VF CurrentHz
 * CH4: Id (Ds)        — d 轴电流反馈, Id=0 控制时 ≈ 0
 * CH5: Iq (Qs)        — q 轴电流反馈 (转矩电流)
 * CH6: BUS_Voltage    — 母线电压
 */
void HFPeriod_RUN(void)
{
    static uint8_t buf[28];
    float *p = (float *)buf;

    float fluxr_mag = sqrtf(FluxR_in_wb[0] * FluxR_in_wb[0] +
                            FluxR_in_wb[1] * FluxR_in_wb[1]);

    p[0] = Foc_observer.PLL_Err;
    p[1] = fluxr_mag;
    p[2] = (motor.Control_Mode == 2 || motor.Control_Mode == 4)
               ? Foc_observer.speed_hz
               : motor.CurrentHz;
    p[3] = PARK_PCurr.Ds;
    p[4] = PARK_PCurr.Qs;
    p[5] = Volt_CurrPara.BUS_Voltage;

    // JustFloat frame tail
    buf[24] = 0x00;
    buf[25] = 0x00;
    buf[26] = 0x80;
    buf[27] = 0x7F;

    CDC_Transmit_FS(buf, sizeof(buf));
}


/**
 * @brief  Vofa JustFloat 协议发送电机数据
 * @note   每帧 = 6个float(各4字节, little-endian) + 帧尾(00 00 80 7F)
 *         共28字节，~50Hz周期调用
 */
void task_send_Rece(void)
{
		LED_TOGGLE();
}


void Balance_Control(void)
{

}


void KEY_RUN(void)  
{
    key_process();

    /* ---- KEY1 (PC9): short = start/stop, long = clear fault ---- */
    if (key_array[key_sw1].key_value == key_click_one) {
        if (motor.Control_Mode == 0) {
            motor.Control_Mode = 3;  // default to VF on start
            Foc_Pwm_Start();
        } else {
            motor.Control_Mode = 0;  // STOP
            Foc_Pwm_Stop();
        }
    }
    if (key_array[key_sw1].key_value == key_long_press) {
        InvProtect_Clear();
        Foc_Pwm_Start();             // re-enable PWM after fault clear
    }
    key_array[key_sw1].key_value = key_none;

    /* ---- KEY2 (PC8): short = mode cycle: STOP->VF->Sensorless->Prepos->STOP ---- */
    if (key_array[key_sw2].key_value == key_click_one) {
        static const uint8_t mode_cycle[] = {0, 3, 4, 2};  // STOP, VF, PREPOS, Sensorless
        uint8_t i;
        for (i = 0; i < 4; i++) {
            if (motor.Control_Mode == mode_cycle[i]) break;
        }
        if (i < 4) {
            motor.Control_Mode = mode_cycle[(i + 1) % 4];
        } else {
            motor.Control_Mode = 3;  // unknown state -> VF
        }
        if (motor.Control_Mode == 0) Foc_Pwm_Stop();
        else                         Foc_Pwm_Start();
    }
    key_array[key_sw2].key_value = key_none;

    /* ---- KEY3 (PC7): short = TargetHz +5, long = continuous +5 ---- */
    if (key_array[key_sw3].key_value == key_click_one ||
        key_array[key_sw3].key_value == key_long_press) {
        motor.TargetHz += 5.0f;
        if (motor.TargetHz > VF_FREQ_MAX) motor.TargetHz = VF_FREQ_MAX;
        /* 无感模式下同步更新速度环目标 */
        if (motor.Control_Mode == 2) {
            SpeedRpm_GXieLv.XieLv_X = motor.TargetHz * 60.0f / (MOTOR_POLES / 2.0f);
        }
    }
    key_array[key_sw3].key_value = key_none;

    /* ---- KEY4 (PC6): short = TargetHz -5, long = continuous -5 ---- */
    if (key_array[key_sw4].key_value == key_click_one ||
        key_array[key_sw4].key_value == key_long_press) {
        motor.TargetHz -= 5.0f;
        if (motor.TargetHz < 0.0f) motor.TargetHz = 0.0f;
        /* 无感模式下同步更新速度环目标 */
        if (motor.Control_Mode == 2) {
            SpeedRpm_GXieLv.XieLv_X = motor.TargetHz * 60.0f / (MOTOR_POLES / 2.0f);
        }
    }
    key_array[key_sw4].key_value = key_none;
}





void Task_DEBUG(void)
{
    // ============ 顶部状态栏：模式 + V_amp ============
    switch (motor.Control_Mode) {
        case 0: LCD_ShowString(75, 5, (uint8_t *)"[STOP]       ", WHITE, DARKBLUE, 12, 0); break;			
        case 1: LCD_ShowString(75, 5, (uint8_t *)"[SENSOR]     ", GREEN, DARKBLUE, 12, 0); break;			//READY
			  case 2: LCD_ShowString(75, 5, (uint8_t *)"[SENSORLESS] ", GREEN, DARKBLUE, 12, 0); break;     // 有感
				case 3: LCD_ShowString(75, 5, (uint8_t *)"[ VF ]       ", GREEN, DARKBLUE, 12, 0); break;     // 无感
        case 4: LCD_ShowString(75, 5, (uint8_t *)"[PREPOS]     ", GREEN, DARKBLUE, 12, 0); break;
        default: LCD_ShowString(75, 5, (uint8_t *)"[ ?? ]      ", WHITE, DARKBLUE, 12, 0); break;
    }
    LCD_ShowFloatNum1(170, 3, Volt_CurrPara.BUS_Voltage, 4, 3, GREEN, DARKBLUE, 16);

    // ============ 母线栏：目标频率 + 实际电压 ============
    // 左：TargetHz
    LCD_ShowFloatNum1(35, 24, motor.TargetHz, 5, 1, YELLOW, LGRAY, 12);
    // 右：V_amp
    LCD_ShowFloatNum1(145, 24, motor.V_amp, 4, 3, GREEN, LGRAY, 12);

    // ============ 模式栏 ============
    switch (motor.Control_Mode) {
        case 0: LCD_ShowString(50, 45, (uint8_t *)"[STOP]       ", WHITE, BLACK, 12, 0); break;			
        case 1: LCD_ShowString(50, 45, (uint8_t *)"[SENSOR]     ", GREEN, BLACK, 12, 0); break;			//READY
			  case 2: LCD_ShowString(50, 45, (uint8_t *)"[SENSORLESS] ", GREEN, BLACK, 12, 0); break;     // 有感
				case 3: LCD_ShowString(50, 45, (uint8_t *)"[ VF ]       ", GREEN, BLACK, 12, 0); break;     // 无感
        case 4: LCD_ShowString(50, 45, (uint8_t *)"[PREPOS]     ", GREEN, BLACK, 12, 0); break;
        default: LCD_ShowString(50, 45, (uint8_t *)"[ ?? ]      ", WHITE, BLACK, 12, 0); break;
    }

    // ============ 数据行 ============
    // y=60: 当前频率 / 估算频率
    LCD_ShowString(5, 60, (uint8_t *)"FHz", WHITE, BLACK, 12, 0);
    LCD_ShowFloatNum1(45, 60, motor.CurrentHz, 5, 1, GREEN, BLACK, 12);

    // y=78: PLL 误差 / 磁链幅值
    LCD_ShowString(5, 78, (uint8_t *)"PLL", WHITE, BLACK, 12, 0);
    LCD_ShowFloatNum1(45, 78, Foc_observer.PLL_Err, 5, 3, YELLOW, BLACK, 12);

    // y=96: 估算速度 (Sensorless) 或 VF 角度 (VF)
    if (motor.Control_Mode == 2 || motor.Control_Mode == 4) {
        LCD_ShowString(5, 96, (uint8_t *)"Est", WHITE, BLACK, 12, 0);
        LCD_ShowFloatNum1(45, 96, Foc_observer.speed_hz, 5, 1, BRRED, BLACK, 12);
    } else {
        LCD_ShowString(5, 96, (uint8_t *)"Ang", WHITE, BLACK, 12, 0);
        LCD_ShowFloatNum1(45, 96, motor.OpenTheta, 5, 2, BRRED, BLACK, 12);
    }

    // y=114: 磁链幅值 |ψr| / 故障码
    {
        float fluxr_mag = sqrtf(FluxR_in_wb[0] * FluxR_in_wb[0] +
                                FluxR_in_wb[1] * FluxR_in_wb[1]);
        LCD_ShowString(5, 114, (uint8_t *)"Psi", WHITE, BLACK, 12, 0);
        LCD_ShowFloatNum1(45, 114, fluxr_mag, 5, 4, MAGENTA, BLACK, 12);
    }
}



void Task_Manage_List_Init(void)
{
    TasksPare[0].Task_Period=HFPeriod_COUNT; 
    TasksPare[0].Task_Count=1;               
    TasksPare[0].Task_Function=HFPeriod_RUN;

    TasksPare[1].Task_Period=FaulPeriod_COUNT;
    TasksPare[1].Task_Count=8;
    TasksPare[1].Task_Function=task_send_Rece;

    TasksPare[2].Task_Period=Balance_COUNT;
    TasksPare[2].Task_Count=15;
    TasksPare[2].Task_Function=Balance_Control;

    TasksPare[3].Task_Period=KEY_COUNT;
    TasksPare[3].Task_Count=80;
    TasksPare[3].Task_Function=KEY_RUN;

    TasksPare[4].Task_Period = DebugPeriod_COUNT;
    TasksPare[4].Task_Count=600;
    TasksPare[4].Task_Function = Task_DEBUG;   
	
		HAL_TIM_Base_Start_IT(&htim3);
}

void Timer_Task_Count(void)  
{
    uint16_t Task_Count = 0;

    for(Task_Count = 0; Task_Count<Task_Num; Task_Count++)  
    {
        if((TasksPare[Task_Count].Task_Count<TasksPare[Task_Count].Task_Period)&&(TasksPare[Task_Count].Task_Period>0))
        {
            TasksPare[Task_Count].Task_Count++;   
        }
    }
}

void Execute_Task_List_RUN(void)
{
    uint16_t Task_Count=0;

    for(Task_Count = 0; Task_Count < Task_Num; Task_Count++)
    {

        if((TasksPare[Task_Count].Task_Count >= TasksPare[Task_Count].Task_Period)&&(TasksPare[Task_Count].Task_Period > 0))
        {
            TasksPare[Task_Count].Task_Function(); 
            TasksPare[Task_Count].Task_Count=0;
        }
    }
}
