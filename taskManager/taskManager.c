/**
 * @file    taskManager.c
 * @author  nono <nono_1007@foxmail.com>
 * @brief   时间片任务调度 + Vofa 数据上报 + 按键处理 + LCD 显示
 */
#include "headline.h"


TaskTime  TasksPare[Task_Num];
/**
 * @brief  Vofa JustFloat 协议发送电机数据
 * @note   每帧 = 6个float(各4字节, little-endian) + 帧尾(00 00 80 7F)
 */

/**
 * Vofa JustFloat 数据上报 (1ms / ~1kHz)
 *
 * CH1: CurrentHz      — 当前电频率 (Hz)
 * CH2: Id (Ds)        — d 轴电流反馈, Id=0 控制时 ≈ 0
 * CH3: Iq (Qs)        — q 轴电流反馈 (转矩电流)
 * CH4: Vd             — d 轴电压输出 (pu)
 * CH5: Vq             — q 轴电压输出 (pu)
 * CH6: BUS_Voltage    — 母线电压 (V)
 * CH7: PhaseU_Curr    — U 相电流 (A)
 * CH8: PhaseV_Curr    — V 相电流 (A)
 * CH9: PhaseW_Curr    — W 相电流 (A)
 */
void HFPeriod_RUN(void)
{
    static uint8_t buf[40];   // 9 floats × 4 + 4 byte tail = 40
    float *p = (float *)buf;

    p[0] = motor.CurrentHz;
    p[1] = PARK_PCurr.Ds;
    p[2] = PARK_PCurr.Qs;
    p[3] = motor.V_d;
    p[4] = motor.V_q;
    p[5] = Volt_CurrPara.BUS_Voltage;
    p[6] = Volt_CurrPara.PhaseU_Curr;
    p[7] = Volt_CurrPara.PhaseV_Curr;
    p[8] = Volt_CurrPara.PhaseW_Curr;

    // JustFloat frame tail
    buf[36] = 0x00;
    buf[37] = 0x00;
    buf[38] = 0x80;
    buf[39] = 0x7F;

    CDC_Transmit_FS(buf, sizeof(buf));
}



void task_send_Rece(void)
{
    LED_TOGGLE();
    NTC_Task();
}


void Balance_Control(void)
{

}


void KEY_RUN(void)  
{
    key_process();

    /* ---- KEY1 (PC9): short = start/stop, long = clear fault ---- */
    if (key_array[key_sw1].key_value == key_click_one) {
        // TODO: start/stop
    }
    if (key_array[key_sw1].key_value == key_long_press) {
        // TODO: clear fault
    }
    key_array[key_sw1].key_value = key_none;

    /* ---- KEY2 (PC8): short = mode cycle: STOP->VF->PREPOS->Sensorless ---- */
    if (key_array[key_sw2].key_value == key_click_one) {
        // TODO: mode cycle
    }
    key_array[key_sw2].key_value = key_none;

    /* ---- KEY3 (PC7): short = TargetHz +5, long = continuous +5 ---- */
    if (key_array[key_sw3].key_value == key_click_one ||
        key_array[key_sw3].key_value == key_long_press) {
        // TODO: TargetHz +5
    }
    key_array[key_sw3].key_value = key_none;

    /* ---- KEY4 (PC6): short = TargetHz -5, long = continuous -5 ---- */
    if (key_array[key_sw4].key_value == key_click_one ||
        key_array[key_sw4].key_value == key_long_press) {
        // TODO: TargetHz -5
    }
    key_array[key_sw4].key_value = key_none;
}





void Task_DEBUG(void)
{
    // ============ 顶部状态栏：模式 + V_amp ============
    switch (motor.Control_Mode) {
        case 0: LCD_ShowString(75, 5, (uint8_t *)"[STOP]       ", WHITE, DARKBLUE, 12, 0); break;			
        case 1: LCD_ShowString(75, 5, (uint8_t *)"[SENSOR]     ", GREEN, DARKBLUE, 12, 0); break;
			  case 2: LCD_ShowString(75, 5, (uint8_t *)"[SENSORLESS] ", GREEN, DARKBLUE, 12, 0); break;     // 无感FOC
				case 3: LCD_ShowString(75, 5, (uint8_t *)"[ VF ]       ", GREEN, DARKBLUE, 12, 0); break;     // VF开环
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
    // y=60: 当前频率
    LCD_ShowString(5, 60, (uint8_t *)"FHz", WHITE, BLACK, 12, 0);
    LCD_ShowFloatNum1(45, 60, motor.CurrentHz, 5, 1, GREEN, BLACK, 12);

    // y=78: PLL 误差
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

    // y=114: 磁链幅值 |ψr|
    {
        float fluxr_mag = sqrtf(FluxR_in_wb[0] * FluxR_in_wb[0] +
                                FluxR_in_wb[1] * FluxR_in_wb[1]);
        LCD_ShowString(5, 114, (uint8_t *)"Psi", WHITE, BLACK, 12, 0);
        LCD_ShowFloatNum1(45, 114, fluxr_mag, 5, 4, MAGENTA, BLACK, 12);
    }

    // y=132: NTC 温度 T1(PB12) / T2(PB1)
    {
        uint16_t c1 = (ntc.ot_fault || ntc.ntc1_c > NTC_OT_WARN_C) ? RED : GREEN;
        uint16_t c2 = (ntc.ot_fault || ntc.ntc2_c > NTC_OT_WARN_C) ? RED : GREEN;

        LCD_ShowString(5, 132, (uint8_t *)"T1", WHITE, BLACK, 12, 0);
        LCD_ShowFloatNum1(25, 132, ntc.ntc1_c, 4, 1, c1, BLACK, 12);

        LCD_ShowString(90, 132, (uint8_t *)"T2", WHITE, BLACK, 12, 0);
        LCD_ShowFloatNum1(110, 132, ntc.ntc2_c, 4, 1, c2, BLACK, 12);
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
