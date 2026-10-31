/**
 * @file    taskManager.c
 * @author  nono <nono_1007@foxmail.com>
 * @brief   应用任务：数据上报、板载监测、按键、显示与 CAN 通信
 */
#include "headline.h"
#include <stdio.h>
#include <string.h>

extern USBD_HandleTypeDef hUsbDeviceFS;


/* 两块采样数组轮流使用；每组依次存 U/V/W 电流，第四项空间存帧尾字节。 */
static float current_samples[2][CURRENT_BATCH_SIZE][4];
/* 复制完整一批后交给 USB，发送完成前保持不变。 */
static float current_tx[CURRENT_BATCH_SIZE][4];
/* 对应两块采样数组：0=可写或正在填充，1=整批已填满、等待发送。 */
static volatile uint8_t current_ready[2];
static uint8_t current_write;  /* 电流中断当前填写的数组编号，0 或 1 */
static uint8_t current_read;   /* 主循环下一次发送的数组编号，0 或 1 */
static uint16_t current_index; /* 当前批次已保存的样本数，也是下一次写入的位置 */

/* 在实际通道数据之后，逐字节写入 VOFA JustFloat 固定帧尾。 */
static void JustFloatAppendTail(float *data, uint16_t channels)
{
    uint8_t *buf = (uint8_t *)data;
    uint16_t end = channels * sizeof(float);
    buf[end]     = 0x00;
    buf[end + 1] = 0x00;
    buf[end + 2] = 0x80;
    buf[end + 3] = 0x7F;
}

/* 电流环内只存数组；缓冲未发送时跳过新样本，不等待 USB。 */
void HyperSpeedDataCapture(float iu, float iv, float iw)
{
    if (current_ready[current_write]) return;
    __DMB(); /* 保证读到就绪状态后再访问对应缓冲，不关闭中断。 */
    float *sample = current_samples[current_write][current_index];
    sample[0] = iu;
    sample[1] = iv;
    sample[2] = iw;
    JustFloatAppendTail(sample, 3U);
    if (++current_index == CURRENT_BATCH_SIZE) {
        current_index = 0U;
        __DMB(); /* 数据写完后再发布就绪标志。 */
        current_ready[current_write] = 1U;
        current_write ^= 1U;
    }
}

/* 必须在填充发送数组之前检查：
 * 配置完成且类指针非空，才可以使用 CDC；TxState=0 表示上一帧已发送完成。
 * 底层虽然也检查忙状态，但若先改数组再调用底层，上一帧的数据已经被覆盖。
 */
static uint8_t TelemetryUsbIsReady(void)
{
    USBD_CDC_HandleTypeDef *cdc = hUsbDeviceFS.pClassData;
    return hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED &&
           cdc != NULL && cdc->TxState == 0U;
}

static void HighSpeedDataSend(void)
{
    if (!TelemetryUsbIsReady() || !current_ready[current_read]) return;
    __DMB();
    memcpy(current_tx, current_samples[current_read], sizeof(current_tx));
    if (CDC_Transmit_FS((uint8_t *)current_tx, sizeof(current_tx)) == USBD_OK) {
        __DMB();
        current_ready[current_read] = 0U;
        current_read ^= 1U;
    }
}

static float low_speed_tx[LOW_SPEED_CHANNEL_CAPACITY + 1U]; /* 最多 16 个通道，另预留 1 项用于追加帧尾 */

/* 低速通道配置入口：在这里追加 data[count++] = 变量；最多 16 个通道。
 * 发送长度按返回的实际通道数计算，不会把预留通道作为零值发送。
 */
static uint16_t LowSpeedDataFill(float *data)
{
    uint16_t count = 0U;
    data[count++] = Volt_CurrPara.PhaseU_Curr; /* 通道 1：U 相电流，A */
    data[count++] = Volt_CurrPara.PhaseV_Curr; /* 通道 2：V 相电流，A */
    data[count++] = Volt_CurrPara.PhaseW_Curr; /* 通道 3：W 相电流，A */
    /* 后续可追加母线电压、转速、dq 电流、观测器误差等调试量。 */
    return count;
}

static void LowSpeedDataSend(void)
{
    uint16_t channels;
    if (!TelemetryUsbIsReady()) return;
    channels = LowSpeedDataFill(low_speed_tx);
    JustFloatAppendTail(low_speed_tx, channels);
    (void)CDC_Transmit_FS((uint8_t *)low_speed_tx, (channels + 1U) * sizeof(float));
}

void TelemetryTaskRun(void)
{
    if (HyperSpeedDataProcessEn) {
        HighSpeedDataSend();
    } else {
        LowSpeedDataSend();
    }
}



void BoardMonitorTaskRun(void)
{
    LED_TOGGLE();
    NTC_Task();
}





void KeyTaskRun(void)  
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





void CanTaskRun(void)
{
    static uint8_t sub_tick = 0;

    CAN_App_PDO1_TX_Send();   // 每10ms发送电机状态

    sub_tick++;
    if (sub_tick >= 5) sub_tick = 0;

    if (sub_tick == 1) CAN_App_PDO2_TX_Send();    // 每50ms发送扩展数据
    if (sub_tick == 2) CAN_App_Heartbeat_Send();  // 每50ms发送心跳
}

/* 固定宽度显示带符号数值；无效值显示 ERR，超宽值显示 #，避免覆盖相邻字段。 */
static void DebugLcdNumber(char *out, float value, unsigned width,
                            unsigned decimals)
{
    char number[48];
    size_t count;
    if (!isfinite(value)) {
        strcpy(number, "ERR");
    } else if (fabsf(value) >= 1.0e9f) {
        memset(out, '#', width);
        out[width] = '\0';
        return;
    } else {
        (void)snprintf(number, sizeof(number), "%.*f", (int)decimals,
                       (double)value);
    }
    count = strlen(number);
    memset(out, (count > width) ? '#' : ' ', width);
    if (count <= width) memcpy(out + width - count, number, count);
    out[width] = '\0';
}

/* 周期刷新仅重绘固定宽度的数值区域，不重复绘制标签和单位。 */
static void DebugLcdValue(uint16_t x, uint16_t y, float value,
                           unsigned width, unsigned decimals, uint16_t color)
{
    char field[16];
    DebugLcdNumber(field, value, width, decimals);
    LCD_ShowString(x, y, (const uint8_t *)field, color, BLACK, 12, 0);
}

void DisplayTaskRun(void)
{
    static uint8_t mode_valid = 0U;
    static uint32_t last_mode;
    uint32_t current_mode = motor.Control_Mode;
    uint8_t estimated = (current_mode == MOTOR_MODE_SENSORLESS ||
                         current_mode == MOTOR_MODE_PREPOS);

    /* 模式名称、角度或频率标签及其单位，仅在首次刷新或模式变化时更新。 */
    if (!mode_valid || last_mode != current_mode) {
        const char *mode;
        char field[15];
        size_t length;
        uint16_t color = LGRAY;
        switch (current_mode) {
            case MOTOR_MODE_STOP:        mode = "[STOP]"; color = LGRAY; break;
            case MOTOR_MODE_SENSOR:      mode = "[SENSOR]"; break;
            case MOTOR_MODE_SENSORLESS:  mode = "[SENSORLESS]"; break;
            case MOTOR_MODE_VF:          mode = "[VF]"; break;
            case MOTOR_MODE_IF:          mode = "[IF]"; break;
            case MOTOR_MODE_PREPOS:      mode = "[PREPOS]"; break;
            case MOTOR_MODE_PARAM_ID:    mode = "[PARAM ID]"; color = LGRAY; break;
            case MOTOR_MODE_IF_STARTUP:  mode = "[I/F START]"; color = LGRAY; break;
            case MOTOR_MODE_HFI_STARTUP: mode = "[HFI START]"; color = LGRAY; break;
            default: mode = "[UNKNOWN]"; color = RED; break;
        }
        memset(field, ' ', 14U);
        length = strlen(mode);
        memcpy(field, mode, length);
        field[14] = '\0';
        LCD_ShowString(81, 3, (const uint8_t *)field, color, BLACK, 12, 0);

        /* I/F复用开环频率栏，右侧改为电流给定；只在模式变化时重画标签。 */
        LCD_ShowString(3, 41, (const uint8_t *)(current_mode == MOTOR_MODE_IF ? "IF Hz" : "VF Hz"),
                       GRAY, BLACK, 12, 0);
        LCD_ShowString(123, 41, (const uint8_t *)(current_mode == MOTOR_MODE_IF ? "IF I" : "VF V"),
                       GRAY, BLACK, 12, 0);
        LCD_ShowString(213, 41, (const uint8_t *)(current_mode == MOTOR_MODE_IF ? "A " : "pu"),
                       GRAY, BLACK, 12, 0);

        LCD_Fill(3, 77, 237, 89, BLACK);
        LCD_ShowString(3, 77, (const uint8_t *)(estimated ? "Est" : "Ang"),
                       GRAY, BLACK, 12, 0);
        LCD_ShowString(93, 77,
                       (const uint8_t *)(estimated ? "Hz" : "rad"),
                       GRAY, BLACK, 12, 0);
        last_mode = current_mode;
        mode_valid = 1U;
    }

    /* 左右两列各宽 120 像素，统一排列标签、右对齐的 9 字符数值及单位。 */
    DebugLcdValue(33, 23, Volt_CurrPara.BUS_Voltage, 9U, 3U, LGRAY);
    /* 开环给定：I/F显示电流斜坡输出，VF显示电压幅值，不冒充模式2的给定。 */
    DebugLcdValue(33, 41, motor.TargetHz, 9U, 1U, LGRAY);
    DebugLcdValue(153, 41, current_mode == MOTOR_MODE_IF ? I_d_GXieLv.XieLv_Y : motor.V_amp,
                   9U, 3U, LGRAY);
    DebugLcdValue(33, 59, motor.CurrentHz, 9U, 1U, LGRAY);
    DebugLcdValue(153, 59, Foc_observer.PLL_Err, 9U, 3U, LGRAY);
    DebugLcdValue(33, 77, estimated ? Foc_observer.speed_hz : motor.OpenTheta,
                   9U, estimated ? 1U : 2U, LGRAY);
    DebugLcdValue(33, 95, sqrtf(FluxR_in_wb[0] * FluxR_in_wb[0] +
                                FluxR_in_wb[1] * FluxR_in_wb[1]),
                   9U, 4U, LGRAY);
    DebugLcdValue(33, 113, ntc.ntc1_c, 9U, 1U,
                   (ntc.ot_fault || ntc.ntc1_c > NTC_OT_WARN_C) ? RED : LGRAY);
    DebugLcdValue(153, 113, ntc.ntc2_c, 9U, 1U,
                   (ntc.ot_fault || ntc.ntc2_c > NTC_OT_WARN_C) ? RED : LGRAY);
}
