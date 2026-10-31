/**
 * @file boardPeripherals.c
 * @brief 板载显示、按键及通信接口初始化，不配置电机控制参数。
 */
#include "boardPeripherals.h"
#include "main.h"
#include "lcd.h"
#include "key_driver.h"
#include "can_app.h"
#include "fdcan.h"
#include "usb_device.h"
#include "debugSerial.h"

void BoardPeripheralInit(void)
{
    LCD_Init();
    key_driver_init();
}

/* 控制层与任务表就绪后再接收外部命令，避免校准过程中被 CAN 启动。 */
void BoardCommunicationInit(void)
{
    CAN_App_Init();
    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK ||
        HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
        Error_Handler();
    }
    MX_USB_Device_Init();
    (void)DebugSerialWrite("USART3 debug ready: 115200 8N1\r\n");
}
