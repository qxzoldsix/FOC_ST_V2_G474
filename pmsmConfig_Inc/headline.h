#ifndef __HEADLINE_H
#define __HEADLINE_H

#ifdef __cplusplus
extern "C" {
#endif
#define _RAM_FUNC  __attribute__((section(".RAM_FUNC")))
#include <stdint.h>

#include <math.h>

#include "main.h"
#include "gpio.h"
#include "dma.h"
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "spi.h"
#include "fdcan.h"

#include "usb_device.h"
#include "usbd_cdc_if.h"

#include "lcd.h"
#include "key_bsp.h"
#include "key_driver.h"
#include "ADC_SAMPLE.h"

#include "Common_Math.h"
#include "PI_Cale.h"
#include "svpwm.h"

#include "InvProtect.h"
#include "motorControl.h"
#include "Foc.h"
#include "VF.h"
#include "DrvPwmout.h"
#include "flux.h"

#include "PI_Cale.h"
#include "taskManager.h"


#define LED_TOGGLE()  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13)
#define LED_ON()		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET)
#define LED_OFF()     HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET)

#ifdef __cplusplus
}
#endif

#endif

