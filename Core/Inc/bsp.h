/** @file bsp.h @brief 芯片资源初始化与 ADC 常规采样缓冲区。 */
#ifndef BSP_H
#define BSP_H
#include <stdint.h>

void BspInit(void);
void SystemClock_Config(void); /* USB 低功耗恢复时也使用该时钟配置。 */
void BspStartAdcSampling(void);
extern uint16_t adc1_buff[2];
extern uint16_t adc2_buff[4];
#endif
