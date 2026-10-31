/** @file debugSerial.h @brief USART3 调试文本输出，PB10/TX、PB11/RX，115200 8N1。 */
#ifndef DEBUG_SERIAL_H
#define DEBUG_SERIAL_H

#include "usart.h"

/* 仅在主循环调用；内部复制文本后 DMA 发送，忙时返回 HAL_BUSY，不等待。
 * 文本最长 255 字节；不在电流中断中格式化或输出日志。
 */
HAL_StatusTypeDef DebugSerialWrite(const char *text);
void DebugStatusTaskRun(void);

#endif
