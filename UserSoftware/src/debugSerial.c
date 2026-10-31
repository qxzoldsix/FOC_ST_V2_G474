/** @file debugSerial.c @brief 主循环调试日志，独立于 USB 波形上报。 */
#include "headline.h"
#include "debugSerial.h"
#include <stdio.h>
#include <string.h>

/* DMA 发送期间保持原内容；只允许主循环提交下一条文本。 */
static char debugTx[256];

HAL_StatusTypeDef DebugSerialWrite(const char *text)
{
    size_t length;
    if (text == NULL) return HAL_ERROR;
    if (huart3.gState != HAL_UART_STATE_READY) return HAL_BUSY;
    length = strlen(text);
    if (length == 0U || length >= sizeof(debugTx)) return HAL_ERROR;
    memcpy(debugTx, text, length);
    return HAL_UART_Transmit_DMA(&huart3, (uint8_t *)debugTx, (uint16_t)length);
}

/* 每 500ms 输出一行；电压用 mV。
 */
void DebugStatusTaskRun(void)
{
    static uint8_t detail; /* 失败后依次输出状态、拟合判据、周期点；每次只提交一条DMA文本。 */
    char line[256];
    if (huart3.gState != HAL_UART_STATE_READY) return;
    /* 辨识结束后保留关键采样证据和候选结果；同一条DMA文本中输出，避免第二条因忙丢失。
     * adc 为三路原始值的 min:max；n=合格周期，cross=上穿次数，drop=拒绝周期。
     */
    if (pid.done && pid.rl_valid && motor.Control_Mode == MOTOR_MODE_STOP) {
        unsigned points = pid.flux_samples < ID_FLUX_TRACE_CAPACITY ?
                          pid.flux_samples : ID_FLUX_TRACE_CAPACITY;
        if (detail > points + 1U) detail = 0U;
if (detail == 1U) {
            snprintf(line, sizeof(line),
                     "拟合(标志=0x%02X 频率mHz=[%ld:%ld] 跨度mV=%ld 残差千分=%ld 斜率误千分=%ld 偏置mV=%ld)\r\n",
                     (unsigned int)pid.flux_fit_flags,
                     (long)(pid.bemf_frequency_min_hz * 1000.0f),
                     (long)(pid.bemf_frequency_max_hz * 1000.0f),
                     (long)(pid.bemf_voltage_span_v * 1000.0f),
                     (long)fminf(pid.flux_fit_error * 1000.0f, 1000000000.0f),
                     (long)fminf(pid.flux_slope_relative_error * 1000.0f, 1000000000.0f),
                     (long)(pid.bemf_offset_v * 1000.0f));
        } else if (detail >= 2U) {
            unsigned index = detail - 2U;
            snprintf(line, sizeof(line), "点(i=%u 频率mHz=%ld 等效峰mV=%ld 原始峰mV=%ld)\r\n", index,
                     (long)(pid.bemf_trace_hz[index] * 1000.0f),
                     (long)(pid.bemf_trace_v[index] * 1000.0f),
                     (long)(pid.bemf_trace_peak_v[index] * 1000.0f));
        } else {
            snprintf(line, sizeof(line),
                     "模式=%u 故障=0x%04X 母线mV=%ld 辨识(步=%u 误=%u rl=%u 通过=%u) "
                     "磁链uWb=%ld 反电动势mVkrpm=%ld bemf(数=%u 上穿=%u 拒=%u 标志=0x%02X 峰mV=%ld adc=[%u:%u,%u:%u,%u:%u])\r\n",
                     (unsigned int)motor.Control_Mode, (unsigned int)motor.Fault_DTC,
                     (long)(Volt_CurrPara.BUS_Voltage * 1000.0f),
                     (unsigned int)pid.step, (unsigned int)pid.error, (unsigned int)pid.rl_valid,
                     (unsigned int)pid.valid,
                     (long)(pid.Flux * 1000000.0f), (long)(pid.Ke * 1000.0f),
                     (unsigned int)pid.flux_samples, (unsigned int)pid.bemf_crossings,
                     (unsigned int)pid.bemf_rejected_cycles, (unsigned int)pid.bemf_reject_flags,
                     (long)(pid.bemf_peak_v * 1000.0f),
                     (unsigned int)pid.bemf_raw_min[0], (unsigned int)pid.bemf_raw_max[0],
                     (unsigned int)pid.bemf_raw_min[1], (unsigned int)pid.bemf_raw_max[1],
                     (unsigned int)pid.bemf_raw_min[2], (unsigned int)pid.bemf_raw_max[2]);
        }
        if (DebugSerialWrite(line) == HAL_OK) detail++;
        return;
    }
    detail = 0U;
    snprintf(line, sizeof(line),
             "模式=%u 故障=0x%04X 母线mV=%ld 转速rpm=%ld "
             "辨识(步=%u 误=%u 完成=%u 通过=%u)\r\n",
             (unsigned int)motor.Control_Mode, (unsigned int)motor.Fault_DTC,
             (long)(Volt_CurrPara.BUS_Voltage * 1000.0f), (long)motor.SpeedRPM,
             (unsigned int)pid.step, (unsigned int)pid.error,
             (unsigned int)pid.done, (unsigned int)pid.valid);
    (void)DebugSerialWrite(line);
}
