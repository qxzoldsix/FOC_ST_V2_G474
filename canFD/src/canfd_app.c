#include "canfd_app.h"
#include <math.h>
#include <string.h>

/* ========== SDO 配置与辅助函数 ========== */
/* 通信侧安全边界，不代表推荐的控制器整定值。 */
#define SDO_SPEED_KP_MAX         1.0f
#define SDO_SPEED_KI_MAX         0.1f
#define SDO_CURRENT_KP_MAX       1.0f
#define SDO_CURRENT_KI_MAX       0.1f
#define SDO_SPEED_UMAX_MIN_A     0.1f
#define SDO_SPEED_UMAX_MAX_A    12.0f
#define SDO_PLL_KP_MAX        1000.0f
#define SDO_PLL_KI_MAX         100.0f
#define SDO_OBSERVER_GAIN_MAX 100000.0f
#define SDO_VF_RAMP_GRAD_MIN      0.00001f
#define SDO_VF_RAMP_GRAD_MAX      0.1f

enum {
    SDO_ACCESS_OK = 0,
    SDO_ACCESS_NO_INDEX = -1,
    SDO_ACCESS_READ_ONLY = -2,
    SDO_ACCESS_RANGE = -3
};

/* 前向声明：寄存器读写实现 */
static int8_t SDO_ReadRegister(SDO_Register reg, uint32_t *value);
static int8_t SDO_WriteRegister(SDO_Register reg, uint32_t value);
static void CANFD_ProcessRxMessage(const FDCAN_RxHeaderTypeDef *header,
                                   const uint8_t *data);

static uint32_t SDO_FloatToRaw(float value)
{
    uint32_t raw;
    memcpy(&raw, &value, sizeof(raw));
    return raw;
}

static float SDO_RawToFloat(uint32_t raw)
{
    float value;
    memcpy(&value, &raw, sizeof(value));
    return value;
}

static uint8_t SDO_FloatInRange(float value, float min_value, float max_value)
{
    return (isfinite(value) && value >= min_value && value <= max_value) ? 1U : 0U;
}

static float SDO_Clamp(float value, float min_value, float max_value)
{
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

static void SDO_SetPiLimit(PI_controller *pi, float limit)
{
    pi->Umax = limit;
    pi->Umin = -limit;
    pi->ui   = SDO_Clamp(pi->ui, pi->Umin, pi->Umax);
    pi->i1   = SDO_Clamp(pi->i1, pi->Umin, pi->Umax);
    pi->v1   = SDO_Clamp(pi->v1, pi->Umin, pi->Umax);
    pi->Out  = SDO_Clamp(pi->Out, pi->Umin, pi->Umax);
    pi->OutF = SDO_Clamp(pi->OutF, pi->Umin, pi->Umax);
}

/* ========== 模块内部变量 ========== */
static uint8_t canfd_node_state = NODE_STATE_OPERATIONAL;
uint32_t canfd_rx_cnt = 0;
uint32_t canfd_tx_cnt = 0;
uint32_t canfd_err_cnt = 0;
static const char canfd_sw_version_text[] = CANFD_SW_VERSION_TEXT;

const char *CANFD_GetSoftwareVersionText(void)
{
    return canfd_sw_version_text;
}

/* ========== SDO 读寄存器 ========== */
static int8_t SDO_ReadRegister(SDO_Register reg, uint32_t *value)
{
    if (value == NULL) return SDO_ACCESS_NO_INDEX;

    switch (reg) {
        case REG_SPEED_KP:      *value = SDO_FloatToRaw(pi_spd.Kp); break;
        case REG_SPEED_KI:      *value = SDO_FloatToRaw(pi_spd.Ki); break;
        case REG_SPEED_UMAX:    *value = SDO_FloatToRaw(pi_spd.Umax); break;
        case REG_ID_KP:         *value = SDO_FloatToRaw(pi_id.Kp); break;
        case REG_ID_KI:         *value = SDO_FloatToRaw(pi_id.Ki); break;
        case REG_ID_UMAX:       *value = SDO_FloatToRaw(pi_id.Umax); break;
        case REG_IQ_KP:         *value = SDO_FloatToRaw(pi_iq.Kp); break;
        case REG_IQ_KI:         *value = SDO_FloatToRaw(pi_iq.Ki); break;
        case REG_IQ_UMAX:       *value = SDO_FloatToRaw(pi_iq.Umax); break;
        case REG_PLL_KP:        *value = SDO_FloatToRaw(Foc_observer.PLL_kp); break;
        case REG_PLL_KI:        *value = SDO_FloatToRaw(Foc_observer.PLL_ki); break;
        case REG_OBS_GAIN:      *value = SDO_FloatToRaw(Foc_observer.Gain); break;
        case REG_OBS_BW:        *value = SDO_FloatToRaw(Foc_observer.PLL_BW_Hz); break;
        case REG_OC_THRESH:     *value = SDO_FloatToRaw(OC_THRESHOLD_A); break;
        case REG_OV_THRESH:     *value = SDO_FloatToRaw(OV_THRESHOLD_V); break;
        case REG_UV_THRESH:     *value = SDO_FloatToRaw(UV_THRESHOLD_V); break;
        case REG_OC_BUS_THRESH: *value = SDO_FloatToRaw(OC_BUS_THRESHOLD_A); break;
        case REG_OT_THRESH:     *value = SDO_FloatToRaw(NTC_OT_FAULT_C); break;
        case REG_VF_VOLT_MAX:   *value = SDO_FloatToRaw(VF_VOLTAGE_MAX); break;
        case REG_VF_FREQ_MAX:   *value = SDO_FloatToRaw(VF_FREQ_MAX); break;
        case REG_VF_RAMP_GRAD:  *value = SDO_FloatToRaw(VF_HzRamp.XieLv_Grad); break;
        case REG_DEVICE_ID:     *value = 0x20260718; break;
        case REG_SW_VERSION:    *value = CANFD_SW_VERSION_CODE; break;
        case REG_NODE_ID:       *value = CANFD_NODE_ID; break;
        case REG_PARAM_ID_STATE:
            /* 小端返回后Byte4~7正好依次为step/error/done/valid。 */
            *value = (uint32_t)pid.step |
                     ((uint32_t)pid.error << 8U) |
                     ((uint32_t)pid.done << 16U) |
                     ((uint32_t)pid.valid << 24U);
            break;
        case REG_PARAM_ID_RS:    *value = SDO_FloatToRaw(pid.Rs); break;
        case REG_PARAM_ID_LD:    *value = SDO_FloatToRaw(pid.Ld); break;
        case REG_PARAM_ID_LQ:    *value = SDO_FloatToRaw(pid.Lq); break;
        default: return SDO_ACCESS_NO_INDEX;
    }
    return SDO_ACCESS_OK;
}

/* ========== SDO 写寄存器 ========== */
static int8_t SDO_WriteRegister(SDO_Register reg, uint32_t value)
{
    float new_value = SDO_RawToFloat(value);

    switch (reg) {
        case REG_SPEED_KP:
            if (!SDO_FloatInRange(new_value, 0.0f, SDO_SPEED_KP_MAX))
                return SDO_ACCESS_RANGE;
            pi_spd.Kp = new_value;
            break;
        case REG_SPEED_KI:
            if (!SDO_FloatInRange(new_value, 0.0f, SDO_SPEED_KI_MAX))
                return SDO_ACCESS_RANGE;
            pi_spd.Ki = new_value;
            break;
        case REG_SPEED_UMAX:
            if (!SDO_FloatInRange(new_value, SDO_SPEED_UMAX_MIN_A,
                                  SDO_SPEED_UMAX_MAX_A))
                return SDO_ACCESS_RANGE;
            SDO_SetPiLimit(&pi_spd, new_value);
            break;
        case REG_ID_KP:
            if (!SDO_FloatInRange(new_value, 0.0f, SDO_CURRENT_KP_MAX))
                return SDO_ACCESS_RANGE;
            pi_id.Kp = new_value;
            break;
        case REG_ID_KI:
            if (!SDO_FloatInRange(new_value, 0.0f, SDO_CURRENT_KI_MAX))
                return SDO_ACCESS_RANGE;
            pi_id.Ki = new_value;
            break;
        case REG_IQ_KP:
            if (!SDO_FloatInRange(new_value, 0.0f, SDO_CURRENT_KP_MAX))
                return SDO_ACCESS_RANGE;
            pi_iq.Kp = new_value;
            break;
        case REG_IQ_KI:
            if (!SDO_FloatInRange(new_value, 0.0f, SDO_CURRENT_KI_MAX))
                return SDO_ACCESS_RANGE;
            pi_iq.Ki = new_value;
            break;
        case REG_PLL_KP:
            if (!SDO_FloatInRange(new_value, 0.0f, SDO_PLL_KP_MAX))
                return SDO_ACCESS_RANGE;
            Foc_observer.PLL_kp = new_value;
            Foc_observer.PLL_kp_target = new_value;
            break;
        case REG_PLL_KI:
            if (!SDO_FloatInRange(new_value, 0.0f, SDO_PLL_KI_MAX))
                return SDO_ACCESS_RANGE;
            Foc_observer.PLL_ki = new_value;
            Foc_observer.PLL_ki_target = new_value;
            break;
        case REG_OBS_GAIN:
            if (!SDO_FloatInRange(new_value, 0.0f, SDO_OBSERVER_GAIN_MAX))
                return SDO_ACCESS_RANGE;
            Foc_observer.Gain = new_value;
            break;
        case REG_VF_RAMP_GRAD:
            if (!SDO_FloatInRange(new_value, SDO_VF_RAMP_GRAD_MIN,
                                  SDO_VF_RAMP_GRAD_MAX))
                return SDO_ACCESS_RANGE;
            VF_HzRamp.XieLv_Grad = new_value;
            break;

        /* Constants, diagnostics, and dynamically managed limits are read-only. */
        case REG_ID_UMAX:
        case REG_IQ_UMAX:
        case REG_OBS_BW:
        case REG_OC_THRESH:
        case REG_OV_THRESH:
        case REG_UV_THRESH:
        case REG_OC_BUS_THRESH:
        case REG_OT_THRESH:
        case REG_VF_VOLT_MAX:
        case REG_VF_FREQ_MAX:
        case REG_DEVICE_ID:
        case REG_SW_VERSION:
        case REG_NODE_ID:
        case REG_PARAM_ID_STATE:
        case REG_PARAM_ID_RS:
        case REG_PARAM_ID_LD:
        case REG_PARAM_ID_LQ:
            return SDO_ACCESS_READ_ONLY;
        default:
            return SDO_ACCESS_NO_INDEX;
    }
    return SDO_ACCESS_OK;
}

/* ========== 字节数 → CAN FD DLC 码 ========== */
static uint32_t CANFD_BytesToDLC(uint32_t bytes)
{
    if (bytes <= 8)  return bytes;                  // DLC 0~8
    if (bytes <= 12) return FDCAN_DLC_BYTES_12;     // DLC 9
    if (bytes <= 16) return FDCAN_DLC_BYTES_16;     // DLC 10
    if (bytes <= 20) return FDCAN_DLC_BYTES_20;     // DLC 11
    if (bytes <= 24) return FDCAN_DLC_BYTES_24;     // DLC 12
    if (bytes <= 32) return FDCAN_DLC_BYTES_32;     // DLC 13
    if (bytes <= 48) return FDCAN_DLC_BYTES_48;     // DLC 14
    return FDCAN_DLC_BYTES_64;                       // DLC 15
}

/* ========== DLC 码 -> 实际接收字节数 ========== */
static uint32_t CANFD_DLCToBytes(uint32_t dlc)
{
    static const uint8_t dlc_to_bytes[16] = {
        0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64
    };

    return (dlc < 16U) ? dlc_to_bytes[dlc] : 0U;
}

/* ========== 发送 CAN FD 消息 ========== */
static int8_t CANFD_SendMessage(uint32_t id, uint8_t *data, uint32_t len, uint8_t is_fd)
{
    FDCAN_TxHeaderTypeDef txHeader = {0};
    txHeader.Identifier = id;
    txHeader.IdType = FDCAN_STANDARD_ID;
    txHeader.TxFrameType = FDCAN_DATA_FRAME;
    txHeader.DataLength = CANFD_BytesToDLC(len);
    txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    txHeader.BitRateSwitch = is_fd ? FDCAN_BRS_ON : FDCAN_BRS_OFF;
    txHeader.FDFormat = is_fd ? FDCAN_FD_CAN : FDCAN_CLASSIC_CAN;
    txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    txHeader.MessageMarker = 0;

    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txHeader, data) != HAL_OK) {
        canfd_err_cnt++;
        return -1;
    }
    canfd_tx_cnt++;
    return 0;
}

static uint32_t SDO_AccessToAbortCode(int8_t result)
{
    if (result == SDO_ACCESS_READ_ONLY) return SDO_ERR_READ_ONLY;
    if (result == SDO_ACCESS_RANGE) return SDO_ERR_RANGE;
    return SDO_ERR_NO_INDEX;
}

static uint32_t SDO_LoadU32(const uint8_t *data)
{
    uint32_t value;
    memcpy(&value, data, sizeof(value));
    return value;
}

static void SDO_StoreU32(uint8_t *data, uint32_t value)
{
    memcpy(data, &value, sizeof(value));
}

static void SDO_SendAbort(const uint8_t *request, uint32_t abort_code)
{
    uint8_t response[8] = {0};
    response[0] = SDO_ERROR;
    response[1] = request[1];
    response[2] = request[2];
    response[3] = request[3];
    SDO_StoreU32(&response[4], abort_code);
    CANFD_SendMessage(COB_SDO_TX, response, sizeof(response), 0);
}

/* ========== 发送 PDO1: 电机状态 ========== */
int8_t CANFD_PDO1_TX_Send(void)
{
    PDO1_TX_Data pdo;
    pdo.SpeedRPM     = motor.SpeedRPM;
    pdo.Fault_DTC    = motor.Fault_DTC;
    /* Control_Mode 按 motorControl.h 枚举上报，包含 I/F=6、HFI=7 启动骨架。 */
    pdo.Control_Mode = motor.Control_Mode;
    pdo.PwmEnabled   = Foc_Pwm_IsEnabled();

    return CANFD_SendMessage(COB_PDO1_TX, (uint8_t*)&pdo, sizeof(pdo), 0);
}

/* ========== 发送 PDO2: 扩展数据 ========== */
int8_t CANFD_PDO2_TX_Send(void)
{
    PDO2_TX_Data pdo;
    pdo.Ia   = Volt_CurrPara.PhaseU_Curr;
    pdo.Ib   = Volt_CurrPara.PhaseV_Curr;
    pdo.Ic   = Volt_CurrPara.PhaseW_Curr;
    pdo.Vbus = Volt_CurrPara.BUS_Voltage;
    pdo.Id   = PARK_PCurr.Ds;
    pdo.Iq   = PARK_PCurr.Qs;

    return CANFD_SendMessage(COB_PDO2_TX, (uint8_t*)&pdo, 8, 0);  // 经典CAN只发前8字节
}

/* ========== 发送心跳 ========== */
int8_t CANFD_Heartbeat_Send(void)
{
    Heartbeat_Data hb;
    hb.NodeState = canfd_node_state;
    return CANFD_SendMessage(COB_HEARTBEAT, (uint8_t*)&hb, sizeof(hb), 0);
}

/* ========== 发送紧急报文 ========== */
int8_t CANFD_EMCY_Send(uint16_t fault_code)
{
    uint8_t data[4];
    data[0] = (uint8_t)(fault_code & 0xFF);
    data[1] = (uint8_t)(fault_code >> 8);
    data[2] = 0x00;
    data[3] = 0x00;
    return CANFD_SendMessage(COB_EMCY, data, 4, 0);
}

/* ========== SDO 处理 ========== */
static void CANFD_ProcessSDO(const uint8_t *data, uint32_t len)
{
    if (len < 4) return;

    uint8_t  cmd = data[0];
    uint16_t reg = data[1] | (data[2] << 8);
    uint8_t  resp[8] = {0};
    uint32_t value = 0;
    int8_t   ret;

    if (data[3] != 0U) {
        SDO_SendAbort(data, SDO_ERR_NO_INDEX);
        return;
    }

    switch (cmd) {
        case SDO_READ_REQ:
            ret = SDO_ReadRegister((SDO_Register)reg, &value);
            if (ret != SDO_ACCESS_OK) {
                SDO_SendAbort(data, SDO_AccessToAbortCode(ret));
                return;
            }

            resp[0] = SDO_READ_RESP;
            resp[1] = data[1];
            resp[2] = data[2];
            resp[3] = data[3];
            SDO_StoreU32(&resp[4], value);
            CANFD_SendMessage(COB_SDO_TX, resp, sizeof(resp), 0);
            break;

        case SDO_WRITE32_REQ:
            if (len < 8U) {
                SDO_SendAbort(data, SDO_ERR_TYPE);
                return;
            }

            value = SDO_LoadU32(&data[4]);
            ret = SDO_WriteRegister((SDO_Register)reg, value);
            if (ret != SDO_ACCESS_OK) {
                SDO_SendAbort(data, SDO_AccessToAbortCode(ret));
                return;
            }

            resp[0] = SDO_WRITE_RESP;
            resp[1] = data[1];
            resp[2] = data[2];
            resp[3] = data[3];
            CANFD_SendMessage(COB_SDO_TX, resp, sizeof(resp), 0);
            break;

        case SDO_WRITE16_REQ:
        case SDO_WRITE8_REQ:
            SDO_SendAbort(data, SDO_ERR_TYPE);
            break;

        default:
            SDO_SendAbort(data, SDO_ERR_TYPE);
            break;
    }
}

/* ========== 处理一条已接收报文 ========== */
static void CANFD_ProcessRxMessage(const FDCAN_RxHeaderTypeDef *header,
                                   const uint8_t *data)
{
    uint32_t id;
    uint32_t len;

    if (header->IdType != FDCAN_STANDARD_ID ||
        header->RxFrameType != FDCAN_DATA_FRAME) {
        return;
    }

    id = header->Identifier;
    len = CANFD_DLCToBytes(header->DataLength);

    /* NMT */
    if (id == COB_NMT && len >= 2U) {
        uint8_t cmd  = data[0];
        uint8_t node = data[1];
        if (node == 0 || node == CANFD_NODE_ID) {
            if (cmd == NMT_START) {
                canfd_node_state = NODE_STATE_OPERATIONAL;
            } else if (cmd == NMT_STOP) {
                canfd_node_state = NODE_STATE_STOPPED;
            } else if (cmd == NMT_PRE_OP) {
                canfd_node_state = NODE_STATE_PRE_OP;
            }
            CANFD_OnNMT(cmd);
        }
        return;
    }

    /*
     * PDO1 RX: STOP/QUICK_STOP/FAULT_CLEAR 只需命令字节；START 必须带齐
     * mode + float。停止命令在任何 NMT 状态下都接受，以保证安全。
     */
    if (id == COB_PDO1_RX && len >= 1U) {
        PDO1_RX_Data pdo = {0};
        uint8_t cmd = data[0];

        if (cmd == CTRL_START) {
            if (canfd_node_state != NODE_STATE_OPERATIONAL ||
                len < sizeof(PDO1_RX_Data)) {
                return;
            }
            memcpy(&pdo, data, sizeof(pdo));
        } else if (cmd == CTRL_STOP || cmd == CTRL_QUICK_STOP ||
                   cmd == CTRL_FAULT_CLEAR) {
            pdo.Command = cmd;
        } else {
            return;
        }

        CANFD_OnPDO1_RX(&pdo);
        return;
    }

    /* PDO2 RX: 扩展指令 */
    if (id == COB_PDO2_RX && canfd_node_state == NODE_STATE_OPERATIONAL &&
        len >= sizeof(PDO2_RX_Data)) {
        PDO2_RX_Data pdo;
        memcpy(&pdo, data, sizeof(pdo));
        CANFD_OnPDO2_RX(&pdo);
        return;
    }

    /* SDO */
    if (id == COB_SDO_RX) {
        CANFD_ProcessSDO(data, len);
        return;
    }
}

/* ========== FDCAN 接收回调（重写 HAL 弱函数） ========== */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U) return;

    /* 一次中断排空 FIFO，避免连续控制帧到达时只处理第一条。 */
    while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0U) {
        FDCAN_RxHeaderTypeDef rx_header;
        uint8_t rx_data[64] = {0};

        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0,
                                   &rx_header, rx_data) != HAL_OK) {
            canfd_err_cnt++;
            break;
        }

        canfd_rx_cnt++;
        CANFD_ProcessRxMessage(&rx_header, rx_data);
    }
}

/* ========== 初始化 ========== */
void CANFD_App_Init(void)
{
    canfd_node_state = NODE_STATE_OPERATIONAL;
    canfd_rx_cnt = 0;
    canfd_tx_cnt = 0;
    canfd_err_cnt = 0;
}

/* ========== 周期处理（在任务中调用） ========== */
void CANFD_App_Process(void)
{
    // 目前不需要额外处理，收发都在中断里完成
    // 可在此添加超时检测、自动重发等逻辑
}
