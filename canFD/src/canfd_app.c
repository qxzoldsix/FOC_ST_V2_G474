#include "canfd_app.h"

/* ========== SDO 寄存器读写表 ========== */
typedef struct {
    SDO_Register reg;
    void        *ptr;
    uint8_t      size;     // 1=u8, 2=u16, 4=float/u32
    uint8_t      readonly; // 1=只读
} SDO_RegEntry;

/* 前向声明：寄存器读写实现 */
static int8_t SDO_ReadRegister(SDO_Register reg, uint32_t *value);
static int8_t SDO_WriteRegister(SDO_Register reg, uint32_t value);

/* ========== 模块内部变量 ========== */
static uint8_t canfd_node_state = NODE_STATE_OPERATIONAL;
static uint32_t canfd_rx_cnt = 0;
static uint32_t canfd_tx_cnt = 0;
static uint32_t canfd_err_cnt = 0;

/* ========== SDO 寄存器写权限过滤 ========== */
static uint8_t SDO_IsReadOnly(SDO_Register reg)
{
    switch (reg) {
        case REG_DEVICE_ID:
        case REG_SW_VERSION:
            return 1;
        default:
            return 0;
    }
}

/* ========== SDO 读寄存器 ========== */
static int8_t SDO_ReadRegister(SDO_Register reg, uint32_t *value)
{
    if (value == NULL) return -1;

    switch (reg) {
        case REG_SPEED_KP:      *value = *(uint32_t*)&pi_spd.Kp;   break;
        case REG_SPEED_KI:      *value = *(uint32_t*)&pi_spd.Ki;   break;
        case REG_SPEED_UMAX:    *value = *(uint32_t*)&pi_spd.Umax; break;
        case REG_ID_KP:         *value = *(uint32_t*)&pi_id.Kp;    break;
        case REG_ID_KI:         *value = *(uint32_t*)&pi_id.Ki;    break;
        case REG_ID_UMAX:       *value = *(uint32_t*)&pi_id.Umax;  break;
        case REG_IQ_KP:         *value = *(uint32_t*)&pi_iq.Kp;    break;
        case REG_IQ_KI:         *value = *(uint32_t*)&pi_iq.Ki;    break;
        case REG_IQ_UMAX:       *value = *(uint32_t*)&pi_iq.Umax;  break;
        case REG_PLL_KP:        *value = *(uint32_t*)&VF_HzRamp.XieLv_Y; break; // TODO: 替换为实际PLL变量
        case REG_PLL_KI:        *value = *(uint32_t*)&VF_HzRamp.XieLv_X; break; // TODO: 替换为实际PLL变量
        case REG_OBS_GAIN:      *value = 5000; break;  // TODO: 替换为实际观测器增益
        case REG_OBS_BW:        *value = 50;   break;  // TODO: 替换为实际带宽
        case REG_OC_THRESH:     *value = *(uint32_t*)&(float){OC_THRESHOLD_A};     break;
        case REG_OV_THRESH:     *value = *(uint32_t*)&(float){OV_THRESHOLD_V};     break;
        case REG_UV_THRESH:     *value = *(uint32_t*)&(float){UV_THRESHOLD_V};     break;
        case REG_OC_BUS_THRESH: *value = *(uint32_t*)&(float){OC_BUS_THRESHOLD_A}; break;
        case REG_OT_THRESH:     *value = 0; break;  // TODO
        case REG_VF_VOLT_MAX:   *value = *(uint32_t*)&(float){VF_VOLTAGE_MAX};     break;
        case REG_VF_FREQ_MAX:   *value = *(uint32_t*)&(float){VF_FREQ_MAX};        break;
        case REG_VF_RAMP_GRAD:  *value = *(uint32_t*)&(float){VF_RAMP_GRAD_DEF};   break;
        case REG_DEVICE_ID:     *value = 0x20260718; break;
        case REG_SW_VERSION:    *value = 0x00010000; break;  // v1.0.0
        case REG_NODE_ID:       *value = CANFD_NODE_ID; break;
        default: return -1;
    }
    return 0;
}

/* ========== SDO 写寄存器 ========== */
static int8_t SDO_WriteRegister(SDO_Register reg, uint32_t value)
{
    if (SDO_IsReadOnly(reg)) return -2;

    switch (reg) {
        case REG_SPEED_KP:      pi_spd.Kp   = *(float*)&value; break;
        case REG_SPEED_KI:      pi_spd.Ki   = *(float*)&value; break;
        case REG_SPEED_UMAX:    pi_spd.Umax = *(float*)&value; break;
        case REG_ID_KP:         pi_id.Kp    = *(float*)&value; break;
        case REG_ID_KI:         pi_id.Ki    = *(float*)&value; break;
        case REG_ID_UMAX:       pi_id.Umax  = *(float*)&value; break;
        case REG_IQ_KP:         pi_iq.Kp    = *(float*)&value; break;
        case REG_IQ_KI:         pi_iq.Ki    = *(float*)&value; break;
        case REG_IQ_UMAX:       pi_iq.Umax  = *(float*)&value; break;
        case REG_PLL_KP:        VF_HzRamp.XieLv_Y = *(float*)&value; break; // TODO
        case REG_PLL_KI:        VF_HzRamp.XieLv_X = *(float*)&value; break; // TODO
        case REG_OC_THRESH:     // 当前为#define宏，暂存但不生效
        case REG_OV_THRESH:
        case REG_UV_THRESH:
        case REG_OC_BUS_THRESH:
        case REG_OT_THRESH:
        case REG_VF_VOLT_MAX:
        case REG_VF_FREQ_MAX:
        case REG_VF_RAMP_GRAD:
        case REG_OBS_GAIN:      // TODO: 替换为实际变量
        case REG_OBS_BW:        // TODO: 替换为实际变量
            break;  // 接受写入但不改变#define（后续改成变量即可生效）
        case REG_NODE_ID:       break; // 暂不支持运行时修改
        default: return -1;
    }
    return 0;
}

/* ========== 发送 CAN FD 消息 ========== */
static int8_t CANFD_SendMessage(uint32_t id, uint8_t *data, uint32_t len, uint8_t is_fd)
{
    FDCAN_TxHeaderTypeDef txHeader = {0};
    txHeader.Identifier = id;
    txHeader.IdType = FDCAN_STANDARD_ID;
    txHeader.TxFrameType = FDCAN_DATA_FRAME;
    txHeader.DataLength = len << 16;  // FDCAN_DLC_BYTES_xx
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

/* ========== 发送 PDO1: 电机状态 ========== */
int8_t CANFD_PDO1_TX_Send(void)
{
    PDO1_TX_Data pdo;
    pdo.SpeedRPM    = motor.SpeedRPM;
    pdo.CurrentHz   = motor.CurrentHz;
    pdo.V_amp       = motor.V_amp;
    pdo.TargetHz    = motor.TargetHz;
    pdo.TargetVolt  = motor.TargetVolt;
    pdo.Fault_DTC   = motor.Fault_DTC;
    pdo.Control_Mode = motor.Control_Mode;
    pdo.IQAngle     = motor.IQAngle;
    pdo.V_d         = motor.V_d;
    pdo.V_q         = motor.V_q;

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

    return CANFD_SendMessage(COB_PDO2_TX, (uint8_t*)&pdo, sizeof(pdo), 0);
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
static void CANFD_ProcessSDO(uint8_t *data, uint32_t len)
{
    if (len < 4) return;

    uint8_t  cmd = data[0];
    uint16_t reg = data[1] | (data[2] << 8);
    // uint8_t  sub = data[3];  // 暂未用

    uint8_t  resp[8] = {0};
    uint32_t value = 0;
    int8_t   ret;

    switch (cmd) {
        case SDO_READ_REQ:
            ret = SDO_ReadRegister((SDO_Register)reg, &value);
            if (ret == 0) {
                resp[0] = SDO_READ_RESP;
                resp[1] = data[1];
                resp[2] = data[2];
                resp[3] = 0;
                *(uint32_t*)&resp[4] = value;
                CANFD_SendMessage(COB_SDO_TX, resp, 8, 0);
            } else {
                resp[0] = SDO_ERROR;
                resp[1] = data[1];
                resp[2] = data[2];
                resp[3] = 0;
                *(uint32_t*)&resp[4] = (ret == -2) ? SDO_ERR_READ_ONLY : SDO_ERR_NO_INDEX;
                CANFD_SendMessage(COB_SDO_TX, resp, 8, 0);
            }
            break;

        case SDO_WRITE32_REQ:
        case SDO_WRITE16_REQ:
        case SDO_WRITE8_REQ:
            value = *(uint32_t*)&data[4];
            ret = SDO_WriteRegister((SDO_Register)reg, value);
            if (ret == 0) {
                resp[0] = SDO_WRITE_RESP;
                resp[1] = data[1];
                resp[2] = data[2];
                resp[3] = 0;
                CANFD_SendMessage(COB_SDO_TX, resp, 8, 0);
            } else {
                resp[0] = SDO_ERROR;
                resp[1] = data[1];
                resp[2] = data[2];
                resp[3] = 0;
                *(uint32_t*)&resp[4] = (ret == -2) ? SDO_ERR_READ_ONLY : SDO_ERR_NO_INDEX;
                CANFD_SendMessage(COB_SDO_TX, resp, 8, 0);
            }
            break;

        default:
            break;
    }
}

/* ========== FDCAN 接收回调（重写 HAL 弱函数） ========== */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0) return;

    FDCAN_RxHeaderTypeDef rxHeader;
    uint8_t rxData[64];

    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rxHeader, rxData) != HAL_OK) {
        canfd_err_cnt++;
        return;
    }

    canfd_rx_cnt++;
    uint32_t id = rxHeader.Identifier;

    /* NMT */
    if (id == COB_NMT && rxHeader.DataLength >= 2) {
        uint8_t cmd  = rxData[0];
        uint8_t node = rxData[1];
        if (node == 0 || node == CANFD_NODE_ID) {
            CANFD_OnNMT(cmd);
        }
        return;
    }

    /* PDO1 RX: 控制指令 */
    if (id == COB_PDO1_RX && rxHeader.DataLength >= sizeof(PDO1_RX_Data)) {
        PDO1_RX_Data *pdo = (PDO1_RX_Data*)rxData;
        CANFD_OnPDO1_RX(pdo);
        return;
    }

    /* PDO2 RX: 扩展指令 */
    if (id == COB_PDO2_RX && rxHeader.DataLength >= sizeof(PDO2_RX_Data)) {
        PDO2_RX_Data *pdo = (PDO2_RX_Data*)rxData;
        CANFD_OnPDO2_RX(pdo);
        return;
    }

    /* SDO */
    if (id == COB_SDO_RX) {
        CANFD_ProcessSDO(rxData, rxHeader.DataLength);
        return;
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
