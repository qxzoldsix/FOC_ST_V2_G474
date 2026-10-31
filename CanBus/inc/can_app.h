#ifndef CAN_APP_H
#define CAN_APP_H

/* CAN 应用协议层；当前使用经典 CAN 帧，底层由 STM32 FDCAN 外设承载。 */

#include "headline.h"

/* ========== 节点配置 ========== */
#define CAN_APP_NODE_ID       0x01
#define CAN_APP_SW_VERSION_CODE  0x0261025AUL

/* ========== CAN ID (11-bit) ========== */
#define COB_NMT             0x000           // 网络管理
#define COB_SYNC            0x080           // 同步帧
#define COB_EMCY            (0x080 + CAN_APP_NODE_ID)   // 紧急报文
#define COB_PDO1_TX         (0x180 + CAN_APP_NODE_ID)   // 电机→主机: 状态
#define COB_PDO1_RX         (0x200 + CAN_APP_NODE_ID)   // 主机→电机: 指令
#define COB_PDO2_TX         (0x280 + CAN_APP_NODE_ID)   // 电机→主机: 扩展数据
#define COB_PDO2_RX         (0x300 + CAN_APP_NODE_ID)   // 主机→电机: 扩展指令
#define COB_SDO_TX          (0x580 + CAN_APP_NODE_ID)   // SDO 应答
#define COB_SDO_RX          (0x600 + CAN_APP_NODE_ID)   // SDO 请求
#define COB_HEARTBEAT       (0x700 + CAN_APP_NODE_ID)   // 心跳

/* ========== NMT 命令 ========== */
#define NMT_START           0x01
#define NMT_STOP            0x02
#define NMT_PRE_OP          0x80
#define NMT_RESET           0x81

/* ========== 控制命令 ========== */
#define CTRL_STOP           0x00
#define CTRL_START          0x01
#define CTRL_FAULT_CLEAR    0x02
#define CTRL_QUICK_STOP     0x04

/* ========== 节点状态 ========== */
#define NODE_STATE_STOPPED      0x04
#define NODE_STATE_OPERATIONAL  0x05
#define NODE_STATE_PRE_OP       0x7F

/* ========== SDO 命令字 ========== */
#define SDO_READ_REQ        0x40
#define SDO_READ_RESP       0x43
#define SDO_WRITE32_REQ     0x23
#define SDO_WRITE16_REQ     0x2B
#define SDO_WRITE8_REQ      0x2F
#define SDO_WRITE_RESP      0x60
#define SDO_ERROR           0x80

/* ========== SDO 错误码 ========== */
#define SDO_ERR_NO_INDEX    0x06020000
#define SDO_ERR_READ_ONLY   0x06010002
#define SDO_ERR_TYPE        0x06070010
#define SDO_ERR_RANGE       0x06090030

/* ========== PDO1 TX: 经典 CAN 电机状态 (8字节, pack(1)) ========== */
#pragma pack(1)
typedef struct {
    float    SpeedRPM;        // [0:3] 实际转速 (RPM)
    uint16_t Fault_DTC;       // [4:5] 故障位掩码
    uint8_t  Control_Mode;    // [6]   当前控制模式
    uint8_t  PwmEnabled;      // [7]   0=封波, 1=PWM已使能
} PDO1_TX_Data;

/* ========== PDO1 RX: 控制指令 (6字节, 适配经典CAN ≤8字节) ========== */
typedef struct {
    uint8_t  Command;         // [0]     控制命令 (CTRL_START/STOP/...)
    uint8_t  Control_Mode;    // [1]     控制模式 (0停机 1有感 2无感直闭环 3VF 4I/F开环 5预定位 6辨识 7I/F启动 8HFI启动)
    float    TargetSpeedRPM;  // [2:5]   目标转速RPM；VF/I-F换算TargetHz，I-F允许负值
} PDO1_RX_Data;

/* ========== PDO2 TX: 扩展数据 (24字节) ========== */
typedef struct {
    float    Ia;              // U相电流
    float    Ib;              // V相电流
    float    Ic;              // W相电流
    float    Vbus;            // 母线电压
    float    Id;              // d轴电流
    float    Iq;              // q轴电流
} PDO2_TX_Data;

/* ========== PDO2 RX: 扩展指令 (8字节) ========== */
typedef struct {
    float    Id_Target;       // d轴目标电流
    float    Iq_Target;       // q轴目标电流
} PDO2_RX_Data;

/* ========== 心跳 (1字节) ========== */
typedef struct {
    uint8_t  NodeState;       // 节点状态
} Heartbeat_Data;
#pragma pack()

/* ========== SDO 寄存器索引 ========== */
typedef enum {
    REG_SPEED_KP      = 0x0000,
    REG_SPEED_KI      = 0x0001,
    REG_SPEED_UMAX    = 0x0002,
    REG_ID_KP         = 0x0003, // pu/A，电压基准见 ControlUnits.h
    REG_ID_KI         = 0x0004, // pu/(A·拍)，已包含离散周期
    REG_ID_UMAX       = 0x0005, // pu，只读，按母线更新
    REG_IQ_KP         = 0x0006, // pu/A
    REG_IQ_KI         = 0x0007, // pu/(A·拍)，已包含离散周期
    REG_IQ_UMAX       = 0x0008, // pu，只读，d 轴剩余电压裕量
    REG_PLL_KP        = 0x0009,
    REG_PLL_KI        = 0x000A,
    REG_OBS_GAIN      = 0x000B,
    REG_OBS_BW        = 0x000C,
    REG_OC_THRESH     = 0x000D,
    REG_OV_THRESH     = 0x000E,
    REG_UV_THRESH     = 0x000F,
    REG_OC_BUS_THRESH = 0x0010,
    REG_OT_THRESH     = 0x0011,
    REG_VF_VOLT_MAX   = 0x0012, // pu，只读，当前母线允许的矢量幅值
    REG_VF_FREQ_MAX   = 0x0013,
    REG_VF_RAMP_GRAD  = 0x0014,
    REG_DEVICE_ID     = 0x1000,  // 只读: 版本码（保留旧名称/地址，与 REG_SW_VERSION 同源）
    REG_SW_VERSION    = 0x1001,  // 只读: 软件版本
    REG_NODE_ID       = 0x1002,  // 节点ID
    REG_PARAM_ID_STATE = 0x1010, // 只读: step/error/done/valid
    REG_PARAM_ID_RS    = 0x1011, // 只读: Rs (ohm, float)
    REG_PARAM_ID_LD    = 0x1012, // 只读: Ld (H, float)
    REG_PARAM_ID_LQ    = 0x1013, // 只读: Lq (H, float)
} SDO_Register;

/* ========== API ========== */
void CAN_App_Init(void);
void CAN_App_Process(void);
int8_t CAN_App_PDO1_TX_Send(void);
int8_t CAN_App_PDO2_TX_Send(void);
int8_t CAN_App_Heartbeat_Send(void);
int8_t CAN_App_EMCY_Send(uint16_t fault_code);

/* 用户需要在应用层实现的回调 */
extern void CAN_App_OnNMT(uint8_t cmd);
extern void CAN_App_OnPDO1_RX(PDO1_RX_Data *data);
extern void CAN_App_OnPDO2_RX(PDO2_RX_Data *data);

#endif
