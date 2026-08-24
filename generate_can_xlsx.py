# -*- coding: utf-8 -*-
"""
根据 FOC_ST_V2 工程 canfd_app.h/c 中的报文定义，
生成 CAN 报文 xlsx 表格（含报文总览 / 报文示例 / SDO 寄存器 / 枚举常量）。

用法: python generate_can_xlsx.py
输出: CAN_Messages.xlsx
"""
import struct
from openpyxl import Workbook
from openpyxl.styles import Font, PatternFill, Alignment, Border, Side
from openpyxl.utils import get_column_letter

NODE_ID = 0x01

# ---------- 工具函数 ----------
def f32(v):
    """float32 -> 小端 16 进制字符串（两位、大写、空格分隔）"""
    b = struct.pack('<f', float(v))
    return ' '.join('%02X' % x for x in b)


def u16(v):
    return ' '.join('%02X' % x for x in struct.pack('<H', v & 0xFFFF))


def u32(v):
    return ' '.join('%02X' % x for x in struct.pack('<I', v & 0xFFFFFFFF))


def u8(v):
    return '%02X' % (v & 0xFF)


# ---------- 样式 ----------
HDR_FILL   = PatternFill('solid', fgColor='1F4E78')
HDR_FONT   = Font(name='微软雅黑', size=11, bold=True, color='FFFFFF')
BODY_FONT  = Font(name='微软雅黑', size=10)
MONO_FONT  = Font(name='Consolas', size=10)
NOTE_FONT  = Font(name='微软雅黑', size=9, italic=True, color='808080')
THIN       = Side(style='thin', color='BFBFBF')
BORDER     = Border(left=THIN, right=THIN, top=THIN, bottom=THIN)
CENTER     = Alignment(horizontal='center', vertical='center')
LEFT       = Alignment(horizontal='left', vertical='center', wrap_text=True)
DIR_TX_FILL = PatternFill('solid', fgColor='E2EFDA')   # 发送 淡绿
DIR_RX_FILL = PatternFill('solid', fgColor='FCE4D6')   # 接收 淡橙
ALT_FILL    = PatternFill('solid', fgColor='F2F2F2')


def setup_sheet(ws, headers, widths):
    for c, h in enumerate(headers, 1):
        cell = ws.cell(row=1, column=c, value=h)
        cell.font = HDR_FONT
        cell.fill = HDR_FILL
        cell.alignment = CENTER
        cell.border = BORDER
    for c, w in enumerate(widths, 1):
        ws.column_dimensions[get_column_letter(c)].width = w
    ws.freeze_panes = 'A2'
    ws.row_dimensions[1].height = 22


def write_row(ws, r, values, fonts=None, fills=None, aligns=None, alt=False):
    for c, v in enumerate(values, 1):
        cell = ws.cell(row=r, column=c, value=v)
        cell.font = (fonts[c-1] if fonts else BODY_FONT) if c != 1 else (fonts[c-1] if fonts else BODY_FONT)
        cell.alignment = (aligns[c-1] if aligns else LEFT)
        cell.border = BORDER
        if fills and fills[c-1]:
            cell.fill = fills[c-1]
        elif alt:
            cell.fill = ALT_FILL
        if isinstance(v, str) and v and c != 1 and not (fonts and fonts[c-1] is MONO_FONT):
            # 表格单元格里包含"0x"等文本默认左对齐
            pass


wb = Workbook()

# =====================================================================
# Sheet 1: 报文总览
# =====================================================================
ws1 = wb.active
ws1.title = '报文总览'
h1 = ['方向', '报文名称', 'ID (Hex)', 'ID (Dec)', '帧类型', 'DLC', '数据格式（字节）', '说明']
w1 = [8, 18, 12, 12, 12, 8, 40, 46]
setup_sheet(ws1, h1, w1)

tx_msgs = [
    ('EMCY 紧急报文', '0x%03X' % (0x080 + NODE_ID), 0x080 + NODE_ID, 4,
     '[0:1] 故障码 u16\n[2:3] 保留',
     '电机检测到故障时发送（过流/过压/欠压等）'),
    ('PDO1_TX 电机状态', '0x%03X' % (0x180 + NODE_ID), 0x180 + NODE_ID, 8,
     '[0:3] SpeedRPM  f32\n[4:7] CurrentHz  f32',
     '完整结构 33 字节，经典 CAN 仅发前 8 字节\n(后续字段: V_amp/TargetHz/TargetVolt/Fault_DTC/Control_Mode/IQAngle/V_d/V_q)'),
    ('PDO2_TX 扩展数据', '0x%03X' % (0x280 + NODE_ID), 0x280 + NODE_ID, 8,
     '[0:3] Ia  f32\n[4:7] Ib  f32',
     '完整结构 24 字节，经典 CAN 仅发前 8 字节\n(后续字段: Ic/Vbus/Id/Iq)'),
    ('SDO_TX 应答', '0x%03X' % (0x580 + NODE_ID), 0x580 + NODE_ID, 8,
     '[0]    cmd u8\n[1:2]  index u16\n[3]    sub u8\n[4:7]  value u32',
     '读/写寄存器应答、错误应答'),
    ('HEARTBEAT 心跳', '0x%03X' % (0x700 + NODE_ID), 0x700 + NODE_ID, 1,
     '[0] NodeState u8',
     '节点状态: 0x04 停止 / 0x05 运行 / 0x7F 预操作'),
]
rx_msgs = [
    ('NMT 网络管理', '0x%03X' % 0x000, 0x000, 2,
     '[0] cmd u8\n[1] node u8',
     'node=0 或本机节点 ID；启动/停止/预操作/复位'),
    ('SYNC 同步帧', '0x%03X' % 0x080, 0x080, 0,
     '无数据',
     '同步帧，可配合 PDO 同步发送'),
    ('PDO1_RX 控制指令', '0x%03X' % (0x200 + NODE_ID), 0x200 + NODE_ID, 6,
     '[0]    Command u8\n[1]    Control_Mode u8\n[2:5]  TargetSpeedRPM f32',
     '命令: 0x00停止/0x01启动/0x02清故障/0x04快速停止\n模式: 0停止/2无感FOC/3VF/4预定位'),
    ('PDO2_RX 扩展指令', '0x%03X' % (0x300 + NODE_ID), 0x300 + NODE_ID, 8,
     '[0:3] Id_Target f32\n[4:7] Iq_Target f32',
     'd/q 轴目标电流（需先进入相应控制模式）'),
    ('SDO_RX 请求', '0x%03X' % (0x600 + NODE_ID), 0x600 + NODE_ID, 8,
     '[0]    cmd u8\n[1:2]  index u16\n[3]    sub u8\n[4:7]  data u32',
     'cmd: 0x40读 / 0x23写32 / 0x2B写16 / 0x2F写8'),
]

r = 2
for d, fill in [('TX 发送', DIR_TX_FILL), ('RX 接收', DIR_RX_FILL)]:
    msgs = tx_msgs if d == 'TX 发送' else rx_msgs
    for name, idh, idd, dlc, fmt, desc in msgs:
        vals = [d, name, idh, idd, '数据帧', dlc, fmt, desc]
        fonts = [BODY_FONT, BODY_FONT, MONO_FONT, BODY_FONT, BODY_FONT, BODY_FONT, MONO_FONT, BODY_FONT]
        fills = [fill, None, None, None, None, None, None, None]
        aligns = [CENTER, LEFT, CENTER, CENTER, CENTER, CENTER, LEFT, LEFT]
        write_row(ws1, r, vals, fonts=fonts, fills=fills, aligns=aligns)
        ws1.cell(row=r, column=1).fill = fill
        r += 1

# =====================================================================
# Sheet 2: 报文示例（ID / DLC / Data）
# =====================================================================
ws2 = wb.create_sheet('报文示例')
h2 = ['方向', '场景', 'ID', 'DLC', 'Data', '说明']
w2 = [8, 26, 10, 8, 30, 52]
setup_sheet(ws2, h2, w2)

rx_examples = [
    ('NMT 启动电机',      '000', 2, '01 01',                'NMT 启动命令，目标节点=0x01'),
    ('NMT 停止电机',      '000', 2, '02 01',                'NMT 停止命令，目标节点=0x01'),
    ('NMT 预操作',        '000', 2, '80 01',                'NMT 预操作状态'),
    ('NMT 复位节点',      '000', 2, '81 01',                'NMT 复位命令'),
    ('启动 FOC(1500rpm)', '201', 6, '01 02 00 00 BB 44',    'CMD=启动, 模式=无感FOC, 1500.0rpm'),
    ('启动 VF 模式',      '201', 6, '01 03 00 00 00 00',    'CMD=启动, 模式=VF, 目标转速置0(按频率控制)'),
    ('停止电机',          '201', 6, '00 00 00 00 00 00',    'CMD=停止'),
    ('清除故障',          '201', 6, '02 00 00 00 00 00',    'CMD=清除故障'),
    ('快速停止',          '201', 6, '04 00 00 00 00 00',    'CMD=快速停止'),
    ('扩展指令 Id/Iq',    '301', 8, '00 00 80 3F 00 00 00 40', 'Id=1.0A, Iq=2.0A'),
    ('SDO 读 SpeedKp',    '601', 8, '40 00 00 00 00 00 00 00', '读 REG_SPEED_KP(0x0000)'),
    ('SDO 写 SpeedKp=10', '601', 8, '23 00 00 00 00 00 20 41', '写32位, 索引0x0000, 值=10.0f'),
    ('SDO 写 SpeedKi=1',  '601', 8, '23 01 00 00 00 00 80 3F', '写32位, 索引0x0001, 值=1.0f'),
]
tx_examples = [
    ('心跳-运行状态',     '701', 1, '05',                   'NodeState=0x05 Operational'),
    ('心跳-停止状态',     '701', 1, '04',                   'NodeState=0x04 Stopped'),
    ('EMCY 故障示例',     '081', 4, '0A 00 00 00',           '故障码=0x000A(示例)，故障码见 Fault_DTC'),
    ('PDO1 状态(1500rpm)', '181', 8, '00 80 BB 44 00 00 48 42', 'SpeedRPM=1500.0, CurrentHz=50.0'),
    ('PDO2 电流 Ia/Ib',   '281', 8, '00 00 80 3F 00 00 00 40', 'Ia=1.0A, Ib=2.0A'),
    ('SDO 读应答 SpeedKp','581', 8, '43 00 00 00 00 00 80 3F', '读应答, 索引0x0000, 值=1.0f'),
]

r = 2
for d, fill, examples in [('RX 接收', DIR_RX_FILL, rx_examples), ('TX 发送', DIR_TX_FILL, tx_examples)]:
    for name, idh, dlc, data, desc in examples:
        vals = [d, name, idh, dlc, data, desc]
        fonts = [BODY_FONT, BODY_FONT, MONO_FONT, BODY_FONT, MONO_FONT, BODY_FONT]
        aligns = [CENTER, LEFT, CENTER, CENTER, CENTER, LEFT]
        write_row(ws2, r, vals, fonts=fonts, aligns=aligns)
        ws2.cell(row=r, column=1).fill = fill
        r += 1

# =====================================================================
# Sheet 3: SDO 寄存器表
# =====================================================================
ws3 = wb.create_sheet('SDO寄存器')
h3 = ['寄存器', '索引 (Hex)', '索引 (Dec)', '读写', '类型', '说明']
w3 = [24, 12, 12, 10, 12, 40]
setup_sheet(ws3, h3, w3)

sdo_regs = [
    ('REG_SPEED_KP',      0x0000, '读写', 'f32', '速度环 Kp'),
    ('REG_SPEED_KI',      0x0001, '读写', 'f32', '速度环 Ki'),
    ('REG_SPEED_UMAX',    0x0002, '读写', 'f32', '速度环输出限幅'),
    ('REG_ID_KP',         0x0003, '读写', 'f32', 'd 轴电流环 Kp'),
    ('REG_ID_KI',         0x0004, '读写', 'f32', 'd 轴电流环 Ki'),
    ('REG_ID_UMAX',       0x0005, '读写', 'f32', 'd 轴电流环限幅'),
    ('REG_IQ_KP',         0x0006, '读写', 'f32', 'q 轴电流环 Kp'),
    ('REG_IQ_KI',         0x0007, '读写', 'f32', 'q 轴电流环 Ki'),
    ('REG_IQ_UMAX',       0x0008, '读写', 'f32', 'q 轴电流环限幅'),
    ('REG_PLL_KP',        0x0009, '读写', 'f32', 'PLL Kp'),
    ('REG_PLL_KI',        0x000A, '读写', 'f32', 'PLL Ki'),
    ('REG_OBS_GAIN',      0x000B, '读写', 'f32', '观测器增益'),
    ('REG_OBS_BW',        0x000C, '读写', 'f32', '观测器带宽'),
    ('REG_OC_THRESH',     0x000D, '读写', 'f32', '过流阈值 (A)'),
    ('REG_OV_THRESH',     0x000E, '读写', 'f32', '过压阈值 (V)'),
    ('REG_UV_THRESH',     0x000F, '读写', 'f32', '欠压阈值 (V)'),
    ('REG_OC_BUS_THRESH', 0x0010, '读写', 'f32', '母线过流阈值 (A)'),
    ('REG_OT_THRESH',     0x0011, '读写', 'f32', '过温阈值 (℃)'),
    ('REG_VF_VOLT_MAX',   0x0012, '读写', 'f32', 'VF 最大输出电压'),
    ('REG_VF_FREQ_MAX',   0x0013, '读写', 'f32', 'VF 最大输出频率'),
    ('REG_VF_RAMP_GRAD',  0x0014, '读写', 'f32', 'VF 斜坡梯度'),
    ('REG_DEVICE_ID',     0x1000, '只读', 'u32', '设备 ID'),
    ('REG_SW_VERSION',    0x1001, '只读', 'u32', '软件版本号'),
    ('REG_NODE_ID',       0x1002, '只读', 'u8',  '节点 ID（暂不支持运行时修改）'),
    ('REG_PARAM_ID_STATE',0x1010, '只读', 'u32', '参数辨识状态: [0]step [1]error [2]done [3]valid'),
    ('REG_PARAM_ID_RS',   0x1011, '只读', 'f32', '定子电阻 Rs (Ω)'),
    ('REG_PARAM_ID_LD',   0x1012, '只读', 'f32', 'd 轴电感 Ld (H)'),
    ('REG_PARAM_ID_LQ',   0x1013, '只读', 'f32', 'q 轴电感 Lq (H)'),
]
r = 2
for name, idx, rw, typ, desc in sdo_regs:
    ro = (rw == '只读')
    vals = [name, '0x%04X' % idx, idx, rw, typ, desc]
    fonts = [MONO_FONT, MONO_FONT, BODY_FONT, BODY_FONT, BODY_FONT, BODY_FONT]
    aligns = [LEFT, CENTER, CENTER, CENTER, CENTER, LEFT]
    write_row(ws3, r, vals, fonts=fonts, aligns=aligns)
    if ro:
        for c in range(1, 7):
            ws3.cell(row=r, column=c).fill = PatternFill('solid', fgColor='F2F2F2')
    r += 1

# =====================================================================
# Sheet 4: 枚举 / 常量表
# =====================================================================
ws4 = wb.create_sheet('常量表')
h4 = ['分类', '值 (Hex)', '值 (Dec)', '名称', '说明']
w4 = [16, 10, 12, 26, 40]
setup_sheet(ws4, h4, w4)

consts = [
    ('NMT 命令', '0x01', 1, 'NMT_START',   '进入 Operational（启动）'),
    ('NMT 命令', '0x02', 2, 'NMT_STOP',    '进入 Stopped（停止）'),
    ('NMT 命令', '0x80', 128, 'NMT_PRE_OP', '进入 Pre-Operational（预操作）'),
    ('NMT 命令', '0x81', 129, 'NMT_RESET',  '复位节点'),
    ('控制命令', '0x00', 0, 'CTRL_STOP',        '停止'),
    ('控制命令', '0x01', 1, 'CTRL_START',       '启动'),
    ('控制命令', '0x02', 2, 'CTRL_FAULT_CLEAR', '清除故障'),
    ('控制命令', '0x04', 4, 'CTRL_QUICK_STOP',  '快速停止'),
    ('控制模式', '0x00', 0, 'MODE_STOP',   '停止'),
    ('控制模式', '0x02', 2, 'MODE_FOC',    '无感 FOC'),
    ('控制模式', '0x03', 3, 'MODE_VF',     'VF 控制'),
    ('控制模式', '0x04', 4, 'MODE_PREPOS', '预定位'),
    ('节点状态', '0x04', 4, 'STOPPED',       'Stopped'),
    ('节点状态', '0x05', 5, 'OPERATIONAL',   'Operational'),
    ('节点状态', '0x7F', 127, 'PRE_OP',      'Pre-Operational'),
    ('SDO 命令', '0x40', 64, 'SDO_READ_REQ',   '读请求'),
    ('SDO 命令', '0x43', 67, 'SDO_READ_RESP',  '读应答'),
    ('SDO 命令', '0x23', 35, 'SDO_WRITE32_REQ','写 32 位请求'),
    ('SDO 命令', '0x2B', 43, 'SDO_WRITE16_REQ','写 16 位请求'),
    ('SDO 命令', '0x2F', 47, 'SDO_WRITE8_REQ', '写 8 位请求'),
    ('SDO 命令', '0x60', 96, 'SDO_WRITE_RESP', '写应答'),
    ('SDO 命令', '0x80', 128, 'SDO_ERROR',     '错误应答'),
]
r = 2
for cat, hv, dv, name, desc in consts:
    vals = [cat, hv, dv, name, desc]
    fonts = [BODY_FONT, MONO_FONT, BODY_FONT, MONO_FONT, BODY_FONT]
    aligns = [LEFT, CENTER, CENTER, LEFT, LEFT]
    write_row(ws4, r, vals, fonts=fonts, aligns=aligns)
    r += 1

# ---------- 保存 ----------
out = 'CAN_Messages.xlsx'
wb.save(out)
print('已生成:', out)
