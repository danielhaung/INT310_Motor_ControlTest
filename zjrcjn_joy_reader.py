#!/usr/bin/env python3
# joystick_reader.py  (Modbus 版) —— 與原 pygame 版相同介面：
#   init_joystick() -> js                 # 這裡 js 會是 serial.Serial 物件
#   read_joystick(js) -> (direction, movement, enable)
#
# 來源數據：讀取 D21..D27（D24~D27 = LX/LY/RX/RY，單位=原值/100）
# 映射：
#   direction = RX -> 死區 -> * DIR_SCALE
#   movement  = LY  -> 反向 + 死區 -> * MOVE_SCALE
#   enable    = (D21.b7 == 1 無線OK) AND (D21.b4 == 1 非急停)
#
# 與 pygame 版一致的參數：
MOVE_DEADZONE = 0.20   # 行進輪死區
DIR_DEADZONE  = 0.20   # 方向輪死區
MOVE_SCALE    = 500    # 行進輪倍率（速度用）
DIR_SCALE     = 1      # 方向輪倍率（目標位置量）

import time
import serial

# ======= 請依現場調整 =======
PORT      = "/dev/ttyUSB1"     # 例如 Windows 可用 "COM3"
BAUDRATE  = 9600
SLAVE_ID  = 2                  # 接收端站號
TIMEOUT_S = 0.2
# ===========================

# --- CRC16(Modbus) ---
def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF

def add_crc(p: bytes) -> bytes:
    c = crc16_modbus(p)
    return p + bytes([c & 0xFF, (c >> 8) & 0xFF])

# 讀取 D21~D27 (共 7 字)
def _frame_read_state_and_axes(slave: int) -> bytes:
    start_addr = 21  # D21
    qty = 7         # D21..D27
    p = bytes([slave, 0x03,
               (start_addr >> 8) & 0xFF, start_addr & 0xFF,
               (qty >> 8) & 0xFF, qty & 0xFF])
    return add_crc(p)

def _u16_to_s16(v: int) -> int:
    return v - 0x10000 if (v & 0x8000) else v

def apply_deadzone(v: float, dz: float) -> float:
    return 0.0 if abs(v) < dz else v

# ===== 對外：與 pygame 版一致的介面 =====

def init_joystick():
    """
    初始化“搖桿”（實際為串口裝置）。
    回傳值 js：serial.Serial 物件（供 read_joystick 使用）
    """
    ser = serial.Serial(PORT, BAUDRATE,
                        bytesize=serial.EIGHTBITS,
                        parity=serial.PARITY_NONE,
                        stopbits=serial.STOPBITS_ONE,
                        timeout=TIMEOUT_S, write_timeout=TIMEOUT_S)
    # 緩衝清空、裝置穩定
    time.sleep(0.2)
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    print(f"✅ 使用 Modbus 遙控器於 {PORT} @ {BAUDRATE} bps，Slave={SLAVE_ID}")
    return ser

def read_joystick(js):
    """
    與原 pygame 版相同回傳：
      return (direction, movement, enable)
    direction:  方向輪目標量（這裡用 RX）
    movement:   行進輪速度量（這裡用 LY，含反向、死區、倍率）
    enable:     bool，(無線連線正常 & 非急停)
    """
    # 準備請求
    frame = _frame_read_state_and_axes(SLAVE_ID)

    # 發送讀取
    try:
        js.reset_input_buffer()
        js.write(frame); js.flush()
        time.sleep(0.03)  # 等回應
        resp = js.read(js.in_waiting or 256)
    except Exception:
        # 通訊異常時視為未使能、輸出 0
        return 0.0, 0.0, False

    # 期望：ID(1)+FUNC(1)+BYTECNT(1)+7*2(=14)+CRC(2)=19 bytes
    if not (len(resp) >= 19 and resp[0] == SLAVE_ID and resp[1] == 0x03 and resp[2] >= 14):
        return 0.0, 0.0, False

    # CRC 驗證
    body = resp[:-2]
    crc_lo, crc_hi = resp[-2], resp[-1]
    c = crc16_modbus(body)
    if (c & 0xFF) != crc_lo or ((c >> 8) & 0xFF) != crc_hi:
        return 0.0, 0.0, False

    # 解析 7 個暫存器（u16）
    regs = []
    for i in range(7):
        hi = resp[3 + i*2]
        lo = resp[4 + i*2]
        regs.append((hi << 8) | lo)

    d21 = regs[0]
    # b4: 急停狀態(1=非急停), b7: 無線連線(1=正常)
    b4 = (d21 >> 4) & 1
    b7 = (d21 >> 7) & 1
    enable = bool(b4 and b7)

    # D24..D27：LX/LY/RX/RY（原始為 s16，/100 後得到 -1.00~+1.00 類型的軸值）
    # 與 pygame 版軸位對齊：
    #   axis0_raw = 方向輪 = RX
    #   axis1_raw = 行進輪 = LY（pygame 版會做反向，這裡也一致做反向）
    LX = _u16_to_s16(regs[3]) / 100.0  # D24
    LY = _u16_to_s16(regs[4]) / 100.0  # D25
    RX = _u16_to_s16(regs[5]) / 100.0  # D26
    # RY = _u16_to_s16(regs[6]) / 100.0  # D27（此版未用）

    axis0_raw = RX          # 方向輪
    axis1_raw = LY          # 行進輪（將在下方反向）

    # 死區
    dir_val  = 0.0 if -DIR_DEADZONE <= axis0_raw <= DIR_DEADZONE else axis0_raw
    move_val = -apply_deadzone(axis1_raw, MOVE_DEADZONE)  # **反向** 與 pygame 版一致

    # 倍率
    dir_val  *= DIR_SCALE
    move_val *= MOVE_SCALE

    # 未使能時輸出歸零（與 pygame 版按鈕未按下一致）
    if not enable:
        return 0.0, 0.0, False

    return float(dir_val), float(move_val), True
