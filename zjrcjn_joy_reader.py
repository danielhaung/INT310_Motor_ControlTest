#!/usr/bin/env python3
# joystick_reader.py  (Modbus 版) —— 與原 pygame 版相同介面：
#   init_joystick() -> js                 # 這裡 js 會是 serial.Serial 物件
#   read_joystick(js) -> (direction, movement, enable)
#
# 來源數據：讀取 D21..D27（D24~D27 = LX/LY/RX/RY，單位=原值/100）
# 映射：
#   direction = RX -> 死區 -> 階梯量化(含微滯回) -> * DIR_SCALE
#   movement  = LY  -> 反向 + 死區 -> * MOVE_SCALE
#   enable    = (D21.b7 == 1 無線OK) AND (D21.b4 == 1 非急停)
#
# 與 pygame 版一致的參數：
MOVE_DEADZONE = 0.20   # 行進輪死區
DIR_DEADZONE  = 0.20   # 方向輪死區
MOVE_SCALE    = 700    # 行進輪倍率（速度用）
DIR_SCALE     = 1      # 方向輪倍率（目標位置量）

# —— 方向輪階梯量化參數（可依需求調整）——
DIR_STEP_SPLIT = 1.0   # 分段臨界：|x| < 1.0 用小階；>=1.0 用大階
DIR_STEP_SMALL = 0.1   # 小階（|x| < 1.0）
DIR_STEP_LARGE = 1.0   # 大階（|x| >= 1.0）
HYSTERESIS_EPS = 0.05  # 微滯回帶寬（單位=軸值），避免臨界抖動

import time
import math
import serial

# ======= 請依現場調整 =======
PORT      = "/dev/ttyUSB2"     # 例如 Windows 可用 "COM3"
BAUDRATE  = 115200
SLAVE_ID  = 1                  # 接收端站號
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

# —— 往零截斷的階梯量化（含微滯回）——
_last_dir_val = 0.0  # 記錄上次方向輪量化後輸出（倍率前）

def _quantize_towards_zero(x: float, step: float) -> float:
    # 正數用 floor，負數用 ceil 的等價：結果一律朝 0 靠攏
    return math.copysign(math.floor(abs(x) / step) * step, x)

def _stair_quantize_rx_with_hysteresis(x: float) -> float:
    """
    先依 |x| 選階距，再做往零量化。若 x 落在上次輸出 ±HYSTERESIS_EPS 內，維持上次輸出避免抖動。
    注意：此函式應在倍率前呼叫（以軸值為單位比較滯回）。
    """
    global _last_dir_val

    # 若已落在死區後為 0，直接重置狀態避免殘留
    if x == 0.0:
        _last_dir_val = 0.0
        return 0.0

    step = DIR_STEP_SMALL if abs(x) < DIR_STEP_SPLIT else DIR_STEP_LARGE
    candidate = _quantize_towards_zero(x, step)

    # 若還在上一次輸出 ±ε 內，維持原值；否則更新為新階梯
    if abs(x - _last_dir_val) <= HYSTERESIS_EPS:
        return _last_dir_val
    else:
        _last_dir_val = candidate
        return candidate

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

    # —— 方向輪：死區 → 階梯量化(含微滯回) → 倍率 ——
    dir_val = 0.0 if -DIR_DEADZONE <= axis0_raw <= DIR_DEADZONE else axis0_raw
    dir_val = _stair_quantize_rx_with_hysteresis(dir_val)  # 階梯量化 + 滯回
    dir_val *= DIR_SCALE

    # —— 行進輪：反向 + 死區 → 倍率 ——
    move_val = -apply_deadzone(axis1_raw, MOVE_DEADZONE)  # **反向** 與 pygame 版一致
    move_val *= MOVE_SCALE

    # 未使能時輸出歸零（與 pygame 版按鈕未按下一致）
    if not enable:
        return 0.0, 0.0, False

    return float(dir_val), float(move_val), True
