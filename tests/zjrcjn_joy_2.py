#!/usr/bin/env python3
# joystick_reader.py  (Modbus 版，可單檔測試)
# 介面相容：
#   init_joystick() -> js                 # js 為 serial.Serial 物件
#   read_joystick(js) -> (direction, movement, enable)
#
# 來源數據：讀取 D21..D27（D24~D27 = LX/LY/RX/RY，單位=原值/100）
# 映射（與 pygame 版一致）：
#   direction = RX -> 死區 -> * DIR_SCALE
#   movement  = LY  -> 反向 + 死區 -> * MOVE_SCALE
#   enable    = (D21.b7 == 1 無線OK) AND (D21.b4 == 1 非急停)

import time
import serial
import argparse
import sys

# ======= 參數（可由命令列覆寫） =======
MOVE_DEADZONE = 0.20   # 行進輪死區
DIR_DEADZONE  = 0.20   # 方向輪死區
MOVE_SCALE    = 800    # 行進輪倍率（速度用）
DIR_SCALE     = 1      # 方向輪倍率（目標位置量）

PORT      = "/dev/ttyUSB1"     # 例如 Windows 可用 "COM3"
BAUDRATE  = 9600
SLAVE_ID  = 2                  # 接收端站號
TIMEOUT_S = 0.2
HZ        = 5                  # 測試迴圈頻率
DEBUG     = False              # 顯示原始值與位元
# =====================================

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
    enable:     bool，(D21.b7 無線連線正常) AND (D21.b4 非急停)
    """
    # 準備請求
    frame = _frame_read_state_and_axes(SLAVE_ID)

    # 發送讀取
    try:
        js.reset_input_buffer()
        js.write(frame); js.flush()
        # 給裝置一點時間回應
        time.sleep(0.03)
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
    LX = _u16_to_s16(regs[3]) / 100.0  # D24
    LY = _u16_to_s16(regs[4]) / 100.0  # D25
    RX = _u16_to_s16(regs[5]) / 100.0  # D26
    # RY = _u16_to_s16(regs[6]) / 100.0  # D27（此版未用）

    # 軸位對齊（與 pygame 版）
    axis0_raw = RX          # 方向輪（右搖桿 X）
    axis1_raw = LY          # 行進輪（左搖桿 Y）

    # 死區
    dir_val  = 0.0 if -DIR_DEADZONE <= axis0_raw <= DIR_DEADZONE else axis0_raw
    move_val = -apply_deadzone(axis1_raw, MOVE_DEADZONE)  # **反向** 與 pygame 版一致

    # 倍率
    dir_val  *= DIR_SCALE
    move_val *= MOVE_SCALE

    # 未使能時輸出歸零（與 pygame 版按鈕未按下一致）
    if not enable:
        return 0.0, 0.0, False

    if DEBUG:
        # 顯示原始資訊供測試
        print(f"[DEBUG] D21=0x{d21:04X} (b7={b7}, b4={b4}) | "
              f"LX={LX:+.2f} LY={LY:+.2f} RX={RX:+.2f}",
              end="\r", flush=True)

    return float(dir_val), float(move_val), True

# ====== 測試主程式：可直接執行本檔 ======
def _parse_args():
    p = argparse.ArgumentParser(description="Modbus 遙控器測試（方向=RX、行進=LY）")
    p.add_argument("--port", default=PORT, help="序列埠 (預設: /dev/ttyUSB1 或 COM3)")
    p.add_argument("--baud", type=int, default=BAUDRATE, help="鮑率 (預設: 9600)")
    p.add_argument("--slave", type=int, default=SLAVE_ID, help="站號 (預設: 2)")
    p.add_argument("--timeout", type=float, default=TIMEOUT_S, help="逾時秒數 (預設: 0.2)")
    p.add_argument("--hz", type=float, default=HZ, help="取樣頻率 Hz (預設: 5)")
    p.add_argument("--move-deadzone", type=float, default=MOVE_DEADZONE, help="行進死區 (預設: 0.20)")
    p.add_argument("--dir-deadzone", type=float, default=DIR_DEADZONE, help="方向死區 (預設: 0.20)")
    p.add_argument("--move-scale", type=float, default=MOVE_SCALE, help="行進倍率 (預設: 800)")
    p.add_argument("--dir-scale", type=float, default=DIR_SCALE, help="方向倍率 (預設: 1)")
    p.add_argument("--debug", action="store_true", help="顯示 D21 與 LX/LY/RX 原始值")
    return p.parse_args()

def _apply_overrides(args):
    global PORT, BAUDRATE, SLAVE_ID, TIMEOUT_S, HZ
    global MOVE_DEADZONE, DIR_DEADZONE, MOVE_SCALE, DIR_SCALE, DEBUG
    PORT = args.port
    BAUDRATE = args.baud
    SLAVE_ID = args.slave
    TIMEOUT_S = args.timeout
    HZ = max(0.5, float(args.hz))
    MOVE_DEADZONE = float(args.move_deadzone)
    DIR_DEADZONE = float(args.dir_deadzone)
    MOVE_SCALE = float(args.move_scale)
    DIR_SCALE = float(args.dir_scale)
    DEBUG = bool(args.debug)

def _run_test():
    js = None
    try:
        js = init_joystick()
        print("Ctrl+C 結束測試\n")
        period = 1.0 / HZ
        while True:
            direction, movement, enable = read_joystick(js)
            # 顯示一行狀態（不會打斷 DEBUG 的即時列印）
            line = f"方向輪={direction:+.3f} | 行進輪={movement:+.1f} | 始能={'ON' if enable else 'OFF'}"
            print("\r" + line + " " * 12, end="", flush=True)
            time.sleep(period)
    except KeyboardInterrupt:
        print("\n結束")
    except serial.SerialException as e:
        print(f"\n❌ 串口錯誤：{e}")
        sys.exit(1)
    finally:
        if js is not None:
            try:
                js.close()
            except Exception:
                pass

if __name__ == "__main__":
    args = _parse_args()
    _apply_overrides(args)
    _run_test()
