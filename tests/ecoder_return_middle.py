import time, serial, argparse
from typing import Tuple

# ===== 連線參數（依現場調整）=====
PORT      = "/dev/ttyUSB0"
BAUDRATE  = 115200           # 若是出廠預設，常見為 9600
PARITY    = serial.PARITY_EVEN   # 若現場是 8N1 -> 改 serial.PARITY_NONE
BYTESIZE  = serial.EIGHTBITS
STOPBITS  = serial.STOPBITS_ONE
SLAVE_ID  = 0x01             # BRT 編碼器預設站號常為 1；如已改過，請同步修改

# ===== BRT 編碼器寄存器（Modbus地址，十六進位）=====
ADDR_POS_32     = 0x0000     # 位置值 32-bit（讀兩個保持暫存器）
ADDR_ZERO_FLAG  = 0x0008     # 寫 0x0001 -> 以當前位置為零點
ADDR_MID_FLAG   = 0x000E     # 寫 0x0001 -> 以當前位置為中點

# ===== Modbus CRC16 (low byte first) =====
def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if (crc & 1) else (crc >> 1)
    return crc & 0xFFFF

def add_crc(p: bytes) -> bytes:
    c = crc16_modbus(p)
    return p + bytes([c & 0xFF, (c >> 8) & 0xFF])

# ===== 32-bit 整數與兩個16-bit字的互換（低字在前 / little-endian words）=====
def int32_to_le_words(v: int) -> Tuple[int, int]:
    u = v & 0xFFFFFFFF
    return (u & 0xFFFF), ((u >> 16) & 0xFFFF)

def le_words_to_int32(lo: int, hi: int) -> int:
    u = (hi << 16) | lo
    return u - 0x100000000 if (u & 0x80000000) else u

# ===== 基本封包 =====
def frame_read_words(slave: int, start_addr: int, qty: int) -> bytes:
    p = bytes([slave, 0x03, (start_addr >> 8) & 0xFF, start_addr & 0xFF,
               (qty >> 8) & 0xFF, qty & 0xFF])
    return add_crc(p)

def frame_write_single(slave: int, addr: int, value: int) -> bytes:
    # 功能碼 0x06：寫單一保持暫存器
    p = bytes([slave, 0x06, (addr >> 8) & 0xFF, addr & 0xFF,
               (value >> 8) & 0xFF, value & 0xFF])
    return add_crc(p)

def frame_write_multi(slave: int, start_addr: int, words: list[int]) -> bytes:
    qty = len(words)
    body = [slave, 0x10, (start_addr >> 8) & 0xFF, start_addr & 0xFF,
            (qty >> 8) & 0xFF, qty & 0xFF, qty * 2]
    for w in words:
        body += [(w >> 8) & 0xFF, w & 0xFF]
    return add_crc(bytes(body))

# ===== IO =====
def xfer(ser: serial.Serial, frame: bytes, tag: str, wait: float = 0.03) -> bytes:
    ser.reset_input_buffer()
    ser.write(frame); ser.flush()
    time.sleep(wait)
    resp = ser.read(ser.in_waiting or 256)
    print(f"→ {tag:14s} | TX: {frame.hex(' ').upper()}  RX: {resp.hex(' ').upper()}")
    return resp

# ===== 讀 BRT 編碼器 32-bit 位置值（地址 0x0000~0x0001）=====
def read_brt_position(ser: serial.Serial) -> int:
    resp = xfer(ser, frame_read_words(SLAVE_ID, ADDR_POS_32, 2), "READ POS32")
    # 期望回應: [id, 0x03, 0x04, hi0, lo0, hi1, lo1, crcL, crcH]
    if len(resp) < 7 or resp[0] != SLAVE_ID or resp[1] != 0x03 or resp[2] != 4:
        raise RuntimeError("讀取回應格式錯誤（POS32）")
    w_hi0 = (resp[3] << 8) | resp[4]
    w_hi1 = (resp[5] << 8) | resp[6]
    # BRT 文件的示例數據是高位在先，但 32-bit 數值邏輯為「低字在前」組合
    # 因此要用 w_lo = 第一字、w_hi = 第二字（依裝置回應格式而定）
    # 這裡沿用與你原程式一致的「低字在前」組合方式：
    w_lo = w_hi0
    w_hi = w_hi1
    return le_words_to_int32(lo=w_lo, hi=w_hi)

# ===== 設為零點 / 中點 =====
def set_zero(ser: serial.Serial):
    resp = xfer(ser, frame_write_single(SLAVE_ID, ADDR_ZERO_FLAG, 0x0001), "SET ZERO")
    if len(resp) < 6 or resp[0] != SLAVE_ID or resp[1] != 0x06:
        raise RuntimeError("設零點回應格式錯誤")

def set_midpoint(ser: serial.Serial):
    resp = xfer(ser, frame_write_single(SLAVE_ID, ADDR_MID_FLAG, 0x0001), "SET MID")
    if len(resp) < 6 or resp[0] != SLAVE_ID or resp[1] != 0x06:
        raise RuntimeError("設中點回應格式錯誤")

# ===== 主程式 =====
def main():
    ap = argparse.ArgumentParser(description="BRT RS485 Encoder: read/zero/midpoint.")
    ap.add_argument("--zero", action="store_true", help="將當前位置設為「零點」")
    ap.add_argument("--mid",  action="store_true", help="將當前位置設為「中點」")
    args = ap.parse_args()

    ser = serial.Serial(PORT, BAUDRATE, bytesize=BYTESIZE, parity=PARITY,
                        stopbits=STOPBITS, timeout=0.3, write_timeout=0.3)
    time.sleep(0.2)
    try:
        pos_before = read_brt_position(ser)
        print(f"📍 設定前位置 = {pos_before}")

        if args.zero:
            set_zero(ser)
        if args.mid:
            set_midpoint(ser)

        if args.zero or args.mid:
            time.sleep(0.05)
            pos_after = read_brt_position(ser)
            print(f"🔁 設定後位置 = {pos_after}")
        else:
            print("（僅讀取；若要設定請加 --zero 或 --mid）")

        print("✅ 完成")
    finally:
        ser.close()

if __name__ == "__main__":
    main()
