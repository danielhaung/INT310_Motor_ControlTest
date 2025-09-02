import time, serial, argparse
from typing import Tuple

# ===== 連線參數 =====
PORT      = "/dev/ttyUSB0"            # Windows: "COM6" / Linux: "/dev/ttyUSB0"
BAUDRATE  = 115200
PARITY    = serial.PARITY_EVEN        # 現場若是 8N1 -> 改 serial.PARITY_NONE
BYTESIZE  = serial.EIGHTBITS
STOPBITS  = serial.STOPBITS_ONE
SLAVE_ID  = 0x03                      # 站號 3

# ===== 寄存器位址 =====
ADDR_ENC_POS_DEC = 616                # 電機位置 (RW), 長度 4 bytes
ADDR_ENC_POS     = 0x0268            # 616 的十六進位位址

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

# ===== 編碼器值 <-> 兩個16位暫存器 (低字在前 / little-endian words) =====
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

# ===== 讀 / 寫 編碼器位置 (DEC 616, 32-bit) =====
def read_encoder_pos(ser: serial.Serial) -> int:
    resp = xfer(ser, frame_read_words(SLAVE_ID, ADDR_ENC_POS, 2), "READ ENC616")
    if len(resp) < 7 or resp[0] != SLAVE_ID or resp[1] != 0x03 or resp[2] != 4:
        raise RuntimeError("讀取回應格式錯誤")
    w0 = (resp[3] << 8) | resp[4]  # 第1暫存器(高位在前) -> 組成 word
    w1 = (resp[5] << 8) | resp[6]  # 第2暫存器
    # 32-bit由「低字在前」組成
    return le_words_to_int32(lo=w0, hi=w1)

def write_encoder_pos(ser: serial.Serial, value: int):
    lo, hi = int32_to_le_words(value)
    xfer(ser, frame_write_multi(SLAVE_ID, ADDR_ENC_POS, [lo, hi]), "WRITE ENC616")

# ===== 主程式 =====
def main():
    ap = argparse.ArgumentParser(description="Read/Write encoder position (DEC 616).")
    ap.add_argument("--set", type=int, default=None,
                    help="要寫入的編碼器位置 (32-bit, 可負值)。例如 --set 0 代表把當前點設為原點。")
    args = ap.parse_args()

    ser = serial.Serial(PORT, BAUDRATE, bytesize=BYTESIZE, parity=PARITY,
                        stopbits=STOPBITS, timeout=0.2, write_timeout=0.2)
    time.sleep(0.2)
    try:
        cur = read_encoder_pos(ser)
        print(f"📍 目前編碼器位置 = {cur}")

        if args.set is not None:
            print(f"✍️  寫入編碼器位置 = {args.set}")
            write_encoder_pos(ser, args.set)
            time.sleep(0.05)
            back = read_encoder_pos(ser)
            print(f"🔁 回讀確認位置 = {back}")
            print("✅ 寫入成功！" if back == args.set else "⚠️ 寫入後回讀不一致，請檢查參數/使能狀態。")
        else:
            print("（僅讀取；若要寫入請加 --set <值>，例如 --set 0）")
    finally:
        ser.close()

if __name__ == "__main__":
    main()
