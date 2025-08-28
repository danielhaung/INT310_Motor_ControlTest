import time
import serial

# ===== 使用者需確認的連線參數 =====
PORT      = "/dev/ttyUSB0"                 # Windows: "COM6" / Linux: "/dev/ttyUSB0"
BAUDRATE  = 115200
PARITY    = serial.PARITY_EVEN     # 常見為 8E1；若現場是 8N1 => 改成 serial.PARITY_NONE
BYTESIZE  = serial.EIGHTBITS
STOPBITS  = serial.STOPBITS_ONE
SLAVE_ID  = 0x03                   # 站號 3

# ===== CRC16 (Modbus RTU, low byte first) =====
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

# ===== Frame builders =====
def frame_06(slave: int, addr: int, value: int) -> bytes:
    # Write Single Register
    p = bytes([
        slave,
        0x06,
        (addr >> 8) & 0xFF, addr & 0xFF,
        (value >> 8) & 0xFF, value & 0xFF
    ])
    return add_crc(p)

def frame_10_words(slave: int, start_addr: int, words: list[int]) -> bytes:
    # Preset Multiple Registers
    qty = len(words)
    byte_count = qty * 2
    body = [
        slave, 0x10,
        (start_addr >> 8) & 0xFF, start_addr & 0xFF,
        (qty >> 8) & 0xFF, qty & 0xFF,
        byte_count
    ]
    for w in words:
        body += [ (w >> 8) & 0xFF, w & 0xFF ]
    return add_crc(bytes(body))

def int32_to_le_words(v: int) -> tuple[int, int]:
    """
    依您提供封包格式：32位整數以「低字在前、 高字在後」兩個 16位暫存器寫入
    例如 +50000 (0x0000C350) -> [0xC350, 0x0000]
         -50000 (0xFFFF3CB0) -> [0x3CB0, 0xFFFF]
    """
    u = v & 0xFFFFFFFF
    lo = u & 0xFFFF
    hi = (u >> 16) & 0xFFFF
    return lo, hi

# ===== I/O =====
def tx(ser: serial.Serial, frame: bytes, tag: str):
    ser.write(frame); ser.flush()
    time.sleep(0.03)
    resp = b""
    if ser.in_waiting:
        resp = ser.read(ser.in_waiting)
    print(f"→ {tag:14s} | TX: {frame.hex(' ').upper()}  RX: {resp.hex(' ').upper()}")

if __name__ == "__main__":
    ser = serial.Serial(
        port=PORT, baudrate=BAUDRATE,
        bytesize=BYTESIZE, parity=PARITY, stopbits=STOPBITS,
        timeout=0.2
    )
    time.sleep(0.2)

    try:
        # 1) 設定 RS485 控制：D110=30 (addr 0x006E, val 0x001E)
        tx(ser, frame_06(SLAVE_ID, 0x006E, 0x001E), "RS485控制")

        # 2) 設置成 位置模式：D653=1 (addr 0x028D, val 0x0001)
        tx(ser, frame_06(SLAVE_ID, 0x028D, 0x0001), "位置模式")

        # 3) 設定加/減速（兩筆）
        #    您提供的「加減速 50」對應值 0x0350；「加減速 200」對應值 0x0D40
        ACC_DEC = 0x0350   # ← 想要 200 就改成 0x0D40
        tx(ser, frame_10_words(SLAVE_ID, 0x008A, [ACC_DEC, 0x0000]), "加速度設定")
        tx(ser, frame_10_words(SLAVE_ID, 0x008C, [ACC_DEC, 0x0000]), "減速度設定")

        # 4) 設定速度參數（示例：1000 rpm）
        #    依您提供樣本：addr 0x0084、兩個暫存器
        #    1000 rpm 範例 words = [0x6E6B, 0x0019]
        #    3000 rpm 範例 words = [0x4B40, 0x004C]
        SPEED_WORDS = [0x6E6B, 0x0019]   # ← 改成 [0x4B40, 0x004C] 即為 3000 rpm
        tx(ser, frame_10_words(SLAVE_ID, 0x0084, SPEED_WORDS), "速度參數")

        # 5) 觸發絕對位置：D28C 0x000F → 0x001F
        tx(ser, frame_06(SLAVE_ID, 0x028C, 0x000F), "始能(0x000F)")
        tx(ser, frame_06(SLAVE_ID, 0x028C, 0x001F), "觸發絕對位")

        # 6) 絕對位置動作：+50000 → 0 → −50000  (D130, addr 0x0082, 2 regs)
        def move_abs(pos: int, tag: str):
            lo, hi = int32_to_le_words(pos)
            tx(ser, frame_10_words(SLAVE_ID, 0x0082, [lo, hi]), tag)
            time.sleep(0.3)
        move_abs(     10000, "到 0")
        # 7) 斷始能
        #tx(ser, frame_06(SLAVE_ID, 0x028C, 0x0000), "始能 OFF")

        print("\n✅ 流程完成（站號 3，位置模式）。")

    finally:
        ser.close()
