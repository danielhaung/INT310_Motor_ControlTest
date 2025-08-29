import time
import serial

# ===== CRC16 =====
def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if (crc & 1) else crc >> 1
    return crc & 0xFFFF

def build_frame_no_crc(payload_without_crc: bytes) -> bytes:
    crc = crc16_modbus(payload_without_crc)
    return payload_without_crc + bytes([crc & 0xFF, (crc >> 8) & 0xFF])

def send_frame(ser: serial.Serial, raw_bytes: bytes, desc: str = ""):
    ser.write(raw_bytes)
    ser.flush()
    time.sleep(0.03)
    resp = b""
    if ser.in_waiting:
        resp = ser.read(ser.in_waiting)
    print(f"→ {desc:12s} | TX: {raw_bytes.hex(' ').upper()}  RX: {resp.hex(' ').upper()}")

def patch_slave_and_send(ser, frame_wo_crc_hex: str, slave_id: int, desc: str):
    parts = [int(x, 16) for x in frame_wo_crc_hex.strip().split()]
    parts[0] = slave_id
    payload = bytes(parts)
    frame = build_frame_no_crc(payload)
    send_frame(ser, frame, desc)

# ===== 小工具：多暫存器寫入 (0x10) =====
def frame_10_words(slave: int, start_addr: int, words: list[int]) -> bytes:
    qty = len(words)
    body = [
        slave, 0x10,
        (start_addr >> 8) & 0xFF, start_addr & 0xFF,
        (qty >> 8) & 0xFF, qty & 0xFF,
        qty * 2
    ]
    for w in words:
        body += [(w >> 8) & 0xFF, w & 0xFF]
    return build_frame_no_crc(bytes(body))

def int32_to_le_words(v: int) -> tuple[int, int]:
    u = v & 0xFFFFFFFF
    return (u & 0xFFFF, (u >> 16) & 0xFFFF)

# ======= 函式 =======
def init_speed_mode(
    ser: serial.Serial,
    slave_id: int,
    acc_dec: int = 0x01F4   # 預設 500；可自行改 2000 = 0x07D0 等
):
    """初始化：RS485控制、速度模式、加減速參數、始能"""

    # 1) RS485 控制：D110=30 (0x006E=0x001E)
    patch_slave_and_send(ser, "01 06 00 6E 00 1E", slave_id, "D110=30 (485控制)")
    time.sleep(0.05)

    # 2) 速度模式：D653=103 (0x028D=0x0067)
    patch_slave_and_send(ser, "01 06 02 8D 00 67", slave_id, "D653=103 (速度模式)")
    time.sleep(0.05)

    # 3) 加/減速：寫入 0x002A 與 0x002C，各兩個暫存器
    lo = (acc_dec >> 8) & 0xFF
    hi = acc_dec & 0xFF
    # 格式："01 10 00 2A 00 02 04 HH LL 00 00"
    hex_acc_a = f"01 10 00 2A 00 02 04 {acc_dec>>8:02X} {acc_dec&0xFF:02X} 00 00"
    hex_acc_b = f"01 10 00 2C 00 02 04 {acc_dec>>8:02X} {acc_dec&0xFF:02X} 00 00"
    patch_slave_and_send(ser, hex_acc_a, slave_id, f"加速設定 {acc_dec}")
    time.sleep(0.05)
    patch_slave_and_send(ser, hex_acc_b, slave_id, f"減速設定 {acc_dec}")
    time.sleep(0.05)

    # 4) 始能
    patch_slave_and_send(ser, "01 06 02 8C 00 0F", slave_id, "始能 ON")

def set_speed(ser: serial.Serial, rpm: int, slave_id: int, label: str | None = None):
    """
    例：
        set_speed(ser, 1000, 3)   -> 速度 1000 rpm, 站號3
        set_speed(ser, -500, 2)   -> 速度 -500 rpm, 站號2
    """
    SCALE = 5_000_000 / 3000.0  # ≈ 1666.666...
    internal_val = int(round(rpm * SCALE))

    lo, hi = int32_to_le_words(internal_val)
    frame = frame_10_words(slave_id, 0x028A, [lo, hi])
    desc = label if label is not None else f"速度 {rpm} rpm" #
    send_frame(ser, frame, desc)
    time.sleep(0.2)
