import time
import serial

# ===== 使用者需確認的連線參數 =====
PORT = "/dev/ttyUSB0"          # Windows: "COM6" / Linux: "/dev/ttyUSB0"
BAUDRATE = 115200
PARITY = serial.PARITY_EVEN   # 多數伺服驅動器 Modbus RTU = 8E1
BYTESIZE = serial.EIGHTBITS
STOPBITS = serial.STOPBITS_ONE
SLAVE_ID = 0x03        # 站號3

# ===== CRC16 (Modbus RTU, Low-Byte first) =====
def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if (crc & 1) != 0:
                crc >>= 1
                crc ^= 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF

def build_frame_no_crc(payload_without_crc: bytes) -> bytes:
    crc = crc16_modbus(payload_without_crc)
    lo = crc & 0xFF
    hi = (crc >> 8) & 0xFF
    return payload_without_crc + bytes([lo, hi])

def send_frame(ser: serial.Serial, raw_bytes: bytes, desc: str = ""):
    ser.write(raw_bytes)
    ser.flush()
    # 視驅動器需求，可讀取回應；若不需要可省略
    time.sleep(0.03)
    resp = b""
    if ser.in_waiting:
        resp = ser.read(ser.in_waiting)
    print(f"→ {desc:12s} | TX: {raw_bytes.hex(' ').upper()}  RX: {resp.hex(' ').upper()}")

def patch_slave_and_send(ser, frame_wo_crc_hex: str, desc: str):
    """
    將您提供的「不含CRC」十六進位字串（含站號位元）改成 SLAVE_ID，重算CRC 後送出。
    frame_wo_crc_hex 例如: "01 06 00 6E 00 1E"（注意：最後兩個位元組 CRC 不要放進來）
    """
    parts = [int(x, 16) for x in frame_wo_crc_hex.strip().split()]
    parts[0] = SLAVE_ID
    payload = bytes(parts)
    frame = build_frame_no_crc(payload)
    send_frame(ser, frame, desc)

if __name__ == "__main__":
    # 開啟序列埠
    ser = serial.Serial(
        port=PORT, baudrate=BAUDRATE,
        bytesize=BYTESIZE, parity=PARITY, stopbits=STOPBITS,
        timeout=0.2
    )
    time.sleep(0.2)

    try:
        # === 1) 設定485模式 D110=30 (0x006E = 0x001E), function 0x06 ===
        # 範本(無CRC)："01 06 00 6E 00 1E"
        patch_slave_and_send(ser, "01 06 00 6E 00 1E", "D110=30")

        time.sleep(0.05)

        # === 2) 設成速度模式 D653=103 (0x028D = 0x0067), function 0x06 ===
        # 範本(無CRC)："01 06 02 8D 00 67"
        patch_slave_and_send(ser, "01 06 02 8D 00 67", "速度模式")

        time.sleep(0.05)

        # === 3) 設定加減速（兩個區塊） function 0x10, each writes 2 registers ===
        # 您提供的例子："01 10 00 2A 00 02 04 01 F4 00 00"  與  "01 10 00 2C 00 02 04 01 F4 00 00"
        # 01F4=500(僅示意)，您可依需求替換
        patch_slave_and_send(ser, "01 10 00 2A 00 02 04 01 F4 00 00", "加/減速參數A")
        time.sleep(0.05)
        patch_slave_and_send(ser, "01 10 00 2C 00 02 04 01 F4 00 00", "加/減速參數B")

        time.sleep(0.05)

        # === 4) 始能  D28C=0x000F（0~3通道始能） function 0x06 ===
        # 範本(無CRC)："01 06 02 8C 00 0F"
        patch_slave_and_send(ser, "01 06 02 8C 00 0F", "始能 ON")

        # === 5) 設定速度（多暫存器 0x028A），依您提供的實測封包 ===
        # *下列皆為「無CRC模板」，會自動改站號=3並重算CRC*
        def set_speed_from_template(hex_wo_crc: str, label: str):
            patch_slave_and_send(ser, hex_wo_crc, label)
            time.sleep(0.2)

        # 速度 = +1000
        # "01 10 02 8A 00 02 04 6E 6B 00 19"
        set_speed_from_template("01 10 02 8A 00 02 04 6E 6B 00 19", "速度 +1000")

        # 速度 = +500
        # "01 10 02 8A 00 02 04 B7 35 00 0C"
        set_speed_from_template("01 10 02 8A 00 02 04 B7 35 00 0C", "速度 +500")

        # 速度 = -500
        # "01 10 02 8A 00 02 04 48 CB FF F3"
        set_speed_from_template("01 10 02 8A 00 02 04 48 CB FF F3", "速度 -500")

        # 速度 = 0
        # "01 10 02 8A 00 02 04 00 00 00 00"
        set_speed_from_template("01 10 02 8A 00 02 04 00 00 00 00", "速度 0")

        # === 6) 斷始能 ===
        # 範本(無CRC)："01 06 02 8C 00 00"
        patch_slave_and_send(ser, "01 06 02 8C 00 00", "始能 OFF")

        print("\n✅ 流程完成（站號 3）。")

    finally:
        ser.close()