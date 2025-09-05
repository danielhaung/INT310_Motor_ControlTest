import time
import serial

# ===== CRC16 =====
def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if (crc & 1) else (crc >> 1)
    return crc & 0xFFFF

def build_frame_no_crc(payload_without_crc: bytes) -> bytes:
    crc = crc16_modbus(payload_without_crc)
    return payload_without_crc + bytes([crc & 0xFF, (crc >> 8) & 0xFF])

# ===== TX helpers: 動態靜默 + 阻塞讀取 + 重試（預設安靜） =====
def _char_time_s(ser: serial.Serial) -> float:
    bits_per_char = 11 if ser.parity != serial.PARITY_NONE else 10
    return bits_per_char / max(ser.baudrate, 300)

def _read_exact(ser: serial.Serial, n: int, deadline: float) -> bytes:
    buf = bytearray()
    while len(buf) < n and time.time() < deadline:
        chunk = ser.read(n - len(buf))
        if chunk:
            buf += chunk
        else:
            time.sleep(0.0005)
    return bytes(buf)

def tx(
    ser: serial.Serial,
    raw_bytes: bytes,
    *,
    expect_len: int = 8,         # 0x06/0x10 常見回覆 8 bytes
    interframe_silent: float | None = None,  # None => 依鮑率估 3.5 chars
    timeout: float = 0.02,       # 單次等待時間
    retries: int = 1,            # 逾時重試次數
    debug: bool = False,
    desc: str = ""
) -> bytes:
    if interframe_silent is None:
        interframe_silent = max(4 * _char_time_s(ser), 0.0005)

    resp = b""
    attempt = 0
    while attempt <= retries:
        attempt += 1
        try:
            ser.write(raw_bytes)
            # ser.flush() # 一般不需要，遇到USB轉串口偶發再開
            time.sleep(interframe_silent)
            deadline = time.time() + timeout
            resp = _read_exact(ser, expect_len, deadline) if expect_len > 0 else b""
            if resp or expect_len == 0:
                if debug:
                    print(f"TX[{attempt}]: {desc} | {raw_bytes.hex(' ').upper()}  RX: {resp.hex(' ').upper()}")
                return resp
        except Exception as e:
            if debug:
                print(f"TX[{attempt}] error: {e}")

        # 清掉殘留資料，避免下次讀到舊回覆
        try:
            if ser.in_waiting:
                ser.read(ser.in_waiting)
        except Exception:
            pass
        time.sleep(0.001)

    if debug:
        print(f"TX FAIL: {desc} | {raw_bytes.hex(' ').upper()} (no response)")
    return resp

# 保持介面相容：send_frame 但改為呼叫 tx（預設不列印）
def send_frame(
    ser: serial.Serial,
    raw_bytes: bytes,
    desc: str = "",
    *,
    expect_len: int = 8,
    timeout: float = 0.02,
    retries: int = 1,
    debug: bool = False
) -> bytes:
    return tx(ser, raw_bytes, expect_len=expect_len, timeout=timeout,
              retries=retries, debug=debug, desc=desc)

def patch_slave_and_send(
    ser: serial.Serial,
    frame_wo_crc_hex: str,
    slave_id: int,
    desc: str,
    *,
    expect_len: int = 8,
    timeout: float = 0.02,
    retries: int = 1,
    debug: bool = False
) -> bytes:
    parts = [int(x, 16) for x in frame_wo_crc_hex.strip().split()]
    parts[0] = slave_id
    payload = bytes(parts)
    frame = build_frame_no_crc(payload)
    return send_frame(ser, frame, desc, expect_len=expect_len,
                      timeout=timeout, retries=retries, debug=debug)

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
    acc_dec: int = 0x01F4,   # 預設 500；可改 0x07D0=2000 等
    *,
    timeout: float = 0.02,
    retries: int = 1,
    debug: bool = False
):
    """初始化：RS485控制、速度模式、加減速參數、始能（高速節拍）"""

    # 1) RS485 控制：D110=30 (0x006E=0x001E)
    patch_slave_and_send(ser, "01 06 00 6E 00 1E", slave_id, "D110=30 (485控制)",
                         expect_len=8, timeout=timeout, retries=retries, debug=debug)

    # 2) 速度模式：D653=103 (0x028D=0x0067)
    patch_slave_and_send(ser, "01 06 02 8D 00 67", slave_id, "D653=103 (速度模式)",
                         expect_len=8, timeout=timeout, retries=retries, debug=debug)

    # 3) 加/減速：寫入 0x002A 與 0x002C，各兩個暫存器（功能碼0x10回覆亦為8 bytes）
    hex_acc_a = f"01 10 00 2A 00 02 04 {acc_dec>>8:02X} {acc_dec&0xFF:02X} 00 00"
    hex_acc_b = f"01 10 00 2C 00 02 04 {acc_dec>>8:02X} {acc_dec&0xFF:02X} 00 00"
    patch_slave_and_send(ser, hex_acc_a, slave_id, f"加速設定 {acc_dec}",
                         expect_len=8, timeout=timeout, retries=retries, debug=debug)
    patch_slave_and_send(ser, hex_acc_b, slave_id, f"減速設定 {acc_dec}",
                         expect_len=8, timeout=timeout, retries=retries, debug=debug)

    # 4) 始能
    patch_slave_and_send(ser, "01 06 02 8C 00 0F", slave_id, "始能 ON",
                         expect_len=8, timeout=timeout, retries=retries, debug=debug)

def set_speed(
    ser: serial.Serial,
    rpm: int,
    slave_id: int,
    label: str | None = None,
    *,
    timeout: float = 0.02,
    retries: int = 1,
    debug: bool = False,
    dwell: float = 0.0
):
    """
    例：
        set_speed(ser, 1000, 3)   -> 速度 1000 rpm, 站號3
        set_speed(ser, -500, 2)   -> 速度 -500 rpm, 站號2
    """
    SCALE = 5_000_000 / 3000.0  # ≈ 1666.666...
    internal_val = int(round(rpm * SCALE))
    lo, hi = int32_to_le_words(internal_val)

    frame = frame_10_words(slave_id, 0x028A, [lo, hi])  # D650/651 假設為速度命令暫存器
    desc = label if label is not None else f"速度 {rpm} rpm"
    send_frame(ser, frame, desc, expect_len=8, timeout=timeout, retries=retries, debug=debug)

    if dwell > 0:
        time.sleep(dwell)
