import time
import serial

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
        slave, 0x06,
        (addr >> 8) & 0xFF, addr & 0xFF,
        (value >> 8) & 0xFF, value & 0xFF
    ])
    return add_crc(p)

def frame_10_words(slave: int, start_addr: int, words: list[int]) -> bytes:
    # Preset Multiple Registers
    qty = len(words)
    body = [
        slave, 0x10,
        (start_addr >> 8) & 0xFF, start_addr & 0xFF,
        (qty >> 8) & 0xFF, qty & 0xFF,
        qty * 2
    ]
    for w in words:
        body += [(w >> 8) & 0xFF, w & 0xFF]
    return add_crc(bytes(body))

def int32_to_le_words(v: int) -> tuple[int, int]:
    """32-bit 整數以『低字在前』兩個 16-bit 暫存器表示。"""
    u = v & 0xFFFFFFFF
    lo = u & 0xFFFF
    hi = (u >> 16) & 0xFFFF
    return lo, hi

def rpm_to_pair(rpm: int) -> tuple[int, int]:
    """
    依您範例：1000 rpm -> [0x6E6B, 0x0019]；3000 rpm -> [0x4B40, 0x004C]
    內部值 = round(rpm * 5_000_000 / 3000) ，再轉低字在前。
    """
    SCALE = 5_000_000 / 3000.0  # ≈ 1666.666...
    internal = int(round(rpm * SCALE))
    return int32_to_le_words(internal)

def tx(ser: serial.Serial, frame: bytes, tag: str):
    ser.write(frame); ser.flush()
    time.sleep(0.03)
    resp = b""
    if ser.in_waiting:
        resp = ser.read(ser.in_waiting)
    print(f"→ {tag:14s} | TX: {frame.hex(' ').upper()}  RX: {resp.hex(' ').upper()}")

# ===== 1) 初始化：位置模式 + 速度/加減速 + 始能/觸發 =====
def init_position_mode(
    ser: serial.Serial,
    slave_id: int,
    acc_dec: int = 0x0350,   # 預設 50；改成 200 時請給 0x0D40
    speed_rpm: int = 1000,   # 速度參數 (rpm)
):
    """初始化：位置模式 + 加/減速 + 速度參數 + 始能/觸發"""

    # 1) RS485 控制：D110=30
    tx(ser, frame_06(slave_id, 0x006E, 0x001E), "RS485控制")
    time.sleep(0.05)

    # 2) 切換到位置模式：D653=1
    tx(ser, frame_06(slave_id, 0x028D, 0x0001), "位置模式")
    time.sleep(0.05)

    # 3) 加/減速設定：D0x008A、0x008C
    tx(ser, frame_10_words(slave_id, 0x008A, [acc_dec, 0x0000]), f"加速設定 {acc_dec}")
    time.sleep(0.05)
    tx(ser, frame_10_words(slave_id, 0x008C, [acc_dec, 0x0000]), f"減速設定 {acc_dec}")
    time.sleep(0.05)

    # 4) 速度參數 (rpm 轉換為 32bit 值後拆成 2 word)
    sp_lo, sp_hi = rpm_to_pair(speed_rpm)
    tx(ser, frame_10_words(slave_id, 0x0084, [sp_lo, sp_hi]), f"速度參數 {speed_rpm} rpm")



# ===== 便利小工具 =====
def enable_motor(ser: serial.Serial, slave_id: int, tag: str = "始能(0x000F)"):
    tx(ser, frame_06(slave_id, 0x028C, 0x000F), tag)

def trigger_absolute(ser: serial.Serial, slave_id: int, tag: str = "觸發絕對位"):
    tx(ser, frame_06(slave_id, 0x028C, 0x001F), tag)

def disable_motor(ser: serial.Serial, slave_id: int, tag: str = "始能 OFF"):
    tx(ser, frame_06(slave_id, 0x028C, 0x0000), tag)


# ===== 2) 後續下達絕對位置（D130/0x0082） =====
def move_to_position(
    ser: serial.Serial,
    pos_or_turns: float,        # 舊用法=counts；新用法=turns(若 use_turns=True)
    slave_id: int,
    label: str | None = None,
    do_trigger: bool = True,    # True：寫完位置後做 0x000F -> 0x001F 上升沿
    dwell: float = 0.3,         # 送完等待時間（秒）
    *,
    use_turns: bool = False,    # ← 設 True 表示以「圈數」為單位
    counts_per_rev: int = 10000,# ← 每圈對應的 counts（依您機型預設 10000）
    gear_ratio: float = 1.0     # ← 減速比：馬達:輸出 = gear_ratio:1（例 5:1 就填 5.0）
):
    """
    用法：
        # 舊：以 counts 直接下達（完全相容）
        move_to_position(ser, -10000, 3)

        # 新：以「圈數(輸出端)」下達，並可套用減速比
        move_to_position(ser, 1.0, 3, use_turns=True)                  # 輸出端 1 圈
        move_to_position(ser, -0.5, 3, use_turns=True)                 # 輸出端 -0.5 圈
        move_to_position(ser, 2.0, 3, use_turns=True, gear_ratio=5.0)  # 減速 5:1，輸出 2 圈
    """
    # 1) 依需求決定 counts
    if use_turns:
        counts = int(round(pos_or_turns * counts_per_rev * gear_ratio))
        auto_label = f"{pos_or_turns} rev (GR={gear_ratio} → {counts} counts)"
    else:
        counts = int(round(pos_or_turns))
        auto_label = f"{counts} counts"

    # 2) 組包（D130 / 0x0082，32-bit 低字在前）
    lo, hi = int32_to_le_words(counts)
    desc = label if label is not None else f"到 {auto_label}"
    tx(ser, frame_10_words(slave_id, 0x0082, [lo, hi]), desc)
    time.sleep(dwell)

    # 3) 可選：做一次觸發上升沿（0x000F -> 0x001F）
    if do_trigger:
        enable_motor(ser, slave_id, "觸發預備(0x000F)")
        trigger_absolute(ser, slave_id)


def immediate_stop_if_not_reached(ser: serial.Serial, slave_id: int, tag: str = "未達目標立即停止"):
    """
    設定：沒有到達目標立即停止
    等同於：01 06 02 8C 01 2F (自動帶入站號與CRC)
    """
    tx(ser, frame_06(slave_id, 0x028C, 0x012F), tag)