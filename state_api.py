# state_api.py
# 監視：實際速度 + 電機位置（以手冊表：MODBUS(DEC) 616=位置、630=實際速度）
# 僅定義函式；匯入時不會自動執行。

import time
import serial
from typing import Optional, List, Dict

# =========================
# 常數（手冊映射）
# =========================
# 各占 4 bytes (32-bit)，以兩個 16-bit 暫存器表示
ADDR_POS_LE_DEC       = 616  # 電機位置（起始暫存器，低位在前）
ADDR_ACT_SPEED_LE_DEC = 630  # 實際速度（起始暫存器，低位在前）

# 若實際速度單位需換算為 counts/sec，部分機型標示命令速度為 0.1 counts/sec。
# 這裡預設同樣以 0.1 進行換算；如不需要，將 scale 改為 1.0。
DEFAULT_SPEED_SCALE = 0.1  # 內部值 * 0.1 -> counts/sec（可依實機修改）

# =========================
# 基礎：CRC / 建包 / 收發
# =========================
def crc16_modbus(data: bytes) -> int:
    """Modbus RTU CRC16（低位在前）。"""
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if (crc & 1) else crc >> 1
    return crc & 0xFFFF

def _build_read_frame(slave: int, start_addr: int, qty_words: int) -> bytes:
    body = bytes([
        slave & 0xFF, 0x03,
        (start_addr >> 8) & 0xFF, start_addr & 0xFF,
        (qty_words  >> 8) & 0xFF, qty_words  & 0xFF
    ])
    crc = crc16_modbus(body)
    return body + bytes([crc & 0xFF, (crc >> 8) & 0xFF])

def _expect_len_for_words(qty_words: int) -> int:
    # slave(1)+func(1)+byte_count(1)+data(2*qty)+CRC(2)
    return 1 + 1 + 1 + (2 * qty_words) + 2

def _send_and_recv(
    ser: serial.Serial,
    frame: bytes,
    expected_bytes: Optional[int] = None,
    read_timeout_s: float = 0.15,
    post_delay_s: float = 0.03
) -> bytes:
    ser.reset_input_buffer()
    ser.write(frame)
    ser.flush()
    time.sleep(post_delay_s)

    deadline = time.time() + read_timeout_s
    buf = bytearray()
    while time.time() < deadline:
        if ser.in_waiting:
            buf += ser.read(ser.in_waiting)
            if expected_bytes is not None and len(buf) >= expected_bytes:
                break
        else:
            time.sleep(0.004)
    return bytes(buf)

def _parse_read_resp(resp: bytes, qty_words: int) -> Optional[List[int]]:
    if len(resp) < 5:
        return None
    if resp[1] != 0x03:
        return None
    byte_count = resp[2]
    expect = 3 + byte_count + 2
    if len(resp) < expect or byte_count != qty_words * 2:
        return None
    # CRC 驗證
    payload = resp[:-2]
    crc_lo, crc_hi = resp[-2], resp[-1]
    if ((crc_hi << 8) | crc_lo) != crc16_modbus(payload):
        return None
    data = resp[3:3 + byte_count]
    words: List[int] = []
    for i in range(0, byte_count, 2):
        words.append((data[i] << 8) | data[i + 1])
    return words if len(words) == qty_words else None

# =========================
# 讀取原語：words / int32(LE)
# =========================
def _read_words(
    ser: serial.Serial,
    slave: int,
    start_addr: int,
    qty_words: int,
    timeout_s: float = 0.15
) -> Optional[List[int]]:
    frame = _build_read_frame(slave, start_addr, qty_words)
    rx = _send_and_recv(ser, frame, expected_bytes=_expect_len_for_words(qty_words), read_timeout_s=timeout_s)
    return _parse_read_resp(rx, qty_words)

def _le_words_to_i32(lo: int, hi: int) -> int:
    u32 = ((hi & 0xFFFF) << 16) | (lo & 0xFFFF)
    return u32 - 0x1_0000_0000 if (u32 & 0x8000_0000) else u32

def _read_int32_le(
    ser: serial.Serial,
    slave: int,
    start_addr_le: int,
    timeout_s: float = 0.15
) -> Optional[int]:
    ws = _read_words(ser, slave, start_addr_le, 2, timeout_s=timeout_s)
    if not ws:
        return None
    return _le_words_to_i32(ws[0], ws[1])

# =========================
# 高階 API（位址已內建）
# =========================
def read_motor_position_counts(
    ser: serial.Serial,
    slave: int,
    timeout_s: float = 0.15
) -> Optional[int]:
    """
    讀「電機位置」32-bit 計數（手冊 MODBUS(DEC)=616）。
    回傳 int32（可能為 encoder counts / pulse），失敗回 None。
    """
    return _read_int32_le(ser, slave, ADDR_POS_LE_DEC, timeout_s=timeout_s)

def read_actual_speed_internal(
    ser: serial.Serial,
    slave: int,
    timeout_s: float = 0.15
) -> Optional[int]:
    """
    讀「實際速度」內部 32-bit 數值（手冊 MODBUS(DEC)=630），未換算。
    回傳 int32，失敗回 None。
    """
    return _read_int32_le(ser, slave, ADDR_ACT_SPEED_LE_DEC, timeout_s=timeout_s)

def read_actual_speed_counts_per_sec(
    ser: serial.Serial,
    slave: int,
    timeout_s: float = 0.15,
    scale: float = DEFAULT_SPEED_SCALE
) -> Optional[float]:
    """
    讀「實際速度」並換算為 counts/sec。
    預設 scale=0.1（若實機回饋已是 counts/sec，請改為 1.0）。
    """
    iv = read_actual_speed_internal(ser, slave, timeout_s=timeout_s)
    if iv is None:
        return None
    return float(iv) * float(scale)

def counts_to_turns(counts: int, pulses_per_rev: int) -> float:
    """計數轉圈數。"""
    return 0.0 if pulses_per_rev == 0 else counts / float(pulses_per_rev)

def cps_to_rpm(cps: float, pulses_per_rev: int) -> float:
    """counts/sec 轉 rpm。"""
    if pulses_per_rev == 0:
        return 0.0
    rps = cps / float(pulses_per_rev)
    return rps * 60.0

# state_api.py （調整 read_state_once 的介面與內部計算）
def read_state_once(
    ser: serial.Serial,
    slave: int,
    *,
    timeout_s: float = 0.15,
    speed_scale: float = DEFAULT_SPEED_SCALE,
    pulses_per_rev: int | None = None,
    angle_degrees: bool = True,     # ← 新增：True 則 position_turns 以角度輸出
) -> Dict[str, Optional[float]]:
    """
    一次回傳：
      - actual_speed_cps: 實際速度（counts/sec，依 scale 換算）
      - actual_speed_rpm: 若提供 pulses_per_rev，會轉 rpm；否則為 None
      - position_counts:  位置計數（int32 以 float 回傳）
      - position_turns:   若 angle_degrees=False -> 圈數；True -> 角度(度)
    """
    pos_i32 = read_motor_position_counts(ser, slave, timeout_s=timeout_s)
    spd_cps = read_actual_speed_counts_per_sec(ser, slave, timeout_s=timeout_s, scale=speed_scale)

    pos_turns = None
    spd_rpm   = None
    if pulses_per_rev and pulses_per_rev > 0:
        if pos_i32 is not None:
            if angle_degrees:
                # 用角度表示（欄位名稱仍為 position_turns）
                pos_turns = counts_to_degrees(pos_i32, pulses_per_rev)
            else:
                pos_turns = counts_to_turns(pos_i32, pulses_per_rev)
        if spd_cps is not None:
            spd_rpm = cps_to_rpm(spd_cps, pulses_per_rev)

    return {
        "actual_speed_cps": spd_cps,
        "actual_speed_rpm": spd_rpm,
        "position_counts": float(pos_i32) if pos_i32 is not None else None,
        "position_turns": pos_turns,  # 可能是「圈數」或「角度(度)」
    }
# state_api.py （新增一個工具函式）
def counts_to_degrees(counts: int, counts_per_rev: int) -> float:
    """計數轉角度（度）。"""
    if counts_per_rev == 0:
        return 0.0
    return (counts / float(counts_per_rev)) * 360.0

