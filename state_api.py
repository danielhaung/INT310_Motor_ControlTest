#state_api.py
# 統一：讀/寫編碼器位置 + 讀取實際速度 + 快速狀態一次讀
import time
import serial
from typing import Optional, List, Dict, Tuple, Any

# =========================
# 常數（手冊映射）
# =========================
# 各占 4 bytes (32-bit)，以兩個 16-bit 暫存器表示；採「低字在前」的 2-word 格式
ADDR_POS_LE_DEC       = 616  # 電機位置（起始暫存器，低位在前）== 0x0268
ADDR_ACT_SPEED_LE_DEC = 630  # 實際速度（起始暫存器，低位在前）== 0x0276

DEFAULT_SPEED_SCALE = 0.1    # 內部值 * 0.1 -> counts/sec（依實機調整）
ADDR_ENC_POS        = ADDR_POS_LE_DEC  # for write/read 一致用法

# =========================
# 基礎：CRC / 建包 / 收發
# =========================
def crc16_modbus(data: bytes) -> int:
    """Modbus RTU CRC16（低位在前）。"""
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if (crc & 1) else (crc >> 1)
    return crc & 0xFFFF

def _add_crc(p: bytes) -> bytes:
    c = crc16_modbus(p)
    return p + bytes([c & 0xFF, (c >> 8) & 0xFF])

def _build_read_frame(slave: int, start_addr: int, qty_words: int) -> bytes:
    body = bytes([
        slave & 0xFF, 0x03,
        (start_addr >> 8) & 0xFF, start_addr & 0xFF,
        (qty_words  >> 8) & 0xFF, qty_words  & 0xFF
    ])
    return _add_crc(body)

def _build_write_multi(slave: int, start_addr: int, words: List[int]) -> bytes:
    qty = len(words)
    body = [
        slave & 0xFF, 0x10,
        (start_addr >> 8) & 0xFF, start_addr & 0xFF,
        (qty >> 8) & 0xFF, qty & 0xFF,
        qty * 2
    ]
    for w in words:
        body += [(w >> 8) & 0xFF, w & 0xFF]
    return _add_crc(bytes(body))

def _expect_len_for_words(qty_words: int) -> int:
    # slave(1)+func(1)+byte_count(1)+data(2*qty)+CRC(2)
    return 1 + 1 + 1 + (2 * qty_words) + 2

def _send_and_recv(
    ser: serial.Serial,
    frame: bytes,
    expected_bytes: Optional[int] = None,
    read_timeout_s: float = 0.15,
    post_delay_s: float = 0.03,
    quiet: bool = False,
    tag: str = ""
) -> bytes:
    ser.reset_input_buffer()
    ser.write(frame); ser.flush()
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
    resp = bytes(buf)
    if not quiet:
        print(f"→ {tag or 'XFER':14s} | TX: {frame.hex(' ').upper()}  RX: {resp.hex(' ').upper()}")
    return resp

def _parse_read_resp(resp: bytes, qty_words: int) -> Optional[List[int]]:
    if len(resp) < 5 or resp[1] != 0x03:
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
    # 轉 16-bit words（高位元組在前），再由呼叫端決定大小端語意
    data = resp[3:3 + byte_count]
    words: List[int] = []
    for i in range(0, byte_count, 2):
        words.append((data[i] << 8) | data[i + 1])
    return words if len(words) == qty_words else None

# =========================
# 32-bit <-> 兩個 16-bit（低字在前 / LE words）
# =========================
def _le_words_to_i32(lo: int, hi: int) -> int:
    u32 = ((hi & 0xFFFF) << 16) | (lo & 0xFFFF)
    return u32 - 0x1_0000_0000 if (u32 & 0x8000_0000) else u32

def _i32_to_le_words(v: int) -> Tuple[int, int]:
    u = v & 0xFFFFFFFF
    return (u & 0xFFFF), ((u >> 16) & 0xFFFF)

# =========================
# 讀取原語：words / int32(LE)
# =========================
def _read_words(
    ser: serial.Serial,
    slave: int,
    start_addr: int,
    qty_words: int,
    timeout_s: float = 0.15,
    quiet: bool = False
) -> Optional[List[int]]:
    frame = _build_read_frame(slave, start_addr, qty_words)
    rx = _send_and_recv(
        ser, frame,
        expected_bytes=_expect_len_for_words(qty_words),
        read_timeout_s=timeout_s,
        quiet=quiet,
        tag=f"READ 0x{start_addr:04X}"
    )
    return _parse_read_resp(rx, qty_words)

def _read_int32_le(
    ser: serial.Serial,
    slave: int,
    start_addr_le: int,
    timeout_s: float = 0.15,
    quiet: bool = False
) -> Optional[int]:
    ws = _read_words(ser, slave, start_addr_le, 2, timeout_s=timeout_s, quiet=quiet)
    if not ws:
        return None
    # 注意：寄存器語意為「低字在前」
    return _le_words_to_i32(ws[0], ws[1])

# =========================
# 高階 API（位址已內建）
# =========================
def read_motor_position_counts(
    ser: serial.Serial,
    slave: int,
    timeout_s: float = 0.15,
    quiet: bool = False
) -> Optional[int]:
    """讀「電機位置」32-bit 計數（MODBUS(DEC)=616）。"""
    return _read_int32_le(ser, slave, ADDR_POS_LE_DEC, timeout_s=timeout_s, quiet=quiet)

def read_actual_speed_internal(
    ser: serial.Serial,
    slave: int,
    timeout_s: float = 0.15,
    quiet: bool = False
) -> Optional[int]:
    """讀「實際速度」內部 32-bit 值（MODBUS(DEC)=630），未換算。"""
    return _read_int32_le(ser, slave, ADDR_ACT_SPEED_LE_DEC, timeout_s=timeout_s, quiet=quiet)

def read_actual_speed_counts_per_sec(
    ser: serial.Serial,
    slave: int,
    timeout_s: float = 0.15,
    scale: float = DEFAULT_SPEED_SCALE,
    quiet: bool = False
) -> Optional[float]:
    """讀「實際速度」並換算為 counts/sec。"""
    iv = read_actual_speed_internal(ser, slave, timeout_s=timeout_s, quiet=quiet)
    if iv is None:
        return None
    return float(iv) * float(scale)

def counts_to_turns(counts: int, pulses_per_rev: int) -> float:
    return 0.0 if pulses_per_rev == 0 else counts / float(pulses_per_rev)

def cps_to_rpm(cps: float, pulses_per_rev: int) -> float:
    if pulses_per_rev == 0:
        return 0.0
    rps = cps / float(pulses_per_rev)
    return rps * 60.0

def counts_to_degrees(counts: int, counts_per_rev: int) -> float:
    if counts_per_rev == 0:
        return 0.0
    return (counts / float(counts_per_rev)) * 360.0

def read_state_once(
    ser: serial.Serial,
    slave: int,
    *,
    timeout_s: float = 0.15,
    speed_scale: float = DEFAULT_SPEED_SCALE,
    pulses_per_rev: int | None = None,
    angle_degrees: bool = True,
    quiet: bool = False
) -> Dict[str, Optional[float]]:
    """
    一次回傳：
      - actual_speed_cps: 實際速度（counts/sec）
      - actual_speed_rpm: 若提供 pulses_per_rev，會轉 rpm；否則為 None
      - position_counts:  位置計數（int32 以 float 回傳）
      - position_turns:   若 angle_degrees=False -> 圈數；True -> 角度(度)
    """
    pos_i32 = read_motor_position_counts(ser, slave, timeout_s=timeout_s, quiet=quiet)
    spd_cps = read_actual_speed_counts_per_sec(ser, slave, timeout_s=timeout_s,
                                               scale=speed_scale, quiet=quiet)

    pos_turns = None
    spd_rpm   = None
    if pulses_per_rev and pulses_per_rev > 0:
        if pos_i32 is not None:
            pos_turns = counts_to_degrees(pos_i32, pulses_per_rev) if angle_degrees \
                        else counts_to_turns(pos_i32, pulses_per_rev)
        if spd_cps is not None:
            spd_rpm = cps_to_rpm(spd_cps, pulses_per_rev)

    return {
        "actual_speed_cps": spd_cps,
        "actual_speed_rpm": spd_rpm,
        "position_counts": float(pos_i32) if pos_i32 is not None else None,
        "position_turns": pos_turns,
    }

# =========================
# 讀/寫 編碼器位置（保留 enc_api 介面）
# =========================
def read_encoder_position(ser: serial.Serial,
                          slave_id: int = 0x03,
                          *,
                          quiet: bool = False) -> int:
    """等同 read_motor_position_counts；回傳 int32（可負）。"""
    val = read_motor_position_counts(ser, slave_id, quiet=quiet)
    if val is None:
        raise RuntimeError("讀取編碼器位置失敗")
    return val

def write_encoder_position(ser: serial.Serial,
                           value: int,
                           slave_id: int = 0x03,
                           *,
                           verify: bool = True,
                           wait_after_write: float = 0.05,
                           quiet: bool = False) -> Dict[str, Any]:
    """寫 DEC 616（32-bit）。回傳 {'after': int|None, 'ok': bool}。"""
    lo, hi = _i32_to_le_words(value)
    frame = _build_write_multi(slave_id, ADDR_ENC_POS, [lo, hi])
    _send_and_recv(ser, frame, read_timeout_s=0.15, post_delay_s=0.03,
                   quiet=quiet, tag="WRITE ENC616")
    time.sleep(wait_after_write)
    if verify:
        after = read_encoder_position(ser, slave_id, quiet=quiet)
        return {"after": after, "ok": (after == value)}
    return {"after": None, "ok": True}

def rw_encoder_position(ser: serial.Serial,
                        slave_id: int = 0x03,
                        set_value: Optional[int] = None,
                        *,
                        verify: bool = True,
                        quiet: bool = False) -> Dict[str, Any]:
    """
    單一入口：
      - 只讀：rw_encoder_position(ser, slave_id)
      - 寫入：rw_encoder_position(ser, slave_id, set_value=0, verify=True)
    """
    before = read_encoder_position(ser, slave_id, quiet=quiet)
    if set_value is None:
        return {"before": before}
    result = write_encoder_position(ser, set_value, slave_id,
                                    verify=verify, quiet=quiet)
    return {"before": before, "after": result["after"], "ok": result["ok"]}
