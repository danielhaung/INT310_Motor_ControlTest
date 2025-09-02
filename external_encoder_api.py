#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import serial, struct

RESOLUTION = 4096  # BRT38-R0M4096… → 單圈 4096 counts

# ---------- 低階工具 ----------
def _crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if (crc & 1) else (crc >> 1)
    return crc & 0xFFFF

def _mb_read(ser: serial.Serial, slave_id: int, start_reg: int, count: int) -> bytes:
    """
    讀取保持暫存器：功能碼 0x03
    回傳原始回應（未驗 CRC；您若需要可再加驗證）
    """
    # ADU: [addr][0x03][regH][regL][cntH][cntL][crcL][crcH]
    pdu = bytes([
        slave_id, 0x03,
        (start_reg >> 8) & 0xFF, start_reg & 0xFF,
        (count     >> 8) & 0xFF, count     & 0xFF
    ])
    pkt = pdu + struct.pack("<H", _crc16_modbus(pdu))
    ser.reset_input_buffer()
    ser.write(pkt)
    ser.flush()
    # 最少長度：addr(1)+func(1)+bc(1)+data(2*count)+crc(2)
    expect_min = 5 + 2*count
    buf = ser.read(expect_min)
    return buf

def _u16_be(bh: int, bl: int) -> int:
    return ((bh & 0xFF) << 8) | (bl & 0xFF)

# ---------- 高階函式（給 main 呼叫） ----------
def read_brt38(ser: serial.Serial, slave_id: int = 4) -> dict:
    """
    從已開啟的 Serial 讀 BRT38 多圈編碼器
    讀取：
      0x0000..0x0001 → 32-bit 多圈值
      0x0002         → 圈數
      0x0003         → 單圈值
    回傳：
        {
          "combined_counts": int,   # 32-bit 多圈值（0x0000~0x0001）
          "turns": int,             # 圈數（0x0002）
          "single_counts": int,     # 單圈（0x0003）
          "single_degrees": float   # 單圈角度（0~360）
        }
    """
    # 0x0000~0x0001 → 32-bit 多圈值
    r = _mb_read(ser, slave_id, 0x0000, 2)
    if len(r) < 9:  # addr, func, bc, d0..d3, crcL, crcH
        raise TimeoutError("讀取 0x0000..0x0001 超時/長度不足")
    combined_counts = (r[3] << 24) | (r[4] << 16) | (r[5] << 8) | r[6]

    # 0x0002 → 圈數
    r = _mb_read(ser, slave_id, 0x0002, 1)
    if len(r) < 7:
        raise TimeoutError("讀取 0x0002 超時/長度不足")
    turns = _u16_be(r[3], r[4])

    # 0x0003 → 單圈值
    r = _mb_read(ser, slave_id, 0x0003, 1)
    if len(r) < 7:
        raise TimeoutError("讀取 0x0003 超時/長度不足")
    single_counts = _u16_be(r[3], r[4]) % RESOLUTION
    single_degrees = (single_counts * 360.0) / RESOLUTION

    return {
        "combined_counts": combined_counts,
        "turns": turns,
        "single_counts": single_counts,
        "single_degrees": single_degrees,
    }
