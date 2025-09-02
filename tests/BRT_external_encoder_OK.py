#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import serial, struct, time

# CRC16-Modbus
def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF

def modbus_read(ser, slave_id: int, start_reg: int, count: int):
    # 組封包 [addr][0x03][regH][regL][cntH][cntL][crcL][crcH]
    pkt = bytes([slave_id, 0x03,
                 (start_reg >> 8) & 0xFF, start_reg & 0xFF,
                 (count >> 8) & 0xFF, count & 0xFF])
    crc = crc16(pkt)
    pkt += struct.pack("<H", crc)
    ser.write(pkt)
    time.sleep(0.05)
    resp = ser.read(5 + count*2)  # addr, func, bytecount, data..., crc
    return resp

if __name__ == "__main__":
    ser = serial.Serial("/dev/ttyUSB0", 9600,
                        parity=serial.PARITY_NONE,  # 手冊預設 8N1
                        stopbits=1,
                        bytesize=8,
                        timeout=0.2)

    slave_id = 1

    # 讀取 0x0000~0x0001 → 32-bit 編碼器值
    resp = modbus_read(ser, slave_id, 0x0000, 2)
    if len(resp) >= 9:
        val = (resp[3]<<24) | (resp[4]<<16) | (resp[5]<<8) | resp[6]
        print(f"多圈值: {val}")

    # 讀取 0x0002 → 圈數
    resp = modbus_read(ser, slave_id, 0x0002, 1)
    if len(resp) >= 7:
        turns = (resp[3]<<8) | resp[4]
        print(f"圈數: {turns}")

    # 讀取 0x0003 → 單圈值
    resp = modbus_read(ser, slave_id, 0x0003, 1)
    if len(resp) >= 7:
        single = (resp[3]<<8) | resp[4]
        print(f"單圈值: {single}")

    ser.close()
