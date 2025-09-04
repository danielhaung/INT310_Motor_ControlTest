#!/usr/bin/env python3
# 單純讀取：D21 狀態 + D24~D27 (LX/LY/RX/RY)；僅需 pyserial
import time, serial

PORT      = "/dev/ttyUSB1"   # 視現場調整，例如 "COM3"
BAUDRATE  = 9600
SLAVE_ID  = 2                # 站號

# --- CRC16(Modbus) ---
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

# --- 讀取 D21~D27 (共7字) 的封包 ---
def frame_read_state_and_axes(slave: int) -> bytes:
    start_addr = 21   # D21
    qty = 7          # D21..D27
    p = bytes([slave, 0x03, (start_addr >> 8) & 0xFF, start_addr & 0xFF,
               (qty >> 8) & 0xFF, qty & 0xFF])
    return add_crc(p)

def u16_to_s16(v: int) -> int:
    return v - 0x10000 if (v & 0x8000) else v

def main():
    ser = serial.Serial(PORT, BAUDRATE, bytesize=8,
                        parity=serial.PARITY_NONE, stopbits=1,
                        timeout=0.2, write_timeout=0.2)
    time.sleep(0.2)

    frame = frame_read_state_and_axes(SLAVE_ID)
    print("Ctrl+C 離開\n")

    try:
        while True:
            ser.reset_input_buffer()
            ser.write(frame); ser.flush()
            time.sleep(0.03)
            resp = ser.read(ser.in_waiting or 256)

            # 期望長度：ID(1)+FUNC(1)+BYTECNT(1)+7*2(=14)+CRC(2)=19 bytes
            if len(resp) >= 19 and resp[0] == SLAVE_ID and resp[1] == 0x03 and resp[2] >= 14:
                # 驗證CRC（可選）
                body = resp[:-2]; crc_lo, crc_hi = resp[-2], resp[-1]
                if crc16_modbus(body) & 0xFF != crc_lo or ((crc16_modbus(body) >> 8) & 0xFF) != crc_hi:
                    print("\rCRC 錯誤", end="", flush=True)
                    time.sleep(0.2); continue

                # 解析 7 個 u16
                regs = []
                for i in range(7):
                    hi = resp[3 + i*2]
                    lo = resp[4 + i*2]
                    regs.append((hi << 8) | lo)

                d21 = regs[0]
                # b4: 急停狀態(1=非急停), b7: 無線連線(1=正常)
                b4 = (d21 >> 4) & 1
                b7 = (d21 >> 7) & 1

                LX = u16_to_s16(regs[3])  # D24
                LY = u16_to_s16(regs[4])  # D25
                RX = u16_to_s16(regs[5])  # D26
                RY = u16_to_s16(regs[6])  # D27

                line = f"D21=0x{d21:04X} (b7={b7}, b4={b4}) | LX {LX:+d} LY {LY:+d} RX {RX:+d} RY {RY:+d}"
                print("\r" + line + " " * 6, end="", flush=True)
            else:
                print("\r讀取失敗...", end="", flush=True)

            time.sleep(0.20)  # ~5Hz
    except KeyboardInterrupt:
        print("\n結束")
    finally:
        ser.close()

if __name__ == "__main__":
    main()
