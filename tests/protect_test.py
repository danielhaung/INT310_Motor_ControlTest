import time
import serial

# ---------- CRC16 ----------
def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if (crc & 1) else crc >> 1
    return crc & 0xFFFF

def _attach_crc(payload: bytes) -> bytes:
    crc = crc16_modbus(payload)
    return payload + bytes([crc & 0xFF, (crc >> 8) & 0xFF])

def _check_crc(frame: bytes) -> bool:
    if len(frame) < 3:
        return False
    data, lo, hi = frame[:-2], frame[-2], frame[-1]
    return (((hi << 8) | lo) & 0xFFFF) == crc16_modbus(data)

def _is_exception(resp: bytes) -> tuple[bool, int | None]:
    # Modbus 例外：功能碼 | 0x80，接著 1 byte 例外碼
    if len(resp) >= 5 and (resp[1] & 0x80):
        return True, resp[2]
    return False, None

def _send(ser: serial.Serial, payload_wo_crc: bytes, desc: str,
          expect_len: int | None = None, pause: float = 0.05, retries: int = 2) -> bytes:
    for attempt in range(1, retries + 1):
        frame = _attach_crc(payload_wo_crc)
        ser.reset_input_buffer()
        ser.write(frame)
        ser.flush()
        time.sleep(pause)

        rx = b""
        deadline = time.time() + 0.4
        while time.time() < deadline:
            n = ser.in_waiting
            if n:
                rx += ser.read(n)
                if expect_len and len(rx) >= expect_len:
                    break
            else:
                time.sleep(0.01)

        ok_crc = _check_crc(rx)
        is_exc, exc_code = _is_exception(rx)

        print(f"→ {desc:20s} | TX: {frame.hex(' ').upper()}  RX: {rx.hex(' ').upper()}"
              f"{' (CRC OK)' if ok_crc else ''}{f' (EXC {exc_code})' if is_exc else ''}")

        if rx and ok_crc and not is_exc:
            return rx
        # 若失敗則稍等後重試
        time.sleep(0.08)
    return rx

# ---------- 單寄存器寫入 0x06 ----------
def write_reg_06(ser: serial.Serial, slave: int, addr_dec: int, value_u16: int, desc: str):
    payload = bytes([
        slave & 0xFF, 0x06,
        (addr_dec >> 8) & 0xFF, addr_dec & 0xFF,
        (value_u16 >> 8) & 0xFF, value_u16 & 0xFF
    ])
    # 0x06 回包與發包相同長度：8 bytes
    return _send(ser, payload, desc, expect_len=8)

# ---------- 讀保持暫存器 0x03 ----------
def read_regs_03(ser: serial.Serial, slave: int, start_dec: int, count: int, desc: str):
    payload = bytes([
        slave & 0xFF, 0x03,
        (start_dec >> 8) & 0xFF, start_dec & 0xFF,
        (count >> 8) & 0xFF, count & 0xFF
    ])
    # 回包: addr(1)+func(1)+bytecount(1)+data(2*count)+CRC(2)
    expect = 1 + 1 + 1 + (2*count) + 2
    rx = _send(ser, payload, desc, expect_len=expect)
    if len(rx) >= expect and rx[1] == 0x03 and rx[2] == 2*count and _check_crc(rx):
        data = rx[3:3+2*count]
        regs = []
        for i in range(0, len(data), 2):
            regs.append((data[i] << 8) | data[i+1])
        return regs
    return None

# ---------- 主程式 ----------
if __name__ == "__main__":
    # 依你的實機調整：手冊預設 115200,E,8,1
    ser = serial.Serial(
        port="/dev/ttyUSB0",
        baudrate=115200,
        bytesize=8,
        parity="E",
        stopbits=1,
        timeout=0.3
    )

    SLAVE = 2
    GUARD_TIME_MS   = 250   # MODBUS(DEC)=331
    GUARD_FACTOR    = 4     # MODBUS(DEC)=333  → 總超時= 250*4 = 1000 ms

    # 寫入節點保護參數
    write_reg_06(ser, SLAVE, 331, GUARD_TIME_MS,   "寫 331 節點保護時間")
    time.sleep(0.05)
    write_reg_06(ser, SLAVE, 333, GUARD_FACTOR,    "寫 333 節點保護因子")
    time.sleep(0.05)

    # 讀回確認（一次讀 331、332、333）
    regs = read_regs_03(ser, SLAVE, 331, 3, "讀回 331..333 確認")
    if regs and len(regs) == 3:
        t_ms, reserved_332, factor = regs
        print(f"✅ 331={t_ms}  332={reserved_332}  333={factor}  → 總超時={t_ms*factor} ms")
    else:
        print("⚠️ 讀回失敗或長度不符")

    # 可選：讀 D666 驅動器狀態寄存器（MODBUS DEC=666，長度4 bytes → 讀2個暫存器）
    regs_d666 = read_regs_03(ser, SLAVE, 666, 2, "讀 D666 驅動器狀態")
    if regs_d666 and len(regs_d666) == 2:
        hi, lo = regs_d666[0], regs_d666[1]
        u32 = (hi << 16) | lo
        print(f"ℹ️ D666 狀態 (U32) = 0x{u32:08X}")

    ser.close()
