import time
import serial
from typing import Optional

# ========= Modbus RTU Helpers =========
def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if (crc & 1) else (crc >> 1)
    return crc & 0xFFFF

def _attach_crc(p: bytes) -> bytes:
    c = crc16_modbus(p)
    return p + bytes([c & 0xFF, (c >> 8) & 0xFF])

def _check_crc(frame: bytes) -> bool:
    if len(frame) < 3:
        return False
    data, lo, hi = frame[:-2], frame[-2], frame[-1]
    return (((hi << 8) | lo) & 0xFFFF) == crc16_modbus(data)

def _send(ser: serial.Serial, payload_wo_crc: bytes,
          expect_len: Optional[int], desc: str,
          pause: float = 0.03, deadline_s: float = 0.35) -> bytes:
    frame = _attach_crc(payload_wo_crc)
    ser.reset_input_buffer()
    ser.write(frame); ser.flush()
    time.sleep(pause)

    rx = b""
    t0 = time.perf_counter()
    while (time.perf_counter() - t0) < deadline_s:
        n = ser.in_waiting
        if n:
            rx += ser.read(n)
            if expect_len is None or len(rx) >= expect_len:
                break
        else:
            time.sleep(0.005)
    ok = _check_crc(rx)
    print(f"→ {desc:18s} | TX: {frame.hex(' ').upper()}  RX: {rx.hex(' ').upper()}{' (CRC OK)' if ok else ''}")
    return rx

def write_reg_06(ser: serial.Serial, slave: int, addr_dec: int, value_u16: int, desc: str):
    p = bytes([slave & 0xFF, 0x06, (addr_dec>>8)&0xFF, addr_dec&0xFF, (value_u16>>8)&0xFF, value_u16&0xFF])
    rx = _send(ser, p, 8, desc)
    return rx if _check_crc(rx) and len(rx) == 8 else None

def read_regs_03(ser: serial.Serial, slave: int, start_dec: int, count: int, desc: str):
    p = bytes([slave & 0xFF, 0x03, (start_dec>>8)&0xFF, start_dec&0xFF, (count>>8)&0xFF, count&0xFF])
    expect = 1 + 1 + 1 + (2*count) + 2
    rx = _send(ser, p, expect, desc)
    if len(rx) >= 5 and rx[1] == 0x83:
        print("❌ Exception code:", rx[2])
        return None
    if not (_check_crc(rx) and len(rx) >= expect and rx[1] == 0x03 and rx[2] == 2*count):
        return None
    data = rx[3:3+2*count]
    regs = [ (data[i]<<8) | data[i+1] for i in range(0, len(data), 2) ]
    return regs

def write_coil_05(ser: serial.Serial, slave: int, coil_addr: int, value_on: bool, desc: str):
    value = 0xFF00 if value_on else 0x0000
    p = bytes([slave & 0xFF, 0x05, (coil_addr>>8)&0xFF, coil_addr&0xFF, (value>>8)&0xFF, value&0xFF])
    rx = _send(ser, p, 8, desc)
    return rx if _check_crc(rx) and len(rx) == 8 else None

# ========= Decode Helpers =========
D666_BITS = {
    0:"短路",1:"驅動器過溫",2:"過電壓",3:"低電壓",4:"電機溫度有效",5:"反饋錯誤",6:"相位錯誤",
    7:"電流輸出限制",8:"電壓輸出限制",9:"正限位",10:"負限位",11:"使能輸入無效",
    12:"軟體禁用驅動器",13:"正在停止電機",14:"電機制動有效",15:"PWM輸出無效",
    16:"正向軟體限位",17:"負向軟體限位",18:"跟隨錯誤",19:"跟隨警告",20:"驅動器復位",
    21:"編碼器計數",22:"驅動錯誤(見D672)",23:"達速度上限",24:"達加速上限",
    25:"位置超出跟隨窗口",26:"原點開關有效"
}

def words_to_u32_be(hi, lo): return (hi << 16) | lo

def decode_d666(u32: int) -> str:
    if u32 == 0:
        return "D666=0x00000000（無告警/無故障）"
    on = [ f"bit{b}:{name}" for b, name in D666_BITS.items() if (u32 >> b) & 1 ]
    return "；".join(on) if on else f"未知位元：0x{u32:08X}"

# ========= Test Logic =========
PORT = "/dev/ttyUSB0"
BAUD = 115200
SLAVE = 2               # 依實機站號調整
GUARD_MS = 250          # 寫入 331
GUARD_FACTOR = 4        # 寫入 333
HEARTBEAT_INTERVAL = 0.10   # 心跳輪詢間隔 (s)
LOSS_THRESHOLD = 1.0        # 連續無回應視為斷線秒數
POLL_ADDR_SPEED_ACT = 630   # 實際速度
REG_D666 = 666
REG_D672 = 672

def open_port() -> serial.Serial:
    return serial.Serial(
        port=PORT, baudrate=BAUD, bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_EVEN, stopbits=serial.STOPBITS_ONE,
        timeout=0.25, write_timeout=0.25
    )

def setup_guard(ser: serial.Serial):
    # 331/333 設定並讀回確認
    write_reg_06(ser, SLAVE, 331, GUARD_MS,   "寫 331 節點保護時間")
    time.sleep(0.05)
    write_reg_06(ser, SLAVE, 333, GUARD_FACTOR,"寫 333 節點保護因子")
    time.sleep(0.05)
    regs = read_regs_03(ser, SLAVE, 331, 3, "讀回 331..333 確認")
    if regs:
        print(f"✅ 331={regs[0]}  332={regs[1]}  333={regs[2]} → 總超時={regs[0]*regs[2]} ms")

def poll_heartbeat(ser: serial.Serial):
    """
    以讀取 D630 當心跳；連續 LOSS_THRESHOLD 秒都沒有成功回應 → 視為斷線。
    斷線後自動等待重連；重連成功後讀 D666/D672，嘗試清錯。
    """
    last_ok = time.monotonic()
    while True:
        regs = read_regs_03(ser, SLAVE, POLL_ADDR_SPEED_ACT, 1, "讀 D630(實際速度)")
        if regs is not None:
            last_ok = time.monotonic()
        else:
            # 檢查是否超過斷線門檻
            if (time.monotonic() - last_ok) > LOSS_THRESHOLD:
                print("⚠️ 偵測到連線中斷，等待重連中…（請插回 USB/RS485）")
                ser.close()
                # 重試開啟
                while True:
                    try:
                        time.sleep(0.5)
                        ser = open_port()
                        print("🔌 已重連 Serial，開始讀取 D666/D672")
                        break
                    except Exception:
                        print("…尚未就緒，持續等待")
                        time.sleep(0.5)
                # 讀 D666 / D672
                d666 = read_regs_03(ser, SLAVE, REG_D666, 2, "讀 D666 狀態")
                d672 = read_regs_03(ser, SLAVE, REG_D672, 2, "讀 D672 鎖存故障")
                if d666:
                    u32 = words_to_u32_be(d666[0], d666[1])
                    print(f"🔎 D666=0x{u32:08X} → {decode_d666(u32)}")
                if d672:
                    u32f = words_to_u32_be(d672[0], d672[1])
                    print(f"🔎 D672=0x{u32f:08X}（鎖存故障）")

                # 若有鎖存或狀態，先嘗試 故障清除（coil 1），再不行就 驅動器復位（coil 3）
                if d672 and words_to_u32_be(d672[0], d672[1]) != 0:
                    print("🧹 嘗試：線圈1（故障清除）")
                    write_coil_05(ser, SLAVE, 1, True,  "coil1=ON")
                    write_coil_05(ser, SLAVE, 1, False, "coil1=OFF")
                    time.sleep(0.2)
                    d672c = read_regs_03(ser, SLAVE, REG_D672, 2, "讀 D672（清除後）")
                    if d672c and words_to_u32_be(d672c[0], d672c[1]) != 0:
                        print("♻️ 再嘗試：線圈3（驅動器復位）")
                        write_coil_05(ser, SLAVE, 3, True,  "coil3=ON")
                        write_coil_05(ser, SLAVE, 3, False, "coil3=OFF")
                last_ok = time.monotonic()
        time.sleep(HEARTBEAT_INTERVAL)

if __name__ == "__main__":
    try:
        ser = open_port()
        setup_guard(ser)
        print("🫀 開始心跳輪詢（每 100ms 讀 D630）。請於任一時刻拔掉 USB 測試超時保護；再插回即可自動重連並讀 D666/D672。")
        poll_heartbeat(ser)
    except KeyboardInterrupt:
        print("👋 結束測試")
    finally:
        try:
            ser.close()
        except Exception:
            pass
