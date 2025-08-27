"""
每一台INT310直流馬達藉由comm作通訊
"""

from pymodbus.client import ModbusSerialClient
import serial
from typing import Optional, Tuple

# ---------------- CRC16 (Modbus) ----------------
def _crc16_modbus(data: bytes) -> int:
    """計算 Modbus RTU CRC16（結果為 16-bit 整數，低位元組先傳）"""
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x0001:
                crc >>= 1
                crc ^= 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF

# ---------------- Frame Builder ----------------
def _build_write_single_register_frame(slave: int, address: int, value: int) -> bytes:
    """
    建立 Modbus RTU 單一暫存器寫入（function 0x06）封包（包含 CRC）。
    slave: 站號 (0-247)
    address: 寄存器位址 (0-0xFFFF)
    value: 要寫入的 16-bit 值 (0-0xFFFF)
    """
    frame = bytearray()
    frame.append(slave & 0xFF)
    frame.append(0x06)  # function code: Write Single Register
    frame.append((address >> 8) & 0xFF)
    frame.append(address & 0xFF)
    frame.append((value >> 8) & 0xFF)
    frame.append(value & 0xFF)
    crc = _crc16_modbus(bytes(frame))
    frame.append(crc & 0xFF)        # CRC low
    frame.append((crc >> 8) & 0xFF) # CRC high
    return bytes(frame)

# ---------------- Serial Format Helper ----------------
def _parse_serial_format(fmt: str) -> Tuple[int, str, float]:
    """
    將 'N81' / 'E81' / 'E82' 轉為 pyserial 參數 (bytesize, parity, stopbits)
    - 僅允許 N81, E81, E82；其他將拋出 ValueError
    """
    fmt = fmt.upper().strip()
    if fmt == "N81":
        return (serial.EIGHTBITS, serial.PARITY_NONE, serial.STOPBITS_ONE)
    elif fmt == "E81":
        return (serial.EIGHTBITS, serial.PARITY_EVEN, serial.STOPBITS_ONE)
    elif fmt == "E82":
        return (serial.EIGHTBITS, serial.PARITY_EVEN, serial.STOPBITS_TWO)
    else:
        raise ValueError(f"Unsupported serial format: {fmt}. Use 'N81', 'E81', or 'E82'.")

# ---------------- Core Send/Recv ----------------
def send_modbus_write_register(
    port: str,
    baudrate: int,
    slave: int,
    address: int,
    value: int,
    timeout: float = 1.0,
    serial_format: str = "N81",
) -> Optional[bytes]:
    """
    使用 pyserial 直接發送 Modbus RTU single-register (0x06) 並讀取回應。
    - serial_format: 'N81' / 'E81' / 'E82'（預設 N81）
    回傳原始回應 bytes（包含 CRC），若沒回應則回傳 None。
    """
    bytesize, parity, stopbits = _parse_serial_format(serial_format)
    frame = _build_write_single_register_frame(slave, address, value)

    with serial.Serial(
        port=port,
        baudrate=baudrate,
        bytesize=bytesize,
        parity=parity,
        stopbits=stopbits,
        timeout=timeout,
        write_timeout=timeout,
    ) as ser:
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        ser.write(frame)

        # Modbus 0x06 回應通常 8 bytes；但保險起見讀多一點（某些設備會多回資料）
        # 先嘗試讀 8 bytes，不足再補讀
        expected_len = 8
        resp = ser.read(expected_len)
        if len(resp) < 5:
            # 再等等看是否還有資料到
            extra = ser.read(256)
            resp += extra

        if len(resp) < 5:
            return None

        # 驗證 CRC（最後兩個字節為 CRC low, CRC high）
        if len(resp) >= 2:
            recv_crc = resp[-2] | (resp[-1] << 8)
            if _crc16_modbus(resp[:-2]) != recv_crc:
                raise IOError("收到回應但 CRC 驗證失敗")
        return resp

# ---------------- Convenience Wrapper ----------------
def send_modbus_command():
    """
    範例：/dev/ttyUSB0、115200、E82，寫寄存器 0x0003 = 1
    可調整 serial_format 為 'N81' / 'E81' / 'E82'
    """
    try:
        resp = send_modbus_write_register(
            port='/dev/ttyUSB0',
            baudrate=115200,
            slave=3,
            address=0x0003,
            value=1,
            timeout=0.3,
            serial_format="E82",   # ← 在這裡選擇 N81 / E81 / E82
        )
        if resp:
            print("Modbus 回應 (hex):", resp.hex())
        else:
            print("沒有收到回應")
    except Exception as e:
        print("發送 Modbus 指令失敗:", e)
