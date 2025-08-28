from pos_api import *
from vel_api import *
import serial

ser = serial.Serial("/dev/ttyUSB0", 115200,
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_EVEN,
                    stopbits=serial.STOPBITS_ONE,
                    timeout=0.2)

# 初始化站號3
#init_speed_mode(ser, 3, acc_dec=50)
# 設定速度
# set_speed(ser, 100, 3)     # 100 rpm
# time.sleep(1)
# set_speed(ser, 500, 3)     # 100 rpm
# time.sleep(1)
# set_speed(ser, 1000, 3)     # 100 rpm
# time.sleep(1)
# set_speed(ser, 0, 3)     # 100 rpm
# time.sleep(1)

# 前置一次
init_position_mode(ser, 3, acc_dec=0x0D40, speed_rpm=100)
move_to_position(ser, 0, 3, use_turns=True, gear_ratio=1, do_trigger=True, dwell=0.05)
immediate_stop_if_not_reached(ser, 3)

# 無限來回：+1圈 ↔ -1圈，Ctrl+C 結束
try:
    while True:
        immediate_stop_if_not_reached(ser, 3)
        move_to_position(ser,  1, 3, use_turns=True, gear_ratio=1, do_trigger=True, dwell=0.05)
        time.sleep(0.3)

        immediate_stop_if_not_reached(ser, 3)
        move_to_position(ser, -1, 3, use_turns=True, gear_ratio=1, do_trigger=True, dwell=0.05)
        time.sleep(0.3)
except KeyboardInterrupt:
    print("🛑 停止")
    # 若您有定義 disable_motor(ser, 3) 可在此呼叫
    ser.close()

