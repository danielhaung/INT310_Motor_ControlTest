from pos_api import *
from vel_api import *
from state_api import *
import serial
import time
from joystick_reader import init_joystick, read_joystick

ser = serial.Serial("/dev/ttyUSB0", 115200,
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_EVEN,
                    stopbits=serial.STOPBITS_ONE,
                    timeout=0.2)

# # 初始化站號3
init_speed_mode(ser, 2, acc_dec=200) #行進輪
#init_speed_mode(ser, 3, acc_dec=200) #轉向輪
# #設定速度
# set_speed(ser, 100, 3)     # 100 rpm
# time.sleep(1)
# set_speed(ser, 500, 3)     # 100 rpm
# time.sleep(1)
# set_speed(ser, -1000, 3)     # 100 rpm
# time.sleep(1)
# set_speed(ser, 0, 3)     # 100 rpm
# time.sleep(1)
# init_position_mode(ser, 2, acc_dec=0x0D00, speed_rpm=1000) #行進輪
init_position_mode(ser, 3, acc_dec=0x0D40, speed_rpm=500)  #轉向輪
# move_to_position(ser, 0, 3, use_turns=True, gear_ratio=1) 
# immediate_stop_if_not_reached(ser, 3)
# move_to_position(ser, 1, 3, use_turns=True, gear_ratio=1) 
pulses_per_rev = 10000  # 驅動器內部 4 倍頻後的解析度
js = init_joystick()

try:
    while True:
        direction, movement, enable = read_joystick(js)
        print(f"方向輪={direction:+.3f} | 行進輪={movement:+.3f} | 始能={'ON' if enable else 'OFF'}")
        state_2 = read_state_once(
            ser, 2,
            pulses_per_rev=pulses_per_rev
        )
        state_3 = read_state_once(
            ser, 3,
            pulses_per_rev=pulses_per_rev
        ) 
        if enable == True:       
            cur_wheel = state_3['position_turns']
            set_speed(ser, movement, 2)

            err = direction - cur_wheel
            kp = 1.0               # 降低比例，原本2.0太大
            deadzone = 360.0         # 誤差在 ±5 以內就不動
            wheel_vel = 0 if abs(err) < deadzone else int(kp * err)

            # 誤差越大速度不一定要衝滿 → 線性縮放到 ±1500
            max_err = 3600         # 假設滿刻度差 900 對應到 1500rpm
            wheel_vel = max(-500, min(500, wheel_vel * 500 // max_err))

            print(f"方向輪:{direction:.2f} | 當前:{cur_wheel:.2f} | 誤差:{err:.2f} | 計算速度:{wheel_vel}")
            set_speed(ser, wheel_vel, 3)

            print(
                f"行進輪: {state_2['actual_speed_rpm']:.1f} rpm | "
                f"方向輪: 位置 {state_3['position_turns']:.2f} 度 | "
                f"速度 {state_3['actual_speed_rpm']:.2f} rpm"
            )   
        else:
            set_speed(ser, 0, 2)
            set_speed(ser, 0, 3)

        time.sleep(0.01)

except KeyboardInterrupt:
    print("停止監視")

finally:
    ser.close()


ser.close()