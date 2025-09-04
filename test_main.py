from pos_api import *
from vel_api import *
from state_api import *
from external_encoder_api import *
import serial
import time
from zjrcjn_joy_reader import init_joystick, read_joystick

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
init_position_mode(ser, 3, acc_dec=0xFE20, speed_rpm=3000)  #轉向輪
# move_to_position(ser, 0, 3, use_turns=True, gear_ratio=1) 
# immediate_stop_if_not_reached(ser, 3)
# move_to_position(ser, 1, 3, use_turns=True, gear_ratio=1) 
pulses_per_rev = 10000  # 驅動器內部 4 倍頻後的解析度
js = init_joystick()
# 新增一個變數紀錄上一次的方向
last_direction = None  

try:
    while True:
        direction, movement, enable = read_joystick(js)
        print(f"方向輪={direction:+.3f} | 行進輪={movement:+.3f} | 始能={'ON' if enable else 'OFF'}")
        if enable == True: 
            state_2 = read_state_once(ser, 2,pulses_per_rev=pulses_per_rev)
            state_3 = read_state_once(ser, 3,pulses_per_rev=pulses_per_rev)       
            #direction = - direction
            movement = -movement
            set_speed(ser, movement, 2)
            # 如果 direction 與上一次相反，呼叫立即停止函式
            if last_direction is not None and (direction * last_direction < 0):
                immediate_stop_if_not_reached(ser, 3, tag="未達目標立即停止")
            if int(state_3['actual_speed_rpm']) == 0:
                move_to_position(ser, direction, 3, use_turns=True, gear_ratio=1)
            print(
                f"行進輪: {state_2['actual_speed_rpm']:.1f} rpm | "
                f"方向輪: 位置 {state_3['position_turns']:.2f} 度 | "
                f"速度 {state_3['actual_speed_rpm']:.2f} rpm"
            )   
            # 更新方向紀錄
            last_direction = direction  
        else:
            set_speed(ser, 0, 2)
            move_to_position(ser, 0, 3, use_turns=True, gear_ratio=1) #跟隨車gear_ratio=10
            # 寫入位置為 0 並驗證

        #time.sleep(0.01)

except KeyboardInterrupt:
    print("停止監視")

finally:
    ser.close()

