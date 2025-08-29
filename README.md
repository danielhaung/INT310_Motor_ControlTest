<!-- # # 初始化站號3
init_speed_mode(ser, 2, acc_dec=200) #行進輪
init_speed_mode(ser, 3, acc_dec=200) #轉向輪
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
# init_position_mode(ser, 3, acc_dec=0x0D40, speed_rpm=1000)  #轉向輪
# move_to_position(ser, 0, 3, use_turns=True, gear_ratio=1) 
# immediate_stop_if_not_reached(ser, 3)
# move_to_position(ser, 1, 3, use_turns=True, gear_ratio=1) 
pulses_per_rev = 10000  # 驅動器內部 4 倍頻後的解析度
js = init_joystick() -->