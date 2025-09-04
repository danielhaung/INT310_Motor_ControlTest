import pygame, sys

# 死區參數
MOVE_DEADZONE = 0.20   # 行進輪死區
DIR_DEADZONE  = 0.20   # 方向輪死區 ±0.200

# 放大倍率
MOVE_SCALE = 800       # 行進輪倍率
DIR_SCALE  = 1      # 方向輪倍率為要到的位置

def apply_deadzone(v, dz):
    """一般死區處理"""
    return 0.0 if abs(v) < dz else v

def init_joystick():
    """初始化搖桿，只需在 main.py 執行一次"""
    pygame.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() == 0:
        print("❌ 沒偵測到搖桿")
        sys.exit(1)

    js = pygame.joystick.Joystick(0)
    js.init()
    print("✅ 使用搖桿:", js.get_name())
    return js

def read_joystick(js):
    """
    讀取搖桿數值
    回傳 (方向輪, 行進輪, 始能)
    """
    pygame.event.pump()

    axis0_raw = js.get_axis(0)   # 方向輪
    axis1_raw = js.get_axis(1)   # 行進輪 (需反轉)
    b0 = js.get_button(0)        # 始能 (1=ON, 0=OFF)

    # 套死區
    dir_val  = 0.0 if -DIR_DEADZONE <= axis0_raw <= DIR_DEADZONE else axis0_raw
    move_val = -apply_deadzone(axis1_raw, MOVE_DEADZONE)  # 反轉 + 死區

    # 放大倍率
    dir_val  *= DIR_SCALE
    move_val *= MOVE_SCALE

    # 始能關閉時輸出 0
    if b0 == 0:
        dir_val, move_val = 0.0, 0.0

    return dir_val, move_val, bool(b0)