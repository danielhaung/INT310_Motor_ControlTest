import pygame, sys, time

# 可選：死區，避免微小抖動
DEADZONE = 0.2   # 行進輪的小死區
DIR_DEADZONE = 0.20  # 方向輪專用死區 ±0.200

def apply_deadzone(v, dz=DEADZONE):
    return 0.0 if abs(v) < dz else v

def apply_dir_deadzone(v, dz=DIR_DEADZONE):
    return 0.0 if -dz <= v <= dz else v

pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    print("❌ 沒偵測到搖桿")
    sys.exit(1)

js = pygame.joystick.Joystick(0)
js.init()
print("使用搖桿:", js.get_name())
print("規則：始能=OFF → 行進輪=0 | 方向輪=0；始能=ON → 顯示實際值 (方向輪有 ±0.200 死區)")
print("Ctrl+C 停止\n")

prev_b0 = None
try:
    while True:
        pygame.event.pump()

        # 讀取實際軸
        axis0_raw = js.get_axis(0)   # 方向輪
        axis1_raw = js.get_axis(1)   # 行進輪 (需反轉)
        b0 = js.get_button(0)        # 始能

        # 套死區
        axis0 = apply_dir_deadzone(axis0_raw)   # 方向輪用 ±0.200 死區
        axis1 = -apply_deadzone(axis1_raw)      # 行進輪用小死區 + 反轉

        # 始能為 OFF 時，強制顯示 0
        if b0 == 0:
            disp_dir, disp_move = 0.0, 0.0
        else:
            disp_dir, disp_move = axis0, axis1

        # 單行覆寫顯示
        line = f"方向輪={disp_dir:+.3f} | 行進輪={disp_move:+.3f} | 始能={'ON' if b0 else 'OFF'}"
        print("\r" + line + " " * 8, end="", flush=True)

        # 邊緣事件列印
        if prev_b0 is None or b0 != prev_b0:
            print()
            print("始能 ON" if b0 else "始能 OFF")
            prev_b0 = b0

        time.sleep(0.05)  # ~20Hz
except KeyboardInterrupt:
    print("\n結束監控")
