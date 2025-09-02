#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from pos_api import *
from vel_api import *
from state_api import *
from external_encoder_api import read_brt38
import serial
import time

# === 埠設定 ===
PORT_MOTOR = "/dev/ttyUSB0"  # 馬達線（方向輪）
PORT_ENCOD = "/dev/ttyUSB1"  # 外掛編碼器線

BAUD1 = 115200
BAUD2 = 9600
TIMEOUT = 0.2

# === 站號 ===
SLAVE_DIR  = 3   # 方向輪馬達站號
SLAVE_EXT  = 1   # 外掛編碼器站號

PULSES_PER_REV = 10000  # 驅動器內部解析度（依實機修正）

# === 外編換算/控制參數 ===
COUNTS_PER_REV = 4096.0     # 外掛編碼器單圈計數
TARGET_COUNTS  = 32767      # 目標多圈計數
DEADZONE_TURNS = 1          # 誤差死區（圈）
AWAY_EPS       = 0.005      # 「遠離目標」判斷閾值（圈）

# 方向輪速度控制（rpm）
DIR_KP_RPM     = 100.0      # 比例增益：rpm/圈
DIR_RPM_MIN    = 100.0      # 最低啟動轉速
DIR_RPM_MAX    = 100.0      # 上限

# 清零條件 & 行為
ZERO_RPM_THRESH   = 5.0      # 視為停止的實際轉速閾值 (rpm)
ZERO_SETTLE_TIME  = 0.2      # 下達停轉後的靜置時間 (秒)，再檢查一次速度
WRITE_WAIT_AFTER  = 0.05     # 寫入ENC後等待時間 (秒)

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def main():
    # === 初始化串口 ===
    ser_motor = serial.Serial(
        PORT_MOTOR, BAUD1,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_EVEN,   # 馬達 → 8E1
        stopbits=serial.STOPBITS_ONE,
        timeout=TIMEOUT, write_timeout=TIMEOUT
    )
    ser_enc = serial.Serial(
        PORT_ENCOD, BAUD2,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,   # 外編 → 8N1
        stopbits=serial.STOPBITS_ONE,
        timeout=TIMEOUT, write_timeout=TIMEOUT
    )

    # === 馬達初始化（僅方向輪用速度模式）===
    init_speed_mode(ser_motor, SLAVE_DIR, acc_dec=200)

    prev_abs_err = None  # 前一次 |誤差|
    zero_done = False    # 是否已完成清零（只做一次）

    try:
        while True:
            # === 讀馬達狀態（顯示用）===
            state_3 = read_state_once(
                ser_motor, SLAVE_DIR,
                pulses_per_rev=PULSES_PER_REV,
                quiet=True
            )
            actual_rpm = float(state_3.get('actual_speed_rpm', 0.0))
            motor_turns = float(state_3.get('position_turns', 0.0))

            # === 讀外掛編碼器，換算角度 ===
            try:
                ext = read_brt38(ser_enc, slave_id=SLAVE_EXT)
                ext_counts = float(ext['combined_counts'])

                # 以目標 32767 為基準計算誤差（單位：圈）
                err_turns  = (ext_counts - TARGET_COUNTS) / COUNTS_PER_REV
                ext_deg    = (ext_counts / COUNTS_PER_REV) * 360.0
                ext_msg    = f"外編: 多圈={ext_counts:.0f} | 角度={ext_deg:.2f}° | 目標={TARGET_COUNTS}"
            except Exception as e:
                err_turns  = None
                ext_msg    = f"外編: 無回應({e})"

            # === 方向輪閉環控制 ===
            if err_turns is not None:
                abs_err = abs(err_turns)

                # 若尚未清零，且已到目標區域，先停車並嘗試清零
                if (not zero_done) and (abs_err <= DEADZONE_TURNS):
                    # 停馬達
                    set_speed(ser_motor, 0, SLAVE_DIR)
                    time.sleep(ZERO_SETTLE_TIME)

                    # 再讀一次速度確認已接近 0
                    state_check = read_state_once(
                        ser_motor, SLAVE_DIR,
                        pulses_per_rev=PULSES_PER_REV,
                        quiet=True
                    )
                    rpm_check = float(state_check.get('actual_speed_rpm', 0.0))

                    if abs(rpm_check) <= ZERO_RPM_THRESH:
                        try:
                            # 把馬達內部編碼器(DEC 616)寫 0
                            res = write_encoder_position(
                                ser_motor, 0,
                                slave_id=SLAVE_DIR,
                                verify=True,
                                wait_after_write=WRITE_WAIT_AFTER,
                                quiet=False
                            )
                            if res.get('ok', False):
                                print(f"[INFO] 已在目標區內清零內部編碼器 (DEC616=0)，motor_turns={motor_turns:.2f} turn")
                                zero_done = True
                            else:
                                print(f"[WARN] 清零失敗：{res}")
                        except Exception as ex:
                            print(f"[ERROR] 清零例外：{ex}")
                    else:
                        # 尚未完全停止，不清零，之後再試
                        print(f"[INFO] 速度未完全停止 (rpm={rpm_check:.2f})，暫不清零")

                    # 停在目標區域時，控制輸出維持 0
                    dir_rpm = 0.0

                else:
                    # 一般閉環（未進入清零流程或清零已完成）
                    if abs_err <= DEADZONE_TURNS:
                        dir_rpm = 0.0
                    else:
                        # 誤差為正 → 馬達往負方向；反之亦然
                        base_sign = -1.0 if err_turns > 0 else 1.0
                        mag = DIR_KP_RPM * abs_err
                        mag = clamp(mag, DIR_RPM_MIN, DIR_RPM_MAX)
                        dir_rpm = base_sign * mag

                        # 若「更遠離目標」，立即反向（處理相位/接線顛倒）
                        if prev_abs_err is not None and (abs_err > prev_abs_err + AWAY_EPS):
                            dir_rpm = -dir_rpm

                set_speed(ser_motor, int(dir_rpm), SLAVE_DIR)
                prev_abs_err = abs_err
            else:
                # 外編沒回應 → 安全停
                set_speed(ser_motor, 0, SLAVE_DIR)

            # === 顯示 ===
            print(
                f"方向輪: 位置 {motor_turns:.2f} turn | "
                f"速度 {actual_rpm:.2f} rpm | "
                f"{ext_msg}"
            )

            time.sleep(0.05)  # 避免 RS485 擁塞

    except KeyboardInterrupt:
        print("停止監視")
    finally:
        ser_motor.close()
        ser_enc.close()

if __name__ == "__main__":
    main()
