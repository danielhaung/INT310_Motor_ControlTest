#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
72V gear 比 275：要轉 90 度 -> 90/360*275 = 69.375 轉（如需用到可換成 GEAR_RATIO）

版本說明：
- 每個 serial port 各一條 Thread（Move / Steer），主執行緒只讀搖桿並丟入佇列。
- 加入超時安全：若超過 JOYSTICK_TIMEOUT_S 沒有新指令，行進輪=0、方向輪回 0。
- 方向反轉保護：新指令與上一筆方向相反時，先立即停（immediate_stop_if_not_reached）。
"""

import time
import threading
import queue
import logging
from typing import Optional

import serial

# 依您環境的 API 模組
from pos_api import init_position_mode, move_to_position, immediate_stop_if_not_reached
from vel_api import init_speed_mode, set_speed
from state_api import read_state_once
from external_encoder_api import *  # 若需外接編碼器可在此使用
from zjrcjn_joy_reader import init_joystick, read_joystick


# ==============================
# 基本參數
# ==============================

SER1_PORT = "/dev/ttyUSB0"   # 行進輪 (站號 1)
SER2_PORT = "/dev/ttyUSB1"   # 方向輪 (站號 2)
BAUD = 115200

STATION_MOVE = 1
STATION_STEER = 2

PULSES_PER_REV = 10000       # 驅動器內部 4 倍頻解析度
GEAR_RATIO = 5              # 您原本的 gear_ratio

JOYSTICK_TIMEOUT_S = 0.6     # 若超過此時間沒有新指令，視為掉線 -> 安全停車
LOOP_SLEEP_S = 0.01          # Worker 迴圈 sleep

# 佇列（行進：rpm、方向：turns）
move_q: "queue.Queue[float]" = queue.Queue(maxsize=5)
steer_q: "queue.Queue[float]" = queue.Queue(maxsize=5)

# 停止旗標
stop_event = threading.Event()


# ==============================
# Logger
# ==============================

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(levelname)s [%(threadName)s] %(message)s",
)


# ==============================
# Serial 工具
# ==============================

def open_serial(port: str, baud: int) -> serial.Serial:
    """統一建立 Serial 連線。"""
    return serial.Serial(
        port,
        baud,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_EVEN,
        stopbits=serial.STOPBITS_ONE,
        timeout=0.2,
        write_timeout=0.5,
    )


def safe_put(q: "queue.Queue[float]", val: float) -> None:
    """嘗試將數值放進 queue，不阻塞且忽略滿佇列。"""
    try:
        q.put_nowait(val)
    except queue.Full:
        pass


# ==============================
# Workers
# ==============================

class MoveWorker(threading.Thread):
    """行進輪：速度模式（rpm）"""

    def __init__(self, port: str):
        super().__init__(daemon=True, name="MoveWorker")
        self.port = port
        self.ser: Optional[serial.Serial] = None
        self.last_cmd_ts = time.time()

    def run(self) -> None:
        try:
            self.ser = open_serial(self.port, BAUD)
            init_speed_mode(self.ser, STATION_MOVE, acc_dec=200)
            logging.info("Move serial opened & speed mode initialized.")
        except Exception as e:
            logging.exception(f"MoveWorker init failed: {e}")
            stop_event.set()
            return

        try:
            while not stop_event.is_set():
                try:
                    # 取最新指令（若多筆就只留最後一筆）
                    speed = move_q.get(timeout=0.1)
                    while not move_q.empty():
                        speed = move_q.get_nowait()
                    self.last_cmd_ts = time.time()
                except queue.Empty:
                    # 超時檢查：太久沒收到控制 -> 安全停
                    if time.time() - self.last_cmd_ts > JOYSTICK_TIMEOUT_S:
                        speed = 0.0
                    else:
                        time.sleep(LOOP_SLEEP_S)
                        continue

                try:
                    set_speed(self.ser, float(speed), STATION_MOVE)
                except Exception as e:
                    logging.exception(f"set_speed failed: {e}")

                # （選用）讀狀態做監控列印
                try:
                    st = read_state_once(self.ser, STATION_MOVE, pulses_per_rev=PULSES_PER_REV)
                    logging.info(f"[MOVE] speed_cmd={speed:+.1f} rpm | actual={st['actual_speed_rpm']:.1f} rpm")
                except Exception as e:
                    logging.debug(f"read_state_once (MOVE) failed: {e}")

                time.sleep(LOOP_SLEEP_S)

        finally:
            try:
                if self.ser:
                    try:
                        set_speed(self.ser, 0, STATION_MOVE)
                    except Exception:
                        pass
                    self.ser.close()
                    logging.info("Move serial closed.")
            except Exception:
                pass


class SteerWorker(threading.Thread):
    """方向輪：位置模式（turns）"""

    def __init__(self, port: str, gear_ratio: float):
        super().__init__(daemon=True, name="SteerWorker")
        self.port = port
        self.gear_ratio = gear_ratio
        self.ser: Optional[serial.Serial] = None
        self.last_cmd_ts = time.time()
        self.last_direction: Optional[float] = None  # 用於判斷反向

    def run(self) -> None:
        try:
            self.ser = open_serial(self.port, BAUD)
            init_position_mode(self.ser, STATION_STEER, acc_dec=0xFF20, speed_rpm=3000)
            logging.info("Steer serial opened & position mode initialized.")
        except Exception as e:
            logging.exception(f"SteerWorker init failed: {e}")
            stop_event.set()
            return

        try:
            while not stop_event.is_set():
                try:
                    # 取最新指令（turns；正負決定方向）
                    turns = steer_q.get(timeout=0.1)
                    while not steer_q.empty():
                        turns = steer_q.get_nowait()
                    self.last_cmd_ts = time.time()
                except queue.Empty:
                    # 超時檢查：太久沒收到控制 -> 回 0
                    if time.time() - self.last_cmd_ts > JOYSTICK_TIMEOUT_S:
                        turns = 0.0
                    else:
                        time.sleep(LOOP_SLEEP_S)
                        continue

                # 反向立即停：新指令與上一筆方向相反
                if self.last_direction is not None and (turns * self.last_direction < 0):
                    try:
                        immediate_stop_if_not_reached(self.ser, STATION_STEER, tag="反向立即停")
                    except Exception as e:
                        logging.debug(f"immediate_stop_if_not_reached failed: {e}")

                self.last_direction = turns

                # 僅當馬達實際速度為 0 時才發新目標
                try:
                    st = read_state_once(self.ser, STATION_STEER, pulses_per_rev=PULSES_PER_REV)
                    if int(st["actual_speed_rpm"]) == 0:
                        move_to_position(
                            self.ser,
                            float(turns),
                            STATION_STEER,
                            use_turns=True,
                            gear_ratio=self.gear_ratio,
                        )
                    logging.info(
                        f"[STEER] turns_cmd={turns:+.3f} | pos={st.get('position_turns', 0.0):.3f} turns | "
                        f"speed={st.get('actual_speed_rpm', 0.0):.1f} rpm"
                    )
                except Exception as e:
                    logging.debug(f"read/move (STEER) failed: {e}")

                time.sleep(LOOP_SLEEP_S)

        finally:
            try:
                if self.ser:
                    try:
                        # 回歸安全位
                        move_to_position(self.ser, 0.0, STATION_STEER, use_turns=True, gear_ratio=self.gear_ratio)
                    except Exception:
                        pass
                    self.ser.close()
                    logging.info("Steer serial closed.")
            except Exception:
                pass


# ==============================
# Main
# ==============================

def main() -> None:
    # 啟動 workers
    move_worker = MoveWorker(SER1_PORT)
    steer_worker = SteerWorker(SER2_PORT, GEAR_RATIO)
    move_worker.start()
    steer_worker.start()

    js = init_joystick()
    logging.info("開始讀取搖桿... 按 Ctrl+C 結束。")

    try:
        while True:
            direction, movement, enable = read_joystick(js)

            # 依您原本邏輯：方向/行進取反
            direction = -float(direction)
            movement = -float(movement)

            logging.info(f"方向輪={direction:+.3f} | 行進輪={movement:+.3f} | 始能={'ON' if enable else 'OFF'}")

            if enable:
                safe_put(move_q, movement)
                safe_put(steer_q, direction)
            else:
                safe_put(move_q, 0.0)
                safe_put(steer_q, 0.0)

            time.sleep(0.01)

    except KeyboardInterrupt:
        logging.info("收到中斷，準備停止...")
    finally:
        stop_event.set()
        # 等待 Worker 善後
        time.sleep(0.5)
        logging.info("已停止。")


if __name__ == "__main__":
    main()
