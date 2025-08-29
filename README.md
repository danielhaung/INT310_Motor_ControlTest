# INT310 Motor Control – Modbus RTU + Joystick 驅動範例

> 以 **Python + Modbus RTU** 操控 INT310 直流伺服馬達，支援「行進輪（速度模式）」與「方向輪（絕對位置模式）」雙馬達控制，並可用 **搖桿** 實時操控。專案同時提供低階封包建構（CRC16、0x06/0x10 指令）、高階 API、狀態讀取（位置/速度）等。

![status](https://img.shields.io/badge/Python-3.10%2B-blue) ![license](https://img.shields.io/badge/License-MIT-green) ![platform](https://img.shields.io/badge/Platform-Linux%20\(Ubuntu\)-lightgrey)

---

## 目錄

* [功能特色](#功能特色)
* [硬體與連線](#硬體與連線)
* [軟體需求](#軟體需求)
* [安裝與設定](#安裝與設定)
* [快速開始](#快速開始)
* [程式架構](#程式架構)
* [主要 API 與範例](#主要-api-與範例)
* [搖桿映射](#搖桿映射)
* [安全與風險](#安全與風險)
* [疑難排解](#疑難排解)
* [暫存器對照 & 協議備註](#暫存器對照--協議備註)
* [Roadmap](#roadmap)
* [授權](#授權)

---

## 功能特色

* **雙馬達控制**：

  * 站號 `2`：速度模式（行進輪）。
  * 站號 `3`：絕對位置模式（方向輪），支援圈數/角度與減速比換算。
* **搖桿即時控制**：軸 0 控方向、軸 1 控速度，按鍵 0 為「始能」。
* **Modbus RTU 低階封包**：CRC16、0x06（單寫）、0x10（多寫）、32-bit 低字在前轉換。
* **高階 API**：`init_speed_mode()`、`set_speed()`、`init_position_mode()`、`move_to_position()`、`read_state_once()`。
* **狀態監視**：位置（counts / 角度）、速度（counts/s、RPM）。
* **可調參數**：加/減速、速度刻度、死區、放大倍率、減速比、解析度（pulses\_per\_rev）。

---

## 硬體與連線

* 馬達與驅動器：INT310（RS-485 Modbus RTU）
* 轉換器：USB-to-RS485（8N1 / 偶同位檢查）
* 預設序列埠：`/dev/ttyUSB0`
* 搖桿：任一支援的 USB Gamepad（`pygame` 可偵測）

> **線路**：USB↔RS485 轉換器接驅動器 `A/B`。請確認接地、終端電阻與屏蔽。

---

## 軟體需求

* Python 3.10+
* 套件：`pyserial`, `pygame`

```bash
pip install pyserial pygame
```

---

## 安裝與設定

1. **權限（Ubuntu）**
   將使用者加入 `dialout` 群組，避免每次用 `sudo`：

   ```bash
   sudo usermod -aG dialout $USER
   # 登出/重登生效
   ```

2. **裝置名稱固定（可選）**
   若有多個 USB 裝置，可建立 udev 規則固定名稱。

3. **Headless 環境（無桌面）**
   僅使用 `pygame.joystick` 無需顯示器；若使用 X11 相關插件報錯，請確保未啟用視窗功能。

---

## 快速開始

以下主程式範例會：

* 打開序列埠（115200, 8-E-1, timeout=0.2）
* 初始化：站 2（速度模式）、站 3（絕對位置）
* 持續讀取搖桿 → 控速/控向 → 讀回狀態顯示

```python
from pos_api import *
from vel_api import *
from state_api import *
import serial, time
from joystick_reader import init_joystick, read_joystick

ser = serial.Serial("/dev/ttyUSB0", 115200,
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_EVEN,
                    stopbits=serial.STOPBITS_ONE,
                    timeout=0.2)

# 行進輪：速度模式（站2）
init_speed_mode(ser, 2, acc_dec=200)

# 方向輪：絕對位置模式（站3）
init_position_mode(ser, 3, acc_dec=0x0D40, speed_rpm=500)

pulses_per_rev = 10000  # 內部四倍頻解析度
js = init_joystick()

try:
    while True:
        direction, movement, enable = read_joystick(js)
        print(f"方向輪={direction:+.3f} | 行進輪={movement:+.3f} | 始能={'ON' if enable else 'OFF'}")

        state_2 = read_state_once(ser, 2, pulses_per_rev=pulses_per_rev)
        state_3 = read_state_once(ser, 3, pulses_per_rev=pulses_per_rev)

        if enable:
            set_speed(ser, movement, 2)  # rpm
            move_to_position(ser, direction, 3, use_turns=True, gear_ratio=10)
            print(
                f"行進輪: {state_2['actual_speed_rpm']:.1f} rpm | "
                f"方向輪: 位置 {state_3['position_turns']:.2f} 度 | "
                f"速度 {state_3['actual_speed_rpm']:.2f} rpm"
            )
        else:
            set_speed(ser, 0, 2)
            move_to_position(ser, 0, 3, use_turns=True, gear_ratio=10)

        time.sleep(0.01)

except KeyboardInterrupt:
    print("停止監視")
finally:
    ser.close()
```

---

## 程式架構

```
.
├─ pos_api.py         # 絕對位置模式：初始化、移動、觸發、急停
├─ vel_api.py         # 速度模式：初始化、設速、始能
├─ state_api.py       # 狀態讀取：位置/速度 (Modbus 0x03)，單位換算工具
├─ joystick_reader.py # 搖桿封裝：死區、放大倍率、按鍵始能
├─ main.py            # 主流程（示例）
└─ README.md
```

---

## 主要 API 與範例

### 速度模式（行進輪，站號 2）

```python
# 初始化（RS485 控制、速度模式、加減速、始能）
init_speed_mode(ser, slave_id=2, acc_dec=200)

# 設定速度（rpm，可正負）
set_speed(ser, rpm=500, slave_id=2)
set_speed(ser, rpm=-1000, slave_id=2)
set_speed(ser, rpm=0, slave_id=2)
```

### 絕對位置模式（方向輪，站號 3）

```python
# 初始化位置模式，並設定速度參數（rpm）
init_position_mode(ser, slave_id=3, acc_dec=0x0D40, speed_rpm=500)

# 以「輸出端圈數」下達，並可指定減速比（馬達:輸出 = gear_ratio:1）
move_to_position(ser, pos_or_turns= 1.0, slave_id=3, use_turns=True, gear_ratio=10)
move_to_position(ser, pos_or_turns=-0.5, slave_id=3, use_turns=True, gear_ratio=10)

# 未達目標立即停止（安全輔助）
immediate_stop_if_not_reached(ser, slave_id=3)
```

### 狀態讀取（速度/位置）

```python
from state_api import read_state_once, DEFAULT_SPEED_SCALE

state = read_state_once(
    ser, slave=2,
    timeout_s=0.15,
    speed_scale=DEFAULT_SPEED_SCALE,   # 0.1：內部值->counts/s，如設備回饋已是 cps 就改 1.0
    pulses_per_rev=10000,
    angle_degrees=True                 # True 時 position_turns 以角度輸出
)
print(state)
# {
#   'actual_speed_cps': ...,
#   'actual_speed_rpm': ...,
#   'position_counts' : ...,
#   'position_turns'  : ...  # 角度(°) 或圈數（依 angle_degrees）
# }
```

### 低階封包（CRC/0x06/0x10）

* 以 `frame_06()`、`frame_10_words()` 建包，`tx()` 或 `send_frame()` 送出
* 32-bit 整數用 `int32_to_le_words()` 拆成「低字在前」

---

## 搖桿映射

* **Axis 0**：方向輪（`DIR_DEADZONE=0.20`，`DIR_SCALE=1` → 直接對應圈數）
* **Axis 1**：行進輪，Y 軸反向（`MOVE_DEADZONE=0.20`，`MOVE_SCALE=500` → rpm）
* **Button 0**：始能（按下 ON）
* 始能關閉時，速度與角度指令自動歸零。

> 可直接在 `joystick_reader.py` 中調整死區與倍率，匹配實機手感。

---

## 安全與風險

* **先空載測試**：確認轉向與比例正確後再掛載。
* **限速/限位**：請在驅動器或上位機加入速限與角度限制。
* **通訊中斷**：若 RS485 中斷，驅動器可能保持最後命令；建議：

  * 啟用驅動器內建「通訊逾時停機」；
  * 在主程式加 **看門狗/心跳**（定期下達 0 速或觸發位）。
* **緊急停止**：硬體 E-STOP 仍為最後保護機制。

---

## 疑難排解

* **`could not connect to display` / Qt xcb**：本專案不開視窗；若環境帶 Qt，請改以純終端執行或安裝 `libxcb-cursor0`。
* **`/dev/ttyUSB0` 無法開啟**：加入 `dialout`、檢查權限與線路、確認 `PARITY_EVEN`。
* **速度卡在 \~500 rpm**：檢查驅動器內部「速度前饋/限制」或模式參數（`D653` 等）；確認加/減速（`0x002A/0x002C`）與速度參數是否正確寫入。
* **搖桿偵測不到**：`pygame.joystick.get_count()==0` → 驗證系統是否辨識（`lsusb`）、權限、或改用其他搖桿。
* **方向輪角度漂移**：確認 `pulses_per_rev` 與 `gear_ratio` 設定；必要時加入「包角」（-180\~+180°）邏輯以避免累積。

---

## 暫存器對照 & 協議備註

**模式/控制（常用）**

* `D110 (0x006E)`：`0x001E` → RS485 控制
* `D653 (0x028D)`：`0x0067` 速度模式 / `0x0001` 位置模式
* `D140~ (0x008A)`：加速值（含高/低字；同位址用 0x10 多寫）
* `D142~ (0x008C)`：減速值
* 位置命令：`D130 (0x0082)`（32-bit，低字在前）
* 速度參數：`0x0084`（32-bit，低字在前；作為位置模式的運動速度）
* 觸發/始能：`0x028C`（`0x000F` 預備、`0x001F` 觸發上升沿、`0x0000` OFF、`0x012F` 未達即停）

**狀態/監看**

* 位置回饋（32-bit LE）：`MODBUS(DEC) 616`
* 實際速度（32-bit LE）：`MODBUS(DEC) 630`（預設 `*0.1` → counts/sec；可於 `DEFAULT_SPEED_SCALE` 調整）

**轉換係數**

* 速度內部值：`internal = round(rpm * 5_000_000 / 3000)` → 拆成低字在前兩個 16-bit。
* 角度/圈數換算：

  * counts → turns：`counts / pulses_per_rev`
  * counts → degrees：`(counts / pulses_per_rev) * 360`

> 各位址以實機手冊為準；若型號差異，請對照修改。

---

## Roadmap

* [ ] 增加 **心跳機制** 與 **通訊逾時保護** 範例
* [ ] 提供 **systemd 服務** 啟動腳本（開機自動執行）
* [ ] Web HMI（狀態看板 + 指令面板）
* [ ] 單元測試與模擬器（離線回放）

---

## 授權

本專案以 **MIT License** 授權。請於工安與法規允許範圍內使用，作者不對任何直接或間接損害負責。

---

### 致貢獻者

歡迎 PR：Bug 修正、文件補充、支援更多驅動器/暫存器、或加入不同控制器與感測器的適配層 🙌

---

> **Tips**：若你要把這份 README 直接放入 GitHub，檔名請用 `README.md`，並依你的實際檔案結構調整匯入路徑與站號設定。
