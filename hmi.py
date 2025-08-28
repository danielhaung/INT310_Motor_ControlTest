import sys, time, math, signal
from typing import Optional, Union

from PySide6.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QPushButton, QGroupBox
)
from PySide6.QtCore import Qt, QTimer, QPointF, QRectF, Signal
from PySide6.QtGui import QFont, QPainter, QPen, QBrush, QColor


FloatOrNone = Optional[Union[float, int]]


class DataSource:
    """
    範例資料源：
    - 預設用正弦波模擬速度，並以數值積分得到位置。
    - 若啟用搖桿「手動速度 manual_speed_mps」則優先用該值。
    之後您把 read() 或 manual_speed_mps 改成實機邏輯即可。
    """
    def __init__(self):
        self.t0 = time.monotonic()
        self.prev_t = self.t0
        self.position_m = 0.0
        self.manual_speed_mps: FloatOrNone = None  # 當為 float 時使用，為 None 時用模擬

    def set_manual_speed(self, v: FloatOrNone):
        self.manual_speed_mps = None if v is None else float(v)

    def read(self):
        # 速度來源
        if self.manual_speed_mps is None:
            t = time.monotonic() - self.t0
            speed_mps = 0.7 + 0.5 * math.sin(2 * math.pi * 0.2 * t)  # 0.2~1.2 m/s
        else:
            speed_mps = float(self.manual_speed_mps)  # 由搖桿提供

        # 數值積分得到位置
        now = time.monotonic()
        dt = now - self.prev_t
        self.prev_t = now
        self.position_m += speed_mps * dt
        return speed_mps, self.position_m


class JoystickPad(QWidget):
    """
    圓形搖桿：
    - 拖曳圓點輸出 (-1..+1, -1..+1) 的 (x,y) 值（相對於中心）。
    - 鬆開時自動以阻尼方式回到 (0,0)，並持續發送 moved 訊號直到歸零。
    - 對外採 y 軸「上為正」。
    """
    moved = Signal(float, float)        # (x, y) in [-1, 1]
    released_center = Signal()          # 回到原點時發送（可選用）

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(220, 220)
        self._radius_px = 0.0
        self._stick_norm = QPointF(0.0, 0.0)  # 目前標準化座標
        self._dragging = False

        # 回中動畫（簡易阻尼）
        self._spring_timer = QTimer(self)
        self._spring_timer.timeout.connect(self._spring_back_step)
        self._spring_timer.setInterval(16)  # 約 60 FPS

    # ---- 互動 ----
    def mousePressEvent(self, e):
        if e.button() == Qt.LeftButton:
            self._dragging = True
            self._spring_timer.stop()
            self._update_from_pos(e.position())

    def mouseMoveEvent(self, e):
        if self._dragging:
            self._update_from_pos(e.position())

    def mouseReleaseEvent(self, e):
        if e.button() == Qt.LeftButton:
            self._dragging = False
            # 啟動回中心
            self._spring_timer.start()

    # ---- 計算 ----
    def _update_from_pos(self, posf: QPointF):
        r = self._radius_px if self._radius_px > 0 else 1.0
        center = self.rect().center()
        dx = (posf.x() - center.x()) / r
        dy = (posf.y() - center.y()) / r

        # 限制在單位圓內
        mag2 = dx*dx + dy*dy
        if mag2 > 1.0:
            mag = mag2 ** 0.5
            dx /= mag
            dy /= mag

        self._stick_norm = QPointF(dx, dy)
        # 對外 y 朝上（UI 座標往下為正，需取反）
        self.moved.emit(self._stick_norm.x(), -self._stick_norm.y())
        self.update()

    def _spring_back_step(self):
        # 線性阻尼回原點
        k = 0.22  # 回彈係數（越大回中越快）
        nx = self._stick_norm.x() * (1.0 - k)
        ny = self._stick_norm.y() * (1.0 - k)

        if abs(nx) < 0.001 and abs(ny) < 0.001:
            self._stick_norm = QPointF(0.0, 0.0)
            self._spring_timer.stop()
            self.moved.emit(0.0, 0.0)
            self.released_center.emit()
        else:
            self._stick_norm = QPointF(nx, ny)
            self.moved.emit(self._stick_norm.x(), -self._stick_norm.y())

        self.update()

    # ---- 外觀 ----
    def paintEvent(self, _):
        w, h = self.width(), self.height()
        R = min(w, h) * 0.45
        self._radius_px = R

        center = QPointF(w/2, h/2)
        stick_px = QPointF(center.x() + self._stick_norm.x() * R,
                           center.y() + self._stick_norm.y() * R)

        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing, True)

        # 背景
        p.fillRect(self.rect(), QBrush(QColor(18, 18, 18)))

        # 外圈
        p.setPen(QPen(QColor(90, 90, 90), 3))
        p.setBrush(QBrush(QColor(35, 35, 35)))
        p.drawEllipse(QRectF(center.x()-R, center.y()-R, 2*R, 2*R))

        # 十字刻度
        p.setPen(QPen(QColor(70, 70, 70), 2))
        p.drawLine(int(center.x()-R), int(center.y()), int(center.x()+R), int(center.y()))
        p.drawLine(int(center.x()), int(center.y()-R), int(center.x()), int(center.y()+R))

        # 搖桿圓頭
        p.setPen(Qt.NoPen)
        p.setBrush(QBrush(QColor(0, 170, 255, 220)))
        p.drawEllipse(QRectF(stick_px.x()-16, stick_px.y()-16, 32, 32))

        # 中心點
        p.setBrush(QBrush(QColor(200, 200, 200, 160)))
        p.drawEllipse(QRectF(center.x()-4, center.y()-4, 8, 8))


class HMIMain(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("HMI - 速度/位置 + 搖桿 (PySide6)")
        self.ds = DataSource()

        # 字體
        self.value_font = QFont("DejaVu Sans Mono", 44, QFont.Weight.Bold)
        self.label_font = QFont("Noto Sans", 16)

        # 數值顯示
        self.speed_value = QLabel("--.--")
        self.speed_value.setAlignment(Qt.AlignCenter)
        self.speed_value.setFont(self.value_font)

        self.pos_value = QLabel("--.--")
        self.pos_value.setAlignment(Qt.AlignCenter)
        self.pos_value.setFont(self.value_font)

        speed_box = QGroupBox("速度 (m/s)")
        speed_box.setFont(self.label_font)
        sv = QVBoxLayout(speed_box); sv.addWidget(self.speed_value)

        pos_box = QGroupBox("位置 (m)")
        pos_box.setFont(self.label_font)
        pv = QVBoxLayout(pos_box); pv.addWidget(self.pos_value)

        # 搖桿與讀數
        self.joy = JoystickPad()
        joy_box = QGroupBox("搖桿（按住控制，鬆開回中）")
        joy_box.setFont(self.label_font)
        jv = QVBoxLayout(joy_box)
        jv.addWidget(self.joy)

        self.jxy_label = QLabel("X: 0.00   Y: 0.00   → cmd v = 0.00 m/s")
        self.jxy_label.setAlignment(Qt.AlignCenter)
        self.jxy_label.setFont(QFont("DejaVu Sans Mono", 14))
        jv.addWidget(self.jxy_label)

        # 按鈕列
        btn_full = QPushButton("全螢幕")
        btn_exit = QPushButton("退出")
        btn_reset = QPushButton("位置歸零")
        btn_full.setFont(self.label_font)
        btn_exit.setFont(self.label_font)
        btn_reset.setFont(self.label_font)

        btn_full.clicked.connect(self.toggle_fullscreen)
        btn_exit.clicked.connect(self.close)
        btn_reset.clicked.connect(self.reset_position)

        btn_row = QHBoxLayout()
        btn_row.addWidget(btn_full)
        btn_row.addWidget(btn_reset)
        btn_row.addStretch(1)
        btn_row.addWidget(btn_exit)

        # 版面
        left_col = QVBoxLayout()
        left_col.addWidget(speed_box)
        left_col.addWidget(pos_box)
        left_col.addStretch(1)

        main_row = QHBoxLayout()
        main_row.addLayout(left_col, 1)
        main_row.addWidget(joy_box, 1)

        root = QVBoxLayout(self)
        root.addLayout(main_row, 1)
        root.addLayout(btn_row)

        # 定時更新
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_values)
        self.timer.start(100)

        # 訊號：Ctrl+C / systemd 停止能優雅退出
        signal.signal(signal.SIGINT,  lambda *_: QApplication.quit())
        signal.signal(signal.SIGTERM, lambda *_: QApplication.quit())

        # 搖桿連接：將 Y 軸轉成速度命令（例：±1.5 m/s）
        self.max_cmd_speed = 1.5  # 您可以依馬達上限調整
        self.ds.set_manual_speed(None)  # 初始為模擬
        self.joy.moved.connect(self.on_joystick_move)
        self.joy.released_center.connect(self.on_joystick_centered)

        self.resize(1000, 560)

    # ---- 搖桿事件 ----
    def on_joystick_move(self, x_norm: float, y_norm: float):
        cmd_v = y_norm * self.max_cmd_speed
        # 使用搖桿 → 啟用手動速度
        self.ds.set_manual_speed(cmd_v)
        self.jxy_label.setText(f"X: {x_norm:+.2f}   Y: {y_norm:+.2f}   → cmd v = {cmd_v:+.2f} m/s")

    def on_joystick_centered(self):
        # 需要「鬆手回原點（機械零）」→ 在此下達回零命令
        # 目前維持速度命令 0.0，由 _spring_back_step() 觸發 moved(0,0)
        pass

    # ---- UI 動作 ----
    def update_values(self):
        speed_mps, position_m = self.ds.read()
        self.speed_value.setText(f"{speed_mps:6.2f}")
        self.pos_value.setText(f"{position_m:9.2f}")

        # 若目前沒有搖桿操控（manual_speed_mps 為 None）則顯示「模擬」
        if self.ds.manual_speed_mps is None:
            self.jxy_label.setText("模擬速度中（使用搖桿即可改為手動）")

    def toggle_fullscreen(self):
        if self.isFullScreen():
            self.showNormal()
        else:
            self.showFullScreen()

    def reset_position(self):
        self.ds.position_m = 0.0


def main():
    app = QApplication(sys.argv)
    ui = HMIMain()
    ui.show()  # 想一開啟就全螢幕：改成 ui.showFullScreen()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
