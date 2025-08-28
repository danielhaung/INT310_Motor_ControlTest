import sys, time, math, signal
from PyQt6.QtWidgets import QApplication, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QGroupBox
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QFont

class DataSource:
    """範例資料源：用正弦波模擬速度，並數值積分出位置。
       之後你把 read() 換成實際讀值即可（例如從串口、TCP、共享記憶體…）。"""
    def __init__(self):
        self.t0 = time.monotonic()
        self.prev_t = self.t0
        self.position_m = 0.0

    def read(self):
        t = time.monotonic() - self.t0
        # 模擬速度（m/s）：介於 0.2~1.2
        speed_mps = 0.7 + 0.5*math.sin(2*math.pi*0.2*t)
        # 積分出位置
        now = time.monotonic()
        dt = now - self.prev_t
        self.prev_t = now
        self.position_m += max(speed_mps, 0.0) * dt
        return speed_mps, self.position_m

class HMIMain(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("HMI - 速度/位置")
        self.ds = DataSource()

        # 大字體樣式
        self.value_font = QFont("DejaVu Sans Mono", 44, QFont.Weight.Bold)
        self.label_font = QFont("Noto Sans", 16)

        # 元件
        self.speed_value = QLabel("--.--")
        self.speed_value.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.speed_value.setFont(self.value_font)

        self.pos_value = QLabel("--.--")
        self.pos_value.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.pos_value.setFont(self.value_font)

        speed_box = QGroupBox("速度 (m/s)")
        speed_box.setFont(self.label_font)
        sv = QVBoxLayout(speed_box); sv.addWidget(self.speed_value)

        pos_box = QGroupBox("位置 (m)")
        pos_box.setFont(self.label_font)
        pv = QVBoxLayout(pos_box); pv.addWidget(self.pos_value)

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

        root = QVBoxLayout(self)
        root.addWidget(speed_box)
        root.addWidget(pos_box)
        root.addLayout(btn_row)

        # 每 100ms 更新一次
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_values)
        self.timer.start(100)

        # 訊號處理：Ctrl+C / systemd 停止能優雅退出
        signal.signal(signal.SIGINT,  lambda *_: QApplication.quit())
        signal.signal(signal.SIGTERM, lambda *_: QApplication.quit())

        self.resize(900, 520)

    def update_values(self):
        speed_mps, position_m = self.ds.read()
        self.speed_value.setText(f"{speed_mps:6.2f}")
        self.pos_value.setText(f"{position_m:9.2f}")

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

