from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QGridLayout, QHBoxLayout, QPushButton
from PyQt5.QtWidgets import QLabel
from PyQt5.QtGui import QFont, QPalette, QColor
from PyQt5.QtCore import Qt, QThread, pyqtSignal

import sys
import os

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

# Import canvas từ module
from layout.sub_widgets import canvas, motor_slider, setting_area
from Mission_planner.status import pc_status as status
from Mission_planner.communication.system_update_timer import SystemStatusManager

class settingLayout(QWidget):
    def __init__(self, status_manager: SystemStatusManager):
        super().__init__()
        self.setFixedSize(910,944)
        palette = self.palette()
        palette.setColor(QPalette.Window, QColor("#395B64"))  # Màu xanh lá
        self.setPalette(palette)
        self.setAutoFillBackground(True)
         
        self.status_manager = status_manager    
        
        self.canvas_init()
        self.motor_slider_init()
        self.button_setting_init()

    def canvas_init(self):
        self.canvas = canvas.CanvasWidget(self.status_manager)
        self.canvas.setParent(self)
        self.canvas.move(15,20)

    def motor_slider_init(self):
        
        setting=QLabel("Setting",self)
        setting.setFont(QFont("Roboto", 25, QFont.Bold))
        setting.setStyleSheet("color: #F3F3E0;")
        setting.move(30, 635)

        self.motor_slider = motor_slider.MotorSlider()
        self.motor_slider.setParent(self)
        self.motor_slider.move(0, 690)

    def button_setting_init(self):
        self.setting_area = setting_area.settingWidget()
        self.setting_area.setParent(self)
        self.setting_area.move(600, 610)


class StatusReaderThread(QThread):
    status_signal = pyqtSignal(str, object)

    def __init__(self, key, parent=None):
        super().__init__(parent)
        self.key = key
        self.running = True

    def run(self):
        while self.running:
            try:
                value = status.read_status(self.key)
                self.status_signal.emit(self.key, value)
            except Exception as e:
                print(f"Error reading status: {e}")

    def stop(self):
        self.running = False
        self.wait()


class MainWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Status Reader")
        self.resize(400, 200)

        self.label = QLabel("Waiting for status...", self)

        layout = QVBoxLayout()
        layout.addWidget(self.label)
        self.setLayout(layout)

        self.status_thread = StatusReaderThread(key="depth")
        self.status_thread.status_signal.connect(self.update_label)
        self.status_thread.start()

    def update_label(self, key, value):
        self.label.setText(f"{key}: {value}")

    def closeEvent(self, event):
        self.status_thread.stop()
        event.accept()

