from PyQt5.QtWidgets import QWidget, QPushButton,  QGridLayout, QApplication
from PyQt5.QtGui import QPalette, QColor
import sys
import os

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from resources.style import setting_button_style

from sub_widgets.pid_setting import PIDTuner
from sub_widgets.calibration import calibration

class SettingWidget(QWidget):
    def __init__(self):
        super().__init__()
        self.setFixedSize(985, 162)
        palette = self.palette()
        palette.setColor(QPalette.Window, QColor("#395B64"))
        self.setAutoFillBackground(True)
        self.setPalette(palette)
        self.settinglayout = QGridLayout()
        self.settinglayout.setSpacing(20)

        self.button_init()
        self.setLayout(self.settinglayout)

    def button_init(self):
        self.button_1 = QPushButton("Calibration")
        self.button_2 = QPushButton("PID Tuning")
        self.button_3 = QPushButton("System check")
        self.button_4 = QPushButton("Failsafe mechanism")

        buttons = [self.button_1, self.button_2, self.button_3, self.button_4]

        for i, button in enumerate(buttons):
            button.setFixedSize(441, 50)
            button.setStyleSheet(setting_button_style)
            row = i // 2  # 0 hoặc 1
            col = i % 2   # 0 hoặc 1
            self.settinglayout.addWidget(button, row, col)
            button.clicked.connect(lambda _, b=i+1: self.button_clicked(b))

    def button_clicked(self, button_number):
        if button_number == 1:
            self.calib= calibration()
            self.calib.show()
        if button_number == 2:
            self.pid_tuning=PIDTuner()
            self.pid_tuning.show()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = SettingWidget()
    window.show()
    sys.exit(app.exec_())
