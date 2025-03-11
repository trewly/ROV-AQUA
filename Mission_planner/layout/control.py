import sys
import cv2
import os
import numpy as np
from PyQt5 import QtCore, QtWidgets
from PyQt5.QtWidgets import QApplication, QLabel, QWidget, QPushButton
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import QTimer, QThread, pyqtSignal, Qt

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../.."))) 

from Mission_planner.controller.joystick import VirtualJoystick
from Mission_planner.controller import control_buttons as buttons_controller
from Mission_planner.controller.video import VideoReceiver

class Ui_Form(QWidget):
    def __init__(self):
        super().__init__()
        self.shift_timer = QTimer(self)
        self.ctrl_timer = QTimer(self)

        self.shift_timer.timeout.connect(lambda: self.surface_button.click())
        self.ctrl_timer.timeout.connect(lambda: self.dive_button.click())

        self.light_on = False

        self.setWindowTitle("ROV Control")
        self.setGeometry(100, 100, 960, 540)

        self.video_receiver = VideoReceiver(parent=self)

        self.overlay_widget = QWidget(self)
        self.overlay_widget.setGeometry(0, 0, 960, 540)
        self.overlay_widget.setAttribute(Qt.WA_TranslucentBackground)
        self.overlay_widget.setWindowFlags(Qt.FramelessWindowHint)

        btn_width = 100
        btn_height = 50
        btn_x = 850
        btn_y_start = 10
        btn_spacing = 60

        self.surface_button = QPushButton("Surface", self.overlay_widget)
        self.surface_button.setGeometry(btn_x, btn_y_start, btn_width, btn_height)
        self.surface_button.clicked.connect(buttons_controller.on_surface_button_clicked)

        self.dive_button = QPushButton("Dive", self.overlay_widget)
        self.dive_button.setGeometry(btn_x, btn_y_start + btn_spacing, btn_width, btn_height)
        self.dive_button.clicked.connect(buttons_controller.on_dive_button_clicked)

        self.mode_change_button = QPushButton("MODE CHANGE", self.overlay_widget)
        self.mode_change_button.setGeometry(btn_x, btn_y_start + 2 * btn_spacing, btn_width, btn_height)
        self.mode_change_button.clicked.connect(buttons_controller.on_mode_change_button_clicked)

        self.light_button = QPushButton("LIGHT", self.overlay_widget)
        self.light_button.setGeometry(btn_x, btn_y_start + 3 * btn_spacing, btn_width, btn_height)
        self.light_button.clicked.connect(lambda: buttons_controller.on_light_button_clicked(self))

        self.joystick = VirtualJoystick(self.overlay_widget)
        self.joystick.setGeometry(QtCore.QRect(10, 370, 120, 120))

    def keyPressEvent(self, event):
        if event.key() == Qt.Key_Shift and not self.shift_timer.isActive():
            self.shift_timer.start(100)
        elif event.key() == Qt.Key_Control and not self.ctrl_timer.isActive():
            self.ctrl_timer.start(100)
        elif event.key() == Qt.Key_L:
            self.light_button.click()
        else:
            super().keyPressEvent(event)

    def keyReleaseEvent(self, event):
        if event.key() == Qt.Key_Shift:
            self.shift_timer.stop()
        elif event.key() == Qt.Key_Control:
            self.ctrl_timer.stop()
        else:
            super().keyReleaseEvent(event)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    MainWindow = Ui_Form()
    MainWindow.show()
    sys.exit(app.exec_())