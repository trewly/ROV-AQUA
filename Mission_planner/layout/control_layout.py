from PyQt5.QtWidgets import QWidget, QPushButton, QGridLayout, QApplication, QStackedLayout, QVBoxLayout, QHBoxLayout, QFrame
from PyQt5.QtGui import QPalette, QColor
from PyQt5.QtCore import Qt, QTimer, pyqtSignal

import sys
import os
import time
from functools import partial

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from resources.style import control_button_style

from layout.sub_widgets import motor_slider
from controller.video_controller import VideoReceiver
from controller.mode_controller import ModeChangeDialog
from controller.joystick_controller import VirtualJoystick
from controller.button_controller import ButtonController

class ControlButton(QPushButton):
    def __init__(self, text, parent=None):
        super().__init__(text, parent)
        self.setMinimumSize(100, 35)
        self.setStyleSheet(control_button_style)
        self.setCursor(Qt.PointingHandCursor)

class ControlWidget(QWidget):
    activityDetected = pyqtSignal()

    IDLE_TIMEOUT = 100

    def __init__(self):
        super().__init__()
        self.light_on = False
        self.last_activity_time = time.time()
        self.is_active = False
        self.setFocusPolicy(Qt.StrongFocus)
        
        self.idle_timer = QTimer(self)
        self.idle_timer.timeout.connect(self.check_idle_time)
        
        self.input_active = False

        self.setFixedSize(985, 785)

        palette = self.palette()
        palette.setColor(QPalette.Window, QColor("#395B64"))
        self.setAutoFillBackground(True)
        self.setPalette(palette)
        
        self.video_container = QWidget(self)
        self.video_container.setGeometry(10, 10, 960, 540)
        
        video_layout = QVBoxLayout(self.video_container)
        video_layout.setContentsMargins(0, 0, 0, 0)
        
        self.video_receiver = VideoReceiver(parent=None) 
        self.video_receiver.setFixedSize(960, 540)
        video_layout.addWidget(self.video_receiver)
        
        self.joystick_container = QFrame(self)
        self.joystick_container.setGeometry(10, 570, 200, 200)
        self.joystick_container.setFrameShape(QFrame.StyledPanel)
        self.joystick_container.setStyleSheet("background-color: #395B64; border-radius: 10px;")

        joystick_layout = QVBoxLayout(self.joystick_container)
        joystick_layout.setContentsMargins(10, 10, 10, 10)
        joystick_layout.setAlignment(Qt.AlignCenter)
        
        self.joystick = VirtualJoystick()
        self.joystick.setFocusPolicy(Qt.StrongFocus)
        joystick_layout.addWidget(self.joystick)
        
        self.buttons_container = QFrame(self)
        self.buttons_container.setGeometry(220, 550, 755, 50)
        self.buttons_container.setFrameShape(QFrame.StyledPanel)
        self.buttons_container.setStyleSheet("background-color: #395B64; border-radius: 10px;")
        
        buttons_layout = QHBoxLayout(self.buttons_container)
        buttons_layout.setContentsMargins(10, 5, 10, 5)
        buttons_layout.setSpacing(2)
        
        self.surface_button = ControlButton("SURFACE")
        self.dive_button = ControlButton("DIVE")
        self.roll_right_button = ControlButton("ROLL RIGHT")
        self.roll_left_button = ControlButton("ROLL LEFT")
        self.mode_change_button = ControlButton("MODE")
        self.connect_signals()

        buttons_layout.addWidget(self.surface_button)
        buttons_layout.addWidget(self.dive_button)
        buttons_layout.addWidget(self.roll_right_button)
        buttons_layout.addWidget(self.roll_left_button)
        buttons_layout.addWidget(self.mode_change_button)

        self.motoSlider = motor_slider.MotorSlider()
        self.motoSlider.setParent(self)
        self.motoSlider.setGeometry(280, 600, 755, 180)
        
    def connect_signals(self):
        self.surface_button.pressed.connect(self.wrap_activity_tracking(
            ButtonController.on_surface_button_clicked))
        self.surface_button.released.connect(self.wrap_activity_tracking(
            ButtonController.on_surface_button_released))
        
        self.dive_button.pressed.connect(self.wrap_activity_tracking(
            ButtonController.on_dive_button_clicked))
        self.dive_button.released.connect(self.wrap_activity_tracking(
            ButtonController.on_dive_button_released))

        self.roll_right_button.pressed.connect(self.wrap_activity_tracking(
            ButtonController.on_roll_right_button_clicked))
        self.roll_right_button.released.connect(self.wrap_activity_tracking(
            ButtonController.on_roll_right_button_released))

        self.roll_left_button.pressed.connect(self.wrap_activity_tracking(
            ButtonController.on_roll_left_button_clicked))
        self.roll_left_button.released.connect(self.wrap_activity_tracking(
            ButtonController.on_roll_left_button_released))
        
        self.mode_change_button.clicked.connect(self.wrap_activity_tracking(
            ButtonController.on_mode_change_button_clicked))
        
    
    def wrap_activity_tracking(self, func):
        def wrapper(*args, **kwargs):
            self.register_activity()
            self.input_active = True
            result = func(*args, **kwargs)
            return result
        return wrapper
    
    def register_activity(self):
        self.last_activity_time = time.time()
        self.activityDetected.emit()
    
    def check_idle_time(self):
        if not self.is_active:
            return
        
        current_time = time.time()
        idle_time_ms = (current_time - self.last_activity_time) * 1000
        
        if idle_time_ms >= self.IDLE_TIMEOUT and not self.input_active:
            if self.isVisible() and not self.joystick.hasFocus():
                self.joystick.setFocus()
    
    def keyPressEvent(self, event):
        self.register_activity()
        self.input_active = True
        if not ButtonController.handle_key_press(self, event):
            super().keyPressEvent(event)

    def keyReleaseEvent(self, event):
        self.register_activity()
        self.input_active = False
        if not ButtonController.handle_key_release(event):
            super().keyReleaseEvent(event)

    def mousePressEvent(self, event):
        self.register_activity()
        self.input_active = True
        super().mousePressEvent(event)
        
    def mouseReleaseEvent(self, event):
        self.register_activity()
        self.input_active = False
        super().mouseReleaseEvent(event)

    def closeEvent(self, event):
        if self.idle_timer.isActive():
            self.idle_timer.stop()
        self.video_receiver.closeEvent(event)
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = ControlWidget()
    window.show()
    sys.exit(app.exec_())