from PyQt5.QtWidgets import QWidget, QPushButton, QGridLayout, QApplication, QStackedLayout, QVBoxLayout, QHBoxLayout, QFrame
from PyQt5.QtGui import QPalette, QColor
from PyQt5.QtCore import Qt, QTimer, pyqtSignal

import sys
import os
import time

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../../..")))

from Mission_planner.layout.resources.style import control_button_style
from Mission_planner.layout.contents.motor_slider import MotorSlider
from Mission_planner.controller.video_controller import VideoReceiver
from Mission_planner.controller import button_controller as ButtonController


class ControlButton(QPushButton):
    def __init__(self, text, parent=None):
        super().__init__(text, parent)  
        self.setMinimumSize(100, 35)
        self.setStyleSheet(control_button_style)
        self.setCursor(Qt.PointingHandCursor)

class DirectionButton(QPushButton):
    def __init__(self, text, parent=None):
        super().__init__(text, parent)
        self.setMinimumSize(60, 60)
        self.setMaximumSize(60, 60)
        
        self.setStyleSheet("""
            QPushButton {
                background-color: #2C3333;
                color: white;
                border: none;
                border-radius: 30px;
                font-weight: bold;
                font-size: 24px;
            }
            QPushButton:hover {
                background-color: #395B64;
            }
            QPushButton:pressed {
                background-color: #A5C9CA;
                color: #2C3333;
            }
        """)
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
        
        self.direction_container = QFrame(self)
        self.direction_container.setGeometry(10, 570, 200, 200)
        self.direction_container.setFrameShape(QFrame.StyledPanel)
        self.direction_container.setStyleSheet("background-color: #395B64; border-radius: 10px;")

        direction_layout = QGridLayout(self.direction_container)
        direction_layout.setContentsMargins(10, 10, 10, 10)
        
        self.forward_button = DirectionButton("▲")
        self.backward_button = DirectionButton("▼")
        self.left_button = DirectionButton("◄")
        self.right_button = DirectionButton("►")
        
        direction_layout.addWidget(self.forward_button, 0, 1)
        direction_layout.addWidget(self.left_button, 1, 0)
        direction_layout.addWidget(self.right_button, 1, 2)
        direction_layout.addWidget(self.backward_button, 2, 1)
        
        direction_layout.setSpacing(10)
        
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
        
        buttons_layout.addWidget(self.surface_button)
        buttons_layout.addWidget(self.dive_button)
        buttons_layout.addWidget(self.roll_right_button)
        buttons_layout.addWidget(self.roll_left_button)
        buttons_layout.addWidget(self.mode_change_button)

        self.motoSlider = MotorSlider()
        self.motoSlider.setParent(self)
        self.motoSlider.setGeometry(280, 600, 755, 180)
        
        self.connect_signals()
        
    def connect_signals(self):
        self.forward_button.pressed.connect(
            self.on_button_activity(ButtonController.controller.on_forward_button_clicked))
        self.forward_button.released.connect(
            self.on_button_activity(ButtonController.controller.on_forward_button_released, False))
            
        self.backward_button.pressed.connect(
            self.on_button_activity(ButtonController.controller.on_backward_button_clicked))
        self.backward_button.released.connect(
            self.on_button_activity(ButtonController.controller.on_backward_button_released, False))
            
        self.left_button.pressed.connect(
            self.on_button_activity(ButtonController.controller.on_left_button_clicked))
        self.left_button.released.connect(
            self.on_button_activity(ButtonController.controller.on_left_button_released, False))
            
        self.right_button.pressed.connect(
            self.on_button_activity(ButtonController.controller.on_right_button_clicked))
        self.right_button.released.connect(
            self.on_button_activity(ButtonController.controller.on_right_button_released, False))
        
        self.surface_button.pressed.connect(
            self.on_button_activity(ButtonController.controller.on_surface_button_clicked))
        self.surface_button.released.connect(
            self.on_button_activity(ButtonController.controller.on_surface_button_released, False))
        
        self.dive_button.pressed.connect(
            self.on_button_activity(ButtonController.controller.on_dive_button_clicked))
        self.dive_button.released.connect(
            self.on_button_activity(ButtonController.controller.on_dive_button_released, False))

        self.roll_right_button.pressed.connect(
            self.on_button_activity(ButtonController.controller.on_roll_right_button_clicked))
        self.roll_right_button.released.connect(
            self.on_button_activity(ButtonController.controller.on_roll_right_button_released, False))

        self.roll_left_button.pressed.connect(
            self.on_button_activity(ButtonController.controller.on_roll_left_button_clicked))
        self.roll_left_button.released.connect(
            self.on_button_activity(ButtonController.controller.on_roll_left_button_released, False))
        
        self.mode_change_button.clicked.connect(
            self.on_button_activity(ButtonController.controller.on_mode_change_button_clicked))
        
    def on_button_activity(self, func, input_active=True):
        def wrapper(*args, **kwargs):
            self.register_activity()
            self.input_active = input_active
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
            pass
    
    def keyPressEvent(self, event):
        self.register_activity()
        self.input_active = True
        if not ButtonController.controller.handle_key_press(event):
            super().keyPressEvent(event)

    def keyReleaseEvent(self, event):
        self.register_activity()
        self.input_active = False
        if not ButtonController.controller.handle_key_release(event):
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
        ButtonController.controller.cleanup()
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = ControlWidget()
    window.show()
    sys.exit(app.exec_())