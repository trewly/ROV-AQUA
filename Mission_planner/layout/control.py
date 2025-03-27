import sys
import os
from functools import partial
import time

from PyQt5 import QtCore
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QVBoxLayout, QHBoxLayout, QGridLayout, QFrame
from PyQt5.QtCore import Qt, QTimer, pyqtSignal

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../..")))

from Mission_planner.layout.resources import style as st

from Mission_planner.controller.joystick_controller import VirtualJoystick
from Mission_planner.controller.button_controller import ButtonController
from Mission_planner.controller.video_controller import VideoReceiver

class ControlButton(QPushButton):
    def __init__(self, text, parent=None):
        super().__init__(text, parent)
        self.setMinimumSize(100, 30)
        self.setStyleSheet(st.control_button_style)
        self.setCursor(Qt.PointingHandCursor)

class ControlPanel(QWidget):
    activityDetected = pyqtSignal()
    
    IDLE_TIMEOUT = 100
    
    def __init__(self):
        super().__init__()
        self.light_on = False
        self.last_activity_time = time.time()
        self.is_active = False
        self.initialize_ui()
        self.connect_signals()
        self.setFocusPolicy(Qt.StrongFocus)
        
        self.idle_timer = QTimer(self)
        self.idle_timer.timeout.connect(self.check_idle_time)
        
        self.input_active = False

    def initialize_ui(self):
        self.setWindowTitle("ROV Control Panel")
        self.setGeometry(100, 100, 960, 540)
        
        self.video_receiver = VideoReceiver(parent=self)
        
        self.overlay_widget = QWidget(self)
        self.overlay_widget.setGeometry(0, 0, 960, 540)
        self.overlay_widget.setAttribute(Qt.WA_TranslucentBackground)
        
        self.control_frame = QFrame(self.overlay_widget)
        self.control_frame.setGeometry(800, 10, 150, 275)
        
        control_layout = QVBoxLayout(self.control_frame)
        control_layout.setContentsMargins(10, 10, 10, 10)
        control_layout.setSpacing(8)
        
        self.surface_button = ControlButton("SURFACE")
        self.dive_button = ControlButton("DIVE")
        self.mode_change_button = ControlButton("MODE CHANGE")
        self.roll_right_button = ControlButton("ROLL RIGHT")
        self.roll_left_button = ControlButton("ROLL LEFT")
        self.light_button = ControlButton("LIGHT OFF")
        
        control_layout.addWidget(self.surface_button)
        control_layout.addWidget(self.dive_button)
        control_layout.addWidget(self.mode_change_button)
        control_layout.addWidget(self.roll_right_button)
        control_layout.addWidget(self.roll_left_button)
        control_layout.addWidget(self.light_button)
        control_layout.addStretch()
        
        self.joystick = VirtualJoystick(self.overlay_widget)
        self.joystick.setGeometry(10, 370, 150, 150)
        
        status_frame = QFrame(self.overlay_widget)
        status_frame.setGeometry(10, 10, 200, 30)

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
        
        self.light_button.clicked.connect(self.wrap_activity_tracking(
            partial(ButtonController.on_light_button_clicked, self)))
    
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
    window = ControlPanel()
    window.show()
    sys.exit(app.exec_())