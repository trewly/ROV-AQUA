import sys
import os
from functools import partial

from PyQt5 import QtCore
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QVBoxLayout, QHBoxLayout, QGridLayout, QFrame
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QFont, QIcon

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../.."))) 

from Mission_planner.controller.joystick_controller import VirtualJoystick
from Mission_planner.controller import button_controller as buttons_controller
from Mission_planner.controller.video_controller import VideoReceiver
from Mission_planner.communication.pc_mavlink import MAV

class ControlButton(QPushButton):
    def __init__(self, text, parent=None):
        super().__init__(text, parent)
        self.setMinimumSize(100, 30)
        self.setFont(QFont('Arial', 9, QFont.Bold))
        self.setCursor(Qt.PointingHandCursor)

class ControlPanel(QWidget):
    def __init__(self):
        super().__init__()
        self.light_on = False
        self.initialize_ui()
        self.connect_signals()
        self.setFocusPolicy(Qt.StrongFocus)

    def initialize_ui(self):
        self.setWindowTitle("ROV Control Panel")
        self.setGeometry(100, 100, 960, 540)
        
        self.video_receiver = VideoReceiver(parent=self)
        
        self.overlay_widget = QWidget(self)
        self.overlay_widget.setGeometry(0, 0, 960, 540)
        self.overlay_widget.setAttribute(Qt.WA_TranslucentBackground)
        
        self.control_frame = QFrame(self.overlay_widget)
        self.control_frame.setGeometry(800, 10, 150, 200)
        
        control_layout = QVBoxLayout(self.control_frame)
        control_layout.setContentsMargins(10, 10, 10, 10)
        control_layout.setSpacing(8)
        
        self.surface_button = ControlButton("SURFACE")
        self.dive_button = ControlButton("DIVE")
        self.mode_change_button = ControlButton("MODE CHANGE")
        self.light_button = ControlButton("LIGHT OFF")
        
        control_layout.addWidget(self.surface_button)
        control_layout.addWidget(self.dive_button)
        control_layout.addWidget(self.mode_change_button)
        control_layout.addWidget(self.light_button)
        control_layout.addStretch()
        
        self.joystick = VirtualJoystick(self.overlay_widget)
        self.joystick.setGeometry(10, 370, 150, 150)
        
        status_frame = QFrame(self.overlay_widget)
        status_frame.setGeometry(10, 10, 200, 30)

    def connect_signals(self):
        self.surface_button.pressed.connect(buttons_controller.controller.on_surface_button_clicked)
        self.surface_button.released.connect(buttons_controller.controller.on_surface_button_released)
        
        self.dive_button.pressed.connect(buttons_controller.controller.on_dive_button_clicked)
        self.dive_button.released.connect(buttons_controller.controller.on_dive_button_released)
        
        self.mode_change_button.clicked.connect(buttons_controller.controller.on_mode_change_button_clicked)
        
        self.light_button.clicked.connect(
            partial(buttons_controller.controller.on_light_button_clicked, self)
        )

    def keyPressEvent(self, event):
        if not buttons_controller.controller.handle_key_press(self, event):
            super().keyPressEvent(event)

    def keyReleaseEvent(self, event):
        if not buttons_controller.controller.handle_key_release(event):
            super().keyReleaseEvent(event)

    def closeEvent(self, event):
        print("Closing ROV Control Panel...")
        self.video_receiver.closeEvent(event)
        MAV.shutdown()
        event.accept()

def main():
    app = QApplication(sys.argv)
    window = ControlPanel()
    window.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()