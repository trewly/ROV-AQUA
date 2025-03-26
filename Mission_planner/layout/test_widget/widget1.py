from PyQt5.QtWidgets import QWidget, QLabel, QVBoxLayout, QApplication

import sys
import os

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../.."))) 
#print(os.path.dirname(os.path.abspath(__file__)))

from communication.system_update_timer import SystemStatusManager

class BasicWidget1(QWidget):
    def __init__(self, status_manager: SystemStatusManager):
        super().__init__()

        # Tạo layout
        layout = QVBoxLayout()

        # Thêm một label
        self.label = QLabel("Hi I am widget 1", self)
        layout.addWidget(self.label)

        # Thiết lập layout cho widget
        self.setLayout(layout)

        #cai dat signal
        self.status_manager = status_manager
        self.status_manager.got_disconnected_info.connect(self.test_disconnect)
    
    def test_disconnect(self):
        print("got info from widget1")