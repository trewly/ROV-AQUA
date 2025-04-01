import sys
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QPixmap, QPalette, QColor
from PyQt5.QtWidgets import  QApplication,QLabel, QMainWindow, QVBoxLayout, QWidget, QPushButton

import os

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../../..")))

from Mission_planner.layout.resources import style as st
from Mission_planner.status import pc_status 
from Mission_planner.connection.pc_mavlink import MAV

YAW_CALIB = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "resources", "calib_image", "yaw_calib.png")

class calibration(QWidget):
    def __init__(self):
        super().__init__()

        self.central_widget = QWidget(self)

        palette = self.palette()
        palette.setColor(QPalette.Window, QColor("#F3F3E0")) 
        self.setPalette(palette)
        self.setAutoFillBackground(True)

        #Image widget label
        self.label = QLabel(self.central_widget)
        pixmap = QPixmap(YAW_CALIB)
        if pixmap.isNull():
            print("Không thể tải hình ảnh.")
        else:
            self.label.setPixmap(pixmap)
            self.label.setAlignment(Qt.AlignCenter)

        #button init
        self.start_button = QPushButton("Start Yaw Calib", self.central_widget)
        self.start_button.clicked.connect(self.start_calib)
        self.start_button.setStyleSheet(st.control_button_style)

        #main window
        layout = QVBoxLayout(self.central_widget)
        layout.addSpacing(30)
        layout.addWidget(self.label)
        layout.addWidget(self.start_button)

        self.setWindowTitle("Yaw Calibration")
        self.setGeometry(100, 100, 520, 600)  # Điều chỉnh kích thước cửa sổ

        #notify
        self.toast_label = QLabel(self)
        self.toast_label.setAlignment(Qt.AlignCenter)
        self.toast_label.setFixedSize(250,30)
        self.toast_label.setText("")
        self.toast_label.move(250, 15)  
        self.toast_label.hide()  #

        # timer
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.finish_calib)

    def start_calib(self):
        global yaw_calib_status
        yaw_calib_status = 0  
        MAV.start_mag_calibration()
        self.start_button.setText("Calibrating...") 
        self.start_button.setEnabled(False) 
        self.timer.start(25000) 

    def finish_calib(self):
        yaw_calib_status = pc_status.read_status("calibrated")
        self.timer.stop()  
        if yaw_calib_status == 1:
            self.show_successful_message()
        else:
            self.show_unsuccessful_message()

    def show_successful_message(self):
        self.show_toast_message("Calibration Successful!", success=True)
        self.start_button.setText("Start Yaw Calib")
        self.start_button.setEnabled(True)

    def show_unsuccessful_message(self):
        self.show_toast_message("Calibration Unsuccessful!", success=False)
        self.start_button.setText("Retry Yaw Calib")
        self.start_button.setEnabled(True)

    def show_toast_message(self, message, success=False):
        self.toast_label.setText(message)
        if success:
            self.toast_label.setStyleSheet(st.success_label_style)
        else:
            self.toast_label.setStyleSheet(st.warning_label_style)
        self.toast_label.show()
        QTimer.singleShot(2000, self.toast_label.hide) 

# app = QApplication(sys.argv)
# window = calibration()
# window.show()
# sys.exit(app.exec_())
