from PyQt5.QtWidgets import QHBoxLayout, QApplication, QWidget, QVBoxLayout, QSlider, QLabel
from PyQt5.QtCore import Qt

import os
import sys

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../../..")))

import Mission_planner.communication.pc_mavlink as mavlink
import time

from Mission_planner.layout.resources import style as st

class MotorSlider(QWidget):
    def __init__(self):

        super().__init__()
        self.setWindowTitle("Motror speed testing")
        self.setGeometry(100, 100, 565, 250)
        self.setFixedSize(580, 250)
        self.setStyleSheet("background-color: #395B64;")
        layout = QVBoxLayout()

        self.z_speed_set()
        self.xy_speed_set()

        # add layout
        layout.addWidget(self.set_xy_speed)
        layout.addWidget(self.set_z_speed)

        self.setLayout(layout)

    def xy_speed_set(self):
        # Tạo Slider 2
        self.set_xy_speed = QWidget()
        xy_layout = QHBoxLayout()
        self.set_xy_speed.setLayout(xy_layout)

        motorxy_slider = QWidget()
        motorxy_layout = QVBoxLayout()
        motorxy_slider.setLayout(motorxy_layout)

        self.motorxy_forward = QSlider(Qt.Horizontal)
        self.motorxy_forward.setMinimum(0)
        self.motorxy_forward.setMaximum(200)
        self.motorxy_forward.setValue(100)
        self.motorxy_forward.sliderReleased.connect(self.update_label1)

        self.motorxy_backward = QSlider(Qt.Horizontal)
        self.motorxy_backward.setMinimum(0)
        self.motorxy_backward.setMaximum(200)
        self.motorxy_backward.setValue(100)
        self.motorxy_backward.sliderReleased.connect(self.update_label1)

        motorxy_layout.addWidget(self.motorxy_forward)
        motorxy_layout.addWidget(self.motorxy_backward)

        self.label1 = QLabel("Motor xy: 100/100")
        self.label1.setStyleSheet(st.normal_text_info_style)
        xy_layout.addWidget(self.label1)
        xy_layout.addWidget(motorxy_slider)

    def z_speed_set(self):
        # Tạo Slider 2
        self.set_z_speed = QWidget()
        z_layout = QHBoxLayout()
        self.set_z_speed.setLayout(z_layout)

        motorz_slider = QWidget()
        motorz_layout = QVBoxLayout()
        motorz_slider.setLayout(motorz_layout)

        self.motorz_forward = QSlider(Qt.Horizontal)
        self.motorz_forward.setMinimum(0)
        self.motorz_forward.setMaximum(200)
        self.motorz_forward.setValue(100)
        self.motorz_forward.sliderReleased.connect(self.update_label2)

        self.motorz_backward = QSlider(Qt.Horizontal)
        self.motorz_backward.setMinimum(0)
        self.motorz_backward.setMaximum(200)
        self.motorz_backward.setValue(100)
        self.motorz_backward.sliderReleased.connect(self.update_label2)

        motorz_layout.addWidget(self.motorz_forward)
        motorz_layout.addWidget(self.motorz_backward)

        self.label2 = QLabel("Motor z: 100/100")
        self.label2.setStyleSheet(st.normal_text_info_style)
        z_layout.addWidget(self.label2)
        z_layout.addWidget(motorz_slider)

    def update_label1(self):
        #mavlink.MAV.set_max_speed_forward(self.motorxy.value())
        # mavlink.MAV.set_max_speed_backward(self.motor1.value())
        print(f"Motor xy: {self.motorxy_forward.value()}/{self.motorxy_backward.value()}")
        self.label1.setText(f"Motor xy: {self.motorxy_forward.value()}/{self.motorxy_backward.value()}")
    
    def update_label2(self):
        #mavlink.MAV.set_max_speed_z(self.motorz_forward.value(), self.motorz_backward.value())
        print(f"Motor z: {self.motorz_forward.value()}/{self.motorz_backward.value()}")
        self.label2.setText(f"Motor z: {self.motorz_forward.value()}/{self.motorz_backward.value()}")
    
