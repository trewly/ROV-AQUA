from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QSlider, QLabel
from PyQt5.QtCore import Qt

import os
import sys

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../../..")))

import Mission_planner.communication.pc_mavlink as mavlink
import time



class SliderApp(QWidget):
    def __init__(self):

        super().__init__()
        self.setWindowTitle("Motror speed testing")
        self.setGeometry(100, 100, 300, 200)

        # Layout chính
        layout = QVBoxLayout()

        # Tạo Slider 1
        self.motor1 = QSlider(Qt.Horizontal)
        self.motor1.setMinimum(0)
        self.motor1.setMaximum(100)
        self.motor1.setValue(50)
        self.motor1.sliderReleased.connect(self.update_label1)

        self.label1 = QLabel("motor xy: 50")
        
        # Tạo Slider 2
        self.motor2 = QSlider(Qt.Horizontal)
        self.motor2.setMinimum(0)
        self.motor2.setMaximum(200)
        self.motor2.setValue(100)
        self.motor2.sliderReleased.connect(self.update_label2)

        self.label2 = QLabel("motor z: 100")

        # Thêm vào layout
        layout.addWidget(self.label1)
        layout.addWidget(self.motor1)
        layout.addWidget(self.label2)
        layout.addWidget(self.motor2)

        self.setLayout(layout)

    def update_label1(self):
        mavlink.MAV.set_max_speed_forward(self.motor1.value())
        mavlink.MAV.set_max_speed_backward(self.motor1.value())
        print(f"motor xy: {self.motor1.value()}")
        self.label1.setText(f"motor xy: {self.motor1.value()}")
    def update_label2(self):
        print(f"motor z: {self.motor2.value()}")
        self.label2.setText(f"motor z: {self.motor2.value()}")

if __name__ == "__main__":
    app = QApplication([])
    window = SliderApp()
    window.show()
    app.exec_()
